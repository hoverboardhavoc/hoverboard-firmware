//! The ESP command client (`specs/esp-tool.md`, "ESP commands"): one request at a time over a
//! [`Wire`], and the flash transfer with the bulk window.
//!
//! This version addresses the ESP as `0x00` ("the one peer" on its own BLE link) and sends from the
//! first guest address; `--node` and the walk come with the ESP's node role.

use std::time::{Duration, Instant};

use espsvc::{
    BulkTx, Ctrl, EspInfo, Msg, Stage, Status, TargetId, MEM_MAX, OP_CLEAR_PROTECTION,
    OP_FLASH_BEGIN, OP_FLASH_CHUNK, OP_MEM_WRITE, OP_TARGET_CTRL,
};
use net::pdu::{Pdu, GUEST_FIRST, NO_ADDRESS};
use sha2::{Digest, Sha256};

use crate::wire::Wire;

/// How long one reply may take.
const REPLY: Duration = Duration::from_secs(3);
/// Tries for a request that is safe to repeat.
const TRIES: u32 = 3;
/// How long a flash transfer may go with no acknowledged progress.
const STALL: Duration = Duration::from_secs(10);
/// How long to wait for an acknowledgement before resending from the acknowledged offset.
const ACK_WAIT: Duration = Duration::from_millis(400);
/// How long the ESP may go between `FLASH_STATUS` reports while it programs.
const STATUS_GAP: Duration = Duration::from_secs(30);

/// Why a command failed.
#[derive(Debug, PartialEq, Eq)]
pub enum ClientError {
    /// No answer in time.
    Timeout,
    /// The ESP answered with this status.
    Status(Status),
    /// A flash stopped in this stage.
    Flash(Stage, Status),
}

impl std::fmt::Display for ClientError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClientError::Timeout => write!(f, "no answer from the ESP"),
            ClientError::Status(s) => write!(f, "{}", s.describe()),
            ClientError::Flash(stage, s) => write!(f, "flash failed ({stage:?}): {}", s.describe()),
        }
    }
}

/// A `FLASH_STATUS` report, for progress output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    /// The stage.
    pub stage: Stage,
    /// Bytes done in it.
    pub done: u32,
    /// Bytes in the image.
    pub total: u32,
}

/// The client.
pub struct Client<W: Wire> {
    wire: W,
    src: u8,
    dst: u8,
}

fn encode(m: &Msg, src: u8, dst: u8) -> Vec<u8> {
    let mut buf = [0u8; espsvc::MAX_PDU];
    let n = m.encode(src, dst, &mut buf).expect("a request fits a PDU");
    buf[..n].to_vec()
}

impl<W: Wire> Client<W> {
    /// A client for the ESP at the other end of `wire`.
    pub fn new(wire: W) -> Self {
        Client {
            wire,
            src: GUEST_FIRST,
            dst: NO_ADDRESS,
        }
    }

    /// The wire.
    pub fn wire(&mut self) -> &mut W {
        &mut self.wire
    }

    fn send(&mut self, m: &Msg) {
        let p = encode(m, self.src, self.dst);
        self.wire.send(&p);
    }

    /// Wait for the first reply `pick` accepts. Packets that are not ESP replies to this client
    /// (the board's own traffic, forwarded) are skipped.
    fn wait<T>(&mut self, within: Duration, mut pick: impl FnMut(&Msg) -> Option<T>) -> Option<T> {
        let deadline = Instant::now() + within;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let pkt = self.wire.recv(left)?;
            let Ok(pdu) = Pdu::decode(&pkt) else { continue };
            if pdu.dst != self.src || !espsvc::is_esp_opcode(pdu.opcode) {
                continue;
            }
            if let Ok(m) = Msg::decode(&pdu) {
                if let Some(t) = pick(&m) {
                    return Some(t);
                }
            }
            if Instant::now() >= deadline {
                return None;
            }
        }
    }

    /// Send `m` and wait for a reply, `tries` times.
    fn request<T>(
        &mut self,
        m: &Msg,
        tries: u32,
        within: Duration,
        mut pick: impl FnMut(&Msg) -> Option<Result<T, ClientError>>,
    ) -> Result<T, ClientError> {
        for _ in 0..tries {
            self.send(m);
            if let Some(r) = self.wait(within, &mut pick) {
                return r;
            }
        }
        Err(ClientError::Timeout)
    }

    /// `RESULT` for `op` as the outcome.
    fn result_for(op: u8) -> impl FnMut(&Msg) -> Option<Result<(), ClientError>> {
        move |m| match *m {
            Msg::Result { op: o, status } if o == op => Some(match status {
                Status::Ok => Ok(()),
                s => Err(ClientError::Status(s)),
            }),
            _ => None,
        }
    }

    /// `ESP_INFO`.
    pub fn info(&mut self) -> Result<EspInfo, ClientError> {
        self.request(&Msg::InfoReq, TRIES, REPLY, |m| match *m {
            Msg::Info(i) => Some(Ok(i)),
            _ => None,
        })
    }

    /// `TARGET_ID`.
    pub fn target_id(&mut self) -> Result<TargetId, ClientError> {
        self.request(&Msg::TargetIdReq, TRIES, REPLY, |m| match *m {
            Msg::TargetId(t) => Some(Ok(t)),
            Msg::Result { op, status } if op == espsvc::OP_TARGET_ID_REQ => {
                Some(Err(ClientError::Status(status)))
            }
            _ => None,
        })
    }

    /// `CLEAR_PROTECTION`. Sent once: it erases the chip.
    pub fn clear_protection(&mut self) -> Result<(), ClientError> {
        self.request(
            &Msg::ClearProtection,
            1,
            Duration::from_secs(10),
            Self::result_for(OP_CLEAR_PROTECTION),
        )
    }

    /// `TARGET_CTRL`. Sent once: a reset repeated is a second reset.
    pub fn ctrl(&mut self, c: Ctrl) -> Result<(), ClientError> {
        self.request(
            &Msg::TargetCtrl(c),
            1,
            REPLY,
            Self::result_for(OP_TARGET_CTRL),
        )
    }

    /// `MEM_READ`, split into [`MEM_MAX`] pieces.
    pub fn read(&mut self, addr: u32, len: usize) -> Result<Vec<u8>, ClientError> {
        let mut out = Vec::with_capacity(len);
        while out.len() < len {
            let a = addr + out.len() as u32;
            let n = (len - out.len()).min(MEM_MAX);
            let m = Msg::MemRead {
                addr: a,
                len: n as u16,
            };
            let part = self.request(&m, TRIES, REPLY, |r| match *r {
                Msg::MemData { addr, data } if addr == a => Some(Ok(data.to_vec())),
                Msg::Result { op, status } if op == espsvc::OP_MEM_READ => {
                    Some(Err(ClientError::Status(status)))
                }
                _ => None,
            })?;
            out.extend(part);
        }
        Ok(out)
    }

    /// `MEM_WRITE` (word-aligned, whole words), split into [`MEM_MAX`] pieces.
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), ClientError> {
        for (i, piece) in data.chunks(MEM_MAX).enumerate() {
            let m = Msg::MemWrite {
                addr: addr + (i * MEM_MAX) as u32,
                data: piece,
            };
            self.request(&m, TRIES, REPLY, Self::result_for(OP_MEM_WRITE))?;
        }
        Ok(())
    }

    /// Flash `image` (linked at the flash base): `FLASH_BEGIN`, the chunks with the window, then
    /// `FLASH_STATUS` until `booted`. `progress` sees every acknowledgement and status.
    pub fn flash(
        &mut self,
        image: &[u8],
        mut progress: impl FnMut(Progress),
    ) -> Result<(), ClientError> {
        let total = image.len() as u32;
        let sha256: [u8; 32] = Sha256::digest(image).into();
        self.request(
            &Msg::FlashBegin { len: total, sha256 },
            TRIES,
            Duration::from_secs(5),
            Self::result_for(OP_FLASH_BEGIN),
        )?;

        let mut tx = BulkTx::new(total);
        let mut last = Instant::now();
        while !tx.done() {
            while let Some((off, n)) = tx.next_chunk() {
                let o = off as usize;
                self.send(&Msg::FlashChunk {
                    offset: off,
                    data: &image[o..o + n],
                });
            }
            let ev = self.wait(ACK_WAIT, |m| match *m {
                Msg::FlashAck { contiguous } => Some(Ok(contiguous)),
                Msg::FlashStatus {
                    stage: Stage::Failed,
                    status,
                    ..
                } => Some(Err(ClientError::Flash(Stage::Receiving, status))),
                Msg::Result { op, status } if op == OP_FLASH_CHUNK => {
                    Some(Err(ClientError::Status(status)))
                }
                _ => None,
            });
            match ev {
                Some(Ok(c)) => {
                    let before = tx.acked();
                    tx.ack(c);
                    if tx.acked() > before {
                        last = Instant::now();
                        progress(Progress {
                            stage: Stage::Receiving,
                            done: tx.acked(),
                            total,
                        });
                    }
                }
                Some(Err(e)) => return Err(e),
                None => {
                    if last.elapsed() > STALL {
                        return Err(ClientError::Timeout);
                    }
                    tx.rewind();
                }
            }
        }

        loop {
            let st = self.wait(STATUS_GAP, |m| match *m {
                Msg::FlashStatus {
                    stage,
                    progress,
                    status,
                } => Some((stage, progress, status)),
                _ => None,
            });
            match st {
                None => return Err(ClientError::Timeout),
                Some((Stage::Failed, _, s)) => return Err(ClientError::Flash(Stage::Failed, s)),
                Some((stage, done, _)) => {
                    progress(Progress { stage, done, total });
                    if stage == Stage::Booted {
                        return Ok(());
                    }
                }
            }
        }
    }
}
