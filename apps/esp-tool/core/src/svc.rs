//! The ESP commands (`specs/esp-tool.md`, "ESP commands"), answered for packets the router hands
//! over, and the board flash run as a job between packets.
//!
//! In this version the ESP has no node address of its own (the walk role is not built), so it
//! answers commands addressed to `dst = 0x00` on its own BLE link or USB console and replies from
//! `0x00`.
//!
//! A flash or a protection clear holds the SWD engine: the mailbox port goes down for it, and the
//! commands that use the target answer `BUSY` until it is over.

use espsvc::{BulkRx, Ctrl, EspInfo, Msg, Stage, Status, CHIP_ESP32C3, MEM_MAX};
use net::pdu::{Pdu, NO_ADDRESS};
use sha2::{Digest, Sha256};

use crate::flash::{self, FlashJob, MAX_IMAGE};
use crate::mailbox::MailboxPort;
use crate::swd::SwdPins;
use crate::target::{self, TargetError};

/// The flash transfer buffer: one image, everything below the store.
pub type ImageBuffer = BulkRx<MAX_IMAGE>;

/// What the ESP reports about itself that does not change while it runs.
#[derive(Debug, Clone, Copy)]
pub struct Build {
    /// `[major, minor, patch]`.
    pub version: [u8; 3],
    /// The first 32 bits of the git commit.
    pub commit: u32,
    /// The SWD clock measured at boot, kHz.
    pub swd_khz: u16,
}

/// The command service.
pub struct Service<'a> {
    build: Build,
    rx: &'a mut ImageBuffer,
    sha: [u8; 32],
    /// The controller the flash transfer came from: the target of `FLASH_ACK` / `FLASH_STATUS`.
    peer: u8,
    job: Option<FlashJob>,
    /// The USER byte recorded by the last protection clear since the ESP booted.
    clear_user: Option<u8>,
}

/// Where replies go.
pub type Out<'o> = &'o mut dyn FnMut(&[u8]);

fn reply(out: Out, dst: u8, m: Msg) {
    let mut buf = [0u8; espsvc::MAX_PDU];
    if let Ok(n) = m.encode(NO_ADDRESS, dst, &mut buf) {
        out(&buf[..n]);
    }
}

fn result(out: Out, dst: u8, op: u8, status: Status) {
    reply(out, dst, Msg::Result { op, status });
}

fn target_status(e: TargetError) -> Status {
    match e {
        TargetError::Armed => Status::Armed,
        TargetError::Swd(_) => Status::NoTarget,
        TargetError::Timeout => Status::Timeout,
    }
}

impl<'a> Service<'a> {
    /// A service with `rx` as its transfer buffer.
    pub fn new(build: Build, rx: &'a mut ImageBuffer) -> Self {
        Service {
            build,
            rx,
            sha: [0; 32],
            peer: NO_ADDRESS,
            job: None,
            clear_user: None,
        }
    }

    /// A flash holds the engine.
    pub fn busy(&self) -> bool {
        self.job.is_some()
    }

    /// Answer one command.
    pub fn handle<P: SwdPins>(&mut self, pdu: &Pdu, port: &mut MailboxPort<P>, out: Out) {
        let Ok(msg) = Msg::decode(pdu) else {
            return; // an unknown or malformed command is ignored, as L3 ignores unknown opcodes
        };
        let src = pdu.src;
        let op = pdu.opcode;
        if self.busy()
            && matches!(
                msg,
                Msg::TargetIdReq
                    | Msg::ClearProtection
                    | Msg::FlashBegin { .. }
                    | Msg::TargetCtrl(_)
                    | Msg::MemRead { .. }
                    | Msg::MemWrite { .. }
            )
        {
            return result(out, src, op, Status::Busy);
        }
        match msg {
            Msg::InfoReq => {
                let c = port.swd().counters();
                let info = EspInfo {
                    version: self.build.version,
                    commit: self.build.commit,
                    chip: CHIP_ESP32C3,
                    swd_khz: self.build.swd_khz,
                    waits: c.waits,
                    faults: c.faults,
                    parity_errors: c.parity_errors,
                    reconnects: c.reconnects,
                    mailbox: port.state() as u8,
                };
                reply(out, src, Msg::Info(info));
            }
            Msg::TargetIdReq => match target::identify(port.swd()) {
                Ok(mut id) => {
                    if let Some(user) = self.clear_user {
                        id.clear_check = if flash::clear_took(&id, user) {
                            espsvc::CLEAR_PASS
                        } else {
                            espsvc::CLEAR_FAIL
                        };
                    }
                    reply(out, src, Msg::TargetId(id));
                }
                Err(_) => self.no_target(port, out, src, op),
            },
            Msg::ClearProtection => {
                port.hold();
                let r = flash::clear_protection(port.swd());
                port.release();
                match r {
                    Ok(user) => {
                        self.clear_user = Some(user);
                        result(out, src, op, Status::Ok);
                    }
                    Err(e) => {
                        port.swd().forget();
                        result(out, src, op, e.status());
                    }
                }
            }
            Msg::FlashBegin { len, sha256 } => {
                let checked = flash::check_range(len as usize)
                    .and_then(|()| flash::check_part(port.swd()).map(|_| ()));
                match checked {
                    Ok(()) => {
                        // `check_range` bounds `len` by the buffer.
                        let _ = self.rx.begin(len);
                        self.sha = sha256;
                        self.peer = src;
                        result(out, src, op, Status::Ok);
                    }
                    Err(e) => result(out, src, op, e.status()),
                }
            }
            Msg::FlashChunk { offset, data } => self.chunk(port, out, src, offset, data),
            Msg::TargetCtrl(c) => {
                let swd = port.swd();
                let r = target::ensure_connected(swd)
                    .map_err(TargetError::Swd)
                    .and_then(|_| match c {
                        Ctrl::Disarm => target::disarm(swd),
                        Ctrl::Halt => target::halt(swd),
                        Ctrl::Resume => target::resume(swd).map_err(TargetError::Swd),
                        Ctrl::Reset => target::reset(swd).map_err(TargetError::Swd),
                        Ctrl::ResetHalt => target::reset_halt(swd),
                    });
                if let Err(TargetError::Swd(_)) = r {
                    port.swd().forget();
                }
                result(out, src, op, r.map_or_else(target_status, |()| Status::Ok));
            }
            Msg::MemRead { addr, len } => {
                if len as usize > MEM_MAX {
                    return result(out, src, op, Status::BadRequest);
                }
                let mut data = [0u8; MEM_MAX];
                let d = &mut data[..len as usize];
                match target::ensure_connected(port.swd())
                    .and_then(|_| port.swd().read_bytes(addr, d))
                {
                    Ok(()) => reply(out, src, Msg::MemData { addr, data: d }),
                    Err(_) => self.no_target(port, out, src, op),
                }
            }
            Msg::MemWrite { addr, data } => {
                if addr % 4 != 0 || data.len() % 4 != 0 {
                    return result(out, src, op, Status::BadRequest);
                }
                match target::ensure_connected(port.swd())
                    .and_then(|_| port.swd().write_bytes(addr, data))
                {
                    Ok(()) => result(out, src, op, Status::Ok),
                    Err(_) => self.no_target(port, out, src, op),
                }
            }
            // Replies are not commands.
            Msg::Info(_)
            | Msg::TargetId(_)
            | Msg::FlashAck { .. }
            | Msg::FlashStatus { .. }
            | Msg::MemData { .. }
            | Msg::Result { .. } => {}
        }
    }

    fn no_target<P: SwdPins>(&mut self, port: &mut MailboxPort<P>, out: Out, src: u8, op: u8) {
        port.swd().forget();
        result(out, src, op, Status::NoTarget);
    }

    fn chunk<P: SwdPins>(
        &mut self,
        port: &mut MailboxPort<P>,
        out: Out,
        src: u8,
        offset: u32,
        data: &[u8],
    ) {
        if self.rx.is_empty() {
            return; // no transfer: a late chunk of a finished or refused one
        }
        if self.job.is_some() || self.rx.complete() {
            // A duplicate after the last chunk: the sender has not seen the final ack yet.
            return reply(
                out,
                src,
                Msg::FlashAck {
                    contiguous: self.rx.len(),
                },
            );
        }
        match self.rx.chunk(offset, data) {
            Ok(contiguous) => reply(out, src, Msg::FlashAck { contiguous }),
            Err(_) => return result(out, src, espsvc::OP_FLASH_CHUNK, Status::BadRequest),
        }
        if !self.rx.complete() {
            return;
        }
        let len = self.rx.len();
        let status = |stage, progress, status| Msg::FlashStatus {
            stage,
            progress,
            status,
        };
        if Sha256::digest(self.rx.data()).as_slice() != self.sha {
            self.rx.reset();
            return reply(
                out,
                self.peer,
                status(Stage::Failed, 0, Status::ShaMismatch),
            );
        }
        match FlashJob::new(len as usize) {
            Ok(job) => {
                port.hold();
                self.job = Some(job);
                reply(out, self.peer, status(Stage::Receiving, len, Status::Ok));
            }
            Err(e) => {
                self.rx.reset();
                reply(out, self.peer, status(Stage::Failed, 0, e.status()));
            }
        }
    }

    /// Advance a running flash by one page and report it.
    pub fn poll<P: SwdPins>(&mut self, port: &mut MailboxPort<P>, out: Out) {
        let Some(job) = &mut self.job else {
            return;
        };
        let ev = job.step(port.swd(), self.rx.data());
        let msg = match ev {
            flash::Event::Progress(stage, progress) => Msg::FlashStatus {
                stage,
                progress,
                status: Status::Ok,
            },
            flash::Event::Finished(r) => {
                let len = self.rx.len();
                self.job = None;
                self.rx.reset();
                port.release();
                match r {
                    Ok(()) => Msg::FlashStatus {
                        stage: Stage::Booted,
                        progress: len,
                        status: Status::Ok,
                    },
                    Err(e) => {
                        if let flash::FlashError::Target(TargetError::Swd(_)) = e {
                            port.swd().forget();
                        }
                        Msg::FlashStatus {
                            stage: Stage::Failed,
                            progress: 0,
                            status: e.status(),
                        }
                    }
                }
            }
        };
        reply(out, self.peer, msg);
    }
}
