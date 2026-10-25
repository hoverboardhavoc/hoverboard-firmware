//! The mailbox port (`specs/hoverboard-bridge.md`, "Mailbox port"): the bridge as the bridge end of the board's
//! RAM mailbox, in place of `swd-bridge` on a host talking to OpenOCD.
//!
//! The ring logic and the attach handshake are `swd-bridge-core`'s; this module gives it a
//! [`MemAp`] on the SWD engine ([`SwdAp`]) and runs the session: open at boot and keep open, attach
//! with the epoch handshake, poll continuously, retry a frame the full ring refused on the next
//! poll (latest-wins for drive frames), and reopen after a board reset or a flash.
//!
//! The port owns the SWD engine (through the `MemAp` it gives the bridge); every other user of the
//! engine borrows it through [`MailboxPort::swd`], so there is one owner of the pins.

use espsvc::MailboxState;
use heapless::Vec;
use link::{Link, SerialTransport};
use swd_bridge_core::{BridgeError, BridgeSerial, HostMailbox, MemAp};
use swd_mailbox::{FRAME_CAPACITY, MAGIC, STREAM_OVERHEAD, VERSION};

use crate::swd::{Swd, SwdError, SwdPins};
use crate::target;

/// The largest packet the port carries (one L3 PDU).
pub const MAX_PACKET: usize = 256;

/// The SWD engine as a [`MemAp`]: word accesses for the header, byte runs for the rings.
pub struct SwdAp<P: SwdPins>(pub Swd<P>);

impl<P: SwdPins> MemAp for SwdAp<P> {
    type Error = SwdError;
    fn read32(&mut self, addr: u32) -> Result<u32, SwdError> {
        self.0.read_word(addr)
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), SwdError> {
        self.0.write_word(addr, val)
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), SwdError> {
        self.0.read_bytes(addr, out)
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), SwdError> {
        self.0.write_bytes(addr, data)
    }
}

type MbLink<P> = Link<
    SerialTransport<BridgeSerial<SwdAp<P>>, { FRAME_CAPACITY + STREAM_OVERHEAD }>,
    MAX_PACKET,
    FRAME_CAPACITY,
>;

/// How often a closed port tries to open, and an open one checks the header for a reset.
const REOPEN_US: u64 = 500_000;
const HEADER_CHECK_US: u64 = 100_000;
/// How long the firmware gets to acknowledge an attach.
const ACK_TIMEOUT_US: u64 = 2_000_000;
/// Frames waiting for room in the board's inbound ring.
const QUEUE: usize = 8;

/// A packet waiting for room in `h2t`.
#[derive(Clone)]
struct Pending {
    bytes: Vec<u8, MAX_PACKET>,
}

impl Pending {
    /// The latest-wins key: `DRIVE_CMD` and `INPUTS` to one destination replace each other.
    fn latest_wins_key(&self) -> Option<(u8, u8)> {
        let op = *self.bytes.first()?;
        let dst = *self.bytes.get(2)?;
        (op == linkctl::OP_DRIVE_CMD || op == linkctl::OP_INPUTS).then_some((op, dst))
    }
}

/// The bridge's end of the board's mailbox.
pub struct MailboxPort<P: SwdPins> {
    link: MbLink<P>,
    state: MailboxState,
    /// When the current state was entered, or the last header check.
    since_us: u64,
    queue: Vec<Pending, QUEUE>,
    /// Frames dropped because the queue was full.
    pub dropped: u32,
}

impl<P: SwdPins> MailboxPort<P> {
    /// A closed port owning the engine. It opens on the first [`poll`](Self::poll).
    pub fn new(swd: Swd<P>) -> Self {
        let mb = HostMailbox::new(SwdAp(swd), swd_mailbox::MAILBOX_BASE);
        MailboxPort {
            link: Link::new(SerialTransport::new(BridgeSerial::new(mb), FRAME_CAPACITY)),
            state: MailboxState::NoTarget,
            since_us: 0,
            queue: Vec::new(),
            dropped: 0,
        }
    }

    fn mb(&mut self) -> &mut HostMailbox<SwdAp<P>> {
        self.link.transport_mut().serial_mut().mailbox()
    }

    /// The SWD engine, for the board operations and the flash.
    pub fn swd(&mut self) -> &mut Swd<P> {
        &mut self.mb().mem().0
    }

    /// The port's state (`ESP_INFO`).
    pub fn state(&self) -> MailboxState {
        self.state
    }

    /// Take the port down while a flash or a protection clear holds the engine.
    pub fn hold(&mut self) {
        self.close(MailboxState::Held);
    }

    /// Give the port back after a hold; it reopens on the next poll.
    pub fn release(&mut self) {
        if self.state == MailboxState::Held {
            self.close(MailboxState::NoHeader);
            self.since_us = 0;
        }
    }

    fn close(&mut self, why: MailboxState) {
        self.mb().detach();
        self.link.transport_mut().reset();
        self.queue.clear();
        self.state = why;
    }

    /// Queue a packet for the board. Drive frames are latest-wins: a newer `DRIVE_CMD` or
    /// `INPUTS` to the same node replaces an unsent older one rather than queueing behind it.
    pub fn send(&mut self, packet: &[u8]) {
        if self.state != MailboxState::Up {
            return;
        }
        let Ok(bytes) = Vec::from_slice(packet) else {
            self.dropped += 1;
            return;
        };
        let p = Pending { bytes };
        if let Some(key) = p.latest_wins_key() {
            if let Some(old) = self
                .queue
                .iter_mut()
                .find(|q| q.latest_wins_key() == Some(key))
            {
                *old = p;
                return;
            }
        }
        if self.queue.push(p).is_err() {
            self.dropped += 1;
        }
    }

    /// Frames waiting for room in the board's inbound ring.
    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    /// Service the port: open or reattach as needed, move queued frames into `h2t`, and hand every
    /// packet the board produced to `deliver`.
    pub fn poll(&mut self, mut deliver: impl FnMut(&[u8])) {
        self.poll_session();
        let mut buf = [0u8; MAX_PACKET];
        while let Some(n) = self.recv(&mut buf) {
            deliver(&buf[..n]);
        }
    }

    /// The session half of [`poll`](Self::poll): open or reattach as needed and move queued frames
    /// into `h2t`. The board's packets are then taken one at a time with [`recv`](Self::recv), so
    /// a caller can answer each with the port in hand.
    pub fn poll_session(&mut self) {
        let now = self.swd().now_us();
        match self.state {
            MailboxState::Held => {}
            MailboxState::Up => self.service(now),
            MailboxState::Attaching => match self.mb().flush_acked() {
                Ok(true) => {
                    self.state = MailboxState::Up;
                    self.since_us = now;
                }
                Ok(false) if now.saturating_sub(self.since_us) > ACK_TIMEOUT_US => {
                    self.close(MailboxState::NoHeader);
                    self.since_us = now;
                }
                Ok(false) => {}
                Err(_) => self.lost(now),
            },
            _ => {
                if self.since_us == 0 || now.saturating_sub(self.since_us) > REOPEN_US {
                    self.open(now);
                }
            }
        }
    }

    /// The next whole packet the board produced, copied into `buf`; its length. Nothing while the
    /// port is not up.
    pub fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        if self.state != MailboxState::Up {
            return None;
        }
        self.link.poll_recv(buf).map(|p| p.len())
    }

    fn lost(&mut self, now: u64) {
        self.close(MailboxState::NoTarget);
        self.since_us = now;
    }

    /// Validate the header and attach: version and registry fingerprint must match this build.
    fn open(&mut self, now: u64) {
        self.since_us = now.max(1);
        if target::ensure_connected(self.swd()).is_err() {
            self.state = MailboxState::NoTarget;
            return;
        }
        let mb = self.mb();
        let header = (|| {
            let magic = mb.magic()?;
            let version = mb.version()?;
            let hash = mb.registry_hash()?;
            Ok::<_, BridgeError<SwdError>>((magic, version, hash))
        })();
        self.state = match header {
            Err(_) => {
                self.swd_lost();
                MailboxState::NoTarget
            }
            Ok((magic, _, _)) if magic != MAGIC => MailboxState::NoHeader,
            Ok((_, version, _)) if version != VERSION => MailboxState::VersionMismatch,
            Ok((_, _, hash)) if hash != swd_mailbox::REGISTRY_HASH => {
                MailboxState::RegistryMismatch
            }
            Ok(_) => match self.mb().attach() {
                Ok(()) => {
                    self.link.transport_mut().reset();
                    MailboxState::Attaching
                }
                Err(_) => MailboxState::NoTarget,
            },
        };
    }

    /// The engine gave up: the next use reconnects.
    fn swd_lost(&mut self) {
        let swd = self.swd();
        if swd.dpidr().is_some() {
            // Forget the connection; `ensure_connected` reconnects on the next open.
            swd.forget();
        }
    }

    fn service(&mut self, now: u64) {
        // A board reset re-initialises the header (epoch back to 0) and a flash clears the magic:
        // both show here, and the port reopens.
        if now.saturating_sub(self.since_us) > HEADER_CHECK_US {
            self.since_us = now;
            let check = {
                let mb = self.mb();
                mb.magic().map(|m| (m, mb.check_session()))
            };
            match check {
                Err(_) => return self.lost(now),
                Ok((m, _)) if m != MAGIC => {
                    self.close(MailboxState::NoHeader);
                    return;
                }
                Ok((_, Err(_))) => {
                    self.close(MailboxState::NoHeader);
                    self.since_us = 0;
                    return;
                }
                Ok(_) => {}
            }
        }
        // Inbound to the board: the oldest queued frame first; a refused frame stays for the next
        // poll.
        while let Some(p) = self.queue.first() {
            let bytes = p.bytes.clone();
            if self.link.send(&bytes).is_err() {
                self.queue.remove(0);
                self.dropped += 1;
                continue;
            }
            match self.link.transport_mut().serial_mut().take_write_error() {
                None => {
                    self.queue.remove(0);
                }
                Some(BridgeError::NotDraining { .. }) => break,
                Some(BridgeError::MemAp(_)) => return self.lost(now),
                Some(_) => {
                    // A second session or producer: reattach.
                    self.close(MailboxState::NoHeader);
                    self.since_us = 0;
                    return;
                }
            }
        }
    }
}
