//! The ESP commands (`specs/esp-tool.md`, "ESP commands"), answered for packets the router hands
//! over, and the board flash run as a job between packets.
//!
//! In this version the ESP has no node address of its own (the walk role is not built), so it
//! answers commands addressed to `dst = 0x00` on its own BLE link or USB console and replies from
//! `0x00`.
//!
//! A flash or a protection clear holds the SWD engine: the mailbox port goes down for it, and the
//! commands that use the target answer `BUSY` until it is over. The ESP's own update
//! ([`crate::update`]) uses no SWD and runs beside either.

use espsvc::{BulkRx, Ctrl, EspInfo, Msg, Stage, Status, WifiState, CHIP_ESP32C3, MEM_MAX};
use net::pdu::{Pdu, NO_ADDRESS};
use sha2::{Digest, Sha256};

use crate::flash::{self, FlashJob, MAX_IMAGE};
use crate::mailbox::MailboxPort;
use crate::swd::SwdPins;
use crate::target::{self, TargetError};
use crate::update::{Step, Update};

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

/// The app slot the ESP runs from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Running {
    /// `0` for `ota_0`, `1` for `ota_1`, [`espsvc::SLOT_NONE`] for neither.
    pub slot: u8,
    /// Booted from an update and not yet confirmed: a reset returns to the other slot.
    pub pending: bool,
}

/// What the service asks of the ESP itself: the firmware's radio, flash and boot selection; a
/// model in the tests.
pub trait Platform {
    /// The Wi-Fi station's state and, while it is [`WifiState::Up`], its address.
    fn wifi(&self) -> (WifiState, [u8; 4]);
    /// The slot running and whether it waits for confirmation.
    fn running(&mut self) -> Running;
    /// Choose the slot not running as the target of an update of `len` bytes. `TooLarge` when it
    /// does not fit the slot.
    fn update_begin(&mut self, len: u32) -> Result<(), Status>;
    /// Erase the sector at `offset` (sector-aligned) in the target slot and write `data` (at most
    /// a sector) there.
    fn update_write(&mut self, offset: u32, data: &[u8]) -> Result<(), Status>;
    /// Read the target slot at `offset` into `buf` (word-aligned offset and length).
    fn update_read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Status>;
    /// Make the target slot the next boot's, marked new.
    fn update_activate(&mut self) -> Result<(), Status>;
    /// Mark the running slot valid if it waits for confirmation; nothing otherwise.
    fn confirm(&mut self) -> Result<(), Status>;
}

/// The command service.
pub struct Service<'a, Pl: Platform> {
    build: Build,
    platform: Pl,
    rx: &'a mut ImageBuffer,
    sha: [u8; 32],
    /// The controller the flash transfer came from: the target of `FLASH_ACK` / `FLASH_STATUS`.
    peer: u8,
    job: Option<FlashJob>,
    /// The USER byte recorded by the last protection clear since the ESP booted.
    clear_user: Option<u8>,
    /// The ESP's own update.
    update: Update,
    /// An update is in the next boot's slot: the firmware resets once the reply is out.
    reboot: bool,
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

impl<'a, Pl: Platform> Service<'a, Pl> {
    /// A service with `rx` as its transfer buffer, on `platform`.
    pub fn new(build: Build, rx: &'a mut ImageBuffer, platform: Pl) -> Self {
        Service {
            build,
            platform,
            rx,
            sha: [0; 32],
            peer: NO_ADDRESS,
            job: None,
            clear_user: None,
            update: Update::new(),
            reboot: false,
        }
    }

    /// An update finished and its slot is the next boot's: reset once the last reply has gone.
    pub fn reboot_requested(&self) -> bool {
        self.reboot
    }

    /// The platform.
    pub fn platform(&mut self) -> &mut Pl {
        &mut self.platform
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
                let (wifi, ip) = self.platform.wifi();
                let running = self.platform.running();
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
                    wifi: wifi as u8,
                    ip,
                    slot: running.slot,
                    pending: running.pending,
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
            Msg::UpdateBegin {
                len,
                sha256,
                version,
            } => {
                let r = self.update.begin(&mut self.platform, len, sha256, version);
                result(out, src, op, r.map_or_else(|e| e, |()| Status::Ok));
            }
            Msg::UpdateChunk { offset, data } => {
                match self.update.chunk(&mut self.platform, offset, data) {
                    Ok(Step::Ignore) => {}
                    Ok(Step::Ack(contiguous)) => reply(out, src, Msg::UpdateAck { contiguous }),
                    Ok(Step::Done(len, r)) => {
                        reply(out, src, Msg::UpdateAck { contiguous: len });
                        self.reboot |= r.is_ok();
                        result(out, src, op, r.map_or_else(|e| e, |()| Status::Ok));
                    }
                    Err(e) => result(out, src, op, e),
                }
            }
            Msg::UpdateConfirm => {
                let r = self.platform.confirm();
                result(out, src, op, r.map_or_else(|e| e, |()| Status::Ok));
            }
            // Replies are not commands.
            Msg::Info(_)
            | Msg::TargetId(_)
            | Msg::FlashAck { .. }
            | Msg::FlashStatus { .. }
            | Msg::MemData { .. }
            | Msg::UpdateAck { .. }
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
