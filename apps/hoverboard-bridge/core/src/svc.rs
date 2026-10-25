//! The ESP commands (`specs/hoverboard-bridge.md`, "ESP commands"), answered for packets the node hands
//! over ([`crate::node`] decides which: `dst = 0x00` on a controller link, or the bridge's own address
//! from anywhere), and the board flash run as a job between packets. Replies go to the caller's
//! `out` with the requester as their destination; the node puts its own address on them and the
//! carrier they leave on.
//!
//! A flash or a protection clear holds the SWD engine: the mailbox port goes down for it, and the
//! commands that use the target answer `BUSY` until it is over. A flash and a protection clear are
//! also refused with `BUSY` while a GDB session holds the target halted. The bridge's own update
//! ([`crate::update`]) uses no SWD and runs beside either.
//!
//! GDB's `load` ([`crate::gdb`]) fills the same image buffer and runs the same [`FlashJob`], with
//! [`Finish::Halt`] and its result kept for the GDB session instead of sent as `FLASH_STATUS`.

use espsvc::{BulkRx, Ctrl, EspInfo, Msg, Stage, Status, WifiState, CHIP_ESP32C3, MEM_MAX};
use net::pdu::{Pdu, NO_ADDRESS};
use sha2::{Digest, Sha256};

use crate::flash::{self, Finish, FlashError, FlashJob, MAX_IMAGE};
use crate::mailbox::MailboxPort;
use crate::swd::SwdPins;
use crate::target::{self, TargetError};
use crate::update::{Step, Update};

/// The flash transfer buffer: one image, everything below the store.
pub type ImageBuffer = BulkRx<MAX_IMAGE>;

/// What the bridge reports about itself that does not change while it runs.
#[derive(Debug, Clone, Copy)]
pub struct Build {
    /// `[major, minor, patch]`.
    pub version: [u8; 3],
    /// The first 32 bits of the git commit.
    pub commit: u32,
    /// The SWD clock measured at boot, kHz.
    pub swd_khz: u16,
}

/// The bridge as a node, for `ESP_INFO` (`specs/hoverboard-bridge.md`, "The bridge as a node").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity {
    /// Its L3 address, `0x00` while unassigned.
    pub addr: u8,
    /// Whether its store mounted at boot.
    pub store_mounted: bool,
}

/// The app slot the bridge runs from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Running {
    /// `0` for `ota_0`, `1` for `ota_1`, [`espsvc::SLOT_NONE`] for neither.
    pub slot: u8,
    /// Booted from an update and not yet confirmed: a reset returns to the other slot.
    pub pending: bool,
}

/// The GDB session as `ESP_INFO` and the engine sharing rules see it; [`crate::gdb`] keeps it
/// current.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GdbStatus {
    /// A client is connected.
    pub connected: bool,
    /// The session holds the core halted.
    pub halted: bool,
    /// The last halt-to-`MOE`-clear time, microseconds ([`espsvc::MOE_CLEAR_NONE`] before any).
    pub moe_clear_us: u32,
}

/// GDB's `load`, as far as the service has it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GdbLoad {
    /// No load: the image buffer is the terminal tool's.
    Idle,
    /// `vFlashErase` and `vFlashWrite` are filling the image buffer.
    Collecting,
    /// The flash job runs for `vFlashDone`.
    Flashing,
    /// The job finished; `vFlashDone` takes the result.
    Done(Result<(), FlashError>),
}

/// What the service asks of the bridge itself: the firmware's radio, flash and boot selection; a
/// model in the tests.
pub trait Platform {
    /// The Wi-Fi station's state and, while it is [`WifiState::Up`], its address.
    fn wifi(&self) -> (WifiState, [u8; 4]);
    /// The slot running and whether it waits for confirmation.
    fn running(&mut self) -> Running;
    /// Choose the slot not running as the target of an update of `len` bytes. `TooLarge` when it
    /// does not fit the slot.
    fn update_begin(&mut self, len: u32) -> Result<(), Status>;
    /// Erase the sector at `offset` (sector-aligned) in the target slot.
    fn update_erase(&mut self, offset: u32) -> Result<(), Status>;
    /// Write `data` (at most a sector) at `offset` (sector-aligned) in the target slot, into a
    /// sector [`update_erase`](Self::update_erase) has erased.
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
    /// The USER byte recorded by the last protection clear since the bridge booted.
    clear_user: Option<u8>,
    /// The bridge's own update (its sector buffer is the size of a stack, so it is the caller's to
    /// place).
    update: &'a mut Update,
    /// An update is in the next boot's slot: the firmware resets once the reply is out.
    reboot: bool,
    gdb: GdbStatus,
    gdb_load: GdbLoad,
}

/// Where replies go: each with the address it is for.
pub type Out<'o> = &'o mut dyn FnMut(u8, &Msg);

fn reply(out: Out, dst: u8, m: Msg) {
    out(dst, &m);
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
    /// A service with `rx` as its board flash buffer and `update` for the bridge's own, on
    /// `platform`.
    pub fn new(
        build: Build,
        rx: &'a mut ImageBuffer,
        update: &'a mut Update,
        platform: Pl,
    ) -> Self {
        Service {
            build,
            platform,
            rx,
            sha: [0; 32],
            peer: NO_ADDRESS,
            job: None,
            clear_user: None,
            update,
            reboot: false,
            gdb: GdbStatus {
                connected: false,
                halted: false,
                moe_clear_us: espsvc::MOE_CLEAR_NONE,
            },
            gdb_load: GdbLoad::Idle,
        }
    }

    /// The GDB session's state, which [`crate::gdb`] keeps current.
    pub fn gdb_status(&mut self) -> &mut GdbStatus {
        &mut self.gdb
    }

    /// The image buffer for GDB's `vFlashErase` and `vFlashWrite`. The first call of a load takes
    /// the buffer, dropping an unfinished transfer from the terminal tool; `Busy` while a flash
    /// runs.
    pub fn gdb_image(&mut self) -> Result<&mut ImageBuffer, Status> {
        if self.busy() {
            return Err(Status::Busy);
        }
        if self.gdb_load != GdbLoad::Collecting {
            self.rx.reset();
            self.gdb_load = GdbLoad::Collecting;
        }
        Ok(self.rx)
    }

    /// `vFlashDone`: start the flash job on the collected image, which halts at the new image's
    /// reset vector instead of booting it. A load with nothing collected is done at once.
    pub fn gdb_load_start<P: SwdPins>(&mut self, port: &mut MailboxPort<P>) {
        if self.gdb_load != GdbLoad::Collecting || self.rx.is_empty() {
            self.gdb_load = GdbLoad::Done(Ok(()));
            return;
        }
        match FlashJob::new(self.rx.len() as usize, Finish::Halt) {
            Ok(job) => {
                port.hold();
                self.job = Some(job);
                self.gdb_load = GdbLoad::Flashing;
            }
            Err(e) => {
                self.rx.reset();
                self.gdb_load = GdbLoad::Done(Err(e));
            }
        }
    }

    /// The finished load's result, once: `None` while the job runs.
    pub fn gdb_load_result(&mut self) -> Option<Result<(), FlashError>> {
        match self.gdb_load {
            GdbLoad::Done(r) => {
                self.gdb_load = GdbLoad::Idle;
                Some(r)
            }
            _ => None,
        }
    }

    /// The flash job for `vFlashDone` runs.
    pub fn gdb_load_running(&self) -> bool {
        self.gdb_load == GdbLoad::Flashing
    }

    /// The GDB client left: a load it was collecting is dropped (one that is flashing finishes).
    pub fn gdb_load_drop(&mut self) {
        match self.gdb_load {
            GdbLoad::Collecting => {
                self.rx.reset();
                self.gdb_load = GdbLoad::Idle;
            }
            GdbLoad::Done(_) => self.gdb_load = GdbLoad::Idle,
            GdbLoad::Idle | GdbLoad::Flashing => {}
        }
    }

    /// The image buffer is GDB's (a load is collecting or flashing).
    fn gdb_owns_image(&self) -> bool {
        matches!(self.gdb_load, GdbLoad::Collecting | GdbLoad::Flashing)
    }

    /// Identify the target, with the check of the last protection clear since the bridge booted.
    pub fn target_id<P: SwdPins>(
        &mut self,
        port: &mut MailboxPort<P>,
    ) -> Result<espsvc::TargetId, crate::swd::SwdError> {
        let mut id = target::identify(port.swd())?;
        if let Some(user) = self.clear_user {
            id.clear_check = if flash::clear_took(&id, user) {
                espsvc::CLEAR_PASS
            } else {
                espsvc::CLEAR_FAIL
            };
        }
        Ok(id)
    }

    /// Clear read protection with the engine held, recording the USER byte for the check after
    /// the power cycle.
    pub fn clear_protection<P: SwdPins>(
        &mut self,
        port: &mut MailboxPort<P>,
    ) -> Result<(), FlashError> {
        port.hold();
        let r = flash::clear_protection(port.swd());
        port.release();
        match r {
            Ok(user) => {
                self.clear_user = Some(user);
                Ok(())
            }
            Err(e) => {
                port.swd().forget();
                Err(e)
            }
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

    /// Answer one command; `me` is the bridge as a node, for `ESP_INFO`.
    pub fn handle<P: SwdPins>(
        &mut self,
        pdu: &Pdu,
        me: Identity,
        port: &mut MailboxPort<P>,
        out: Out,
    ) {
        let Ok(msg) = Msg::decode(pdu) else {
            return; // an unknown or malformed command is ignored, as L3 ignores unknown opcodes
        };
        let src = pdu.src;
        let op = pdu.opcode;
        if (self.gdb.halted || self.gdb_owns_image())
            && matches!(msg, Msg::ClearProtection | Msg::FlashBegin { .. })
        {
            return result(out, src, op, Status::Busy);
        }
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
                let d = port.swd().diag();
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
                    bad_read: d.bad_read.map(|b| espsvc::BadRead {
                        ack: b.ack,
                        data: b.data,
                    }),
                    no_ack: d.no_ack,
                    mailbox: port.state() as u8,
                    wifi: wifi as u8,
                    ip,
                    slot: running.slot,
                    pending: running.pending,
                    node: me.addr,
                    store_mounted: me.store_mounted,
                    gdb: self.gdb.connected,
                    moe_clear_us: self.gdb.moe_clear_us,
                };
                reply(out, src, Msg::Info(info));
            }
            Msg::TargetIdReq => match self.target_id(port) {
                Ok(id) => reply(out, src, Msg::TargetId(id)),
                Err(_) => self.no_target(port, out, src, op),
            },
            Msg::ClearProtection => {
                let r = self.clear_protection(port);
                result(out, src, op, r.map_or_else(|e| e.status(), |()| Status::Ok));
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
        if self.rx.is_empty() || self.gdb_owns_image() {
            return; // no transfer: a late chunk of a finished, refused or displaced one
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
        if self.rx.chunk(offset, data).is_err() {
            return result(out, src, espsvc::OP_FLASH_CHUNK, Status::BadRequest);
        }
        if let Some(contiguous) = self.rx.take_ack() {
            reply(out, src, Msg::FlashAck { contiguous });
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
        match FlashJob::new(len as usize, Finish::Boot) {
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

    /// A service pass that took no controller packet: the bridge's own update erases ahead.
    pub fn idle(&mut self) {
        self.update.erase_ahead(&mut self.platform);
    }

    /// Advance a running flash by one bounded piece and report any progress.
    pub fn poll<P: SwdPins>(&mut self, port: &mut MailboxPort<P>, out: Out) {
        let Some(job) = &mut self.job else {
            return;
        };
        let ev = job.step(port.swd(), self.rx.data());
        let msg = match ev {
            flash::Event::Pending => return,
            flash::Event::Progress(..) if self.gdb_load == GdbLoad::Flashing => return,
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
                if self.gdb_load == GdbLoad::Flashing {
                    if let Err(flash::FlashError::Target(TargetError::Swd(_))) = r {
                        port.swd().forget();
                    }
                    self.gdb_load = GdbLoad::Done(r);
                    return;
                }
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
