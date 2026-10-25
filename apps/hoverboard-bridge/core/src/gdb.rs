//! The GDB server (`specs/hoverboard-bridge.md`, "Debugging"): the `gdbstub` target over the board
//! operations ([`crate::target`]), and its session, driven from the service loop without blocking
//! it.
//!
//! | GDB request | Here |
//! | --- | --- |
//! | target XML | [`TARGET_XML`]: the Cortex-M profile registers |
//! | memory map | [`memory_map`]: flash in 1 KB blocks up to the store, RAM at `0x2000_0000` |
//! | `g`, `G`, `p`, `P` | `DCRSR` / `DCRDR` with `S_REGRDY` polling ([`target::read_core_reg`]) |
//! | `m`, `M`, `X` | memory access, which works while the core runs |
//! | `c`, `s`, `vCont` | [`target::resume`], [`target::step`] |
//! | interrupt (`0x03`) | [`target::halt`], through disarm |
//! | stop reply | the `DHCSR` poll while running, the reason from `DFSR` |
//! | `Z0`, `Z1` | the flash patch unit's code comparators (flash cannot take a `bkpt`) |
//! | `Z2`, `Z3`, `Z4` | the DWT comparators |
//! | `vFlashErase`, `vFlashWrite`, `vFlashDone` | the image buffer and [`crate::flash::FlashJob`] through [`Service`] |
//! | `qRcmd` | `reset`, `reset halt`, `disarm`, `armed`, `id`, `counters`, `protection`, `clear-protection` |
//!
//! **Attach and detach.** A client connecting halts the core (through disarm), reads the comparator
//! counts from `FP_CTRL` and `DWT_CTRL`, and clears every comparator. A client leaving (`D`, `k`,
//! or the connection dropping) clears them again and resumes a halted core, as OpenOCD does on a
//! detach.
//!
//! **The armed-breakpoint rule.** While a session is open and the core may run, every service pass
//! polls `DHCSR`; a poll that finds the core halted reads `CCHP` at once and clears `MOE`
//! ([`target::poll_halt`]). The time from that poll to the clear is kept for `monitor counters`
//! and `ESP_INFO` (bench gate 9).
//!
//! **The mailbox.** While the session holds the core halted the mailbox port is paused
//! ([`MailboxPort::pause`]): the board's telemetry has stopped with its core, and the bridge does
//! not drive the mailbox until the resume.
//!
//! **`vFlashDone` without blocking.** A flash takes seconds and runs in steps from the service
//! loop. `gdbstub` answers a packet in the call that completes it, so the server watches the bytes
//! it feeds for `$vFlashDone#`: there it starts the flash job and holds the packet's two checksum
//! characters back until the job has finished, then feeds them, and the completed packet takes the
//! job's result. `$` and `#` never occur unescaped inside a packet, so binary `vFlashWrite` data
//! cannot match.

extern crate alloc;

use alloc::collections::VecDeque;
use core::num::NonZeroUsize;

use gdbstub::arch::{Arch, RegId, Registers};
use gdbstub::common::Signal;
use gdbstub::conn::Connection;
use gdbstub::outputln;
use gdbstub::stub::state_machine::GdbStubStateMachine;
use gdbstub::stub::{GdbStubBuilder, GdbStubError, SingleThreadStopReason};
use gdbstub::target::ext::base::single_register_access::{
    SingleRegisterAccess, SingleRegisterAccessOps,
};
use gdbstub::target::ext::base::singlethread::{
    SingleThreadBase, SingleThreadResume, SingleThreadResumeOps, SingleThreadSingleStep,
    SingleThreadSingleStepOps,
};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::target::ext::breakpoints::{
    Breakpoints, BreakpointsOps, HwBreakpoint, HwBreakpointOps, HwWatchpoint, HwWatchpointOps,
    SwBreakpoint, SwBreakpointOps, WatchKind,
};
use gdbstub::target::ext::flash::{Flash, FlashOps};
use gdbstub::target::ext::memory_map::{MemoryMap, MemoryMapOps};
use gdbstub::target::ext::monitor_cmd::{ConsoleOutput, MonitorCmd, MonitorCmdOps};
use gdbstub::target::{Target, TargetError as GdbError, TargetResult};
use heapless::Deque;

use crate::flash::{FLASH_BASE, PAGE, STORE_START};
use crate::mailbox::MailboxPort;
use crate::svc::{Platform, Service};
use crate::swd::{Swd, SwdPins};
use crate::target::{self, dfsr, reg, HaltPoll, TargetError, Watch};

/// The packet buffer, advertised to GDB as `PacketSize`: a `vFlashWrite` carries up to this much.
pub const PACKET: usize = 4096;
/// The most response bytes held for the client: one packet's worth (an `m` reply is two hex
/// characters a byte of at most half a packet), the framing, and a stop reply beside it.
const OUT_CAP: usize = PACKET + 512;
/// Client bytes waiting to be fed to the protocol.
pub const RX: usize = 1024;
/// The most comparators of each kind tracked (a Cortex-M3 has six code and four DWT comparators).
const MAX_COMP: usize = 8;
/// What a refused or failed request answers (`EIO`, `EBUSY`).
const EIO: u8 = 5;
const EBUSY: u8 = 16;

/// The Cortex-M profile registers, in the target XML's order (`specs/hoverboard-bridge.md`,
/// "Debugging"): `r0`-`r12`, `sp`, `lr`, `pc`, `xpsr`, `msp`, `psp` (32 bits each, `DCRSR`
/// selectors 0 to 18), then `primask`, `basepri`, `faultmask`, `control` (8 bits each, the bytes
/// of selector 20 from the bottom).
pub const TARGET_XML: &str = r#"<?xml version="1.0"?>
<target version="1.0">
<architecture>arm</architecture>
<feature name="org.gnu.gdb.arm.m-profile">
<reg name="r0" bitsize="32" regnum="0" type="uint32" group="general"/>
<reg name="r1" bitsize="32" regnum="1" type="uint32" group="general"/>
<reg name="r2" bitsize="32" regnum="2" type="uint32" group="general"/>
<reg name="r3" bitsize="32" regnum="3" type="uint32" group="general"/>
<reg name="r4" bitsize="32" regnum="4" type="uint32" group="general"/>
<reg name="r5" bitsize="32" regnum="5" type="uint32" group="general"/>
<reg name="r6" bitsize="32" regnum="6" type="uint32" group="general"/>
<reg name="r7" bitsize="32" regnum="7" type="uint32" group="general"/>
<reg name="r8" bitsize="32" regnum="8" type="uint32" group="general"/>
<reg name="r9" bitsize="32" regnum="9" type="uint32" group="general"/>
<reg name="r10" bitsize="32" regnum="10" type="uint32" group="general"/>
<reg name="r11" bitsize="32" regnum="11" type="uint32" group="general"/>
<reg name="r12" bitsize="32" regnum="12" type="uint32" group="general"/>
<reg name="sp" bitsize="32" regnum="13" type="data_ptr" group="general"/>
<reg name="lr" bitsize="32" regnum="14" type="code_ptr" group="general"/>
<reg name="pc" bitsize="32" regnum="15" type="code_ptr" group="general"/>
<reg name="xpsr" bitsize="32" regnum="16" type="uint32" group="general"/>
</feature>
<feature name="org.gnu.gdb.arm.m-system">
<reg name="msp" bitsize="32" regnum="17" type="data_ptr" group="system"/>
<reg name="psp" bitsize="32" regnum="18" type="data_ptr" group="system"/>
<reg name="primask" bitsize="8" regnum="19" type="uint8" group="system"/>
<reg name="basepri" bitsize="8" regnum="20" type="uint8" group="system"/>
<reg name="faultmask" bitsize="8" regnum="21" type="uint8" group="system"/>
<reg name="control" bitsize="8" regnum="22" type="uint8" group="system"/>
</feature>
</target>"#;

/// The 32-bit registers: `r0`-`r12`, `sp`, `lr`, `pc`, `xpsr`, `msp`, `psp`, which are also their
/// `DCRSR` selectors.
const CORE_REGS: usize = 19;
/// The 8-bit registers, the bytes of [`reg::SPECIAL`]: `primask`, `basepri`, `faultmask`, `control`.
const SPECIAL_REGS: usize = 4;

/// The register file `g` and `G` carry.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Regs {
    /// The 32-bit registers, by `DCRSR` selector.
    pub core: [u32; CORE_REGS],
    /// `primask`, `basepri`, `faultmask`, `control`.
    pub special: [u8; SPECIAL_REGS],
}

impl Registers for Regs {
    type ProgramCounter = u32;

    fn pc(&self) -> u32 {
        self.core[reg::PC as usize]
    }

    fn gdb_serialize(&self, mut write_byte: impl FnMut(Option<u8>)) {
        for r in self.core {
            r.to_le_bytes()
                .into_iter()
                .for_each(|b| write_byte(Some(b)));
        }
        self.special.into_iter().for_each(|b| write_byte(Some(b)));
    }

    fn gdb_deserialize(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if bytes.len() != 4 * CORE_REGS + SPECIAL_REGS {
            return Err(());
        }
        for (r, b) in self.core.iter_mut().zip(bytes.chunks_exact(4)) {
            *r = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        }
        self.special.copy_from_slice(&bytes[4 * CORE_REGS..]);
        Ok(())
    }
}

/// One register, by the target XML's number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reg {
    /// A 32-bit register: its `DCRSR` selector.
    Core(u32),
    /// A byte of [`reg::SPECIAL`]: 0 `primask`, 1 `basepri`, 2 `faultmask`, 3 `control`.
    Special(u32),
}

impl RegId for Reg {
    fn from_raw_id(id: usize) -> Option<(Self, Option<NonZeroUsize>)> {
        let (r, size) = match id {
            i if i < CORE_REGS => (Reg::Core(i as u32), 4),
            i if i < CORE_REGS + SPECIAL_REGS => (Reg::Special((i - CORE_REGS) as u32), 1),
            _ => return None,
        };
        Some((r, NonZeroUsize::new(size)))
    }
}

/// The Cortex-M profile as `gdbstub` sees it.
pub enum CortexM {}

impl Arch for CortexM {
    type Usize = u32;
    type Registers = Regs;
    type BreakpointKind = usize;
    type RegId = Reg;

    fn target_description_xml() -> Option<&'static str> {
        Some(TARGET_XML)
    }
}

/// The memory map GDB's `load` works from (`specs/hoverboard-bridge.md`, "Debugging"): flash at
/// [`FLASH_BASE`] in [`PAGE`] blocks, ending at the store's first page ([`STORE_START`]), so `load`
/// arrives as flash erase and write requests and cannot address the store; RAM at `0x2000_0000`.
/// With a map, GDB refuses any address outside it, so the RAM region runs to the end of the
/// address space and the other gaps are RAM too (as OpenOCD's map does): the peripherals, the
/// system block and the option bytes stay readable.
pub fn memory_map(out: &mut heapless::String<512>) {
    use core::fmt::Write;
    const RAM: u32 = 0x2000_0000;
    let _ = write!(
        out,
        "<?xml version=\"1.0\"?>\n<memory-map>\n\
         <memory type=\"ram\" start=\"0x0\" length=\"{FLASH_BASE:#x}\"/>\n\
         <memory type=\"flash\" start=\"{FLASH_BASE:#x}\" length=\"{:#x}\">\
         <property name=\"blocksize\">{PAGE:#x}</property></memory>\n\
         <memory type=\"ram\" start=\"{STORE_START:#x}\" length=\"{:#x}\"/>\n\
         <memory type=\"ram\" start=\"{RAM:#x}\" length=\"{:#x}\"/>\n\
         </memory-map>",
        STORE_START - FLASH_BASE,
        RAM - STORE_START,
        0u32.wrapping_sub(RAM),
    );
}

/// The connection's outgoing bytes, which the firmware drains to the TCP socket.
pub struct Conn {
    out: VecDeque<u8>,
}

/// The response did not fit [`OUT_CAP`] (a client asking for more than one packet's worth).
#[derive(Debug)]
pub struct Full;

impl Connection for Conn {
    type Error = Full;
    fn write(&mut self, byte: u8) -> Result<(), Full> {
        if self.out.len() >= OUT_CAP {
            return Err(Full);
        }
        self.out.push_back(byte);
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Full> {
        Ok(())
    }
}

/// A code comparator in use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Bp {
    addr: u32,
    /// Set by `Z0` (reported as a software breakpoint), not `Z1`.
    sw: bool,
}

/// A DWT comparator in use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Wp {
    addr: u32,
    len: u32,
    kind: WatchKind,
}

/// The target `gdbstub` drives: the board through the mailbox port's engine, the command service
/// for the image buffer, the flash job and the shared status, and the session's debug state.
pub struct Board<'b, 's, P: SwdPins, Pl: Platform> {
    port: &'b mut MailboxPort<P>,
    svc: &'b mut Service<'s, Pl>,
    bps: [Option<Bp>; MAX_COMP],
    n_bp: usize,
    wps: [Option<Wp>; MAX_COMP],
    n_wp: usize,
    /// The core may be running: the service loop polls `DHCSR`.
    running: bool,
    /// The last resume was a step.
    stepping: bool,
    /// A step left `C_MASKINTS` set.
    masked: bool,
    /// A halt was seen that the session has not reported yet.
    stopped: bool,
}

type Sm<'b, 's, P, Pl> = GdbStubStateMachine<'static, Board<'b, 's, P, Pl>, Conn>;

impl<'b, 's, P: SwdPins, Pl: Platform> Board<'b, 's, P, Pl> {
    /// The engine, connected, for a request; refused while a flash holds it.
    fn swd(&mut self) -> Result<&mut Swd<P>, GdbError<TargetError>> {
        if self.svc.busy() {
            return Err(GdbError::Errno(EBUSY));
        }
        if target::ensure_connected(self.port.swd()).is_err() {
            return Err(self.lost());
        }
        Ok(self.port.swd())
    }

    /// A request failed on the wire: forget the connection so the next one reconnects.
    fn lost(&mut self) -> GdbError<TargetError> {
        self.port.swd().forget();
        GdbError::Errno(EIO)
    }

    /// The session's view of the core: halted (mailbox paused) or running.
    fn set_running(&mut self, running: bool) {
        self.running = running;
        self.port.pause(!running);
        self.svc.gdb_status().halted = !running;
    }

    /// Halt for an attach or an interrupt, through disarm. `false` when the board refused (still
    /// armed) or did not answer.
    fn halt(&mut self) -> bool {
        let ok = self.swd().is_ok() && target::halt(self.port.swd()).is_ok();
        if ok {
            let _ = target::take_halt_reason(self.port.swd());
            self.set_running(false);
        }
        ok
    }

    /// Clear every comparator this session can use.
    fn clear_comparators(&mut self) {
        let swd = self.port.swd();
        for n in 0..self.n_bp {
            let _ = target::fpb_set(swd, n as u32, None);
        }
        for n in 0..self.n_wp {
            let _ = target::dwt_set(swd, n as u32, None);
        }
        self.bps = [None; MAX_COMP];
        self.wps = [None; MAX_COMP];
    }

    /// Open a session: halt, count the comparators, clear them, enable the flash patch unit.
    fn attach(&mut self) {
        self.svc.gdb_status().connected = true;
        let halted = self.halt();
        if self.swd().is_ok() {
            let swd = self.port.swd();
            self.n_bp = target::fpb_count(swd).map_or(0, |n| (n as usize).min(MAX_COMP));
            self.n_wp = target::dwt_count(swd).map_or(0, |n| (n as usize).min(MAX_COMP));
            self.clear_comparators();
            let _ = target::fpb_enable(self.port.swd());
        }
        self.stepping = false;
        self.masked = false;
        self.stopped = false;
        self.set_running(!halted);
    }

    /// Close a session: clear the comparators, and resume a halted core.
    fn detach(&mut self) {
        if !self.svc.busy() && target::ensure_connected(self.port.swd()).is_ok() {
            self.clear_comparators();
            if !self.running {
                let _ = self.resume_core();
            }
        }
        self.running = false;
        self.port.pause(false);
        self.svc.gdb_load_drop();
        let status = self.svc.gdb_status();
        status.connected = false;
        status.halted = false;
    }

    /// Clear the last halt's reason, unmask after a step, and let the core run.
    fn resume_core(&mut self) -> Result<(), crate::swd::SwdError> {
        let swd = self.port.swd();
        if self.masked {
            target::unmask(swd)?;
            self.masked = false;
        }
        target::take_halt_reason(swd)?;
        target::resume(swd)
    }

    /// The armed-breakpoint rule, once per service pass while the core may run: a halted core has
    /// `MOE` cleared at once and the time recorded.
    fn poll(&mut self) {
        if !self.running || self.svc.busy() {
            return;
        }
        match target::poll_halt(self.port.swd()) {
            Ok(HaltPoll::Running) => {}
            Ok(HaltPoll::Halted { moe_clear_us }) => {
                if let Some(us) = moe_clear_us {
                    self.svc.gdb_status().moe_clear_us = us;
                }
                self.set_running(false);
                self.stopped = true;
            }
            // A failed poll is tried again on the next pass; the engine's deadline bounds it.
            Err(_) => {}
        }
    }

    /// Why the core halted, from `DFSR` (cleared as it is read).
    fn stop_reason(&mut self) -> SingleThreadStopReason<u32> {
        let stepping = core::mem::replace(&mut self.stepping, false);
        let swd = self.port.swd();
        let d = target::take_halt_reason(swd).unwrap_or(0);
        if d & dfsr::DWTTRAP != 0 {
            for (n, wp) in self.wps.iter().enumerate().take(self.n_wp) {
                if let Some(wp) = wp {
                    if target::dwt_matched(swd, n as u32).unwrap_or(false) {
                        return SingleThreadStopReason::Watch {
                            tid: (),
                            kind: wp.kind,
                            addr: wp.addr,
                        };
                    }
                }
            }
        }
        if d & dfsr::BKPT != 0 {
            if let Ok(pc) = target::read_core_reg(swd, reg::PC) {
                if let Some(bp) = self.bps.iter().flatten().find(|b| b.addr == pc) {
                    return if bp.sw {
                        SingleThreadStopReason::SwBreak(())
                    } else {
                        SingleThreadStopReason::HwBreak(())
                    };
                }
            }
        }
        if d & dfsr::HALTED != 0 && d & (dfsr::BKPT | dfsr::DWTTRAP) == 0 {
            return if stepping {
                SingleThreadStopReason::DoneStep
            } else {
                SingleThreadStopReason::Signal(Signal::SIGINT)
            };
        }
        SingleThreadStopReason::Signal(Signal::SIGTRAP)
    }

    fn add_bp(&mut self, addr: u32, sw: bool) -> TargetResult<bool, Self> {
        if addr >= target::FP_CODE_END {
            return Ok(false);
        }
        let Some(n) = self.bps[..self.n_bp].iter().position(Option::is_none) else {
            return Ok(false);
        };
        let swd = self.swd()?;
        if target::fpb_set(swd, n as u32, Some(addr)).is_err() {
            return Err(self.lost());
        }
        self.bps[n] = Some(Bp { addr, sw });
        Ok(true)
    }

    fn remove_bp(&mut self, addr: u32, sw: bool) -> TargetResult<bool, Self> {
        let Some(n) = self.bps.iter().position(|b| *b == Some(Bp { addr, sw })) else {
            return Ok(false);
        };
        let swd = self.swd()?;
        if target::fpb_set(swd, n as u32, None).is_err() {
            return Err(self.lost());
        }
        self.bps[n] = None;
        Ok(true)
    }

    fn read_reg(&mut self, r: Reg) -> TargetResult<u32, Self> {
        let swd = self.swd()?;
        let v = match r {
            Reg::Core(sel) => target::read_core_reg(swd, sel),
            Reg::Special(i) => target::read_core_reg(swd, reg::SPECIAL).map(|w| w >> (8 * i)),
        };
        v.map_err(|_| self.lost())
    }

    fn write_reg(&mut self, r: Reg, v: u32) -> TargetResult<(), Self> {
        let swd = self.swd()?;
        let done = match r {
            Reg::Core(sel) => target::write_core_reg(swd, sel, v),
            Reg::Special(i) => target::read_core_reg(swd, reg::SPECIAL).and_then(|w| {
                let shift = 8 * i;
                let w = w & !(0xFF << shift) | (v & 0xFF) << shift;
                target::write_core_reg(swd, reg::SPECIAL, w)
            }),
        };
        done.map_err(|_| self.lost())
    }

    /// A flash request's range, as an offset into the image buffer; refused when it reaches the
    /// store's pages or lies outside flash.
    fn image_range(addr: u32, len: u32) -> TargetResult<u32, Self> {
        if addr < FLASH_BASE || addr.saturating_add(len) > STORE_START {
            return Err(GdbError::Errno(EIO));
        }
        Ok(addr - FLASH_BASE)
    }
}

impl<P: SwdPins, Pl: Platform> Target for Board<'_, '_, P, Pl> {
    type Arch = CortexM;
    type Error = TargetError;

    fn base_ops(&mut self) -> BaseOps<'_, CortexM, TargetError> {
        BaseOps::SingleThread(self)
    }
    fn support_breakpoints(&mut self) -> Option<BreakpointsOps<'_, Self>> {
        Some(self)
    }
    fn support_monitor_cmd(&mut self) -> Option<MonitorCmdOps<'_, Self>> {
        Some(self)
    }
    fn support_memory_map(&mut self) -> Option<MemoryMapOps<'_, Self>> {
        Some(self)
    }
    fn support_flash_operations(&mut self) -> Option<FlashOps<'_, Self>> {
        Some(self)
    }
}

impl<P: SwdPins, Pl: Platform> SingleThreadBase for Board<'_, '_, P, Pl> {
    fn read_registers(&mut self, regs: &mut Regs) -> TargetResult<(), Self> {
        for (sel, r) in regs.core.iter_mut().enumerate() {
            *r = self.read_reg(Reg::Core(sel as u32))?;
        }
        let w = self.read_reg(Reg::Special(0))?;
        regs.special = w.to_le_bytes();
        Ok(())
    }

    fn write_registers(&mut self, regs: &Regs) -> TargetResult<(), Self> {
        for (sel, r) in regs.core.iter().enumerate() {
            self.write_reg(Reg::Core(sel as u32), *r)?;
        }
        let swd = self.swd()?;
        let w = u32::from_le_bytes(regs.special);
        target::write_core_reg(swd, reg::SPECIAL, w).map_err(|_| self.lost())
    }

    fn support_single_register_access(&mut self) -> Option<SingleRegisterAccessOps<'_, (), Self>> {
        Some(self)
    }

    fn read_addrs(&mut self, start: u32, data: &mut [u8]) -> TargetResult<usize, Self> {
        let swd = self.swd()?;
        match swd.read_bytes(start, data) {
            Ok(()) => Ok(data.len()),
            Err(_) => Err(self.lost()),
        }
    }

    fn write_addrs(&mut self, start: u32, data: &[u8]) -> TargetResult<(), Self> {
        let swd = self.swd()?;
        swd.write_bytes(start, data).map_err(|_| self.lost())
    }

    fn support_resume(&mut self) -> Option<SingleThreadResumeOps<'_, Self>> {
        Some(self)
    }
}

impl<P: SwdPins, Pl: Platform> SingleRegisterAccess<()> for Board<'_, '_, P, Pl> {
    fn read_register(&mut self, _: (), r: Reg, buf: &mut [u8]) -> TargetResult<usize, Self> {
        let v = self.read_reg(r)?.to_le_bytes();
        let n = match r {
            Reg::Core(_) => 4,
            Reg::Special(_) => 1,
        };
        buf[..n].copy_from_slice(&v[..n]);
        Ok(n)
    }

    fn write_register(&mut self, _: (), r: Reg, val: &[u8]) -> TargetResult<(), Self> {
        let mut b = [0u8; 4];
        let n = val.len().min(4);
        b[..n].copy_from_slice(&val[..n]);
        self.write_reg(r, u32::from_le_bytes(b))
    }
}

impl<P: SwdPins, Pl: Platform> SingleThreadResume for Board<'_, '_, P, Pl> {
    /// Resume. A failed write leaves the core halted, which the next poll reports as a stop, so
    /// the session carries on.
    fn resume(&mut self, _signal: Option<Signal>) -> Result<(), TargetError> {
        self.stepping = false;
        if self.swd().is_ok() {
            let _ = self.resume_core();
        }
        self.set_running(true);
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<SingleThreadSingleStepOps<'_, Self>> {
        Some(self)
    }
}

impl<P: SwdPins, Pl: Platform> SingleThreadSingleStep for Board<'_, '_, P, Pl> {
    /// Step one instruction, through disarm. A refused step (the board still armed) leaves the
    /// core halted with no `DFSR` reason, which the poll reports as `SIGTRAP`, not a step.
    fn step(&mut self, _signal: Option<Signal>) -> Result<(), TargetError> {
        if self.swd().is_ok() {
            let swd = self.port.swd();
            let _ = target::take_halt_reason(swd);
            if target::step(swd).is_ok() {
                self.stepping = true;
                self.masked = true;
            }
        }
        self.set_running(true);
        Ok(())
    }
}

impl<P: SwdPins, Pl: Platform> Breakpoints for Board<'_, '_, P, Pl> {
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        Some(self)
    }
    fn support_hw_breakpoint(&mut self) -> Option<HwBreakpointOps<'_, Self>> {
        Some(self)
    }
    fn support_hw_watchpoint(&mut self) -> Option<HwWatchpointOps<'_, Self>> {
        Some(self)
    }
}

/// `Z0`: flash cannot hold a `bkpt` instruction, so a software breakpoint is a code comparator too.
impl<P: SwdPins, Pl: Platform> SwBreakpoint for Board<'_, '_, P, Pl> {
    fn add_sw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.add_bp(addr, true)
    }
    fn remove_sw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.remove_bp(addr, true)
    }
}

impl<P: SwdPins, Pl: Platform> HwBreakpoint for Board<'_, '_, P, Pl> {
    fn add_hw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.add_bp(addr, false)
    }
    fn remove_hw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.remove_bp(addr, false)
    }
}

impl<P: SwdPins, Pl: Platform> HwWatchpoint for Board<'_, '_, P, Pl> {
    /// A DWT comparator watches a power-of-two range aligned to its size; anything else is
    /// refused.
    fn add_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        if !len.is_power_of_two() || !addr.is_multiple_of(len) {
            return Ok(false);
        }
        let Some(n) = self.wps[..self.n_wp].iter().position(Option::is_none) else {
            return Ok(false);
        };
        let watch = match kind {
            WatchKind::Read => Watch::Read,
            WatchKind::Write => Watch::Write,
            WatchKind::ReadWrite => Watch::Access,
        };
        let swd = self.swd()?;
        if target::dwt_set(swd, n as u32, Some((addr, len.trailing_zeros(), watch))).is_err() {
            return Err(self.lost());
        }
        self.wps[n] = Some(Wp { addr, len, kind });
        Ok(true)
    }

    fn remove_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        let Some(n) = self
            .wps
            .iter()
            .position(|w| *w == Some(Wp { addr, len, kind }))
        else {
            return Ok(false);
        };
        let swd = self.swd()?;
        if target::dwt_set(swd, n as u32, None).is_err() {
            return Err(self.lost());
        }
        self.wps[n] = None;
        Ok(true)
    }
}

impl<P: SwdPins, Pl: Platform> MemoryMap for Board<'_, '_, P, Pl> {
    fn memory_map_xml(
        &self,
        offset: u64,
        length: usize,
        buf: &mut [u8],
    ) -> TargetResult<usize, Self> {
        let mut xml = heapless::String::new();
        memory_map(&mut xml);
        let xml = xml.as_bytes();
        let start = (offset as usize).min(xml.len());
        let n = length.min(xml.len() - start).min(buf.len());
        buf[..n].copy_from_slice(&xml[start..start + n]);
        Ok(n)
    }
}

/// `load`: the erases and writes fill the image buffer, and nothing touches the board until
/// `vFlashDone` ([`Gdb`] starts the flash job when that packet arrives; the packet takes its result).
impl<P: SwdPins, Pl: Platform> Flash for Board<'_, '_, P, Pl> {
    fn flash_erase(&mut self, addr: u32, len: u32) -> TargetResult<(), Self> {
        let off = Self::image_range(addr, len)?;
        let image = self.svc.gdb_image().map_err(|_| GdbError::Errno(EBUSY))?;
        image.erase(off, len).map_err(|_| GdbError::Errno(EIO))
    }

    fn flash_write(&mut self, addr: u32, data: &[u8]) -> TargetResult<(), Self> {
        let off = Self::image_range(addr, data.len() as u32)?;
        let image = self.svc.gdb_image().map_err(|_| GdbError::Errno(EBUSY))?;
        image.place(off, data).map_err(|_| GdbError::Errno(EIO))
    }

    /// The job's result: a refused part, an erase or program error, or a verify mismatch fails the
    /// packet with the `ESP_INFO`-style status as the error number.
    fn flash_done(&mut self) -> TargetResult<(), Self> {
        match self.svc.gdb_load_result() {
            Some(Ok(())) => {
                // The job left the core halted at the new image's reset vector.
                self.stopped = false;
                self.set_running(false);
                Ok(())
            }
            Some(Err(e)) => Err(GdbError::Errno(e.status() as u8)),
            None => Err(GdbError::Errno(EBUSY)),
        }
    }
}

impl<P: SwdPins, Pl: Platform> MonitorCmd for Board<'_, '_, P, Pl> {
    fn handle_monitor_cmd(
        &mut self,
        cmd: &[u8],
        out: ConsoleOutput<'_>,
    ) -> Result<(), TargetError> {
        let mut out = out;
        let cmd = core::str::from_utf8(cmd).unwrap_or("").trim();
        if self.svc.busy() {
            outputln!(out, "busy: a flash holds the engine");
            return Ok(());
        }
        let connected = target::ensure_connected(self.port.swd()).is_ok();
        if !connected && cmd != "counters" {
            self.port.swd().forget();
            outputln!(out, "no target on SWD");
            return Ok(());
        }
        match cmd {
            "reset" => match target::reset(self.port.swd()) {
                Ok(()) => {
                    self.set_running(true);
                    outputln!(out, "reset: the core runs");
                }
                Err(_) => outputln!(out, "reset failed on SWD"),
            },
            "reset halt" => match target::reset_halt(self.port.swd()) {
                Ok(()) => {
                    let _ = target::take_halt_reason(self.port.swd());
                    self.set_running(false);
                    outputln!(out, "reset: halted at the reset vector");
                }
                Err(e) => outputln!(out, "reset halt failed: {}", describe(e)),
            },
            "disarm" => match target::disarm(self.port.swd()) {
                Ok(()) => outputln!(out, "disarmed: MOE clear"),
                Err(e) => outputln!(out, "disarm failed: {}", describe(e)),
            },
            "armed" => match target::armed(self.port.swd()) {
                Ok(true) => outputln!(out, "armed: MOE set"),
                Ok(false) => outputln!(out, "disarmed: MOE clear"),
                Err(_) => outputln!(out, "CCHP read failed on SWD"),
            },
            "id" => match self.svc.target_id(self.port) {
                Ok(id) => {
                    let family = match id.family {
                        espsvc::FAMILY_F10X => "GD32F10x",
                        espsvc::FAMILY_F1X0 => "GD32F1x0",
                        _ => "unknown",
                    };
                    outputln!(out, "dpidr   {:#010x}", id.dpidr);
                    outputln!(out, "mcu id  {:#010x}", id.mcu_id);
                    outputln!(out, "cpuid   {:#010x}", id.cpuid);
                    outputln!(out, "family  {family}");
                    let mut uid: heapless::String<24> = heapless::String::new();
                    for b in id.uid {
                        let _ = core::fmt::Write::write_fmt(&mut uid, format_args!("{b:02x}"));
                    }
                    outputln!(out, "uid     {uid}");
                    outputln!(out, "flash   {} KiB", id.flash_kib);
                    outputln!(out, "armed   {}", if id.armed { "YES" } else { "no" });
                }
                Err(_) => outputln!(out, "identify failed on SWD"),
            },
            "counters" => {
                let c = self.port.swd().counters();
                outputln!(
                    out,
                    "swd     wait {} fault {} parity {} reconnect {}",
                    c.waits,
                    c.faults,
                    c.parity_errors,
                    c.reconnects
                );
                match self.svc.gdb_status().moe_clear_us {
                    espsvc::MOE_CLEAR_NONE => outputln!(out, "halt to MOE clear: none yet"),
                    us => outputln!(out, "halt to MOE clear: {us} us"),
                }
            }
            "protection" => match self.svc.target_id(self.port) {
                Ok(id) => {
                    if id.rdp == target::RDP_OFF {
                        outputln!(out, "read protection off ({:#04x})", id.rdp);
                    } else {
                        outputln!(out, "read protection ON ({:#04x})", id.rdp);
                    }
                    outputln!(out, "user    {:#04x}", id.user);
                    match id.clear_check {
                        espsvc::CLEAR_PASS => outputln!(out, "clear   took (0xA5, USER unchanged)"),
                        espsvc::CLEAR_FAIL => outputln!(out, "clear   FAILED: check after a power cycle"),
                        _ => {}
                    }
                }
                Err(_) => outputln!(out, "identify failed on SWD"),
            },
            "clear-protection" => match self.svc.clear_protection(self.port) {
                Ok(()) => outputln!(
                    out,
                    "cleared (the chip is mass-erased): power-cycle the board, then monitor protection"
                ),
                Err(e) => outputln!(out, "clear failed: {:?}", e),
            },
            _ => outputln!(
                out,
                "monitor commands: reset, reset halt, disarm, armed, id, counters, protection, clear-protection"
            ),
        }
        Ok(())
    }
}

fn describe(e: TargetError) -> &'static str {
    match e {
        TargetError::Armed => "MOE still set after the disarm: cut the rail",
        TargetError::Swd(_) => "SWD failed",
        TargetError::Timeout => "the core did not halt",
    }
}

/// The GDB server: one client at a time, its bytes fed to `gdbstub` from the service loop, its
/// replies drained by the firmware. It owns the board's handles; the rest of the service loop
/// borrows them through [`Gdb::parts`].
pub struct Gdb<'b, 's, P: SwdPins, Pl: Platform> {
    board: Board<'b, 's, P, Pl>,
    sm: Option<Sm<'b, 's, P, Pl>>,
    rx: Deque<u8, RX>,
    /// Bytes of `$vFlashDone#` matched in the stream fed so far.
    done: usize,
    /// The `vFlashDone` checksum is held back while the flash job runs.
    held: bool,
    /// The session ended from the protocol side (`D`, `k`, or an error): the firmware closes the
    /// connection.
    ended: bool,
}

const FLASH_DONE: &[u8] = b"$vFlashDone#";

impl<'b, 's, P: SwdPins, Pl: Platform> Gdb<'b, 's, P, Pl> {
    /// A server over the mailbox port (the engine's owner) and the command service.
    pub fn new(port: &'b mut MailboxPort<P>, svc: &'b mut Service<'s, Pl>) -> Self {
        Gdb {
            board: Board {
                port,
                svc,
                bps: [None; MAX_COMP],
                n_bp: 0,
                wps: [None; MAX_COMP],
                n_wp: 0,
                running: false,
                stepping: false,
                masked: false,
                stopped: false,
            },
            sm: None,
            rx: Deque::new(),
            done: 0,
            held: false,
            ended: false,
        }
    }

    /// The mailbox port and the service, for the rest of the service loop.
    pub fn parts(&mut self) -> (&mut MailboxPort<P>, &mut Service<'s, Pl>) {
        (&mut *self.board.port, &mut *self.board.svc)
    }

    /// A client is connected.
    pub fn connected(&self) -> bool {
        self.sm.is_some()
    }

    /// A client connected: attach (halt, comparators) and start the protocol.
    pub fn open(&mut self) {
        self.close();
        self.board.attach();
        let conn = Conn {
            out: VecDeque::with_capacity(OUT_CAP),
        };
        let started = GdbStubBuilder::new(conn)
            .packet_buffer_size(PACKET)
            .build()
            .map_err(|_| ())
            .and_then(|stub| stub.run_state_machine(&mut self.board).map_err(|_| ()));
        match started {
            Ok(sm) => self.sm = Some(sm),
            Err(()) => self.end(),
        }
    }

    /// The client went away: detach.
    pub fn close(&mut self) {
        if self.sm.take().is_some() {
            self.board.detach();
        }
        self.rx.clear();
        self.done = 0;
        self.held = false;
        self.ended = false;
    }

    /// The session ended on the protocol side; the firmware should close the connection.
    /// Reading it clears it.
    pub fn take_ended(&mut self) -> bool {
        core::mem::replace(&mut self.ended, false)
    }

    fn end(&mut self) {
        if self.sm.take().is_some() {
            self.board.detach();
        }
        self.board.svc.gdb_status().connected = false;
        self.rx.clear();
        self.ended = true;
    }

    /// Room for client bytes.
    pub fn room(&self) -> usize {
        RX - self.rx.len()
    }

    /// Queue client bytes, up to [`room`](Self::room); the rest are the caller's to offer again.
    /// Returns how many were taken. Bytes with no session are dropped.
    pub fn push(&mut self, data: &[u8]) -> usize {
        if self.sm.is_none() {
            return data.len();
        }
        let mut n = 0;
        for &b in data {
            if self.rx.push_back(b).is_err() {
                break;
            }
            n += 1;
        }
        n
    }

    /// Move up to `buf.len()` reply bytes out; how many.
    pub fn take_out(&mut self, buf: &mut [u8]) -> usize {
        let Some(conn) = self.conn() else {
            return 0;
        };
        let n = buf.len().min(conn.out.len());
        for (d, s) in buf.iter_mut().zip(conn.out.drain(..n)) {
            *d = s;
        }
        n
    }

    fn conn(&mut self) -> Option<&mut Conn> {
        Some(match self.sm.as_mut()? {
            GdbStubStateMachine::Idle(s) => s.borrow_conn(),
            GdbStubStateMachine::Running(s) => s.borrow_conn(),
            GdbStubStateMachine::CtrlCInterrupt(s) => s.borrow_conn(),
            GdbStubStateMachine::Disconnected(s) => s.borrow_conn(),
        })
    }

    fn out_len(&mut self) -> usize {
        self.conn().map_or(0, |c| c.out.len())
    }

    /// One service pass: the armed-breakpoint rule while the core may run, a stop reply for a
    /// halt seen, and the client's queued bytes fed while the replies have room.
    pub fn poll(&mut self) {
        if self.sm.is_none() {
            return;
        }
        self.board.poll();
        if self.board.stopped && self.out_len() + 64 <= OUT_CAP {
            self.board.stopped = false;
            match self.sm.take() {
                Some(GdbStubStateMachine::Running(s)) => {
                    let reason = self.board.stop_reason();
                    let next = s.report_stop(&mut self.board, reason);
                    self.settle(next);
                }
                // GDB is not waiting for a stop (a halt after `monitor reset`): only the reason
                // is cleared.
                other => {
                    let _ = target::take_halt_reason(self.board.port.swd());
                    self.sm = other;
                }
            }
        }
        self.feed();
    }

    fn feed(&mut self) {
        loop {
            if self.held {
                if self.board.svc.gdb_load_running() {
                    return;
                }
                self.held = false;
            }
            if self.out_len() != 0 {
                return;
            }
            let Some(sm) = self.sm.take() else {
                return;
            };
            let Some(b) = self.rx.pop_front() else {
                self.sm = Some(sm);
                return;
            };
            let next = match sm {
                GdbStubStateMachine::Idle(s) => s.incoming_data(&mut self.board, b),
                GdbStubStateMachine::Running(s) => s.incoming_data(&mut self.board, b),
                other => Ok(other),
            };
            self.settle(next);
            self.done = if b == FLASH_DONE[self.done] {
                self.done + 1
            } else {
                usize::from(b == b'$')
            };
            if self.done == FLASH_DONE.len() {
                self.done = 0;
                self.held = true;
                let (port, svc) = self.parts();
                svc.gdb_load_start(port);
            }
        }
    }

    /// Store the state machine's next state, answering an interrupt and ending the session on a
    /// disconnect or an error.
    fn settle(&mut self, next: Result<Sm<'b, 's, P, Pl>, GdbStubError<TargetError, Full>>) {
        let sm = match next {
            Ok(GdbStubStateMachine::CtrlCInterrupt(s)) => {
                let reason = self
                    .board
                    .halt()
                    .then_some(SingleThreadStopReason::<u32>::Signal(Signal::SIGINT));
                s.interrupt_handled(&mut self.board, reason)
            }
            other => other,
        };
        match sm {
            Ok(GdbStubStateMachine::Disconnected(_)) | Err(_) => self.end(),
            Ok(sm) => self.sm = Some(sm),
        }
    }
}
