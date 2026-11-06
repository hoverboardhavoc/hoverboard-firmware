//! Board operations (`specs/hoverboard-bridge.md`, "Board operations"): each operation OpenOCD performed for
//! the tools, as a short register sequence on the SWD engine. The ESP commands and the flash module
//! call these rather than writing the registers themselves.
//!
//! **Every operation that halts goes through [`disarm`] first**, here, in the one module that
//! halts: a halt freezes timer 0 with its outputs at their last level, so a phase can sit at 100%
//! duty (`tools/swd-disarm-halt.sh` carries the standing rule and the incident). The disarm clears
//! every advanced timer of the part's row: timer 0's, and timer 7's on the GD32F103RC
//! ([`part::Part::timers`]).

use espsvc::{TargetId, FAMILY_F10X, FAMILY_F1X0, FAMILY_UNSUPPORTED};

use crate::part;
use crate::swd::{MemPort, Swd, SwdError, SwdPins};

/// The GD32 debug MCU ID.
pub const DBGMCU_ID: u32 = 0xE004_2000;
/// The MM32's `FLASH_OBR`: the option bytes as loaded, readable while protected.
pub const MM32_FLASH_OBR: u32 = 0x4002_201C;
/// `FLASH_OBR` `RDPRT`: read protection on.
pub const MM32_OBR_RDPRT: u32 = 1 << 1;
/// `CPUID`.
pub const CPUID: u32 = 0xE000_ED00;
/// `DHCSR`.
pub const DHCSR: u32 = 0xE000_EDF0;
/// `AIRCR`.
pub const AIRCR: u32 = 0xE000_ED0C;
/// `DEMCR`.
pub const DEMCR: u32 = 0xE000_EDFC;
/// `DCRSR`: selects a core register for a transfer through [`DCRDR`].
pub const DCRSR: u32 = 0xE000_EDF4;
/// `DCRDR`: the core register transfer's data.
pub const DCRDR: u32 = 0xE000_EDF8;
/// `DCRSR` `REGWnR`: the transfer writes the register.
const REGWNR: u32 = 1 << 16;
/// `DHCSR` `S_REGRDY`: the last core register transfer finished.
const S_REGRDY: u32 = 1 << 16;
/// `DHCSR` reads before a core register transfer counts as stuck. It finishes in a few core
/// cycles, well inside one SWD transfer.
const REG_POLLS: u32 = 16;
/// Timer 0 `CCHP`; `MOE` is bit 15. The MM32's TIM1 `BDTR` is at the same address with `MOE` at
/// the same bit (MindMotion `reg_tim.h`).
pub const CCHP: u32 = part::TIMER0_CCHP;
const MOE: u32 = 1 << 15;
const DBGKEY: u32 = 0xA05F_0000;
const C_DEBUGEN: u32 = 1;
const C_HALT: u32 = 1 << 1;
const C_STEP: u32 = 1 << 2;
const C_MASKINTS: u32 = 1 << 3;
const S_HALT: u32 = 1 << 17;
const SYSRESETREQ: u32 = 0x05FA_0004;
const VC_CORERESET: u32 = 1;
/// `DEMCR` `TRCENA`: powers the DWT.
const TRCENA: u32 = 1 << 24;
/// `DFSR`: why the core last halted (write ones to clear).
pub const DFSR: u32 = 0xE000_ED30;
/// `DFSR` bits.
pub mod dfsr {
    /// A halt request or a step.
    pub const HALTED: u32 = 1;
    /// A breakpoint: the flash patch unit or a `bkpt` instruction.
    pub const BKPT: u32 = 1 << 1;
    /// A DWT watchpoint.
    pub const DWTTRAP: u32 = 1 << 2;
    /// A vector catch.
    pub const VCATCH: u32 = 1 << 3;
    /// The external debug request.
    pub const EXTERNAL: u32 = 1 << 4;
}
/// The flash patch unit's control register: comparator count, enable.
pub const FP_CTRL: u32 = 0xE000_2000;
/// The first flash patch comparator; the rest follow a word apart.
pub const FP_COMP0: u32 = 0xE000_2008;
/// `FP_CTRL` `KEY` and `ENABLE`: a write takes only with `KEY` set.
const FP_KEY_ENABLE: u32 = 0b11;
/// The flash patch unit compares addresses below this (the code region) only.
pub const FP_CODE_END: u32 = 0x2000_0000;
/// The DWT's control register: comparator count in bits 31:28.
pub const DWT_CTRL: u32 = 0xE000_1000;
/// The first DWT comparator; each has `COMP`, `MASK` and `FUNCTION`, 16 B apart.
pub const DWT_COMP0: u32 = 0xE000_1020;
/// `DWT_FUNCTION` `MATCHED`: this comparator fired since the register was last read.
const DWT_MATCHED: u32 = 1 << 24;
/// The option byte block: the protection byte, then USER.
pub const OPTION_BYTES: u32 = 0x1FFF_F800;
/// The GD32 flash size register (KiB in the low half), the same address on both families.
pub const FLASH_SIZE: u32 = 0x1FFF_F7E0;
/// The F10x UID, and the MM32's. On an F1x0 this word reads all ones.
pub const UID_F10X: u32 = 0x1FFF_F7E8;
/// The F1x0 UID.
pub const UID_F1X0: u32 = 0x1FFF_F7AC;
/// The protection byte value that means read protection off.
pub const RDP_OFF: u8 = 0xA5;

/// The core registers' `DCRSR` selectors.
pub mod reg {
    /// `r0`; `r1` to `r12` follow.
    pub const R0: u32 = 0;
    /// `r1`.
    pub const R1: u32 = 1;
    /// `r2`.
    pub const R2: u32 = 2;
    /// `r3`.
    pub const R3: u32 = 3;
    /// The current stack pointer.
    pub const SP: u32 = 13;
    /// The link register.
    pub const LR: u32 = 14;
    /// The debug return address: where the core resumes.
    pub const PC: u32 = 15;
    /// `xPSR`.
    pub const XPSR: u32 = 16;
    /// The main stack pointer.
    pub const MSP: u32 = 17;
    /// The process stack pointer.
    pub const PSP: u32 = 18;
    /// `CONTROL`, `FAULTMASK`, `BASEPRI` and `PRIMASK`, one byte each from the top.
    pub const SPECIAL: u32 = 20;
    /// `xPSR`'s Thumb bit, which must be set for the core to execute at all.
    pub const XPSR_T: u32 = 1 << 24;
}

/// Why an operation over a [`MemPort`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortError<E> {
    /// The access failed.
    Port(E),
    /// The core did not report the register transfer finished.
    Timeout,
}

impl<E> From<E> for PortError<E> {
    fn from(e: E) -> Self {
        PortError::Port(e)
    }
}

impl From<PortError<SwdError>> for TargetError {
    fn from(e: PortError<SwdError>) -> Self {
        match e {
            PortError::Port(e) => TargetError::Swd(e),
            PortError::Timeout => TargetError::Timeout,
        }
    }
}

/// Why a board operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetError {
    /// The SWD engine gave up.
    Swd(SwdError),
    /// `MOE` still reads set after the disarm write: refuse, and report "cut the rail".
    Armed,
    /// The core did not report halted in time.
    Timeout,
}

impl From<SwdError> for TargetError {
    fn from(e: SwdError) -> Self {
        TargetError::Swd(e)
    }
}

/// Connect if the engine is not connected, for a command: the connect tries for its window
/// ([`Swd::connect`]).
pub fn ensure_connected<P: SwdPins>(swd: &mut Swd<P>) -> Result<u32, SwdError> {
    match swd.dpidr() {
        Some(id) => Ok(id),
        None => swd.connect(),
    }
}

/// Connect if the engine is not connected, for the mailbox session's background retry: one wake
/// ([`Swd::connect_background`]).
pub fn ensure_connected_background<P: SwdPins>(swd: &mut Swd<P>) -> Result<u32, SwdError> {
    match swd.dpidr() {
        Some(id) => Ok(id),
        None => swd.connect_background(),
    }
}

/// Identify the target (`specs/bridge-chip-support.md`, section 3). It reads, in this order, only
/// addresses every supported part decodes: `CPUID` first, then
///
/// - on a Cortex-M0, the ID register at [`part::M0_ID`]; for the MM32SPIN05 `FLASH_OBR`,
///   [`CCHP`] and the UID at `0x1FFF_F7E8` (0 while protected). The flash size is the row's: the
///   MM32 has no flash size register. `rdp` is `0xA5` with `FLASH_OBR` `RDPRT` clear and 0 with it
///   set; `user` is `0xFF` (unreadable while protected, and the clear does not restore it);
/// - on a Cortex-M3, a GD32: the debug MCU ID, the option bytes, the flash size, [`CCHP`], then
///   the UID by family. The debug MCU ID does not tell the families apart (it reads `0x13030410`
///   on both); the UID's location does: `0x1FFF_F7E8` reads all ones on an F1x0. Last, when the
///   family and flash size name a row with a second advanced timer (the GD32F103RC), that timer's
///   `CCHP` for `armed`: it is read only once the row is known;
/// - on any other core, or a Cortex-M0 in no row: nothing more, [`FAMILY_UNSUPPORTED`], with
///   `cpuid`, `mcu_id` (the M0's ID register, else 0) and `dpidr` filled and the rest 0.
///
/// `clear_check` is the caller's.
///
/// It connects afresh every time (line reset, `DPIDR`, power-up), so each `TARGET_ID` is one
/// connect: bench gate 1 counts them. A line reset leaves the debug port's registers as they
/// were, so a mailbox session running beside it carries on.
pub fn identify<P: SwdPins>(swd: &mut Swd<P>) -> Result<TargetId, SwdError> {
    let dpidr = swd.connect()?;
    let cpuid = swd.read_word(CPUID)?;
    let unsupported = |mcu_id| TargetId {
        dpidr,
        mcu_id,
        cpuid,
        family: FAMILY_UNSUPPORTED,
        clear_check: espsvc::CLEAR_NONE,
        ..TargetId::default()
    };
    match part::partno(cpuid) {
        part::PARTNO_M0 => {
            let mcu_id = swd.read_word(part::M0_ID)?;
            match part::m0(mcu_id) {
                Some(row) => identify_mm32(swd, dpidr, cpuid, mcu_id, row),
                None => Ok(unsupported(mcu_id)),
            }
        }
        part::PARTNO_M3 => identify_gd32(swd, dpidr, cpuid),
        _ => Ok(unsupported(0)),
    }
}

/// The GD32 reads, in the order `identify` has always made them.
fn identify_gd32<P: SwdPins>(
    swd: &mut Swd<P>,
    dpidr: u32,
    cpuid: u32,
) -> Result<TargetId, SwdError> {
    let mcu_id = swd.read_word(DBGMCU_ID)?;
    let ob = swd.read_word(OPTION_BYTES)?;
    let flash_kib = swd.read_word(FLASH_SIZE)? as u16;
    let mut armed = swd.read_word(CCHP)? & MOE != 0;
    let (family, uid_addr) = gd32_family(swd)?;
    if let Some(row) = part::lookup(family, flash_kib) {
        armed |= any_armed(swd, &row.timers[1..])?;
    }
    Ok(TargetId {
        dpidr,
        mcu_id,
        cpuid,
        family,
        uid: read_uid(swd, uid_addr)?,
        rdp: ob as u8,
        user: (ob >> 16) as u8,
        clear_check: espsvc::CLEAR_NONE,
        flash_kib,
        armed,
    })
}

/// The MM32SPIN05 reads: `FLASH_OBR`, [`CCHP`], the UID. Nothing at `0xE004_2000`,
/// `0x1FFF_F7E0` or `0x1FFF_F800`.
fn identify_mm32<P: SwdPins>(
    swd: &mut Swd<P>,
    dpidr: u32,
    cpuid: u32,
    mcu_id: u32,
    row: &part::Part,
) -> Result<TargetId, SwdError> {
    let obr = swd.read_word(MM32_FLASH_OBR)?;
    let armed = any_armed(swd, row.timers)?;
    Ok(TargetId {
        dpidr,
        mcu_id,
        cpuid,
        family: row.family,
        uid: read_uid(swd, UID_F10X)?,
        rdp: if obr & MM32_OBR_RDPRT == 0 {
            RDP_OFF
        } else {
            0
        },
        user: 0xFF,
        clear_check: espsvc::CLEAR_NONE,
        flash_kib: row.flash_kib,
        armed,
    })
}

/// The GD32 family and its UID's address, by the UID probe: `0x1FFF_F7E8` reads all ones on an
/// F1x0.
fn gd32_family<P: SwdPins>(swd: &mut Swd<P>) -> Result<(u8, u32), SwdError> {
    Ok(if swd.read_word(UID_F10X)? == 0xFFFF_FFFF {
        (FAMILY_F1X0, UID_F1X0)
    } else {
        (FAMILY_F10X, UID_F10X)
    })
}

fn read_uid<P: SwdPins>(swd: &mut Swd<P>, addr: u32) -> Result<[u8; 12], SwdError> {
    let mut words = [0u32; 3];
    swd.read_words(addr, &mut words)?;
    let mut uid = [0u8; 12];
    for (i, w) in words.iter().enumerate() {
        uid[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
    }
    Ok(uid)
}

/// The part on the line as a halt sees it, picked per call (`specs/bridge-chip-support.md`,
/// section 4, "Where the row comes from on a halt"): its debug hold, and the advanced timers the
/// disarm clears. Nothing is kept between calls, so a halt after a board swap uses the new board's.
#[derive(Debug, Clone, Copy)]
struct Line {
    /// `None` on an unsupported part.
    hold: Option<part::Hold>,
    /// The row's timers; timer 0's alone on a part in no row.
    timers: &'static [u32],
}

/// Pick [`Line`]: `CPUID`; on a Cortex-M3 the flash size register and the UID probe, the two reads
/// `identify` names a GD32's row by, so a second timer is read only on a row that has one; on a
/// Cortex-M0 the ID register.
fn line<P: SwdPins>(swd: &mut Swd<P>) -> Result<Line, SwdError> {
    let timers = |row: Option<&'static part::Part>| row.map_or(part::TIMER0_ONLY, |p| p.timers);
    Ok(match part::partno(swd.read_word(CPUID)?) {
        part::PARTNO_M3 => {
            let flash_kib = swd.read_word(FLASH_SIZE)? as u16;
            let (family, _) = gd32_family(swd)?;
            Line {
                hold: Some(part::M3_DEBUG.hold()),
                timers: timers(part::lookup(family, flash_kib)),
            }
        }
        part::PARTNO_M0 => {
            let row = part::m0(swd.read_word(part::M0_ID)?);
            Line {
                hold: row.map(|p| p.hold()),
                timers: timers(row),
            }
        }
        _ => Line {
            hold: None,
            timers: part::TIMER0_ONLY,
        },
    })
}

/// Is any of `timers`' `MOE` set?
fn any_armed<P: SwdPins>(swd: &mut Swd<P>, timers: &[u32]) -> Result<bool, SwdError> {
    for &t in timers {
        if swd.read_word(t)? & MOE != 0 {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Is any advanced timer's `MOE` set (the bridge can switch)? Timer 0's, and timer 7's on the
/// GD32F103RC.
pub fn armed<P: SwdPins>(swd: &mut Swd<P>) -> Result<bool, SwdError> {
    let l = line(swd)?;
    any_armed(swd, l.timers)
}

/// Clear `MOE` in `cchp` if it is set; whether it was.
fn clear_moe<P: SwdPins>(swd: &mut Swd<P>, cchp: u32) -> Result<bool, SwdError> {
    let v = swd.read_word(cchp)?;
    if v & MOE == 0 {
        return Ok(false);
    }
    swd.write_word(cchp, v & !MOE)?;
    Ok(true)
}

/// Write each of `timers`' `CCHP` with `MOE` cleared and read them back. One still set is
/// [`TargetError::Armed`].
fn disarm_timers<P: SwdPins>(swd: &mut Swd<P>, timers: &[u32]) -> Result<(), TargetError> {
    for &t in timers {
        clear_moe(swd, t)?;
    }
    if any_armed(swd, timers)? {
        return Err(TargetError::Armed);
    }
    Ok(())
}

/// Clear `MOE` on every advanced timer of the part and read it back: timer 0's, and timer 7's on
/// the GD32F103RC. Still set is [`TargetError::Armed`].
pub fn disarm<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    let l = line(swd)?;
    disarm_timers(swd, l.timers)
}

/// The debug hold of the part on the line, picked per call by [`Line`]'s reads. `None` on an
/// unsupported part.
pub fn hold<P: SwdPins>(swd: &mut Swd<P>) -> Result<Option<part::Hold>, SwdError> {
    Ok(line(swd)?.hold)
}

/// Set the part's debug hold (the free watchdog and the advanced timers frozen while halted):
/// `DBG_CTL0` at `0xE004_2004` on a GD32, `DBGMCU_CR` at `0x4001_3404` on the MM32SPIN05, nothing
/// on an unsupported part ([`hold`]). Set again before every halt: the register resets at
/// power-on, and the bridge cannot tell whether the board has been power-cycled since.
pub fn debug_hold<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    let h = hold(swd)?;
    set_hold(swd, h)
}

fn set_hold<P: SwdPins>(swd: &mut Swd<P>, h: Option<part::Hold>) -> Result<(), SwdError> {
    let Some(h) = h else {
        return Ok(());
    };
    let v = swd.read_word(h.addr)?;
    if v & h.bits != h.bits {
        swd.write_word(h.addr, v | h.bits)?;
    }
    Ok(())
}

fn wait_halted<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    for _ in 0..100 {
        if swd.read_word(DHCSR)? & S_HALT != 0 {
            return Ok(());
        }
    }
    Err(TargetError::Timeout)
}

/// Disarm, set the debug hold, halt (`DHCSR` = `0xA05F0003`), poll `S_HALT`.
pub fn halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    let l = line(swd)?;
    disarm_timers(swd, l.timers)?;
    set_hold(swd, l.hold)?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)?;
    wait_halted(swd)?;
    // A firmware that set `MOE` again between the disarm and the halt is caught here, where
    // nothing can set it again.
    disarm_timers(swd, l.timers)
}

/// Disarm, set the debug hold, and step one instruction with interrupts masked: `C_MASKINTS` is
/// set while the core is still halted, then `DHCSR` = `0xA05F000D` (`C_STEP | C_MASKINTS`). The
/// core halts again after the instruction; [`unmask`] clears the mask before the next resume.
pub fn step<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    let l = line(swd)?;
    disarm_timers(swd, l.timers)?;
    set_hold(swd, l.hold)?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_HALT | C_MASKINTS)?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_MASKINTS | C_STEP)?;
    Ok(())
}

/// Clear `C_MASKINTS` on a halted core, keeping it halted: the bit may change only while
/// `C_HALT` is set.
pub fn unmask<M: MemPort>(m: &mut M) -> Result<(), M::Error> {
    m.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)
}

/// What [`poll_halt`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HaltPoll {
    /// The core runs.
    Running,
    /// The core is halted. `moe_clear_us` is the time from the `DHCSR` read that saw the halt to
    /// the last `CCHP` write that cleared a `MOE`, when one was set.
    Halted {
        /// Microseconds from the halt observed to the last `MOE` cleared; `None` when every one
        /// was clear.
        moe_clear_us: Option<u32>,
    },
}

/// The armed-breakpoint rule (`specs/hoverboard-bridge.md`, "A breakpoint hit on an armed
/// board"): read `DHCSR`, and when the core is halted read timer 0's `CCHP` at once and clear
/// `MOE` if it is set, then pick the part ([`Line`]) and do the same on its other advanced timers
/// (timer 7 on the GD32F103RC). A breakpoint halts the core with the timers frozen and their
/// outputs held; this takes the motors off for the rest of the halt, and the board needs
/// re-arming after the resume.
pub fn poll_halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<HaltPoll, SwdError> {
    if swd.read_word(DHCSR)? & S_HALT == 0 {
        return Ok(HaltPoll::Running);
    }
    let seen = swd.now_us();
    let mut cleared = None;
    if clear_moe(swd, CCHP)? {
        cleared = Some(swd.now_us());
    }
    let l = line(swd)?;
    for &t in &l.timers[1..] {
        if clear_moe(swd, t)? {
            cleared = Some(swd.now_us());
        }
    }
    Ok(HaltPoll::Halted {
        moe_clear_us: cleared.map(|t| t.saturating_sub(seen).min(u32::MAX as u64) as u32),
    })
}

/// Read `DFSR` and clear the bits read, so the next halt's reason is its own ([`dfsr`]).
pub fn take_halt_reason<M: MemPort>(m: &mut M) -> Result<u32, M::Error> {
    let v = m.read_word(DFSR)?;
    if v != 0 {
        m.write_word(DFSR, v)?;
    }
    Ok(v)
}

/// The flash patch unit's code comparator count, from `FP_CTRL`.
pub fn fpb_count<M: MemPort>(m: &mut M) -> Result<u32, M::Error> {
    let c = m.read_word(FP_CTRL)?;
    Ok((c >> 8) & 0x70 | (c >> 4) & 0xF)
}

/// Enable the flash patch unit.
pub fn fpb_enable<M: MemPort>(m: &mut M) -> Result<(), M::Error> {
    m.write_word(FP_CTRL, FP_KEY_ENABLE)
}

/// Set code comparator `n` to break on the halfword at `addr` (below [`FP_CODE_END`]), or clear
/// it (`None`). The comparator matches a word; `REPLACE` picks the halfword.
pub fn fpb_set<M: MemPort>(m: &mut M, n: u32, addr: Option<u32>) -> Result<(), M::Error> {
    let v = match addr {
        None => 0,
        Some(a) => {
            let replace = if a & 2 != 0 { 0b10 } else { 0b01 };
            replace << 30 | (a & 0x1FFF_FFFC) | 1
        }
    };
    m.write_word(FP_COMP0 + 4 * n, v)
}

/// The DWT's comparator count, from `DWT_CTRL`.
pub fn dwt_count<M: MemPort>(m: &mut M) -> Result<u32, M::Error> {
    Ok(m.read_word(DWT_CTRL)? >> 28)
}

/// What a DWT comparator watches: the `DWT_FUNCTION` data-address encodings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Watch {
    /// Reads.
    Read = 0b0101,
    /// Writes.
    Write = 0b0110,
    /// Both.
    Access = 0b0111,
}

/// Set DWT comparator `n` to halt on `kind` accesses to the `1 << mask_bits` bytes at `addr`
/// (aligned to that size), or clear it (`None`). Setting one powers the DWT (`DEMCR.TRCENA`).
pub fn dwt_set<M: MemPort>(
    m: &mut M,
    n: u32,
    watch: Option<(u32, u32, Watch)>,
) -> Result<(), M::Error> {
    let base = DWT_COMP0 + 16 * n;
    m.write_word(base + 8, 0)?;
    if let Some((addr, mask_bits, kind)) = watch {
        let demcr = m.read_word(DEMCR)?;
        if demcr & TRCENA == 0 {
            m.write_word(DEMCR, demcr | TRCENA)?;
        }
        m.write_word(base, addr)?;
        m.write_word(base + 4, mask_bits)?;
        m.write_word(base + 8, kind as u32)?;
    }
    Ok(())
}

/// Did DWT comparator `n` fire since it was last asked? Reading `DWT_FUNCTION` clears `MATCHED`.
pub fn dwt_matched<M: MemPort>(m: &mut M, n: u32) -> Result<bool, M::Error> {
    Ok(m.read_word(DWT_COMP0 + 16 * n + 8)? & DWT_MATCHED != 0)
}

/// Resume a halted core (`DHCSR` = `0xA05F0001`).
pub fn resume<M: MemPort>(m: &mut M) -> Result<(), M::Error> {
    m.write_word(DHCSR, DBGKEY | C_DEBUGEN)
}

/// Is the core halted (`S_HALT`)?
pub fn halted<M: MemPort>(m: &mut M) -> Result<bool, M::Error> {
    Ok(m.read_word(DHCSR)? & S_HALT != 0)
}

fn reg_ready<M: MemPort>(m: &mut M) -> Result<(), PortError<M::Error>> {
    for _ in 0..REG_POLLS {
        if m.read_word(DHCSR)? & S_REGRDY != 0 {
            return Ok(());
        }
    }
    Err(PortError::Timeout)
}

/// Write a core register of a halted core through `DCRDR` and `DCRSR` ([`reg`]).
pub fn write_core_reg<M: MemPort>(m: &mut M, sel: u32, v: u32) -> Result<(), PortError<M::Error>> {
    m.write_word(DCRDR, v)?;
    m.write_word(DCRSR, REGWNR | sel)?;
    reg_ready(m)
}

/// Read a core register of a halted core through `DCRSR` and `DCRDR` ([`reg`]).
pub fn read_core_reg<M: MemPort>(m: &mut M, sel: u32) -> Result<u32, PortError<M::Error>> {
    m.write_word(DCRSR, sel)?;
    reg_ready(m)?;
    Ok(m.read_word(DCRDR)?)
}

/// `SYSRESETREQ`: it works on every board, NRST wired or not.
pub fn reset<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    swd.write_word(AIRCR, SYSRESETREQ)
}

/// Disarm, set the debug hold, then reset with `VC_CORERESET` set so the core halts at the reset
/// vector, poll `S_HALT`, clear the vector catch. A core already halted (the RAM loader at its
/// `bkpt`) stays halted until the reset rather than running on.
pub fn reset_halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    let l = line(swd)?;
    disarm_timers(swd, l.timers)?;
    set_hold(swd, l.hold)?;
    let hold = if halted(swd)? { C_HALT } else { 0 };
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | hold)?;
    let demcr = swd.read_word(DEMCR)?;
    swd.write_word(DEMCR, demcr | VC_CORERESET)?;
    reset(swd)?;
    let halted = wait_halted(swd);
    swd.write_word(DEMCR, demcr & !VC_CORERESET)?;
    halted?;
    disarm_timers(swd, l.timers)
}

/// Clear `C_DEBUGEN` and leave the core running.
pub fn release<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    swd.write_word(DHCSR, DBGKEY)
}

/// How long NRST is held before the debug port is touched, in microseconds: GD32 NRST needs far
/// less, and this also covers a board-side capacitor on the line.
pub const NRST_PULSE_US: u32 = 10_000;
/// How long the core has after NRST is released to report halted at the reset vector, in
/// microseconds.
pub const NRST_HALT_US: u64 = 100_000;

/// Connect under reset (`specs/bridge-nrst.md` section 2), for a board whose firmware blocks SWD:
/// pull NRST low for [`NRST_PULSE_US`], connect with it held, enable halting debug and set
/// `VC_CORERESET`, release NRST, poll `S_HALT` for up to [`NRST_HALT_US`], clear the vector catch,
/// set the debug hold, and identify the target. The core is left halted at its reset vector, before
/// any of the board's firmware has run.
///
/// Nothing reads the board's layout or disarms its timers first, as [`reset_halt`] does: a board
/// whose firmware blocks SWD gives nothing to read, and a board held in reset drives no gate.
///
/// NRST is released on every path. A debug port that does not answer under reset is
/// [`TargetError::Swd`]; a core that does not report halted in time is [`TargetError::Timeout`].
pub fn connect_under_reset<P: SwdPins>(swd: &mut Swd<P>) -> Result<TargetId, TargetError> {
    swd.forget();
    swd.assert_reset(true);
    swd.delay_us(NRST_PULSE_US);
    let caught = catch_reset_vector(swd);
    swd.assert_reset(false);
    let demcr = caught?;
    let halted = wait_halted_us(swd, NRST_HALT_US);
    swd.write_word(DEMCR, demcr & !VC_CORERESET)?;
    halted?;
    debug_hold(swd)?;
    Ok(identify(swd)?)
}

/// With NRST held: connect, enable halting debug with a halt request, and set `VC_CORERESET`.
/// Returns `DEMCR` as it was.
fn catch_reset_vector<P: SwdPins>(swd: &mut Swd<P>) -> Result<u32, SwdError> {
    swd.connect()?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)?;
    let demcr = swd.read_word(DEMCR)?;
    swd.write_word(DEMCR, demcr | VC_CORERESET)?;
    Ok(demcr)
}

/// Poll `S_HALT` until it is set or `us` has passed.
fn wait_halted_us<P: SwdPins>(swd: &mut Swd<P>, us: u64) -> Result<(), TargetError> {
    let since = swd.now_us();
    loop {
        if halted(swd)? {
            return Ok(());
        }
        if swd.now_us().saturating_sub(since) >= us {
            return Err(TargetError::Timeout);
        }
    }
}
