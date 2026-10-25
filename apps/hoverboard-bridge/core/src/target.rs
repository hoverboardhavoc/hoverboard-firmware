//! Board operations (`specs/hoverboard-bridge.md`, "Board operations"): each operation OpenOCD performed for
//! the tools, as a short register sequence on the SWD engine. The ESP commands and the flash module
//! call these rather than writing the registers themselves.
//!
//! **Every operation that halts goes through [`disarm`] first**, here, in the one module that
//! halts: a halt freezes timer 0 with its outputs at their last level, so a phase can sit at 100%
//! duty (`tools/swd-disarm-halt.sh` carries the standing rule and the incident).

use espsvc::{TargetId, FAMILY_F10X, FAMILY_F1X0};

use crate::swd::{MemPort, Swd, SwdError, SwdPins};

/// The debug MCU ID.
pub const DBGMCU_ID: u32 = 0xE004_2000;
/// `DBG_CTL0`.
pub const DBG_CTL0: u32 = 0xE004_2004;
/// `DBG_CTL0` `FWDGT_HOLD` (bit 8) and `TIMER0_HOLD` (bit 10), the same bits on F1x0 and F10x.
pub const DEBUG_HOLD: u32 = 1 << 8 | 1 << 10;
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
/// Timer 0 `CCHP`; `MOE` is bit 15.
pub const CCHP: u32 = 0x4001_2C44;
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
/// The flash size register (KiB in the low half), the same address on both families.
pub const FLASH_SIZE: u32 = 0x1FFF_F7E0;
/// The F10x UID. On an F1x0 this word reads all ones.
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

/// Connect if the engine is not connected.
pub fn ensure_connected<P: SwdPins>(swd: &mut Swd<P>) -> Result<u32, SwdError> {
    match swd.dpidr() {
        Some(id) => Ok(id),
        None => swd.connect(),
    }
}

/// Identify the target: `DPIDR`, the debug MCU ID, `CPUID`, the option bytes, the flash size, the
/// armed state, and the UID by family. The debug MCU ID does not tell the families apart (it reads
/// `0x13030410` on both); the UID's location does: `0x1FFF_F7E8` reads all ones on an F1x0.
/// `clear_check` is the caller's.
///
/// It connects afresh every time (line reset, `DPIDR`, power-up), so each `TARGET_ID` is one
/// connect: bench gate 1 counts them. A line reset leaves the debug port's registers as they
/// were, so a mailbox session running beside it carries on.
pub fn identify<P: SwdPins>(swd: &mut Swd<P>) -> Result<TargetId, SwdError> {
    let dpidr = swd.connect()?;
    let mcu_id = swd.read_word(DBGMCU_ID)?;
    let cpuid = swd.read_word(CPUID)?;
    let ob = swd.read_word(OPTION_BYTES)?;
    let flash_kib = swd.read_word(FLASH_SIZE)? as u16;
    let armed = swd.read_word(CCHP)? & MOE != 0;
    let probe = swd.read_word(UID_F10X)?;
    let (family, uid_addr) = if probe == 0xFFFF_FFFF {
        (FAMILY_F1X0, UID_F1X0)
    } else {
        (FAMILY_F10X, UID_F10X)
    };
    let mut words = [0u32; 3];
    swd.read_words(uid_addr, &mut words)?;
    let mut uid = [0u8; 12];
    for (i, w) in words.iter().enumerate() {
        uid[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
    }
    Ok(TargetId {
        dpidr,
        mcu_id,
        cpuid,
        family,
        uid,
        rdp: ob as u8,
        user: (ob >> 16) as u8,
        clear_check: espsvc::CLEAR_NONE,
        flash_kib,
        armed,
    })
}

/// Is timer 0's `MOE` set (the bridge can switch)?
pub fn armed<P: SwdPins>(swd: &mut Swd<P>) -> Result<bool, SwdError> {
    Ok(swd.read_word(CCHP)? & MOE != 0)
}

/// Write `CCHP` with `MOE` cleared and read it back. Still set is [`TargetError::Armed`].
pub fn disarm<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    let v = swd.read_word(CCHP)?;
    if v & MOE != 0 {
        swd.write_word(CCHP, v & !MOE)?;
    }
    if swd.read_word(CCHP)? & MOE != 0 {
        return Err(TargetError::Armed);
    }
    Ok(())
}

/// Set `FWDGT_HOLD` and `TIMER0_HOLD` in `DBG_CTL0`. Set again before every halt: the register
/// resets at power-on, and the bridge cannot tell whether the board has been power-cycled since.
pub fn debug_hold<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    let v = swd.read_word(DBG_CTL0)?;
    if v & DEBUG_HOLD != DEBUG_HOLD {
        swd.write_word(DBG_CTL0, v | DEBUG_HOLD)?;
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
    disarm(swd)?;
    debug_hold(swd)?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | C_HALT)?;
    wait_halted(swd)?;
    // A firmware that set `MOE` again between the disarm and the halt is caught here, where
    // nothing can set it again.
    disarm(swd)
}

/// Disarm, set the debug hold, and step one instruction with interrupts masked: `C_MASKINTS` is
/// set while the core is still halted, then `DHCSR` = `0xA05F000D` (`C_STEP | C_MASKINTS`). The
/// core halts again after the instruction; [`unmask`] clears the mask before the next resume.
pub fn step<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    disarm(swd)?;
    debug_hold(swd)?;
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
    /// the `CCHP` write that cleared `MOE`, when `MOE` was set.
    Halted {
        /// Microseconds from the halt observed to `MOE` cleared; `None` when it was clear.
        moe_clear_us: Option<u32>,
    },
}

/// The armed-breakpoint rule (`specs/hoverboard-bridge.md`, "A breakpoint hit on an armed
/// board"): read `DHCSR`, and when the core is halted read `CCHP` at once and clear `MOE` if it is
/// set. A breakpoint halts the core with timer 0 frozen and its outputs held; this takes the motor
/// off for the rest of the halt, and the board needs re-arming after the resume.
pub fn poll_halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<HaltPoll, SwdError> {
    if swd.read_word(DHCSR)? & S_HALT == 0 {
        return Ok(HaltPoll::Running);
    }
    let seen = swd.now_us();
    let v = swd.read_word(CCHP)?;
    if v & MOE == 0 {
        return Ok(HaltPoll::Halted { moe_clear_us: None });
    }
    swd.write_word(CCHP, v & !MOE)?;
    let us = swd.now_us().saturating_sub(seen).min(u32::MAX as u64) as u32;
    Ok(HaltPoll::Halted {
        moe_clear_us: Some(us),
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

/// `SYSRESETREQ` (the fleet's boards do not wire NRST).
pub fn reset<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    swd.write_word(AIRCR, SYSRESETREQ)
}

/// Disarm, set the debug hold, then reset with `VC_CORERESET` set so the core halts at the reset
/// vector, poll `S_HALT`, clear the vector catch. A core already halted (the RAM loader at its
/// `bkpt`) stays halted until the reset rather than running on.
pub fn reset_halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    disarm(swd)?;
    debug_hold(swd)?;
    let hold = if halted(swd)? { C_HALT } else { 0 };
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN | hold)?;
    let demcr = swd.read_word(DEMCR)?;
    swd.write_word(DEMCR, demcr | VC_CORERESET)?;
    reset(swd)?;
    let halted = wait_halted(swd);
    swd.write_word(DEMCR, demcr & !VC_CORERESET)?;
    halted?;
    disarm(swd)
}

/// Clear `C_DEBUGEN` and leave the core running.
pub fn release<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), SwdError> {
    swd.write_word(DHCSR, DBGKEY)
}
