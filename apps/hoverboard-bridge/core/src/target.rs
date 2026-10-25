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
const S_HALT: u32 = 1 << 17;
const SYSRESETREQ: u32 = 0x05FA_0004;
const VC_CORERESET: u32 = 1;
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
    /// The debug return address: where the core resumes.
    pub const PC: u32 = 15;
    /// `xPSR`.
    pub const XPSR: u32 = 16;
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
/// vector, poll `S_HALT`, clear the vector catch.
pub fn reset_halt<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), TargetError> {
    disarm(swd)?;
    debug_hold(swd)?;
    swd.write_word(DHCSR, DBGKEY | C_DEBUGEN)?;
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
