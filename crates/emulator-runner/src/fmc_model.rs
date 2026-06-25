//! A host model of the GD32 FMC (flash memory controller) that services the EXACT register sequence
//! `runtime-hal::fmc` drives, so the model and the real driver provably agree.
//!
//! Cross-checked against `runtime-hal/src/fmc.rs` (the `target` RAM-resident sequence the Unicorn
//! image actually runs, the `cfg(target_arch = "arm")` path):
//!
//! - **unlock**: read `CTL` (`+0x10`); if `LK` (b7) set, write `KEY` (`+0x04`) = KEY1 (`0x4567_0123`)
//!   then KEY2 (`0xCDEF_89AB`). The model clears `LK` after the KEY1+KEY2 pair.
//! - **erase**: `CTL = PER` (b1), `ADDR` (`+0x14`) = absolute page address, `CTL = PER|START` (b6),
//!   poll `STAT` (`+0x0C`) until `BUSY` (b0) clears, write `STAT` to clear flags, `CTL = 0`. The model
//!   performs the page erase (fill `0xFFFF`) when `START` is written with `PER` set, and reports
//!   `BUSY = 0` on every `STAT` read (the charge pump has completed by the time the driver polls).
//! - **program**: `CTL = PG` (b0), then per halfword a DIRECT 16-bit store to the absolute flash
//!   address (NOT through `ADDR`), each followed by a `STAT` poll; `CTL = 0` at the end. The model
//!   applies write-once on each halfword store: a store to a still-`0xFFFF` cell lands; a re-program
//!   of a non-`0xFFFF` cell sets `PGERR` (b2) and leaves the cell UNCHANGED (it does NOT AND).
//! - **STAT flags**: `BUSY` b0 (reads 0), `PGERR` b2, `WPERR` b4, `ENDF` b5; the error/end flags are
//!   write-1-to-clear (the driver clears `ENDF|WPERR|PGERR` = `0x34` after every op).
//!
//! This pure-logic struct holds the FMC register state and the flash-region geometry; the Unicorn
//! glue in `lib.rs` wires it to the mapped FMC MMIO and the flash write-protect fault. The
//! halfword-apply and erase decisions are unit-tested here directly.

/// FMC peripheral base (both families).
pub const FMC_BASE: u64 = 0x4002_2000;
/// FMC MMIO window to map (4 KiB, the `mmio_map` alignment unit).
pub const FMC_WINDOW: u64 = 0x1000;

/// Register offsets from [`FMC_BASE`] (bank0; identical to `runtime-hal/src/fmc.rs`).
pub const KEY: u64 = 0x04;
pub const STAT: u64 = 0x0C;
pub const CTL: u64 = 0x10;
pub const ADDR: u64 = 0x14;

/// Unlock keys.
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

// CTL bits.
const CTL_PG: u32 = 1 << 0;
const CTL_PER: u32 = 1 << 1;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;

// STAT bits.
const STAT_BUSY: u32 = 1 << 0;
const STAT_PGERR: u32 = 1 << 2;
const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;
/// The write-1-to-clear flags the driver clears after every op (`0x34`).
const STAT_CLEAR: u32 = STAT_ENDF | STAT_WPERR | STAT_PGERR;

/// The erased halfword value.
const ERASED: u16 = 0xFFFF;

/// What a write to a flash region halfword resolves to, given the current FMC state. The Unicorn glue
/// applies this against the mapped flash memory.
#[derive(Debug, PartialEq, Eq)]
pub enum ProgramOutcome {
    /// The cell was erased: write the halfword (write-once OK).
    Write,
    /// `PG` was not set (a stray write outside a program sequence): reject as a genuine fault.
    NotProgramming,
    /// Re-program of a non-erased cell: leave the cell unchanged, `PGERR` is now set.
    Rejected,
}

/// What a `CTL` write triggers.
#[derive(Debug, PartialEq, Eq)]
pub enum CtlEffect {
    /// Nothing beyond storing the value (e.g. setting `PG`, or `CTL = 0`).
    None,
    /// `PER|START`: erase the page at the current `ADDR` (fill `0xFFFF`).
    ErasePage(u32),
}

/// The FMC register model: the unlock state, the latched `ADDR`, the `STAT` flags, and the
/// erase/program geometry. Pure logic, no Unicorn handle.
pub struct FmcModel {
    /// Page size in bytes (1024 or 2048), so an erase fills exactly one page.
    page_size: u32,
    /// `LK` is set after reset; cleared by the KEY1+KEY2 unlock pair.
    locked: bool,
    /// Tracks the first KEY write so the second completes the unlock pair.
    key1_seen: bool,
    /// The CTL bits the driver last wrote (PG / PER drive the program / erase behaviour).
    ctl: u32,
    /// The latched erase address (`ADDR`).
    addr: u32,
    /// Sticky `STAT` error/end flags (write-1-to-clear).
    stat_flags: u32,
}

impl FmcModel {
    /// A reset-state model for a part with the given `page_size` (bytes).
    pub fn new(page_size: u32) -> Self {
        Self {
            page_size,
            locked: true,
            key1_seen: false,
            ctl: 0,
            addr: 0,
            stat_flags: 0,
        }
    }

    /// True iff a program is in progress (`PG` set), so a flash write is a halfword program.
    pub fn programming(&self) -> bool {
        self.ctl & CTL_PG != 0
    }

    /// Service a read of an FMC register at `off` (from [`FMC_BASE`]).
    pub fn read_reg(&self, off: u64) -> u32 {
        match off {
            CTL => {
                // Report LK so the driver's unlock decides whether to write the keys; the other CTL
                // bits read back as last written.
                let lk = if self.locked { CTL_LK } else { 0 };
                (self.ctl & !CTL_LK) | lk
            }
            // STAT: BUSY always reads 0 (the op has completed by the time the driver polls), plus the
            // sticky error/end flags.
            STAT => self.stat_flags & !STAT_BUSY,
            // ADDR / KEY read back their last value / 0; not read by the driver, but harmless.
            ADDR => self.addr,
            _ => 0,
        }
    }

    /// Service a write of `val` to the FMC register at `off`. Returns the side effect the Unicorn glue
    /// must carry out against flash memory (a page erase), or [`CtlEffect::None`].
    pub fn write_reg(&mut self, off: u64, val: u32) -> CtlEffect {
        match off {
            KEY => {
                if self.locked {
                    if !self.key1_seen && val == KEY1 {
                        self.key1_seen = true;
                    } else if self.key1_seen && val == KEY2 {
                        self.locked = false;
                        self.key1_seen = false;
                    } else {
                        self.key1_seen = false;
                    }
                }
                CtlEffect::None
            }
            STAT => {
                // Write-1-to-clear the sticky flags the driver clears (ENDF|WPERR|PGERR).
                self.stat_flags &= !(val & STAT_CLEAR);
                CtlEffect::None
            }
            ADDR => {
                self.addr = val;
                CtlEffect::None
            }
            CTL => {
                self.ctl = val;
                if val & CTL_PER != 0 && val & CTL_START != 0 {
                    // Page erase kicked: the page containing ADDR (ADDR is page-aligned by the driver).
                    let page_base = self.addr & !(self.page_size - 1);
                    // ENDF on completion (cosmetic; the driver only decodes PGERR/WPERR/Timeout).
                    self.stat_flags |= STAT_ENDF;
                    return CtlEffect::ErasePage(page_base);
                }
                CtlEffect::None
            }
            _ => CtlEffect::None,
        }
    }

    /// Decide a halfword flash store at `cur` (the current cell value): write-once. Sets `PGERR` on a
    /// re-program. The Unicorn glue performs (or skips) the actual memory write per the outcome.
    pub fn apply_program_halfword(&mut self, cur: u16) -> ProgramOutcome {
        if !self.programming() {
            return ProgramOutcome::NotProgramming;
        }
        if cur == ERASED {
            self.stat_flags |= STAT_ENDF;
            ProgramOutcome::Write
        } else {
            // Write-once: a re-program is refused with PGERR, content unchanged (NOT ANDed).
            self.stat_flags |= STAT_PGERR;
            ProgramOutcome::Rejected
        }
    }

    /// The page size in bytes.
    pub fn page_size(&self) -> u32 {
        self.page_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unlock_clears_lk_only_on_key1_then_key2() {
        let mut m = FmcModel::new(1024);
        assert_eq!(m.read_reg(CTL) & CTL_LK, CTL_LK); // locked after reset
        m.write_reg(KEY, KEY1);
        assert_eq!(m.read_reg(CTL) & CTL_LK, CTL_LK); // still locked after KEY1 alone
        m.write_reg(KEY, KEY2);
        assert_eq!(m.read_reg(CTL) & CTL_LK, 0); // unlocked after the pair
    }

    #[test]
    fn stat_busy_reads_zero() {
        let m = FmcModel::new(1024);
        assert_eq!(m.read_reg(STAT) & STAT_BUSY, 0);
    }

    #[test]
    fn ctl_per_start_triggers_page_erase_at_addr_page() {
        let mut m = FmcModel::new(1024);
        m.write_reg(ADDR, 0x0800_F800 + 0x40); // an address inside the page
        m.write_reg(CTL, CTL_PER);
        let eff = m.write_reg(CTL, CTL_PER | CTL_START);
        assert_eq!(eff, CtlEffect::ErasePage(0x0800_F800)); // aligned down to the page base
    }

    #[test]
    fn program_halfword_is_write_once() {
        let mut m = FmcModel::new(1024);
        // Not programming yet: a stray write is a genuine fault.
        assert_eq!(
            m.apply_program_halfword(0xFFFF),
            ProgramOutcome::NotProgramming
        );
        m.write_reg(CTL, CTL_PG);
        // Erased cell: the write lands.
        assert_eq!(m.apply_program_halfword(0xFFFF), ProgramOutcome::Write);
        assert_eq!(m.read_reg(STAT) & STAT_PGERR, 0);
        // Non-erased cell: rejected, PGERR set, content unchanged (the glue skips the write).
        assert_eq!(m.apply_program_halfword(0x1234), ProgramOutcome::Rejected);
        assert_eq!(m.read_reg(STAT) & STAT_PGERR, STAT_PGERR);
        // The driver clears PGERR (write-1-to-clear) after the op.
        m.write_reg(STAT, STAT_PGERR);
        assert_eq!(m.read_reg(STAT) & STAT_PGERR, 0);
    }
}
