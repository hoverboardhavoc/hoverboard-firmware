// SPDX-License-Identifier: GPL-3.0-or-later
//! The flash controller: page erase and half-word program at absolute flash addresses.
//!
//! The mechanism only. Where the store or an image lives in flash is its caller's
//! (`specs/storage-layer.md`, "Ownership and the HAL boundary"): [`Fmc`] takes absolute addresses,
//! checks them against the compiled chip's flash extent and page size, and drives the controller.
//!
//! One register model on every chip this HAL builds (bank 0 of the GD32F10x and GD32F1x0, the
//! same offsets, bits and keys; every part here is single-bank):
//!
//! | Register | Offset | Used here |
//! |---|---|---|
//! | `WS` | `0x00` | wait states, owned by [`crate::clock::configure_tree`] |
//! | `KEY` | `0x04` | unlock: `0x4567_0123` then `0xCDEF_89AB` |
//! | `STAT` | `0x0C` | `BUSY` 0, `PGERR` 2, `WPERR` 4, `ENDF` 5 (the last three write-1-to-clear) |
//! | `CTL` | `0x10` | `PG` 0, `PER` 1, `START` 6, `LK` 7 |
//! | `ADDR` | `0x14` | the page to erase |
//!
//! # Program rules the controller enforces
//!
//! A half-word is the program unit and is write-once per erase: programming one that is not
//! `0xFFFF` is refused with `PGERR` and the cell is left as it was. And programming the low
//! half-word of a 32-bit word closes the high half-word of the word below it, in the same page,
//! until the page is erased: a later program there is refused with `PGERR` while the cell still
//! reads `0xFFFF` (`runtime-hal`, measured on a GD32F130C8 on 2026-10-23; it does not cross a page
//! boundary). So a page is programmed in ascending address order. [`Fmc::program`] checks both
//! before it touches the controller, refusing with [`FmcError::NotErased`] or [`FmcError::Closed`],
//! so a call either programs its whole span or none of it; `PGERR` from the controller is the
//! backstop.
//!
//! # Running from RAM
//!
//! An erase or program stalls instruction fetch from flash for its duration (a page erase is tens
//! of milliseconds). The sequence from the unlock to the last busy poll is one function placed in
//! `.data`, which cortex-m-rt copies to RAM at startup, and it runs with interrupts masked
//! ([`cortex_m::interrupt::free`]), one page or one span per masked section. Its body reaches the
//! registers through the always-inlined [`Reg32`]/[`Reg16`] and calls nothing in flash. The
//! controller is locked again (`CTL = LK`) at the end of every operation.

use crate::chip::Chip;
use crate::descriptor::DESCRIPTOR;
use crate::error::FmcError;
use crate::reg::{Reg16, Reg32};

/// The controller's base, the same on every chip here.
pub(crate) const FMC_BASE: u32 = 0x4002_2000;
/// The wait-state register.
pub(crate) const WS: u32 = 0x00;
const KEY: u32 = 0x04;
const STAT: u32 = 0x0C;
const CTL: u32 = 0x10;
const ADDR: u32 = 0x14;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

const CTL_PG: u32 = 1 << 0;
const CTL_PER: u32 = 1 << 1;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;

const STAT_BUSY: u32 = 1 << 0;
const STAT_PGERR: u32 = 1 << 2;
const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;
/// The sticky flags, cleared after every operation: a `PGERR` left set fails the next one.
const STAT_CLEAR: u32 = STAT_ENDF | STAT_WPERR | STAT_PGERR;

/// Main flash, on every chip here.
const FLASH_BASE: u32 = 0x0800_0000;
const ERASED: u16 = 0xFFFF;

/// Polls one busy wait may spend. An erase or program is timed by the flash itself (datasheet
/// worst cases: page erase 400 ms, half-word program 105 us), so the poll count per millisecond
/// grows with the core clock; sized at the fastest part's 108 MHz and about two cycles a poll this
/// is 1.8 s, 4.5 times the erase worst case. At a slower clock it is a longer bound, still finite.
const BUSY_SPINS: u32 = 100_000_000;

/// The flash controller of the compiled chip.
#[derive(Debug)]
pub struct Fmc {
    _private: (),
}

impl Fmc {
    /// The controller. The base is fixed and the geometry is the descriptor's, so there is nothing
    /// to refuse.
    pub fn new(_chip: &Chip) -> Fmc {
        Fmc { _private: () }
    }

    /// The erase page size in bytes.
    #[inline]
    pub const fn page_size(&self) -> u32 {
        DESCRIPTOR.flash_page.bytes()
    }

    /// The flash size in bytes.
    #[inline]
    pub const fn flash_size_bytes(&self) -> u32 {
        DESCRIPTOR.flash_size_bytes()
    }

    /// Erase the page at absolute address `addr`, which must be a page start inside flash
    /// (`BadArg` otherwise, with the controller untouched). `WriteProtect` for a protected page,
    /// `Timeout` if the controller stays busy past its budget.
    pub fn erase_page(&mut self, addr: u32) -> Result<(), FmcError> {
        check_erase(addr)?;
        let status = cortex_m::interrupt::free(|_| erase_ram(addr));
        decode(status)
    }

    /// Program `bytes` at absolute address `addr`, half-word by half-word in ascending order. The
    /// address and length must be even and the span inside flash (`BadArg`); every target
    /// half-word must read erased (`NotErased`) and the last must not be closed by its successor
    /// (`Closed`). Those three are checked before the controller is touched. `WriteProtect`,
    /// `ProgramError` (a `PGERR` the checks did not foresee) or `Timeout` come from the controller.
    /// An empty `bytes` does nothing.
    pub fn program(&mut self, addr: u32, bytes: &[u8]) -> Result<(), FmcError> {
        if bytes.is_empty() {
            return Ok(());
        }
        let len = u32::try_from(bytes.len()).map_err(|_| FmcError::BadArg)?;
        check_program(addr, len)?;
        precheck(addr, len, |a| Reg16::new(a, 0).read())?;
        let status = cortex_m::interrupt::free(|_| program_ram(addr, bytes));
        decode(status)
    }
}

/// Whether `[addr, addr + len)` lies inside main flash.
const fn in_flash(addr: u32, len: u32) -> bool {
    if addr < FLASH_BASE {
        return false;
    }
    match addr.checked_add(len) {
        Some(end) => end <= FLASH_BASE + DESCRIPTOR.flash_size_bytes(),
        None => false,
    }
}

/// An erase address: a page start inside flash.
const fn check_erase(addr: u32) -> Result<(), FmcError> {
    let page = DESCRIPTOR.flash_page.bytes();
    if in_flash(addr, page) && (addr - FLASH_BASE).is_multiple_of(page) {
        Ok(())
    } else {
        Err(FmcError::BadArg)
    }
}

/// A program span: a non-empty even length at an even address, inside flash.
const fn check_program(addr: u32, len: u32) -> Result<(), FmcError> {
    if len != 0 && addr.is_multiple_of(2) && len.is_multiple_of(2) && in_flash(addr, len) {
        Ok(())
    } else {
        Err(FmcError::BadArg)
    }
}

/// The checks against flash content, through `read` (the half-word at an absolute address): every
/// target half-word erased, and the last one not closed. Only the last can be closed: the
/// successor of any other target is itself a target, just checked erased. And only a high
/// half-word (address 2 mod 4) can be, by a programmed successor in the same page. `addr` and
/// `len` have passed [`check_program`].
fn precheck(addr: u32, len: u32, read: impl Fn(u32) -> u16) -> Result<(), FmcError> {
    let mut off = 0;
    while off < len {
        if read(addr + off) != ERASED {
            return Err(FmcError::NotErased);
        }
        off += 2;
    }
    let last = addr + len - 2;
    let page = DESCRIPTOR.flash_page.bytes();
    let successor_in_page = !(last + 2 - FLASH_BASE).is_multiple_of(page);
    if last % 4 == 2 && successor_in_page && read(last + 2) != ERASED {
        return Err(FmcError::Closed);
    }
    Ok(())
}

/// The status of a finished operation: the sticky `STAT` bits it raised, or a spent busy budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Status {
    stat: u32,
    timed_out: bool,
}

/// `WPERR` (the more specific cause) before `PGERR`; a timeout before both.
const fn decode(s: Status) -> Result<(), FmcError> {
    if s.timed_out {
        Err(FmcError::Timeout)
    } else if s.stat & STAT_WPERR != 0 {
        Err(FmcError::WriteProtect)
    } else if s.stat & STAT_PGERR != 0 {
        Err(FmcError::ProgramError)
    } else {
        Ok(())
    }
}

/// Unlock the controller if `LK` is set (the key pair is written only then).
#[inline(always)]
fn unlock() {
    if Reg32::new(FMC_BASE, CTL).read() & CTL_LK != 0 {
        Reg32::new(FMC_BASE, KEY).write(KEY1);
        Reg32::new(FMC_BASE, KEY).write(KEY2);
    }
}

/// Wait for `BUSY` to clear within [`BUSY_SPINS`], then clear the sticky flags. Returns the flags
/// the operation raised.
#[inline(always)]
fn wait_idle() -> Status {
    let stat = Reg32::new(FMC_BASE, STAT);
    let mut budget = BUSY_SPINS;
    loop {
        let s = stat.read();
        if s & STAT_BUSY == 0 {
            stat.write(STAT_CLEAR);
            return Status {
                stat: s,
                timed_out: false,
            };
        }
        budget -= 1;
        if budget == 0 {
            stat.write(STAT_CLEAR);
            return Status {
                stat: s,
                timed_out: true,
            };
        }
    }
}

/// Erase the page at `addr`: unlock, `PER`, `ADDR`, `PER | START`, wait, lock. From RAM.
#[cfg_attr(target_arch = "arm", link_section = ".data")]
#[inline(never)]
fn erase_ram(addr: u32) -> Status {
    let ctl = Reg32::new(FMC_BASE, CTL);
    unlock();
    ctl.write(CTL_PER);
    Reg32::new(FMC_BASE, ADDR).write(addr);
    ctl.write(CTL_PER | CTL_START);
    let s = wait_idle();
    ctl.write(CTL_LK);
    s
}

/// Program `bytes` at `addr`: unlock, `PG`, then per half-word a 16-bit store to its address and a
/// wait, then lock. The flags of every half-word accumulate. From RAM. An odd tail byte, which
/// [`check_program`] excludes, would be left unprogrammed.
#[cfg_attr(target_arch = "arm", link_section = ".data")]
#[inline(never)]
fn program_ram(addr: u32, bytes: &[u8]) -> Status {
    let ctl = Reg32::new(FMC_BASE, CTL);
    unlock();
    ctl.write(CTL_PG);
    let mut acc = Status {
        stat: 0,
        timed_out: false,
    };
    let mut a = addr;
    let (pairs, _) = bytes.as_chunks::<2>();
    for &pair in pairs {
        Reg16::new(a, 0).write(u16::from_le_bytes(pair));
        let s = wait_idle();
        acc.stat |= s.stat;
        acc.timed_out |= s.timed_out;
        a += 2;
    }
    ctl.write(CTL_LK);
    acc
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    const PAGE: u32 = DESCRIPTOR.flash_page.bytes();
    const END: u32 = FLASH_BASE + DESCRIPTOR.flash_size_bytes();

    #[test]
    fn erase_takes_page_starts_inside_flash_only() {
        assert_eq!(check_erase(FLASH_BASE), Ok(()));
        assert_eq!(check_erase(END - PAGE), Ok(()));
        assert_eq!(check_erase(FLASH_BASE + PAGE), Ok(()));
        assert_eq!(check_erase(END), Err(FmcError::BadArg));
        assert_eq!(check_erase(FLASH_BASE + PAGE / 2), Err(FmcError::BadArg));
        assert_eq!(check_erase(FLASH_BASE - PAGE), Err(FmcError::BadArg));
        assert_eq!(check_erase(0), Err(FmcError::BadArg));
        assert_eq!(check_erase(u32::MAX - PAGE + 1), Err(FmcError::BadArg));
    }

    #[test]
    fn program_takes_even_spans_inside_flash_only() {
        assert_eq!(check_program(FLASH_BASE, 2), Ok(()));
        assert_eq!(check_program(END - 2, 2), Ok(()));
        assert_eq!(check_program(END - 4, 4), Ok(()));
        assert_eq!(check_program(END - 2, 4), Err(FmcError::BadArg));
        assert_eq!(check_program(FLASH_BASE + 1, 2), Err(FmcError::BadArg));
        assert_eq!(check_program(FLASH_BASE, 3), Err(FmcError::BadArg));
        assert_eq!(check_program(FLASH_BASE, 0), Err(FmcError::BadArg));
        assert_eq!(check_program(FLASH_BASE - 2, 2), Err(FmcError::BadArg));
        assert_eq!(check_program(u32::MAX - 1, 2), Err(FmcError::BadArg));
    }

    /// Flash content: erased except the programmed half-words given.
    fn flash(programmed: &[(u32, u16)]) -> impl Fn(u32) -> u16 {
        let cells: HashMap<u32, u16> = programmed.iter().copied().collect();
        move |a| cells.get(&a).copied().unwrap_or(ERASED)
    }

    #[test]
    fn a_span_with_a_programmed_half_word_is_not_erased() {
        let base = END - 2 * PAGE;
        let f = flash(&[(base + 6, 0x1234)]);
        assert_eq!(precheck(base, 8, &f), Err(FmcError::NotErased));
        assert_eq!(precheck(base, 6, &f), Ok(()));
        assert_eq!(precheck(base + 8, 8, &f), Ok(()));
    }

    #[test]
    fn a_high_half_word_under_a_programmed_successor_is_closed() {
        let base = END - 2 * PAGE;
        // +4 programmed closes +2 (a high half-word), not +0 (a low one).
        let f = flash(&[(base + 4, 0x5354)]);
        assert_eq!(precheck(base + 2, 2, &f), Err(FmcError::Closed));
        assert_eq!(precheck(base, 4, &f), Err(FmcError::Closed));
        assert_eq!(precheck(base, 2, &f), Ok(()));
        // A low half-word is never closed, whatever sits above it.
        let f = flash(&[(base + 2, 0x0001)]);
        assert_eq!(precheck(base, 2, &f), Ok(()));
    }

    #[test]
    fn closing_stops_at_the_page_boundary() {
        let base = END - 2 * PAGE;
        // The page's last half-word, with the next page's first programmed.
        let f = flash(&[(base + PAGE, 0xAAAA)]);
        assert_eq!(precheck(base + PAGE - 2, 2, &f), Ok(()));
    }

    #[test]
    fn errors_decode_in_order() {
        let s = |stat, timed_out| Status { stat, timed_out };
        assert_eq!(decode(s(0, false)), Ok(()));
        assert_eq!(decode(s(STAT_ENDF, false)), Ok(()));
        assert_eq!(decode(s(STAT_PGERR, false)), Err(FmcError::ProgramError));
        assert_eq!(
            decode(s(STAT_PGERR | STAT_WPERR, false)),
            Err(FmcError::WriteProtect)
        );
        assert_eq!(decode(s(STAT_WPERR, true)), Err(FmcError::Timeout));
    }

    #[test]
    fn the_geometry_is_the_descriptors() {
        // SAFETY: page_size and flash_size_bytes touch no register.
        let fmc = Fmc::new(&unsafe { Chip::steal() });
        assert_eq!(fmc.page_size(), DESCRIPTOR.flash_page.bytes());
        assert_eq!(
            fmc.flash_size_bytes(),
            u32::from(DESCRIPTOR.flash_kib) * 1024
        );
    }
}
