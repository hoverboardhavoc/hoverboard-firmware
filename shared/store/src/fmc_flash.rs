//! The on-target [`Flash`] adapter over the HAL's FMC primitive (the store's only `runtime-hal`
//! dependency, target-gated to thumbv7m).
//!
//! `FmcFlash` is the **policy** half of the mechanism/policy split (see the storage spec, "Ownership
//! and the HAL boundary"): `runtime-hal::fmc::Fmc` owns the unlock/erase/program register dance at
//! ABSOLUTE flash addresses and knows nothing about placement; this adapter owns the store region
//! (base + length) and maps the store's region-relative offset / page index to an absolute address.
//! It is NOT a pass-through of `Fmc`, it adds exactly the things the HAL deliberately omits:
//!
//! - **owns the region**: `store_base = (FLASH_BASE + flash_size_bytes()) - 2 * page_size()`, the
//!   top two detected pages, both numbers derived from the detected [`Chip`];
//! - **maps** region-relative `off` / `page` to an absolute address and **bounds-checks** every
//!   access against the region length (an own `OutOfBounds`, not trusting the HAL with an
//!   out-of-region absolute address);
//! - **provides `as_bytes`**: the whole region as a memory-mapped slice (flash is read-mapped at
//!   `0x0800_0000`, so a read never touches the FMC and a variable value is a borrowed sub-slice);
//! - **maps errors**: `FmcError` -> `base::error::FlashError` per the spec's PINNED mapping.
//!
//! Only `page_size()` forwards straight to the HAL.
//!
//! This module is `cfg(target_arch = "arm")`-gated, so the host `cargo test` build never compiles it
//! (it uses [`crate::flash::MockFlash`]) and never links the HAL.

use base::error::FlashError;
use runtime_hal::chip::Chip;
use runtime_hal::error::FmcError;
use runtime_hal::fmc::Fmc;

use crate::flash::Flash;

/// Main flash base on both families (the absolute-address space the store region lives in).
use crate::geometry;

/// Whether a [`FmcError`] arose from a program or an erase, so the `NotErased`/`ProgramError`/
/// `Timeout` group maps to `ProgramFailed` (from a program) or `EraseFailed` (from an erase) per the
/// spec's pinned mapping.
#[derive(Clone, Copy)]
enum Op {
    Program,
    Erase,
}

/// Map a HAL [`FmcError`] to the store's [`FlashError`] per the spec's PINNED mapping: `BadArg` ->
/// `Misaligned`; `WriteProtect` -> `Locked`; `NotErased` / `ProgramError` / `Timeout` ->
/// `ProgramFailed` from a program, `EraseFailed` from an erase. (The adapter's own region bounds
/// check yields `OutOfBounds` directly, never reaching the HAL.)
fn map_err(e: FmcError, op: Op) -> FlashError {
    match e {
        FmcError::BadArg => FlashError::Misaligned,
        FmcError::WriteProtect => FlashError::Locked,
        // `NotErased` / `ProgramError` / `Timeout` (and any future `#[non_exhaustive]` variant) are
        // an op failure: ProgramFailed from a program, EraseFailed from an erase.
        FmcError::NotErased | FmcError::ProgramError | FmcError::Timeout | _ => match op {
            Op::Program => FlashError::ProgramFailed,
            Op::Erase => FlashError::EraseFailed,
        },
    }
}

/// The on-target [`Flash`] implementation: the HAL's FMC primitive plus the store region geometry.
///
/// Takes the detected [`Chip`] at construction (the chip is detected once at boot and threaded to the
/// store at mount), since the HAL's `Fmc` needs it and the region base/length derive from it.
pub struct FmcFlash {
    fmc: Fmc,
    /// Absolute base of the store region (`(FLASH_BASE + flash_size) - 2 * page_size`).
    store_base: u32,
    /// Region length in bytes (`2 * page_size`).
    region_len: usize,
    /// One detected page in bytes.
    page_size: usize,
}

impl FmcFlash {
    /// Build the adapter for the detected `chip`: the FMC primitive plus the top-two-pages store
    /// region derived from the chip's flash size + page size.
    pub fn new(chip: &Chip) -> Self {
        let fmc = Fmc::new(chip);
        let page_size = fmc.page_size() as usize;
        let flash_size = fmc.flash_size_bytes();
        // The placement rule's one owner (crate::geometry): the top two detected pages of flash.
        let store_base = geometry::store_base(flash_size, page_size as u32);
        Self {
            fmc,
            store_base,
            region_len: geometry::region_len(page_size),
            page_size,
        }
    }

    /// Absolute address of region offset `off`, bounds-checked against the region length.
    fn abs_off(&self, off: usize, len: usize) -> Result<u32, FlashError> {
        let end = off.checked_add(len).ok_or(FlashError::OutOfBounds)?;
        if end > self.region_len {
            return Err(FlashError::OutOfBounds);
        }
        Ok(self.store_base + off as u32)
    }

    /// Absolute base address of region page `page` (0 or 1), bounds-checked.
    fn abs_page(&self, page: usize) -> Result<u32, FlashError> {
        if page >= 2 {
            return Err(FlashError::OutOfBounds);
        }
        Ok(self.store_base + (page * self.page_size) as u32)
    }
}

impl Flash for FmcFlash {
    fn page_size(&self) -> usize {
        // The one straight forward to the HAL.
        self.fmc.page_size() as usize
    }

    fn as_bytes(&self) -> &[u8] {
        // Flash is memory-mapped at the absolute store base, so the whole region is a plain borrowed
        // slice (a read never touches the FMC). SAFETY: `[store_base, store_base + region_len)` is
        // inside main flash by construction (the top two pages of the detected extent), readable,
        // and immutable for the borrow's lifetime (the store holds `&mut self` for any write, so no
        // concurrent program/erase can run while this slice is live).
        unsafe { core::slice::from_raw_parts(self.store_base as *const u8, self.region_len) }
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let addr = self.abs_page(page)?;
        self.fmc.erase_page(addr).map_err(|e| map_err(e, Op::Erase))
    }

    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        let addr = self.abs_off(off, bytes.len())?;
        self.fmc
            .program(addr, bytes)
            .map_err(|e| map_err(e, Op::Program))
    }
}
