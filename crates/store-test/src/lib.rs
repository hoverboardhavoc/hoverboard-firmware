//! The on-target store backend and the device-side helpers for the `store-test` images.
//!
//! [`FmcFlash`] is the region-relative [`store::Flash`] impl over the HAL's [`runtime_hal::fmc::Fmc`].
//! It is a THIN ADAPTER but NOT a pass-through: the HAL `Fmc` is region-agnostic, absolute-addressed,
//! and has no `read`, so `FmcFlash` adds what the store needs that the HAL deliberately omits:
//!
//! - **owns the region** (base + length) and maps the store's region-relative offset / page index to
//!   an absolute flash address (the store works in region coordinates, never an absolute address);
//! - **bounds-checks** every access against the region length (`OutOfBounds`);
//! - **provides `read`** as a direct memory-mapped load (flash is read-mapped, a read never touches
//!   the FMC);
//! - **maps errors** from the HAL's [`runtime_hal::error::FmcError`] onto [`base::error::FlashError`].
//!
//! The store region is the TOP TWO detected pages of flash:
//! `store_base = (FLASH_BASE + flash_size_bytes()) - 2 * page_size()`.
//!
//! The library half compiles for the host too (so the workspace host build / tests stay green); the
//! memory-mapped `read` is only meaningful on the target, but the type and its mapping are
//! host-checkable.

#![cfg_attr(target_os = "none", no_std)]

use base::error::FlashError;
use runtime_hal::chip::Chip;
use runtime_hal::error::FmcError;
use runtime_hal::fmc::Fmc;
use store::Flash;

/// The absolute base of main flash on both families (memory-mapped).
const FLASH_BASE: u32 = 0x0800_0000;

/// The region-relative [`Flash`] adapter over the HAL's [`Fmc`], owning the top-two-pages store
/// region. Constructed from the detected [`Chip`] (the same chip the bootloader's app-slot writes use).
pub struct FmcFlash {
    fmc: Fmc,
    /// The absolute base of the store region (top two detected pages).
    region_base: u32,
    /// The region length in bytes (`2 * page_size`).
    region_len: usize,
    /// The detected page size in bytes (1 or 2 KiB).
    page_size: usize,
}

impl FmcFlash {
    /// Build the store backend for the detected `chip`: acquire the HAL `Fmc`, then compute the store
    /// region as the top two detected pages of flash.
    pub fn new(chip: &Chip) -> FmcFlash {
        let fmc = Fmc::new(chip);
        let page_size = fmc.page_size() as usize;
        let region_len = 2 * page_size;
        // store_base = (FLASH_BASE + flash_size_bytes()) - 2 * page_size().
        let region_base = (FLASH_BASE + fmc.flash_size_bytes()) - region_len as u32;
        FmcFlash {
            fmc,
            region_base,
            region_len,
            page_size,
        }
    }

    /// The absolute base of the store region (the top two detected pages). Exposed so the host runner
    /// can erase the region for a clean slate; it is also the address the memory-mapped `read` uses.
    #[inline]
    pub fn region_base(&self) -> u32 {
        self.region_base
    }

    /// Map a region-relative offset to an absolute flash address, bounds-checked against the region.
    #[inline]
    fn abs(&self, off: usize, span: usize) -> Result<u32, FlashError> {
        let end = off.checked_add(span).ok_or(FlashError::OutOfBounds)?;
        if end > self.region_len {
            return Err(FlashError::OutOfBounds);
        }
        Ok(self.region_base + off as u32)
    }
}

/// Map a HAL [`FmcError`] from a PROGRAM op onto [`FlashError`] (the pinned table): `BadArg` ->
/// `Misaligned`, `WriteProtect` -> `Locked`, `NotErased`/`ProgramError`/`Timeout` -> `ProgramFailed`.
fn map_program_err(e: FmcError) -> FlashError {
    match e {
        FmcError::BadArg => FlashError::Misaligned,
        FmcError::WriteProtect => FlashError::Locked,
        FmcError::NotErased | FmcError::ProgramError | FmcError::Timeout => {
            FlashError::ProgramFailed
        }
        // `FmcError` is non-exhaustive; any future program-side variant is a failed program.
        _ => FlashError::ProgramFailed,
    }
}

/// Map a HAL [`FmcError`] from an ERASE op onto [`FlashError`]: `BadArg` -> `Misaligned`,
/// `WriteProtect` -> `Locked`, the rest (`NotErased`/`ProgramError`/`Timeout`) -> `EraseFailed`.
fn map_erase_err(e: FmcError) -> FlashError {
    match e {
        FmcError::BadArg => FlashError::Misaligned,
        FmcError::WriteProtect => FlashError::Locked,
        FmcError::NotErased | FmcError::ProgramError | FmcError::Timeout => FlashError::EraseFailed,
        // `FmcError` is non-exhaustive; any future erase-side variant is a failed erase.
        _ => FlashError::EraseFailed,
    }
}

impl Flash for FmcFlash {
    #[inline]
    fn page_size(&self) -> usize {
        self.page_size
    }

    fn read(&self, off: usize, buf: &mut [u8]) -> Result<(), FlashError> {
        let abs = self.abs(off, buf.len())?;
        // Flash is memory-mapped: a direct load, no FMC. Read byte-by-byte from the absolute address.
        for (i, b) in buf.iter_mut().enumerate() {
            // SAFETY: `abs + i` is within the bounds-checked store region of memory-mapped flash.
            *b = unsafe { core::ptr::read_volatile((abs + i as u32) as *const u8) };
        }
        Ok(())
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        // A page index (0 or 1) within the two-page region -> the absolute page-aligned address.
        let off = page
            .checked_mul(self.page_size)
            .ok_or(FlashError::OutOfBounds)?;
        let abs = self.abs(off, self.page_size)?;
        self.fmc.erase_page(abs).map_err(map_erase_err)
    }

    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        let abs = self.abs(off, bytes.len())?;
        self.fmc.program(abs, bytes).map_err(map_program_err)
    }
}

// --- device-side helpers (target only): the fixed-address result/command channel -----------------
//
// The same reserved-RAM-tail channel the dummy uses (test-shared). These are no-ops worth nothing on
// the host, so they are target-gated; the host build of this crate is just the FmcFlash type + its
// mapping (host-checkable).

#[cfg(target_os = "none")]
mod device {
    use core::ptr;
    use test_shared::{CMD_ADDR, RESULT_ADDR, RESULT_READY};

    /// Read the host-supplied phase index from `CMD_ADDR` (written before the run, survives reset).
    #[inline(always)]
    pub fn read_phase() -> u32 {
        // SAFETY: a fixed RAM address in the reserved tail, valid + aligned on the target.
        unsafe { ptr::read_volatile(CMD_ADDR as *const u32) }
    }

    /// Publish the read-back as `TestResult.output`, with `ready` written LAST.
    #[inline(always)]
    pub fn publish(output: u32) {
        // SAFETY: fixed RAM addresses in the reserved tail, valid + aligned on the target.
        unsafe {
            ptr::write_volatile((RESULT_ADDR + 4) as *mut u32, output);
            ptr::write_volatile(RESULT_ADDR as *mut u32, RESULT_READY);
        }
    }
}

#[cfg(target_os = "none")]
pub use device::{publish, read_phase};

// --- the injected per-part Chip ------------------------------------------------------------------
//
// `store-test` injects the detected part's `Chip` at mount (FmcFlash needs it to resolve the FMC page
// size + flash extent). Which chip is a build-time choice so one set of binaries covers all parts:
//
// - `f103` (default): the bench GD32F103C8, 64 KiB / 1 KiB pages (`descriptor_f103`).
// - `f130`: the bench GD32F130C8, 64 KiB / 1 KiB pages (`descriptor_f130`).
// - `chip2k`: a synthesized F103RC-style 256 KiB / 2 KiB-page part, for the 12-FET 2 KiB-page CI
//   path. There is no public 2 KiB descriptor, so this uses `runtime-hal/detect-internals` to
//   `synthesize` a 256 KiB F10x `Detected` (page size resolves to K2 = 2 KiB above the 128 KiB
//   threshold). No 12-FET hardware needed for the tier-2 model.
//
// A 64 KiB / 1 KiB chip works for the FMC on BOTH the F103 master and the F130 slave (the FMC
// register contract is family-identical), so the 1 KiB silicon path can reuse one chip; the feature
// split exists so each part's real descriptor is available where wanted.

/// The detected part's [`Chip`] this image is built for (see the module note on the feature split).
#[cfg(all(feature = "chip2k", not(any(feature = "f103", feature = "f130"))))]
pub fn selected_chip() -> Chip {
    use runtime_hal::detect::probe::Detected;
    use runtime_hal::detect::{synthesize, Family};
    // An F103RC-style high-density part: 256 KiB flash (> 128 KiB => K2 = 2 KiB pages), 2 advanced
    // timers / 2 ADCs (the high-density capability). Only the flash page size + extent matter to the
    // FMC, but the rest is the real high-density shape.
    Chip::from_descriptor(synthesize(&Detected {
        family: Family::F10x,
        flash_kib: 256,
        adv_timers: 2,
        adc_count: 2,
    }))
}

/// The detected part's [`Chip`] for the F130 slave build (1 KiB pages).
#[cfg(all(feature = "f130", not(feature = "chip2k")))]
pub fn selected_chip() -> Chip {
    Chip::from_descriptor(runtime_hal::detect::descriptor_f130())
}

/// The detected part's [`Chip`] for the default / F103 master build (1 KiB pages).
#[cfg(all(not(feature = "f130"), not(feature = "chip2k")))]
pub fn selected_chip() -> Chip {
    Chip::from_descriptor(runtime_hal::detect::descriptor_f103())
}
