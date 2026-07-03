//! The store region's placement rule, host-visible: ONE owner for
//! `store_base = (FLASH_BASE + flash_size) - 2 * page_size` (the top two pages of flash).
//!
//! [`crate::FmcFlash`] applies this rule on-target from the detected chip; host tools (the SWD
//! bridge's store readback, the hardware runner's flash driver) mirror the same region and used to
//! hardcode the derived addresses. They now read them from here, so the placement rule has exactly
//! one home. Always compiled (no HAL dependency): the inputs are plain part parameters.

/// Base address of the flash region on every supported part (the Cortex-M code alias).
pub const FLASH_BASE: u32 = 0x0800_0000;

/// Absolute base of the store region for a part: the top two pages of its flash.
#[inline]
pub const fn store_base(flash_size: u32, page_size: u32) -> u32 {
    (FLASH_BASE + flash_size) - 2 * page_size
}

/// The store region length in bytes: two pages (active + spare).
#[inline]
pub const fn region_len(page_size: usize) -> usize {
    2 * page_size
}
