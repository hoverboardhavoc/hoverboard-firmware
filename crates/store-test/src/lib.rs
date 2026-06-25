//! Device-side helpers for the store firmware image: the RAM result/command channel and the
//! build-feature-selected `Chip`.
//!
//! The entry reads the host-packed `cmd` from `CMD_ADDR`, constructs the store's `FmcFlash` from the
//! compile-time-selected `Chip`, runs the store `run` / `run_var` step, publishes a `TestResult`
//! (scalar `output`, or the variable-value `buf`/`len`), and busy-spins (NEVER `wfi`).
//!
//! The `Chip` is selected by build feature (`chip1k` / `chip2k`), NEVER by runtime `detect_chip()`:
//! detection's bus-fault probe is hardware-only and cannot run under Unicorn, so a detecting image
//! would hang the CI gate (see the storage spec, Tier 2).

#![no_std]

use core::ptr;
use test_shared::{CMD_ADDR, RESULT_ADDR, RESULT_BUF_LEN, RESULT_READY};

/// Volatile read of a `u32` at a fixed absolute address.
#[inline(always)]
pub fn read_u32(addr: u32) -> u32 {
    // SAFETY: `addr` is a fixed RAM address in the reserved tail, valid and aligned on the target.
    unsafe { ptr::read_volatile(addr as *const u32) }
}

/// Volatile write of a `u32` to a fixed absolute address.
#[inline(always)]
pub fn write_u32(addr: u32, value: u32) {
    // SAFETY: `addr` is a fixed RAM address in the reserved tail, valid and aligned on the target.
    unsafe { ptr::write_volatile(addr as *mut u32, value) }
}

/// Read the host-supplied `cmd = (scenario << 16) | phase` from `CMD_ADDR`.
#[inline(always)]
pub fn read_cmd() -> u32 {
    read_u32(CMD_ADDR)
}

/// Publish a SCALAR result: write `output` (+4), zero `len` (+8), then `ready` (+0) LAST, so the host
/// never observes a half-published result.
#[inline(always)]
pub fn publish_scalar(output: u32) {
    write_u32(RESULT_ADDR + 4, output);
    publish_len(0);
    write_u32(RESULT_ADDR, RESULT_READY);
}

/// Publish a VARIABLE-VALUE result: copy `bytes` into `TestResult.buf` (+10), set `len` (+8), zero
/// `output` (+4), then `ready` (+0) LAST. `bytes.len()` is capped at [`RESULT_BUF_LEN`].
#[inline(always)]
pub fn publish_var(bytes: &[u8]) {
    let n = if bytes.len() > RESULT_BUF_LEN {
        RESULT_BUF_LEN
    } else {
        bytes.len()
    };
    // buf is at RESULT_ADDR + 10 (ready@+0, output@+4, len@+8, buf@+10).
    for (i, &b) in bytes[..n].iter().enumerate() {
        // SAFETY: RESULT_ADDR + 10 + i is inside the reserved RAM tail for i < RESULT_BUF_LEN.
        unsafe { ptr::write_volatile((RESULT_ADDR + 10 + i as u32) as *mut u8, b) };
    }
    write_u32(RESULT_ADDR + 4, 0); // output unused for a variable case
    publish_len(n as u16);
    write_u32(RESULT_ADDR, RESULT_READY);
}

/// Write `TestResult.len` (a `u16` at +8) without disturbing the adjacent bytes.
#[inline(always)]
fn publish_len(len: u16) {
    // SAFETY: RESULT_ADDR + 8 is the `len` field (u16) in the reserved RAM tail.
    unsafe { ptr::write_volatile((RESULT_ADDR + 8) as *mut u16, len) };
}

/// The build-feature-selected detected-`Chip` the store's `FmcFlash` is constructed from.
///
/// `chip1k` builds it from the public 1 KiB descriptor (the F103 reference part); `chip2k` builds it
/// from `synthesize` behind `runtime-hal/detect-internals` (an F10x high-density 256 KiB / 2 KiB-page
/// part, the 12-FET-class `Chip`, since the public descriptors are both 1 KiB). NEVER `detect_chip()`.
///
/// Target-only: it names `runtime-hal`, which the host build does not link.
#[cfg(all(target_arch = "arm", feature = "chip1k", not(feature = "chip2k")))]
pub fn selected_chip() -> runtime_hal::Chip {
    runtime_hal::Chip::from_descriptor(runtime_hal::descriptor_f103())
}

/// The 2 KiB-page `Chip` (chip2k): an F10x high-density 256 KiB part, page size K2 (2 KiB), built
/// through the HAL's `synthesize` (no public 2 KiB descriptor exists). `flash_kib = 256 > 128` selects
/// K2; `adv_timers = 2` / `adc_count = 2` make a self-consistent high-density descriptor (the bench
/// 12-FET shape). store_base resolves to 0x0803_F000 (top of the 256 KiB flash).
#[cfg(all(target_arch = "arm", feature = "chip2k"))]
pub fn selected_chip() -> runtime_hal::Chip {
    use runtime_hal::detect::probe::Detected;
    let detected = Detected {
        family: runtime_hal::Family::F10x,
        flash_kib: 256,
        adv_timers: 2,
        adc_count: 2,
    };
    runtime_hal::Chip::from_descriptor(runtime_hal::synthesize(&detected))
}
