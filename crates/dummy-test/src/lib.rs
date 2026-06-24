//! Device-side helpers shared by the dummy firmware binaries.
//!
//! Each binary reads its input from `CMD_ADDR`, computes an output, and publishes a `TestResult` to
//! the fixed RAM tail with `ready` written LAST. The addresses are read/written directly with
//! volatile stores; we do NOT place a `#[link_section]` static at the RAM origin (that collides with
//! cortex-m-rt startup, see the spec).

#![no_std]

use core::ptr;
use test_shared::{CMD_ADDR, RESULT_ADDR, RESULT_READY};

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

/// Read the host-supplied input from `CMD_ADDR`.
#[inline(always)]
pub fn read_input() -> u32 {
    read_u32(CMD_ADDR)
}

/// Publish a completed run: write `output` first, then `ready` LAST, so the host never observes a
/// half-published result.
#[inline(always)]
pub fn publish(output: u32) {
    write_u32(RESULT_ADDR + 4, output);
    write_u32(RESULT_ADDR, RESULT_READY);
}
