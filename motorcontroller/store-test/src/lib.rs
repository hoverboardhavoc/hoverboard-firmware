// SPDX-License-Identifier: GPL-3.0-or-later
//! Device-side helpers for the store firmware image: the RAM result/command channel.
//!
//! The entry reads the host-packed `cmd` from `CMD_ADDR`, constructs the store's `FmcFlash` from the
//! compiled chip's `hal::Chip` (the build's chip feature: 1 KiB pages on the GD32F103C8 and
//! GD32F130C8, 2 KiB on the GD32F103RC), runs the store `run` / `run_var` step, publishes a
//! `TestResult` (scalar `output`, or the variable-value `buf`/`len`), and busy-spins (NEVER `wfi`).

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
