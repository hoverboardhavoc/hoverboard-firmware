// SPDX-License-Identifier: GPL-3.0-or-later
//! The shared part of the hal bench validators: the result block the host reads over SWD.
//!
//! Every validator writes its fields into [`HB_RESULT`] (word 0 is the magic, word 1 the validator
//! id, the rest its own layout, documented in its source and decoded by `tools/hal-bench.py`), and
//! writes the magic LAST, so a block that carries it is complete. Then it busy-spins: a bare `wfi`
//! with `DBG_CTL0 = 0` locks SWD re-attach on these parts, and the fleet's boards do not wire NRST.
#![no_std]

/// Words in the result block.
pub const RESULT_WORDS: usize = 64;
/// The magic word 0 carries when the run completed.
pub const MAGIC: u32 = 0x4842_0001;

/// The result block, at the symbol the host looks up in the ELF.
#[no_mangle]
pub static mut HB_RESULT: [u32; RESULT_WORDS] = [0; RESULT_WORDS];

/// Write word `i` of the block.
#[inline(never)]
pub fn put(i: usize, v: u32) {
    if i < RESULT_WORDS {
        // SAFETY: single-threaded, no interrupt touches the block; volatile so a halted read
        // mid-run sees what has been written.
        unsafe { core::ptr::addr_of_mut!(HB_RESULT[i]).write_volatile(v) };
    }
}

/// Mark the run complete (id in word 1, the magic in word 0, last) and spin forever.
pub fn finish(id: u32) -> ! {
    put(1, id);
    put(0, MAGIC);
    loop {
        cortex_m::asm::nop();
    }
}

/// The DWT cycle counter (started by `hal::irq::enable_cycle_counter`).
#[inline]
pub fn cycles() -> u32 {
    // SAFETY: a read of the word-aligned DWT_CYCCNT register.
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

/// Spin `ms` milliseconds at 72 MHz on the cycle counter.
pub fn wait_ms(ms: u32) {
    let start = cycles();
    while cycles().wrapping_sub(start) < ms * 72_000 {}
}
