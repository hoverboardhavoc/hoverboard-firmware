//! Shared no_std contract for the firmware test harness (the bench/emulator rig).
//!
//! Each test in this rig IS the real thumbv7m Cortex-M firmware image, run two ways (under
//! Unicorn in CI, on real GD32 silicon over SWD on the bench) that reach their verdict the
//! same way: the subject writes a fixed-layout `#[repr(C)]` result struct to a fixed RAM
//! address with the `magic` word written LAST, and the harness polls `magic` then decodes by
//! field offset. A command word is handed in at a second fixed RAM address before the run.
//!
//! This crate is the single source of truth for that contract so a subject and its two host
//! readers never re-implement the assertion or disagree on an address. It is `no_std` and
//! alloc-free; the host readers (the Unicorn runner, the probe-rs bench driver) link it as a
//! plain library.
//!
//! The addresses live in a reserved RAM tail (see [`RESULT_ADDR`]/[`CMD_ADDR`] and the
//! [`MEMORY_X`] snippet): pinning a section at RAM origin ahead of `.data`/`.bss` collides
//! with cortex-m-rt startup (the `.bss` clear runs off the end into a bus fault, observed on
//! silicon). The proven fix is to shrink RAM in `memory.x` and write to a fixed address in the
//! carved-off tail, which cortex-m-rt never touches.

#![no_std]
// The host tests need std (for `assert!`, formatting); the crate itself is no_std. Matches `base`.
#[cfg(test)]
extern crate std;

/// The smoke subject's fixed-layout result struct.
///
/// `#[repr(C)]` fixes the field order/offsets so each host reader can decode the struct by byte
/// offset (the size-optimised release ELF drops `.symtab`, so there is no symbol to resolve).
/// `magic` is field 0 and is written LAST by the subject: a reader that sees [`SMOKE_MAGIC`] at
/// offset 0 knows `verdict` and `echo` below it are already committed. `magic == 0` means the
/// run never completed (it hung or faulted before the final store).
#[repr(C)]
pub struct SmokeObs {
    /// [`SMOKE_MAGIC`] once the run completed, written LAST. `0` = never completed.
    pub magic: u32,
    /// [`VERDICT_PASS`] or [`VERDICT_FAIL`]: the subject's self-check of its own echo.
    pub verdict: u32,
    /// `smoke(cmd)`: kept for the failure message so a fail shows what the subject computed.
    pub echo: u32,
}

/// The smoke kernel: transform the delivered command word. The XOR means a correct echo proves
/// the subject actually READ the command word the harness wrote, not a constant.
pub fn smoke(cmd: u32) -> u32 {
    cmd ^ 0xA5A5_A5A5
}

/// The shared assertion, run by BOTH the subject (to self-judge into [`SmokeObs::verdict`]) and
/// the host `cargo test`. The harness only transports and reads the verdict; it never re-derives
/// the expected value, so the two sides cannot drift.
pub fn smoke_ok(cmd: u32, echo: u32) -> bool {
    echo == smoke(cmd)
}

/// `magic` sentinel for [`SmokeObs`]: the run completed.
pub const SMOKE_MAGIC: u32 = 0x5A1E_5A1E;

/// `verdict` value for a passing self-check.
pub const VERDICT_PASS: u32 = 1;

/// `verdict` value for a failing self-check.
pub const VERDICT_FAIL: u32 = 0;

/// Fixed RAM address of the result struct: the start of the reserved RAM tail.
///
/// The tail is the top 256 bytes of the smallest part's 8 KiB RAM (the F130). `memory.x` shrinks
/// the linked RAM region to end at `0x2000_1F00`, so cortex-m-rt lays `.data`/`.bss`/stack out
/// below this and never touches the tail. Both host readers read this constant directly.
pub const RESULT_ADDR: u32 = 0x2000_1F00;

/// Fixed RAM address of the command word the harness writes BEFORE running.
///
/// It lives in the same reserved tail as [`RESULT_ADDR`], high enough that the 12-word
/// [`SmokeObs`] (and room for future larger result structs) never overlaps it: the tail spans
/// `0x2000_1F00..0x2000_2000` (256 bytes), the result struct starts at the bottom, and the
/// command word sits at the top at `0x2000_1FF0`. The subject reads it once at entry.
pub const CMD_ADDR: u32 = 0x2000_1FF0;

/// The `memory.x` every subject ships (the addresses above are the single source of truth; this
/// is the linker view of the same reserved-tail layout). FLASH is the smallest fleet flash
/// (64 KiB), RAM is the smallest fleet RAM (F130, 8 KiB) MINUS the 256-byte tail, so one image
/// links valid on every part and never places anything past 8 KiB (which would fault the F130 at
/// reset). The tail at `0x2000_1F00..0x2000_2000` holds [`RESULT_ADDR`] and [`CMD_ADDR`].
pub const MEMORY_X: &str = "\
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B harness tail @ 0x2000_1F00 */
}
";

#[cfg(test)]
mod tests {
    use super::*;

    // Pin the kernel with a literal so a change to the transform is a deliberate, reviewed edit.
    #[test]
    fn smoke_kernel_is_pinned() {
        assert_eq!(smoke(0xDEAD_0000), 0x7B08_A5A5);
    }

    // Exercise the shared check the subject also runs: a correct echo verifies.
    #[test]
    fn smoke_ok_accepts_correct_echo() {
        let cmd = 0xDEAD_0000;
        assert!(smoke_ok(cmd, smoke(cmd)));
        // A wrong echo is rejected (a constant or stale value would fail this).
        assert!(!smoke_ok(cmd, smoke(cmd).wrapping_add(1)));
    }

    // The result and command addresses must be distinct and both inside the reserved tail, or a
    // run would clobber its own command word or write outside the carved region.
    #[test]
    fn addresses_are_distinct_and_in_the_tail() {
        assert_ne!(RESULT_ADDR, CMD_ADDR);
        let tail_start = 0x2000_1F00u32;
        let tail_end = 0x2000_2000u32; // one past the 256-byte tail
        assert!(RESULT_ADDR >= tail_start && RESULT_ADDR < tail_end);
        assert!(CMD_ADDR >= tail_start && CMD_ADDR < tail_end);
        // The result struct (3 u32 words) must not reach the command word.
        assert!(RESULT_ADDR + core::mem::size_of::<SmokeObs>() as u32 <= CMD_ADDR);
    }
}
