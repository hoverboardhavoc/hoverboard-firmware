//! The contract the dummy firmware and the host runners agree on.
//!
//! A "test" in this harness runs the *exact target image* two ways: under the Unicorn emulator on
//! the host (the CI gate) and on real GD32 silicon over the debug probe (the authoritative check).
//! Both executors need the same three things, so they live here, the one crate both the firmware
//! (thumbv7m) and the host tools (laptop / Pi) import:
//!
//! - [`dummy`], the function under test. It lives here, not in the firmware crate, because the host
//!   tools must recompute the expected value to judge the chip's output, and you cannot import from a
//!   firmware binary.
//! - [`TestResult`], the fixed-layout struct the firmware publishes and the host reads back.
//! - the fixed RAM addresses ([`RESULT_ADDR`], [`CMD_ADDR`]) and the [`RESULT_READY`] sentinel.
//!
//! The struct is `#[repr(C)]` so the host decodes it by byte offset with no symbol lookup (the
//! size-optimized release ELF drops its symbol table).
//!
//! `no_std`, alloc-free: it compiles for the chip. The `#[cfg(test)]` unit tests use `std`.

#![no_std]
// The host test harness needs std (for assert!, formatting, etc.); the crate itself is no_std.
#[cfg(test)]
extern crate std;

/// The function under test. XOR (a transform, not a copy) so a correct output can only happen if the
/// chip actually received the input the host sent: a firmware that ignored the input and echoed a
/// constant could not match.
pub fn dummy(input: u32) -> u32 {
    input ^ 0xA5A5_A5A5
}

/// The byte capacity of [`TestResult::buf`], the variable-value readback channel. 128 bytes fits
/// `device.name` and the store test blob and stays well under the `RESULT_ADDR..CMD_ADDR` gap
/// (~240 bytes), so the extension does not move `RESULT_ADDR` / `CMD_ADDR`.
pub const RESULT_BUF_LEN: usize = 128;

/// What the firmware publishes; the host reads it back and checks `output` (scalar) or
/// `buf[..len]` (variable-value cases).
///
/// `#[repr(C)]` pins the field offsets: `ready` at +0, `output` at +4, `len` at +8, `buf` at +10.
/// The host decodes by offset, not by symbol. The `len` / `buf` fields are an ADDITIVE extension for
/// the store's variable-value (`get_str` / `get_bytes`) readback: the existing `ready@+0` /
/// `output@+4` offsets the dummy harness depends on are unchanged, so the dummy images are
/// unaffected (they read/write only `ready` + `output`). There is deliberately no `verdict` field:
/// the device does not judge itself (that would be the device comparing its own output to its own
/// recomputation, a circular check). The host does the judging from the input it sent.
#[repr(C)]
pub struct TestResult {
    /// [`RESULT_READY`] once the run completed. Written LAST, after `output` / `len` / `buf`, so the
    /// host never reads a half-published result.
    pub ready: u32,
    /// The scalar output (e.g. `dummy(input)`, or the store scalar read-back). At offset +4.
    pub output: u32,
    /// The number of valid bytes in `buf` for a variable-value readback (`0` for a scalar case).
    /// At offset +8.
    pub len: u16,
    /// The variable-value readback bytes (`buf[..len]`), written by the store's `get_str` /
    /// `get_bytes` cases; the host compares them byte-identical to the expected literal. At offset
    /// +10 (immediately after `len`, no padding: `len` is a `u16` and `buf` is `u8`-aligned).
    pub buf: [u8; RESULT_BUF_LEN],
}

// Pin the `#[repr(C)]` field offsets at compile time: the host decodes the published struct by byte
// offset (the size-optimized ELF drops its symbol table), and the dummy harness depends on
// `ready@+0` / `output@+4` staying put across the additive `len` / `buf` extension.
const _: () = {
    assert!(core::mem::offset_of!(TestResult, ready) == 0);
    assert!(core::mem::offset_of!(TestResult, output) == 4);
    assert!(core::mem::offset_of!(TestResult, len) == 8);
    assert!(core::mem::offset_of!(TestResult, buf) == 10);
};

/// `ready` holds this once the run finished. A distinctive sentinel so a zeroed or uninitialized RAM
/// word is never mistaken for "ready".
pub const RESULT_READY: u32 = 0x5A1E_5A1E;

/// Start of the reserved RAM tail: the firmware writes [`TestResult`] here (`ready` at this address,
/// `output` at +4). Outside the linked RAM region, so it survives reset and the startup `.bss` clear.
pub const RESULT_ADDR: u32 = 0x2000_1F00;

/// The host writes the input word here before running; the device reads it after reset. Also in the
/// reserved tail, so it survives reset and is not zeroed by startup.
pub const CMD_ADDR: u32 = 0x2000_1FF0;

#[cfg(test)]
mod tests {
    use super::dummy;

    // Level 1: the correct value is accepted.
    #[test]
    fn dummy_pass() {
        assert_eq!(dummy(0xDEAD_0000), 0x7B08_A5A5);
    }

    // Level 1: a wrong value (the correct one with one bit flipped) is rejected.
    #[test]
    fn dummy_fail() {
        assert_ne!(dummy(0xDEAD_0000), 0x7B08_A5A5 ^ 1);
    }
}
