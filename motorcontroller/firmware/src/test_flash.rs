//! A host-test [`store::Flash`]: the one in-RAM flash the crate's tests mount a REAL store over.
//!
//! The store's own `MockFlash` is `#[cfg(test)]`-internal to that crate, so every consumer has to
//! bring its own (`crates/net`'s walk tests do the same). This module is that one: it was three
//! copies in this crate before the arm-time re-read's tests needed a fourth (the `ble_name` tests,
//! the `ble_bringup` tests, and now `arm`'s `read_arm_values` tests), so the model lives here once
//! and each test module borrows it.
//!
//! It models the silicon write rules the store relies on: a two-page region erased to `0xFF`,
//! halfword-aligned `program`, and no write that changes an already-programmed halfword.

use base::error::FlashError;
use store::Flash;

/// One page of the modelled region (so the region is two of these).
pub const PAGE: usize = 1024;

/// A two-page in-RAM region behaving as the store's flash seam expects.
pub struct TestFlash {
    bytes: std::vec::Vec<u8>,
}

impl TestFlash {
    /// An erased region (both pages all-`0xFF`), the state a virgin board mounts from.
    pub fn erased() -> Self {
        TestFlash {
            bytes: std::vec![0xFFu8; 2 * PAGE],
        }
    }
}

impl Flash for TestFlash {
    fn page_size(&self) -> usize {
        PAGE
    }
    fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let start = page * PAGE;
        let end = start + PAGE;
        if end > self.bytes.len() {
            return Err(FlashError::OutOfBounds);
        }
        self.bytes[start..end].fill(0xFF);
        Ok(())
    }
    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
            return Err(FlashError::Misaligned);
        }
        if off + bytes.len() > self.bytes.len() {
            return Err(FlashError::OutOfBounds);
        }
        for (i, &b) in bytes.iter().enumerate() {
            if self.bytes[off + i] != 0xFF && b != self.bytes[off + i] {
                return Err(FlashError::ProgramFailed);
            }
        }
        self.bytes[off..off + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}
