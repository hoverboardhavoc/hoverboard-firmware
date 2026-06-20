//! Shared error/result vocabulary.
//!
//! A lean set of errors that more than one layer must agree on. It is deliberately small: each
//! crate still owns its own domain errors; only the cross-cutting ones live here. The first is
//! [`FlashError`], which Layer 1's `Flash` trait (the config-store backend) returns.

use core::fmt;

/// Errors from the flash backend that Layer 1's `Flash` trait surfaces.
///
/// The store logic is written against this so it stays host-testable on a RAM mock, while the real
/// FMC backend maps hardware faults onto the same variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum FlashError {
    /// An address or length fell outside the writable region (or was misaligned for the part).
    OutOfBounds,
    /// A program (write) operation did not take: the readback did not match what was written.
    ProgramFailed,
    /// An erase operation did not leave the page in the erased state.
    EraseFailed,
    /// The operation violated the part's alignment requirement (page or word).
    Misaligned,
    /// The region is locked or otherwise not writable in the current state.
    Locked,
}

impl fmt::Display for FlashError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            FlashError::OutOfBounds => "flash access out of bounds",
            FlashError::ProgramFailed => "flash program failed (readback mismatch)",
            FlashError::EraseFailed => "flash erase failed",
            FlashError::Misaligned => "flash access misaligned",
            FlashError::Locked => "flash region locked",
        };
        f.write_str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::FlashError;

    #[test]
    fn variants_are_distinct() {
        assert_ne!(FlashError::OutOfBounds, FlashError::ProgramFailed);
        assert_ne!(FlashError::EraseFailed, FlashError::Misaligned);
        assert_ne!(FlashError::Locked, FlashError::OutOfBounds);
    }

    #[test]
    fn display_is_nonempty() {
        for e in [
            FlashError::OutOfBounds,
            FlashError::ProgramFailed,
            FlashError::EraseFailed,
            FlashError::Misaligned,
            FlashError::Locked,
        ] {
            extern crate std;
            let s = std::format!("{e}");
            assert!(!s.is_empty());
        }
    }
}
