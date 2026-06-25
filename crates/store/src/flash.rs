//! The store's only flash dependency, plus the host-test `MockFlash`.
//!
//! [`Flash`] is the store's region-relative seam: it deals only in offsets and page indices inside
//! the two-page store region, never an absolute flash address (the real `FmcFlash` adapter adds the
//! region base; that adapter is Pass 2 and lives behind a target gate). The whole store core is
//! generic over `Flash`, so the record format, scan, append, and compaction are exercised host-side
//! against [`MockFlash`].
//!
//! `MockFlash` models the silicon's real write rules (see the spec's "Flash write model"): an erase
//! fills a page with `0xFFFF`; `program` requires a halfword-aligned offset and even length and is
//! **write-once** (programming a halfword that is not still `0xFFFF` fails with [`FlashError`], it
//! does **not** AND bits in). Modeling write-once is the point: an AND-model mock would silently
//! pass a regression the silicon would reject.

use base::error::FlashError;

/// The store's region-relative flash seam (its only flash dependency).
///
/// Offsets and page indices are relative to the store region; the real adapter maps them to absolute
/// flash addresses. `as_bytes` is the whole region as a memory-mapped slice, so a variable value is
/// returned as a borrowed sub-slice of flash (no copy).
pub trait Flash {
    /// The erase granularity, one physical page (1 or 2 KiB on the detected part). One side of the
    /// ping-pong region is exactly one page.
    fn page_size(&self) -> usize;

    /// The whole region, memory-mapped. A read is a borrowed sub-slice (no copy).
    fn as_bytes(&self) -> &[u8];

    /// Erase one page back to all-`0xFFFF`. `page` is the page index within the region (0 or 1).
    fn erase_page(&mut self, page: usize) -> Result<(), FlashError>;

    /// Program `bytes` at region offset `off`. Halfword-aligned (`off` and `bytes.len()` both even)
    /// and **write-once**: each target halfword must still be `0xFFFF`.
    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError>;
}

/// An in-RAM [`Flash`] that models the silicon write rules, for host tests.
///
/// The backing store is the whole two-page region (`2 * page_size` bytes). It starts all-`0xFFFF`
/// (the erased state) and enforces halfword alignment + write-once on `program`, so the codec, log,
/// and compaction are tested without hardware. It is `#[cfg(test)]` (host-only): the no_std core
/// build never compiles it and so never needs an allocator.
#[cfg(test)]
pub struct MockFlash {
    page_size: usize,
    data: std::vec::Vec<u8>,
}

#[cfg(test)]
impl MockFlash {
    /// An erased two-page region with the given page size (so the region is `2 * page_size` bytes).
    /// The argument is the **page size**, not a region length: `1024` is two 1 KiB pages.
    pub fn erased(page_size: usize) -> Self {
        Self {
            page_size,
            data: std::vec![0xFF; 2 * page_size],
        }
    }

    /// Build a mock from a hand-crafted region image (for the planted torn-write / compaction
    /// scenarios). `image.len()` must be exactly `2 * page_size`.
    pub fn from_image(page_size: usize, image: &[u8]) -> Self {
        assert_eq!(
            image.len(),
            2 * page_size,
            "image must be exactly two pages"
        );
        Self {
            page_size,
            data: image.to_vec(),
        }
    }
}

#[cfg(test)]
impl Flash for MockFlash {
    fn page_size(&self) -> usize {
        self.page_size
    }

    fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let start = page
            .checked_mul(self.page_size)
            .ok_or(FlashError::OutOfBounds)?;
        let end = start
            .checked_add(self.page_size)
            .ok_or(FlashError::OutOfBounds)?;
        if end > self.data.len() {
            return Err(FlashError::OutOfBounds);
        }
        for b in &mut self.data[start..end] {
            *b = 0xFF;
        }
        Ok(())
    }

    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        // Halfword-aligned offset and even length: the FMC has no byte program and rejects a
        // straddling/odd write (`BadArg` -> `Misaligned`).
        if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
            return Err(FlashError::Misaligned);
        }
        let end = off
            .checked_add(bytes.len())
            .ok_or(FlashError::OutOfBounds)?;
        if end > self.data.len() {
            return Err(FlashError::OutOfBounds);
        }
        // Write-once: every target halfword must still be erased (`0xFFFF`). The silicon refuses a
        // re-program (the HAL's `NotErased` pre-check, mapped to `ProgramFailed`); it does NOT AND
        // bits in. Check the whole span first so a rejected write leaves flash untouched.
        for i in (off..end).step_by(2) {
            if self.data[i] != 0xFF || self.data[i + 1] != 0xFF {
                return Err(FlashError::ProgramFailed);
            }
        }
        self.data[off..end].copy_from_slice(bytes);
        Ok(())
    }
}

/// A [`Flash`] wrapper that injects a backend fault on the Nth program/erase, for the Tier-1
/// `Flash(..)`-on-auto-compaction-failure path (the one scenario that needs a failing backend and so
/// stays host-only).
#[cfg(test)]
pub struct FailingMockFlash {
    inner: MockFlash,
    /// Fail the program at this 0-based call index (and every later program). `None` never fails.
    fail_program_at: Option<usize>,
    program_calls: usize,
}

#[cfg(test)]
impl FailingMockFlash {
    /// Wrap an existing region image; the `n`th `program` (0-based) and all later ones return
    /// `ProgramFailed`.
    pub fn new(inner: MockFlash, fail_program_at: usize) -> Self {
        Self {
            inner,
            fail_program_at: Some(fail_program_at),
            program_calls: 0,
        }
    }
}

#[cfg(test)]
impl Flash for FailingMockFlash {
    fn page_size(&self) -> usize {
        self.inner.page_size()
    }

    fn as_bytes(&self) -> &[u8] {
        self.inner.as_bytes()
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        self.inner.erase_page(page)
    }

    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        let n = self.program_calls;
        self.program_calls += 1;
        if self.fail_program_at.is_some_and(|f| n >= f) {
            return Err(FlashError::ProgramFailed);
        }
        self.inner.program(off, bytes)
    }
}
