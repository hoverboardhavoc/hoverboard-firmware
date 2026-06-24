//! The store's only flash dependency: the region-relative [`Flash`] trait, plus a host [`MockFlash`].
//!
//! The whole store core is generic over [`Flash`], so the record format, scan, append, compaction,
//! and RAM table are exercised host-side against [`MockFlash`]. The trait is REGION-RELATIVE: offsets
//! and page indices are relative to the store region; the real on-target impl (`FmcFlash`, in the
//! `store-test` crate) adds the region base and maps to absolute flash. The trait returns
//! [`base::error::FlashError`].

use base::error::FlashError;

/// The store's only flash dependency. Region-relative (the impl adds the region base). The real impl
/// adapts the HAL's `Fmc`; the host tests use [`MockFlash`].
pub trait Flash {
    /// The flash page size in bytes (the detected part: 1 or 2 KiB). The store region is two pages.
    fn page_size(&self) -> usize;
    /// Read `buf.len()` bytes from region offset `off` into `buf` (flash is memory-mapped, a load).
    fn read(&self, off: usize, buf: &mut [u8]) -> Result<(), FlashError>;
    /// Erase the page at region page index `page` (0 or 1) back to `0xFF`.
    fn erase_page(&mut self, page: usize) -> Result<(), FlashError>;
    /// Program `bytes` at region offset `off`. Halfword-aligned `off` + even length, WRITE-ONCE:
    /// programming a halfword that is not still `0xFFFF` fails ([`FlashError::ProgramFailed`]).
    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError>;
}

/// A host mock that models the silicon's real write rules: erase fills a page with `0xFF`; `program`
/// requires a halfword-aligned offset + even length and is WRITE-ONCE (programming into any halfword
/// that is not still `0xFFFF` fails). This is NOT the old NOR-AND model (disproven on silicon): a
/// re-program is REFUSED, it does not AND bits in.
///
/// [`MockFlash::erased`]'s argument is the PAGE SIZE; the mock allocates the TWO-PAGE region
/// (`2 * page_size`). So `erased(1024)` is two 1 KiB pages and `erased(2048)` is two 2 KiB pages.
#[cfg(any(test, feature = "std"))]
pub struct MockFlash {
    page_size: usize,
    mem: std::vec::Vec<u8>,
}

#[cfg(any(test, feature = "std"))]
impl MockFlash {
    /// A fully-erased two-page region (`2 * page_size` bytes, all `0xFF`). The argument is the PAGE
    /// SIZE, not the region length.
    pub fn erased(page_size: usize) -> Self {
        Self {
            page_size,
            mem: std::vec![0xFF; 2 * page_size],
        }
    }

    /// The region length (`2 * page_size`).
    #[inline]
    pub fn region_len(&self) -> usize {
        self.mem.len()
    }
}

#[cfg(any(test, feature = "std"))]
impl Flash for MockFlash {
    fn page_size(&self) -> usize {
        self.page_size
    }

    fn read(&self, off: usize, buf: &mut [u8]) -> Result<(), FlashError> {
        let end = off.checked_add(buf.len()).ok_or(FlashError::OutOfBounds)?;
        if end > self.mem.len() {
            return Err(FlashError::OutOfBounds);
        }
        buf.copy_from_slice(&self.mem[off..end]);
        Ok(())
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let start = page
            .checked_mul(self.page_size)
            .ok_or(FlashError::OutOfBounds)?;
        let end = start
            .checked_add(self.page_size)
            .ok_or(FlashError::OutOfBounds)?;
        if end > self.mem.len() {
            return Err(FlashError::OutOfBounds);
        }
        for b in &mut self.mem[start..end] {
            *b = 0xFF;
        }
        Ok(())
    }

    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        if off & 1 != 0 || bytes.len() & 1 != 0 {
            return Err(FlashError::Misaligned);
        }
        let end = off
            .checked_add(bytes.len())
            .ok_or(FlashError::OutOfBounds)?;
        if end > self.mem.len() {
            return Err(FlashError::OutOfBounds);
        }
        // Write-once per halfword: a halfword that is not still 0xFFFF cannot be re-programmed. Check
        // the WHOLE span first so a partial write never lands (all-or-nothing, like the HAL pre-check).
        let mut i = 0;
        while i < bytes.len() {
            let cur = u16::from_le_bytes([self.mem[off + i], self.mem[off + i + 1]]);
            if cur != 0xFFFF {
                return Err(FlashError::ProgramFailed);
            }
            i += 2;
        }
        self.mem[off..end].copy_from_slice(bytes);
        Ok(())
    }
}
