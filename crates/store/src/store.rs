//! The store core: the two-page ping-pong log, mount/scan, append, compaction, and the typed API.
//!
//! The region is the top two detected pages of flash, modeled here region-relative (the `Flash`
//! seam adds the base). Each side is one physical page starting with an 8-byte page header
//! `[ magic:u32 | seq:u16 | reserved:u16 ]`; the active side is the one with a valid `magic` and the
//! higher `seq`, and records run from just after its header to the frontier.
//!
//! The store holds `&mut F` for its lifetime, so every read is `&self` and every write is `&mut self`,
//! which makes a flash-borrowing `get_str`/`get_bytes` slice and a concurrent mutation a *compile*
//! error (the "no append/erase while a flash slice is live" invariant, enforced by the borrow checker).

use base::error::FlashError;

use crate::field::{BlobField, Field, StrField};
use crate::flash::Flash;
use crate::key::{Key, Scalar, Type};
use crate::record::{self, Header, HeaderScan, MAGIC, PAGE_HEADER_LEN};

/// The store's error set on the typed path (no `TypeMismatch`/`UnknownKey`, those are compile
/// errors here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoreError {
    /// The value fits a clean page but not the active page's remaining space. `compact()` then retry.
    Full,
    /// The value can never fit a page's data area (compaction cannot help). Nothing is erased.
    ValueTooLarge,
    /// A backend erase/program failed; the `FlashError` is surfaced unchanged.
    Flash(FlashError),
}

impl From<FlashError> for StoreError {
    fn from(e: FlashError) -> Self {
        StoreError::Flash(e)
    }
}

/// The log-structured flash config store. Generic over the [`Flash`] seam.
pub struct Store<'f, F: Flash> {
    flash: &'f mut F,
    /// The page-relative byte offset of the active page (0 or `page_size`).
    active_page_off: usize,
    /// The region offset of the next free record slot (the append point) within the active page.
    frontier: usize,
    /// The active page's `seq` (bumped once per compaction).
    seq: u16,
}

impl<'f, F: Flash> Store<'f, F> {
    /// Mount the region: take the valid-`magic` side with the higher `seq` as active and scan its log
    /// for the frontier. Returns `Ok` in every normal case (virgin region, clean log, or a torn
    /// *payload* skipped); returns `Err(Flash(..))` only when a torn *header* at the frontier forces
    /// an auto-compaction whose erase/program fails.
    pub fn mount(flash: &'f mut F) -> Result<Self, StoreError> {
        let page_size = flash.page_size();
        let region = flash.as_bytes();

        // Pick the active side: valid magic, higher seq.
        let side0 = read_page_header(region, 0);
        let side1 = read_page_header(region, page_size);
        let active = match (side0, side1) {
            (Some(s0), Some(s1)) => {
                if s1 >= s0 {
                    Some((page_size, s1))
                } else {
                    Some((0, s0))
                }
            }
            (Some(s0), None) => Some((0, s0)),
            (None, Some(s1)) => Some((page_size, s1)),
            (None, None) => None,
        };

        let (active_page_off, seq) = match active {
            Some(a) => a,
            None => {
                // Virgin region: mount empty, frontier at page 0's first record slot. The first
                // append lazily programs page 0's header.
                return Ok(Self {
                    flash,
                    active_page_off: 0,
                    frontier: PAGE_HEADER_LEN,
                    seq: 0,
                });
            }
        };

        // Scan the active log to the frontier.
        match scan_frontier(region, active_page_off, page_size) {
            ScanResult::Frontier(frontier) => Ok(Self {
                flash,
                active_page_off,
                frontier,
                seq,
            }),
            ScanResult::TornHeader => {
                // The one place mount erases: pack survivors into the spare page and erase the torn
                // one (auto-compaction). Its erase/program failing is the sole reason mount is
                // fallible.
                let mut store = Self {
                    flash,
                    active_page_off,
                    frontier: 0, // unused before compaction rebuilds it
                    seq,
                };
                store.compact()?;
                Ok(store)
            }
        }
    }

    /// Read the newest committed record of a scalar field and decode it as `T`, else the field's
    /// default. The result is owned (the borrow ends immediately), so a `get` never blocks a `set`.
    pub fn get<T: Scalar>(&self, field: Field<T>) -> T {
        let key = field.key();
        match self.find_latest(key) {
            Some((off, h)) if h.type_tag == T::KIND.tag() => {
                let bytes = record::value_bytes(self.flash.as_bytes(), off, &h);
                if bytes.len() == T::WIDTH {
                    return T::read_le(bytes);
                }
                field.default()
            }
            _ => field.default(),
        }
    }

    /// Read the newest committed `STR` record as a flash-borrowing `&str`, else the default. A record
    /// that fails the UTF-8 check is ignored like any other and the read falls back to the default.
    /// Holding the slice borrows the store immutably, so a concurrent `set`/`compact` is a compile
    /// error until it is dropped.
    pub fn get_str(&self, field: StrField) -> &str {
        let key = field.key();
        if let Some((off, h)) = self.find_latest(key) {
            if h.type_tag == Type::Str.tag() {
                let bytes = record::value_bytes(self.flash.as_bytes(), off, &h);
                if let Ok(s) = core::str::from_utf8(bytes) {
                    return s;
                }
            }
        }
        field.default()
    }

    /// Read the newest committed `BLOB` record as flash-borrowing raw bytes, else the default.
    pub fn get_bytes(&self, field: BlobField) -> &[u8] {
        let key = field.key();
        if let Some((off, h)) = self.find_latest(key) {
            if h.type_tag == Type::Blob.tag() {
                return record::value_bytes(self.flash.as_bytes(), off, &h);
            }
        }
        field.default()
    }

    /// Append a scalar record now. The handle fixes the type, so a wrong-type write does not compile.
    pub fn set<T: Scalar>(&mut self, field: Field<T>, value: T) -> Result<(), StoreError> {
        let mut buf = [0u8; 8];
        value.write_le(&mut buf);
        self.append(field.key(), T::KIND.tag(), &buf[..T::WIDTH])
    }

    /// Append a `STR` record now.
    pub fn set_str(&mut self, field: StrField, value: &str) -> Result<(), StoreError> {
        self.append(field.key(), Type::Str.tag(), value.as_bytes())
    }

    /// Append a `BLOB` record now.
    pub fn set_bytes(&mut self, field: BlobField, value: &[u8]) -> Result<(), StoreError> {
        self.append(field.key(), Type::Blob.tag(), value)
    }

    /// Pack the latest record per key into the spare page (header-last), then erase the old page. The
    /// only erase. Key-agnostic: it preserves the latest record of every key it sees, known or not.
    pub fn compact(&mut self) -> Result<(), StoreError> {
        let page_size = self.flash.page_size();
        let old_off = self.active_page_off;
        let spare_off = if old_off == 0 { page_size } else { 0 };
        let spare_page = spare_off / page_size;
        let new_seq = self.seq.wrapping_add(1);

        // (0) Ensure the spare page is clean (normally already erased; a torn prior compaction may
        // have left partial data).
        if !page_is_erased(self.flash.as_bytes(), spare_off, page_size) {
            self.flash.erase_page(spare_page)?;
        }

        // (1) Pack the latest record per key into the spare, writing from offset PAGE_HEADER_LEN
        // upward while its header stays erased. Iterate the live keys of the active page; copy each
        // latest committed record verbatim (key-agnostic: payloads are never parsed).
        let mut write_off = spare_off + PAGE_HEADER_LEN;
        // Collect distinct latest records by walking the active log; re-fetch as_bytes per copy so
        // the borrow does not overlap the program call.
        let mut cursor = old_off + PAGE_HEADER_LEN;
        // Track which keys we have already copied (walk newest-wins by copying only the latest per
        // key). We walk forward and, for each good committed record, check whether a later record of
        // the same key exists ahead of it; if so skip (an earlier, superseded copy).
        loop {
            let region = self.flash.as_bytes();
            let here = cursor;
            let h = match record::parse_header(region, here) {
                HeaderScan::Good(h) => h,
                _ => break, // blank or torn: end of the walkable log
            };
            let next = here + record::record_size(h.len as usize);
            cursor = next;
            // Skip uncommitted (torn payload) records.
            if !record::is_committed(region, here, &h) {
                continue;
            }
            let key = Key {
                field_id: h.field_id,
                index: h.index,
            };
            // Is there a later committed record of the same key? If so this one is superseded.
            if has_later_record(region, next, old_off + page_size, key) {
                continue;
            }
            // Copy this record verbatim into the spare.
            let total = record::record_size(h.len as usize);
            // Bound copy buffer to the max record (page minus the two headers and val_crc fits).
            let mut tmp = [0u8; MAX_RECORD];
            tmp[..total].copy_from_slice(&region[here..here + total]);
            self.flash.program(write_off, &tmp[..total])?;
            write_off += total;
        }
        let new_frontier = write_off;

        // (2) Commit the new page's header LAST: program seq/reserved, then magic last of all (the
        // safety hinge, so a torn copy leaves the OLD page as the only valid side).
        let mut hdr = [0u8; PAGE_HEADER_LEN];
        hdr[4..6].copy_from_slice(&new_seq.to_le_bytes());
        // reserved (hdr[6..8]) left 0.
        // Program seq+reserved first (the upper halfword), magic (lower 4 bytes) last.
        self.flash
            .program(spare_off + 4, &hdr[4..PAGE_HEADER_LEN])?;
        hdr[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        self.flash.program(spare_off, &hdr[0..4])?;

        // (3) Only now erase the old page.
        let old_page = old_off / page_size;
        self.flash.erase_page(old_page)?;

        self.active_page_off = spare_off;
        self.frontier = new_frontier;
        self.seq = new_seq;
        Ok(())
    }

    // ---- internals ----

    /// The newest committed record of `key` in the active log, as `(offset, header)`, or `None`.
    fn find_latest(&self, key: Key) -> Option<(usize, Header)> {
        let region = self.flash.as_bytes();
        let page_size = self.flash.page_size();
        let page_end = self.active_page_off + page_size;
        let mut cursor = self.active_page_off + PAGE_HEADER_LEN;
        let mut found = None;
        while let HeaderScan::Good(h) = record::parse_header(region, cursor) {
            let here = cursor;
            cursor = here + record::record_size(h.len as usize);
            if cursor > page_end {
                break;
            }
            if h.field_id == key.field_id
                && h.index == key.index
                && record::is_committed(region, here, &h)
            {
                found = Some((here, h)); // newest wins: keep updating
            }
        }
        found
    }

    /// Append a record at the frontier. Lazily programs the active page header on a virgin region.
    fn append(&mut self, key: Key, type_tag: u8, value: &[u8]) -> Result<(), StoreError> {
        let page_size = self.flash.page_size();
        let len = value.len();
        let data_area = page_size - PAGE_HEADER_LEN;

        // Can it ever fit a clean page? (page minus page header minus this record).
        if record::record_size(len) > data_area {
            return Err(StoreError::ValueTooLarge);
        }

        // Lazily format a virgin page 0 header just before the first record (header-first is safe
        // here: the other side is blank, so a torn first record cannot out-rank surviving data).
        let need_header = read_page_header(self.flash.as_bytes(), self.active_page_off).is_none();
        if need_header {
            let hdr = record::encode_page_header(self.seq);
            self.flash.program(self.active_page_off, &hdr)?;
        }

        // Does it fit the active page's remaining space?
        let page_end = self.active_page_off + page_size;
        if self.frontier + record::record_size(len) > page_end {
            return Err(StoreError::Full);
        }

        let mut buf = [0u8; MAX_RECORD];
        let total = record::encode(key.field_id, key.index, type_tag, value, &mut buf);
        self.flash.program(self.frontier, &buf[..total])?;
        self.frontier += total;
        Ok(())
    }
}

/// The largest record buffer the store needs: a 2 KiB page minus the two 8-byte headers and the
/// 2-byte val_crc, rounded up to a convenient bound. A record never spans pages, so this caps every
/// encode/copy.
const MAX_RECORD: usize = 2048;

/// Read and validate a page header at `off`; returns its `seq` if `magic` is fully present, else
/// `None` (erased or torn).
fn read_page_header(region: &[u8], off: usize) -> Option<u16> {
    if off + PAGE_HEADER_LEN > region.len() {
        return None;
    }
    let magic = u32::from_le_bytes([
        region[off],
        region[off + 1],
        region[off + 2],
        region[off + 3],
    ]);
    if magic != MAGIC {
        return None;
    }
    Some(u16::from_le_bytes([region[off + 4], region[off + 5]]))
}

/// Is the page at `off..off+page_size` entirely erased (`0xFF`)?
fn page_is_erased(region: &[u8], off: usize, page_size: usize) -> bool {
    region[off..off + page_size].iter().all(|&b| b == 0xFF)
}

enum ScanResult {
    /// The frontier (first blank slot / clean end of log).
    Frontier(usize),
    /// A torn header was hit: the log is not walkable past here, mount must auto-compact.
    TornHeader,
}

/// Walk the active page from its first record; stop at a blank (the frontier) or a torn header.
fn scan_frontier(region: &[u8], page_off: usize, page_size: usize) -> ScanResult {
    let page_end = page_off + page_size;
    let mut cursor = page_off + PAGE_HEADER_LEN;
    loop {
        match record::parse_header(region, cursor) {
            HeaderScan::Blank => return ScanResult::Frontier(cursor),
            HeaderScan::Torn => return ScanResult::TornHeader,
            HeaderScan::Good(h) => {
                let next = cursor + record::record_size(h.len as usize);
                // A good header whose record would run past the page end is a corrupt len despite a
                // matching hdr_crc (cannot happen on a sane region); treat as torn to be safe.
                if next > page_end {
                    return ScanResult::TornHeader;
                }
                cursor = next;
                if cursor == page_end {
                    // Page exactly full, no blank slot: the frontier is the page end.
                    return ScanResult::Frontier(cursor);
                }
            }
        }
    }
}

/// Is there a later committed record of `key` in `region[start..end]`? Used during compaction to
/// keep only the latest per key.
fn has_later_record(region: &[u8], start: usize, end: usize, key: Key) -> bool {
    let mut cursor = start;
    while cursor < end {
        let h = match record::parse_header(region, cursor) {
            HeaderScan::Good(h) => h,
            _ => break,
        };
        let here = cursor;
        cursor = here + record::record_size(h.len as usize);
        if h.field_id == key.field_id
            && h.index == key.index
            && record::is_committed(region, here, &h)
        {
            return true;
        }
    }
    false
}
