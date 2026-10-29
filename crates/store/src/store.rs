//! The store core: the two-page ping-pong log, mount/scan, append, compaction, and the typed API.
//!
//! The region is the top two detected pages of flash, modeled here region-relative (the `Flash`
//! seam adds the base). Each side is one physical page starting with an 8-byte page header
//! `[ magic:u32 | seq:u16 | reserved:u16 ]`; the active side is the one with a valid `magic` and the
//! wrapping-newer `seq` ([`side1_is_newer`], because `seq` wraps at 65535), and records run from just
//! after its header to the frontier.
//!
//! The store holds `&mut F` for its lifetime, so every read is `&self` and every write is `&mut self`,
//! which makes a flash-borrowing `get_text`/`get_bytes` slice and a concurrent mutation a *compile*
//! error (the "no append/erase while a flash slice is live" invariant, enforced by the borrow checker).

use base::error::FlashError;

use crate::field::{lookup_key, BlobField, Field, StrField};
use crate::flash::Flash;
use crate::key::{Key, Scalar, Type, ERASED_TAG};
use crate::record::{self, Header, HeaderScan, MAGIC, PAGE_HEADER_LEN};
use crate::value::Value;

/// The store's error set on the typed path (no `TypeMismatch`/`UnknownKey`, those are compile
/// errors here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoreError {
    /// The value fits a clean page but not the active page's remaining space. `compact()` then retry.
    Full,
    /// The value can never fit a page's data area (compaction cannot help). Nothing is erased.
    ValueTooLarge,
    /// A compaction packed the spare page but could not COMMIT it: the committed header did not
    /// read back, so the new page is not valid and the old one still is. No data is lost (the old
    /// page is never erased unless the commit verified), but the region cannot be reclaimed and
    /// every further write that needs space will fail the same way. The spare is erased on the way
    /// out so the region is not left holding a half-built page.
    CommitFailed,
    /// A backend erase/program failed; the `FlashError` is surfaced unchanged.
    Flash(FlashError),
}

impl From<FlashError> for StoreError {
    fn from(e: FlashError) -> Self {
        StoreError::Flash(e)
    }
}

/// The store's error set on the **dynamic** `Key`/[`Value`] path (`get_value` / `set_value`), which a
/// schema-less consumer (L3's `CONFIG_*`) uses. Unlike the typed handles, a raw `Key` may name no
/// field or carry a value of the wrong type, so this path has the two errors the typed path makes
/// compile-time impossible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DynError {
    /// No field declares this `field_id` (not in the registry).
    UnknownKey,
    /// The value's [`Type`] does not match the field's registered type.
    TypeMismatch,
    /// The field is read-only ([`crate::FieldDef::read_only`]): the firmware's own fact, never
    /// stored.
    ReadOnly,
    /// The underlying append failed (`Full` / `ValueTooLarge` / a flash error).
    Store(StoreError),
}

impl From<StoreError> for DynError {
    fn from(e: StoreError) -> Self {
        DynError::Store(e)
    }
}

/// Whether [`Store::get_value`]'s answer came from a record or from the field's default
/// (`specs/cli-config.md` section 6.1): what the L3 `CONFIG_READ` reports as `CFG_OK` against
/// `CFG_DEFAULT`, so a client can tell a key the board holds from one that follows the image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stored {
    /// The newest committed record of the key holds a value.
    Yes,
    /// The key has no record, or its newest record is a tombstone: the value is the default.
    No,
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
    /// Mount the region: take the valid-`magic` side with the wrapping-newer `seq` as active and scan its log
    /// for the frontier. Returns `Ok` in every normal case (virgin region, clean log, or a torn
    /// *payload* skipped); returns `Err(Flash(..))` only when a torn *header* at the frontier forces
    /// an auto-compaction whose erase/program fails.
    pub fn mount(flash: &'f mut F) -> Result<Self, StoreError> {
        let page_size = flash.page_size();
        let region = flash.as_bytes();

        // Pick the active side: valid magic, wrapping-newer seq (see `side1_is_newer`).
        let side0 = read_page_header(region, 0);
        let side1 = read_page_header(region, page_size);
        let active = match (side0, side1) {
            (Some(s0), Some(s1)) => {
                if side1_is_newer(s0, s1) {
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
            ScanResult::TornHeader(_) => {
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

    /// Read the newest committed `STR` record as flash-borrowing BYTES, else the field's default.
    /// Holding the slice borrows the store immutably, so a concurrent `set`/`compact` is a compile
    /// error until it is dropped.
    ///
    /// **Bytes, not a `&str`, and the name says so.** The board does not validate a `STR` record as
    /// UTF-8 (`specs/decision-flash-budget.md`, shrink round 2 item 4: the validator cost 496 B of
    /// flashed span to serve one field), so it makes no encoding claim about a record it did not
    /// write. The asymmetry against [`Store::set_str`] is deliberate: a WRITE takes text, because
    /// the caller has a validated string, while a READ returns whatever was last written. An absent
    /// or wrong-type record still falls back to the default, exactly as before.
    pub fn get_text(&self, field: StrField) -> &[u8] {
        let key = field.key();
        if let Some((off, h)) = self.find_latest(key) {
            if h.type_tag == Type::Str.tag() {
                return record::value_bytes(self.flash.as_bytes(), off, &h);
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

    /// Walk the active page ONCE, handing `f` every committed record whose key is in `keys`, in
    /// log order, as `(key, type tag, value bytes)`.
    ///
    /// The reason it exists, and the arithmetic (`specs/storage-layer.md`, "What a row read
    /// costs"). [`Store::get`] resolves ONE field by walking the whole page: the log is
    /// append-only and newest-wins, so the walk cannot stop at the first match, it has to reach
    /// the frontier. That is the right cost for one field and the wrong one for a ROW. The
    /// firmware's arm-time value row is twenty-one fields, and read a field at a time it walked
    /// the page twenty-one times per arm, CRC-checking every record it passed on every walk.
    /// Measured on the offroad master on 2026-10-22 at a 67%-full page (53 records): **23-36 ms
    /// of blocked main loop per arm**, which overran both RX rings and tripped the peer-staleness
    /// supervision on both boards, stopping the wheels about once a second.
    ///
    /// **Newest-wins is the pass's own order**, not a comparison: records arrive oldest first and
    /// the caller assigns, so the last one it sees for a key is the one that stands. That is
    /// exactly [`Store::get`]'s rule, which is why a caller built this way reads what `get` reads.
    ///
    /// **It hands back nothing and holds nothing.** The caller's own destination is the only
    /// storage, deliberately: a row of resolved slots is a STACK object on the control task's
    /// path, and the margin there is a few hundred bytes (`CTRL_OBS` word 30 read 408 B free on
    /// the offroad slave). Twenty-one slots of anything is a real fraction of that.
    ///
    /// `keys` is the filter, so `is_committed` (a CRC over the value) is paid only for records
    /// the caller wants rather than for every record on the page.
    pub fn for_each_in(&self, keys: &[Key], mut f: impl FnMut(Key, u8, &[u8])) {
        let region = self.flash.as_bytes();
        let page_end = self.active_page_off + self.flash.page_size();
        let mut cursor = self.active_page_off + PAGE_HEADER_LEN;
        while let HeaderScan::Good(h) = record::parse_header(region, cursor) {
            let here = cursor;
            cursor = here + record::record_size(h.len as usize);
            if cursor > page_end {
                break;
            }
            let key = Key {
                field_id: h.field_id,
                index: h.index,
            };
            if keys.contains(&key) && record::is_committed(region, here, &h) {
                f(key, h.type_tag, record::value_bytes(region, here, &h));
            }
        }
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

    /// Dynamic read by raw [`Key`] (the L3 `CONFIG_READ` path). Looks the field up in the registry for
    /// its [`Type`], decodes the newest committed record as a [`Value`] (flash-borrowing for
    /// `Str`/`Bytes`), and falls back to the registered default when the key is absent, the stored
    /// record is the wrong type, or its bytes are malformed. `UnknownKey` if no field declares the id.
    ///
    /// [`Stored`] comes from the same walk: `No` when the key has no record or its newest record is
    /// a tombstone, `Yes` otherwise (a wrong-type record is still a record, and reads the default).
    ///
    /// This is the dynamic `set`/`get` the store offers directly today (write-through); there is no
    /// apply-live / persist-when-disarmed split (`specs/l3.md`, "`CONFIG_*` is the wire face").
    pub fn get_value(&self, key: Key) -> Result<(Value<'_>, Stored), DynError> {
        let def = lookup_key(key).ok_or(DynError::UnknownKey)?;
        match self.find_latest(key) {
            Some((off, h)) if h.type_tag != ERASED_TAG => {
                let mut v = def.default;
                if h.type_tag == def.kind.tag() {
                    let bytes = record::value_bytes(self.flash.as_bytes(), off, &h);
                    if let Some(d) = Value::decode(def.kind, bytes) {
                        v = d;
                    }
                }
                Ok((v, Stored::Yes))
            }
            _ => Ok((def.default, Stored::No)),
        }
    }

    /// Un-store `key` (`specs/storage-layer.md`, "Read / write model"): append a tombstone, so the
    /// next read returns the default and reports [`Stored::No`]. Registry-checked like
    /// [`Store::set_value`] (`UnknownKey`, `ReadOnly`). A key with no record, or whose newest record
    /// is already a tombstone, is left alone and nothing is written.
    pub fn erase_value(&mut self, key: Key) -> Result<(), DynError> {
        let def = lookup_key(key).ok_or(DynError::UnknownKey)?;
        if def.read_only {
            return Err(DynError::ReadOnly);
        }
        match self.find_latest(key) {
            Some((_, h)) if h.type_tag != ERASED_TAG => Ok(self.append(key, ERASED_TAG, &[])?),
            _ => Ok(()),
        }
    }

    /// Dynamic write by raw [`Key`] (the L3 `CONFIG_WRITE` path). Validates the value's [`Type`]
    /// against the registry (`UnknownKey` / `TypeMismatch`) and appends the record now (write-through).
    /// A read-only field is refused (`ReadOnly`) whatever the value, and nothing is appended.
    pub fn set_value(&mut self, key: Key, value: Value) -> Result<(), DynError> {
        let def = lookup_key(key).ok_or(DynError::UnknownKey)?;
        if def.read_only {
            return Err(DynError::ReadOnly);
        }
        if value.kind() != def.kind {
            return Err(DynError::TypeMismatch);
        }
        // `Str`/`Bytes` already hold their payload; a scalar is encoded into a small buffer (<= 8 B).
        let mut scratch = [0u8; 8];
        let bytes: &[u8] = match value {
            Value::Str(b) | Value::Bytes(b) => b,
            scalar => {
                let n = scalar.encode(&mut scratch);
                &scratch[..n]
            }
        };
        self.append(key, def.kind.tag(), bytes)?;
        Ok(())
    }

    /// Pack the latest record per key into the spare page (header-last), then erase the old page. The
    /// only erase. Key-agnostic: it preserves the latest record of every key it sees, known or not,
    /// except a key whose latest record is a tombstone, which it drops.
    ///
    /// Every header of the source log is CRC-checked exactly once, by the pass at (1a); the copy
    /// walk and the supersede scan then hop that validated chain with
    /// [`record::header_unchecked`]. The whole repack has to fit the firmware's watchdog window,
    /// and the CRC is what it used to spend its time on (`specs/storage-layer.md`, "Compaction and
    /// the watchdog window").
    pub fn compact(&mut self) -> Result<(), StoreError> {
        let page_size = self.flash.page_size();
        let old_off = self.active_page_off;
        let spare_off = if old_off == 0 { page_size } else { 0 };
        let spare_page = spare_off / page_size;
        // `SEQ_UNCOMMITTED` is the "not committed yet" reading of the hinge halfword, so it is not
        // a sequence number: skip it on the wrap. One value out of 65,536 costs nothing, and the
        // wrapping comparison in `side1_is_newer` is unaffected by a single skipped step.
        let mut new_seq = self.seq.wrapping_add(1);
        if new_seq == record::SEQ_UNCOMMITTED {
            new_seq = new_seq.wrapping_add(1);
        }

        // (0) Ensure the spare page is clean (normally already erased; a torn prior compaction may
        // have left partial data).
        if !page_is_erased(self.flash.as_bytes(), spare_off, page_size) {
            self.flash.erase_page(spare_page)?;
        }

        // (1a) Walk the active log ONCE with the CRC, to the end of the chain: its frontier, or a
        // torn header (which compaction is also the recovery for, so both are just "the end"). The
        // pass is what makes every header in `[first record, log_end)` known-good, which is what
        // (1b) and [`has_later_record`] then rely on to hop the chain without re-CRCing a header.
        //
        // Before this pass existed, every hop of the supersede scan re-parsed and re-CRC'd the
        // header it landed on, and that scan runs once per source record: a full 1 KiB page of 76
        // records cost about 2,000 header CRCs and took 419 ms on an F130, against the firmware's
        // 500 ms free watchdog (`specs/storage-layer.md`, "Compaction and the watchdog window";
        // `specs/bench-evidence/2026-10-23/store-compact-window/`).
        let log_end = match scan_frontier(self.flash.as_bytes(), old_off, page_size) {
            ScanResult::Frontier(end) | ScanResult::TornHeader(end) => end,
        };

        // (1b) Pack the latest record per key into the spare, writing from offset PAGE_HEADER_LEN
        // upward while its header stays erased. Iterate the live keys of the active page; copy each
        // latest committed record verbatim (key-agnostic: payloads are never parsed).
        let mut write_off = spare_off + PAGE_HEADER_LEN;
        // Collect distinct latest records by walking the active log; re-fetch as_bytes per copy so
        // the borrow does not overlap the program call.
        let mut cursor = old_off + PAGE_HEADER_LEN;
        // Track which keys we have already copied (walk newest-wins by copying only the latest per
        // key). We walk forward and, for each good committed record, check whether a later record of
        // the same key exists ahead of it; if so skip (an earlier, superseded copy).
        while cursor < log_end {
            let region = self.flash.as_bytes();
            let here = cursor;
            let Some(h) = record::header_unchecked(region, here) else {
                break; // not reachable inside a chain (1a) validated; the end of the walk either way
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
            if has_later_record(region, next, log_end, key) {
                continue;
            }
            // The latest record is a tombstone: nothing is copied, so the key is gone from the new
            // page (`specs/storage-layer.md`, "A tombstone is dropped, not copied").
            if h.type_tag == ERASED_TAG {
                continue;
            }
            // Copy this record verbatim into the spare, in ascending [`COMPACT_CHUNK`] pieces. The
            // staging buffer is the chunk, not the record: this frame sits at the bottom of the
            // deepest chain in the firmware image, and a record-sized one spent 272 B of an 8 KiB
            // part's stack there (see [`COMPACT_CHUNK`]).
            let total = record::record_size(h.len as usize);
            let mut done = 0;
            while done < total {
                let n = if total - done < COMPACT_CHUNK {
                    total - done
                } else {
                    COMPACT_CHUNK
                };
                let mut tmp = [0u8; COMPACT_CHUNK];
                // Re-fetch the source each piece: the immutable `as_bytes()` borrow has to end
                // before the `&mut` program below, which is the same reason the whole-record copy
                // staged in RAM at all.
                tmp[..n].copy_from_slice(&self.flash.as_bytes()[here + done..here + done + n]);
                self.flash.program(write_off + done, &tmp[..n])?;
                done += n;
            }
            write_off += total;
        }
        let new_frontier = write_off;

        // (2) Commit the new page's header LAST, and ASCENDING: magic first, `seq` last of all.
        // `seq` is the hinge, so a copy torn before its final halfword leaves the new side reading
        // `SEQ_UNCOMMITTED` and the OLD page as the only valid side.
        //
        // The hinge is the HIGHEST halfword of the header and not the lowest, because a descending
        // write cannot land on this silicon: programming the low halfword of a word closes the high
        // halfword of the word below it, so the old "seq then magic" order could never finish its
        // magic once `seq` was down (`runtime_hal::fmc`, and
        // `specs/bench-evidence/2026-10-23/store-compact-commit/`). Programming the records first
        // and the header after is still fine: the records start at `PAGE_HEADER_LEN`, which closes
        // only `hdr[6..8]`, and the store never writes there.
        let hdr = record::encode_page_header(new_seq);
        let committed = self
            .flash
            .program(spare_off, &hdr[0..4])
            .and_then(|()| self.flash.program(spare_off + 4, &hdr[4..]));

        // (3) VERIFY the commit before acting on it, and treat a refused program and a program
        // that silently did not take as the SAME outcome. An unverified commit is indistinguishable
        // from a torn one: both leave a page the next mount ignores, while the caller was told the
        // write succeeded. Read the committed header back through the same memory-mapped view a
        // mount would use; if the programs errored, or it is not this page at this `seq`, erase the
        // spare (so the region is not left holding a half-built page for the next compaction to
        // clean up) and report `CommitFailed` - never the generic `Full` a caller reads as
        // "out of space", and never `Ok` on an uncommitted page.
        let spare_page = spare_off / page_size;
        if committed.is_err() || read_page_header(self.flash.as_bytes(), spare_off) != Some(new_seq)
        {
            // Best effort: the spare is already not a valid side, so a failed erase here changes
            // nothing about what mounts, and the commit failure is the error worth reporting.
            let _ = self.flash.erase_page(spare_page);
            return Err(StoreError::CommitFailed);
        }

        // (4) The new page is committed and proven. Only now erase the old one.
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

        // Can it ever fit a clean page? The writable record is the smaller of the page's data area
        // and [`MAX_RECORD`], the format cap. Neither write path stages a record-sized buffer any
        // more (`append` streams to flash, below; [`Store::compact`] copies in [`COMPACT_CHUNK`]
        // pieces), so this bound is about what a page can hold, not about RAM.
        if record::record_size(len) > data_area.min(MAX_RECORD) {
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

        // Stream the record straight to flash: a tiny fixed header block, the value programmed
        // **directly from the caller's `&[u8]`**, then the `val_crc` commit halfword last. No
        // full-record staging buffer exists, so `append`'s stack frame is a few bytes rather than
        // `MAX_RECORD` - which matters because `append` is the deepest frame on the hot
        // `ASSIGN -> set -> append -> program` path in the 8 KiB-RAM image. The bytes written are
        // byte-identical to `record::encode` (shared `encode_header` / `val_crc_of` codec), so the
        // torn-payload, torn-header, write-once, and halfword-alignment semantics are preserved.
        let header = record::encode_header(key.field_id, key.index, type_tag, len as u16);
        let vc = record::val_crc_of(&header, value);
        let off = self.frontier;

        // (1) The 8-byte header (even length, halfword-aligned: the frontier is always even).
        self.flash.program(off, &header)?;
        // (2) The value, halfword-programmed. The even prefix streams straight from the caller's
        // slice; an odd tail byte is paired with one `0xFF` pad (the exact pad `encode` writes) in a
        // 2-byte block, so the value span stays even and the next record stays halfword-aligned.
        let even = len & !1;
        if even != 0 {
            self.flash
                .program(off + record::HEADER_LEN, &value[..even])?;
        }
        if len & 1 == 1 {
            let tail = [value[len - 1], 0xFF];
            self.flash.program(off + record::HEADER_LEN + even, &tail)?;
        }
        // (3) The `val_crc` commit halfword, programmed LAST (the commit marker): a record is valid
        // only once this lands, so a power loss before it leaves a skippable torn payload.
        let padded = record::padded_len(len);
        self.flash
            .program(off + record::HEADER_LEN + padded, &vc.to_le_bytes())?;

        self.frontier += record::record_size(len);
        Ok(())
    }
}

/// The largest record the store will write, in bytes (header + padded value + `val_crc`).
///
/// A FORMAT cap, not a RAM one. `append` streams the record to flash (header /
/// value-from-the-caller's-slice / `val_crc`, in halfword programs) and [`Store::compact`] copies each
/// survivor in [`COMPACT_CHUNK`] pieces, so no buffer on either path is sized by this number; it only
/// says how big a value a page may hold. 272 B covers every record the fleet writes (a `node_address`
/// u8, a `CONFIG_*` scalar or short string, the largest planted store-test record ~266 B) and leaves
/// a 1 KiB page able to hold several.
const MAX_RECORD: usize = 272;

/// Bytes [`Store::compact`] stages in RAM per `program` call while copying a surviving record.
///
/// Compaction copies each record *verbatim within the same flash*, which cannot stream the way
/// `append` does: the immutable `as_bytes()` borrow of the source and the `&mut` `program` would
/// alias, and on silicon the core cannot read the flash the FMC is programming either. So the bytes
/// go through RAM. What they do NOT have to do is go through RAM a whole record at a time.
///
/// **This is a stack decision and the frame it sits in is the deepest in the firmware image.** The
/// chain is `service_loop -> Responder::ingest -> apply_write -> Store::compact`, reached by any
/// `CONFIG_WRITE` that lands on a full page, and a [`MAX_RECORD`]-sized staging buffer put 272 B of
/// an 8 KiB part's stack at the bottom of it: measured on the offroad F130 master on 2026-10-23, a
/// compacting `CONFIG_WRITE` took the stack high-water margin to **224 B of 2,072 B painted**, under
/// the >= 250 B floor in `specs/firmware.md` ("RAM/stack budget"). The chunk costs 32 B there instead.
///
/// 32 B and not less: every record the fleet writes (12 to 18 B) still goes in ONE `program` call, so
/// the common case pays no extra FMC cycles, and only a blob longer than 32 B is split. It must stay
/// EVEN, because `Flash::program` takes halfword-aligned offsets and even lengths and `record_size`
/// is always even, which keeps every piece and every piece's offset even.
///
/// Pieces are written in ASCENDING address order, like everything else the store programs into a
/// page (`runtime_hal::fmc`: programming a word's low halfword closes the halfword below it for the
/// rest of that page's erase cycle).
pub(crate) const COMPACT_CHUNK: usize = 32;

/// Of two valid sides, is side 1 (the high page) the newer one?
///
/// `compact()` advances `seq` with `wrapping_add(1)`, so a magnitude compare picks the OLD page
/// across the one wrap the counter has: at `65535 -> 0`, `0 >= 65535` is false. The compare is by
/// wrapping distance instead: side 1 is newer when `seq1 - seq0` (wrapping) lands in the low half of
/// the range, so `0` beats `65535` while `5` still beats `4`.
///
/// Only one pair can actually arise from this format, the pair compaction writes: the two sides
/// differ by exactly one. The rule is still total, and these are the cases outside that pair:
/// - **equal sequences** (`distance 0`): side 1 wins, which is what the magnitude compare did;
/// - **distance exactly 32768**, the antipodal pair where neither side is nearer: side 0 wins, the
///   arbitrary half of a tie that the `<` makes, named here so it is defined rather than incidental;
/// - **any larger backwards distance** (side 0 ahead by 2..=32767): side 0 wins, the mirror of the
///   forward case.
///
/// A region holding either of the first two came from something other than this code (a planted or
/// corrupt image), and both choices mount a structurally valid side.
fn side1_is_newer(seq0: u16, seq1: u16) -> bool {
    seq1.wrapping_sub(seq0) < 0x8000
}

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
    let seq = u16::from_le_bytes([region[off + 4], region[off + 5]]);
    // The commit hinge: `seq` is the last halfword the store programs, so a page whose magic landed
    // and whose `seq` did not is a TORN compaction, not a valid side. `compact` never issues
    // `SEQ_UNCOMMITTED` as a real sequence number, so this cannot reject a committed page.
    if seq == record::SEQ_UNCOMMITTED {
        return None;
    }
    Some(seq)
}

/// Is the page at `off..off+page_size` entirely erased (`0xFF`)?
fn page_is_erased(region: &[u8], off: usize, page_size: usize) -> bool {
    region[off..off + page_size].iter().all(|&b| b == 0xFF)
}

enum ScanResult {
    /// The frontier (first blank slot / clean end of log).
    Frontier(usize),
    /// A torn header was hit at this offset: the log is not walkable past here, mount must
    /// auto-compact. The offset is the end of the walkable chain, which is what compaction packs
    /// up to (it carries the same meaning as `Frontier`'s, so a caller that only needs the end of
    /// the chain takes either).
    TornHeader(usize),
}

/// Walk the active page from its first record; stop at a blank (the frontier) or a torn header.
fn scan_frontier(region: &[u8], page_off: usize, page_size: usize) -> ScanResult {
    let page_end = page_off + page_size;
    let mut cursor = page_off + PAGE_HEADER_LEN;
    loop {
        match record::parse_header(region, cursor) {
            HeaderScan::Blank => return ScanResult::Frontier(cursor),
            HeaderScan::Torn => return ScanResult::TornHeader(cursor),
            HeaderScan::Good(h) => {
                let next = cursor + record::record_size(h.len as usize);
                // A good header whose record would run past the page end is a corrupt len despite a
                // matching hdr_crc (cannot happen on a sane region); treat as torn to be safe.
                if next > page_end {
                    return ScanResult::TornHeader(cursor);
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
///
/// `start..end` is a span of the chain the caller has ALREADY walked header by header with
/// [`record::parse_header`] (compaction does that once, before it copies anything), so every
/// `hdr_crc` in it is known good and every `len` trustworthy. That is why the hop reads the header
/// fields with [`record::header_unchecked`]: this scan runs once per source record, so re-CRCing
/// each header on every hop made the repack quadratic in CRC16 and pushed a full page outside the
/// firmware's watchdog window (`specs/storage-layer.md`, "Compaction and the watchdog window").
/// `is_committed` still CRCs, but only for a header whose key matches, which ends the scan.
fn has_later_record(region: &[u8], start: usize, end: usize, key: Key) -> bool {
    let mut cursor = start;
    while cursor < end {
        let Some(h) = record::header_unchecked(region, cursor) else {
            break;
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

// --- decoding what a single pass handed back -----------------------------------------------

/// Decode one record's `(tag, bytes)` as `field`'s scalar, or `field`'s registered default.
///
/// [`Store::get`]'s own type-and-width rule, lifted out so a caller resolving a row through
/// [`Store::for_each_in`] applies the identical one: a wrong-type or wrong-width record reads the
/// default, exactly as it does through `get`.
pub fn scalar_or_default<T: Scalar>(field: Field<T>, tag: u8, bytes: &[u8]) -> T {
    if tag == T::KIND.tag() && bytes.len() == T::WIDTH {
        T::read_le(bytes)
    } else {
        field.default()
    }
}
