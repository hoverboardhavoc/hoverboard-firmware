//! The log-structured store: ping-pong region, scan, append, compaction, the RAM live-table, and the
//! [`Store`] API.
//!
//! Two pages, ping-pong: one detected page per side, each `[ magic:u32 | seq:u16 | reserved:u16 ]`;
//! the active side is the higher `seq`. The store is built generic over [`Flash`] so all of this is
//! host-tested against [`crate::flash::MockFlash`]; the on-target [`FmcFlash`] (in `store-test`) is the
//! same trait over the HAL's `Fmc`.
//!
//! [`Store`] borrows `flash` per call (it does not own the handle), so `mount(flash)` then
//! `persist(flash)` both pass it.

use base::error::FlashError;

use crate::flash::Flash;
use crate::params::{Key, Type, Value};
use crate::record::{self, HeaderScan, HEADER_LEN, MAX_RECORD_LEN, MAX_VAL_LEN, VAL_CRC_LEN};
use crate::registry::{self, Persist};

/// The page magic marking an initialized ping-pong side (`b"HVST"` little-endian).
const PAGE_MAGIC: u32 = 0x5453_5648; // "HVST"
/// The per-page header length: `magic:u32 | seq:u16 | reserved:u16`.
const PAGE_HEADER_LEN: usize = 8;

/// Max distinct fixed-width keys the RAM live-table caches. The registry is a small curated set and
/// each field has a bounded instance count; this is over-provisioned. A variable-length (`STR`/`BLOB`)
/// value is NOT cached (left in flash, read on demand), so it never consumes a slot.
const MAX_LIVE_KEYS: usize = 32;

/// One cached fixed-width live value: the key, the storage type, the value as a `u64` payload (every
/// fixed-width type fits in 8 bytes), and the dirty flag (set by `set_live`, cleared by `persist`).
#[derive(Clone, Copy)]
struct LiveEntry {
    key: Key,
    kind: Type,
    payload: u64,
    dirty: bool,
}

/// The mounted store: the active side, its frontier (next free offset within the active page), the
/// next `seq` to write on compaction, and the RAM live-table. Borrows the flash handle per call.
pub struct Store {
    page_size: usize,
    /// The active ping-pong side (0 or 1).
    active: usize,
    /// The active side's `seq`.
    seq: u16,
    /// The next free offset WITHIN the active page (region-relative once `page_base(active)` is added).
    frontier: usize,
    live: [Option<LiveEntry>; MAX_LIVE_KEYS],
}

impl Store {
    /// Mount the store: pick the active side (higher `seq`, or initialize side 0 if neither is valid),
    /// replay its log into the RAM live-table (latest-per-key wins, absent keys from defaults), and
    /// find the frontier. This is the cold mount the `run_phase` "reboot" uses.
    pub fn mount<F: Flash>(flash: &mut F) -> Store {
        let page_size = flash.page_size();
        let (active, seq) = pick_active(flash, page_size);

        let mut store = Store {
            page_size,
            active,
            seq,
            frontier: PAGE_HEADER_LEN,
            live: [None; MAX_LIVE_KEYS],
        };

        // If the active side is uninitialized (no magic on either side), format side 0.
        if !store.side_initialized(flash, active) {
            store.format_side(flash, 0, 1).ok();
            store.active = 0;
            store.seq = 1;
        }

        store.replay(flash);
        store
    }

    /// The region-relative base offset of ping-pong side `side` (0 -> 0, 1 -> `page_size`).
    #[inline]
    fn page_base(&self, side: usize) -> usize {
        side * self.page_size
    }

    /// True iff `side`'s page header carries the store magic.
    fn side_initialized<F: Flash>(&self, flash: &F, side: usize) -> bool {
        side_magic(flash, side * self.page_size) == PAGE_MAGIC
    }

    /// Format `side`: erase its page and write `[ magic | seq ]` at its head. Leaves the rest erased
    /// (records append after the header).
    fn format_side<F: Flash>(
        &self,
        flash: &mut F,
        side: usize,
        seq: u16,
    ) -> Result<(), FlashError> {
        flash.erase_page(side)?;
        let mut hdr = [0xFFu8; PAGE_HEADER_LEN];
        hdr[0..4].copy_from_slice(&PAGE_MAGIC.to_le_bytes());
        hdr[4..6].copy_from_slice(&seq.to_le_bytes());
        // reserved (bytes 6..8) stays 0xFFFF.
        flash.program(side * self.page_size, &hdr)
    }

    /// Replay the active side's log into the RAM live-table: latest-per-key wins for fixed-width keys,
    /// then fill any absent registry default. Variable-length values are left in flash (not cached).
    /// Also sets `self.frontier` to the first blank/torn offset.
    fn replay<F: Flash>(&mut self, flash: &mut F) {
        self.live = [None; MAX_LIVE_KEYS];
        let base = self.page_base(self.active);
        let mut off = PAGE_HEADER_LEN;

        loop {
            // A header must fully fit in the page; otherwise the page is full (frontier = here).
            if off + HEADER_LEN > self.page_size {
                break;
            }
            let mut hdr = [0u8; HEADER_LEN];
            if flash.read(base + off, &mut hdr).is_err() {
                break;
            }
            match record::classify_header(&hdr) {
                HeaderScan::Blank | HeaderScan::Torn => break, // the frontier / end of log
                HeaderScan::Valid(h) => {
                    let span = record::record_span(h.len as usize);
                    if off + span > self.page_size {
                        break; // a header claiming a value past the page end: stop at the frontier.
                    }
                    // Read the value bytes (the len payload) and its committing val_crc.
                    let mut val = [0u8; MAX_VAL_LEN];
                    let vlen = h.len as usize;
                    let val_off = base + off + HEADER_LEN;
                    let crc_off = base + off + HEADER_LEN + record::padded_len(vlen);
                    let mut crc_bytes = [0u8; VAL_CRC_LEN];
                    let value_ok = flash.read(val_off, &mut val[..vlen]).is_ok()
                        && flash.read(crc_off, &mut crc_bytes).is_ok()
                        && record::value_committed(&val[..vlen], u16::from_le_bytes(crc_bytes));
                    if value_ok {
                        // Cache fixed-width keys; skip variable-length ones (read on demand). An
                        // unknown type byte is still walkable but not cached.
                        if let Some(kind) = Type::from_u8(h.type_byte) {
                            if !kind.is_variable() {
                                if let Some(v) = record::decode_fixed(kind, &val[..vlen]) {
                                    self.cache_put(Key::new(h.field_id, h.index), kind, &v);
                                }
                            }
                        }
                    }
                    // Whether or not the value committed, the header's trusted len lets us hop on (a
                    // torn VALUE leaves a good header, so we skip its span; the val_crc gate already
                    // discarded the value above).
                    off += span;
                }
            }
        }
        self.frontier = off;
    }

    /// Store `value` into the live cache under `key` (insert or overwrite), NOT dirty. Used by replay.
    fn cache_put(&mut self, key: Key, kind: Type, value: &Value) {
        let payload = fixed_payload(value);
        if let Some(slot) = self.live.iter_mut().flatten().find(|e| e.key == key) {
            slot.kind = kind;
            slot.payload = payload;
            slot.dirty = false;
            return;
        }
        if let Some(empty) = self.live.iter_mut().find(|e| e.is_none()) {
            *empty = Some(LiveEntry {
                key,
                kind,
                payload,
                dirty: false,
            });
        }
        // If full, the key is dropped from the cache (it still reads its default; the registry is
        // sized so this does not happen for real fields).
    }

    /// Read a key: the RAM cache if present, else the registry default. Missing -> default.
    pub fn get(&self, key: Key) -> Value<'static> {
        if let Some(e) = self.live.iter().flatten().find(|e| e.key == key) {
            return value_from_payload(e.kind, e.payload);
        }
        // No cached value: the registry default (or U32(0) for an unknown field, which never happens
        // for a real key, only a stray test probe).
        match registry::lookup(key.field_id) {
            Some(d) => d.default,
            None => Value::U32(0),
        }
    }

    /// Write a fixed-width key live: update the RAM cache and mark it dirty (apply-live, no flash,
    /// safe while armed). A variable-length write is not handled here (it is a disarmed-only direct
    /// append, out of scope for the live table); calling with [`Value::Bytes`] is a no-op.
    pub fn set_live(&mut self, key: Key, value: Value) {
        if matches!(value, Value::Bytes(_)) {
            return;
        }
        let kind = value.kind();
        let payload = fixed_payload(&value);
        if let Some(slot) = self.live.iter_mut().flatten().find(|e| e.key == key) {
            slot.kind = kind;
            slot.payload = payload;
            slot.dirty = true;
            return;
        }
        if let Some(empty) = self.live.iter_mut().find(|e| e.is_none()) {
            *empty = Some(LiveEntry {
                key,
                kind,
                payload,
                dirty: true,
            });
        }
    }

    /// Flush dirty `Persistent` keys to flash (disarmed only). Each dirty key is appended as one new
    /// record at the frontier (write-once log append). `Volatile` keys are never written. On a full
    /// page, compact first (one spare-page rewrite) then retry the append.
    pub fn persist<F: Flash>(&mut self, flash: &mut F) -> Result<(), FlashError> {
        // Collect the dirty Persistent keys to write (index into the live array), so the borrow of
        // self.live does not conflict with the append (which mutates frontier/seq).
        for i in 0..self.live.len() {
            let entry = match self.live[i] {
                Some(e) if e.dirty => e,
                _ => continue,
            };
            // Volatile keys are never written; an unknown field is not persisted.
            match registry::lookup(entry.key.field_id) {
                Some(d) if d.persist == Persist::Persistent => {}
                _ => {
                    // Volatile (or unknown): clear dirty without writing.
                    if let Some(slot) = self.live[i].as_mut() {
                        slot.dirty = false;
                    }
                    continue;
                }
            }
            let value = value_from_payload(entry.kind, entry.payload);
            self.append(flash, entry.key, entry.kind, &value)?;
            if let Some(slot) = self.live[i].as_mut() {
                slot.dirty = false;
            }
        }
        Ok(())
    }

    /// Append one fixed-width record to the active frontier (write-once), compacting first if the page
    /// is full. Updates `self.frontier`.
    fn append<F: Flash>(
        &mut self,
        flash: &mut F,
        key: Key,
        kind: Type,
        value: &Value,
    ) -> Result<(), FlashError> {
        let mut vbuf = [0u8; 8];
        let vlen = record::encode_fixed(value, &mut vbuf);
        let span = record::record_span(vlen);

        // If it does not fit in the active page, compact (rewrite latest-per-key into the spare page),
        // then the append goes into the freshly compacted page.
        if PAGE_HEADER_LEN > self.page_size || self.frontier + span > self.page_size {
            self.compact(flash)?;
        }

        let mut rbuf = [0u8; MAX_RECORD_LEN];
        let n = record::build(key.field_id, key.index, kind, &vbuf[..vlen], &mut rbuf);
        debug_assert_eq!(n, span);
        let base = self.page_base(self.active);
        flash.program(base + self.frontier, &rbuf[..n])?;
        self.frontier += n;
        Ok(())
    }

    /// Compaction (the only erase): write the latest record PER KEY (key-agnostic, preserving every
    /// key, known or not), packed, into the spare erased page with a higher `seq`, then erase the old
    /// page. Power-safe: the new page is complete before the old is erased, and `mount` picks the
    /// higher `seq`. After this the spare page is the active one with the live records compacted.
    pub fn compact<F: Flash>(&mut self, flash: &mut F) -> Result<(), FlashError> {
        let spare = 1 - self.active;
        let new_seq = self.seq.wrapping_add(1);
        self.format_side(flash, spare, new_seq)?;

        // Walk the OLD active page, keeping the latest record per (field_id, index). To stay
        // key-agnostic (preserve keys this build does not know) we copy records by their raw bytes.
        // We do a two-pass dedup: for each record offset, only copy it if no LATER record in the page
        // has the same key (the latest wins).
        let old_base = self.page_base(self.active);
        let new_base = self.page_base(spare);
        let mut new_off = PAGE_HEADER_LEN;

        let mut off = PAGE_HEADER_LEN;
        loop {
            if off + HEADER_LEN > self.page_size {
                break;
            }
            let mut hdr = [0u8; HEADER_LEN];
            if flash.read(old_base + off, &mut hdr).is_err() {
                break;
            }
            let h = match record::classify_header(&hdr) {
                HeaderScan::Valid(h) => h,
                _ => break,
            };
            let span = record::record_span(h.len as usize);
            if off + span > self.page_size {
                break;
            }
            // Is this the latest record for its key? Scan forward for a later same-key committed record.
            if self.is_latest_for_key(flash, old_base, off, h.field_id, h.index) {
                // Copy the whole record (header + padded value + val_crc) verbatim, key-agnostic.
                let mut rec = [0u8; MAX_RECORD_LEN];
                if flash.read(old_base + off, &mut rec[..span]).is_ok() {
                    // Only copy a committed record (a torn value is dropped on compaction).
                    let vlen = h.len as usize;
                    let crc_off = HEADER_LEN + record::padded_len(vlen);
                    let stored = u16::from_le_bytes([rec[crc_off], rec[crc_off + 1]]);
                    if record::value_committed(&rec[HEADER_LEN..HEADER_LEN + vlen], stored)
                        && new_off + span <= self.page_size
                    {
                        flash.program(new_base + new_off, &rec[..span])?;
                        new_off += span;
                    }
                }
            }
            off += span;
        }

        // Erase the old page LAST (the new page is already complete + has the higher seq).
        flash.erase_page(self.active)?;

        self.active = spare;
        self.seq = new_seq;
        self.frontier = new_off;
        Ok(())
    }

    /// True iff the record at `off` is the latest committed record for `(field_id, index)` in the page
    /// (no later same-key committed record exists). Used by compaction to keep only the newest.
    fn is_latest_for_key<F: Flash>(
        &self,
        flash: &F,
        base: usize,
        off: usize,
        field_id: u8,
        index: u8,
    ) -> bool {
        let mut cur = off;
        // Advance past `off` first.
        {
            let mut hdr = [0u8; HEADER_LEN];
            if flash.read(base + cur, &mut hdr).is_err() {
                return true;
            }
            let span = match record::classify_header(&hdr) {
                HeaderScan::Valid(h) => record::record_span(h.len as usize),
                _ => return true,
            };
            cur += span;
        }
        loop {
            if cur + HEADER_LEN > self.page_size {
                return true;
            }
            let mut hdr = [0u8; HEADER_LEN];
            if flash.read(base + cur, &mut hdr).is_err() {
                return true;
            }
            let h = match record::classify_header(&hdr) {
                HeaderScan::Valid(h) => h,
                _ => return true, // frontier reached, no later same-key record
            };
            let span = record::record_span(h.len as usize);
            if cur + span > self.page_size {
                return true;
            }
            if h.field_id == field_id && h.index == index {
                // A later record with the same key: check it is committed (a torn later record does
                // not supersede). If committed, this earlier record is NOT the latest.
                let vlen = h.len as usize;
                let mut val = [0u8; MAX_VAL_LEN];
                let crc_off = base + cur + HEADER_LEN + record::padded_len(vlen);
                let mut crc_bytes = [0u8; VAL_CRC_LEN];
                let committed = flash
                    .read(base + cur + HEADER_LEN, &mut val[..vlen])
                    .is_ok()
                    && flash.read(crc_off, &mut crc_bytes).is_ok()
                    && record::value_committed(&val[..vlen], u16::from_le_bytes(crc_bytes));
                if committed {
                    return false;
                }
            }
            cur += span;
        }
    }
}

/// The reserved store-test field key: a `Persistent` `U32` field. `T_KEY` is `field_id = 0xFE`,
/// `index = 0`.
pub const T_KEY: Key = Key {
    field_id: registry::T_FIELD_ID,
    index: 0,
};
/// The store-test value the pass image sets and persists; the host recomputes it to assert.
pub const T_VAL: u32 = 0x00C0_FFEE;

/// One function drives all three tiers: it cold-mounts (the "reboot": the RAM table is discarded and
/// rebuilt from flash, so a surviving value is provably from flash, not RAM) and does one step.
///
/// phase 0 = set + persist; phase 1 (any other) = cold-mount + read. Returns the read-back (0 on the
/// write phase). The device publishes the read-back as `TestResult.output`; the host computes the
/// expected (`T_VAL`) and asserts.
pub fn run_phase<F: Flash>(flash: &mut F, phase: u32) -> u32 {
    let mut store = Store::mount(flash); // cold mount = the "reboot"
    match phase {
        0 => {
            store.set_live(T_KEY, Value::U32(T_VAL));
            let _ = store.persist(flash);
            0
        }
        _ => store.get(T_KEY).u32(),
    }
}

// --- helpers ----------------------------------------------------------------------------------

/// Read a side's 4-byte magic at region offset `base`.
fn side_magic<F: Flash>(flash: &F, base: usize) -> u32 {
    let mut m = [0u8; 4];
    if flash.read(base, &mut m).is_err() {
        return 0;
    }
    u32::from_le_bytes(m)
}

/// Read a side's `seq` at region offset `base + 4`.
fn side_seq<F: Flash>(flash: &F, base: usize) -> u16 {
    let mut s = [0u8; 2];
    if flash.read(base + 4, &mut s).is_err() {
        return 0;
    }
    u16::from_le_bytes(s)
}

/// Pick the active ping-pong side (the initialized side with the higher `seq`), returning
/// `(side, seq)`. If neither side is initialized, returns `(0, 0)` (mount then formats side 0).
fn pick_active<F: Flash>(flash: &F, page_size: usize) -> (usize, u16) {
    let a0 = side_magic(flash, 0) == PAGE_MAGIC;
    let a1 = side_magic(flash, page_size) == PAGE_MAGIC;
    match (a0, a1) {
        (false, false) => (0, 0),
        (true, false) => (0, side_seq(flash, 0)),
        (false, true) => (1, side_seq(flash, page_size)),
        (true, true) => {
            let s0 = side_seq(flash, 0);
            let s1 = side_seq(flash, page_size);
            // Higher seq wins (the power-loss-mid-compaction tie-break: the completed new page).
            if s1 > s0 {
                (1, s1)
            } else {
                (0, s0)
            }
        }
    }
}

/// Pack a fixed-width [`Value`] into a `u64` payload (zero-extended / sign-preserving as appropriate).
fn fixed_payload(value: &Value) -> u64 {
    match value {
        Value::U8(v) => *v as u64,
        Value::Bool(v) => *v as u64,
        Value::U16(v) => *v as u64,
        Value::U32(v) => *v as u64,
        Value::U64(v) => *v,
        Value::I16(v) => *v as u16 as u64,
        Value::I32(v) => *v as u32 as u64,
        Value::I64(v) => *v as u64,
        Value::Bytes(_) => 0,
    }
}

/// Reconstruct a fixed-width [`Value`] of `kind` from a `u64` payload.
fn value_from_payload(kind: Type, payload: u64) -> Value<'static> {
    match kind {
        Type::U8 => Value::U8(payload as u8),
        Type::Bool => Value::Bool(payload != 0),
        Type::U16 => Value::U16(payload as u16),
        Type::U32 => Value::U32(payload as u32),
        Type::U64 => Value::U64(payload),
        Type::I16 => Value::I16(payload as u16 as i16),
        Type::I32 => Value::I32(payload as u32 as i32),
        Type::I64 => Value::I64(payload as i64),
        // Variable types are never in the live cache; reaching here is a logic error, default to 0.
        Type::Blob | Type::Str => Value::U32(0),
    }
}
