//! The crafted store-region builders for the host-planted Tier-2/Tier-3 scenarios, shared verbatim by
//! the emulator runner (Unicorn) and the hardware runner (silicon over probe-rs).
//!
//! The torn-write / `Full` / compaction scenarios cannot tear a real FMC write on cue, so the **host**
//! plants a hand-built store-region byte image into the device before the read phase; the device then
//! only cold-mounts and reports recovery. A torn record is just a normal program sequence stopped
//! early (a complete header with the value/`val_crc` omitted = torn payload, `hdr_crc` good; or a
//! corrupted covered header byte = torn header, `hdr_crc` bad), so a crafted region is deterministic
//! and **identical across mock, Unicorn, and silicon**.
//!
//! Byte-identity is by construction, not just by convention: every byte these builders emit comes from
//! the store's own codec ([`record::encode`] / [`record::encode_page_header`]), and BOTH tiers call
//! *these same functions*. The emulator's `StoreEmu::load_region` and the hardware driver's
//! `load_region` write the produced bytes into flash unchanged, so a planted record is byte-identical
//! to what the firmware itself would write, and the two tiers plant the exact same region.
//!
//! `no_std` and alloc-free: each builder writes into a caller-supplied `&mut [u8]` sized to the region
//! (`2 * page_size`), so this compiles for the chip too (it is gated behind `test-fields` and is not in
//! a production build). The longest region these scenarios need is two 2 KiB pages (4096 bytes); the
//! callers size their buffer to the active part's `2 * page_size`.

use crate::field::{T_KEY, T_VAL};
use crate::key::{Key, Type};
use crate::record;

/// A crafted store-region image builder over a caller-supplied buffer. `buf.len()` must be exactly the
/// region length (`2 * page_size`); the buffer starts all-`0xFF` (erased) and the builder programs
/// records / page headers into it via the store's own codec.
pub struct Region<'a> {
    page_size: usize,
    bytes: &'a mut [u8],
}

impl<'a> Region<'a> {
    /// Wrap `buf` as a two-page erased region. `buf.len()` must equal `2 * page_size`; the buffer is
    /// filled with `0xFF` (the erased NOR state the store's virgin-region logic expects).
    pub fn new(buf: &'a mut [u8], page_size: usize) -> Self {
        assert_eq!(
            buf.len(),
            2 * page_size,
            "region buffer must be exactly two pages"
        );
        buf.fill(0xFF);
        Self {
            page_size,
            bytes: buf,
        }
    }

    /// Write page `p`'s full header (`magic` + `seq`) via the store's own encoder, so it is
    /// byte-identical to what the firmware writes for a valid side.
    pub fn page_header(&mut self, p: usize, seq: u16) -> &mut Self {
        let off = p * self.page_size;
        let hdr = record::encode_page_header(seq);
        self.bytes[off..off + hdr.len()].copy_from_slice(&hdr);
        self
    }

    /// Append a complete record (via the store codec) at absolute `off`; returns the next offset.
    pub fn record(&mut self, off: usize, key: Key, type_tag: u8, value: &[u8]) -> usize {
        // The longest planted value here is the 200-byte FULL filler; a 256-byte scratch covers it.
        let mut tmp = [0u8; 256 + record::HEADER_LEN + record::VAL_CRC_LEN];
        let n = record::encode(key.field_id, key.index, type_tag, value, &mut tmp);
        self.bytes[off..off + n].copy_from_slice(&tmp[..n]);
        off + n
    }

    /// Overwrite `len` bytes at absolute `off` (used to corrupt a covered byte / value byte to forge a
    /// torn record after a normal `encode`).
    pub fn poke(&mut self, off: usize, bytes: &[u8]) {
        self.bytes[off..off + bytes.len()].copy_from_slice(bytes);
    }

    /// The built region bytes (length `2 * page_size`).
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes
    }
}

// ---------------------------------------------------------------------------
// The planted scenarios. Each fills `buf` (sized `2 * page_size`) with the crafted region the
// corresponding `store-test` scenario expects, using the store codec, so the emulator and the hardware
// driver plant the byte-identical region. The device then only cold-mounts and reports recovery.
// ---------------------------------------------------------------------------

/// COMPACT: a multi-record region (known [`T_KEY`] + a couple of unknown keys), with an OLDER `T_KEY`
/// value superseded by [`T_VAL`], so the latest-per-key survivor the device reads is `T_VAL`.
pub fn build_compact(buf: &mut [u8], page_size: usize) {
    let mut region = Region::new(buf, page_size);
    region.page_header(0, 0);
    let unknown_a = Key {
        field_id: 0x71,
        index: 0,
    };
    let unknown_b = Key {
        field_id: 0x72,
        index: 1,
    };
    let mut off = region.record(
        record::PAGE_HEADER_LEN,
        unknown_a,
        Type::U16.tag(),
        &0xBEEFu16.to_le_bytes(),
    );
    off = region.record(
        off,
        T_KEY.key(),
        Type::U32.tag(),
        &0x1111_2222u32.to_le_bytes(),
    ); // older
    off = region.record(
        off,
        unknown_b,
        Type::U32.tag(),
        &0xCAFE_F00Du32.to_le_bytes(),
    );
    let _ = region.record(off, T_KEY.key(), Type::U32.tag(), &T_VAL.to_le_bytes());
    // newest wins
}

/// TORN_PAYLOAD: a good `T_KEY = T_VAL` record, then a half-written *payload* of a newer value
/// (`hdr_crc` good, `val_crc` corrupt). Mount skips the torn record; the last good value (`T_VAL`)
/// reads, no erase.
pub fn build_torn_payload(buf: &mut [u8], page_size: usize) {
    let mut region = Region::new(buf, page_size);
    region.page_header(0, 0);
    let off = region.record(
        record::PAGE_HEADER_LEN,
        T_KEY.key(),
        Type::U32.tag(),
        &T_VAL.to_le_bytes(),
    );
    // Encode a newer value then corrupt the first value byte so val_crc fails (the header is intact,
    // so hdr_crc stays good and the record is still walkable).
    let mut torn = [0u8; record::record_size(4)];
    record::encode(
        T_KEY.key().field_id,
        T_KEY.key().index,
        Type::U32.tag(),
        &0xDEAD_BEEFu32.to_le_bytes(),
        &mut torn,
    );
    torn[record::HEADER_LEN] ^= 0xFF; // corrupt the first value byte
    region.poke(off, &torn);
}

/// TORN_HEADER: a good `T_KEY = T_VAL` record, then a torn *header* (`hdr_crc` bad). Mount
/// auto-compacts; the survivor reads back and the frontier is clean.
pub fn build_torn_header(buf: &mut [u8], page_size: usize) {
    let mut region = Region::new(buf, page_size);
    region.page_header(0, 3);
    let off = region.record(
        record::PAGE_HEADER_LEN,
        T_KEY.key(),
        Type::U32.tag(),
        &T_VAL.to_le_bytes(),
    );
    let mut torn = [0u8; record::record_size(4)];
    record::encode(
        T_KEY.key().field_id,
        T_KEY.key().index,
        Type::U32.tag(),
        &0x5u32.to_le_bytes(),
        &mut torn,
    );
    torn[4] ^= 0xFF; // corrupt a covered header byte -> hdr_crc fails
    region.poke(off, &torn);
}

/// FULL: a near-full active page of a large unknown blob, with NO `T_KEY` record yet. The device's
/// `set(T_KEY)` returns `Full` -> `compact()` -> retry succeeds; phase 1 reads back `T_VAL`.
pub fn build_full(buf: &mut [u8], page_size: usize) {
    let mut region = Region::new(buf, page_size);
    region.page_header(0, 0);
    let filler = Key {
        field_id: 0x73,
        index: 0,
    };
    let payload = [0xAAu8; 200];
    let mut off = record::PAGE_HEADER_LEN;
    // Fill until another 200-byte record would not fit the page, then top up so the remaining free
    // space is smaller than a small T_KEY record (forcing Full on the device-side set).
    loop {
        let next = off + record::record_size(payload.len());
        if next + record::record_size(4) > page_size {
            break;
        }
        off = region.record(off, filler, Type::Blob.tag(), &payload);
    }
    if off + record::record_size(payload.len()) <= page_size {
        let _ = region.record(off, filler, Type::Blob.tag(), &payload);
    }
}

/// Build the crafted region for a planted scenario id into `buf` (sized `2 * page_size`). Returns
/// `false` for a scenario that is not host-planted (PERSIST / VAR_VALUE are device-driven and plant
/// nothing), so a caller can plant only when needed.
pub fn build_planted_region(scenario: u32, buf: &mut [u8], page_size: usize) -> bool {
    use crate::field::{COMPACT, FULL, TORN_HEADER, TORN_PAYLOAD};
    match scenario {
        COMPACT => build_compact(buf, page_size),
        TORN_PAYLOAD => build_torn_payload(buf, page_size),
        TORN_HEADER => build_torn_header(buf, page_size),
        FULL => build_full(buf, page_size),
        _ => return false,
    }
    true
}
