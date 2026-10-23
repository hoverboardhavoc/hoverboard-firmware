//! The record format and codec.
//!
//! ```text
//! [ field_id:u8 | index:u8 | type:u8 | reserved:u8 | len:u16 | hdr_crc:u16 | value[len] (+pad to even) | val_crc:u16 ]
//!    \___________________________ 8-byte header ___________________________/   \_____ value _____/
//!    field_id == 0xFF  =>  blank (erased 0xFFFF)  =>  end of log (frontier)
//! ```
//!
//! - `hdr_crc` covers the first 6 bytes (`field_id`+`index`+`type`+`reserved`+`len`), so a good
//!   `hdr_crc` means `len` is trustworthy and the variable-length log stays walkable even past a torn
//!   payload.
//! - `reserved` is a zero pad keeping the header even (8 bytes) so the value starts halfword-aligned;
//!   it is covered by `hdr_crc`.
//! - An odd-`len` value gets **one `0xFF` pad byte** so `val_crc` (and the next record) stay
//!   halfword-aligned. The walk hops `next = here + 8 + (len + (len & 1)) + 2`.
//! - `val_crc` is the **commit marker**: it is the highest (last-written) halfword, so a record is
//!   valid only when complete. It is computed over the header bytes followed by the value bytes
//!   (without the pad), so a torn payload fails it and never wins a read.

use base::crc16::Crc16;

/// The page-header `magic`: a valid page side carries this in full (`"STOR"`, little-endian). The
/// single source of truth: the store core writes it, [`encode_page_header`] emits it, and the test
/// region builders reference it (so a planted page header is byte-identical to what the store writes).
pub const MAGIC: u32 = 0x5354_4F52;
/// The page-header size in bytes: `[ magic:u32 | seq:u16 | tail:u16 ]`, even so the first record
/// stays halfword-aligned. Only the first [`PAGE_HEADER_PROGRAMMED`] bytes are ever programmed.
pub const PAGE_HEADER_LEN: usize = 8;

/// How much of the page header the store PROGRAMS: `magic` + `seq`, the first 6 bytes. The trailing
/// halfword (`hdr[6..8]`) is never written and stays erased.
///
/// It is not a spare field, it is unreachable. The first record is programmed at
/// [`PAGE_HEADER_LEN`], and on the GD32F1x0 programming the low halfword of a word CLOSES the high
/// halfword of the word below it (see `runtime_hal::fmc`, "A high halfword is closed by its
/// successor"), so the moment the first record lands, `hdr[6..8]` can never be programmed again.
pub const PAGE_HEADER_PROGRAMMED: usize = 6;

/// The `seq` value of an UNCOMMITTED page: the erased halfword. A page side counts as valid only
/// once its `seq` is something else, which is what makes `seq` the commit hinge (see
/// [`encode_page_header`]).
pub const SEQ_UNCOMMITTED: u16 = 0xFFFF;

/// Encode the PROGRAMMED part of a page header for `seq` (`[ magic:u32 | seq:u16 ]`), the exact
/// bytes the store writes for a valid page side, in the order it writes them.
///
/// **`seq` is the commit hinge, and it is last because it is highest.** The store programs this
/// buffer low-to-high: `magic` first, `seq` last. A page whose magic landed but whose `seq` did not
/// reads `seq == `[`SEQ_UNCOMMITTED`], which the mount rejects, so a compaction torn anywhere before
/// its final halfword leaves the OLD page as the only valid side. The hinge cannot be the magic (the
/// older design, where `magic` was programmed after `seq`): that is a descending write within the
/// page, which the F1x0 refuses outright (`specs/bench-evidence/2026-10-23/store-compact-commit/`).
pub fn encode_page_header(seq: u16) -> [u8; PAGE_HEADER_PROGRAMMED] {
    let mut hdr = [0u8; PAGE_HEADER_PROGRAMMED];
    hdr[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    hdr[4..6].copy_from_slice(&seq.to_le_bytes());
    hdr
}

/// The fixed record header size in bytes (the 6 covered bytes plus the 2-byte `hdr_crc`).
pub const HEADER_LEN: usize = 8;
/// The `val_crc` commit-marker size in bytes.
pub const VAL_CRC_LEN: usize = 2;
/// Bytes the `hdr_crc` covers: `field_id`+`index`+`type`+`reserved`+`len:u16`.
const HDR_COVERED: usize = 6;

/// `len` rounded up to even (the padded value span, what the walk hops over).
#[inline]
pub const fn padded_len(len: usize) -> usize {
    len + (len & 1)
}

/// Total on-flash size of a record carrying a `len`-byte value (header + padded value + val_crc).
#[inline]
pub const fn record_size(len: usize) -> usize {
    HEADER_LEN + padded_len(len) + VAL_CRC_LEN
}

/// The decoded view of a record header (after a good `hdr_crc`).
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub field_id: u8,
    pub index: u8,
    pub type_tag: u8,
    pub len: u16,
}

/// What a header parse at a given offset found.
pub enum HeaderScan {
    /// A blank (`field_id == 0xFF`) slot: the frontier (end of log).
    Blank,
    /// A header whose `hdr_crc` matched: `len` is trustworthy, the walk can hop it.
    Good(Header),
    /// A header whose `hdr_crc` failed: a torn header, `len` is garbage, the log is no longer
    /// walkable past here.
    Torn,
}

/// Parse the 8-byte header at `region[off..]`. Returns [`HeaderScan::Blank`] at the frontier,
/// `Good` with a trustworthy `len`, or `Torn` on an `hdr_crc` failure. Returns `Torn` if the header
/// would run off the region end (an `hdr_crc` cannot be read, so it is not walkable).
pub fn parse_header(region: &[u8], off: usize) -> HeaderScan {
    // Fixed-size, so every index below is provably in range and the walk carries no bounds
    // check. That is not style: this runs on the 250 Hz control task's arm-time row read, which
    // `specs/panic-free.md` requires to be panic-free, and a slice index is a panic site the
    // compiler can only elide when it can see the length (it could while this was inlined into
    // the monomorphised `get`, and could not once the single-pass row read called it out of line).
    let Some(h) = region.get(off..off + HEADER_LEN) else {
        return HeaderScan::Torn;
    };
    let Ok(h) = <&[u8; HEADER_LEN]>::try_from(h) else {
        return HeaderScan::Torn;
    };
    if h[0] == crate::key::BLANK_FIELD_ID {
        return HeaderScan::Blank;
    }
    let stored = u16::from_le_bytes([h[6], h[7]]);
    if base::crc16::modbus(&h[..HDR_COVERED]) != stored {
        return HeaderScan::Torn;
    }
    HeaderScan::Good(Header {
        field_id: h[0],
        index: h[1],
        type_tag: h[2],
        len: u16::from_le_bytes([h[4], h[5]]),
    })
}

/// Is the record at `off` (header already known-good) committed? True when its `val_crc` matches the
/// CRC over the header's 6 covered bytes plus the value bytes (a torn payload fails this).
pub fn is_committed(region: &[u8], off: usize, h: &Header) -> bool {
    let len = h.len as usize;
    let val_crc_off = off + HEADER_LEN + padded_len(len);
    // Every span taken through `get`, for the panic-free reason in `parse_header`. A record that
    // does not fit the region is not committed, which is the same answer the length check gave.
    let (Some(crc), Some(covered), Some(value)) = (
        region
            .get(val_crc_off..val_crc_off + VAL_CRC_LEN)
            .and_then(|b| <&[u8; VAL_CRC_LEN]>::try_from(b).ok()),
        region.get(off..off + HDR_COVERED),
        region.get(off + HEADER_LEN..off + HEADER_LEN + len),
    ) else {
        return false;
    };
    val_crc(covered, value) == u16::from_le_bytes(*crc)
}

/// Borrow the committed value bytes of the record at `off` (no copy). The caller has confirmed
/// [`is_committed`].
pub fn value_bytes<'a>(region: &'a [u8], off: usize, h: &Header) -> &'a [u8] {
    let start = off + HEADER_LEN;
    // `get`, not a slice index: the caller has confirmed `is_committed`, so the span is there, but
    // the compiler cannot see that across the call and a panic site here is on the control task's
    // arm-time path (`specs/panic-free.md`). An empty slice is the fail-soft, and a caller that
    // length-checks (every one does, against the field's width) reads it as absent.
    region.get(start..start + h.len as usize).unwrap_or(&[])
}

/// The `hdr_crc` over the 6 covered header bytes.
fn hdr_crc(covered: &[u8]) -> u16 {
    base::crc16::modbus(covered)
}

/// The `val_crc`: CRC over the 6 covered header bytes followed by the value (no pad). Incremental so
/// header and value are fed in pieces, matching how the store builds the record.
fn val_crc(covered: &[u8], value: &[u8]) -> u16 {
    let mut c = Crc16::new();
    c.update(covered);
    c.update(value);
    c.finish()
}

/// Encode just the 8-byte record header (`field_id`+`index`+`type`+`reserved`+`len`+`hdr_crc`) into a
/// small fixed array. The streaming append path programs this halfword-aligned block first, then the
/// value, then `val_crc`, so it never stages the whole record in RAM. [`encode`] builds the same
/// header for the host/test path, so the two cannot drift.
pub fn encode_header(field_id: u8, index: u8, type_tag: u8, len: u16) -> [u8; HEADER_LEN] {
    let mut h = [0u8; HEADER_LEN];
    h[0] = field_id;
    h[1] = index;
    h[2] = type_tag;
    h[3] = 0; // reserved, covered by hdr_crc
    h[4..6].copy_from_slice(&len.to_le_bytes());
    let hc = hdr_crc(&h[..HDR_COVERED]);
    h[6..8].copy_from_slice(&hc.to_le_bytes());
    h
}

/// The `val_crc` commit marker for a record, given its already-encoded `header` and the `value` bytes
/// (no pad). The streaming append path computes it over the header's 6 covered bytes plus the value,
/// matching [`encode`], so the commit halfword is byte-identical whether the record is staged or
/// streamed.
pub fn val_crc_of(header: &[u8; HEADER_LEN], value: &[u8]) -> u16 {
    val_crc(&header[..HDR_COVERED], value)
}

/// Encode a complete record into `out`, returning the number of bytes written (`= record_size(len)`).
///
/// `out` must be at least `record_size(value.len())`. The pad byte (for an odd `len`) is `0xFF` so it
/// reads as an erased byte. `val_crc` is written last in the buffer, mirroring that it is the
/// last-programmed halfword on flash (the commit marker). Shares [`encode_header`] / [`val_crc_of`]
/// with the streaming append path, so a staged record is byte-identical to a streamed one.
pub fn encode(field_id: u8, index: u8, type_tag: u8, value: &[u8], out: &mut [u8]) -> usize {
    let len = value.len();
    let total = record_size(len);
    debug_assert!(out.len() >= total);

    let header = encode_header(field_id, index, type_tag, len as u16);
    out[..HEADER_LEN].copy_from_slice(&header);

    out[HEADER_LEN..HEADER_LEN + len].copy_from_slice(value);
    if len & 1 == 1 {
        out[HEADER_LEN + len] = 0xFF; // pad, reads as erased
    }
    let padded = padded_len(len);
    let vc = val_crc_of(&header, value);
    out[HEADER_LEN + padded..HEADER_LEN + padded + VAL_CRC_LEN].copy_from_slice(&vc.to_le_bytes());

    total
}
