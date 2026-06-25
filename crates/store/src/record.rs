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
/// The page-header size in bytes: `[ magic:u32 | seq:u16 | reserved:u16 ]`, even so the first record
/// stays halfword-aligned.
pub const PAGE_HEADER_LEN: usize = 8;

/// Encode a page header for `seq` (`[ magic:u32 | seq:u16 | reserved=0:u16 ]`), the exact bytes the
/// store programs for a valid page side. The store itself programs `magic` last (the power-safety
/// hinge); this emits the committed end state, which the planted-region test builders use verbatim.
pub fn encode_page_header(seq: u16) -> [u8; PAGE_HEADER_LEN] {
    let mut hdr = [0u8; PAGE_HEADER_LEN];
    hdr[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    hdr[4..6].copy_from_slice(&seq.to_le_bytes());
    // reserved (hdr[6..8]) left 0.
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
    if off + HEADER_LEN > region.len() {
        return HeaderScan::Torn;
    }
    let h = &region[off..off + HEADER_LEN];
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
    if val_crc_off + VAL_CRC_LEN > region.len() {
        return false;
    }
    let stored = u16::from_le_bytes([region[val_crc_off], region[val_crc_off + 1]]);
    val_crc(
        &region[off..off + HDR_COVERED],
        &region[off + HEADER_LEN..off + HEADER_LEN + len],
    ) == stored
}

/// Borrow the committed value bytes of the record at `off` (no copy). The caller has confirmed
/// [`is_committed`].
pub fn value_bytes<'a>(region: &'a [u8], off: usize, h: &Header) -> &'a [u8] {
    let start = off + HEADER_LEN;
    &region[start..start + h.len as usize]
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

/// Encode a complete record into `out`, returning the number of bytes written (`= record_size(len)`).
///
/// `out` must be at least `record_size(value.len())`. The pad byte (for an odd `len`) is `0xFF` so it
/// reads as an erased byte. `val_crc` is written last in the buffer, mirroring that it is the
/// last-programmed halfword on flash (the commit marker).
pub fn encode(field_id: u8, index: u8, type_tag: u8, value: &[u8], out: &mut [u8]) -> usize {
    let len = value.len();
    let total = record_size(len);
    debug_assert!(out.len() >= total);

    out[0] = field_id;
    out[1] = index;
    out[2] = type_tag;
    out[3] = 0; // reserved, covered by hdr_crc
    out[4..6].copy_from_slice(&(len as u16).to_le_bytes());
    let hc = hdr_crc(&out[..HDR_COVERED]);
    out[6..8].copy_from_slice(&hc.to_le_bytes());

    out[HEADER_LEN..HEADER_LEN + len].copy_from_slice(value);
    if len & 1 == 1 {
        out[HEADER_LEN + len] = 0xFF; // pad, reads as erased
    }
    let padded = padded_len(len);
    let vc = val_crc(&out[..HDR_COVERED], value);
    out[HEADER_LEN + padded..HEADER_LEN + padded + VAL_CRC_LEN].copy_from_slice(&vc.to_le_bytes());

    total
}
