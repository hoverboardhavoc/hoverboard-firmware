//! The on-flash record codec.
//!
//! ```text
//! [ field_id:u8 | index:u8 | type:u8 | len:u8 | hdr_crc:u16 | value[len] (+pad to even) | val_crc:u16 ]
//!    \____________________ 6-byte header ____________________/   \_____ value _____/
//!    field_id == 0xFF  =>  blank (erased 0xFFFF)  =>  end of log (frontier)
//! ```
//!
//! - `hdr_crc` covers `field_id`+`index`+`type`+`len` (the first 4 bytes), so `len` is trustworthy and
//!   the variable-length log stays walkable + torn-write-safe.
//! - An odd `len` value is followed by one `0xFF` pad byte so `val_crc` and the next record stay
//!   halfword-aligned. The walk hops `next = here + 6 + (len + (len & 1)) + 2`.
//! - `val_crc` validates the value bytes (the `len` payload, NOT the pad) and is the commit marker,
//!   the LAST halfword written. A record is valid only when complete.

use base::crc16::Crc16;

use crate::params::{Type, Value};

/// The fixed header length (`field_id|index|type|len|hdr_crc`), always even.
pub const HEADER_LEN: usize = 6;
/// The trailing `val_crc` length, always even.
pub const VAL_CRC_LEN: usize = 2;
/// The blank/erased `field_id` sentinel: an erased halfword reads `0xFF` in the first byte, marking
/// the frontier (end of log).
pub const BLANK_ID: u8 = 0xFF;

/// The maximum value length a record can carry (`len` is a `u8`).
pub const MAX_VAL_LEN: usize = 255;
/// A scratch buffer big enough for the largest possible record (header + padded value + val_crc).
pub const MAX_RECORD_LEN: usize = HEADER_LEN + MAX_VAL_LEN + 1 + VAL_CRC_LEN;

/// Round a value length up to an even number of bytes (the on-flash padded span the value occupies).
#[inline]
pub const fn padded_len(len: usize) -> usize {
    len + (len & 1)
}

/// The total on-flash span of a record whose value is `len` bytes (header + padded value + val_crc).
/// This is the `next - here` hop the scan uses.
#[inline]
pub const fn record_span(len: usize) -> usize {
    HEADER_LEN + padded_len(len) + VAL_CRC_LEN
}

/// A decoded, validated record header (the first 6 bytes). `hdr_crc` is already verified when this is
/// produced, so `len` is trustworthy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// `field_id`.
    pub field_id: u8,
    /// `index`.
    pub index: u8,
    /// The raw `type` byte (decode with [`Type::from_u8`]; an unknown type is still walkable).
    pub type_byte: u8,
    /// The value length in bytes.
    pub len: u8,
}

/// The outcome of trying to decode a header at an offset during the scan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderScan {
    /// A blank (erased) header: the frontier. The scan stops here and appends after it.
    Blank,
    /// A header whose `hdr_crc` failed: a torn header. Treated as the end of the log (compact).
    Torn,
    /// A valid header (its `hdr_crc` checked out); carries the trusted fields.
    Valid(Header),
}

/// Compute `hdr_crc` over the 4 header bytes.
#[inline]
pub fn hdr_crc(field_id: u8, index: u8, type_byte: u8, len: u8) -> u16 {
    let mut c = Crc16::new();
    c.update(&[field_id, index, type_byte, len]);
    c.finish()
}

/// Compute `val_crc` over the value bytes (the `len` payload, NOT the pad).
#[inline]
pub fn val_crc(value: &[u8]) -> u16 {
    let mut c = Crc16::new();
    c.update(value);
    c.finish()
}

/// Classify a 6-byte header slice: blank, torn, or valid. `raw` must be at least [`HEADER_LEN`] long.
pub fn classify_header(raw: &[u8]) -> HeaderScan {
    // A blank header: the first byte is the erased sentinel. (An erased halfword is 0xFFFF, so a blank
    // record's field_id reads 0xFF.)
    if raw[0] == BLANK_ID {
        return HeaderScan::Blank;
    }
    let field_id = raw[0];
    let index = raw[1];
    let type_byte = raw[2];
    let len = raw[3];
    let stored = u16::from_le_bytes([raw[4], raw[5]]);
    if hdr_crc(field_id, index, type_byte, len) != stored {
        return HeaderScan::Torn;
    }
    HeaderScan::Valid(Header {
        field_id,
        index,
        type_byte,
        len,
    })
}

/// Verify a record's `val_crc` against its value bytes. `value` is the `len` payload (no pad);
/// `stored` is the trailing `val_crc` halfword read from flash. True iff the value is committed.
#[inline]
pub fn value_committed(value: &[u8], stored: u16) -> bool {
    val_crc(value) == stored
}

/// Serialize a fixed-width [`Value`] into `out` (which must be at least 8 bytes), returning the value
/// length. Little-endian, matching the on-flash byte order. A variable-length [`Value::Bytes`] is NOT
/// handled here (it is appended directly from its slice); calling this with `Bytes` returns 0.
pub fn encode_fixed(value: &Value, out: &mut [u8]) -> usize {
    match value {
        Value::U8(v) => {
            out[0] = *v;
            1
        }
        Value::Bool(v) => {
            out[0] = *v as u8;
            1
        }
        Value::U16(v) => {
            out[..2].copy_from_slice(&v.to_le_bytes());
            2
        }
        Value::I16(v) => {
            out[..2].copy_from_slice(&v.to_le_bytes());
            2
        }
        Value::U32(v) => {
            out[..4].copy_from_slice(&v.to_le_bytes());
            4
        }
        Value::I32(v) => {
            out[..4].copy_from_slice(&v.to_le_bytes());
            4
        }
        Value::U64(v) => {
            out[..8].copy_from_slice(&v.to_le_bytes());
            8
        }
        Value::I64(v) => {
            out[..8].copy_from_slice(&v.to_le_bytes());
            8
        }
        // Variable bytes are appended straight from the slice, not through this fixed encoder.
        Value::Bytes(_) => 0,
    }
}

/// Decode a fixed-width value of `kind` from its `len`-byte flash payload into an owned [`Value`].
/// Returns `None` if the length does not match the type's fixed size (a corrupt or mistyped record).
/// Variable types ([`Type::Blob`]/[`Type::Str`]) are NOT decoded here (they borrow flash on demand).
pub fn decode_fixed(kind: Type, bytes: &[u8]) -> Option<Value<'static>> {
    match kind {
        Type::U8 => bytes
            .first()
            .map(|b| Value::U8(*b))
            .filter(|_| bytes.len() == 1),
        Type::Bool => bytes
            .first()
            .map(|b| Value::Bool(*b != 0))
            .filter(|_| bytes.len() == 1),
        Type::U16 => {
            (bytes.len() == 2).then(|| Value::U16(u16::from_le_bytes([bytes[0], bytes[1]])))
        }
        Type::I16 => {
            (bytes.len() == 2).then(|| Value::I16(i16::from_le_bytes([bytes[0], bytes[1]])))
        }
        Type::U32 => (bytes.len() == 4)
            .then(|| Value::U32(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))),
        Type::I32 => (bytes.len() == 4)
            .then(|| Value::I32(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))),
        Type::U64 => (bytes.len() == 8).then(|| {
            Value::U64(u64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ]))
        }),
        Type::I64 => (bytes.len() == 8).then(|| {
            Value::I64(i64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ]))
        }),
        Type::Blob | Type::Str => None,
    }
}

/// Build a complete record (header + padded value + val_crc) into `out` for `field_id`/`index`/`kind`
/// with value bytes `value`. Returns the total record span written. `out` must be at least
/// [`record_span`]`(value.len())` long. The pad byte (when `len` is odd) is `0xFF`. This is the exact
/// byte image an append programs to the frontier, val_crc last.
pub fn build(field_id: u8, index: u8, kind: Type, value: &[u8], out: &mut [u8]) -> usize {
    let len = value.len();
    let type_byte = kind.as_u8();
    let hc = hdr_crc(field_id, index, type_byte, len as u8);
    out[0] = field_id;
    out[1] = index;
    out[2] = type_byte;
    out[3] = len as u8;
    out[4..6].copy_from_slice(&hc.to_le_bytes());
    out[HEADER_LEN..HEADER_LEN + len].copy_from_slice(value);
    let mut pos = HEADER_LEN + len;
    if len & 1 == 1 {
        out[pos] = 0xFF; // pad to even so val_crc stays halfword-aligned.
        pos += 1;
    }
    let vc = val_crc(value);
    out[pos..pos + VAL_CRC_LEN].copy_from_slice(&vc.to_le_bytes());
    pos + VAL_CRC_LEN
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn even_value_round_trips() {
        let mut buf = [0u8; MAX_RECORD_LEN];
        let n = build(0x42, 1, Type::U32, &0xDEAD_BEEFu32.to_le_bytes(), &mut buf);
        assert_eq!(n, record_span(4));
        // No pad on an even length.
        assert_eq!(n, HEADER_LEN + 4 + VAL_CRC_LEN);
        match classify_header(&buf) {
            HeaderScan::Valid(h) => {
                assert_eq!(h.field_id, 0x42);
                assert_eq!(h.index, 1);
                assert_eq!(h.len, 4);
                let val = &buf[HEADER_LEN..HEADER_LEN + 4];
                let stored = u16::from_le_bytes([buf[HEADER_LEN + 4], buf[HEADER_LEN + 5]]);
                assert!(value_committed(val, stored));
                assert_eq!(decode_fixed(Type::U32, val), Some(Value::U32(0xDEAD_BEEF)));
            }
            other => panic!("expected Valid, got {other:?}"),
        }
    }

    #[test]
    fn odd_value_gets_a_pad_and_stays_even() {
        let mut buf = [0u8; MAX_RECORD_LEN];
        // A 1-byte value (U8): padded to 2 so the record span is even.
        let n = build(0x10, 0, Type::U8, &[0xAB], &mut buf);
        assert_eq!(n, record_span(1));
        assert_eq!(
            n % 2,
            0,
            "the on-flash record must be an even number of bytes"
        );
        // The pad byte is 0xFF.
        assert_eq!(buf[HEADER_LEN + 1], 0xFF);
        // val_crc covers only the value byte, not the pad.
        let stored = u16::from_le_bytes([buf[HEADER_LEN + 2], buf[HEADER_LEN + 3]]);
        assert!(value_committed(&[0xAB], stored));
    }

    #[test]
    fn blank_header_is_the_frontier() {
        let raw = [0xFFu8; HEADER_LEN];
        assert_eq!(classify_header(&raw), HeaderScan::Blank);
    }

    #[test]
    fn a_torn_header_crc_fails() {
        let mut buf = [0u8; MAX_RECORD_LEN];
        build(0x42, 0, Type::U16, &[1, 2], &mut buf);
        // Corrupt the len byte AFTER the crc was computed: hdr_crc no longer matches.
        buf[3] ^= 0xFF;
        assert_eq!(classify_header(&buf), HeaderScan::Torn);
    }

    #[test]
    fn a_torn_value_fails_val_crc_but_header_is_still_walkable() {
        let mut buf = [0u8; MAX_RECORD_LEN];
        let n = build(0x42, 0, Type::U32, &0x1122_3344u32.to_le_bytes(), &mut buf);
        // The header is intact (still walkable): we can hop by its trusted len.
        let h = match classify_header(&buf) {
            HeaderScan::Valid(h) => h,
            other => panic!("{other:?}"),
        };
        assert_eq!(record_span(h.len as usize), n);
        // But corrupt a value byte: val_crc now fails, so the record is ignored on scan.
        buf[HEADER_LEN] ^= 0xFF;
        let val = &buf[HEADER_LEN..HEADER_LEN + 4];
        let stored = u16::from_le_bytes([buf[HEADER_LEN + 4], buf[HEADER_LEN + 5]]);
        assert!(!value_committed(val, stored));
    }
}
