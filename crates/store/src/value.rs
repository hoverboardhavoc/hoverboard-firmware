//! The dynamic tagged [`Value`]: the form where a [`Type`] tag travels with the data, so a consumer
//! with **no schema** can `match` it or ask [`Value::kind`].
//!
//! This is the deferred Layer-3 path's vocabulary (`specs/storage-layer.md`, "Key, types, and the
//! registry"; `specs/l3.md`, "`CONFIG_*` is the wire face of the store"), **un-deferred now** because
//! L3's `CONFIG_*`/registry is the consumer that exercises it. The firmware still names its own fields
//! through the typed handles (which need no runtime enum); `Value` is the generic
//! `CONFIG_READ`/`CONFIG_WRITE` face.
//!
//! Scalar variants are owned; the variable types borrow (`Str(&'a [u8])` / `Bytes(&'a [u8])`), so a
//! `get` can hand back a flash-borrowing slice with no copy, exactly as `get_text` / `get_bytes` do.

use crate::key::Type;

/// A dynamically typed config value: a [`Type`] tag plus the data.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Value<'a> {
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I16(i16),
    I32(i32),
    I64(i64),
    Bool(bool),
    /// A `STR` field's bytes. NOT validated as UTF-8, and deliberately not a `&str`: the board
    /// makes no encoding claim about a record it did not write (`specs/storage-layer.md`, the
    /// `StrField` paragraph; `specs/decision-flash-budget.md`, shrink round 2 item 4, which bought
    /// 496 B of flashed span by dropping the validator). `STR` and `BLOB` differ here only in the
    /// [`Type`] tag they carry, which is unchanged on the wire and on flash.
    Str(&'a [u8]),
    /// Raw bytes.
    Bytes(&'a [u8]),
}

impl<'a> Value<'a> {
    /// The storage [`Type`] this value carries (the tag a schema-less consumer reads).
    pub fn kind(&self) -> Type {
        match self {
            Value::U8(_) => Type::U8,
            Value::U16(_) => Type::U16,
            Value::U32(_) => Type::U32,
            Value::U64(_) => Type::U64,
            Value::I16(_) => Type::I16,
            Value::I32(_) => Type::I32,
            Value::I64(_) => Type::I64,
            Value::Bool(_) => Type::Bool,
            Value::Str(_) => Type::Str,
            Value::Bytes(_) => Type::Blob,
        }
    }

    /// Encode the value payload little-endian into `out`, returning the byte count (the record `type`
    /// byte is [`Value::kind`]`.tag()`, written separately by the store). `out` must be at least as
    /// large as the value: scalars need <= 8 bytes, `Str`/`Bytes` need their own length. Panics (slice
    /// bounds) if `out` is too small, so the caller sizes it (the store uses its max-record buffer).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        match self {
            Value::U8(v) => {
                out[0] = *v;
                1
            }
            Value::Bool(b) => {
                out[0] = *b as u8;
                1
            }
            Value::U16(v) => write_le(&v.to_le_bytes(), out),
            Value::I16(v) => write_le(&v.to_le_bytes(), out),
            Value::U32(v) => write_le(&v.to_le_bytes(), out),
            Value::I32(v) => write_le(&v.to_le_bytes(), out),
            Value::U64(v) => write_le(&v.to_le_bytes(), out),
            Value::I64(v) => write_le(&v.to_le_bytes(), out),
            Value::Str(s) => write_le(s, out),
            Value::Bytes(b) => write_le(b, out),
        }
    }

    /// Decode a value of storage `kind` from its little-endian payload `bytes`, borrowing for the
    /// variable types. Returns `None` on a width mismatch (a fixed type whose bytes are the wrong
    /// length) - the same "ignore a malformed record" rule the typed getters use. A `STR` has no
    /// width and no charset to be wrong about, so, like a `BLOB`, it always decodes: whatever bytes
    /// the record holds come back as [`Value::Str`].
    pub fn decode(kind: Type, bytes: &'a [u8]) -> Option<Value<'a>> {
        Some(match kind {
            Type::U8 => Value::U8(*bytes.first()?),
            Type::Bool => Value::Bool(*bytes.first()? != 0),
            Type::U16 => Value::U16(u16::from_le_bytes(fixed(bytes)?)),
            Type::I16 => Value::I16(i16::from_le_bytes(fixed(bytes)?)),
            Type::U32 => Value::U32(u32::from_le_bytes(fixed(bytes)?)),
            Type::I32 => Value::I32(i32::from_le_bytes(fixed(bytes)?)),
            Type::U64 => Value::U64(u64::from_le_bytes(fixed(bytes)?)),
            Type::I64 => Value::I64(i64::from_le_bytes(fixed(bytes)?)),
            Type::Str => Value::Str(bytes),
            Type::Blob => Value::Bytes(bytes),
        })
    }
}

/// Copy `src` into `out[..src.len()]`, returning the length. Single bounds-checked copy.
fn write_le(src: &[u8], out: &mut [u8]) -> usize {
    out[..src.len()].copy_from_slice(src);
    src.len()
}

/// Read exactly `N` bytes (the fixed scalar width) or `None` if the length is wrong. The `Type::U8`
/// path uses `first` instead; this serves the multi-byte fixed types.
fn fixed<const N: usize>(bytes: &[u8]) -> Option<[u8; N]> {
    if bytes.len() != N {
        return None;
    }
    let mut b = [0u8; N];
    b.copy_from_slice(bytes);
    Some(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kind_reports_the_tag() {
        assert_eq!(Value::U32(7).kind(), Type::U32);
        assert_eq!(Value::I16(-1).kind(), Type::I16);
        assert_eq!(Value::Bool(true).kind(), Type::Bool);
        assert_eq!(Value::Str(b"hi").kind(), Type::Str);
        assert_eq!(Value::Bytes(&[1, 2]).kind(), Type::Blob);
    }

    #[test]
    fn scalar_round_trips_through_encode_decode() {
        let cases = [
            Value::U8(0xAB),
            Value::U16(0xBEEF),
            Value::U32(0xDEAD_BEEF),
            Value::U64(0x0102_0304_0506_0708),
            Value::I16(-2),
            Value::I32(-3),
            Value::I64(-4),
            Value::Bool(true),
            Value::Bool(false),
        ];
        for v in cases {
            let mut buf = [0u8; 8];
            let n = v.encode(&mut buf);
            assert_eq!(n, v.kind().fixed_len().unwrap());
            let got = Value::decode(v.kind(), &buf[..n]).unwrap();
            assert_eq!(got, v);
        }
    }

    #[test]
    fn variable_round_trips_and_borrows() {
        let mut buf = [0u8; 32];
        let s = Value::Str(b"Hoverboard");
        let n = s.encode(&mut buf);
        assert_eq!(Value::decode(Type::Str, &buf[..n]).unwrap(), s);

        let b = Value::Bytes(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let n = b.encode(&mut buf);
        assert_eq!(Value::decode(Type::Blob, &buf[..n]).unwrap(), b);
    }

    #[test]
    fn decode_rejects_width_mismatch_only() {
        // A U32 whose payload is 2 bytes is malformed.
        assert_eq!(Value::decode(Type::U32, &[1, 2]), None);
        assert_eq!(Value::decode(Type::U8, &[]), None);
        // STR and BLOB both accept any bytes: no width, no charset. Bytes that are not valid UTF-8
        // decode as the STR they are, which is the shrink-round-2 behaviour change (they used to
        // decode as `None` and so read as the field's default).
        assert_eq!(
            Value::decode(Type::Str, &[0xFF, 0xFE]),
            Some(Value::Str(&[0xFF, 0xFE]))
        );
        assert_eq!(
            Value::decode(Type::Blob, &[0xFF, 0xFE]),
            Some(Value::Bytes(&[0xFF, 0xFE]))
        );
    }

    #[test]
    fn str_and_blob_decode_identically_for_the_same_bytes() {
        // The spec's requirement: STR and BLOB are byte-identical on flash AND on the read. Only
        // the tag differs.
        let bytes: &[u8] = &[0x68, 0x69, 0xFF];
        let s = Value::decode(Type::Str, bytes).unwrap();
        let b = Value::decode(Type::Blob, bytes).unwrap();
        assert_eq!(s, Value::Str(bytes));
        assert_eq!(b, Value::Bytes(bytes));
        assert_eq!(s.kind(), Type::Str);
        assert_eq!(b.kind(), Type::Blob);
        let (mut bs, mut bb) = ([0u8; 8], [0u8; 8]);
        let (ns, nb) = (s.encode(&mut bs), b.encode(&mut bb));
        assert_eq!(&bs[..ns], &bb[..nb]);
    }
}
