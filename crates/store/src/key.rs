//! The wire vocabulary: [`Key`], the storage-layout [`Type`], and the sealed [`Scalar`] trait that
//! pins each Rust scalar to its `Type` tag and width.
//!
//! These are the on-flash / on-wire forms underneath the typed handles. The dynamic tagged `Value`
//! enum and the generic `get(Key) -> Value` lookup the deferred Layer-3 link/bridge path needs are
//! **not** built here (per "don't build what nothing exercises yet"); the firmware names its own
//! fields through the typed handles in `field`, which need no runtime enum.

/// A store key: `field_id` names the field, `index` selects the instance (motor 0/1; a singleton
/// uses `index = 0`).
///
/// Two explicit bytes, not a bit-packed `u16`: code reads `key.field_id` / `key.index`, never
/// `key >> 8`. Byte-identical to a `u16` on the wire, in RAM, and on flash. The derives keep it
/// usable directly as a record/map key.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Key {
    /// Names the field (permanent, append-only; never reused for a new meaning).
    pub field_id: u8,
    /// Selects the instance. The store never interprets it; how many instances exist is the
    /// caller's concern.
    pub index: u8,
}

/// A blank `field_id`: an erased (`0xFF`) header byte, which marks the end of the log (the frontier).
/// No real field may use it.
pub const BLANK_FIELD_ID: u8 = 0xFF;

/// The storage-layout type. It sizes/validates a record and (eventually) rides in the link's
/// `CONFIG_RESP` so a consumer with no registry can interpret a value generically.
///
/// Fixed scalar types imply their length; `Blob`/`Str` are variable and use the record `len`.
/// Encoded as the record's `type` byte via [`Type::tag`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Type {
    U8,
    U16,
    U32,
    U64,
    I16,
    I32,
    I64,
    Bool,
    Blob,
    Str,
}

impl Type {
    /// The on-flash `type` byte for this storage type.
    pub const fn tag(self) -> u8 {
        match self {
            Type::U8 => 0x01,
            Type::U16 => 0x02,
            Type::U32 => 0x03,
            Type::U64 => 0x04,
            Type::I16 => 0x05,
            Type::I32 => 0x06,
            Type::I64 => 0x07,
            Type::Bool => 0x08,
            Type::Blob => 0x09,
            Type::Str => 0x0A,
        }
    }

    /// The fixed value width in bytes for a scalar type, or `None` for the variable types
    /// (`Blob`/`Str`, whose length is the record `len`).
    pub const fn fixed_len(self) -> Option<usize> {
        match self {
            Type::U8 | Type::Bool => Some(1),
            Type::U16 | Type::I16 => Some(2),
            Type::U32 | Type::I32 => Some(4),
            Type::U64 | Type::I64 => Some(8),
            Type::Blob | Type::Str => None,
        }
    }
}

/// Sealed marker pinning each Rust scalar to its storage [`Type`] and its little-endian byte form.
///
/// A scalar field's storage type is `<T as Scalar>::KIND`, the same tag `set` writes as the record
/// `type` byte and `get` uses to decode the value width, so a scalar field needs no hand-written
/// type. Sealed (the `private::Sealed` supertrait) so only the scalars listed here implement it;
/// the firmware cannot add a stray scalar with an inconsistent width.
pub trait Scalar: private::Sealed + Copy {
    /// The storage type tag for this Rust scalar.
    const KIND: Type;
    /// The fixed encoded width in bytes (`= KIND.fixed_len().unwrap()`).
    const WIDTH: usize;

    /// Little-endian encode into `out[..WIDTH]`.
    fn write_le(self, out: &mut [u8]);
    /// Little-endian decode from `bytes[..WIDTH]` (the caller has length-checked).
    fn read_le(bytes: &[u8]) -> Self;
}

mod private {
    /// Seals [`super::Scalar`]: only this module's listed scalars can implement it.
    pub trait Sealed {}
}

macro_rules! impl_scalar_int {
    ($($t:ty => $kind:expr, $w:expr;)*) => {$(
        impl private::Sealed for $t {}
        impl Scalar for $t {
            const KIND: Type = $kind;
            const WIDTH: usize = $w;
            #[inline]
            fn write_le(self, out: &mut [u8]) {
                out[..$w].copy_from_slice(&self.to_le_bytes());
            }
            #[inline]
            fn read_le(bytes: &[u8]) -> Self {
                let mut b = [0u8; $w];
                b.copy_from_slice(&bytes[..$w]);
                <$t>::from_le_bytes(b)
            }
        }
    )*};
}

impl_scalar_int! {
    u8  => Type::U8,  1;
    u16 => Type::U16, 2;
    u32 => Type::U32, 4;
    u64 => Type::U64, 8;
    i16 => Type::I16, 2;
    i32 => Type::I32, 4;
    i64 => Type::I64, 8;
}

impl private::Sealed for bool {}
impl Scalar for bool {
    const KIND: Type = Type::Bool;
    const WIDTH: usize = 1;
    #[inline]
    fn write_le(self, out: &mut [u8]) {
        out[0] = self as u8;
    }
    #[inline]
    fn read_le(bytes: &[u8]) -> Self {
        bytes[0] != 0
    }
}
