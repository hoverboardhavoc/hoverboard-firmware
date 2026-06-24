//! The wire vocabulary: [`Key`], [`Type`], and [`Value`].
//!
//! One 16-bit key namespace unifies the flash record key, the RAM live-value table, and (later) the
//! link register protocol. The vocabulary lives here in the store crate for now; it splits into a
//! `params` crate only when a non-store consumer needs to name keys without the flash store (see
//! `specs/params.md`). There is no second consumer yet, so it stays here.

/// A field key: two explicit bytes, NOT a bit-packed `u16`. `field_id` names the field; `index`
/// selects the instance (motor 0/1; singletons use `index = 0`). The layout is two bytes on the wire
/// (`field_id` then `index`), byte-identical to a little-endian `u16`, but code reads `key.field_id`
/// / `key.index`, never `key >> 8`. The derives keep it usable directly as a record/map key.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Key {
    /// The field id (matches `FieldDef.id`).
    pub field_id: u8,
    /// The instance selector (0 for singletons).
    pub index: u8,
}

impl Key {
    /// A new key.
    #[inline]
    pub const fn new(field_id: u8, index: u8) -> Self {
        Self { field_id, index }
    }
}

/// The storage-layout type: it sizes and validates a record and rides in the link's `CONFIG_RESP`, so
/// a consumer with no registry still interprets a value generically. `#[repr(u8)]` so the discriminant
/// is the stable on-flash / on-wire `type` byte.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Type {
    /// Unsigned 8-bit.
    U8 = 0,
    /// Unsigned 16-bit.
    U16 = 1,
    /// Unsigned 32-bit.
    U32 = 2,
    /// Unsigned 64-bit.
    U64 = 3,
    /// Signed 16-bit.
    I16 = 4,
    /// Signed 32-bit.
    I32 = 5,
    /// Signed 64-bit.
    I64 = 6,
    /// Variable-length opaque bytes (uses `len`).
    Blob = 7,
    /// Variable-length UTF-8 string (uses `len`).
    Str = 8,
    /// A boolean, 1 byte on flash.
    Bool = 9,
}

impl Type {
    /// The on-flash `type` byte for this type (the `#[repr(u8)]` discriminant).
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode a `type` byte back into a [`Type`], or `None` for an unknown discriminant (a record
    /// written by a newer firmware with a type this build does not model).
    #[inline]
    pub const fn from_u8(b: u8) -> Option<Type> {
        match b {
            0 => Some(Type::U8),
            1 => Some(Type::U16),
            2 => Some(Type::U32),
            3 => Some(Type::U64),
            4 => Some(Type::I16),
            5 => Some(Type::I32),
            6 => Some(Type::I64),
            7 => Some(Type::Blob),
            8 => Some(Type::Str),
            9 => Some(Type::Bool),
            _ => None,
        }
    }

    /// The fixed on-flash value length for a fixed-width type, or `None` for the variable-length
    /// types ([`Type::Blob`] / [`Type::Str`], whose length is the record's `len`).
    #[inline]
    pub const fn fixed_len(self) -> Option<usize> {
        match self {
            Type::U8 | Type::Bool => Some(1),
            Type::U16 | Type::I16 => Some(2),
            Type::U32 | Type::I32 => Some(4),
            Type::U64 | Type::I64 => Some(8),
            Type::Blob | Type::Str => None,
        }
    }

    /// Whether this is a variable-length type ([`Type::Blob`] / [`Type::Str`]). Variable values are
    /// left in flash and read on demand, never RAM-cached.
    #[inline]
    pub const fn is_variable(self) -> bool {
        matches!(self, Type::Blob | Type::Str)
    }
}

/// The tagged value `get()` returns and `set()` takes. Scalars are by value; the variable-length
/// types borrow a flash-resident byte slice on demand (no heap, no RAM cache). The typed accessors
/// (`.u32()`, `.bytes()`, etc.) read a known field without a match at the call site.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Value<'a> {
    /// Unsigned 8-bit.
    U8(u8),
    /// Unsigned 16-bit.
    U16(u16),
    /// Unsigned 32-bit.
    U32(u32),
    /// Unsigned 64-bit.
    U64(u64),
    /// Signed 16-bit.
    I16(i16),
    /// Signed 32-bit.
    I32(i32),
    /// Signed 64-bit.
    I64(i64),
    /// A boolean.
    Bool(bool),
    /// Variable-length bytes (`BLOB`/`STR`), borrowing flash on demand.
    Bytes(&'a [u8]),
}

impl<'a> Value<'a> {
    /// This value's storage [`Type`].
    #[inline]
    pub const fn kind(&self) -> Type {
        match self {
            Value::U8(_) => Type::U8,
            Value::U16(_) => Type::U16,
            Value::U32(_) => Type::U32,
            Value::U64(_) => Type::U64,
            Value::I16(_) => Type::I16,
            Value::I32(_) => Type::I32,
            Value::I64(_) => Type::I64,
            Value::Bool(_) => Type::Bool,
            // A borrowed-bytes value is BLOB at the storage layer (STR is the same bytes; the registry
            // carries the STR-vs-BLOB distinction, the value tag does not).
            Value::Bytes(_) => Type::Blob,
        }
    }

    /// Read this value as a `u32`, panicking on a type mismatch. The typed accessor `run_phase` uses
    /// (`store.get(T_KEY).u32()`): the caller knows the field's type, so a mismatch is a bug, not a
    /// recoverable condition.
    #[inline]
    pub fn u32(&self) -> u32 {
        match self {
            Value::U32(v) => *v,
            other => panic!("Value::u32() on a {:?}", other.kind()),
        }
    }

    /// Read this value as a `u64`, panicking on a type mismatch.
    #[inline]
    pub fn u64(&self) -> u64 {
        match self {
            Value::U64(v) => *v,
            other => panic!("Value::u64() on a {:?}", other.kind()),
        }
    }

    /// Read this value as a `bool`, panicking on a type mismatch.
    #[inline]
    pub fn bool(&self) -> bool {
        match self {
            Value::Bool(v) => *v,
            other => panic!("Value::bool() on a {:?}", other.kind()),
        }
    }

    /// Read this value as borrowed bytes (`BLOB`/`STR`), panicking on a type mismatch.
    #[inline]
    pub fn bytes(&self) -> &'a [u8] {
        match self {
            Value::Bytes(b) => b,
            other => panic!("Value::bytes() on a {:?}", other.kind()),
        }
    }
}
