//! General `CONFIG_WRITE`/`CONFIG_READ` argument handling: parse a CLI-style typed value for ANY
//! registered field, honoring the field's [`store::Type`] from the store REGISTRY (the single
//! source of id/type/default; no second field table is hardcoded here), and encode the
//! `CONFIG_WRITE` payload the wire carries.
//!
//! This is the seam the `swd-mailbox-config` bin drives to stage a whole board-layout preset
//! (packed `port|pin` bytes, `imu.model`, scalars) over the SWD mailbox before a reboot, exactly
//! as a configurator would push it. Type SAFETY is honored (a value that cannot parse as the
//! field's registered type is rejected with a clear error), but there is NO board-model semantic
//! validation here: a duplicate pin, or a gate-capable pin in a LED field, is a well-typed write
//! that this tool passes through unchanged. The FIRMWARE's boot validator (`crates/board`) is the
//! thing under test, so staging a deliberately-invalid layout must succeed at this layer.

use std::fmt;

use store::{lookup, Key, Type, Value};

/// Why a field argument could not be turned into a typed [`Value`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldArgError {
    /// No registered field declares this id (the registry is the source of truth).
    UnknownField(u8),
    /// The raw string does not parse as the field's registered type (out of range, not a number,
    /// not a bool, ...). Carries the field, its type, the raw text, and a short reason.
    BadValue {
        field_id: u8,
        kind: Type,
        raw: String,
        why: String,
    },
    /// The field's type is not writable through this CLI (only `Blob` today: no board-layout field
    /// is a blob, and a hex-blob arg surface is unneeded until one exists).
    UnsupportedType { field_id: u8, kind: Type },
    /// The `FIELD[:INDEX]=VALUE` argument is malformed (missing `=`, bad field id, bad index).
    BadArg { arg: String, why: String },
}

impl fmt::Display for FieldArgError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FieldArgError::UnknownField(id) => {
                write!(f, "unknown field id {id:#04x} (not in the store registry)")
            }
            FieldArgError::BadValue {
                field_id,
                kind,
                raw,
                why,
            } => write!(
                f,
                "value {raw:?} is not a valid {kind:?} for field {field_id:#04x}: {why}"
            ),
            FieldArgError::UnsupportedType { field_id, kind } => write!(
                f,
                "field {field_id:#04x} has type {kind:?}, which this tool cannot write"
            ),
            FieldArgError::BadArg { arg, why } => {
                write!(
                    f,
                    "malformed field argument {arg:?}: {why} (want FIELD[:INDEX]=VALUE)"
                )
            }
        }
    }
}

impl std::error::Error for FieldArgError {}

/// Parse an unsigned magnitude accepting `0x`/`0X` hex or decimal.
fn parse_u64(t: &str) -> Result<u64, String> {
    let t = t.trim();
    let r = if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u64::from_str_radix(h, 16)
    } else {
        t.parse::<u64>()
    };
    r.map_err(|e| e.to_string())
}

/// Parse a signed integer accepting a leading `-` and `0x`/`0X` hex or decimal.
fn parse_i64(t: &str) -> Result<i64, String> {
    let t = t.trim();
    let (neg, body) = t.strip_prefix('-').map(|b| (true, b)).unwrap_or((false, t));
    let mag = if let Some(h) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
        i64::from_str_radix(h, 16)
    } else {
        body.parse::<i64>()
    }
    .map_err(|e| e.to_string())?;
    Ok(if neg { -mag } else { mag })
}

/// Parse `raw` into a [`Value`] of `field_id`'s registered [`Type`] (the store registry is the
/// single source of that type; see [`store::lookup`]).
///
/// Integer fields accept `0x`-hex or decimal (a packed `port|pin` byte such as `0x16` = PB6 is a
/// `u8` field, so it round-trips exactly); `Bool` accepts `true`/`false`/`1`/`0`; `Str` takes the
/// raw text. The returned value BORROWS `raw` for the `Str` case, so `raw` must outlive the value.
/// A value that cannot parse as the field's type, or is out of the type's range, is a
/// [`FieldArgError::BadValue`]; an unknown field id is [`FieldArgError::UnknownField`]. There is NO
/// board-model semantic check here (that is the firmware's boot validator).
pub fn parse_field_value(field_id: u8, raw: &str) -> Result<Value<'_>, FieldArgError> {
    let def = lookup(field_id).ok_or(FieldArgError::UnknownField(field_id))?;
    let bad = |why: String| FieldArgError::BadValue {
        field_id,
        kind: def.kind,
        raw: raw.to_string(),
        why,
    };
    // Range-check an unsigned parse against the target width's max.
    let uint = |max: u64| -> Result<u64, FieldArgError> {
        let v = parse_u64(raw).map_err(&bad)?;
        if v > max {
            Err(bad(format!("out of range (max {max})")))
        } else {
            Ok(v)
        }
    };
    // Range-check a signed parse against the target width's [min, max].
    let sint = |min: i64, max: i64| -> Result<i64, FieldArgError> {
        let v = parse_i64(raw).map_err(&bad)?;
        if v < min || v > max {
            Err(bad(format!("out of range ([{min}, {max}])")))
        } else {
            Ok(v)
        }
    };
    let value = match def.kind {
        Type::U8 => Value::U8(uint(u8::MAX as u64)? as u8),
        Type::U16 => Value::U16(uint(u16::MAX as u64)? as u16),
        Type::U32 => Value::U32(uint(u32::MAX as u64)? as u32),
        Type::U64 => Value::U64(parse_u64(raw).map_err(&bad)?),
        Type::I16 => Value::I16(sint(i16::MIN as i64, i16::MAX as i64)? as i16),
        Type::I32 => Value::I32(sint(i32::MIN as i64, i32::MAX as i64)? as i32),
        Type::I64 => Value::I64(parse_i64(raw).map_err(&bad)?),
        Type::Bool => match raw.trim() {
            "true" | "1" => Value::Bool(true),
            "false" | "0" => Value::Bool(false),
            _ => return Err(bad("expected true/false/1/0".into())),
        },
        Type::Str => Value::Str(raw),
        Type::Blob => {
            return Err(FieldArgError::UnsupportedType {
                field_id,
                kind: def.kind,
            })
        }
    };
    Ok(value)
}

/// Split one `FIELD[:INDEX]=VALUE` CLI argument into `(field_id, index, value_str)`.
///
/// `FIELD` is a `0x`-hex or decimal field id; `INDEX` (optional, default 0) is a decimal per-motor
/// index; `VALUE` is the raw text parsed by [`parse_field_value`] against the field's type. The id
/// is checked against the registry so a typo fails here rather than silently.
pub fn parse_field_arg(arg: &str) -> Result<(u8, u8, &str), FieldArgError> {
    let bad = |why: &str| FieldArgError::BadArg {
        arg: arg.to_string(),
        why: why.to_string(),
    };
    let (lhs, value) = arg.split_once('=').ok_or_else(|| bad("missing '='"))?;
    let (field_str, index_str) = match lhs.split_once(':') {
        Some((f, i)) => (f, Some(i)),
        None => (lhs, None),
    };
    let field_id: u8 = {
        let t = field_str.trim();
        let r = if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
            u8::from_str_radix(h, 16)
        } else {
            t.parse::<u8>()
        };
        r.map_err(|_| bad("field id is not a byte (0x40 or 64)"))?
    };
    if lookup(field_id).is_none() {
        return Err(FieldArgError::UnknownField(field_id));
    }
    let index: u8 = match index_str {
        Some(i) => i.trim().parse().map_err(|_| bad("index is not a byte"))?,
        None => 0,
    };
    Ok((field_id, index, value))
}

/// Encode the `CONFIG_WRITE` payload the wire carries: `[field_id, index, type_tag, value_le...]`.
/// The single owner of that layout (both [`crate::walk::WalkDriver::config_write`] and the tests
/// call this, so the encoding is never duplicated).
pub fn encode_config_write(key: Key, value: &Value) -> Vec<u8> {
    let mut payload = vec![key.field_id, key.index, value.kind().tag()];
    let mut vb = [0u8; 64];
    let vn = value.encode(&mut vb);
    payload.extend_from_slice(&vb[..vn]);
    payload
}
