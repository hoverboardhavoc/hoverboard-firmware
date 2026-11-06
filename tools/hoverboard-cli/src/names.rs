//! The name table's rules (`specs/cli-config.md` sections 2.1 and 2.2): the path a key goes by on
//! the command line and in a file, and the form a value takes in a file. The table itself is the
//! store's (`store::NAMES`, `shared/store/src/field.rs`); this module only applies it.

use net::walk::{CONFIG_RESP_VALUE_MAX, CONFIG_WRITE_VALUE_MAX};
use serde_json::{Map, Value as Json};
use store::{by_id, FieldName, Index, Key, Kind, Type, Value, NAMES};

use crate::nodes::{parse_int, parse_key};

/// A problem with a path or a value (`specs/cli-config.md` section 5). `force` marks the ones
/// `--force` turns into a warning; the others always stop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Problem {
    /// What is wrong, and what would allow it.
    pub msg: String,
    /// `--force` turns it into a warning.
    pub force: bool,
}

impl Problem {
    /// A problem that always stops.
    pub fn stop(msg: impl Into<String>) -> Self {
        Problem {
            msg: msg.into(),
            force: false,
        }
    }

    /// A problem `--force` turns into a warning.
    pub fn forceable(msg: impl Into<String>) -> Self {
        Problem {
            msg: msg.into(),
            force: true,
        }
    }

    /// Whether it stops a command run with `force`.
    pub fn stops(&self, force: bool) -> bool {
        !(self.force && force)
    }
}

/// The table's row for `key`'s id, if it names one.
pub fn row_of(key: Key) -> Option<&'static FieldName> {
    by_id(key.field_id)
}

/// The path `key` prints as: `machine.mode`, `battery_sense.cal[1]`, `motors[0].hall.a`,
/// `motors[1].observer.shadow[4]`, `links[0].tx`, `tone.horn`, or the raw `0x99.0` for an id the
/// table does not name. A single field at an index past its one, or a named field past its names,
/// prints with the index, so a forced write names what it wrote.
pub fn path_of(key: Key) -> String {
    match row_of(key) {
        Some(r) => match r.index {
            Index::Motor | Index::Link => format!(
                "{}[{}].{}",
                r.index.array().expect("an object array"),
                key.index,
                r.path
            ),
            Index::MotorArray(n) => format!(
                "motors[{}].{}[{}]",
                key.index / n.max(1),
                r.path,
                key.index % n.max(1)
            ),
            Index::Single if key.index == 0 => r.path.to_string(),
            Index::Named(_) if r.index.name(key.index).is_some() => {
                format!("{}.{}", r.path, r.index.name(key.index).unwrap_or_default())
            }
            Index::Single | Index::Array(_) | Index::Named(_) => {
                format!("{}[{}]", r.path, key.index)
            }
        },
        None => format!("{:#04x}.{}", key.field_id, key.index),
    }
}

/// Every key the table names, in table order: each motor index of a motor field and each index of
/// a family.
pub fn table_keys() -> Vec<Key> {
    NAMES
        .iter()
        .flat_map(|r| {
            (0..r.index.count()).map(|index| Key {
                field_id: r.id,
                index,
            })
        })
        .collect()
}

/// The row named `path` among the fields of the object array `array` (`motors`, `links`), or
/// among the others with `None`.
pub fn find(path: &str, array: Option<&str>) -> Option<&'static FieldName> {
    NAMES
        .iter()
        .find(|r| r.path == path && r.index.array() == array)
}

/// The object array a field of index kind `index` sits in and what one element of it is
/// (`motors` and `motor`, `links` and `link`; `specs/machine-settings.md` section 3.1), or `None`
/// for a field at its own path. The array's key is the store's ([`Index::array`]).
pub fn element(index: Index) -> Option<(&'static str, &'static str)> {
    let what = match index {
        Index::Motor | Index::MotorArray(_) => "motor",
        Index::Link => "link",
        Index::Single | Index::Array(_) | Index::Named(_) => return None,
    };
    Some((index.array()?, what))
}

/// Every object array a file holds, as [`element`] names it: `motors`, then `links`.
pub fn object_arrays() -> impl Iterator<Item = (&'static str, &'static str)> {
    [Index::Motor, Index::Link].into_iter().filter_map(element)
}

/// The problem with writing index `index` of `row`, if it is past the table's count.
fn index_check(row: &FieldName, index: u8) -> Option<Problem> {
    if index < row.index.count() {
        return None;
    }
    Some(Problem::forceable(match row.index {
        Index::Single => format!(
            "{} has one index (--force writes index {index} anyway)",
            row.path
        ),
        Index::Array(n) => format!(
            "{} has {n} indices (--force writes index {index} anyway)",
            row.path
        ),
        Index::Named(names) => format!(
            "{} has {} values ({}) (--force writes index {index} anyway)",
            row.path,
            names.len(),
            names.join(", ")
        ),
        Index::Motor | Index::Link => {
            let (array, what) = element(row.index).expect("an object array");
            format!(
                "{} has {} {what}s (--force writes {array}[{index}] anyway)",
                row.path,
                row.index.count()
            )
        }
        Index::MotorArray(n) => format!(
            "{} has {n} indices on each of {} motors (--force writes index {index} anyway)",
            row.path,
            store::MOTORS
        ),
    }))
}

/// The problems with `key` that do not depend on a value: an index past the table's count, or an id
/// the table does not name.
pub fn key_checks(key: Key) -> Vec<Problem> {
    match row_of(key) {
        Some(row) => index_check(row, key.index).into_iter().collect(),
        None => vec![Problem::forceable(format!(
            "{:#04x} is not in the name table (--force writes it with the type the board reports)",
            key.field_id
        ))],
    }
}

/// The problems with WRITING `key` beyond [`key_checks`]: a board-owned field.
pub fn write_checks(key: Key) -> Vec<Problem> {
    let mut p = key_checks(key);
    if let Some(row) = row_of(key) {
        if row.board_owned() {
            p.push(Problem::forceable(format!(
                "{} is board-owned (--force writes it anyway)",
                path_of(key)
            )));
        }
    }
    p
}

/// A command-line path (`specs/cli-config.md` section 2.1): `motors.1.hall.a`, `links.0.tx`,
/// `battery_sense.cal.0`, `machine.mode`, `tone.horn`, or the raw `0x40.1` / `0x20`. The form a path prints as
/// (`motors[1].hall.a`) is taken too. The key, with [`key_checks`]'s problems; `Err` when the path
/// names no key.
pub fn parse_path(s: &str) -> Result<(Key, Vec<Problem>), Problem> {
    let norm = s.replace('[', ".").replace(']', "");
    if norm.starts_with(|c: char| c.is_ascii_digit()) {
        let key = parse_key(&norm).map_err(Problem::stop)?;
        return Ok((key, key_checks(key)));
    }
    let key = |row: &FieldName, index| Key {
        field_id: row.id,
        index,
    };
    for (array, _) in object_arrays() {
        if let Some(rest) = norm.strip_prefix(array).and_then(|r| r.strip_prefix('.')) {
            let (i, name) = rest.split_once('.').ok_or_else(|| unknown(s))?;
            let i = i.parse::<u8>().map_err(|_| unknown(s))?;
            if let Some(row) = find(name, Some(array)) {
                if let Index::MotorArray(n) = row.index {
                    return Err(Problem::stop(format!(
                        "{} has {n} values per motor ({array}.{i}.{}.<index>)",
                        row.path, row.path
                    )));
                }
                let k = key(row, i);
                return Ok((k, key_checks(k)));
            }
            // A per-motor family's element: `motors.<m>.<path>.<e>`, index `m * n + e`.
            let found = name.rsplit_once('.').and_then(|(name, e)| {
                let row = find(name, Some(array))?;
                let Index::MotorArray(n) = row.index else {
                    return None;
                };
                let e = e.parse::<u8>().ok().filter(|e| *e < n)?;
                Some(key(row, i.checked_mul(n)?.checked_add(e)?))
            });
            let k = found.ok_or_else(|| unknown(s))?;
            return Ok((k, key_checks(k)));
        }
    }
    if let Some(row) = find(&norm, None) {
        return match row.index {
            Index::Array(n) => Err(Problem::stop(format!(
                "{} has {n} indices ({}.<index>)",
                row.path, row.path
            ))),
            Index::Named(names) => Err(Problem::stop(format!(
                "{} has one value per name ({}.<{}>)",
                row.path,
                row.path,
                names.join("|")
            ))),
            _ => Ok((key(row, 0), Vec::new())),
        };
    }
    for (array, what) in object_arrays() {
        if let Some(row) = find(&norm, Some(array)) {
            return Err(Problem::stop(format!(
                "{} is per {what} ({array}.<n>.{})",
                row.path, row.path
            )));
        }
    }
    if let Some((name, i)) = norm.rsplit_once('.') {
        if let Some((row, i)) = find_named(name, i) {
            return Ok((key(row, i), Vec::new()));
        }
        if let (Some(row), Ok(i)) = (find(name, None), i.parse::<u8>()) {
            let k = key(row, i);
            return Ok((k, key_checks(k)));
        }
    }
    Err(unknown(s))
}

/// The top-level named field at `path` ([`Index::Named`]) and the index its name `name` stands
/// for: `tone` and `horn` are `tone.horn`.
pub fn find_named(path: &str, name: &str) -> Option<(&'static FieldName, u8)> {
    let row = find(path, None)?;
    Some((row, row.index.named(name)?))
}

/// "unknown field X", with the closest name when one is close.
pub fn unknown(s: &str) -> Problem {
    Problem::stop(format!("unknown field {s}{}", suggest(s)))
}

/// ` (did you mean X?)` for the name closest to `s`, or nothing when none is close. A motor or
/// link field is suggested under the motor or link `s` names.
fn suggest(s: &str) -> String {
    let element = |array: &str| {
        s.strip_prefix(array)
            .and_then(|r| r.strip_prefix('.'))
            .and_then(|r| r.split_once('.'))
            .map(|(i, _)| i.to_string())
    };
    let best = NAMES
        .iter()
        .flat_map(|r| -> Vec<String> {
            if let Index::Named(names) = r.index {
                return names.iter().map(|n| format!("{}.{n}", r.path)).collect();
            }
            vec![match r.index.array().and_then(|a| Some((a, element(a)?))) {
                Some((a, i)) => format!("{a}.{i}.{}", r.path),
                None => r.path.to_string(),
            }]
        })
        .min_by_key(|c| distance(s, c));
    match best {
        Some(b) if distance(s, &b) <= (s.len() / 3).max(2) => format!(" (did you mean {b}?)"),
        _ => String::new(),
    }
}

/// The edit distance between `a` and `b`.
fn distance(a: &str, b: &str) -> usize {
    let b: Vec<char> = b.chars().collect();
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.chars().enumerate() {
        let mut prev = row[0];
        row[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let here = row[j + 1];
            row[j + 1] = (prev + usize::from(ca != *cb))
                .min(row[j] + 1)
                .min(here + 1);
            prev = here;
        }
    }
    row[b.len()]
}

/// A pin as a line names it: `PA8`.
pub fn pin_text(port: u8, pin: u8) -> String {
    format!("P{}{pin}", port_letter(port))
}

/// A port number as a file writes it: `0` is `"A"`.
fn port_letter(port: u8) -> String {
    char::from(b'A' + port.min(25)).to_string()
}

/// A pin value's bytes as a file writes them (`specs/config-read-once.md` section 3): an object
/// of the port letter, the pin number, and the `af` of a [`Type::PinAf`] or the `adc` of a
/// [`Type::PinAdc`]. The absent default (port `0xFF`) is `null`.
fn pin_json(t: Type, b: &[u8]) -> Json {
    if b.first() == Some(&0xFF) {
        return Json::Null;
    }
    let mut o = Map::new();
    o.insert("port".into(), Json::String(port_letter(b[0])));
    o.insert("pin".into(), Json::from(b[1]));
    match t {
        Type::PinAf => {
            o.insert("af".into(), Json::from(b[2]));
        }
        Type::PinAdc => {
            o.insert("adc".into(), Json::from(b[2]));
        }
        _ => {}
    }
    Json::Object(o)
}

/// A pin object's bytes for storage type `t`: `Err` naming what is wrong.
fn pin_bytes(t: Type, v: &Json) -> Result<Vec<u8>, String> {
    let o = v
        .as_object()
        .ok_or("a pin object, {\"port\": \"A\", \"pin\": 8, ...}")?;
    let third = match t {
        Type::PinAf => Some("af"),
        Type::PinAdc => Some("adc"),
        _ => None,
    };
    if let Some(k) = o
        .keys()
        .find(|k| !["port", "pin"].contains(&k.as_str()) && Some(k.as_str()) != third)
    {
        return Err(format!("a pin object, without {k}"));
    }
    let port = o
        .get("port")
        .and_then(Json::as_str)
        .filter(|p| p.len() == 1)
        .and_then(|p| p.chars().next())
        .map(|c| c.to_ascii_uppercase())
        .filter(|c| ('A'..='F').contains(c))
        .ok_or("a port letter A to F")?;
    let pin = o
        .get("pin")
        .and_then(Json::as_u64)
        .filter(|p| *p <= 15)
        .ok_or("a pin number 0 to 15")?;
    let mut b = vec![port as u8 - b'A', pin as u8];
    match (t, third) {
        (Type::PinAf, Some(k)) => {
            // Left out on a chip that remaps, whose af byte is not used: stored as 0.
            let af = match o.get(k) {
                Some(x) => x.as_u64().filter(|a| *a <= 15).ok_or("an af 0 to 15")?,
                None => 0,
            };
            b.push(af as u8);
        }
        (Type::PinAdc, Some(k)) => {
            let adc = o
                .get(k)
                .and_then(Json::as_u64)
                .filter(|a| *a <= 255)
                .ok_or("an adc channel")?;
            b.push(adc as u8);
        }
        _ => {}
    }
    Ok(b)
}

/// An integer of a JSON value, if it is one.
fn json_int(v: &Json) -> Option<i128> {
    v.as_i64()
        .map(i128::from)
        .or_else(|| v.as_u64().map(i128::from))
}

/// A JSON number for `i` (beyond what JSON's integers hold, the text, which then fails to encode).
fn number(i: i128) -> Json {
    u64::try_from(i)
        .map(Json::from)
        .or_else(|_| i64::try_from(i).map(Json::from))
        .unwrap_or_else(|_| Json::String(i.to_string()))
}

/// Bytes as a file writes a blob: lowercase hex.
fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// A value as `CONFIG_RESP` carries it (`tag`, `bytes`), as a file writes it for `row`
/// (`specs/cli-config.md` section 2.2). A value the table cannot name (an unlabelled enum value, a
/// `bool` byte of 2, a bit with no name) is written as an integer, so nothing the board holds is
/// lost.
pub fn to_json(row: Option<&FieldName>, tag: u8, bytes: &[u8]) -> Json {
    let Some(v) = Type::from_tag(tag).and_then(|t| Value::decode(t, bytes)) else {
        return Json::String(hex(bytes));
    };
    let int = match v {
        Value::U8(x) => i128::from(x),
        Value::U16(x) => i128::from(x),
        Value::U32(x) => i128::from(x),
        Value::U64(x) => i128::from(x),
        Value::I16(x) => i128::from(x),
        Value::I32(x) => i128::from(x),
        Value::I64(x) => i128::from(x),
        Value::Bool(b) => return Json::Bool(b),
        Value::Str(s) => return Json::String(String::from_utf8_lossy(s).into_owned()),
        Value::Bytes(b) if row.is_some_and(|r| r.kind == Kind::Tune) => {
            return Json::String(crate::tune::render(b))
        }
        Value::Bytes(b) => return Json::String(hex(b)),
        Value::Pin(b) => return pin_json(Type::Pin, &b),
        Value::PinAf(b) => return pin_json(Type::PinAf, &b),
        Value::PinAdc(b) => return pin_json(Type::PinAdc, &b),
    };
    let byte = u8::try_from(int).ok();
    match (row.map(|r| r.kind), byte) {
        (Some(Kind::Bool), Some(0)) => Json::Bool(false),
        (Some(Kind::Bool), Some(1)) => Json::Bool(true),
        (Some(Kind::Enum(labels)), Some(b)) => match labels.iter().find(|(v, _)| *v == b) {
            Some((_, l)) => Json::String((*l).into()),
            None => number(int),
        },
        (Some(Kind::Bits(bits)), Some(b)) => {
            let named: u8 = bits.iter().fold(0, |m, (bit, _)| m | 1 << bit);
            if b & !named != 0 {
                return number(int);
            }
            let set = bits.iter().filter(|(bit, _)| b & (1 << bit) != 0);
            Json::Array(set.map(|(_, n)| Json::String((*n).into())).collect())
        }
        _ => number(int),
    }
}

/// A value for `key` as a line shows it: a label bare, a pin as `PB5`, `PA8 af2` or `PA4 adc4`
/// (`none` when absent), anything else as the file writes it.
pub fn show(row: Option<&FieldName>, v: &Json) -> String {
    match (row.map(|r| r.kind), v) {
        (Some(Kind::Enum(_)), Json::String(s)) => s.clone(),
        (Some(Kind::Pin), Json::Null) => "none".into(),
        (Some(Kind::Pin), Json::Object(o)) => {
            let mut t = format!(
                "P{}{}",
                o.get("port").and_then(Json::as_str).unwrap_or("?"),
                o.get("pin").map_or_else(String::new, Json::to_string)
            );
            for k in ["af", "adc"] {
                if let Some(x) = o.get(k) {
                    t = format!("{t} {k}{x}");
                }
            }
            t
        }
        _ => v.to_string(),
    }
}

/// A value on the command line as the file would write it, for `row` (or, for an id the table
/// does not name, the board's `storage` type): `null` un-stores, a pin is its JSON object, a label,
/// string or blob is text, bit names may be listed with commas, and everything else is a number.
pub fn from_text(row: Option<&FieldName>, storage: Type, text: &str) -> Json {
    if text == "null" {
        return Json::Null;
    }
    if row.is_some_and(|r| r.kind == Kind::Pin) {
        return serde_json::from_str(text).unwrap_or_else(|_| Json::String(text.into()));
    }
    let textual = matches!(
        row.map(|r| r.kind),
        Some(Kind::Str | Kind::Blob | Kind::Tune)
    ) || (row.is_none() && matches!(storage, Type::Str | Type::Blob));
    if textual {
        return Json::String(text.into());
    }
    if let Some(i) = parse_int(text) {
        return number(i);
    }
    match (row.map(|r| r.kind), text) {
        (Some(Kind::Bits(_)), _) => Json::Array(
            text.split(',')
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .map(|s| Json::String(s.into()))
                .collect(),
        ),
        (_, "true") => Json::Bool(true),
        (_, "false") => Json::Bool(false),
        _ => Json::String(text.into()),
    }
}

/// The lowest and highest integer a fixed storage type holds.
fn bounds(t: Type) -> Option<(i128, i128)> {
    Some(match t {
        Type::U8 => (0, u8::MAX.into()),
        Type::U16 => (0, u16::MAX.into()),
        Type::U32 => (0, u32::MAX.into()),
        Type::U64 => (0, u64::MAX.into()),
        Type::I16 => (i16::MIN.into(), i16::MAX.into()),
        Type::I32 => (i32::MIN.into(), i32::MAX.into()),
        Type::I64 => (i64::MIN.into(), i64::MAX.into()),
        Type::Bool => (0, 1),
        Type::Str | Type::Blob | Type::Pin | Type::PinAf | Type::PinAdc => return None,
    })
}

/// A file or command-line value `v` for `key`, as `CONFIG_WRITE` carries it: the bytes of a
/// `storage` value, or `None` for `null` (un-store). `Err` for a value that cannot be encoded (a
/// pin that does not parse, an unknown label, a string in an `int` field, a number outside the
/// storage type), which always stops; the `Ok` problems are the table's own range and label
/// checks, which `--force` lets through.
pub fn encode(
    key: Key,
    storage: Type,
    v: &Json,
) -> Result<(Option<Vec<u8>>, Vec<Problem>), Problem> {
    if v.is_null() {
        return Ok((None, Vec::new()));
    }
    let row = row_of(key);
    let path = path_of(key);
    // A secret's value is never printed (`specs/bridge-onboarding.md` section 2.2): a refusal names
    // what was given by its JSON type alone.
    let given = if row.is_some_and(|r| r.secret()) {
        match v {
            Json::Bool(_) => "a boolean".to_string(),
            Json::Number(_) => "a number".to_string(),
            Json::String(_) => "a string".to_string(),
            Json::Array(_) => "a list".to_string(),
            Json::Object(_) | Json::Null => "an object".to_string(),
        }
    } else {
        v.to_string()
    };
    let bad = |what: &str| Problem::stop(format!("{path}: {what}, not {given}"));
    let mut warn = Vec::new();
    let checked = |warn: &mut Vec<Problem>, ok: bool, what: String| {
        if !ok {
            warn.push(Problem::forceable(format!(
                "{path} takes {what}, not {v} (--force writes it anyway)"
            )));
        }
    };
    let int = match (row.map(|r| r.kind), v) {
        (Some(Kind::Pin), _) => {
            return pin_bytes(storage, v)
                .map(|b| (Some(b), warn))
                .map_err(|what| bad(&what));
        }
        (Some(Kind::Bool), Json::Bool(b)) => i128::from(*b),
        (Some(Kind::Bool), _) => {
            let i = json_int(v).ok_or_else(|| bad("true or false"))?;
            checked(&mut warn, i == 0 || i == 1, "true or false".into());
            i
        }
        (Some(Kind::Enum(labels)), Json::String(s)) => {
            let names = || {
                labels
                    .iter()
                    .map(|(_, l)| *l)
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            let (b, _) = labels
                .iter()
                .find(|(_, l)| l == s)
                .ok_or_else(|| Problem::stop(format!("{path}: unknown label {s} ({})", names())))?;
            i128::from(*b)
        }
        (Some(Kind::Enum(labels)), _) => {
            let i = json_int(v).ok_or_else(|| bad("a label or an integer"))?;
            let labelled = labels.iter().any(|(b, _)| i128::from(*b) == i);
            let names = labels.iter().map(|(_, l)| *l).collect::<Vec<_>>();
            checked(&mut warn, labelled, names.join(", "));
            i
        }
        (Some(Kind::Bits(bits)), Json::Array(items)) => {
            let mut m = 0i128;
            for item in items {
                let name = item.as_str().unwrap_or_default();
                let (bit, _) = bits.iter().find(|(_, n)| *n == name).ok_or_else(|| {
                    let names: Vec<_> = bits.iter().map(|(_, n)| *n).collect();
                    Problem::stop(format!("{path}: unknown bit {item} ({})", names.join(", ")))
                })?;
                m |= 1 << bit;
            }
            m
        }
        (Some(Kind::Bits(bits)), _) => {
            let i = json_int(v).ok_or_else(|| bad("a list of bit names or an integer"))?;
            let named: i128 = bits.iter().fold(0, |m, (bit, _)| m | 1 << bit);
            let names: Vec<_> = bits.iter().map(|(_, n)| *n).collect();
            checked(
                &mut warn,
                i & !named == 0,
                format!("the bits {}", names.join(", ")),
            );
            i
        }
        (Some(Kind::Str), _) | (None, _) if storage == Type::Str => {
            let s = v.as_str().ok_or_else(|| bad("a string"))?;
            return fits(key, s.as_bytes().to_vec()).map(|b| (Some(b), warn));
        }
        (Some(Kind::Tune), _) => {
            let t = v
                .as_str()
                .ok_or_else(|| bad("a tune, hz/dur notes or RTTTL"))?;
            let b = crate::tune::parse(t).map_err(|e| Problem::stop(format!("{path}: {e}")))?;
            return fits(key, b).map(|b| (Some(b), warn));
        }
        (Some(Kind::Blob), _) | (None, _) if storage == Type::Blob => {
            let h = v.as_str().ok_or_else(|| bad("a hex string"))?;
            let h = h.strip_prefix("0x").unwrap_or(h);
            let b = (0..h.len())
                .step_by(2)
                .map(|i| h.get(i..i + 2).and_then(|x| u8::from_str_radix(x, 16).ok()))
                .collect::<Option<Vec<u8>>>()
                .ok_or_else(|| bad("a hex string"))?;
            return fits(key, b).map(|b| (Some(b), warn));
        }
        (None, Json::Bool(b)) if storage == Type::Bool => i128::from(*b),
        _ => json_int(v).ok_or_else(|| bad("an integer"))?,
    };
    let (lo, hi) = bounds(storage).ok_or_else(|| bad(&format!("a {storage:?}")))?;
    if !(lo..=hi).contains(&int) {
        return Err(Problem::stop(format!(
            "{path}: {int} is outside {storage:?} ({lo}..{hi})"
        )));
    }
    // The range table. An arm-row or calibration field's range is a board check, made on the
    // board as the write would leave it and never overridden (`crate::checks`,
    // `specs/config-read-once.md` section 4); any other field's is this forceable one.
    let board_check = store::lookup_key(key).is_some_and(|d| d.effect == store::Effect::Arm)
        || store::CALIBRATION.contains(&key.field_id);
    if let Some((lo, hi)) = store::range_of(key).filter(|_| !board_check) {
        checked(
            &mut warn,
            (i128::from(lo)..=i128::from(hi)).contains(&int),
            format!("{lo}..{hi}"),
        );
    }
    let width = storage.fixed_len().unwrap_or(0);
    Ok((Some(int.to_le_bytes()[..width].to_vec()), warn))
}

/// `bytes` for `key` if the board can take them and answer: a write carries at most
/// [`CONFIG_WRITE_VALUE_MAX`] value bytes, and its echo and every later read of the key at most
/// [`CONFIG_RESP_VALUE_MAX`], except for a secret's, which carry no value. A longer value always
/// stops, naming its length but not the value.
fn fits(key: Key, bytes: Vec<u8>) -> Result<Vec<u8>, Problem> {
    let (max, carrier) = if row_of(key).is_some_and(|r| r.secret()) {
        (CONFIG_WRITE_VALUE_MAX, "one CONFIG_WRITE carries")
    } else {
        (CONFIG_RESP_VALUE_MAX, "a CONFIG_RESP carries back")
    };
    if bytes.len() > max {
        return Err(Problem::stop(format!(
            "{}: {} bytes, more than the {max} {carrier}",
            path_of(key),
            bytes.len()
        )));
    }
    Ok(bytes)
}

/// When a written field takes effect, as the effect lines and the field table say it
/// (`specs/machine-settings.md` section 5).
pub fn effect_name(e: store::Effect) -> &'static str {
    match e {
        store::Effect::Live => "live",
        store::Effect::Arm => "arm",
        store::Effect::Boot => "boot",
        store::Effect::Client => "client only",
    }
}

/// The name table as `board fields --json` prints it (`specs/cli-config.md` section 3, "The other
/// consumers"): one object per row, for the tools that check their own copy of the names.
pub fn table_json() -> Json {
    let rows = NAMES.iter().map(|r| {
        let mut o = Map::new();
        o.insert("id".into(), r.id.into());
        o.insert("path".into(), r.path.into());
        o.insert(
            "type".into(),
            format!("{:?}", r.storage).to_lowercase().into(),
        );
        let index = match r.index {
            Index::Single => "single",
            Index::Array(_) => "array",
            Index::Named(_) => "named",
            Index::MotorArray(_) => "motor_array",
            Index::Motor | Index::Link => element(r.index).expect("an object array").1,
        };
        o.insert("index".into(), index.into());
        if let Index::Named(names) = r.index {
            o.insert("names".into(), names.to_vec().into());
        }
        o.insert("count".into(), r.index.count().into());
        let (kind, labels) = match r.kind {
            Kind::Pin => ("pin", None),
            Kind::Int => ("int", None),
            Kind::Bool => ("bool", None),
            Kind::Enum(l) => ("enum", Some(l)),
            Kind::Bits(b) => ("bits", Some(b)),
            Kind::Str => ("str", None),
            Kind::Blob => ("blob", None),
            Kind::Tune => ("tune", None),
        };
        o.insert("kind".into(), kind.into());
        if let Some(l) = labels {
            let m: Map<String, Json> = l.iter().map(|(v, n)| ((*n).into(), (*v).into())).collect();
            o.insert(
                if kind == "enum" { "labels" } else { "bits" }.into(),
                m.into(),
            );
        }
        let ranges: Vec<Json> = store::RANGES
            .iter()
            .filter(|g| g.id == r.id)
            .map(|g| match g.index {
                Some(i) => serde_json::json!({ "index": i, "range": [g.lo, g.hi] }),
                None => serde_json::json!({ "range": [g.lo, g.hi] }),
            })
            .collect();
        if !ranges.is_empty() {
            o.insert("ranges".into(), Json::Array(ranges));
        }
        let scope = match r.scope() {
            Some(store::Scope::Node) => "node",
            Some(store::Scope::Machine) => "machine",
            Some(store::Scope::Wheel) => "wheel",
            None => "none",
        };
        o.insert("scope".into(), scope.into());
        o.insert("effect".into(), effect_name(r.effect()).into());
        o.insert("board_owned".into(), r.board_owned().into());
        Json::Object(o)
    });
    Json::Array(rows.collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn k(field_id: u8, index: u8) -> Key {
        Key { field_id, index }
    }

    #[test]
    fn every_key_in_the_table_maps_to_a_path_and_back() {
        let keys = table_keys();
        assert!(keys.len() > NAMES.len());
        for key in keys {
            let printed = path_of(key);
            let (back, problems) =
                parse_path(&printed).unwrap_or_else(|p| panic!("{printed}: {p:?}"));
            assert_eq!(back, key, "{printed}");
            assert!(problems.is_empty(), "{printed}: {problems:?}");
            // The command line's dotted form, and the raw form, name the same key.
            let dotted = printed.replace('[', ".").replace(']', "");
            assert_eq!(parse_path(&dotted).unwrap().0, key, "{dotted}");
            let raw = format!("{:#04x}.{}", key.field_id, key.index);
            assert_eq!(parse_path(&raw).unwrap(), (key, Vec::new()), "{raw}");
        }
    }

    #[test]
    fn the_spec_paths_name_their_keys() {
        assert_eq!(parse_path("motors.1.hall.a").unwrap().0, k(0x40, 1));
        assert_eq!(parse_path("battery_sense.cal.0").unwrap().0, k(0x18, 0));
        assert_eq!(parse_path("machine.mode").unwrap().0, k(0x20, 0));
        assert_eq!(parse_path("0x40.1").unwrap().0, k(0x40, 1));
        assert_eq!(parse_path("0x20").unwrap().0, k(0x20, 0));
        assert_eq!(parse_path("0x4B.0").unwrap().0, k(0x4B, 0));
        assert_eq!(path_of(k(0x4B, 0)), "motors[0].gate.dead_time");
        assert_eq!(path_of(k(0x27, 1)), "machine.balance.gains[1]");
        assert_eq!(path_of(k(0x99, 0)), "0x99.0");
        // A per-motor family (`specs/flux-observer.md`, "Fields"): motor m's element e is
        // index m * n + e.
        assert_eq!(path_of(k(0x61, 11)), "motors[1].observer.shadow[4]");
        assert_eq!(
            parse_path("motors.1.observer.shadow.4").unwrap().0,
            k(0x61, 11)
        );
        assert_eq!(
            parse_path("motors[0].observer.model[1]").unwrap().0,
            k(0x60, 1)
        );
        assert!(parse_path("motors.0.observer.model.4").is_err());
        assert!(parse_path("motors.0.observer.model").is_err());
    }

    /// `specs/machine-settings.md` section 3.1: the node paths are grouped by function, and a
    /// link's pins sit in its element of `links`, as a motor's fields do in `motors`.
    #[test]
    fn the_regrouped_node_paths_and_the_links_name_their_keys() {
        for (path, key) in [
            ("power.self_hold", k(0x06, 0)),
            ("power.button", k(0x0E, 0)),
            ("battery_sense.pin", k(0x07, 0)),
            ("battery_sense.cal.1", k(0x18, 1)),
            ("buzzer", k(0x08, 0)),
            ("leds.green", k(0x09, 0)),
            ("leds.red", k(0x0B, 0)),
            ("pads.a", k(0x0C, 0)),
            ("pads.b", k(0x0D, 0)),
            ("active_high", k(0x19, 0)),
            ("links.0.tx", k(0x0F, 0)),
            ("links.1.rx", k(0x10, 1)),
            ("links[1].tx", k(0x0F, 1)),
            ("machine.battery.floor", k(0x26, 0)),
            ("machine.turn.1", k(0x2B, 1)),
            ("motors.1.side", k(0x50, 1)),
        ] {
            assert_eq!(parse_path(path), Ok((key, Vec::new())), "{path}");
        }
        assert_eq!(path_of(k(0x0F, 0)), "links[0].tx");
        assert_eq!(path_of(k(0x10, 1)), "links[1].rx");
        assert_eq!(path_of(k(0x50, 0)), "motors[0].side");
        // A link's pin is per link, never a top-level path or a motor's.
        assert_eq!(
            parse_path("tx").unwrap_err().msg,
            "tx is per link (links.<n>.tx)"
        );
        assert_eq!(
            parse_path("motors.0.tx").unwrap_err().msg,
            "unknown field motors.0.tx"
        );
        let (key, p) = parse_path("links.2.tx").unwrap();
        assert_eq!(key, k(0x0F, 2));
        assert_eq!(
            p,
            [Problem::forceable(
                "tx has 2 links (--force writes links[2] anyway)"
            )]
        );
        // The old paths name nothing.
        for old in ["board.self_hold", "led.green", "control.mode", "link.tx.0"] {
            assert!(parse_path(old).is_err(), "{old}");
        }
    }

    /// `specs/tones.md` section 4: the tunes are one node field, `tone.<event>`, the event naming
    /// the index; the old machine paths name nothing.
    #[test]
    fn the_tunes_are_named_by_event() {
        for (path, key) in [
            ("tone.horn", k(0x1F, 0)),
            ("tone.battery_low", k(0x1F, 1)),
            ("tone.battery_critical", k(0x1F, 2)),
            ("tone.1", k(0x1F, 1)),
        ] {
            assert_eq!(parse_path(path), Ok((key, Vec::new())), "{path}");
        }
        assert_eq!(path_of(k(0x1F, 0)), "tone.horn");
        assert_eq!(path_of(k(0x1F, 2)), "tone.battery_critical");
        assert_eq!(path_of(k(0x1F, 3)), "tone[3]");
        assert_eq!(
            parse_path("tone").unwrap_err().msg,
            "tone has one value per name (tone.<horn|battery_low|battery_critical>)"
        );
        assert_eq!(
            parse_path("tone.hron").unwrap_err().msg,
            "unknown field tone.hron (did you mean tone.horn?)"
        );
        let (key, p) = parse_path("tone.3").unwrap();
        assert_eq!(key, k(0x1F, 3));
        assert_eq!(
            p,
            [Problem::forceable(
                "tone has 3 values (horn, battery_low, battery_critical) (--force writes index 3 anyway)"
            )]
        );
        assert_eq!(
            row_of(k(0x1F, 0)).unwrap().scope(),
            Some(store::Scope::Node)
        );
        for old in [
            "machine.tone.horn",
            "machine.tone.battery_low",
            "machine.tone.battery_critical",
        ] {
            assert!(parse_path(old).is_err(), "{old}");
        }
        for id in 0x2D..=0x2F {
            assert!(row_of(k(id, 0)).is_none(), "{id:#04x}");
        }
    }

    /// Every object array a row sits in is one a file holds.
    #[test]
    fn every_object_array_row_has_its_array() {
        let arrays: Vec<&str> = object_arrays().map(|(a, _)| a).collect();
        assert_eq!(arrays, ["motors", "links"]);
        for r in NAMES {
            if let Some(a) = r.index.array() {
                assert!(arrays.contains(&a), "{}", r.path);
            }
        }
    }

    #[test]
    fn a_path_that_names_no_key_is_refused_with_the_closest_name() {
        let e = parse_path("machine.mod").unwrap_err();
        assert_eq!(
            e.msg,
            "unknown field machine.mod (did you mean machine.mode?)"
        );
        assert!(!e.force);
        let e = parse_path("motors.1.hall.x").unwrap_err();
        assert_eq!(
            e.msg,
            "unknown field motors.1.hall.x (did you mean motors.1.hall.a?)"
        );
        let e = parse_path("links.0.xt").unwrap_err();
        assert_eq!(e.msg, "unknown field links.0.xt (did you mean links.0.tx?)");
        assert_eq!(
            parse_path("zzzzzzzz").unwrap_err().msg,
            "unknown field zzzzzzzz"
        );
        assert_eq!(
            parse_path("hall.a").unwrap_err().msg,
            "hall.a is per motor (motors.<n>.hall.a)"
        );
        assert_eq!(
            parse_path("battery_sense.cal").unwrap_err().msg,
            "battery_sense.cal has 2 indices (battery_sense.cal.<index>)"
        );
    }

    #[test]
    fn an_index_past_the_tables_count_is_forceable() {
        let (key, p) = parse_path("machine.mode.1").unwrap();
        assert_eq!(key, k(0x20, 1));
        assert_eq!(
            p,
            [Problem::forceable(
                "machine.mode has one index (--force writes index 1 anyway)"
            )]
        );
        assert!(p[0].stops(false) && !p[0].stops(true));
        let (_, p) = parse_path("battery_sense.cal.2").unwrap();
        assert_eq!(
            p[0].msg,
            "battery_sense.cal has 2 indices (--force writes index 2 anyway)"
        );
        let (_, p) = parse_path("motors.2.hall.a").unwrap();
        assert_eq!(
            p[0].msg,
            "hall.a has 2 motors (--force writes motors[2] anyway)"
        );
        let (key, p) = parse_path("0x99").unwrap();
        assert_eq!(key, k(0x99, 0));
        assert_eq!(
            p,
            [Problem::forceable(
                "0x99 is not in the name table (--force writes it with the type the board reports)"
            )]
        );
    }

    #[test]
    fn board_owned_is_forceable() {
        assert_eq!(
            write_checks(k(0x01, 0)),
            [Problem::forceable(
                "node.address is board-owned (--force writes it anyway)"
            )]
        );
        assert_eq!(
            write_checks(k(0x54, 1)),
            [Problem::forceable(
                "motors[1].odometer is board-owned (--force writes it anyway)"
            )]
        );
        assert!(write_checks(k(0x20, 0)).is_empty());
        // Ids 0x03 and 0x05 are ordinary fields (`specs/machine-settings.md` section 6.1).
        assert!(write_checks(k(0x03, 0)).is_empty());
        assert!(write_checks(k(0x05, 0)).is_empty());
    }

    #[test]
    fn pins_are_objects_of_their_type_both_ways() {
        use serde_json::json;
        let hall = k(0x40, 0);
        let (b, _) = encode(hall, Type::Pin, &json!({"port": "C", "pin": 13})).unwrap();
        assert_eq!(b, Some(vec![2, 13]));
        assert_eq!(
            to_json(row_of(hall), Type::Pin.tag(), &[2, 13]),
            json!({"port": "C", "pin": 13})
        );
        let gate = k(0x43, 0);
        let v = json!({"port": "A", "pin": 8, "af": 2});
        assert_eq!(
            encode(gate, Type::PinAf, &v).unwrap().0,
            Some(vec![0, 8, 2])
        );
        assert_eq!(to_json(row_of(gate), Type::PinAf.tag(), &[0, 8, 2]), v);
        // Left out (a chip that remaps), the af is stored as 0.
        let (b, _) = encode(gate, Type::PinAf, &json!({"port": "A", "pin": 8})).unwrap();
        assert_eq!(b, Some(vec![0, 8, 0]));
        let phase = k(0x49, 0);
        let v = json!({"port": "B", "pin": 0, "adc": 8});
        assert_eq!(
            encode(phase, Type::PinAdc, &v).unwrap().0,
            Some(vec![1, 0, 8])
        );
        assert_eq!(to_json(row_of(phase), Type::PinAdc.tag(), &[1, 0, 8]), v);
        // The absent default reads as null.
        assert_eq!(
            to_json(row_of(hall), Type::Pin.tag(), &[0xFF, 0xFF]),
            Json::Null
        );
        assert_eq!(pin_text(1, 12), "PB12");
    }

    #[test]
    fn a_pin_that_is_not_its_types_object_always_stops() {
        use serde_json::json;
        let stop = |key: Key, v: Json| {
            let row = row_of(key).unwrap();
            let e = encode(key, row.storage, &v).unwrap_err();
            assert!(!e.force, "{}", e.msg);
            e.msg
        };
        assert_eq!(
            stop(k(0x40, 0), json!("PC13")),
            "motors[0].hall.a: a pin object, {\"port\": \"A\", \"pin\": 8, ...}, not \"PC13\""
        );
        assert_eq!(
            stop(k(0x40, 0), json!({"port": "G", "pin": 1})),
            "motors[0].hall.a: a port letter A to F, not {\"port\":\"G\",\"pin\":1}"
        );
        assert_eq!(
            stop(k(0x40, 0), json!({"port": "C", "pin": 16})),
            "motors[0].hall.a: a pin number 0 to 15, not {\"port\":\"C\",\"pin\":16}"
        );
        assert_eq!(
            stop(k(0x40, 0), json!({"port": "C", "pin": 1, "af": 1})),
            "motors[0].hall.a: a pin object, without af, not {\"port\":\"C\",\"pin\":1,\"af\":1}"
        );
        assert_eq!(
            stop(k(0x49, 0), json!({"port": "A", "pin": 0})),
            "motors[0].current.phase_a: an adc channel, not {\"port\":\"A\",\"pin\":0}"
        );
    }

    /// Encode then show back: the file form of `v` for `key`.
    fn round(key: Key, v: Json) -> Json {
        let row = row_of(key).unwrap();
        let (bytes, p) = encode(key, row.storage, &v).unwrap();
        assert!(p.is_empty(), "{p:?}");
        to_json(Some(row), row.storage.tag(), &bytes.unwrap())
    }

    #[test]
    fn every_display_kind_round_trips() {
        use serde_json::json;
        for (key, v) in [
            (k(0x40, 1), json!({"port": "B", "pin": 5})),
            (k(0x07, 0), json!({"port": "A", "pin": 4, "adc": 4})),
            (k(0x0F, 1), json!({"port": "B", "pin": 6, "af": 0})),
            (k(0x20, 0), json!("balance")),
            (k(0x50, 1), json!("right")),
            (k(0x22, 0), json!(false)),
            (k(0x22, 0), json!(true)),
            (k(0x16, 1), json!("x")),
            (k(0x19, 0), json!(["power.button", "pads.b"])),
            (k(0x19, 0), json!(["pads.a", "pads.b"])),
            (k(0x19, 0), json!([])),
            (k(0x18, 1), json!(-5)),
            (k(0x4E, 0), json!(4_000_000_000u32)),
            (k(0x02, 0), json!("rover-left")),
            (k(0x2C, 0), json!(3300)),
            (k(0x1F, 2), json!("2000/15 0/10 2000/15 0/100")),
            (k(0x1F, 0), json!("")),
        ] {
            assert_eq!(round(key, v.clone()), v, "{}", path_of(key));
        }
        // A tune written as RTTTL reads back as its raw notes.
        assert_eq!(
            round(k(0x1F, 0), json!("t:d=4,o=5,b=120:c6,8p")),
            json!("1047/50 0/25")
        );
        assert_eq!(path_of(k(0x1F, 1)), "tone.battery_low");
        assert_eq!(
            from_text(row_of(k(0x1F, 0)), Type::Blob, "2000/50"),
            json!("2000/50")
        );
        // A refusal names the field and the note.
        let e = encode(k(0x1F, 0), Type::Blob, &json!("2000/0")).unwrap_err();
        assert!(e.msg.starts_with("tone.horn: note 1 (2000/0)"), "{}", e.msg);
        assert!(e.stops(true));
        // A blob: no named field is one, so an id the table does not name with the board's type.
        let raw = k(0x99, 0);
        let (bytes, _) = encode(raw, Type::Blob, &json!("deadbeef")).unwrap();
        assert_eq!(bytes, Some(vec![0xDE, 0xAD, 0xBE, 0xEF]));
        assert_eq!(
            to_json(None, Type::Blob.tag(), &bytes.unwrap()),
            json!("deadbeef")
        );
    }

    #[test]
    fn a_value_the_table_cannot_name_is_written_as_an_integer_and_round_trips() {
        use serde_json::json;
        let mode = row_of(k(0x20, 0));
        assert_eq!(to_json(mode, Type::U8.tag(), &[7]), json!(7));
        let rider = row_of(k(0x22, 0));
        assert_eq!(to_json(rider, Type::U8.tag(), &[2]), json!(2));
        let high = row_of(k(0x19, 0));
        assert_eq!(to_json(high, Type::U8.tag(), &[0b10_0001]), json!(33));
        // Writing them back is the table's own check, which --force lets through.
        for (key, v) in [(k(0x20, 0), 7), (k(0x22, 0), 2), (k(0x19, 0), 33)] {
            let (bytes, p) = encode(key, Type::U8, &json!(v)).unwrap();
            assert_eq!(bytes, Some(vec![v as u8]));
            assert_eq!(p.len(), 1, "{}", path_of(key));
            assert!(p[0].force);
        }
    }

    #[test]
    fn a_value_that_cannot_be_encoded_always_stops() {
        use serde_json::json;
        let stop = |key: Key, v: Json| {
            let row = row_of(key).unwrap();
            let e = encode(key, row.storage, &v).unwrap_err();
            assert!(!e.force, "{}", e.msg);
            e.msg
        };
        assert_eq!(
            stop(k(0x10, 0), json!("PC9")),
            "links[0].rx: a pin object, {\"port\": \"A\", \"pin\": 8, ...}, not \"PC9\""
        );
        assert_eq!(
            stop(k(0x20, 0), json!("balanse")),
            "machine.mode: unknown label balanse (throttle, balance)"
        );
        assert_eq!(
            stop(k(0x26, 0), json!("low")),
            "machine.battery.floor: an integer, not \"low\""
        );
        assert_eq!(
            stop(k(0x26, 0), json!(40000)),
            "machine.battery.floor: 40000 is outside I16 (-32768..32767)"
        );
        assert_eq!(
            stop(k(0x19, 0), json!(["power.button", "pads.c"])),
            "active_high: unknown bit \"pads.c\" (power.button, pads.a, pads.b, \
             motors[0].gate.break.pin, motors[1].gate.break.pin)"
        );
        assert_eq!(stop(k(0x02, 0), json!(5)), "device.name: a string, not 5");
    }

    /// `specs/bridge-onboarding.md` section 2.1: a value longer than the board can take in one
    /// `CONFIG_WRITE` and answer in one `CONFIG_RESP` stops before anything is sent, naming its
    /// length and the limit and never a secret's value.
    #[test]
    fn a_value_longer_than_one_pdu_carries_always_stops() {
        use serde_json::json;
        let at = |key: Key, v: Json| {
            let row = row_of(key).unwrap();
            encode(key, row.storage, &v)
        };
        let resp = CONFIG_RESP_VALUE_MAX;
        let write = CONFIG_WRITE_VALUE_MAX;
        let name = k(0x02, 0);
        assert!(at(name, json!("n".repeat(resp))).is_ok());
        let e = at(name, json!("n".repeat(resp + 1))).unwrap_err();
        assert!(!e.force);
        assert_eq!(
            e.msg,
            format!(
                "device.name: {} bytes, more than the {resp} a CONFIG_RESP carries back",
                resp + 1
            )
        );
        // An unnamed id with the board's type takes the same limit, a blob's as a string's.
        let raw = k(0x99, 0);
        assert!(encode(raw, Type::Str, &json!("r".repeat(resp + 1))).is_err());
        assert!(encode(raw, Type::Blob, &json!("ab".repeat(resp))).is_ok());
        let e = encode(raw, Type::Blob, &json!("ab".repeat(resp + 1))).unwrap_err();
        assert!(e.msg.contains(&format!("{} bytes", resp + 1)), "{}", e.msg);

        // A secret's echo carries no value: the write is its only bound.
        let pass = k(0x04, 0);
        assert!(at(pass, json!("p".repeat(write))).is_ok());
        let long = "p".repeat(write + 1);
        let e = at(pass, json!(long)).unwrap_err();
        assert_eq!(
            e.msg,
            format!(
                "wifi.pass: {} bytes, more than the {write} one CONFIG_WRITE carries",
                write + 1
            )
        );
        assert!(!e.msg.contains(&long));
    }

    /// The range table outside the arm row and the calibration is forceable; inside them it is a
    /// board check (`crate::checks`), which encoding leaves alone.
    #[test]
    fn the_tables_range_is_forceable_outside_the_board_checks() {
        use serde_json::json;
        let power = k(0x05, 0);
        let (bytes, p) = encode(power, Type::U8, &json!(90)).unwrap();
        assert_eq!(bytes, Some(vec![90]));
        assert_eq!(
            p,
            [Problem::forceable(
                "wifi.tx_power takes 8..84, not 90 (--force writes it anyway)"
            )]
        );
        assert!(encode(power, Type::U8, &json!(34)).unwrap().1.is_empty());
        let (bytes, p) = encode(k(0x51, 0), Type::U8, &json!(6)).unwrap();
        assert_eq!((bytes, p), (Some(vec![6]), Vec::new()));
    }

    #[test]
    fn null_unstores_and_command_line_text_takes_the_files_forms() {
        assert_eq!(
            encode(k(0x20, 0), Type::U8, &Json::Null).unwrap(),
            (None, Vec::new())
        );
        let text = |key: Key, t: &str| from_text(row_of(key), row_of(key).unwrap().storage, t);
        assert_eq!(text(k(0x20, 0), "balance"), Json::from("balance"));
        assert_eq!(text(k(0x20, 0), "1"), Json::from(1));
        assert_eq!(text(k(0x18, 1), "-5"), Json::from(-5));
        assert_eq!(text(k(0x40, 1), "PB5"), Json::from("PB5"));
        assert_eq!(text(k(0x0F, 0), "PA2"), Json::from("PA2"));
        assert_eq!(text(k(0x4C, 0), "null"), Json::Null);
        assert_eq!(text(k(0x22, 0), "true"), Json::Bool(true));
        assert_eq!(
            text(k(0x19, 0), "power.button,pads.a"),
            serde_json::json!(["power.button", "pads.a"])
        );
        assert_eq!(text(k(0x02, 0), "12"), Json::from("12"));
        // An id the table does not name takes the board's type.
        assert_eq!(from_text(None, Type::U16, "0x10"), Json::from(16));
        assert_eq!(from_text(None, Type::Str, "7"), Json::from("7"));
    }

    #[test]
    fn the_table_json_carries_every_row() {
        let t = table_json();
        let rows = t.as_array().unwrap();
        assert_eq!(rows.len(), NAMES.len());
        let row = |path: &str| rows.iter().find(|r| r["path"] == path).unwrap();
        let mode = row("machine.mode");
        assert_eq!(mode["id"], 0x20);
        assert_eq!(mode["kind"], "enum");
        assert_eq!(mode["labels"]["balance"], 1);
        assert_eq!(
            (&mode["scope"], &mode["effect"]),
            (&Json::from("machine"), &Json::from("arm"))
        );
        let hall = row("hall.a");
        assert_eq!(
            (&hall["index"], &hall["count"], &hall["scope"]),
            (&Json::from("motor"), &Json::from(2), &Json::from("wheel"))
        );
        assert_eq!(hall["effect"], "boot");
        let tx = row("tx");
        assert_eq!(
            (&tx["index"], &tx["count"], &tx["scope"]),
            (&Json::from("link"), &Json::from(2), &Json::from("node"))
        );
        assert_eq!(row("machine.balance.gains")["effect"], "live");
        assert_eq!(row("pole_pairs")["effect"], "client only");
        let high = row("active_high");
        assert_eq!(high["bits"]["pads.b"], 2);
        let odo = row("odometer");
        assert_eq!(odo["board_owned"], true);
        let align = row("align_offset");
        assert_eq!(align["ranges"], serde_json::json!([{ "range": [0, 5] }]));
        assert_eq!(
            row("machine.turn")["ranges"],
            serde_json::json!([{ "index": 0, "range": [0, 28500] }, { "index": 1, "range": [1, 1000] }])
        );
        assert_eq!(row("gate.hi_a")["type"], "pinaf");
    }
}
