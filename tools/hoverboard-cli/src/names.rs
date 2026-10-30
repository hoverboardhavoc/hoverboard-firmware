//! The name table's rules (`specs/cli-config.md` sections 2.1 and 2.2): the path a key goes by on
//! the command line and in a file, and the form a value takes in a file. The table itself is the
//! store's (`store::NAMES`, `crates/store/src/field.rs`); this module only applies it.

use net::walk::{CONFIG_RESP_VALUE_MAX, CONFIG_WRITE_VALUE_MAX};
use serde_json::{Map, Value as Json};
use store::{by_id, FieldName, Index, Key, Kind, Type, Value, MOTORS, NAMES, PIN_ABSENT};

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

/// The path `key` prints as: `control.mode`, `board.battery.cal[1]`, `motors[0].hall.a`, or the raw
/// `0x99.0` for an id the table does not name. A single field at an index past its one prints with
/// the index, so a forced write names what it wrote.
pub fn path_of(key: Key) -> String {
    match row_of(key) {
        Some(r) => match r.index {
            Index::Motor => format!("motors[{}].{}", key.index, r.path),
            Index::Single if key.index == 0 => r.path.to_string(),
            Index::Single | Index::Array(_) => format!("{}[{}]", r.path, key.index),
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

/// The row named `path` among the motor fields (`motor`) or the others.
pub fn find(path: &str, motor: bool) -> Option<&'static FieldName> {
    NAMES
        .iter()
        .find(|r| r.path == path && (r.index == Index::Motor) == motor)
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
        Index::Motor => format!(
            "{} has {MOTORS} motors (--force writes motors[{index}] anyway)",
            row.path
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

/// The problems with WRITING `key` beyond [`key_checks`]: a board-owned field, or a read-only one.
pub fn write_checks(key: Key) -> Vec<Problem> {
    let mut p = key_checks(key);
    if let Some(row) = row_of(key) {
        if row.read_only() {
            p.push(Problem::stop(format!("{} is read-only", row.path)));
        } else if row.board_owned() {
            p.push(Problem::forceable(format!(
                "{} is board-owned (--force writes it anyway)",
                path_of(key)
            )));
        }
    }
    p
}

/// A command-line path (`specs/cli-config.md` section 2.1): `motors.1.hall.a`,
/// `board.battery.cal.0`, `control.mode`, or the raw `0x4A.1` / `0x22`. The form a path prints as
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
    if let Some(rest) = norm.strip_prefix("motors.") {
        let found = rest.split_once('.').and_then(|(i, name)| {
            let row = find(name, true)?;
            Some((row, i.parse::<u8>().ok()?))
        });
        let (row, i) = found.ok_or_else(|| unknown(s))?;
        let k = key(row, i);
        return Ok((k, key_checks(k)));
    }
    if let Some(row) = find(&norm, false) {
        return match row.index {
            Index::Array(n) => Err(Problem::stop(format!(
                "{} has {n} indices ({}.<index>)",
                row.path, row.path
            ))),
            _ => Ok((key(row, 0), Vec::new())),
        };
    }
    if let Some(row) = find(&norm, true) {
        return Err(Problem::stop(format!(
            "{} is per motor (motors.<n>.{})",
            row.path, row.path
        )));
    }
    if let Some((name, i)) = norm.rsplit_once('.') {
        if let (Some(row), Ok(i)) = (find(name, false), i.parse::<u8>()) {
            let k = key(row, i);
            return Ok((k, key_checks(k)));
        }
    }
    Err(unknown(s))
}

/// "unknown field X", with the closest name when one is close.
pub fn unknown(s: &str) -> Problem {
    Problem::stop(format!("unknown field {s}{}", suggest(s)))
}

/// ` (did you mean X?)` for the name closest to `s`, or nothing when none is close. A motor field
/// is suggested under the motor `s` names.
fn suggest(s: &str) -> String {
    let motor = s
        .strip_prefix("motors.")
        .and_then(|r| r.split_once('.'))
        .map(|(i, _)| i);
    let best = NAMES
        .iter()
        .map(|r| match (r.index, motor) {
            (Index::Motor, Some(i)) => format!("motors.{i}.{}", r.path),
            _ => r.path.to_string(),
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

/// A pin byte as a file writes it: `"PC13"`, or `"none"` for `0xFF`.
pub fn pin_name(b: u8) -> String {
    if b == PIN_ABSENT {
        "none".into()
    } else {
        format!("P{}{}", char::from(b'A' + (b >> 4)), b & 0x0F)
    }
}

/// The byte a pin token packs to: `(port << 4) | pin`, `none` for `0xFF`.
pub fn parse_pin(s: &str) -> Option<u8> {
    if s.eq_ignore_ascii_case("none") {
        return Some(PIN_ABSENT);
    }
    let rest = s.strip_prefix(['P', 'p'])?;
    let mut chars = rest.chars();
    let port = chars.next()?.to_ascii_uppercase();
    let pin: u8 = chars.as_str().parse().ok()?;
    if !('A'..='P').contains(&port) || pin > 15 || chars.as_str().starts_with('+') {
        return None;
    }
    let b = ((port as u8 - b'A') << 4) | pin;
    (b != PIN_ABSENT).then_some(b)
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
        Value::Bytes(b) => return Json::String(hex(b)),
    };
    let byte = u8::try_from(int).ok();
    match (row.map(|r| r.kind), byte) {
        (Some(Kind::Pin), Some(b)) => Json::String(pin_name(b)),
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

/// A value for `key` as a line shows it: a pin or a label bare, anything else as the file writes it.
pub fn show(row: Option<&FieldName>, v: &Json) -> String {
    match (row.map(|r| r.kind), v) {
        (Some(Kind::Pin | Kind::Enum(_)), Json::String(s)) => s.clone(),
        _ => v.to_string(),
    }
}

/// A value on the command line as the file would write it, for `row` (or, for an id the table
/// does not name, the board's `storage` type): `null` un-stores, a pin, label, string or blob is
/// text, bit names may be listed with commas, and everything else is a number.
pub fn from_text(row: Option<&FieldName>, storage: Type, text: &str) -> Json {
    if text == "null" {
        return Json::Null;
    }
    let textual = matches!(
        row.map(|r| r.kind),
        Some(Kind::Pin | Kind::Str | Kind::Blob)
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
        Type::Str | Type::Blob => return None,
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
            let b = v
                .as_str()
                .and_then(parse_pin)
                .ok_or_else(|| bad("a pin (PC13 or none)"))?;
            i128::from(b)
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
    if let Some((lo, hi)) = row.and_then(|r| r.range) {
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

/// The name table as `fields --json` prints it (`specs/cli-config.md` section 3, "The other
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
        let (index, count) = match r.index {
            Index::Single => ("single", 1),
            Index::Array(n) => ("array", n),
            Index::Motor => ("motor", MOTORS),
        };
        o.insert("index".into(), index.into());
        o.insert("count".into(), count.into());
        let (kind, labels) = match r.kind {
            Kind::Pin => ("pin", None),
            Kind::Int => ("int", None),
            Kind::Bool => ("bool", None),
            Kind::Enum(l) => ("enum", Some(l)),
            Kind::Bits(b) => ("bits", Some(b)),
            Kind::Str => ("str", None),
            Kind::Blob => ("blob", None),
        };
        o.insert("kind".into(), kind.into());
        if let Some(l) = labels {
            let m: Map<String, Json> = l.iter().map(|(v, n)| ((*n).into(), (*v).into())).collect();
            o.insert(
                if kind == "enum" { "labels" } else { "bits" }.into(),
                m.into(),
            );
        }
        if let Some((lo, hi)) = r.range {
            o.insert("range".into(), vec![lo, hi].into());
        }
        o.insert("board_owned".into(), r.board_owned().into());
        o.insert("read_only".into(), r.read_only().into());
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
        assert_eq!(parse_path("motors.1.hall.a").unwrap().0, k(0x4A, 1));
        assert_eq!(parse_path("board.battery.cal.0").unwrap().0, k(0x69, 0));
        assert_eq!(parse_path("control.mode").unwrap().0, k(0x22, 0));
        assert_eq!(parse_path("0x4A.1").unwrap().0, k(0x4A, 1));
        assert_eq!(parse_path("0x22").unwrap().0, k(0x22, 0));
        assert_eq!(parse_path("0x64.0").unwrap().0, k(0x64, 0));
        assert_eq!(path_of(k(0x64, 0)), "motors[0].gate.dead_time");
        assert_eq!(path_of(k(0x79, 1)), "control.sf_gains[1]");
        assert_eq!(path_of(k(0x99, 0)), "0x99.0");
    }

    #[test]
    fn a_path_that_names_no_key_is_refused_with_the_closest_name() {
        let e = parse_path("control.mod").unwrap_err();
        assert_eq!(
            e.msg,
            "unknown field control.mod (did you mean control.mode?)"
        );
        assert!(!e.force);
        let e = parse_path("motors.1.hall.x").unwrap_err();
        assert_eq!(
            e.msg,
            "unknown field motors.1.hall.x (did you mean motors.1.hall.a?)"
        );
        assert_eq!(
            parse_path("zzzzzzzz").unwrap_err().msg,
            "unknown field zzzzzzzz"
        );
        assert_eq!(
            parse_path("hall.a").unwrap_err().msg,
            "hall.a is per motor (motors.<n>.hall.a)"
        );
        assert_eq!(
            parse_path("board.battery.cal").unwrap_err().msg,
            "board.battery.cal has 2 indices (board.battery.cal.<index>)"
        );
    }

    #[test]
    fn an_index_past_the_tables_count_is_forceable() {
        let (key, p) = parse_path("control.mode.1").unwrap();
        assert_eq!(key, k(0x22, 1));
        assert_eq!(
            p,
            [Problem::forceable(
                "control.mode has one index (--force writes index 1 anyway)"
            )]
        );
        assert!(p[0].stops(false) && !p[0].stops(true));
        let (_, p) = parse_path("board.battery.cal.2").unwrap();
        assert_eq!(
            p[0].msg,
            "board.battery.cal has 2 indices (--force writes index 2 anyway)"
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
    fn board_owned_is_forceable_and_read_only_always_stops() {
        assert_eq!(
            write_checks(k(0x01, 0)),
            [Problem::forceable(
                "node.address is board-owned (--force writes it anyway)"
            )]
        );
        assert_eq!(
            write_checks(k(0x6D, 1)),
            [Problem::forceable(
                "motors[1].odometer is board-owned (--force writes it anyway)"
            )]
        );
        let p = write_checks(k(0x03, 0));
        assert_eq!(p, [Problem::stop("node.class is read-only")]);
        assert!(p[0].stops(true));
        assert!(write_checks(k(0x22, 0)).is_empty());
    }

    #[test]
    fn pins_round_trip_every_byte() {
        for b in 0..=255u8 {
            let name = pin_name(b);
            assert_eq!(parse_pin(&name), Some(b), "{name}");
        }
        assert_eq!(pin_name(0x2D), "PC13");
        assert_eq!(pin_name(0x1C), "PB12");
        assert_eq!(pin_name(0xFF), "none");
        assert_eq!(parse_pin("pa15"), Some(0x0F));
        for bad in ["PC16", "PQ1", "C13", "P", "PC", "PC+1", "PP15"] {
            assert_eq!(parse_pin(bad), None, "{bad}");
        }
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
            (k(0x4A, 1), json!("PB5")),
            (k(0x41, 0), json!("none")),
            (k(0x22, 0), json!("balance")),
            (k(0x23, 0), json!(false)),
            (k(0x66, 0), json!(true)),
            (k(0x68, 1), json!("x")),
            (k(0x6C, 0), json!(["button", "pad_b"])),
            (k(0x6C, 0), json!([])),
            (k(0x69, 1), json!(-5)),
            (k(0x20, 0), json!(4_000_000_000u32)),
            (k(0x10, 0), json!("rover-left")),
            (k(0x30, 0), json!("deadbeef")),
        ] {
            assert_eq!(round(key, v.clone()), v, "{}", path_of(key));
        }
    }

    #[test]
    fn a_value_the_table_cannot_name_is_written_as_an_integer_and_round_trips() {
        use serde_json::json;
        let mode = row_of(k(0x22, 0));
        assert_eq!(to_json(mode, Type::U8.tag(), &[7]), json!(7));
        let rider = row_of(k(0x23, 0));
        assert_eq!(to_json(rider, Type::U8.tag(), &[2]), json!(2));
        let pol = row_of(k(0x6C, 0));
        assert_eq!(to_json(pol, Type::U8.tag(), &[0b1001]), json!(9));
        // Writing them back is the table's own check, which --force lets through.
        for (key, v) in [(k(0x22, 0), 7), (k(0x23, 0), 2), (k(0x6C, 0), 9)] {
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
            stop(k(0x4A, 0), json!("PC99")),
            "motors[0].hall.a: a pin (PC13 or none), not \"PC99\""
        );
        assert_eq!(
            stop(k(0x22, 0), json!("balanse")),
            "control.mode: unknown label balanse (throttle, balance)"
        );
        assert_eq!(
            stop(k(0x24, 0), json!("low")),
            "board.battery.floor: an integer, not \"low\""
        );
        assert_eq!(
            stop(k(0x24, 0), json!(40000)),
            "board.battery.floor: 40000 is outside I16 (-32768..32767)"
        );
        assert_eq!(
            stop(k(0x6C, 0), json!(["button", "pad_c"])),
            "board.input_polarity: unknown bit \"pad_c\" (button, pad_a, pad_b)"
        );
        assert_eq!(stop(k(0x10, 0), json!(5)), "device.name: a string, not 5");
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
        let name = k(0x10, 0);
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
        let blob = k(0x30, 0);
        assert!(at(blob, json!("ab".repeat(resp))).is_ok());
        let e = at(blob, json!("ab".repeat(resp + 1))).unwrap_err();
        assert!(e.msg.contains(&format!("{} bytes", resp + 1)), "{}", e.msg);
        // An unnamed id with the board's type takes the same limit.
        let raw = k(0x99, 0);
        assert!(encode(raw, Type::Str, &json!("r".repeat(resp + 1))).is_err());

        // A secret's echo carries no value: the write is its only bound.
        let pass = k(0x12, 0);
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

    #[test]
    fn the_tables_range_is_forceable() {
        use serde_json::json;
        let key = k(0x63, 0);
        let (bytes, p) = encode(key, Type::U8, &json!(6)).unwrap();
        assert_eq!(bytes, Some(vec![6]));
        assert_eq!(
            p,
            [Problem::forceable(
                "motors[0].align_offset takes 0..5, not 6 (--force writes it anyway)"
            )]
        );
        assert!(encode(key, Type::U8, &json!(5)).unwrap().1.is_empty());
    }

    #[test]
    fn null_unstores_and_command_line_text_takes_the_files_forms() {
        assert_eq!(
            encode(k(0x22, 0), Type::U8, &Json::Null).unwrap(),
            (None, Vec::new())
        );
        let text = |key: Key, t: &str| from_text(row_of(key), row_of(key).unwrap().storage, t);
        assert_eq!(text(k(0x22, 0), "balance"), Json::from("balance"));
        assert_eq!(text(k(0x22, 0), "1"), Json::from(1));
        assert_eq!(text(k(0x69, 1), "-5"), Json::from(-5));
        assert_eq!(text(k(0x4A, 1), "PB5"), Json::from("PB5"));
        assert_eq!(text(k(0x67, 0), "null"), Json::Null);
        assert_eq!(text(k(0x23, 0), "true"), Json::Bool(true));
        assert_eq!(
            text(k(0x6C, 0), "button,pad_a"),
            serde_json::json!(["button", "pad_a"])
        );
        assert_eq!(text(k(0x10, 0), "12"), Json::from("12"));
        // An id the table does not name takes the board's type.
        assert_eq!(from_text(None, Type::U16, "0x10"), Json::from(16));
        assert_eq!(from_text(None, Type::Str, "7"), Json::from("7"));
    }

    #[test]
    fn the_table_json_carries_every_row() {
        let t = table_json();
        let rows = t.as_array().unwrap();
        assert_eq!(rows.len(), NAMES.len());
        let mode = rows.iter().find(|r| r["path"] == "control.mode").unwrap();
        assert_eq!(mode["id"], 0x22);
        assert_eq!(mode["kind"], "enum");
        assert_eq!(mode["labels"]["balance"], 1);
        let hall = rows.iter().find(|r| r["path"] == "hall.a").unwrap();
        assert_eq!(
            (&hall["index"], &hall["count"]),
            (&Json::from("motor"), &Json::from(2))
        );
        let odo = rows.iter().find(|r| r["path"] == "odometer").unwrap();
        assert_eq!(odo["board_owned"], true);
        let align = rows.iter().find(|r| r["path"] == "align_offset").unwrap();
        assert_eq!(align["range"], serde_json::json!([0, 5]));
    }
}
