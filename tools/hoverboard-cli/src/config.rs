//! A board's store as a file (`specs/cli-config.md`): `dump`, `apply`, `replace`, `diff`, and one
//! field at a time with `config`. A file holds one value per stored `(field_id, index)` at
//! the path the name table gives it ([`crate::names`]) and nothing else.
//!
//! Every command checks the files completely before it reads the board, reads every key the table
//! names (and every key a file names), plans, and only then writes: one `CONFIG_WRITE` per key,
//! values and un-stores alike, then reads every written key back and reports what the board holds.

use std::collections::{BTreeMap, HashMap};

use net::walk::{CFG_DEFAULT, CFG_OK, CFG_UNKNOWN_KEY};
use serde_json::{Map, Value as Json};
use store::{Index, Key, Kind, Type, ERASED_TAG};

use crate::names::{
    encode, find, path_of, row_of, show, table_keys, to_json, unknown, write_checks, Problem,
};
use crate::nodes::{self, parse_key, status_name, ConfigResp};
use crate::wire::Wire;

/// One board as the configuration commands reach it: the link, the tool's guest address, the
/// board's address, and the name every line about it starts with (its `device.name`).
pub struct Board<'w, W: Wire> {
    /// The link.
    pub wire: &'w mut W,
    /// The tool's address.
    pub src: u8,
    /// The board's address.
    pub dst: u8,
    /// The board's `device.name`, or its address when it answered none.
    pub name: String,
}

impl<W: Wire> Board<'_, W> {
    fn read(&mut self, key: Key) -> Result<ConfigResp, String> {
        nodes::read(self.wire, self.src, self.dst, key)
    }

    /// Write `bytes` of `storage` to `key`, or un-store it (`None`).
    fn write(&mut self, key: Key, storage: Type, bytes: Option<&[u8]>) -> Result<u8, String> {
        let r = match bytes {
            Some(b) => nodes::write(self.wire, self.src, self.dst, key, storage, b)?,
            None => nodes::unstore(self.wire, self.src, self.dst, key)?,
        };
        Ok(r.status)
    }
}

/// One key a file names, with its value as the file gives it.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    /// The key.
    pub key: Key,
    /// The value, `null` to un-store.
    pub value: Json,
}

/// What the files name, and every problem found in them.
#[derive(Debug, Default)]
pub struct Files {
    /// Every key the files name, in file order.
    pub entries: Vec<Entry>,
    /// Every problem, each prefixed with its file.
    pub problems: Vec<Problem>,
}

/// Read `files` (`(name, text)`) as one board's store (`specs/cli-config.md` sections 2 and 5):
/// merged in order, a key in two files (or twice in one, by its path and its raw form) refused.
pub fn read_files(files: &[(String, String)]) -> Files {
    let mut out = Files::default();
    let mut seen: HashMap<Key, String> = HashMap::new();
    for (name, text) in files {
        let mut f = FileReader {
            file: name,
            out: &mut out,
            seen: &mut seen,
        };
        match serde_json::from_str::<Json>(text) {
            Ok(Json::Object(top)) => f.top(&top),
            Ok(_) => f.stop("a board's store is a JSON object".into()),
            Err(e) => f.stop(format!("not JSON: {e}")),
        }
    }
    out
}

/// One file's walk (`specs/cli-config.md` section 2.1, the two rules run backwards).
struct FileReader<'a> {
    file: &'a str,
    out: &'a mut Files,
    seen: &'a mut HashMap<Key, String>,
}

impl FileReader<'_> {
    fn stop(&mut self, msg: String) {
        self.problem(Problem::stop(msg));
    }

    fn problem(&mut self, p: Problem) {
        self.out.problems.push(Problem {
            msg: format!("{}: {}", self.file, p.msg),
            force: p.force,
        });
    }

    fn top(&mut self, top: &Map<String, Json>) {
        for (k, v) in top {
            if k == "motors" {
                let Some(motors) = v.as_array() else {
                    self.stop("motors takes a list, one object per motor".into());
                    continue;
                };
                for (m, obj) in motors.iter().enumerate() {
                    match (u8::try_from(m), obj.as_object()) {
                        (Ok(m), Some(obj)) => self.object(obj, "", Some(m)),
                        _ => self.stop(format!("motors[{m}] is not a motor's object")),
                    }
                }
            } else if k.starts_with(|c: char| c.is_ascii_digit()) {
                match parse_key(k) {
                    Ok(key) => self.add(key, v),
                    Err(_) => self.problem(unknown(k)),
                }
            } else {
                self.member(k, v, "", None);
            }
        }
    }

    fn object(&mut self, obj: &Map<String, Json>, prefix: &str, motor: Option<u8>) {
        for (k, v) in obj {
            self.member(k, v, prefix, motor);
        }
    }

    /// One member `k: v` of an object at `prefix`, under `motor` when inside `motors`.
    fn member(&mut self, k: &str, v: &Json, prefix: &str, motor: Option<u8>) {
        let path = if prefix.is_empty() {
            k.to_string()
        } else {
            format!("{prefix}.{k}")
        };
        if let Some(row) = find(&path, motor.is_some()) {
            let at = |index| Key {
                field_id: row.id,
                index,
            };
            match row.index {
                Index::Array(n) => match v.as_array() {
                    Some(items) if items.len() <= 256 => {
                        for (i, item) in items.iter().enumerate() {
                            self.add(at(i as u8), item);
                        }
                    }
                    _ => self.stop(format!("{path} takes a list of {n} values")),
                },
                _ => self.add(at(motor.unwrap_or(0)), v),
            }
            return;
        }
        let inner = format!("{path}.");
        let nests = |per_motor: bool| {
            store::NAMES
                .iter()
                .any(|r| (r.index == Index::Motor) == per_motor && r.path.starts_with(&inner))
        };
        match v.as_object() {
            Some(obj) if nests(motor.is_some()) => self.object(obj, &path, motor),
            _ if find(&path, motor.is_none()).is_some() || nests(motor.is_none()) => {
                self.stop(if motor.is_none() {
                    format!("{path} is per motor (motors[<n>].{path})")
                } else {
                    format!("{path} is not per motor (it goes at the top level)")
                })
            }
            _ => {
                let shown = match motor {
                    Some(m) => format!("motors.{m}.{path}"),
                    None => path,
                };
                self.problem(unknown(&shown));
            }
        }
    }

    /// The file names `key` with `v`.
    fn add(&mut self, key: Key, v: &Json) {
        let path = path_of(key);
        if let Some(first) = self.seen.insert(key, self.file.to_string()) {
            let msg = if first == self.file {
                format!("{path} is named twice")
            } else {
                format!("{path} is also in {first}")
            };
            self.stop(msg);
            return;
        }
        for p in write_checks(key) {
            self.problem(p);
        }
        // A named key's value is checked now, before the board is read; an id the table does not
        // name takes the type the board reports, so its value is checked after the read.
        if let Some(row) = row_of(key) {
            match encode(key, row.storage, v) {
                Ok((_, warn)) => warn.into_iter().for_each(|p| self.problem(p)),
                Err(p) => self.problem(p),
            }
        }
        self.out.entries.push(Entry {
            key,
            value: v.clone(),
        });
    }
}

/// What the board answered for every key read (`CFG_OK` or `CFG_DEFAULT`, with the value).
pub type Reads = BTreeMap<Key, ConfigResp>;

/// The keys `dump` reads: every key the table names, less the board-owned and read-only fields.
pub fn dump_keys() -> Vec<Key> {
    table_keys()
        .into_iter()
        .filter(|k| row_of(*k).is_some_and(|r| !r.board_owned() && !r.read_only()))
        .collect()
}

/// The keys `diff`, `apply` and `replace` read: every key the table names but `node.class` (which
/// the node answers and never stores), and every key `files` name.
pub fn plan_keys(files: &Files) -> Vec<Key> {
    let mut keys: Vec<Key> = table_keys()
        .into_iter()
        .filter(|k| !row_of(*k).is_some_and(|r| r.read_only()))
        .collect();
    for e in &files.entries {
        if !keys.contains(&e.key) {
            keys.push(e.key);
        }
    }
    keys
}

/// Read `keys` from the board. A key the board does not know is left out of the reads and
/// returned: for a key a file names that stops the command (the image and the tool's table
/// disagree); for one of the table's own it means the image predates the field.
pub fn read_board<W: Wire>(
    b: &mut Board<'_, W>,
    keys: &[Key],
) -> Result<(Reads, Vec<Key>), String> {
    let mut reads = Reads::new();
    let mut unknown = Vec::new();
    for &key in keys {
        let r = b.read(key)?;
        match r.status {
            CFG_OK | CFG_DEFAULT => {
                reads.insert(key, r);
            }
            CFG_UNKNOWN_KEY => unknown.push(key),
            s => {
                return Err(format!(
                    "{}: reading {}: {}",
                    b.name,
                    path_of(key),
                    status_name(s)
                ))
            }
        }
    }
    Ok((reads, unknown))
}

/// The board's stored keys as a file (`specs/cli-config.md` section 2): every `CFG_OK` answer
/// among [`dump_keys`] at its path, nothing else.
pub fn to_file(reads: &Reads) -> Json {
    let mut top = Map::new();
    let mut motors: Vec<Map<String, Json>> = Vec::new();
    for key in dump_keys() {
        let Some(r) = reads.get(&key).filter(|r| r.status == CFG_OK) else {
            continue;
        };
        let row = row_of(key).expect("a table key");
        let v = to_json(Some(row), r.tag, &r.bytes);
        match row.index {
            Index::Motor => {
                let m = key.index as usize;
                if motors.len() <= m {
                    motors.resize_with(m + 1, Map::new);
                }
                *slot(&mut motors[m], row.path) = v;
            }
            Index::Array(n) => {
                let s = slot(&mut top, row.path);
                if !s.is_array() {
                    *s = Json::Array(vec![Json::Null; n as usize]);
                }
                s[key.index as usize] = v;
            }
            Index::Single => *slot(&mut top, row.path) = v,
        }
    }
    if !motors.is_empty() {
        top.insert(
            "motors".into(),
            Json::Array(motors.into_iter().map(Json::Object).collect()),
        );
    }
    Json::Object(top)
}

/// The place for dotted `path` in `obj`, making the objects on the way (rule 1, section 2.1).
fn slot<'a>(obj: &'a mut Map<String, Json>, path: &str) -> &'a mut Json {
    let (head, rest) = match path.split_once('.') {
        Some((h, r)) => (h, Some(r)),
        None => (path, None),
    };
    let next = obj.entry(head).or_insert(Json::Null);
    match rest {
        None => next,
        Some(rest) => {
            if !next.is_object() {
                *next = Json::Object(Map::new());
            }
            slot(next.as_object_mut().expect("just made"), rest)
        }
    }
}

/// What a planned write does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Act {
    /// A stored value changes, or a default changes to a different value.
    Change,
    /// The board's default is written as a value, so it no longer follows the image.
    Pin,
    /// A stored key is un-stored.
    Unstore,
}

/// One planned `CONFIG_WRITE`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Step {
    /// The key.
    pub key: Key,
    /// What it does.
    pub act: Act,
    /// The storage type written.
    pub storage: Type,
    /// The value bytes; `None` un-stores.
    pub bytes: Option<Vec<u8>>,
}

/// What `apply` (or `replace`) would do to a board.
#[derive(Debug, Default)]
pub struct Plan {
    /// The writes, files' keys first in file order, then the un-stores `replace` adds.
    pub steps: Vec<Step>,
    /// Keys the files name whose value the board already holds.
    pub unchanged: usize,
    /// Keys the board stores that no file names (`apply` leaves them alone).
    pub kept: Vec<Key>,
}

impl Plan {
    /// Whether it writes anything.
    pub fn changes(&self) -> bool {
        !self.steps.is_empty()
    }

    fn count(&self, act: Act) -> usize {
        self.steps.iter().filter(|s| s.act == act).count()
    }

    /// How many keys it un-stores.
    pub fn unstores(&self) -> usize {
        self.count(Act::Unstore)
    }
}

/// Plan the files against what the board answered (`specs/cli-config.md` section 4.3). Only
/// `replace` un-stores: a `null` for a key the board stores, and every key the board stores that no
/// file names, except the board-owned ones. `apply` leaves both alone (section 2.1), so applying
/// one board's dump to another never removes the second board's values. `Err` for a file's key the board does not know, or a value for an id the table does not
/// name that the board's type cannot take.
pub fn plan(
    files: &Files,
    reads: &Reads,
    unknown: &[Key],
    replace: bool,
) -> Result<Plan, Vec<Problem>> {
    let mut plan = Plan::default();
    let mut problems = Vec::new();
    for e in &files.entries {
        if unknown.contains(&e.key) {
            problems.push(Problem::stop(format!(
                "{}: the board does not know this key, so its image is older or newer than this \
                 tool's table: update whichever is behind",
                path_of(e.key)
            )));
            continue;
        }
        let Some(r) = reads.get(&e.key) else {
            continue;
        };
        let storage = match row_of(e.key) {
            Some(row) => row.storage,
            None => match Type::from_tag(r.tag) {
                Some(t) => t,
                None => continue,
            },
        };
        let bytes = match encode(e.key, storage, &e.value) {
            Ok((b, _)) => b,
            Err(p) => {
                problems.push(p);
                continue;
            }
        };
        let holds = |b: &[u8]| r.tag == storage.tag() && r.bytes == b;
        let act = match &bytes {
            None if r.status == CFG_OK && replace => Some(Act::Unstore),
            None => None,
            Some(b) if holds(b) && r.status == CFG_OK => None,
            Some(b) if holds(b) => Some(Act::Pin),
            Some(_) => Some(Act::Change),
        };
        match act {
            Some(act) => plan.steps.push(Step {
                key: e.key,
                act,
                storage,
                bytes,
            }),
            None => plan.unchanged += 1,
        }
    }
    if !problems.is_empty() {
        return Err(problems);
    }
    for key in table_keys() {
        let named = files.entries.iter().any(|e| e.key == key);
        let row = row_of(key).expect("a table key");
        let stored = reads.get(&key).is_some_and(|r| r.status == CFG_OK);
        if named || !stored || row.board_owned() || row.read_only() {
            continue;
        }
        if replace {
            plan.steps.push(Step {
                key,
                act: Act::Unstore,
                storage: row.storage,
                bytes: None,
            });
        } else {
            plan.kept.push(key);
        }
    }
    Ok(plan)
}

/// A value as a line shows it, with ` (default)` when the board stores nothing.
pub fn shown(key: Key, r: &ConfigResp) -> String {
    if r.tag == ERASED_TAG && r.bytes.is_empty() {
        return "(default)".into();
    }
    let s = show(row_of(key), &to_json(row_of(key), r.tag, &r.bytes));
    if r.status == CFG_DEFAULT {
        format!("{s} (default)")
    } else {
        s
    }
}

/// What the board will answer for `key` after `step`, as `CONFIG_RESP` carries it: the value, or
/// for an un-store the registry's default.
fn predicted(step: &Step) -> ConfigResp {
    match &step.bytes {
        Some(b) => ConfigResp {
            status: CFG_OK,
            tag: step.storage.tag(),
            bytes: b.clone(),
        },
        None => match store::lookup_key(step.key) {
            Some(def) => {
                let mut buf = [0u8; 64];
                let n = def.default.encode(&mut buf);
                ConfigResp {
                    status: CFG_DEFAULT,
                    tag: def.kind.tag(),
                    bytes: buf[..n].to_vec(),
                }
            }
            None => ConfigResp {
                status: CFG_DEFAULT,
                tag: ERASED_TAG,
                bytes: Vec::new(),
            },
        },
    }
}

/// The note a change of a pin field ends with.
pub const PIN_NOTE: &str = "pin fields take effect at the next boot";

/// The report of `plan` on board `name`: the summary, a line per write (from `before` to `after`,
/// what the board answered for the key after it, or, before any write, what it will), the kept keys,
/// and the pin note when a pin field changes.
pub fn report(
    name: &str,
    plan: &Plan,
    before: &Reads,
    after: &BTreeMap<Key, ConfigResp>,
) -> Vec<String> {
    let mut summary = format!(
        "{name}: {} changed, {} pinned, {} unchanged",
        plan.count(Act::Change),
        plan.count(Act::Pin),
        plan.unchanged
    );
    if plan.unstores() > 0 {
        summary += &format!(", {} un-stored", plan.unstores());
    }
    if !plan.kept.is_empty() {
        summary += &format!(", {} kept", plan.kept.len());
    }
    let mut lines = vec![summary];
    for s in &plan.steps {
        let then = after.get(&s.key).cloned().unwrap_or_else(|| predicted(s));
        lines.push(change_line(name, s.key, &before[&s.key], &then));
    }
    for k in &plan.kept {
        lines.push(format!(
            "{name}: kept: {} = {} (stored, not in the file)",
            path_of(*k),
            shown(*k, &before[k])
        ));
    }
    if plan
        .steps
        .iter()
        .any(|s| row_of(s.key).is_some_and(|r| r.kind == Kind::Pin))
    {
        lines.push(PIN_NOTE.into());
    }
    lines
}

/// The line for one written key: `name: path: before -> after`.
pub fn change_line(name: &str, key: Key, before: &ConfigResp, after: &ConfigResp) -> String {
    format!(
        "{name}: {}: {} -> {}",
        path_of(key),
        shown(key, before),
        shown(key, after)
    )
}

/// A plan refused partway (`specs/cli-config.md` section 5): why, and what the keys written
/// before the refusal now read on the board.
#[derive(Debug, PartialEq, Eq)]
pub struct Refused {
    /// The refused key and the status, or the link's error.
    pub msg: String,
    /// Each key written before it, as read back.
    pub written: BTreeMap<Key, ConfigResp>,
}

/// Carry out `plan` on `b`: one `CONFIG_WRITE` per step, then a read of every written key. At the
/// first refusal it stops, and still reads back the keys written before it, so the report says what
/// the board now holds.
pub fn execute<W: Wire>(
    b: &mut Board<'_, W>,
    plan: &Plan,
) -> Result<BTreeMap<Key, ConfigResp>, Refused> {
    let mut written = Vec::new();
    let mut failed = None;
    for s in &plan.steps {
        match b.write(s.key, s.storage, s.bytes.as_deref()) {
            Ok(CFG_OK) => written.push(s.key),
            Ok(status) => {
                failed = Some(format!(
                    "{}: {}: refused: {}",
                    b.name,
                    path_of(s.key),
                    status_name(status)
                ));
                break;
            }
            Err(e) => {
                failed = Some(e);
                break;
            }
        }
    }
    let mut after = BTreeMap::new();
    for k in written {
        match b.read(k) {
            Ok(r) => {
                after.insert(k, r);
            }
            Err(e) => {
                failed.get_or_insert(e);
                break;
            }
        }
    }
    match failed {
        None => Ok(after),
        Some(msg) => Err(Refused {
            msg,
            written: after,
        }),
    }
}

/// The lines for the keys a refused plan wrote before the refusal, from their read-back.
pub fn written_lines(name: &str, plan: &Plan, before: &Reads, r: &Refused) -> Vec<String> {
    let lines = plan.steps.iter().filter_map(|s| {
        Some(change_line(
            name,
            s.key,
            &before[&s.key],
            r.written.get(&s.key)?,
        ))
    });
    let mut out: Vec<String> = lines.collect();
    if out.is_empty() {
        out.push(format!("{name}: nothing was written"));
    } else {
        out.insert(0, format!("{name}: written before the refusal:"));
    }
    out
}

/// `config <node> <path>`: the line for the key's value, ` (default)` when nothing is stored.
pub fn config_read<W: Wire>(b: &mut Board<'_, W>, key: Key) -> Result<String, String> {
    let r = b.read(key)?;
    match r.status {
        CFG_OK | CFG_DEFAULT => Ok(format!("{}: {} = {}", b.name, path_of(key), shown(key, &r))),
        s => Err(format!("{}: {}: {}", b.name, path_of(key), status_name(s))),
    }
}

/// What `config <node> <path> <value>` did.
#[derive(Debug, PartialEq, Eq)]
pub struct Written {
    /// The value checks `--force` let through (the key's own, [`write_checks`], are the caller's
    /// to print, before the board is reached).
    pub warnings: Vec<String>,
    /// The key's line from the read-back, and the pin note after a pin field.
    pub lines: Vec<String>,
}

/// `config <node> <path> <value>`: write `text` to `key` in the file's value forms. `force` lets
/// through the checks of `specs/cli-config.md` section 5 that it may; the board's own refusals
/// stand.
pub fn config_write<W: Wire>(
    b: &mut Board<'_, W>,
    key: Key,
    text: &str,
    force: bool,
) -> Result<Written, String> {
    stop_on(&write_checks(key), force)?;
    let before = b.read(key)?;
    if before.status == CFG_UNKNOWN_KEY {
        return Err(format!(
            "{}: {}: the board does not know this key, so its image is older or newer than this \
             tool's table: update whichever is behind",
            b.name,
            path_of(key)
        ));
    }
    if before.status != CFG_OK && before.status != CFG_DEFAULT {
        return Err(format!(
            "{}: {}: {}",
            b.name,
            path_of(key),
            status_name(before.status)
        ));
    }
    let row = row_of(key);
    let storage = match row {
        Some(r) => r.storage,
        None => Type::from_tag(before.tag)
            .ok_or_else(|| format!("{}: the board reports no type", path_of(key)))?,
    };
    let value = crate::names::from_text(row, storage, text);
    let (bytes, warn) = encode(key, storage, &value).map_err(|p| p.msg)?;
    stop_on(&warn, force)?;
    let status = b.write(key, storage, bytes.as_deref())?;
    if status != CFG_OK {
        return Err(format!(
            "{}: {}: refused: {}",
            b.name,
            path_of(key),
            status_name(status)
        ));
    }
    let after = b.read(key)?;
    let mut lines = vec![change_line(&b.name, key, &before, &after)];
    if row.is_some_and(|r| r.kind == Kind::Pin) {
        lines.push(PIN_NOTE.into());
    }
    Ok(Written {
        warnings: warnings(&warn, force),
        lines,
    })
}

/// `Err` naming every problem that stops a command run with `force`.
pub fn stop_on(problems: &[Problem], force: bool) -> Result<(), String> {
    let stops: Vec<&str> = problems
        .iter()
        .filter(|p| p.stops(force))
        .map(|p| p.msg.as_str())
        .collect();
    if stops.is_empty() {
        Ok(())
    } else {
        Err(stops.join("\nerror: "))
    }
}

/// The problems `force` lets through, as warnings.
pub fn warnings(problems: &[Problem], force: bool) -> Vec<String> {
    problems
        .iter()
        .filter(|p| !p.stops(force))
        .map(|p| format!("warning: {}", p.msg))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hoverboard_bridge_core::sim::SimBoard;
    use net::pdu::{Opcode, Pdu};
    use serde_json::json;
    use std::collections::VecDeque;
    use std::time::Duration;

    /// A board model: one board's real `net` responder over its own store, answering in-process.
    /// With `arm_after`, it arms once that many `CONFIG_WRITE`s have reached it.
    struct Loop {
        board: SimBoard,
        replies: VecDeque<Vec<u8>>,
        writes: usize,
        arm_after: Option<usize>,
    }

    impl Loop {
        fn board_wire(&mut self) -> &mut Self {
            self
        }

        fn new() -> Self {
            Loop {
                board: SimBoard::holding(0x01),
                replies: VecDeque::new(),
                writes: 0,
                arm_after: None,
            }
        }
    }

    impl Wire for Loop {
        fn send(&mut self, pdu: &[u8]) {
            if Pdu::decode(pdu).is_ok_and(|p| p.known() == Some(Opcode::ConfigWrite)) {
                if self.arm_after == Some(self.writes) {
                    self.board.resp.set_armed(true);
                }
                self.writes += 1;
            }
            for e in &self.board.ingest(0, pdu) {
                if e.port == 0 {
                    self.replies.push_back(e.bytes.to_vec());
                }
            }
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            self.replies.pop_front()
        }
        fn window(&self) -> usize {
            1
        }
    }

    fn board(lp: &mut Loop) -> Board<'_, Loop> {
        Board {
            wire: lp,
            src: 0x80,
            dst: 0x01,
            name: "rover-left".into(),
        }
    }

    fn k(field_id: u8, index: u8) -> Key {
        Key { field_id, index }
    }

    fn files(texts: &[(&str, Json)]) -> Files {
        let t: Vec<(String, String)> = texts
            .iter()
            .map(|(n, j)| (n.to_string(), j.to_string()))
            .collect();
        read_files(&t)
    }

    /// Plan `f` against the board and carry it out; the report.
    fn run(lp: &mut Loop, f: &Files, replace: bool) -> Result<(Plan, Vec<String>), String> {
        assert!(f.problems.is_empty(), "{:?}", f.problems);
        let mut b = board(lp);
        let (reads, unknown) = read_board(&mut b, &plan_keys(f))?;
        let plan = plan(f, &reads, &unknown, replace).map_err(|p| format!("{p:?}"))?;
        let after = execute(&mut b, &plan).map_err(|r| r.msg)?;
        let lines = report(&b.name, &plan, &reads, &after);
        Ok((plan, lines))
    }

    /// Every key's answer, `node.class` and the board-owned fields included: the record set and its
    /// `Stored` flags as the wire shows them.
    fn records(lp: &mut Loop) -> Reads {
        let mut b = board(lp);
        read_board(&mut b, &table_keys()).unwrap().0
    }

    /// A board model holding a configuration, written through the commands themselves.
    fn staged() -> Loop {
        let mut lp = Loop::new();
        let f = files(&[(
            "layout.json",
            json!({
                "device": { "name": "rover-left" },
                "control": { "mode": "balance", "sf_gains": [3491, null, 197] },
                "board": { "vbatt_cal": [null, -3] },
                "motors": [
                    { "hall": { "a": "PC13" }, "gate": { "dead_time": 25 }, "current": { "cal": 470 } },
                    {},
                ],
            }),
        )]);
        assert!(f.problems.is_empty(), "{:?}", f.problems);
        run(&mut lp, &f, false).unwrap();
        lp
    }

    #[test]
    fn a_dump_applied_to_the_board_it_came_from_writes_nothing() {
        let mut lp = staged();
        let mut b = board(&mut lp);
        let (reads, _) = read_board(&mut b, &dump_keys()).unwrap();
        let dumped = to_file(&reads);
        assert_eq!(
            dumped,
            json!({
                "board": { "vbatt_cal": [null, -3] },
                "control": { "mode": "balance", "sf_gains": [3491, null, 197, null] },
                "device": { "name": "rover-left" },
                "motors": [
                    { "current": { "cal": 470 }, "gate": { "dead_time": 25 }, "hall": { "a": "PC13" } },
                ],
            })
        );
        let f = files(&[("dump.json", dumped)]);
        assert!(f.problems.is_empty());
        let before = lp.writes;
        let (plan, lines) = run(&mut lp, &f, false).unwrap();
        assert!(!plan.changes());
        assert_eq!(lp.writes, before, "nothing written");
        assert_eq!(lines, ["rover-left: 0 changed, 0 pinned, 11 unchanged"]);
    }

    #[test]
    fn a_replace_from_a_dump_unstores_exactly_the_extra_keys_and_reproduces_the_records() {
        let mut source = staged();
        let dumped = {
            let mut b = board(&mut source);
            to_file(&read_board(&mut b, &dump_keys()).unwrap().0)
        };
        // The same layout plus records from an earlier life, a board-owned one among them.
        let mut target = staged();
        let extra = files(&[(
            "old.json",
            json!({
                "motors": [{}, { "hall": { "a": "PB5" } }],
                "board": { "vbatt_cal": [25000] },
                "control": { "frame": "rigid" },
            }),
        )]);
        run(&mut target, &extra, false).unwrap();
        let odo = Key {
            field_id: store::MOTOR_ODOMETER.id(),
            index: 0,
        };
        let w = nodes::write(
            target.board_wire(),
            0x80,
            0x01,
            odo,
            Type::U32,
            &[9, 0, 0, 0],
        );
        assert_eq!(w.unwrap().status, CFG_OK);

        let f = files(&[("dump.json", dumped)]);
        let (plan, lines) = run(&mut target, &f, true).unwrap();
        let unstored: Vec<Key> = plan.steps.iter().map(|s| s.key).collect();
        assert!(plan.steps.iter().all(|s| s.act == Act::Unstore));
        // The dump's null position first (the file names it), then the keys no file names.
        assert_eq!(unstored, [k(0x69, 0), k(0x25, 0), k(0x4A, 1)]);
        assert_eq!(
            lines,
            [
                "rover-left: 0 changed, 0 pinned, 10 unchanged, 3 un-stored",
                "rover-left: board.vbatt_cal[0]: 25000 -> 25200 (default)",
                "rover-left: control.frame: rigid -> pivoting (default)",
                "rover-left: motors[1].hall.a: PB5 -> none (default)",
                PIN_NOTE,
            ]
        );
        // The record set, Stored flags included, is the source's, but the board-owned odometer.
        let mut want = records(&mut source);
        let mut got = records(&mut target);
        let odo_after = got.remove(&odo).unwrap();
        assert_eq!(
            (odo_after.status, &odo_after.bytes[..]),
            (CFG_OK, &[9u8, 0, 0, 0][..])
        );
        want.remove(&odo);
        assert_eq!(got, want);
    }

    #[test]
    fn apply_reports_changes_pins_and_kept_keys_from_the_read_back() {
        let mut lp = staged();
        let f = files(&[(
            "machine.json",
            json!({
                "control": { "sf_gains": [3491, 4002] },
                "motors": [{ "gate": { "dead_time": 28 }, "hall": { "a": "PC13" }, "current": { "noise_floor": 2100 } }],
            }),
        )]);
        let (plan, lines) = run(&mut lp, &f, false).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 2 changed, 1 pinned, 2 unchanged, 5 kept",
                "rover-left: control.sf_gains[1]: 0 (default) -> 4002",
                "rover-left: motors[0].gate.dead_time: 25 -> 28",
                "rover-left: motors[0].current.noise_floor: 2100 (default) -> 2100",
                "rover-left: kept: device.name = \"rover-left\" (stored, not in the file)",
                "rover-left: kept: control.mode = balance (stored, not in the file)",
                "rover-left: kept: motors[0].current.cal = 470 (stored, not in the file)",
                "rover-left: kept: board.vbatt_cal[1] = -3 (stored, not in the file)",
                "rover-left: kept: control.sf_gains[2] = 197 (stored, not in the file)",
            ]
        );
        assert_eq!(plan.kept.len(), 5);
        // No pin field changed, so no pin note; the pinned value is now stored.
        let r = nodes::read(lp.board_wire(), 0x80, 0x01, k(0x6B, 0)).unwrap();
        assert_eq!(r.status, CFG_OK);
    }

    #[test]
    fn an_armed_board_refuses_and_a_board_that_arms_partway_keeps_the_keys_before_it() {
        let f = files(&[(
            "m.json",
            json!({ "control": { "mode": "balance", "frame": "rigid", "side": "right" } }),
        )]);
        // Run the plan; on a refusal, the message and the lines for what was written before it.
        let refused = |lp: &mut Loop| {
            let mut b = board(lp);
            let (reads, unknown) = read_board(&mut b, &plan_keys(&f)).unwrap();
            let plan = plan(&f, &reads, &unknown, false).unwrap();
            let r = execute(&mut b, &plan).unwrap_err();
            let lines = written_lines(&b.name, &plan, &reads, &r);
            (r.msg, lines)
        };
        // Armed from the first write: the board is unchanged, whatever --force says.
        let mut lp = Loop::new();
        lp.board.resp.set_armed(true);
        assert_eq!(
            refused(&mut lp),
            (
                "rover-left: control.mode: refused: armed".to_string(),
                vec!["rover-left: nothing was written".to_string()]
            )
        );
        assert_eq!(records(&mut lp)[&k(0x22, 0)].status, CFG_DEFAULT);

        // Arms after the first write: that key is read back and reported as the board holds it.
        let mut lp = Loop::new();
        lp.arm_after = Some(1);
        assert_eq!(
            refused(&mut lp),
            (
                "rover-left: control.frame: refused: armed".to_string(),
                vec![
                    "rover-left: written before the refusal:".to_string(),
                    "rover-left: control.mode: throttle (default) -> balance".to_string(),
                ]
            )
        );
        let r = records(&mut lp);
        assert_eq!(
            (r[&k(0x22, 0)].status, r[&k(0x25, 0)].status),
            (CFG_OK, CFG_DEFAULT)
        );
    }

    /// `apply` never un-stores: a `null` (a position a dump of another board did not store) leaves
    /// the key alone; `replace` un-stores it (`specs/cli-config.md` section 2.1).
    #[test]
    fn a_null_unstores_under_replace_and_is_left_alone_under_apply() {
        let f = files(&[(
            "dump.json",
            json!({ "control": { "sf_gains": [3491, null] } }),
        )]);
        let stored = |v: i16| ConfigResp {
            status: CFG_OK,
            tag: Type::I16.tag(),
            bytes: v.to_le_bytes().to_vec(),
        };
        let reads: Reads = [(k(0x79, 0), stored(3491)), (k(0x79, 1), stored(4002))]
            .into_iter()
            .collect();
        let apply = plan(&f, &reads, &[], false).unwrap();
        assert!(apply.steps.is_empty(), "{:?}", apply.steps);
        assert_eq!(apply.unchanged, 2);
        let replace = plan(&f, &reads, &[], true).unwrap();
        let steps: Vec<(Key, Act)> = replace.steps.iter().map(|s| (s.key, s.act)).collect();
        assert_eq!(steps, [(k(0x79, 1), Act::Unstore)]);
    }

    #[test]
    fn the_file_refusals_stop_and_the_forceable_ones_say_what_force_does() {
        let f = files(&[(
            "a.json",
            json!({
                "control": { "mod": "balance", "frame": "bent", "side": 5 },
                "motors": [{ "align_offset": 6 }, {}, { "hall": { "a": "PA1" } }],
                "board": { "vbatt_cal": [1, 2, 3], "pack_cells": "ten" },
                "node": { "address": 4 },
                "hall": { "b": "PA2" },
                "0x99.0": 5,
                "0x25": "rigid",
            }),
        )]);
        let got: Vec<(bool, &str)> = f
            .problems
            .iter()
            .map(|p| (p.force, p.msg.as_str()))
            .collect();
        assert_eq!(
            got,
            [
                (false, "a.json: unknown field control.mod (did you mean control.mode?)"),
                (false, "a.json: control.frame: unknown label bent (pivoting, rigid)"),
                (true, "a.json: control.side takes left, right, not 5 (--force writes it anyway)"),
                (true, "a.json: motors[0].align_offset takes 0..5, not 6 (--force writes it anyway)"),
                (true, "a.json: hall.a has 2 motors (--force writes motors[2] anyway)"),
                (true, "a.json: board.vbatt_cal has 2 indices (--force writes index 2 anyway)"),
                (false, "a.json: board.pack_cells: an integer, not \"ten\""),
                (true, "a.json: node.address is board-owned (--force writes it anyway)"),
                (false, "a.json: hall is per motor (motors[<n>].hall)"),
                (
                    true,
                    "a.json: 0x99 is not in the name table (--force writes it with the type the board reports)"
                ),
                (false, "a.json: control.frame is named twice"),
            ]
        );
        // Every stop stops with --force too; without it, the forceable ones stop as well.
        assert!(stop_on(&f.problems, true).is_err());
        let forced: Vec<_> = f.problems.iter().filter(|p| p.force).cloned().collect();
        assert!(stop_on(&forced, false).is_err());
        assert!(stop_on(&forced, true).is_ok());
        assert_eq!(warnings(&forced, true).len(), forced.len());
    }

    #[test]
    fn a_key_in_two_files_is_refused_and_files_merge_in_order() {
        let f = files(&[
            (
                "layout.json",
                json!({ "motors": [{ "hall": { "a": "PC13" } }] }),
            ),
            (
                "machine.json",
                json!({ "control": { "mode": "balance" }, "0x4A.0": "PB5" }),
            ),
        ]);
        assert_eq!(
            f.problems,
            [Problem::stop(
                "machine.json: motors[0].hall.a is also in layout.json"
            )]
        );
        let f = files(&[
            (
                "layout.json",
                json!({ "motors": [{ "hall": { "a": "PC13" } }] }),
            ),
            ("machine.json", json!({ "control": { "mode": "balance" } })),
        ]);
        assert!(f.problems.is_empty());
        let keys: Vec<Key> = f.entries.iter().map(|e| e.key).collect();
        assert_eq!(keys, [k(0x4A, 0), k(0x22, 0)]);
    }

    #[test]
    fn a_key_the_board_does_not_know_stops_and_an_unnamed_id_takes_the_boards_type() {
        let mut lp = Loop::new();
        let f = files(&[("x.json", json!({ "0x99": 5 }))]);
        let mut b = board(&mut lp);
        let (reads, unknown) = read_board(&mut b, &plan_keys(&f)).unwrap();
        assert_eq!(unknown, [k(0x99, 0)]);
        let e = plan(&f, &reads, &unknown, false).unwrap_err();
        assert_eq!(
            e[0].msg,
            "0x99.0: the board does not know this key, so its image is older or newer than this \
             tool's table: update whichever is behind"
        );
        // A registered id the table did not name would be written with the board's type: the raw
        // form of a named one is the same key, checked the same way.
        let f = files(&[("x.json", json!({ "0x20": 12000 }))]);
        assert!(f.problems.is_empty());
        let (plan, _) = run(&mut lp, &f, false).unwrap();
        assert_eq!(plan.steps[0].storage, Type::U32);
        assert_eq!(plan.steps[0].bytes, Some(12000u32.to_le_bytes().to_vec()));
    }

    #[test]
    fn config_reads_and_writes_one_field_with_the_spec_lines() {
        let mut lp = Loop::new();
        let mut b = board(&mut lp);
        assert_eq!(
            config_read(&mut b, k(0x67, 0)).unwrap(),
            "rover-left: motors[0].current.cal = 455 (default)"
        );
        let lines =
            |b: &mut Board<'_, Loop>, key, text| config_write(b, key, text, false).unwrap().lines;
        assert_eq!(
            lines(&mut b, k(0x22, 0), "balance"),
            ["rover-left: control.mode: throttle (default) -> balance"]
        );
        assert_eq!(
            lines(&mut b, k(0x4A, 1), "PB5"),
            [
                "rover-left: motors[1].hall.a: none (default) -> PB5",
                PIN_NOTE
            ]
        );
        lines(&mut b, k(0x67, 0), "470");
        assert_eq!(
            lines(&mut b, k(0x67, 0), "null"),
            ["rover-left: motors[0].current.cal: 470 -> 455 (default)"]
        );
        assert_eq!(
            lines(&mut b, k(0x69, 1), "-5"),
            ["rover-left: board.vbatt_cal[1]: -5 (default) -> -5"]
        );
        assert_eq!(
            config_write(&mut b, k(0x22, 1), "balance", false).unwrap_err(),
            "control.mode has one index (--force writes index 1 anyway)"
        );
        assert_eq!(
            config_write(&mut b, k(0x22, 1), "balance", true).unwrap(),
            Written {
                warnings: Vec::new(),
                lines: vec!["rover-left: control.mode[1]: throttle (default) -> balance".into()],
            }
        );
        assert_eq!(
            config_write(&mut b, k(0x63, 0), "9", false).unwrap_err(),
            "motors[0].align_offset takes 0..5, not 9 (--force writes it anyway)"
        );
        assert_eq!(
            config_write(&mut b, k(0x63, 0), "9", true)
                .unwrap()
                .warnings,
            ["warning: motors[0].align_offset takes 0..5, not 9 (--force writes it anyway)"]
        );
        assert_eq!(
            config_write(&mut b, k(0x03, 0), "2", true).unwrap_err(),
            "node.class is read-only"
        );
        b.wire.board.resp.set_armed(true);
        assert_eq!(
            config_write(&mut b, k(0x22, 0), "throttle", true).unwrap_err(),
            "rover-left: control.mode: refused: armed"
        );
    }
}
