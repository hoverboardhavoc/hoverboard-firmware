//! A board's store as a file (`specs/cli-config.md`): `dump`, `apply`, `replace`, `diff`, and one
//! field at a time with `get` and `set`, under `board` and `bridge`. A file holds one value per stored `(field_id, index)` at
//! the path the name table gives it ([`crate::names`]) and nothing else.
//!
//! Every command checks the files completely before it reads the board, reads every key the table
//! names (and every key a file names), plans, and only then writes: one `CONFIG_WRITE` per key,
//! values and un-stores alike, then reads every written key back and reports what the board holds.
//!
//! A secret (`store::SECRET`, `wifi.pass`; `specs/bridge-onboarding.md` section 2.2) is never
//! printed, the board's or the file's: both show as `(set)`. `dump` leaves a stored one out and
//! says so, `apply` and `replace` write one a file gives every time and count it as changed, and
//! `replace` never un-stores one the files leave out.

use std::collections::{BTreeMap, HashMap};

use net::walk::{CFG_DEFAULT, CFG_OK, CFG_SECRET, CFG_UNKNOWN_KEY};
use serde_json::{Map, Value as Json};
use store::{Effect, Index, Key, Type, ERASED_TAG};

use crate::names::{
    encode, find, find_named, object_arrays, path_of, row_of, show, table_keys, to_json, unknown,
    write_checks, Problem,
};
use crate::nodes::{self, parse_key, status_name, ConfigResp};
use crate::wire::Wire;

/// Every problem of a refused plan, one per line.
pub fn joined(p: Vec<Problem>) -> String {
    let msgs: Vec<String> = p.into_iter().map(|p| p.msg).collect();
    msgs.join("\nerror: ")
}

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
    /// Its `IDENTITY` class, which names a bridge's refusal of a name it cannot advertise.
    pub class: u8,
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

/// The element of an object array a member sits in: the array and the index, `None` at the top.
type Elem = Option<(&'static str, u8)>;

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
            if k == "boards" {
                self.stop(
                    "a machine file (it has \"boards\"): give it to the machine commands".into(),
                );
            } else if let Some((array, what)) = object_arrays().find(|(a, _)| a == k) {
                let Some(items) = v.as_array() else {
                    self.stop(format!("{array} takes a list, one object per {what}"));
                    continue;
                };
                for (i, obj) in items.iter().enumerate() {
                    match (u8::try_from(i), obj.as_object()) {
                        (Ok(i), Some(obj)) => self.object(obj, "", Some((array, i))),
                        _ => self.stop(format!("{array}[{i}] is not a {what}'s object")),
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

    fn object(&mut self, obj: &Map<String, Json>, prefix: &str, elem: Elem) {
        for (k, v) in obj {
            self.member(k, v, prefix, elem);
        }
    }

    /// One member `k: v` of an object at `prefix`, under `elem` when inside an object array.
    fn member(&mut self, k: &str, v: &Json, prefix: &str, elem: Elem) {
        let path = if prefix.is_empty() {
            k.to_string()
        } else {
            format!("{prefix}.{k}")
        };
        let array = elem.map(|(a, _)| a);
        if let Some(row) = find(&path, array) {
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
                Index::Named(names) => match v.as_object() {
                    Some(obj) => {
                        for (name, item) in obj {
                            match row.index.named(name) {
                                Some(i) => self.add(at(i), item),
                                None => self.problem(unknown(&format!("{path}.{name}"))),
                            }
                        }
                    }
                    None => self.stop(format!("{path} takes an object of {}", names.join(", "))),
                },
                _ => self.add(at(elem.map_or(0, |(_, i)| i)), v),
            }
            return;
        }
        // A dotted member naming one value of a named field (`"tone.horn": ...`).
        if elem.is_none() {
            if let Some((row, i)) = path.rsplit_once('.').and_then(|(p, n)| find_named(p, n)) {
                self.add(
                    Key {
                        field_id: row.id,
                        index: i,
                    },
                    v,
                );
                return;
            }
        }
        let inner = format!("{path}.");
        let nests = |array: Option<&str>| {
            store::NAMES
                .iter()
                .any(|r| r.index.array() == array && r.path.starts_with(&inner))
        };
        if let Some(obj) = v.as_object().filter(|_| nests(array)) {
            self.object(obj, &path, elem);
            return;
        }
        // Where the path belongs instead, if anywhere: in an object array, or at the top level.
        let belongs = |a: Option<&str>| a != array && (find(&path, a).is_some() || nests(a));
        let other = object_arrays().find(|(a, _)| belongs(Some(a)));
        match (other, elem) {
            (Some((a, what)), _) => self.stop(format!("{path} is per {what} ({a}[<n>].{path})")),
            (None, Some((a, _))) if belongs(None) => {
                self.stop(format!("{path} is not in {a} (it goes at the top level)"))
            }
            _ => {
                let shown = match elem {
                    Some((a, i)) => format!("{a}.{i}.{path}"),
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
        // A pin token is resolved once the board's chip is known (`crate::checks::resolve`).
        if let Some(row) = row_of(key).filter(|_| !crate::names::is_token(key, v)) {
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

/// What the board answered for every key read (`CFG_OK` or `CFG_DEFAULT` with the value, or
/// `CFG_SECRET` with none).
pub type Reads = BTreeMap<Key, ConfigResp>;

/// Whether a read answered a successful read: `CFG_OK`, `CFG_DEFAULT`, or `CFG_SECRET` for a stored
/// secret.
fn answered(r: &ConfigResp) -> bool {
    matches!(r.status, CFG_OK | CFG_DEFAULT | CFG_SECRET)
}

/// Whether the board stores the key: `CFG_OK`, or `CFG_SECRET` for a secret.
fn stored(r: &ConfigResp) -> bool {
    matches!(r.status, CFG_OK | CFG_SECRET)
}

/// Whether `key` is a secret ([`store::SECRET`]).
fn secret(key: Key) -> bool {
    row_of(key).is_some_and(|r| r.secret())
}

/// The keys `dump` reads: every key the table names, less the board-owned fields.
pub fn dump_keys() -> Vec<Key> {
    table_keys()
        .into_iter()
        .filter(|k| row_of(*k).is_some_and(|r| !r.board_owned()))
        .collect()
}

/// The keys a store erase loses that `dump` leaves out: the board-owned ones (`node.address` and
/// each motor's `odometer`), which `board erase` prints before it asks.
pub fn board_owned_keys() -> Vec<Key> {
    table_keys()
        .into_iter()
        .filter(|k| row_of(*k).is_some_and(|r| r.board_owned()))
        .collect()
}

/// The keys `diff`, `apply` and `replace` read: every key the table names, and every key `files`
/// name.
pub fn plan_keys(files: &Files) -> Vec<Key> {
    let mut keys: Vec<Key> = table_keys();
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
            _ if answered(&r) => {
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

/// How a dump writes a board's pins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pins {
    /// The board's chip is known: a pin is its token where resolving the token on that chip gives
    /// back the stored bytes (`crate::checks::token_of`), else its object
    /// (`specs/cli-onboarding.md` part C, "Output").
    Chip(hal::pincap::Chip),
    /// Every pin as its object. `remaps` is a board whose chip routes by the AFIO remap (the
    /// GD32F10x, `IDENTITY`'s family), whose pins carry no `af` in a file
    /// (`specs/config-read-once.md` section 3).
    Objects {
        /// The chip routes by its remap.
        remaps: bool,
    },
}

impl Pins {
    fn remaps(self) -> bool {
        match self {
            Pins::Chip(c) => c.remaps(),
            Pins::Objects { remaps } => remaps,
        }
    }
}

/// The board's stored keys as a file (`specs/cli-config.md` section 2): every `CFG_OK` answer
/// among [`dump_keys`] at its path, nothing else, its pins as `pins` says.
pub fn to_file(reads: &Reads, pins: Pins) -> Json {
    let stored = state_after(reads, &Plan::default());
    let mut top = Map::new();
    let mut arrays: BTreeMap<&str, Vec<Map<String, Json>>> = BTreeMap::new();
    for key in dump_keys() {
        let Some(r) = reads.get(&key).filter(|r| r.status == CFG_OK) else {
            continue;
        };
        let row = row_of(key).expect("a table key");
        let token = match pins {
            Pins::Chip(chip) if row.kind == store::Kind::Pin => {
                crate::checks::token_of(chip, key, &r.bytes, &stored)
            }
            _ => None,
        };
        let mut v = token.map_or_else(|| to_json(Some(row), r.tag, &r.bytes), Json::String);
        if pins.remaps() && row.storage == Type::PinAf {
            if let Json::Object(o) = &mut v {
                o.remove("af");
            }
        }
        match row.index {
            Index::Motor | Index::Link => {
                let elems = arrays
                    .entry(row.index.array().expect("an object array"))
                    .or_default();
                let i = key.index as usize;
                if elems.len() <= i {
                    elems.resize_with(i + 1, Map::new);
                }
                *slot(&mut elems[i], row.path) = v;
            }
            Index::Array(n) => {
                let s = slot(&mut top, row.path);
                if !s.is_array() {
                    *s = Json::Array(vec![Json::Null; n as usize]);
                }
                s[key.index as usize] = v;
            }
            Index::Named(_) => {
                let s = slot(&mut top, row.path);
                if !s.is_object() {
                    *s = Json::Object(Map::new());
                }
                let name = row.index.name(key.index).expect("a table key's name");
                s[name] = v;
            }
            Index::Single => *slot(&mut top, row.path) = v,
        }
    }
    // The arrays in a fixed order, after the top-level fields: `links`, then `motors`.
    for (name, elems) in arrays {
        top.insert(
            name.into(),
            Json::Array(elems.into_iter().map(Json::Object).collect()),
        );
    }
    Json::Object(top)
}

/// The lines `dump` prints for what it left out of the file: each stored secret, which no node
/// reads back (`specs/bridge-onboarding.md` section 2.2).
pub fn left_out(name: &str, reads: &Reads) -> Vec<String> {
    dump_keys()
        .into_iter()
        .filter(|k| reads.get(k).is_some_and(|r| r.status == CFG_SECRET))
        .map(|k| format!("{name}: {} stored, not readable, left out", path_of(k)))
        .collect()
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
    /// Keys the board stores that `apply` leaves alone, and why.
    pub kept: Vec<(Key, Kept)>,
}

/// Why `apply` leaves a stored key alone (`specs/cli-config.md` section 4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kept {
    /// No file names it.
    NotInFile,
    /// A file gives it as `null`.
    NullInFile,
}

impl Kept {
    /// How its line ends.
    fn note(self) -> &'static str {
        match self {
            Kept::NotInFile => "(stored, not in the file)",
            Kept::NullInFile => "(stored, null in the file)",
        }
    }
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
/// file names, except the board-owned ones. `apply` leaves both alone (section 2.1) and lists them
/// as kept, so applying one board's dump to another never removes the second board's values; a
/// `null` for a key the board does not store is unchanged. `Err` for a file's key the board does not know, or a value for an id the table does not
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
            None if stored(r) && replace => Some(Act::Unstore),
            None if stored(r) => {
                plan.kept.push((e.key, Kept::NullInFile));
                continue;
            }
            None => None,
            // A secret cannot be compared, so a value for one is always written.
            Some(_) if secret(e.key) => Some(Act::Change),
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
        let is_stored = reads.get(&key).is_some_and(stored);
        if named || !is_stored || row.board_owned() || row.secret() {
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
            plan.kept.push((key, Kept::NotInFile));
        }
    }
    Ok(plan)
}

/// The board's stored values as `plan` would leave them: every stored key read, with the plan's
/// writes and un-stores applied.
pub fn state_after(reads: &Reads, plan: &Plan) -> crate::checks::State {
    let mut state = crate::checks::State::new();
    for (key, r) in reads {
        if let (CFG_OK, Some(t)) = (r.status, Type::from_tag(r.tag)) {
            state.insert(*key, (t, r.bytes.clone()));
        }
    }
    for s in &plan.steps {
        match &s.bytes {
            Some(b) => {
                state.insert(s.key, (s.storage, b.clone()));
            }
            None => {
                state.remove(&s.key);
            }
        }
    }
    state
}

/// The plan a `set` of `key` to `text` makes, for a key the table names: one write, or an un-store
/// for `null`. What the leader check applies to the board's read before the write
/// (`specs/rigid-leader.md` section 6.1).
pub fn set_plan(key: Key, text: &str) -> Result<Plan, String> {
    let row = row_of(key).ok_or_else(|| format!("{}: not in the table", path_of(key)))?;
    let value = crate::names::from_text(Some(row), row.storage, text);
    let (bytes, _) = encode(key, row.storage, &value).map_err(|p| p.msg)?;
    Ok(Plan {
        steps: vec![Step {
            key,
            act: Act::Change,
            storage: row.storage,
            bytes,
        }],
        ..Plan::default()
    })
}

/// The checks of `specs/config-read-once.md` section 4 on the board as `plan` would leave it: the
/// value checks always, the pin checks with the board's `chip`, and the `af` rule of the files'
/// pins. `Err` lists every refusal; `--force` overrides none. A plan that changes a pin, instance,
/// remap, dead-time or `imu.model` key, or files that carry a pin, is refused without the chip.
pub fn check_plan(
    chip: Option<hal::pincap::Chip>,
    files: &Files,
    reads: &Reads,
    plan: &Plan,
) -> Result<(), String> {
    let state = state_after(reads, plan);
    let mut refused = crate::checks::check_values(&state);
    match chip {
        Some(c) => {
            refused.extend(crate::checks::check_forms(c, &files.entries));
            refused.extend(crate::checks::check_pins(c, &state));
        }
        None => {
            let key = plan
                .steps
                .iter()
                .map(|s| s.key)
                .find(|k| crate::checks::needs_chip(*k))
                .or(crate::checks::first_pin(&files.entries));
            if let Some(k) = key {
                return Err(format!(
                    "{}: the pin checks need the board's chip, read as TARGET_ID through the \
                     bridge wired to it",
                    path_of(k)
                ));
            }
        }
    }
    if refused.is_empty() {
        Ok(())
    } else {
        Err(refused
            .into_iter()
            .map(|r| r.msg)
            .collect::<Vec<_>>()
            .join("\nerror: "))
    }
}

/// A value as a line shows it, with ` (default)` when the board stores nothing. A secret shows as
/// `(set)` when stored and `(default)` when not, never as its value.
pub fn shown(key: Key, r: &ConfigResp) -> String {
    if secret(key) {
        return if r.status == CFG_DEFAULT {
            "(default)"
        } else {
            "(set)"
        }
        .into();
    }
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
/// for an un-store the registry's default. A default a node may answer itself
/// ([`store::NODE_DEFAULTS`]) is not known before the write, so it carries no value and shows as
/// `(default)` alone (`specs/cli-config.md` section 4.3).
fn predicted(step: &Step) -> ConfigResp {
    let unknown = ConfigResp {
        status: CFG_DEFAULT,
        tag: ERASED_TAG,
        bytes: Vec::new(),
    };
    match &step.bytes {
        Some(b) => ConfigResp {
            status: CFG_OK,
            tag: step.storage.tag(),
            bytes: b.clone(),
        },
        None if row_of(step.key).is_some_and(|r| r.node_default()) => unknown,
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
            None => unknown,
        },
    }
}

/// When the changed `keys` take effect, from each key's registry effect
/// (`specs/machine-settings.md` section 5): `3 take effect at the next arm, 1 at the next boot`.
/// `None` when none waits for an arm or a boot: a live field applies as it is written, and a
/// client-only one has no reader on the board.
pub fn effect_line(keys: impl IntoIterator<Item = Key>) -> Option<String> {
    let (mut arm, mut boot) = (0usize, 0usize);
    for k in keys {
        match store::lookup(k.field_id).map(|d| d.effect) {
            Some(Effect::Arm) => arm += 1,
            Some(Effect::Boot) => boot += 1,
            _ => {}
        }
    }
    let parts: Vec<String> = [(arm, "at the next arm"), (boot, "at the next boot")]
        .into_iter()
        .filter(|(n, _)| *n > 0)
        .enumerate()
        .map(|(i, (n, when))| match (i, n) {
            (0, 1) => format!("1 takes effect {when}"),
            (0, n) => format!("{n} take effect {when}"),
            (_, n) => format!("{n} {when}"),
        })
        .collect();
    (!parts.is_empty()).then(|| parts.join(", "))
}

/// The keys `plan` changes on the board: every write but a pin, which stores the value the board
/// already runs.
pub fn changed_keys(plan: &Plan) -> impl Iterator<Item = Key> + '_ {
    plan.steps
        .iter()
        .filter(|s| s.act != Act::Pin)
        .map(|s| s.key)
}

/// The report of `plan` on board `name`: the summary, a line per write (from `before` to `after`,
/// what the board answered for the key after it, or, before any write, what it will), the kept keys,
/// and when the changed keys take effect ([`effect_line`]).
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
    for (k, why) in &plan.kept {
        lines.push(format!(
            "{name}: kept: {} = {} {}",
            path_of(*k),
            shown(*k, &before[k]),
            why.note()
        ));
    }
    lines.extend(effect_line(changed_keys(plan)));
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

/// `get <node> <path>`: the line for the key's value, ` (default)` when nothing is stored.
pub fn config_read<W: Wire>(b: &mut Board<'_, W>, key: Key) -> Result<String, String> {
    let r = b.read(key)?;
    if answered(&r) {
        Ok(format!("{}: {} = {}", b.name, path_of(key), shown(key, &r)))
    } else {
        Err(format!(
            "{}: {}: {}",
            b.name,
            path_of(key),
            status_name(r.status)
        ))
    }
}

/// What `set <node> <path> <value>` did.
#[derive(Debug, PartialEq, Eq)]
pub struct Written {
    /// The value checks `--force` let through (the key's own, [`write_checks`], are the caller's
    /// to print, before the board is reached).
    pub warnings: Vec<String>,
    /// The key's line from the read-back.
    pub line: String,
    /// The key, when the write changed the value the board runs.
    pub changed: Option<Key>,
}

impl Written {
    /// The key's line, and when the change takes effect ([`effect_line`]).
    pub fn lines(&self) -> Vec<String> {
        let mut l = vec![self.line.clone()];
        l.extend(effect_line(self.changed));
        l
    }
}

/// `set <node> <path> <value>`: write `text` to `key` in the file's value forms. `force` lets
/// through the checks of `specs/cli-config.md` section 5 that it may; the board's own refusals
/// stand.
///
/// The checks of `specs/config-read-once.md` section 4 run on the board as the write would leave
/// it, with `chip` for the pin checks.
pub fn config_write<W: Wire>(
    b: &mut Board<'_, W>,
    key: Key,
    text: &str,
    force: bool,
    chip: Option<hal::pincap::Chip>,
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
    if !answered(&before) {
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
    let mut value = crate::names::from_text(row, storage, text);
    let (reads, _) = read_board(b, &table_keys())?;
    if let Some(chip) = chip.filter(|_| crate::names::is_token(key, &value)) {
        let mut entries = vec![Entry { key, value }];
        let stored = state_after(&reads, &Plan::default());
        crate::checks::resolve(chip, &mut entries, &stored)
            .map_err(|e| format!("{}: {e}", b.name))?;
        if let Some(e) = entries.get(1) {
            return Err(format!(
                "{}: {}: {text} needs {} {}: write it first, or give the pin as an object",
                b.name,
                path_of(key),
                path_of(e.key),
                e.value
            ));
        }
        value = entries.remove(0).value;
    }
    let (bytes, warn) = encode(key, storage, &value).map_err(|p| p.msg)?;
    stop_on(&warn, force)?;
    let plan = Plan {
        steps: vec![Step {
            key,
            act: Act::Change,
            storage,
            bytes: bytes.clone(),
        }],
        ..Plan::default()
    };
    let files = Files {
        entries: vec![Entry { key, value }],
        problems: Vec::new(),
    };
    check_plan(chip, &files, &reads, &plan).map_err(|e| format!("{}: {e}", b.name))?;
    let status = b.write(key, storage, bytes.as_deref())?;
    if status != CFG_OK {
        let why = if key == store::DEVICE_NAME.key() {
            nodes::name_status(Some(b.class), status)
        } else {
            status_name(status)
        };
        return Err(format!("{}: {}: refused: {why}", b.name, path_of(key)));
    }
    let after = b.read(key)?;
    let changed = (after.tag, &after.bytes) != (before.tag, &before.bytes);
    Ok(Written {
        warnings: warnings(&warn, force),
        line: change_line(&b.name, key, &before, &after),
        changed: changed.then_some(key),
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
            class: net::walk::CLASS_MOTOR,
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

    /// Every key's answer, the board-owned fields included: the record set and its `Stored` flags
    /// as the wire shows them.
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
                "machine": { "mode": "balance", "balance": { "gains": [3491, null, 197] } },
                "battery_sense": { "cal": [null, -3] },
                "motors": [
                    { "hall": { "a": { "port": "C", "pin": 13 } }, "gate": { "dead_time": 25 }, "current": { "cal": 470 } },
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
        let dumped = to_file(&reads, Pins::Objects { remaps: false });
        assert_eq!(
            dumped,
            json!({
                "battery_sense": { "cal": [null, -3] },
                "machine": { "mode": "balance", "balance": { "gains": [3491, null, 197, null] } },
                "device": { "name": "rover-left" },
                "motors": [
                    { "current": { "cal": 470 }, "gate": { "dead_time": 25 }, "hall": { "a": { "port": "C", "pin": 13 } } },
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

    /// `f` with its tokens resolved on `chip` against the board's store.
    fn resolved(lp: &mut Loop, chip: hal::pincap::Chip, mut f: Files) -> Files {
        let reads = records(lp);
        let stored = state_after(&reads, &Plan::default());
        crate::checks::resolve(chip, &mut f.entries, &stored).unwrap();
        f
    }

    /// `specs/cli-onboarding.md` test C4: a board holding the split 2.x.20 store dumps its pins as
    /// tokens, the same file from either chip, and applying the dump writes nothing; a pin whose
    /// stored `af` the table would not give dumps as its object.
    #[test]
    fn a_dump_writes_a_pin_as_its_token_where_the_token_resolves_back() {
        use hal::pincap::Chip;
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../boards/split/2.x.20/master.json"
        );
        let text = std::fs::read_to_string(path).unwrap();
        let tokens: Json = serde_json::from_str(&text).unwrap();
        let mut dumps = Vec::new();
        for chip in [Chip::Gd32f130c8, Chip::Gd32f103c8] {
            let mut lp = Loop::new();
            let f = resolved(&mut lp, chip, files(&[("master.json", tokens.clone())]));
            run(&mut lp, &f, false).unwrap();
            let mut b = board(&mut lp);
            let (reads, _) = read_board(&mut b, &dump_keys()).unwrap();
            let dumped = to_file(&reads, Pins::Chip(chip));
            assert_eq!(dumped, tokens, "{}", chip.name());
            let before = lp.writes;
            let f = resolved(&mut lp, chip, files(&[("dump.json", dumped.clone())]));
            let (plan, _) = run(&mut lp, &f, false).unwrap();
            assert!(!plan.changes(), "{}", chip.name());
            assert_eq!(lp.writes, before, "{}: nothing written", chip.name());
            dumps.push(serde_json::to_string_pretty(&dumped).unwrap());
        }
        assert_eq!(dumps[0], dumps[1], "the same text from either chip");
        // USART1's TX is AF1 on PA2 on the GD32F130: a stored AF0 is no token's.
        let mut lp = Loop::new();
        let link = json!({ "links": [{ "tx": { "port": "A", "pin": 2, "af": 0 }, "rx": "PA3", "usart": 1 }] });
        let f = resolved(&mut lp, Chip::Gd32f130c8, files(&[("link.json", link)]));
        run(&mut lp, &f, false).unwrap();
        let mut b = board(&mut lp);
        let (reads, _) = read_board(&mut b, &dump_keys()).unwrap();
        assert_eq!(
            to_file(&reads, Pins::Chip(Chip::Gd32f130c8))["links"][0],
            json!({ "tx": { "port": "A", "pin": 2, "af": 0 }, "rx": "PA3", "usart": 1 })
        );
    }

    /// `specs/machine-settings.md` sections 3.1 and 6.2: the regrouped node paths, `links` as an
    /// array of objects like `motors`, and `active_high` as a list of input pin paths, from a file
    /// to the board and back to the same file.
    #[test]
    fn the_node_paths_links_and_active_high_round_trip() {
        let layout = json!({
            "power": { "self_hold": { "port": "B", "pin": 12 }, "button": { "port": "A", "pin": 5 } },
            "battery_sense": { "pin": { "port": "A", "pin": 4, "adc": 4 }, "cal": [25000, -5] },
            "buzzer": { "port": "B", "pin": 9 },
            "leds": { "green": { "port": "B", "pin": 3 }, "orange": { "port": "A", "pin": 15 }, "red": { "port": "B", "pin": 4 } },
            "pads": { "a": { "port": "A", "pin": 0 } },
            "active_high": ["power.button", "pads.b"],
            "links": [{}, { "tx": { "port": "B", "pin": 6, "af": 1 }, "rx": { "port": "B", "pin": 7, "af": 1 } }],
            "motors": [{ "side": "right" }],
        });
        let mut lp = Loop::new();
        let f = files(&[("layout.json", layout.clone())]);
        let (plan, _) = run(&mut lp, &f, false).unwrap();
        let keys: Vec<Key> = plan.steps.iter().map(|s| s.key).collect();
        assert!(keys.contains(&k(0x0F, 1)) && keys.contains(&k(0x10, 1)));
        assert!(keys.contains(&k(0x19, 0)) && keys.contains(&k(0x50, 0)));
        let r = nodes::read(lp.board_wire(), 0x80, 0x01, k(0x19, 0)).unwrap();
        assert_eq!(r.bytes, [0b101], "button b0 and pad B b2");

        let mut b = board(&mut lp);
        let dumped = to_file(
            &read_board(&mut b, &dump_keys()).unwrap().0,
            Pins::Objects { remaps: false },
        );
        assert_eq!(dumped, layout);
        let f = files(&[("dump.json", dumped)]);
        let (plan, _) = run(&mut lp, &f, false).unwrap();
        assert!(!plan.changes(), "{:?}", plan.steps);

        // Link 0 alone is a one-element array.
        let mut lp = Loop::new();
        let link0 = json!({ "links": [{ "tx": { "port": "A", "pin": 2, "af": 1 }, "rx": { "port": "A", "pin": 3, "af": 1 } }] });
        run(&mut lp, &files(&[("l.json", link0.clone())]), false).unwrap();
        let mut b = board(&mut lp);
        assert_eq!(
            to_file(
                &read_board(&mut b, &dump_keys()).unwrap().0,
                Pins::Objects { remaps: false },
            ),
            link0
        );
    }

    /// `specs/tones.md` section 4: a file names the tunes by event under the node's `tone`, from a
    /// file to the board and back to the same file; a dotted member names one too, and an unknown
    /// event is refused.
    #[test]
    fn the_tunes_round_trip_by_event_name() {
        let layout = json!({
            "tone": { "horn": "4000/30", "battery_critical": "" },
        });
        let mut lp = Loop::new();
        let f = files(&[("tones.json", layout.clone())]);
        let (plan, _) = run(&mut lp, &f, false).unwrap();
        let keys: Vec<Key> = plan.steps.iter().map(|s| s.key).collect();
        assert!(keys.contains(&k(0x1F, 0)) && keys.contains(&k(0x1F, 2)));
        let r = nodes::read(lp.board_wire(), 0x80, 0x01, k(0x1F, 0)).unwrap();
        assert_eq!(r.bytes, [0xA0, 0x0F, 30]);

        let mut b = board(&mut lp);
        let dumped = to_file(
            &read_board(&mut b, &dump_keys()).unwrap().0,
            Pins::Objects { remaps: false },
        );
        assert_eq!(dumped, layout);
        let (plan, _) = run(&mut lp, &files(&[("dump.json", dumped)]), false).unwrap();
        assert!(!plan.changes(), "{:?}", plan.steps);

        let f = files(&[("d.json", json!({ "tone.battery_low": "2000/10" }))]);
        assert!(f.problems.is_empty(), "{:?}", f.problems);
        assert_eq!(f.entries[0].key, k(0x1F, 1));

        let f = files(&[(
            "e.json",
            json!({ "tone": { "hron": "2000/10" }, "machine": { "tone": { "horn": "2000/10" } } }),
        )]);
        let msgs: Vec<&str> = f.problems.iter().map(|p| p.msg.as_str()).collect();
        assert_eq!(
            msgs,
            [
                "e.json: unknown field tone.hron (did you mean tone.horn?)",
                "e.json: unknown field machine.tone (did you mean machine.mode?)",
            ]
        );
    }

    #[test]
    fn a_field_outside_its_object_array_is_refused_naming_where_it_goes() {
        let f = files(&[(
            "a.json",
            json!({
                "tx": { "port": "A", "pin": 2, "af": 1 },
                "links": [{ "hall": { "a": { "port": "C", "pin": 13 } }, "power": { "button": { "port": "A", "pin": 5 } } }],
                "motors": [{ "rx": { "port": "A", "pin": 3, "af": 1 } }],
                "leds": [{ "green": { "port": "B", "pin": 3 } }],
            }),
        )]);
        let msgs: Vec<&str> = f.problems.iter().map(|p| p.msg.as_str()).collect();
        assert_eq!(
            msgs,
            [
                "a.json: tx is per link (links[<n>].tx)",
                "a.json: hall is per motor (motors[<n>].hall)",
                "a.json: power is not in links (it goes at the top level)",
                "a.json: rx is per link (links[<n>].rx)",
                "a.json: unknown field leds",
            ]
        );
        let f = files(&[(
            "b.json",
            json!({ "links": { "tx": { "port": "A", "pin": 2, "af": 1 } } }),
        )]);
        assert_eq!(
            f.problems,
            [Problem::stop(
                "b.json: links takes a list, one object per link"
            )]
        );
    }

    #[test]
    fn a_replace_from_a_dump_unstores_exactly_the_extra_keys_and_reproduces_the_records() {
        let mut source = staged();
        let dumped = {
            let mut b = board(&mut source);
            to_file(
                &read_board(&mut b, &dump_keys()).unwrap().0,
                Pins::Objects { remaps: false },
            )
        };
        // The same layout plus records from an earlier life, a board-owned one among them.
        let mut target = staged();
        let extra = files(&[(
            "old.json",
            json!({
                "motors": [{}, { "hall": { "a": { "port": "B", "pin": 5 } } }],
                "battery_sense": { "cal": [25000] },
                "machine": { "frame": "rigid" },
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
        assert_eq!(unstored, [k(0x18, 0), k(0x21, 0), k(0x40, 1)]);
        assert_eq!(
            lines,
            [
                "rover-left: 0 changed, 0 pinned, 10 unchanged, 3 un-stored",
                "rover-left: battery_sense.cal[0]: 25000 -> 25200 (default)",
                "rover-left: machine.frame: rigid -> pivoting (default)",
                "rover-left: motors[1].hall.a: PB5 -> none (default)",
                "1 takes effect at the next arm, 2 at the next boot",
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
                "machine": { "balance": { "gains": [3491, 4002] } },
                "motors": [{ "gate": { "dead_time": 28 }, "hall": { "a": { "port": "C", "pin": 13 } }, "current": { "noise_floor": 2100 } }],
            }),
        )]);
        let (plan, lines) = run(&mut lp, &f, false).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 2 changed, 1 pinned, 2 unchanged, 5 kept",
                "rover-left: machine.balance.gains[1]: 0 (default) -> 4002",
                "rover-left: motors[0].gate.dead_time: 25 -> 28",
                "rover-left: motors[0].current.noise_floor: 2100 (default) -> 2100",
                "rover-left: kept: device.name = \"rover-left\" (stored, not in the file)",
                "rover-left: kept: battery_sense.cal[1] = -3 (stored, not in the file)",
                "rover-left: kept: machine.mode = balance (stored, not in the file)",
                "rover-left: kept: machine.balance.gains[2] = 197 (stored, not in the file)",
                "rover-left: kept: motors[0].current.cal = 470 (stored, not in the file)",
                // The gain is live and the pinned value runs already: only the dead time waits.
                "1 takes effect at the next boot",
            ]
        );
        assert_eq!(plan.kept.len(), 5);
        // The pinned value is now stored.
        let r = nodes::read(lp.board_wire(), 0x80, 0x01, k(0x4D, 0)).unwrap();
        assert_eq!(r.status, CFG_OK);
    }

    /// `specs/machine-settings.md` section 5: the line counts the changed keys by the registry's
    /// effect; a live or client-only field adds nothing.
    #[test]
    fn the_effect_line_counts_arm_and_boot_keys() {
        assert_eq!(
            effect_line([k(0x20, 0), k(0x2B, 1), k(0x50, 0), k(0x40, 0)]),
            Some("3 take effect at the next arm, 1 at the next boot".into())
        );
        assert_eq!(
            effect_line([k(0x0F, 0), k(0x10, 0)]),
            Some("2 take effect at the next boot".into())
        );
        assert_eq!(
            effect_line([k(0x15, 2)]),
            Some("1 takes effect at the next arm".into())
        );
        assert_eq!(effect_line([k(0x27, 0), k(0x29, 1), k(0x24, 0)]), None);
        assert_eq!(effect_line([k(0x99, 0)]), None);
    }

    #[test]
    fn an_armed_board_refuses_and_a_board_that_arms_partway_keeps_the_keys_before_it() {
        let f = files(&[(
            "m.json",
            json!({ "machine": { "mode": "balance", "frame": "rigid", "rider_required": false } }),
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
                "rover-left: machine.mode: refused: armed".to_string(),
                vec!["rover-left: nothing was written".to_string()]
            )
        );
        assert_eq!(records(&mut lp)[&k(0x20, 0)].status, CFG_DEFAULT);

        // Arms after the first write: that key is read back and reported as the board holds it.
        let mut lp = Loop::new();
        lp.arm_after = Some(1);
        assert_eq!(
            refused(&mut lp),
            (
                "rover-left: machine.frame: refused: armed".to_string(),
                vec![
                    "rover-left: written before the refusal:".to_string(),
                    "rover-left: machine.mode: throttle (default) -> balance".to_string(),
                ]
            )
        );
        let r = records(&mut lp);
        assert_eq!(
            (r[&k(0x20, 0)].status, r[&k(0x21, 0)].status),
            (CFG_OK, CFG_DEFAULT)
        );
    }

    /// `apply` never un-stores: a `null` (a position a dump of another board did not store) leaves
    /// the key alone, listed as kept when the board stores it and unchanged when it does not;
    /// `replace` un-stores it (`specs/cli-config.md` sections 2.1 and 4.3).
    #[test]
    fn a_null_unstores_under_replace_and_is_left_alone_under_apply() {
        let f = files(&[(
            "dump.json",
            json!({ "machine": { "balance": { "gains": [3491, null, null] } } }),
        )]);
        let stored = |v: i16| ConfigResp {
            status: CFG_OK,
            tag: Type::I16.tag(),
            bytes: v.to_le_bytes().to_vec(),
        };
        let default = ConfigResp {
            status: CFG_DEFAULT,
            ..stored(0)
        };
        let reads: Reads = [
            (k(0x27, 0), stored(3491)),
            (k(0x27, 1), stored(4002)),
            (k(0x27, 2), default),
        ]
        .into_iter()
        .collect();
        let apply = plan(&f, &reads, &[], false).unwrap();
        assert!(apply.steps.is_empty(), "{:?}", apply.steps);
        assert_eq!(apply.unchanged, 2);
        assert_eq!(apply.kept, [(k(0x27, 1), Kept::NullInFile)]);
        assert_eq!(
            report("rover-left", &apply, &reads, &BTreeMap::new()),
            [
                "rover-left: 0 changed, 0 pinned, 2 unchanged, 1 kept",
                "rover-left: kept: machine.balance.gains[1] = 4002 (stored, null in the file)",
            ]
        );
        let replace = plan(&f, &reads, &[], true).unwrap();
        let steps: Vec<(Key, Act)> = replace.steps.iter().map(|s| (s.key, s.act)).collect();
        assert_eq!(steps, [(k(0x27, 1), Act::Unstore)]);
        assert!(replace.kept.is_empty());
    }

    /// An un-store of a key whose default is the node's (a bridge's `device.name`) shows
    /// `(default)` with no value before the write, and the read-back's value after it; a key whose
    /// default is the table's shows that default (`specs/cli-config.md` section 4.3).
    #[test]
    fn an_unstored_node_default_shows_no_value_until_read_back() {
        let f = files(&[("empty.json", json!({}))]);
        let name = ConfigResp {
            status: CFG_OK,
            tag: Type::Str.tag(),
            bytes: b"rover-esp".to_vec(),
        };
        let mode = ConfigResp {
            status: CFG_OK,
            tag: Type::U8.tag(),
            bytes: vec![1],
        };
        let reads: Reads = [(k(0x02, 0), name), (k(0x20, 0), mode)]
            .into_iter()
            .collect();
        let plan = plan(&f, &reads, &[], true).unwrap();
        assert_eq!(
            report("esp", &plan, &reads, &BTreeMap::new()),
            [
                "esp: 0 changed, 0 pinned, 0 unchanged, 2 un-stored",
                "esp: device.name: \"rover-esp\" -> (default)",
                "esp: machine.mode: balance -> throttle (default)",
                "1 takes effect at the next arm, 1 at the next boot",
            ]
        );
        let after: BTreeMap<Key, ConfigResp> = [
            (
                k(0x02, 0),
                ConfigResp {
                    status: CFG_DEFAULT,
                    tag: Type::Str.tag(),
                    bytes: b"hb-bridge-ab12".to_vec(),
                },
            ),
            (
                k(0x20, 0),
                ConfigResp {
                    status: CFG_DEFAULT,
                    tag: Type::U8.tag(),
                    bytes: vec![0],
                },
            ),
        ]
        .into_iter()
        .collect();
        assert_eq!(
            report("esp", &plan, &reads, &after)[1],
            "esp: device.name: \"rover-esp\" -> \"hb-bridge-ab12\" (default)"
        );
    }

    #[test]
    fn the_file_refusals_stop_and_the_forceable_ones_say_what_force_does() {
        let f = files(&[(
            "a.json",
            json!({
                "machine": { "mod": "balance", "frame": "bent", "battery": { "cells": "ten" } },
                "motors": [{ "side": 5 }, {}, { "hall": { "a": { "port": "A", "pin": 1 } } }],
                "battery_sense": { "cal": [1, 2, 3] },
                "node": { "address": 4 },
                "hall": { "b": { "port": "A", "pin": 2 } },
                "0x99.0": 5,
                "0x21": "rigid",
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
                (false, "a.json: unknown field machine.mod (did you mean machine.mode?)"),
                (false, "a.json: machine.frame: unknown label bent (pivoting, rigid)"),
                (false, "a.json: machine.battery.cells: an integer, not \"ten\""),
                (true, "a.json: motors[0].side takes left, right, not 5 (--force writes it anyway)"),
                (true, "a.json: hall.a has 2 motors (--force writes motors[2] anyway)"),
                (true, "a.json: battery_sense.cal has 2 indices (--force writes index 2 anyway)"),
                (true, "a.json: node.address is board-owned (--force writes it anyway)"),
                (false, "a.json: hall is per motor (motors[<n>].hall)"),
                (
                    true,
                    "a.json: 0x99 is not in the name table (--force writes it with the type the board reports)"
                ),
                (false, "a.json: machine.frame is named twice"),
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
                json!({ "motors": [{ "hall": { "a": { "port": "C", "pin": 13 } } }] }),
            ),
            (
                "machine.json",
                json!({ "machine": { "mode": "balance" }, "0x40.0": "PB5" }),
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
                json!({ "motors": [{ "hall": { "a": { "port": "C", "pin": 13 } } }] }),
            ),
            ("machine.json", json!({ "machine": { "mode": "balance" } })),
        ]);
        assert!(f.problems.is_empty());
        let keys: Vec<Key> = f.entries.iter().map(|e| e.key).collect();
        assert_eq!(keys, [k(0x40, 0), k(0x20, 0)]);
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
        let f = files(&[("x.json", json!({ "0x4E": 12000 }))]);
        assert!(f.problems.is_empty());
        let (plan, _) = run(&mut lp, &f, false).unwrap();
        assert_eq!(plan.steps[0].storage, Type::U32);
        assert_eq!(plan.steps[0].bytes, Some(12000u32.to_le_bytes().to_vec()));
    }

    /// `specs/bridge-onboarding.md` section 2.2 on a board model: `get`, `set`, `dump`, `apply`,
    /// `replace` and `diff` (the plan `diff` prints) never print the password, write it whenever a
    /// file gives it, and `replace` never un-stores it unless a file says `null`.
    #[test]
    fn a_secret_is_never_printed_written_whenever_given_and_kept_by_replace() {
        const PASS: &str = "hunter2hunter2";
        let pass = k(0x04, 0);
        let mut lp = Loop::new();
        let mut all: Vec<String> = Vec::new();

        // config: (default) unstored, (set) once written.
        let mut b = board(&mut lp);
        let l = config_read(&mut b, pass).unwrap();
        assert_eq!(l, "rover-left: wifi.pass = (default)");
        all.push(l);
        let w = config_write(&mut b, pass, PASS, false, None).unwrap();
        assert_eq!(
            w.lines(),
            [
                "rover-left: wifi.pass: (default) -> (set)",
                "1 takes effect at the next boot"
            ]
        );
        all.extend(w.lines());
        let l = config_read(&mut b, pass).unwrap();
        assert_eq!(l, "rover-left: wifi.pass = (set)");
        all.push(l);

        // apply: the password is written every time and counted as changed, so diff's plan
        // always has a change for a file that carries it.
        let creds = files(&[(
            "wifi.json",
            json!({ "wifi": { "ssid": "bench", "pass": PASS } }),
        )]);
        let (plan, lines) = run(&mut lp, &creds, false).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 2 changed, 0 pinned, 0 unchanged",
                "rover-left: wifi.ssid: \"\" (default) -> \"bench\"",
                "rover-left: wifi.pass: (set) -> (set)",
                "2 take effect at the next boot",
            ]
        );
        assert!(plan.changes());
        all.extend(lines);
        let (plan, lines) = run(&mut lp, &creds, false).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 1 changed, 0 pinned, 1 unchanged",
                "rover-left: wifi.pass: (set) -> (set)",
                "1 takes effect at the next boot",
            ]
        );
        assert!(
            plan.changes(),
            "diff exits 1 for a file that carries the password"
        );
        all.extend(lines);

        // dump: the password is left out of the file and named on its own line.
        let mut b = board(&mut lp);
        let (reads, _) = read_board(&mut b, &dump_keys()).unwrap();
        assert_eq!(reads[&pass].status, CFG_SECRET);
        let dumped = to_file(&reads, Pins::Objects { remaps: false });
        assert_eq!(dumped, json!({ "wifi": { "ssid": "bench" } }));
        let notes = left_out("rover-left", &reads);
        assert_eq!(
            notes,
            ["rover-left: wifi.pass stored, not readable, left out"]
        );
        all.extend(notes);

        // replace from the dump keeps the password: nothing to write, nothing to un-store.
        let f = files(&[("dump.json", dumped)]);
        let (plan, lines) = run(&mut lp, &f, true).unwrap();
        assert!(!plan.changes(), "{:?}", plan.steps);
        assert_eq!(lines, ["rover-left: 0 changed, 0 pinned, 1 unchanged"]);
        assert_eq!(records(&mut lp)[&pass].status, CFG_SECRET);
        // apply leaves it alone under a null, and lists it as kept.
        let null = files(&[("null.json", json!({ "wifi": { "pass": null } }))]);
        let (_, lines) = run(&mut lp, &null, false).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 0 changed, 0 pinned, 0 unchanged, 2 kept",
                "rover-left: kept: wifi.pass = (set) (stored, null in the file)",
                "rover-left: kept: wifi.ssid = \"bench\" (stored, not in the file)",
            ]
        );
        all.extend(lines);
        // replace un-stores it only when a file says null.
        let (_, lines) = run(&mut lp, &null, true).unwrap();
        assert_eq!(
            lines,
            [
                "rover-left: 0 changed, 0 pinned, 0 unchanged, 2 un-stored",
                "rover-left: wifi.pass: (set) -> (default)",
                "rover-left: wifi.ssid: \"bench\" -> \"\" (default)",
                "2 take effect at the next boot",
            ]
        );
        all.extend(lines);
        assert_eq!(records(&mut lp)[&pass].status, CFG_DEFAULT);

        // A value of the wrong type is refused without echoing it.
        let wrong = files(&[("w.json", json!({ "wifi": { "pass": 12345678 } }))]);
        let msgs: Vec<&str> = wrong.problems.iter().map(|p| p.msg.as_str()).collect();
        assert_eq!(msgs, ["w.json: wifi.pass: a string, not a number"]);
        all.extend(msgs.iter().map(|m| m.to_string()));
        // One longer than a CONFIG_WRITE carries is refused by the file check, before any read.
        let max = net::walk::CONFIG_WRITE_VALUE_MAX;
        let long = "p".repeat(max + 1);
        let f = files(&[("w.json", json!({ "wifi": { "pass": long } }))]);
        let msgs: Vec<&str> = f.problems.iter().map(|p| p.msg.as_str()).collect();
        assert_eq!(
            msgs,
            [format!(
                "w.json: wifi.pass: {} bytes, more than the {max} one CONFIG_WRITE carries",
                max + 1
            )]
        );
        assert!(!msgs[0].contains(&long));

        for l in &all {
            assert!(!l.contains(PASS) && !l.contains("12345678"), "{l}");
        }
    }

    /// A file that carries a pin has its `af` form checked whether or not the write changes the
    /// pin: the chip is needed, and a GD32F103 refuses an `af` the board already stores as 0.
    #[test]
    fn a_file_with_a_pin_is_form_checked_when_the_plan_changes_nothing() {
        let f = files(&[(
            "b.json",
            json!({ "imu": { "scl": { "port": "B", "pin": 6, "af": 0 } } }),
        )]);
        let plan = Plan::default();
        let reads = Reads::default();
        let e = check_plan(None, &f, &reads, &plan).unwrap_err();
        assert!(
            e.starts_with("imu.scl: the pin checks need the board's chip"),
            "{e}"
        );
        let e = check_plan(Some(hal::pincap::Chip::Gd32f103c8), &f, &reads, &plan).unwrap_err();
        assert_eq!(
            e,
            "imu.scl: the gd32f103c8 routes by its remap and stores no af: leave af out"
        );
        assert_eq!(
            check_plan(Some(hal::pincap::Chip::Gd32f130c8), &f, &reads, &plan),
            Ok(())
        );
    }

    /// `specs/no-fleet-pins.md` section 2: an unstored latch, buzzer or LED reads unset like every
    /// other pin, with no fleet pin shown, and a dump leaves it out.
    #[test]
    fn an_unstored_latch_buzzer_or_led_reads_unset_and_is_not_dumped() {
        let mut lp = Loop::new();
        let mut b = board(&mut lp);
        for (key, path) in [
            (k(0x06, 0), "power.self_hold"),
            (k(0x08, 0), "buzzer"),
            (k(0x09, 0), "leds.green"),
            (k(0x0A, 0), "leds.orange"),
            (k(0x0B, 0), "leds.red"),
        ] {
            assert_eq!(
                config_read(&mut b, key).unwrap(),
                format!("rover-left: {path} = none (default)")
            );
        }
        let (reads, _) = read_board(&mut b, &dump_keys()).unwrap();
        assert_eq!(to_file(&reads, Pins::Objects { remaps: false }), json!({}));
    }

    /// `specs/cli-onboarding.md` part C and bench gate G4 on the host: `board set` takes a pin as
    /// its token, resolved on the board's chip with the group's stored instance, and refuses one it
    /// cannot resolve.
    #[test]
    fn board_set_takes_a_pin_token() {
        use hal::pincap::Chip;
        for chip in [Chip::Gd32f103c8, Chip::Gd32f130c8] {
            let mut lp = Loop::new();
            let mut b = board(&mut lp);
            let w = config_write(
                &mut b,
                store::BOARD_SELF_HOLD.key(),
                "PB12",
                false,
                Some(chip),
            )
            .unwrap();
            assert_eq!(
                w.line,
                "rover-left: power.self_hold: none (default) -> PB12"
            );
            let e = config_write(&mut b, store::LINK_TX.at(0).key(), "PA2", false, Some(chip))
                .unwrap_err();
            assert_eq!(
                e,
                "rover-left: links[0].tx: give links[0].usart to resolve PA2"
            );
        }
        // On the GD32F130 the token takes the table's AF, on the instance the board stores.
        let mut lp = Loop::new();
        let link = json!({ "links": [{
            "tx": { "port": "A", "pin": 2, "af": 1 },
            "rx": { "port": "A", "pin": 3, "af": 1 },
            "usart": 1,
        }] });
        run(&mut lp, &files(&[("link.json", link)]), false).unwrap();
        let mut b = board(&mut lp);
        let f130 = Some(Chip::Gd32f130c8);
        let w = config_write(&mut b, store::LINK_RX.at(0).key(), "PA3", false, f130).unwrap();
        assert_eq!(w.line, "rover-left: links[0].rx: PA3 af1 -> PA3 af1");
        assert_eq!(w.changed, None);
    }

    #[test]
    fn config_reads_and_writes_one_field_with_the_spec_lines() {
        let mut lp = Loop::new();
        let mut b = board(&mut lp);
        assert_eq!(
            config_read(&mut b, k(0x4C, 0)).unwrap(),
            "rover-left: motors[0].current.cal = 455 (default)"
        );
        let lines = |b: &mut Board<'_, Loop>, key, text| {
            config_write(b, key, text, false, Some(hal::pincap::Chip::Gd32f130c8))
                .unwrap()
                .lines()
        };
        assert_eq!(
            lines(&mut b, k(0x20, 0), "balance"),
            [
                "rover-left: machine.mode: throttle (default) -> balance",
                "1 takes effect at the next arm"
            ]
        );
        assert_eq!(
            lines(&mut b, k(0x0C, 0), r#"{"port": "B", "pin": 5}"#),
            [
                "rover-left: pads.a: none (default) -> PB5",
                "1 takes effect at the next boot"
            ]
        );
        assert_eq!(
            lines(&mut b, k(0x07, 0), r#"{"port": "A", "pin": 4, "adc": 4}"#),
            [
                "rover-left: battery_sense.pin: none (default) -> PA4 adc4",
                "1 takes effect at the next boot"
            ]
        );
        // One pin of a group alone is an incomplete group, and the board checks refuse it.
        let e = config_write(
            &mut b,
            k(0x40, 1),
            r#"{"port": "B", "pin": 6}"#,
            true,
            Some(hal::pincap::Chip::Gd32f130c8),
        )
        .unwrap_err();
        assert_eq!(
            e,
            "rover-left: motors[1].hall.a: the group is incomplete: motors[1].hall.b is not stored"
        );
        // The un-store path (`board set <path> null`) runs the checks on the board it would
        // leave: with the IMU switched on, un-storing a pin leaves its group incomplete.
        let f130 = Some(hal::pincap::Chip::Gd32f130c8);
        for (key, text) in [
            (
                store::IMU_SCL_PIN.key(),
                r#"{"port": "B", "pin": 6, "af": 1}"#,
            ),
            (
                store::IMU_SDA_PIN.key(),
                r#"{"port": "B", "pin": 7, "af": 1}"#,
            ),
            (store::IMU_I2C.key(), "0"),
            (store::IMU_MODEL.key(), "2"),
        ] {
            config_write(&mut b, key, text, false, f130).unwrap();
        }
        let e = config_write(&mut b, store::IMU_SDA_PIN.key(), "null", false, f130).unwrap_err();
        assert_eq!(
            e,
            "rover-left: imu.scl: the group is incomplete: imu.sda is not stored"
        );
        // With the IMU switched off the bus may be partly stored, so the same un-store passes.
        config_write(&mut b, store::IMU_MODEL.key(), "0", false, f130).unwrap();
        config_write(&mut b, store::IMU_SDA_PIN.key(), "null", false, f130).unwrap();
        // Without the chip, a pin write is refused before anything is written.
        let e = config_write(
            &mut b,
            k(0x0D, 0),
            r#"{"port": "B", "pin": 7}"#,
            false,
            None,
        )
        .unwrap_err();
        assert!(e.contains("the pin checks need the board's chip"), "{e}");
        assert_eq!(
            lines(&mut b, k(0x19, 0), "power.button"),
            [
                "rover-left: active_high: [\"pads.a\",\"pads.b\"] (default) -> [\"power.button\"]",
                "1 takes effect at the next boot"
            ]
        );
        // A live field takes effect as it is written: no line says when.
        assert_eq!(
            lines(&mut b, k(0x27, 0), "3200"),
            ["rover-left: machine.balance.gains[0]: 0 (default) -> 3200"]
        );
        lines(&mut b, k(0x4C, 0), "470");
        assert_eq!(
            lines(&mut b, k(0x4C, 0), "null"),
            [
                "rover-left: motors[0].current.cal: 470 -> 455 (default)",
                "1 takes effect at the next arm"
            ]
        );
        // Writing the default changes nothing the board runs.
        assert_eq!(
            lines(&mut b, k(0x18, 1), "-5"),
            ["rover-left: battery_sense.cal[1]: -5 (default) -> -5"]
        );
        assert_eq!(
            config_write(&mut b, k(0x20, 1), "balance", false, None).unwrap_err(),
            "machine.mode has one index (--force writes index 1 anyway)"
        );
        assert_eq!(
            config_write(&mut b, k(0x20, 1), "balance", true, None).unwrap(),
            Written {
                warnings: Vec::new(),
                line: "rover-left: machine.mode[1]: throttle (default) -> balance".into(),
                changed: Some(k(0x20, 1)),
            }
        );
        // An arm-row range is a board check: --force does not override it.
        for force in [false, true] {
            assert_eq!(
                config_write(&mut b, k(0x51, 0), "9", force, None).unwrap_err(),
                "rover-left: motors[0].align_offset: 9 is outside 0..5"
            );
        }
        b.wire.board.resp.set_armed(true);
        assert_eq!(
            config_write(&mut b, k(0x20, 0), "throttle", true, None).unwrap_err(),
            "rover-left: machine.mode: refused: armed"
        );
    }
}
