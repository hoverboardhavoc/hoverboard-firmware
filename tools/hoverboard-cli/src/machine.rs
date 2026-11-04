//! The `machine` commands' rules (`specs/machine-settings.md` sections 6.2 and 6.3): the machine
//! file, matching its `boards` entries to the motor controllers the walk found, the machine fields
//! whose copies differ, the digests the boards report, and the marks `machine nodes` prints.

use linkctl::CyclicState;
use net::walk::{CLASS_MOTOR, UID_LEN};
use serde_json::{Map, Value as Json};
use store::{Key, Scope};

use crate::config::{read_files, shown, to_file, Files, Reads};
use crate::names::{path_of, row_of, table_keys, Problem};
use crate::nodes::uid_hex;

/// A factory unique ID.
pub type Uid = [u8; UID_LEN];

/// The `uid` a `boards` key or a `--map` names: 24 hex digits.
pub fn parse_uid(s: &str) -> Option<Uid> {
    if s.len() != 2 * UID_LEN || !s.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let mut u = [0u8; UID_LEN];
    for (i, b) in u.iter_mut().enumerate() {
        *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(u)
}

/// A `uid` as a line shows it: its first four digits.
pub fn short(uid: &Uid) -> String {
    format!("{}...", &uid_hex(uid)[..4])
}

/// One `boards` entry: the board's `uid` and its settings, checked.
#[derive(Debug)]
pub struct Entry {
    /// The key it is filed under.
    pub uid: Uid,
    /// Its settings, a board file without `machine`.
    pub files: Files,
}

/// A machine file (section 6.2): the machine settings once, and one entry per board.
#[derive(Debug, Default)]
pub struct MachineFile {
    /// The `machine` section; empty when the file has none.
    pub machine: Files,
    /// The `boards` entries, in file order.
    pub entries: Vec<Entry>,
}

impl MachineFile {
    /// What one board is given: its entry, when it has one, and the machine section.
    pub fn board_files(&self, entry: Option<usize>) -> Files {
        let mut entries = self.machine.entries.clone();
        if let Some(e) = entry.and_then(|i| self.entries.get(i)) {
            entries.extend(e.files.entries.iter().cloned());
        }
        Files {
            entries,
            problems: Vec::new(),
        }
    }

    /// Every problem in the file, each prefixed with where it is.
    pub fn problems(&self) -> Vec<Problem> {
        let entries = self.entries.iter().flat_map(|e| e.files.problems.clone());
        self.machine
            .problems
            .iter()
            .cloned()
            .chain(entries)
            .collect()
    }
}

/// Read `text` (file `name`) as a machine file. A file with any top-level key but `machine` and
/// `boards` is a board file, refused naming the noun that takes it. `Err` for a file that is no
/// machine file at all; the problems inside one are in [`MachineFile::problems`].
pub fn read_machine_file(name: &str, text: &str) -> Result<MachineFile, String> {
    let top = match serde_json::from_str::<Json>(text) {
        Ok(Json::Object(top)) => top,
        Ok(_) => return Err(format!("{name}: a machine file is a JSON object")),
        Err(e) => return Err(format!("{name}: not JSON: {e}")),
    };
    if let Some(k) = top.keys().find(|k| *k != "machine" && *k != "boards") {
        return Err(format!(
            "{name} is a board file (it has \"{k}\"): give it to the board commands"
        ));
    }
    let mut file = MachineFile::default();
    if let Some(m) = top.get("machine") {
        let only = Json::Object(Map::from_iter([("machine".to_string(), m.clone())]));
        file.machine = read_files(&[(name.to_string(), only.to_string())]);
    }
    let Some(boards) = top.get("boards") else {
        return Ok(file);
    };
    let Some(boards) = boards.as_object() else {
        return Err(format!(
            "{name}: boards takes an object, one entry per board uid"
        ));
    };
    for (key, entry) in boards {
        let uid = parse_uid(key).ok_or_else(|| {
            format!("{name}: boards.{key}: a board is filed under its uid, 24 hex digits")
        })?;
        if file.entries.iter().any(|e| e.uid == uid) {
            return Err(format!("{name}: boards.{key} is filed twice"));
        }
        let label = format!("{name}: boards.{}", uid_hex(&uid));
        if entry.get("machine").is_some() {
            return Err(format!(
                "{label} holds machine settings: they go in the file's own machine section"
            ));
        }
        file.entries.push(Entry {
            uid,
            files: read_files(&[(label, entry.to_string())]),
        });
    }
    Ok(file)
}

/// A motor controller the walk found, as matching sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Board {
    /// Its address.
    pub addr: u8,
    /// How lines name it.
    pub label: String,
    /// Its `uid`, when a client may match by it (`crate::target::usable_uid`).
    pub uid: Option<Uid>,
}

/// Which entry goes to which board.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Matching {
    /// Each entry given to a board: the entry's index and the board's address.
    pub pairs: Vec<(usize, u8)>,
    /// The boards no entry goes to.
    pub unlisted: Vec<u8>,
    /// The entries that match no board and have no board to offer them to.
    pub skipped: Vec<usize>,
}

impl Matching {
    /// The entry that goes to `board`.
    pub fn entry_of(&self, board: u8) -> Option<usize> {
        self.pairs.iter().find(|p| p.1 == board).map(|p| p.0)
    }
}

/// What matching decided.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// Every entry placed, given, or skipped.
    Ready(Matching),
    /// One entry matches no board and one board has no entry (a replaced controller): ask whether
    /// the entry goes to that board.
    Ask {
        /// The question, without the `[y/N]`.
        question: String,
        /// The option that answers it without a terminal.
        map: String,
        /// The entry.
        entry: usize,
        /// The board.
        board: u8,
        /// The rest, decided.
        rest: Matching,
    },
}

impl Outcome {
    /// Resolve the question: `answer` is `None` without a terminal, else whether it was yes.
    pub fn decide(self, answer: impl FnOnce(&str) -> Option<bool>) -> Result<Matching, String> {
        match self {
            Outcome::Ready(m) => Ok(m),
            Outcome::Ask {
                question,
                map,
                entry,
                board,
                mut rest,
            } => match answer(&question) {
                Some(true) => {
                    rest.pairs.push((entry, board));
                    rest.unlisted.retain(|&b| b != board);
                    Ok(rest)
                }
                Some(false) => Err("nothing written".into()),
                None => Err(format!(
                    "{question}: give {map} to write it without a terminal"
                )),
            },
        }
    }
}

/// Match each entry (`entries`, the uids it is filed under) to the motor controller reporting
/// that `uid` (section 6.3), with `maps` pairing entries that match none (entry index, board
/// address). A board with no entry is unlisted; an entry with no board is offered to the one
/// unlisted board when there is exactly one of each, skipped when no board is unlisted, and
/// refused with every other count, listing both sides.
pub fn match_entries(
    entries: &[Uid],
    boards: &[Board],
    maps: &[(usize, u8)],
) -> Result<Outcome, String> {
    let board = |a: u8| boards.iter().find(|b| b.addr == a);
    let mut pairs: Vec<(usize, u8)> = Vec::new();
    for (i, uid) in entries.iter().enumerate() {
        if let Some(b) = boards.iter().find(|b| b.uid == Some(*uid)) {
            pairs.push((i, b.addr));
        }
    }
    for &(i, a) in maps {
        let uid = short(&entries[i]);
        let b =
            board(a).ok_or_else(|| format!("--map {uid}: {a:#04x} is not a motor controller"))?;
        if let Some(&(_, at)) = pairs.iter().find(|p| p.0 == i) {
            return Err(format!(
                "--map {uid}: the entry already goes to {}",
                board(at).map_or(String::new(), |b| b.label.clone())
            ));
        }
        if pairs.iter().any(|p| p.1 == a) {
            return Err(format!("--map {uid}: {} already has an entry", b.label));
        }
        pairs.push((i, a));
    }
    let unmatched: Vec<usize> = (0..entries.len())
        .filter(|i| !pairs.iter().any(|p| p.0 == *i))
        .collect();
    let unlisted: Vec<u8> = boards
        .iter()
        .map(|b| b.addr)
        .filter(|a| !pairs.iter().any(|p| p.1 == *a))
        .collect();
    let rest = Matching {
        pairs,
        unlisted: unlisted.clone(),
        skipped: Vec::new(),
    };
    match (&unmatched[..], &unlisted[..]) {
        ([], _) => Ok(Outcome::Ready(rest)),
        (&[entry], &[b]) => {
            let to = board(b).expect("an unlisted board is a board");
            let uid = uid_hex(&entries[entry]);
            Ok(Outcome::Ask {
                question: format!(
                    "{} is not on this machine; write its entry to {}",
                    short(&entries[entry]),
                    to.label
                ),
                map: format!("--map {uid}={b:#04x}"),
                entry,
                board: b,
                rest,
            })
        }
        (_, []) => Ok(Outcome::Ready(Matching {
            skipped: unmatched,
            ..rest
        })),
        _ => {
            let e: Vec<String> = unmatched.iter().map(|&i| uid_hex(&entries[i])).collect();
            let b: Vec<String> = unlisted
                .iter()
                .filter_map(|&a| board(a))
                .map(|b| b.label.clone())
                .collect();
            Err(format!(
                "entries that match no board: {}; boards with no entry: {}; pair them with \
                 --map <uid>=<board>",
                e.join(", "),
                b.join(", ")
            ))
        }
    }
}

/// The entry a `--map` names: the one `uid` key that starts with `prefix` (4 to 24 hex digits).
pub fn map_entry(entries: &[Uid], prefix: &str) -> Result<usize, String> {
    let want = prefix.to_ascii_lowercase();
    let hits: Vec<usize> = (0..entries.len())
        .filter(|&i| crate::target::is_uid_prefix(&want) && uid_hex(&entries[i]).starts_with(&want))
        .collect();
    match hits[..] {
        [i] => Ok(i),
        [] => Err(format!(
            "--map {prefix}: no entry of the file is filed under it"
        )),
        _ => Err(format!(
            "--map {prefix}: more than one entry starts with it"
        )),
    }
}

/// Every key of the machine block (section 3.2), in table order.
pub fn machine_keys() -> Vec<Key> {
    table_keys()
        .into_iter()
        .filter(|k| row_of(*k).is_some_and(|r| r.scope() == Some(Scope::Machine)))
        .collect()
}

/// Whether `path`'s key is a machine field; the refusal that names the board commands when not.
pub fn machine_key(path: &str) -> Result<Key, String> {
    let (key, _) = crate::names::parse_path(path).map_err(|p| p.msg)?;
    match row_of(key).and_then(|r| r.scope()) {
        Some(Scope::Machine) => Ok(key),
        _ => Err(format!(
            "{path} is not a machine field: the machine commands take machine.* paths; use the \
             board commands for it"
        )),
    }
}

/// The value a board runs for a key it answered: the bytes, stored or default.
fn effective(r: &crate::nodes::ConfigResp) -> (u8, &[u8]) {
    (r.tag, &r.bytes)
}

/// The machine fields whose copies differ across `copies` (each board's name and its reads of
/// [`machine_keys`]): one line per key, naming each board's value
/// (`machine.mode: rover-left balance, rover-right throttle (default)`).
pub fn differing(copies: &[(String, Reads)]) -> Vec<String> {
    let mut out = Vec::new();
    for key in machine_keys() {
        let got: Vec<(&str, &crate::nodes::ConfigResp)> = copies
            .iter()
            .filter_map(|(n, r)| Some((n.as_str(), r.get(&key)?)))
            .collect();
        let Some((_, first)) = got.first() else {
            continue;
        };
        if got.iter().all(|(_, r)| effective(r) == effective(first)) {
            continue;
        }
        let each: Vec<String> = got
            .iter()
            .map(|(n, r)| format!("{n} {}", shown(key, r)))
            .collect();
        out.push(format!("{}: {}", path_of(key), each.join(", ")));
    }
    out
}

/// `machine get`'s lines for `key`: one when every copy holds the same value, else one per board.
pub fn get_lines(key: Key, copies: &[(String, crate::nodes::ConfigResp)]) -> Vec<String> {
    let path = path_of(key);
    match copies.first() {
        Some((_, first)) if copies.iter().all(|(_, r)| effective(r) == effective(first)) => {
            let all_default = copies
                .iter()
                .all(|(_, r)| r.status == net::walk::CFG_DEFAULT);
            let r = if all_default {
                first
            } else {
                copies
                    .iter()
                    .map(|(_, r)| r)
                    .find(|r| r.status != net::walk::CFG_DEFAULT)
                    .unwrap_or(first)
            };
            vec![format!("{path} = {}", shown(key, r))]
        }
        _ => copies
            .iter()
            .map(|(n, r)| format!("{n}: {path} = {}", shown(key, r)))
            .collect(),
    }
}

/// The machine file `machine dump` writes: the machine section of copy `from`, and each board's
/// file without it, filed under its `uid`. Each board comes with whether its chip remaps
/// ([`to_file`]).
pub fn dump_file(boards: &[(Uid, Reads, bool)], from: usize) -> Json {
    let mut out = Map::new();
    let mut entries = Map::new();
    for (i, (uid, reads, remaps)) in boards.iter().enumerate() {
        let Json::Object(mut file) = to_file(reads, *remaps) else {
            unreachable!("a board's file is an object")
        };
        let machine = file.remove("machine");
        if i == from {
            if let Some(m) = machine {
                out.insert("machine".into(), m);
            }
        }
        entries.insert(uid_hex(uid), Json::Object(file));
    }
    out.insert("boards".into(), Json::Object(entries));
    Json::Object(out)
}

/// Whether the boards agree on the machine, from the digest each last reported in `CYCLIC_STATE`
/// (`specs/machine-settings.md` section 4.1): each board's name and its digest, `None` when it was
/// not heard.
pub fn digest_line(heard: &[(String, Option<u16>)]) -> String {
    let got: Vec<(&str, u16)> = heard
        .iter()
        .filter_map(|(n, d)| Some((n.as_str(), (*d)?)))
        .collect();
    let silent: Vec<&str> = heard
        .iter()
        .filter(|(_, d)| d.is_none())
        .map(|(n, _)| n.as_str())
        .collect();
    let names = |v: &[(&str, u16)]| v.iter().map(|(n, _)| *n).collect::<Vec<_>>().join(", ");
    let mut line = match &got[..] {
        [] => format!("machine digests: nothing heard from {}", silent.join(", ")),
        [(_, d), ..] if got.iter().all(|(_, e)| e == d) => {
            format!("machine digests agree: {d:#06x} on {}", names(&got))
        }
        _ => {
            let each: Vec<String> = got.iter().map(|(n, d)| format!("{n} {d:#06x}")).collect();
            format!(
                "machine digests differ: {} (\"machine diff\")",
                each.join(", ")
            )
        }
    };
    if !got.is_empty() && !silent.is_empty() {
        line += &format!("; not heard: {}", silent.join(", "));
    }
    line
}

/// What `machine nodes` marks a node with (section 6.3): for a motor controller, `silent` when no
/// `CYCLIC_STATE` of it was heard, else its `EV_MACHINE` by the cause `FLAG_PEER_SILENT` names
/// (`specs/board-route.md` section 4), `FLAG_REARM` and `FLAG_REBOOT`; and the count of its boot
/// report's entries.
pub fn marks(class: Option<u8>, heard: Option<&CyclicState>, status_entries: usize) -> Vec<String> {
    let mut out = Vec::new();
    if class != Some(CLASS_MOTOR) {
        return out;
    }
    match heard {
        None => out.push("silent".to_string()),
        Some(s) => {
            if s.machine_mismatch() && s.peer_silent() {
                out.push("a link peer not heard".into());
            } else if s.machine_mismatch() {
                out.push("machine differs (\"machine diff\")".into());
            }
            if s.rearm() {
                out.push("written, runs at the next arm".into());
            }
            if s.reboot() {
                out.push("written, runs at the next boot".into());
            }
        }
    }
    if status_entries > 0 {
        out.push(format!(
            "{status_entries} status entries (\"board status\")"
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nodes::ConfigResp;
    use net::walk::{CFG_DEFAULT, CFG_OK};
    use serde_json::json;
    use store::Type;

    fn uid(hex: &str) -> Uid {
        parse_uid(hex).unwrap()
    }

    const L: &str = "8adc39323335381255343333";
    const R: &str = "84fc41333235390839393638";
    const NEW: &str = "51ce39393333370134313739";
    const SPARE: &str = "58224535393631174a353930";

    fn board(addr: u8, name: &str, hex: Option<&str>) -> Board {
        Board {
            addr,
            label: format!("{name} ({addr:#04x})"),
            uid: hex.map(uid),
        }
    }

    #[test]
    fn a_machine_file_and_the_files_of_the_wrong_kind() {
        let text = json!({
            "machine": { "mode": "balance", "balance": { "gains": [3200, 880, 240, 300] } },
            "boards": {
                L: { "device": { "name": "rover-left" }, "motors": [{ "side": "left" }] },
                R.to_uppercase(): { "device": { "name": "rover-right" }, "motors": [{ "side": "right" }] },
            },
        })
        .to_string();
        let f = read_machine_file("m.json", &text).unwrap();
        assert!(f.problems().is_empty(), "{:?}", f.problems());
        assert_eq!(f.machine.entries.len(), 5);
        let uids: Vec<Uid> = f.entries.iter().map(|e| e.uid).collect();
        assert_eq!(uids, [uid(L), uid(R)]);
        let keys: Vec<Key> = f
            .board_files(Some(1))
            .entries
            .iter()
            .map(|e| e.key)
            .collect();
        assert_eq!(keys.len(), 7);
        assert_eq!(keys[0], store::CONTROL_MODE.key());
        assert_eq!(keys[5], store::DEVICE_NAME.key());

        // A machine section alone, or boards alone, is a machine file.
        assert!(read_machine_file("m", r#"{ "machine": { "mode": "balance" } }"#).is_ok());
        assert!(read_machine_file("m", r#"{ "boards": {} }"#).is_ok());
        // A board file is refused, naming the noun that takes it.
        assert_eq!(
            read_machine_file("b.json", r#"{ "device": { "name": "x" }, "machine": {} }"#)
                .unwrap_err(),
            "b.json is a board file (it has \"device\"): give it to the board commands"
        );
        assert!(read_machine_file("m", r#"{ "boards": { "8adc": {} } }"#)
            .unwrap_err()
            .contains("24 hex digits"));
        let dup = format!(
            r#"{{ "boards": {{ "{L}": {{}}, "{}": {{}} }} }}"#,
            L.to_uppercase()
        );
        assert!(read_machine_file("m", &dup)
            .unwrap_err()
            .contains("filed twice"));
        let nested =
            format!(r#"{{ "boards": {{ "{L}": {{ "machine": {{ "mode": "balance" }} }} }} }}"#);
        assert!(read_machine_file("m", &nested)
            .unwrap_err()
            .contains("holds machine settings"));
        // Problems inside an entry name it.
        let bad = format!(r#"{{ "boards": {{ "{L}": {{ "leds": {{ "green": "PZ9" }} }} }} }}"#);
        let f = read_machine_file("m", &bad).unwrap();
        assert_eq!(
            f.problems()[0].msg,
            format!("m: boards.{L}: leds.green: a pin object, {{\"port\": \"A\", \"pin\": 8, ...}}, not \"PZ9\"")
        );
        // A machine section holding a board field names it as unknown under machine.
        let f = read_machine_file(
            "m",
            r#"{ "machine": { "leds": { "green": { "port": "B", "pin": 3 } } } }"#,
        )
        .unwrap();
        assert!(
            f.problems()[0].msg.contains("unknown field machine.leds"),
            "{:?}",
            f.problems()
        );
    }

    #[test]
    fn a_board_file_with_boards_is_refused_naming_the_machine_commands() {
        let f = read_files(&[(
            "m.json".into(),
            json!({ "machine": { "mode": "balance" }, "boards": {} }).to_string(),
        )]);
        assert_eq!(
            f.problems,
            [Problem::stop(
                "m.json: a machine file (it has \"boards\"): give it to the machine commands"
            )]
        );
        // A board file's machine section is that board's copy.
        let f = read_files(&[(
            "b.json".into(),
            json!({ "machine": { "mode": "balance" } }).to_string(),
        )]);
        assert!(f.problems.is_empty());
        assert_eq!(f.entries[0].key, store::CONTROL_MODE.key());
    }

    #[test]
    fn every_entry_matches_its_board() {
        let boards = [
            board(0x01, "rover-left", Some(L)),
            board(0x02, "rover-right", Some(R)),
        ];
        let out = match_entries(&[uid(R), uid(L)], &boards, &[]).unwrap();
        assert_eq!(
            out,
            Outcome::Ready(Matching {
                pairs: vec![(0, 0x02), (1, 0x01)],
                unlisted: Vec::new(),
                skipped: Vec::new(),
            })
        );
    }

    /// A board not in the file has no entry: it is unlisted, and given nothing of its own.
    #[test]
    fn a_board_the_file_does_not_list_is_left_alone() {
        let boards = [
            board(0x01, "rover-left", Some(L)),
            board(0x02, "rover-right", Some(R)),
        ];
        let m = match_entries(&[uid(L)], &boards, &[])
            .unwrap()
            .decide(|_| unreachable!())
            .unwrap();
        assert_eq!(m.pairs, [(0, 0x01)]);
        assert_eq!(m.unlisted, [0x02]);
        assert_eq!(m.entry_of(0x02), None);
    }

    #[test]
    fn one_entry_and_one_unlisted_board_asks() {
        let boards = [
            board(0x01, "rover-left", Some(L)),
            board(0x02, "rover-new", Some(NEW)),
        ];
        let out = match_entries(&[uid(L), uid(R)], &boards, &[]).unwrap();
        let Outcome::Ask { question, map, .. } = &out else {
            panic!("{out:?}")
        };
        assert_eq!(
            question,
            "84fc... is not on this machine; write its entry to rover-new (0x02)"
        );
        assert_eq!(map, &format!("--map {R}=0x02"));
        // Yes: the entry goes to the new board.
        let m = out.clone().decide(|_| Some(true)).unwrap();
        assert_eq!(m.pairs, [(0, 0x01), (1, 0x02)]);
        assert!(m.unlisted.is_empty());
        // No: nothing is written.
        assert_eq!(
            out.clone().decide(|_| Some(false)),
            Err("nothing written".into())
        );
        // Without a terminal: refused, naming the --map that answers it.
        let e = out.decide(|_| None).unwrap_err();
        assert!(
            e.ends_with(&format!(
                "give --map {R}=0x02 to write it without a terminal"
            )),
            "{e}"
        );
        // With that --map, nothing is asked.
        let m = match_entries(&[uid(L), uid(R)], &boards, &[(1, 0x02)]).unwrap();
        assert_eq!(
            m,
            Outcome::Ready(Matching {
                pairs: vec![(0, 0x01), (1, 0x02)],
                unlisted: Vec::new(),
                skipped: Vec::new(),
            })
        );
    }

    #[test]
    fn two_of_each_is_refused_and_maps_pair_them() {
        let boards = [
            board(0x01, "rover-new", Some(NEW)),
            board(0x02, "rover-spare", Some(SPARE)),
        ];
        let e = match_entries(&[uid(L), uid(R)], &boards, &[]).unwrap_err();
        assert_eq!(
            e,
            format!(
                "entries that match no board: {L}, {R}; boards with no entry: rover-new (0x01), \
                 rover-spare (0x02); pair them with --map <uid>=<board>"
            )
        );
        let m = match_entries(&[uid(L), uid(R)], &boards, &[(0, 0x02), (1, 0x01)]).unwrap();
        assert_eq!(
            m,
            Outcome::Ready(Matching {
                pairs: vec![(0, 0x02), (1, 0x01)],
                unlisted: Vec::new(),
                skipped: Vec::new(),
            })
        );
        // A map for an entry that matches by its uid, or onto a board already taken, is refused.
        let boards = [
            board(0x01, "rover-left", Some(L)),
            board(0x02, "rover-new", Some(NEW)),
        ];
        assert!(match_entries(&[uid(L), uid(R)], &boards, &[(0, 0x02)])
            .unwrap_err()
            .contains("already goes to rover-left"));
        assert!(match_entries(&[uid(L), uid(R)], &boards, &[(1, 0x01)])
            .unwrap_err()
            .contains("rover-left (0x01) already has an entry"));
        assert!(match_entries(&[uid(L), uid(R)], &boards, &[(1, 0x09)])
            .unwrap_err()
            .contains("0x09 is not a motor controller"));
    }

    #[test]
    fn an_entry_with_no_candidate_is_skipped() {
        let boards = [board(0x01, "rover-left", Some(L))];
        let m = match_entries(&[uid(L), uid(R)], &boards, &[])
            .unwrap()
            .decide(|_| unreachable!())
            .unwrap();
        assert_eq!(m.pairs, [(0, 0x01)]);
        assert_eq!(m.skipped, [1]);
    }

    /// A `uid` two nodes report, or a blank one, is no usable uid (`crate::target::usable_uid`),
    /// so a board reporting it never matches an entry, not even one filed under that uid.
    #[test]
    fn a_duplicate_or_blank_uid_never_matches() {
        let zero = "000000000000000000000000";
        let boards = [
            board(0x01, "twin-a", None),
            board(0x02, "twin-b", None),
            board(0x03, "z", None),
        ];
        let e = match_entries(&[uid(L), uid(zero)], &boards, &[]).unwrap_err();
        assert!(e.starts_with("entries that match no board"), "{e}");
        assert!(e.contains("twin-a (0x01), twin-b (0x02), z (0x03)"), "{e}");
    }

    #[test]
    fn a_map_names_an_entry_by_its_uid_or_a_prefix() {
        let entries = [uid(L), uid(R)];
        assert_eq!(map_entry(&entries, L), Ok(0));
        assert_eq!(map_entry(&entries, "84FC"), Ok(1));
        assert!(map_entry(&entries, "8").is_err());
        assert!(map_entry(&entries, "cafe").is_err());
    }

    fn resp(status: u8, ty: Type, bytes: &[u8]) -> ConfigResp {
        ConfigResp {
            status,
            tag: ty.tag(),
            bytes: bytes.to_vec(),
        }
    }

    #[test]
    fn the_copies_that_differ_and_one_line_for_a_value_they_share() {
        let mode = store::CONTROL_MODE.key();
        let frame = store::CONTROL_FRAME.key();
        let left: Reads = [
            (mode, resp(CFG_OK, Type::U8, &[1])),
            (frame, resp(CFG_DEFAULT, Type::U8, &[0])),
        ]
        .into_iter()
        .collect();
        let right: Reads = [
            (mode, resp(CFG_DEFAULT, Type::U8, &[0])),
            (frame, resp(CFG_OK, Type::U8, &[0])),
        ]
        .into_iter()
        .collect();
        let copies = [
            ("rover-left".to_string(), left),
            ("rover-right".to_string(), right),
        ];
        // A stored value equal to the other's default does not differ: the digest is over the
        // effective values.
        assert_eq!(
            differing(&copies),
            ["machine.mode: rover-left balance, rover-right throttle (default)"]
        );
        let at = |k: Key| -> Vec<(String, ConfigResp)> {
            copies
                .iter()
                .map(|(n, r)| (n.clone(), r[&k].clone()))
                .collect()
        };
        assert_eq!(
            get_lines(mode, &at(mode)),
            [
                "rover-left: machine.mode = balance",
                "rover-right: machine.mode = throttle (default)"
            ]
        );
        assert_eq!(get_lines(frame, &at(frame)), ["machine.frame = pivoting"]);
        assert!(machine_keys()
            .iter()
            .all(|k| row_of(*k).unwrap().path.starts_with("machine.")));
        assert_eq!(
            machine_key("machine.turn.1"),
            Ok(store::CONTROL_DRIVE_TURN.at(1).key())
        );
        assert!(machine_key("leds.green")
            .unwrap_err()
            .contains("not a machine field"));
        assert!(machine_key("motors.0.side").is_err());
        // The tunes are a node field (`specs/tones.md` section 4): the board commands take them.
        assert!(machine_key("tone.horn")
            .unwrap_err()
            .contains("not a machine field"));
        assert!(machine_key("machine.tone.horn").is_err());
    }

    #[test]
    fn a_machine_dump_files_each_board_under_its_uid() {
        let mode = store::CONTROL_MODE.key();
        let name = store::DEVICE_NAME.key();
        let reads = |m: u8, n: &[u8]| -> Reads {
            [
                (mode, resp(CFG_OK, Type::U8, &[m])),
                (name, resp(CFG_OK, Type::Str, n)),
            ]
            .into_iter()
            .collect()
        };
        let boards = [
            (uid(L), reads(1, b"rover-left"), false),
            (uid(R), reads(0, b"rover-right"), false),
        ];
        assert_eq!(
            dump_file(&boards, 1),
            json!({
                "machine": { "mode": "throttle" },
                "boards": {
                    L: { "device": { "name": "rover-left" } },
                    R: { "device": { "name": "rover-right" } },
                },
            })
        );
    }

    #[test]
    fn the_digest_line() {
        let h = |v: &[(&str, Option<u16>)]| -> Vec<(String, Option<u16>)> {
            v.iter().map(|(n, d)| (n.to_string(), *d)).collect()
        };
        assert_eq!(
            digest_line(&h(&[
                ("rover-left", Some(0x1a2b)),
                ("rover-right", Some(0x1a2b))
            ])),
            "machine digests agree: 0x1a2b on rover-left, rover-right"
        );
        assert_eq!(
            digest_line(&h(&[
                ("rover-left", Some(0x1a2b)),
                ("rover-right", Some(0x3c4d))
            ])),
            "machine digests differ: rover-left 0x1a2b, rover-right 0x3c4d (\"machine diff\")"
        );
        assert_eq!(
            digest_line(&h(&[("rover-left", Some(0x1a2b)), ("rover-right", None)])),
            "machine digests agree: 0x1a2b on rover-left; not heard: rover-right"
        );
        assert_eq!(
            digest_line(&h(&[("rover-left", None), ("rover-right", None)])),
            "machine digests: nothing heard from rover-left, rover-right"
        );
    }

    /// A frame with `fault` and `flags`, every other word zero.
    fn frame(fault: u16, flags: u8) -> CyclicState {
        CyclicState {
            pitch: 0,
            roll: 0,
            wheel_speed: 0,
            battery: 0,
            mode: 0,
            fault,
            flags,
            peer_demand: 0,
            wheel_pos: 0,
            obs: linkctl::CyclicObs {
                phase_peak: 0,
                phase_mean: 0,
                duty_on: 0,
                boot_tag: 0,
                chip: linkctl::ChipTag::Unknown,
                odometer: 0,
            },
            wheel_rate: 0,
            machine: 0,
        }
    }

    #[test]
    fn the_marks_come_from_the_heard_frame_and_the_boot_report() {
        assert!(marks(Some(CLASS_MOTOR), Some(&frame(0, 0)), 0).is_empty());
        assert_eq!(marks(Some(CLASS_MOTOR), None, 0), ["silent"]);
        let s = frame(
            CyclicState::FAULT_MACHINE,
            CyclicState::FLAG_REARM | CyclicState::FLAG_REBOOT,
        );
        assert_eq!(
            marks(Some(CLASS_MOTOR), Some(&s), 2),
            [
                "machine differs (\"machine diff\")",
                "written, runs at the next arm",
                "written, runs at the next boot",
                "2 status entries (\"board status\")",
            ]
        );
        // The cause comes from the flag, not from the digests.
        let unheard = frame(CyclicState::FAULT_MACHINE, CyclicState::FLAG_PEER_SILENT);
        assert_eq!(
            marks(Some(CLASS_MOTOR), Some(&unheard), 0),
            ["a link peer not heard"]
        );
        assert!(marks(Some(net::walk::CLASS_BRIDGE), None, 0).is_empty());
    }
}
