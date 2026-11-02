//! The shared check cases (`shared/config-checks/`, `specs/config-read-once.md` section 4), run
//! here and by `protocol-kotlin`'s tests. One file per check, named by [`Check::id`], each a list
//! of cases: a `name`, the `chip` (a `hal::pincap::Chip` name), a `board` file fragment, and
//! `refused`, the path of the key the first refusal names, or `null` for a board every check
//! passes. A refused case is refused by its file's check alone.

use std::path::PathBuf;

use serde_json::Value as Json;

use super::*;
use crate::config::read_files;
use crate::names::{encode, row_of};

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../shared/config-checks")
}

/// Every check a board file fragment fails on `chip`, in the order the checks run.
fn refusals(chip: Chip, board: &Json) -> Vec<Refusal> {
    let files = read_files(&[("case".into(), board.to_string())]);
    assert!(
        files.problems.iter().all(|p| p.force),
        "{:?}",
        files.problems
    );
    let mut state = State::new();
    for e in &files.entries {
        let row = row_of(e.key).expect("a named key");
        let (bytes, _) = encode(e.key, row.storage, &e.value)
            .unwrap_or_else(|p| panic!("{}: {}", path_of(e.key), p.msg));
        if let Some(b) = bytes {
            state.insert(e.key, (row.storage, b));
        }
    }
    let mut out = check_values(&state);
    out.extend(check_forms(chip, &files.entries));
    out.extend(check_pins(chip, &state));
    out
}

#[test]
fn every_shared_case_gets_its_verdict() {
    for check in Check::ALL {
        let path = dir().join(format!("{}.json", check.id()));
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let cases: Vec<Json> = serde_json::from_str(&text).unwrap();
        let (mut passes, mut refuses) = (0, 0);
        for c in &cases {
            let name = c["name"].as_str().unwrap();
            let chip = Chip::from_name(c["chip"].as_str().unwrap()).unwrap();
            let got = refusals(chip, &c["board"]);
            match c["refused"].as_str() {
                None => {
                    assert!(got.is_empty(), "{}: {name}: {got:?}", check.id());
                    passes += 1;
                }
                Some(field) => {
                    assert!(!got.is_empty(), "{}: {name}: nothing refused", check.id());
                    assert!(
                        got.iter().all(|r| r.check == check),
                        "{}: {name}: {got:?}",
                        check.id()
                    );
                    assert_eq!(path_of(got[0].key), field, "{}: {name}", check.id());
                    refuses += 1;
                }
            }
        }
        assert!(
            passes > 0 && refuses > 0,
            "{} needs a passing and a refused case",
            check.id()
        );
    }
}

#[test]
fn the_case_files_are_exactly_the_checks() {
    let mut files: Vec<String> = std::fs::read_dir(dir())
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .filter(|n| n.ends_with(".json"))
        .collect();
    files.sort();
    let mut want: Vec<String> = Check::ALL
        .iter()
        .map(|c| format!("{}.json", c.id()))
        .collect();
    want.sort();
    assert_eq!(files, want);
}

#[test]
fn chip_of_maps_target_id() {
    assert_eq!(chip_of(espsvc::FAMILY_F10X, 64), Some(Chip::Gd32f103c8));
    assert_eq!(chip_of(espsvc::FAMILY_F10X, 256), Some(Chip::Gd32f103rc));
    assert_eq!(chip_of(espsvc::FAMILY_F1X0, 64), Some(Chip::Gd32f130c8));
    assert_eq!(
        chip_of(espsvc::FAMILY_MM32SPIN05, 32),
        Some(Chip::Mm32spin05)
    );
    assert_eq!(chip_of(espsvc::FAMILY_F1X0, 32), None);
}

/// Every check on the board as `files` would leave a board that stores nothing else.
fn file_refusals(chip: Chip, files: &crate::config::Files) -> Vec<Refusal> {
    let mut state = State::new();
    for e in &files.entries {
        let row = row_of(e.key).expect("a named key");
        let (bytes, _) = encode(e.key, row.storage, &e.value)
            .unwrap_or_else(|p| panic!("{}: {}", path_of(e.key), p.msg));
        if let Some(b) = bytes {
            state.insert(e.key, (row.storage, b));
        }
    }
    let mut out = check_values(&state);
    out.extend(check_forms(chip, &files.entries));
    out.extend(check_pins(chip, &state));
    out
}

/// `specs/config-read-once.md` section 9: the rewritten rover and offroad machine files pass on
/// their boards' chips, and a board's file is refused on the other chip.
#[test]
fn the_fleets_machine_files_pass_on_their_chips_and_not_on_the_other() {
    let rover_master = "8adc39323335381255343333";
    for name in ["rover", "offroad"] {
        let path = dir().join(format!("machines/{name}.json"));
        let text = std::fs::read_to_string(&path).unwrap();
        let file = crate::machine::read_machine_file(name, &text).unwrap();
        assert!(!file.entries.is_empty());
        for (i, e) in file.entries.iter().enumerate() {
            let uid = crate::nodes::uid_hex(&e.uid);
            let (chip, other) = if uid == rover_master {
                (Chip::Gd32f103c8, Chip::Gd32f130c8)
            } else {
                (Chip::Gd32f130c8, Chip::Gd32f103c8)
            };
            let files = file.board_files(Some(i));
            assert!(
                files.problems.is_empty(),
                "{name} {uid}: {:?}",
                files.problems
            );
            let got = file_refusals(chip, &files);
            assert!(got.is_empty(), "{name} {uid}: {got:?}");
            let wrong = file_refusals(other, &files);
            assert!(
                wrong.iter().any(|r| r.check == Check::AfForm),
                "{name} {uid} on the {}: {wrong:?}",
                other.name()
            );
        }
    }
}
