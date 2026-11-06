//! The shared check cases (`shared/config-checks/`, `specs/config-read-once.md` section 4), run
//! here and by `protocol-kotlin`'s tests. One file per check, named by [`Check::id`], each a list
//! of cases: a `name`, the `chip` (a `hal::pincap::Chip` name), a `board` file fragment, and
//! `refused`, the path of the key the first refusal names, or `null` for a board every check
//! passes. A refused case is refused by its file's check alone.
//!
//! The leader check's cases (`leader.json`) are machine cases: a `machine` section and a list of
//! `boards` (board file fragments, no chip), each board being the machine section plus its
//! fragment, run through [`check_leader`] alone.

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

/// A board file fragment's stored values, as encoding it leaves them.
fn state_of(board: &Json) -> State {
    let files = read_files(&[("case".into(), board.to_string())]);
    assert!(files.problems.is_empty(), "{:?}", files.problems);
    let mut state = State::new();
    for e in &files.entries {
        let row = row_of(e.key).expect("a named key");
        let (bytes, _) = encode(e.key, row.storage, &e.value)
            .unwrap_or_else(|p| panic!("{}: {}", path_of(e.key), p.msg));
        if let Some(b) = bytes {
            state.insert(e.key, (row.storage, b));
        }
    }
    state
}

/// A machine case's refusals: every board is the case's machine section plus its own fragment,
/// addressed 0x01 onward.
fn machine_refusals(case: &Json) -> Vec<Refusal> {
    let states: Vec<State> = case["boards"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| {
            let mut board = b.clone();
            board["machine"] = case["machine"].clone();
            state_of(&board)
        })
        .collect();
    let boards: Vec<(u8, &State)> = states
        .iter()
        .enumerate()
        .map(|(i, s)| (i as u8 + 1, s))
        .collect();
    check_leader(&boards).into_iter().collect()
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
            let got = if check == Check::Leader {
                machine_refusals(c)
            } else {
                let chip = Chip::from_name(c["chip"].as_str().unwrap()).unwrap();
                refusals(chip, &c["board"])
            };
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

#[test]
fn the_leader_refusal_names_the_boards() {
    let rigid_balance = |leader: bool| {
        let mut b = serde_json::json!({ "machine": { "mode": "balance", "frame": "rigid" } });
        if leader {
            b["control"] = serde_json::json!({ "leader": true });
        }
        state_of(&b)
    };
    let (set, clear) = (rigid_balance(true), rigid_balance(false));
    let msg = |boards: &[(u8, &State)]| check_leader(boards).map(|r| r.msg);
    assert_eq!(msg(&[(0x01, &set), (0x03, &clear)]), None);
    assert_eq!(
        msg(&[(0x01, &clear), (0x03, &clear)]).unwrap(),
        "control.leader: a balance machine in the rigid frame needs exactly one board set as \
         leader; none is set"
    );
    assert_eq!(
        msg(&[(0x01, &set), (0x03, &set)]).unwrap(),
        "control.leader: a balance machine in the rigid frame needs exactly one board set as \
         leader; 0x01 and 0x03 are both set"
    );
    // One board left at rigid balance is enough to need a leader: a write that leaves the boards
    // differing is judged by the board that would run it.
    let throttle = state_of(&serde_json::json!({ "machine": { "frame": "rigid" } }));
    assert!(check_leader(&[(0x01, &clear), (0x03, &throttle)]).is_some());
    assert!(check_leader(&[(0x01, &throttle), (0x03, &throttle)]).is_none());
}

#[test]
fn only_the_mode_the_frame_and_the_leader_run_the_leader_check() {
    for key in LEADER_KEYS {
        assert!(changes_leader(key), "{}", path_of(key));
    }
    for key in [
        store::DEVICE_NAME.key(),
        store::IMU_MODEL.key(),
        store::CONTROL_SIDE.at(0).key(),
        store::CONTROL_RIDER_REQUIRED.key(),
    ] {
        assert!(!changes_leader(key), "{}", path_of(key));
    }
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
            // `specs/no-fleet-pins.md` section 3: the latch and the LEDs written out on every
            // board, the buzzer only on the masters (the slaves carry none).
            let stored: Vec<_> = files.entries.iter().map(|e| e.key).collect();
            for f in [
                store::BOARD_SELF_HOLD,
                store::LED_GREEN,
                store::LED_ORANGE,
                store::LED_RED,
            ] {
                assert!(
                    stored.contains(&f.key()),
                    "{name} {uid}: {}",
                    path_of(f.key())
                );
            }
            let slave = ["51ce39393333370134313739", "84fc41333235390839393638"];
            assert_eq!(
                stored.contains(&store::BOARD_BUZZER.key()),
                !slave.contains(&uid.as_str()),
                "{name} {uid}: buzzer"
            );
            let wrong = file_refusals(other, &files);
            assert!(
                wrong.iter().any(|r| r.check == Check::AfForm),
                "{name} {uid} on the {}: {wrong:?}",
                other.name()
            );
        }
        // The machine as the file leaves it passes the leader check (the rover's master is
        // flagged; the offroad pair is pivoting).
        let states: Vec<State> = (0..file.entries.len())
            .map(|i| file_state(&file.board_files(Some(i))))
            .collect();
        let boards: Vec<(u8, &State)> = states
            .iter()
            .enumerate()
            .map(|(i, s)| (i as u8 + 1, s))
            .collect();
        assert!(check_leader(&boards).is_none(), "{name}");
    }
}

/// The stored values `files` leave on a board that stores nothing else.
fn file_state(files: &crate::config::Files) -> State {
    let mut state = State::new();
    for e in &files.entries {
        let row = row_of(e.key).expect("a named key");
        if let Ok((Some(b), _)) = encode(e.key, row.storage, &e.value) {
            state.insert(e.key, (row.storage, b));
        }
    }
    state
}

/// `specs/twelve-fet.md` sections 9 and 12.1: the 12-FET's machine file passes every check on the
/// GD32F103RC, its two motors' pairs (B and C on motor 0, A and B on motor 1) and timers included.
/// Its one board is filed under a blank uid, which matches no board, so `machine apply` offers the
/// entry to the one motor controller on the bridge (or takes `--map`).
#[test]
fn the_twelve_fet_machine_file_passes_on_the_gd32f103rc() {
    let text = std::fs::read_to_string(dir().join("machines/EFeru.json")).unwrap();
    let file = crate::machine::read_machine_file("EFeru", &text).unwrap();
    assert_eq!(file.entries.len(), 1);
    let files = file.board_files(Some(0));
    assert!(files.problems.is_empty(), "{:?}", files.problems);
    let got = file_refusals(Chip::Gd32f103rc, &files);
    assert!(got.is_empty(), "{got:?}");
    let state = file_state(&files);
    for (m, phases) in [(0, [false, true, true]), (1, [true, true, false])] {
        for ((f, _), want) in SENSE_PHASES.iter().zip(phases) {
            assert_eq!(state.contains_key(&f.at(m).key()), want, "motor {m}");
        }
        // `specs/optional-protections.md` section 5.3: both protection inputs stored, the break
        // input off until its driver is known, the DC-link limit off until its scale is measured.
        for f in [store::MOTOR_DC_LINK_PIN.at(m), store::MOTOR_BREAK_PIN.at(m)] {
            assert!(
                state.contains_key(&f.key()),
                "motor {m}: {}",
                path_of(f.key())
            );
        }
        let value = |k: Key| state.get(&k).and_then(|(t, b)| int_of(*t, b));
        assert_eq!(value(store::MOTOR_BREAK_ENABLE.at(m).key()), Some(0));
        assert_eq!(value(store::MOTOR_DC_LINK_LIMIT.at(m).key()), Some(0));
        assert_eq!(value(store::MOTOR_RAIL_SKIP.at(m).key()), Some(3_848));
    }
    let peak = |m: u8| {
        state
            .get(&store::MOTOR_PEAK_TRIP_PERIODS.at(m).key())
            .cloned()
    };
    assert_eq!(peak(0), None, "motor 0 keeps the default count");
    assert_eq!(
        peak(1),
        Some((Type::U8, vec![2])),
        "motor 1 trips on two readings in a row"
    );
    assert!(check_leader(&[(0x01, &state)]).is_none());
}

/// `specs/config-read-once.md` section 4, `limit_floor`: the refusal names the limit's counts, the
/// floor's and the lowest limit that passes, by the firmware's arithmetic (`board::limit`).
#[test]
fn the_limit_floor_refusal_names_the_counts_and_the_lowest_passing_limit() {
    // The 12-FET's first drives: 3,000 mA at the defaults is 1,365 counts under 2,100.
    assert_eq!(
        limit_floor(3_000, 455, 2_100).unwrap(),
        "3000 mA is 1365 counts at a current.cal of 455, at or below the noise floor of 2100 \
         counts, so FOC's full scale is 0; the lowest limit that passes is 4700 mA"
    );
    assert_eq!(limit_floor(4_700, 455, 2_100), None);
    // The calibration is clamped into the seam's band first, as `limit_counts` clamps it: a
    // stored 50 converts at 100 counts per amp.
    assert_eq!(
        limit_floor(10_000, 50, 2_100).unwrap(),
        "10000 mA is 1000 counts at a current.cal of 100, at or below the noise floor of 2100 \
         counts, so FOC's full scale is 0; the lowest limit that passes is 21100 mA"
    );
    // `floor * 1000 / cal + 1` rounded up to 10,100 mA converts to 1,020 counts at 101 counts per
    // amp, the floor itself, so the next 100 mA is the lowest that passes.
    assert!(limit_floor(10_100, 101, 1_020).is_some());
    assert!(limit_floor(10_200, 101, 1_020).is_none());
    assert!(limit_floor(1_000, 101, 1_020)
        .unwrap()
        .ends_with("the lowest limit that passes is 10200 mA"));
    // A floor at the window's top leaves no limit above it.
    assert!(limit_floor(163_840, 455, 16_383)
        .unwrap()
        .ends_with("no limit passes a floor at the top of the limit window"));
}

/// The firmware repo's root.
fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A board file's entries with its tokens resolved on `chip`, on a board that stores nothing.
fn resolved(chip: Chip, text: &str) -> Result<Vec<Entry>, String> {
    let mut files = read_files(&[("file".into(), text.to_string())]);
    assert!(files.problems.is_empty(), "{:?}", files.problems);
    resolve(chip, &mut files.entries, &State::new())?;
    Ok(files.entries)
}

/// Entries by key, for a comparison that does not depend on their order.
fn by_key(entries: &[Entry]) -> BTreeMap<Key, Json> {
    entries.iter().map(|e| (e.key, e.value.clone())).collect()
}

/// `specs/cli-onboarding.md` test C2, and on the host what bench gate G3 checks on the boards:
/// every pin of the split 2.x.20 token files resolves, on each chip, to the object the per-chip
/// files of 2026-11-06 held (`af` 1 for the link and IMU pins and 2 for the gates on the
/// GD32F130, none on the GD32F103; `adc` 0, 8 and 4), and every other field is the same.
#[test]
fn the_split_token_files_resolve_to_the_per_chip_files() {
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/split-per-chip");
    for role in ["master", "slave"] {
        let text = std::fs::read_to_string(repo().join(format!("boards/split/2.x.20/{role}.json")))
            .unwrap();
        for (chip, old) in [(Chip::Gd32f130c8, "2.1.20"), (Chip::Gd32f103c8, "2.2.20")] {
            let got = resolved(chip, &text).unwrap_or_else(|e| panic!("{role} {old}: {e}"));
            let old_text =
                std::fs::read_to_string(fixtures.join(format!("{old}-{role}.json"))).unwrap();
            let want = read_files(&[("old".into(), old_text)]);
            assert!(want.problems.is_empty(), "{:?}", want.problems);
            assert_eq!(
                by_key(&got),
                by_key(&want.entries),
                "{role} on {}",
                chip.name()
            );
        }
    }
}

/// `specs/cli-onboarding.md` test C5: every board file under `boards/` passes every check on
/// every chip its kind lists, its tokens resolved there.
#[test]
fn every_board_file_passes_on_every_chip_of_its_kind() {
    let kinds: &[(&str, &[Chip])] = &[("split", &[Chip::Gd32f103c8, Chip::Gd32f130c8])];
    let mut files = Vec::new();
    let mut dirs = vec![repo().join("boards")];
    while let Some(d) = dirs.pop() {
        for e in std::fs::read_dir(&d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                dirs.push(p);
            } else if p.extension().is_some_and(|x| x == "json") {
                files.push(p);
            }
        }
    }
    assert!(!files.is_empty());
    for p in files {
        let rel = p.strip_prefix(repo().join("boards")).unwrap().to_owned();
        let kind = rel.iter().next().unwrap().to_str().unwrap();
        let (_, chips) = kinds
            .iter()
            .find(|(k, _)| *k == kind)
            .unwrap_or_else(|| panic!("{}: a kind the test lists no chips for", rel.display()));
        let text = std::fs::read_to_string(&p).unwrap();
        for chip in *chips {
            let entries = resolved(*chip, &text)
                .unwrap_or_else(|e| panic!("{} on {}: {e}", rel.display(), chip.name()));
            let files = crate::config::Files {
                entries,
                problems: Vec::new(),
            };
            let got = file_refusals(*chip, &files);
            assert!(
                got.is_empty(),
                "{} on {}: {got:?}",
                rel.display(),
                chip.name()
            );
        }
    }
}

/// `specs/cli-onboarding.md` test C3: a token is refused when its group's instance is in neither
/// the file nor the store, when the pin has no route for the function or is no ADC input, when it
/// has two routes, and on the GD32F10x when the group's pins need different remaps.
#[test]
fn a_token_that_does_not_resolve_is_refused() {
    let refused = |chip: Chip, board: Json| resolved(chip, &board.to_string()).unwrap_err();
    // No instance.
    assert_eq!(
        refused(
            Chip::Gd32f130c8,
            serde_json::json!({"links": [{"tx": "PA2", "rx": "PA3"}]})
        ),
        "links[0].tx: give links[0].usart to resolve PA2\nerror: links[0].rx: give links[0].usart \
         to resolve PA3"
    );
    // The instance stored on the board resolves it.
    let mut stored = State::new();
    stored.insert(store::LINK_USART.at(0).key(), (Type::U8, vec![1]));
    let mut entries = read_files(&[("f".into(), r#"{"links": [{"tx": "PA2"}]}"#.into())]).entries;
    resolve(Chip::Gd32f130c8, &mut entries, &stored).unwrap();
    assert_eq!(
        entries[0].value,
        serde_json::json!({"port": "A", "pin": 2, "af": 1})
    );
    // No route, no ADC input.
    assert_eq!(
        refused(
            Chip::Gd32f130c8,
            serde_json::json!({"links": [{"tx": "PB12", "rx": "PA3", "usart": 1}]})
        ),
        "links[0].tx: PB12 cannot carry Tx(1) on the gd32f130c8"
    );
    assert_eq!(
        refused(
            Chip::Gd32f103c8,
            serde_json::json!({"battery_sense": {"pin": "PB12"}})
        ),
        "battery_sense.pin: PB12 is not an ADC input on the gd32f103c8"
    );
    // Two routes, and a group whose pins need different remaps: no fleet table has either, so a
    // table that does.
    static TWO: [hal::pincap::Route; 4] = [
        hal::pincap::Route {
            port: 0,
            pin: 2,
            func: Func::Tx(1),
            sel: 0,
        },
        hal::pincap::Route {
            port: 0,
            pin: 2,
            func: Func::Tx(1),
            sel: 1,
        },
        hal::pincap::Route {
            port: 0,
            pin: 3,
            func: Func::Rx(1),
            sel: 1,
        },
        hal::pincap::Route {
            port: 1,
            pin: 2,
            func: Func::Tx(1),
            sel: 0,
        },
    ];
    let base = Chip::Gd32f103c8.table();
    let t = Table {
        bonding: base.bonding,
        routes: &TWO,
        adc: base.adc,
        sense_pairs: base.sense_pairs,
        inputs: base.inputs,
    };
    let on = |board: Json| {
        let mut e = read_files(&[("f".into(), board.to_string())]).entries;
        resolve_on(&t, Chip::Gd32f103c8, &mut e, &State::new()).map(|_| e)
    };
    assert_eq!(
        on(serde_json::json!({"links": [{"tx": "PA2", "rx": "PA3", "usart": 1}]})).unwrap_err(),
        "links[0].tx: PA2 has remap 0 and remap 1 routes: give the pin as an object"
    );
    assert_eq!(
        on(serde_json::json!({"links": [{"tx": "PB2", "rx": "PA3", "usart": 1}]})).unwrap_err(),
        "links[0].remap: the group's pins need different remaps: links[0].tx PB2 at remap 0, \
         links[0].rx PA3 at remap 1"
    );
    // A remap the file gives is kept, and checked by the checks as now.
    let e = on(serde_json::json!({"links": [{"tx": "PB2", "rx": "PA3", "usart": 1, "remap": 1}]}))
        .unwrap();
    assert_eq!(e.len(), 4);
}

/// `specs/cli-onboarding.md` part C, the GD32F10x remap: a group of tokens all routed at remap 1
/// stores that remap; at remap 0, nothing is added on a board that stores no remap.
#[test]
fn a_group_of_tokens_gets_its_remap_on_the_gd32f10x() {
    let e = resolved(
        Chip::Gd32f103c8,
        &serde_json::json!({"links": [{"tx": "PB6", "rx": "PB7", "usart": 0}]}).to_string(),
    )
    .unwrap();
    let got = by_key(&e);
    assert_eq!(got[&store::LINK_REMAP.at(0).key()], Json::from(1));
    assert_eq!(
        got[&store::LINK_TX.at(0).key()],
        serde_json::json!({"port": "B", "pin": 6})
    );
    let e = resolved(
        Chip::Gd32f103c8,
        &serde_json::json!({"links": [{"tx": "PA2", "rx": "PA3", "usart": 1}]}).to_string(),
    )
    .unwrap();
    assert_eq!(e.len(), 3, "no remap added at 0");
    // A board storing remap 1 gets 0 written back.
    let mut stored = State::new();
    stored.insert(store::LINK_REMAP.at(0).key(), (Type::U8, vec![1]));
    let mut entries = read_files(&[(
        "f".into(),
        r#"{"links": [{"tx": "PA2", "rx": "PA3", "usart": 1}]}"#.into(),
    )])
    .entries;
    resolve(Chip::Gd32f103c8, &mut entries, &stored).unwrap();
    assert_eq!(
        by_key(&entries)[&store::LINK_REMAP.at(0).key()],
        Json::from(0)
    );
    // One token beside a stored pin of its group: no remap inferred.
    let mut stored = State::new();
    stored.insert(store::LINK_RX.at(0).key(), (Type::PinAf, vec![1, 7, 0]));
    let mut entries = read_files(&[(
        "f".into(),
        r#"{"links": [{"tx": "PB6", "usart": 0}]}"#.into(),
    )])
    .entries;
    resolve(Chip::Gd32f103c8, &mut entries, &stored).unwrap();
    assert_eq!(entries.len(), 2);
}
