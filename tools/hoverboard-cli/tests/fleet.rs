//! `machine update`'s plan and order (`specs/firmware-bundle.md` sections 4.3 and 6) against a
//! simulated machine of two pairs: each pair over its own bridge's link, the given link's pair
//! last, a reconnect after each bridge's reboot, a bridge with no board, a board no bridge is wired
//! to, a stop at a bridge that does not return with its board left untouched, and the lines that
//! report done, failed and not-started pairs.

mod common;

use std::collections::{BTreeMap, BTreeSet};

use common::{names_of, release_guards, Fixture};
use espsvc::{EspInfo, TargetId, CHIP_ESP32C3, FAMILY_F10X, FAMILY_F1X0};
use hoverboard_cli::bundle::{image_table, pack, Bundle, PackEntry};
use hoverboard_cli::elf;
use hoverboard_cli::fleet::{self, Outcome, Pair, PlanBoard, Steps};
use hoverboard_cli::nodes::{Found, Topology, Walked, CLASS_MOTOR};
use net::walk::{Identity, CLASS_BRIDGE, MCU_ESP32C3, MCU_F10X, MCU_F1X0, UID_LEN};

const FIRMWARE_TOML: &str = include_str!("../../../motorcontroller/firmware/Cargo.toml");
const BRIDGE_TOML: &str = include_str!("../../../bridge/Cargo.toml");
const COMMIT: &str = "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd";

fn uid(hex: &str) -> [u8; UID_LEN] {
    let mut u = [0u8; UID_LEN];
    for (i, b) in u.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
    }
    u
}

const SLAVE: &str = "84fc41333235390839393638";
const MASTER: &str = "8adc39323335381255343333";

fn node(addr: u8, class: u8, name: &str, hex: &str) -> Found {
    let mcu = match class {
        CLASS_BRIDGE => MCU_ESP32C3,
        _ if hex == MASTER => MCU_F10X,
        _ => MCU_F1X0,
    };
    Found {
        addr,
        identity: Some(Identity {
            class,
            mcu,
            fw_ver: 1,
            uid: uid(hex),
        }),
        name: Some(name.into()),
    }
}

/// The rover as the walk over the master's bridge (`0x04`) finds it: the slave board `0x01` on
/// bridge `0x02`'s mailbox, the master board `0x03` on bridge `0x04`'s, the boards linked by UART.
fn rover() -> Walked {
    Walked {
        src: 0x88,
        found: vec![
            node(0x01, CLASS_MOTOR, "Rover slave", SLAVE),
            node(
                0x02,
                CLASS_BRIDGE,
                "rover-slave",
                "44bd8d245f64000000000000",
            ),
            node(0x03, CLASS_MOTOR, "Rover master", MASTER),
            node(
                0x04,
                CLASS_BRIDGE,
                "rover-master",
                "44bd8d25b278000000000000",
            ),
        ],
        topology: Topology {
            gateway: Some(0x04),
            neighbours: BTreeMap::from([
                (0x02, vec![0x01]),
                (0x01, vec![0x02, 0x03]),
                (0x03, vec![0x01, 0x04]),
                (0x04, vec![0x03]),
            ]),
            mailbox: BTreeMap::from([(0x02, 0), (0x04, 0)]),
        },
    }
}

/// An ESP32-C3 app image, as `espflash save-image` writes one.
fn app_image() -> Vec<u8> {
    let mut img = vec![0x33u8; 2048];
    img[0] = 0xE9;
    img[12..14].copy_from_slice(&5u16.to_le_bytes());
    img[0x20..0x24].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    img[0x30..0x50].fill(0);
    img[0x30..0x35].copy_from_slice(b"0.2.0");
    img
}

/// A bundle of both mainboard images and the bridge's app image.
fn bundle() -> Bundle {
    let f130 = common::flashable_elf(&[0x11, 0x22, 0x33, 0x44]);
    let g103 = elf::guards_for(common::GUARD_FILE, "mainboard-gd32f103c8").unwrap();
    let f103 = Fixture::new(&[0x66; 8], g103.code_floor, names_of(&g103)).build();
    let app = app_image();
    let z = pack(
        &[
            PackEntry {
                name: "mainboard-gd32f103c8",
                table: image_table(FIRMWARE_TOML, "mainboard-gd32f103c8").unwrap(),
                guards: Some(g103),
                file: &f103,
            },
            PackEntry {
                name: "mainboard-gd32f130c8",
                table: image_table(FIRMWARE_TOML, "mainboard-gd32f130c8").unwrap(),
                guards: Some(release_guards()),
                file: &f130,
            },
            PackEntry {
                name: "bridge-esp32c3",
                table: image_table(BRIDGE_TOML, "bridge-esp32c3").unwrap(),
                guards: None,
                file: &app,
            },
        ],
        COMMIT,
        "2026-11-01T17:03:58Z",
    )
    .unwrap();
    Bundle::open(&z).unwrap()
}

/// The rover's devices as the plan reads them: each bridge's `ESP_INFO`, each board's
/// `TARGET_ID` by the bridge it is read through.
struct Reads {
    info: BTreeMap<u8, Option<EspInfo>>,
    ids: BTreeMap<u8, TargetId>,
    /// The `(bridge, board)` pairs `TARGET_ID` was read through.
    through: Vec<(u8, u8)>,
}

impl Reads {
    fn rover() -> Self {
        let info = |commit| EspInfo {
            commit,
            chip: CHIP_ESP32C3,
            ..EspInfo::default()
        };
        let id = |family, hex| TargetId {
            family,
            uid: uid(hex),
            flash_kib: 64,
            rdp: 0xA5,
            ..TargetId::default()
        };
        Reads {
            info: BTreeMap::from([
                (0x02, Some(info(0x50a4_6c91))),
                (0x04, Some(info(0x50a4_6c91))),
            ]),
            ids: BTreeMap::from([
                (0x02, id(FAMILY_F1X0, SLAVE)),
                (0x04, id(FAMILY_F10X, MASTER)),
            ]),
            through: Vec::new(),
        }
    }

    fn plan(&mut self, w: &Walked, b: &Bundle) -> Result<fleet::Plan, String> {
        let info = self.info.clone();
        let ids = self.ids.clone();
        let through = &mut self.through;
        fleet::plan(
            w,
            b,
            |bridge| info.get(&bridge).copied().ok_or("no answer".into()),
            |bridge, board| {
                through.push((bridge, board));
                ids.get(&bridge).copied().ok_or("no answer".into())
            },
        )
    }
}

/// A link the fake opened: the bridge it reaches and the connection's number.
#[derive(Debug)]
struct Conn {
    bridge: String,
    given: bool,
    n: u32,
}

/// The machine's bridges and boards as [`Steps`] see them. A bridge reboots into its update,
/// which ends every connection to it; one in `gone` never comes back, one in `unreachable` has no
/// link, and a board in `bad_status` reports a boot-report entry.
#[derive(Default)]
struct Fake {
    log: Vec<String>,
    next: u32,
    dropped: BTreeSet<u32>,
    gone: BTreeSet<String>,
    unreachable: BTreeSet<String>,
    bad_status: BTreeSet<String>,
}

impl Fake {
    fn conn(&mut self, pair: &Pair) -> Conn {
        self.next += 1;
        Conn {
            bridge: pair.bridge.name.clone(),
            given: pair.given,
            n: self.next,
        }
    }

    fn via(c: &Conn) -> &'static str {
        if c.given {
            "the given link"
        } else {
            "its own link"
        }
    }
}

impl Steps for Fake {
    type Link = Conn;

    fn open(&mut self, pair: &Pair) -> Result<Conn, String> {
        if self.unreachable.contains(&pair.bridge.name) {
            self.log
                .push(format!("open {}: unreachable", pair.bridge.name));
            return Err("no answer on TCP or BLE".into());
        }
        let c = self.conn(pair);
        self.log
            .push(format!("open {} over {}", c.bridge, Fake::via(&c)));
        Ok(c)
    }

    fn bridge(&mut self, link: &Conn, pair: &Pair) -> Result<(), String> {
        assert_eq!(
            link.bridge, pair.bridge.name,
            "the bridge over its own link"
        );
        assert!(!self.dropped.contains(&link.n));
        self.log
            .push(format!("update {} to {}", link.bridge, pair.bridge.image));
        // The reboot into the new image ends the connection.
        self.dropped.insert(link.n);
        Ok(())
    }

    fn rejoin(&mut self, pair: &Pair) -> Result<Conn, String> {
        if self.gone.contains(&pair.bridge.name) {
            self.log
                .push(format!("rejoin {}: no answer", pair.bridge.name));
            return Err("it did not answer within 120 s of its update".into());
        }
        let c = self.conn(pair);
        self.log.push(format!("rejoin {}", c.bridge));
        Ok(c)
    }

    fn board(&mut self, link: &Conn, board: &PlanBoard) -> Result<(), String> {
        assert!(
            !self.dropped.contains(&link.n),
            "{}: flashed over a connection the reboot ended",
            board.name
        );
        self.log.push(format!(
            "flash {} with {} through {}",
            board.name, board.image, link.bridge
        ));
        Ok(())
    }

    fn status(&mut self, link: &Conn, board: &PlanBoard) -> Result<(), String> {
        self.log
            .push(format!("status {} through {}", board.name, link.bridge));
        if self.bad_status.contains(&board.name) {
            return Err(format!(
                "{}: imu: the IMU did not answer its probe",
                board.name
            ));
        }
        Ok(())
    }
}

#[test]
fn the_plan_pairs_each_board_with_its_bridge_and_takes_the_given_link_last() {
    let b = bundle();
    let mut reads = Reads::rover();
    let plan = reads.plan(&rover(), &b).unwrap();
    // Each TARGET_ID through the bridge wired to the board.
    assert_eq!(reads.through, vec![(0x02, 0x01), (0x04, 0x03)]);
    let order: Vec<(&str, Option<&str>, bool)> = plan
        .pairs
        .iter()
        .map(|p| {
            (
                p.bridge.name.as_str(),
                p.board.as_ref().map(|d| d.image.as_str()),
                p.given,
            )
        })
        .collect();
    assert_eq!(
        order,
        vec![
            ("rover-slave", Some("mainboard-gd32f130c8"), false),
            ("rover-master", Some("mainboard-gd32f103c8"), true),
        ]
    );
    assert_eq!(plan.bins.len(), 2, "each board image prepared once");
    assert_eq!(
        fleet::plan_lines(&plan),
        vec![
            format!("bundle built from commit {COMMIT}"),
            "bridge rover-slave (0x02): 50a46c91 -> c0eb2591 (bridge-esp32c3), over its own link"
                .to_string(),
            "board  Rover slave (0x01): gd32f1x0 64 KiB, gets mainboard-gd32f130c8, flashed by \
             rover-slave"
                .to_string(),
            "bridge rover-master (0x04): 50a46c91 -> c0eb2591 (bridge-esp32c3), over the given \
             link"
                .to_string(),
            "board  Rover master (0x03): gd32f10x 64 KiB, gets mainboard-gd32f103c8, flashed by \
             rover-master"
                .to_string(),
            "order: rover-slave and Rover slave, then rover-master and Rover master".to_string(),
        ]
    );
}

#[test]
fn each_pair_goes_over_its_own_link_and_the_board_after_the_bridge_rejoins() {
    let b = bundle();
    let plan = Reads::rover().plan(&rover(), &b).unwrap();
    let mut fake = Fake::default();
    let out = fleet::run(&plan, &mut fake);
    assert_eq!(out, vec![Outcome::Done, Outcome::Done]);
    assert!(fleet::all_done(&out));
    assert_eq!(
        fake.log,
        vec![
            "open rover-slave over its own link",
            "update rover-slave to bridge-esp32c3",
            "rejoin rover-slave",
            "flash Rover slave with mainboard-gd32f130c8 through rover-slave",
            "status Rover slave through rover-slave",
            "open rover-master over the given link",
            "update rover-master to bridge-esp32c3",
            "rejoin rover-master",
            "flash Rover master with mainboard-gd32f103c8 through rover-master",
            "status Rover master through rover-master",
        ]
    );
    assert_eq!(
        fleet::outcome_lines(&plan, &out),
        vec![
            "rover-slave and Rover slave: done",
            "rover-master and Rover master: done",
        ]
    );
}

#[test]
fn a_bridge_that_does_not_return_stops_the_run_with_its_board_untouched() {
    let b = bundle();
    let plan = Reads::rover().plan(&rover(), &b).unwrap();
    let mut fake = Fake {
        gone: BTreeSet::from(["rover-slave".to_string()]),
        ..Fake::default()
    };
    let out = fleet::run(&plan, &mut fake);
    assert!(matches!(out[0], Outcome::BridgeFailed(_)), "{out:?}");
    assert_eq!(out[1], Outcome::NotStarted);
    assert!(!fleet::all_done(&out));
    assert_eq!(
        fake.log,
        vec![
            "open rover-slave over its own link",
            "update rover-slave to bridge-esp32c3",
            "rejoin rover-slave: no answer",
        ],
        "nothing after the bridge that did not return"
    );
    let lines = fleet::outcome_lines(&plan, &out);
    assert_eq!(
        lines[0],
        "rover-slave and Rover slave: FAILED updating rover-slave (it did not answer within 120 \
         s of its update); until it is confirmed it returns to its previous slot at a reset; \
         Rover slave untouched"
    );
    assert_eq!(lines[1], "rover-master and Rover master: not started");
}

#[test]
fn an_unreachable_bridge_is_reported_and_its_pair_not_started() {
    let b = bundle();
    let plan = Reads::rover().plan(&rover(), &b).unwrap();
    let mut fake = Fake {
        unreachable: BTreeSet::from(["rover-slave".to_string()]),
        ..Fake::default()
    };
    let out = fleet::run(&plan, &mut fake);
    assert_eq!(
        out,
        vec![
            Outcome::Unreachable("no answer on TCP or BLE".into()),
            Outcome::NotStarted
        ]
    );
    assert_eq!(fake.log, vec!["open rover-slave: unreachable"]);
    assert_eq!(
        fleet::outcome_lines(&plan, &out)[0],
        "rover-slave and Rover slave: not started: rover-slave is not reachable (no answer on \
         TCP or BLE)"
    );
}

#[test]
fn a_board_whose_report_is_not_ok_stops_the_run_after_its_pair() {
    let b = bundle();
    let plan = Reads::rover().plan(&rover(), &b).unwrap();
    let mut fake = Fake {
        bad_status: BTreeSet::from(["Rover slave".to_string()]),
        ..Fake::default()
    };
    let out = fleet::run(&plan, &mut fake);
    assert!(matches!(out[0], Outcome::StatusFailed(_)), "{out:?}");
    assert_eq!(out[1], Outcome::NotStarted);
    assert_eq!(
        fleet::outcome_lines(&plan, &out)[0],
        "rover-slave and Rover slave: rover-slave done, Rover slave booted; its boot report is \
         not ok: Rover slave: imu: the IMU did not answer its probe"
    );
}

#[test]
fn a_bridge_with_no_board_is_updated_alone_and_a_board_with_no_bridge_is_listed() {
    let b = bundle();
    // The slave's bridge is gone from the machine: its board has no bridge on its SWD header. A
    // spare bridge with nothing on its mailbox is the node the given link reaches.
    let mut w = rover();
    w.found.retain(|f| f.addr != 0x02);
    w.found.push(node(
        0x05,
        CLASS_BRIDGE,
        "spare",
        "aabbccddeeff000000000005",
    ));
    w.topology.neighbours.remove(&0x02);
    w.topology.neighbours.insert(0x01, vec![0x03]);
    w.topology.neighbours.insert(0x05, vec![]);
    w.topology.mailbox.remove(&0x02);
    w.topology.mailbox.insert(0x05, 0);
    w.topology.gateway = Some(0x05);
    let mut reads = Reads::rover();
    reads.info.insert(0x05, None);
    let plan = reads.plan(&w, &b).unwrap();
    assert_eq!(
        reads.through,
        vec![(0x04, 0x03)],
        "no TARGET_ID for the slave"
    );
    let lines = fleet::plan_lines(&plan);
    assert!(
        lines.contains(&"bridge spare (0x05): commit unknown -> c0eb2591 (bridge-esp32c3), over the given link".to_string()),
        "{lines:#?}"
    );
    assert!(
        lines.contains(
            &"        no board on spare's mailbox: the bridge is updated alone".to_string()
        ),
        "{lines:#?}"
    );
    assert!(
        lines.contains(
            &"board  Rover slave: not updatable: no bridge is wired to its SWD header".to_string()
        ),
        "{lines:#?}"
    );
    let mut fake = Fake::default();
    let out = fleet::run(&plan, &mut fake);
    assert_eq!(out, vec![Outcome::Done, Outcome::Done]);
    assert_eq!(
        fake.log,
        vec![
            "open rover-master over its own link",
            "update rover-master to bridge-esp32c3",
            "rejoin rover-master",
            "flash Rover master with mainboard-gd32f103c8 through rover-master",
            "status Rover master through rover-master",
            "open spare over the given link",
            "update spare to bridge-esp32c3",
            "rejoin spare",
        ]
    );
    assert_eq!(fleet::outcome_lines(&plan, &out)[1], "spare: done");
}

#[test]
fn nothing_is_planned_unless_every_image_is_chosen() {
    let b = bundle();
    // A board whose chip no image is for.
    let mut reads = Reads::rover();
    reads.ids.get_mut(&0x02).unwrap().flash_kib = 128;
    let e = reads.plan(&rover(), &b).unwrap_err();
    assert!(
        e.starts_with(
            "Rover slave: REFUSED: no board image in the bundle is for the chip (gd32f1x0 128 KiB)"
        ),
        "{e}"
    );
    // A match the bridge cannot flash.
    let mut reads = Reads::rover();
    reads.ids.get_mut(&0x02).unwrap().flash_kib = 32;
    let e = reads.plan(&rover(), &b).unwrap_err();
    assert!(e.contains("no board image"), "{e}");
    // A TARGET_ID that reports another uid than the walk's.
    let mut reads = Reads::rover();
    reads.ids.get_mut(&0x02).unwrap().uid = uid(MASTER);
    let e = reads.plan(&rover(), &b).unwrap_err();
    assert!(
        e.contains(&format!(
            "reads the uid {MASTER} over SWD, not the walk's {SLAVE}"
        )),
        "{e}"
    );
    // A TARGET_ID that does not answer.
    let mut reads = Reads::rover();
    reads.ids.remove(&0x04);
    let e = reads.plan(&rover(), &b).unwrap_err();
    assert_eq!(e, "Rover master: TARGET_ID: no answer");
    // A bridge chip no image is for.
    let mut reads = Reads::rover();
    reads.info.insert(
        0x02,
        Some(EspInfo {
            chip: 7,
            ..EspInfo::default()
        }),
    );
    let e = reads.plan(&rover(), &b).unwrap_err();
    assert!(
        e.contains("rover-slave: REFUSED: ESP_INFO reports chip 7"),
        "{e}"
    );
}
