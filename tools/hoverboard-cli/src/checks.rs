//! The checks a board's configuration passes before it is written (`specs/config-read-once.md`
//! section 4). The firmware applies what is stored and refuses nothing, so these are the only
//! guard between a configuration and the registers: every `board apply`, `board replace`,
//! `board set` and `machine apply` runs them on the board as it would be after the write, and
//! writes nothing if one refuses. `--force` overrides none of them.
//!
//! Two kinds. The pin checks need the chip, which the CLI reads from `TARGET_ID` through the
//! board's bridge, and test the pin, instance and remap keys against `hal::pincap`'s table of that
//! chip. The value checks need no chip: the arm row and the calibration against the store's range
//! table (`store::RANGES`), each motor's current limit against its noise floor by the firmware's
//! own conversion (`board::limit`), and the IMU frame against `imu::Config::frame_error`.
//!
//! One check looks at the machine rather than at one board: [`check_leader`], the rigid frame's
//! leader (`specs/rigid-leader.md` section 6.1), on every motor controller as the write would leave
//! it, run only by a write that changes `machine.mode`, `machine.frame` or `control.leader`.
//!
//! `protocol-kotlin` ports the same checks for the app. The two are held together by the shared
//! check cases (`shared/config-checks/`), which this module's tests and the Kotlin tests both run:
//! every [`Check`] has a passing and a refused case there.

use std::collections::{BTreeMap, HashMap};

use hal::pincap::{Chip, Func, Phase, Table, ADC2, SWD};
use serde_json::Value as Json;
use store::{Key, PinField, Type, Value};

use crate::config::Entry;
use crate::names::{path_of, pin_text};

/// One check of section 4, by the id the shared cases name it with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Check {
    /// The port and pin exist on the chip.
    PinExists,
    /// PA13 and PA14 (SWD) are never used.
    Swd,
    /// No pin is used twice.
    PinTwice,
    /// A group is complete or absent.
    Group,
    /// The instance key names a peripheral the chip has.
    Instance,
    /// No peripheral instance serves two groups.
    InstanceTwice,
    /// On a chip with a per-pin mux, the stored `af` selects the function the field needs.
    Af,
    /// On the GD32F10x, the group's remap routes the pin to the function the field needs.
    Remap,
    /// A `PinAf` carries `af` in the file on a mux chip and leaves it out on the GD32F10x.
    AfForm,
    /// A remap key is stored only on a chip with AFIO.
    RemapChip,
    /// A `PinAdc` stores the ADC channel of its pin.
    Adc,
    /// A configured gate group's dead time is at or above the floor.
    DeadTime,
    /// A motor's sensed phase pair is one the chip's image reads.
    PhasePair,
    /// Motor 1 is stored only beside motor 0, and on a two-motor chip each motor's gates are on
    /// its own timer.
    MotorTimer,
    /// An arm-row or calibration value is in its range.
    Range,
    /// A motor's current limit converts above its noise floor.
    LimitFloor,
    /// The IMU axis signs and roles form a proper rotation.
    Frame,
    /// A balance machine in the rigid frame has exactly one board set as leader.
    Leader,
    /// A protection is enabled only on a motor that stores its input
    /// (`specs/optional-protections.md` section 7).
    ProtectInput,
    /// A protection input is stored only on a chip whose image reads it.
    ProtectChip,
}

impl Check {
    /// Every check.
    pub const ALL: [Check; 20] = [
        Check::PinExists,
        Check::Swd,
        Check::PinTwice,
        Check::Group,
        Check::Instance,
        Check::InstanceTwice,
        Check::Af,
        Check::Remap,
        Check::AfForm,
        Check::RemapChip,
        Check::Adc,
        Check::DeadTime,
        Check::PhasePair,
        Check::MotorTimer,
        Check::Range,
        Check::LimitFloor,
        Check::Frame,
        Check::Leader,
        Check::ProtectInput,
        Check::ProtectChip,
    ];

    /// The id the shared cases use.
    pub fn id(self) -> &'static str {
        match self {
            Check::PinExists => "pin_exists",
            Check::Swd => "swd",
            Check::PinTwice => "pin_twice",
            Check::Group => "group",
            Check::Instance => "instance",
            Check::InstanceTwice => "instance_twice",
            Check::Af => "af",
            Check::Remap => "remap",
            Check::AfForm => "af_form",
            Check::RemapChip => "remap_chip",
            Check::Adc => "adc",
            Check::DeadTime => "dead_time",
            Check::PhasePair => "phase_pair",
            Check::MotorTimer => "motor_timer",
            Check::Range => "range",
            Check::LimitFloor => "limit_floor",
            Check::Frame => "frame",
            Check::Leader => "leader",
            Check::ProtectInput => "protect_input",
            Check::ProtectChip => "protect_chip",
        }
    }
}

/// A check that refused, the key it names and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal {
    /// Which check.
    pub check: Check,
    /// The key it names.
    pub key: Key,
    /// What is wrong, starting with the key's path.
    pub msg: String,
}

/// A board's stored values: the newest record of each stored key, its type and bytes.
pub type State = BTreeMap<Key, (Type, Vec<u8>)>;

/// What a pin of a group carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    /// A plain input or output: no alternate function.
    Plain,
    /// An ADC input.
    Adc,
    /// A DC-link current input: an ADC input on ADC2, whose inserted group converts it
    /// (`specs/optional-protections.md` section 3.2).
    DcLink,
    /// The gate timer's break input.
    Break,
    /// The gate timer's channel `n` output.
    Hi(u8),
    /// The gate timer's complementary channel `n` output.
    Lo(u8),
    /// The USART's transmit.
    Tx,
    /// The USART's receive.
    Rx,
    /// The I2C clock.
    Scl,
    /// The I2C data.
    Sda,
}

impl Role {
    /// The function the pin carries on peripheral instance `n`.
    fn func(self, n: u8) -> Option<Func> {
        Some(match self {
            Role::Hi(c) => Func::Gate(n, c),
            Role::Lo(c) => Func::GateN(n, c),
            Role::Tx => Func::Tx(n),
            Role::Rx => Func::Rx(n),
            Role::Scl => Func::Scl(n),
            Role::Sda => Func::Sda(n),
            Role::Break => Func::Break(n),
            Role::Plain | Role::Adc | Role::DcLink => return None,
        })
    }
}

/// What an instance key names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Periph {
    Timer,
    Usart,
    I2c,
}

impl Periph {
    fn has(self, t: &Table, n: u8) -> bool {
        match self {
            Periph::Timer => t.has_timer(n),
            Periph::Usart => t.has_usart(n),
            Periph::I2c => t.has_i2c(n),
        }
    }

    fn name(self) -> &'static str {
        match self {
            Periph::Timer => "advanced timer",
            Periph::Usart => "USART",
            Periph::I2c => "I2C",
        }
    }
}

/// A function's keys: its pins with their roles, and for a peripheral group its instance key and
/// its remap key. Complete when every pin and the instance are stored, absent when none is. A group
/// with a `switch` (the IMU's `imu.model`) is all or none with the switch: off (0 or not stored),
/// its pins and instance may be stored or not. A group with a `pick` (a motor's phase-current
/// sense) is complete when exactly that many of its pins are stored. A group with a `via` (a
/// motor's break input) has no instance of its own: its pin is routed on the timer that key names,
/// another group's instance.
struct Group {
    pins: Vec<(PinField, Role)>,
    instance: Option<(Key, Periph)>,
    remap: Option<Key>,
    switch: Option<Key>,
    pick: Option<usize>,
    via: Option<Key>,
}

impl Group {
    fn single(f: PinField, role: Role) -> Group {
        Group {
            pins: vec![(f, role)],
            instance: None,
            remap: None,
            switch: None,
            pick: None,
            via: None,
        }
    }
}

/// Every function a board can configure, by its groups (`specs/config-read-once.md` section 3).
fn groups() -> Vec<Group> {
    let mut g = vec![
        Group::single(store::BOARD_SELF_HOLD, Role::Plain),
        Group::single(store::BOARD_BATTERY_PIN, Role::Adc),
        Group::single(store::BOARD_BUZZER, Role::Plain),
        Group::single(store::LED_GREEN, Role::Plain),
        Group::single(store::LED_ORANGE, Role::Plain),
        Group::single(store::LED_RED, Role::Plain),
        Group::single(store::PAD_A, Role::Plain),
        Group::single(store::PAD_B, Role::Plain),
        Group::single(store::BOARD_BUTTON, Role::Plain),
        Group {
            pins: vec![
                (store::IMU_SCL_PIN, Role::Scl),
                (store::IMU_SDA_PIN, Role::Sda),
            ],
            instance: Some((store::IMU_I2C.key(), Periph::I2c)),
            remap: Some(store::IMU_REMAP.key()),
            switch: Some(store::IMU_MODEL.key()),
            pick: None,
            via: None,
        },
    ];
    for i in 0..store::LINKS {
        g.push(Group {
            pins: vec![
                (store::LINK_TX.at(i), Role::Tx),
                (store::LINK_RX.at(i), Role::Rx),
            ],
            instance: Some((store::LINK_USART.at(i).key(), Periph::Usart)),
            remap: Some(store::LINK_REMAP.at(i).key()),
            switch: None,
            pick: None,
            via: None,
        });
    }
    for m in 0..store::MOTORS {
        g.push(Group {
            pins: vec![
                (store::MOTOR_HALL_A.at(m), Role::Plain),
                (store::MOTOR_HALL_B.at(m), Role::Plain),
                (store::MOTOR_HALL_C.at(m), Role::Plain),
            ],
            instance: None,
            remap: None,
            switch: None,
            pick: None,
            via: None,
        });
        g.push(Group {
            pins: vec![
                (store::MOTOR_GATE_HI_A.at(m), Role::Hi(0)),
                (store::MOTOR_GATE_HI_B.at(m), Role::Hi(1)),
                (store::MOTOR_GATE_HI_C.at(m), Role::Hi(2)),
                (store::MOTOR_GATE_LO_A.at(m), Role::Lo(0)),
                (store::MOTOR_GATE_LO_B.at(m), Role::Lo(1)),
                (store::MOTOR_GATE_LO_C.at(m), Role::Lo(2)),
            ],
            instance: Some((store::MOTOR_GATE_TIMER.at(m).key(), Periph::Timer)),
            remap: Some(store::MOTOR_GATE_REMAP.at(m).key()),
            switch: None,
            pick: None,
            via: None,
        });
        g.push(Group {
            pins: SENSE_PHASES
                .iter()
                .map(|(f, _)| (f.at(m), Role::Adc))
                .collect(),
            instance: None,
            remap: None,
            switch: None,
            pick: Some(2),
            via: None,
        });
        g.push(Group::single(store::MOTOR_DC_LINK_PIN.at(m), Role::DcLink));
        g.push(Group {
            pins: vec![(store::MOTOR_BREAK_PIN.at(m), Role::Break)],
            instance: None,
            remap: Some(store::MOTOR_GATE_REMAP.at(m).key()),
            switch: None,
            pick: None,
            via: Some(store::MOTOR_GATE_TIMER.at(m).key()),
        });
    }
    g
}

/// Whether the chip's image reads a pin of `role`: a protection input only where the chip's table
/// says so (`hal::pincap::Inputs`), every other pin everywhere.
fn image_reads(t: &Table, role: Role) -> bool {
    match role {
        Role::DcLink => t.inputs.dc_link,
        Role::Break => t.inputs.brk,
        _ => true,
    }
}

/// Whether motor `m` stores a sensed pair: exactly two of its phase pins.
fn senses(state: &State, m: u8) -> bool {
    SENSE_PHASES
        .iter()
        .filter(|(f, _)| state.contains_key(&f.at(m).key()))
        .count()
        == 2
}

/// A motor's phase-current sense pins with the phase each names, in phase order: a motor senses
/// the two that are stored (`specs/twelve-fet.md` section 5.3).
const SENSE_PHASES: [(PinField, Phase); 3] = [
    (store::MOTOR_PHASE_A, Phase::A),
    (store::MOTOR_PHASE_B, Phase::B),
    (store::MOTOR_PHASE_C, Phase::C),
];

/// The advanced timer each motor's gates use where the chip drives two (`specs/twelve-fet.md`
/// section 5.2): TIMER0 triggers the injected ADC group and starts TIMER7, so motor 0 is on TIMER0
/// and motor 1 on TIMER7.
const MOTOR_TIMER: [u8; store::MOTORS as usize] = [0, 7];

/// The stored bytes of a pin field, or `None` when it is not stored (an absent function).
fn pin_of(state: &State, f: PinField) -> Option<Vec<u8>> {
    match state.get(&f.key()) {
        Some((t, b)) if *t == f.kind() && b.len() == f.kind().fixed_len().unwrap_or(0) => {
            Some(b.clone())
        }
        _ => None,
    }
}

/// A stored `u8` key's value.
fn byte_of(state: &State, key: Key) -> Option<u8> {
    match state.get(&key) {
        Some((Type::U8, b)) => b.first().copied(),
        _ => None,
    }
}

/// A stored integer key's value, any integer type.
fn int_of(t: Type, b: &[u8]) -> Option<i64> {
    Some(match Value::decode(t, b)? {
        Value::U8(v) => v.into(),
        Value::U16(v) => v.into(),
        Value::U32(v) => v.into(),
        Value::I16(v) => v.into(),
        Value::I32(v) => v.into(),
        Value::Bool(v) => v.into(),
        _ => return None,
    })
}

/// Every check that needs the chip, on the board as `state` would leave it.
pub fn check_pins(chip: Chip, state: &State) -> Vec<Refusal> {
    let t = chip.table();
    let mut out = Vec::new();
    let refuse = |out: &mut Vec<Refusal>, check, key, msg: String| {
        out.push(Refusal {
            check,
            key,
            msg: format!("{}: {msg}", path_of(key)),
        })
    };
    // Every pin that is stored.
    let mut used: HashMap<(u8, u8), Key> = HashMap::new();
    for g in groups() {
        for (f, _) in &g.pins {
            let Some(b) = pin_of(state, *f) else { continue };
            let (port, pin) = (b[0], b[1]);
            let at = pin_text(port, pin);
            if !t.bonded(port, pin) {
                refuse(
                    &mut out,
                    Check::PinExists,
                    f.key(),
                    format!("{at} does not exist on the {}", chip.name()),
                );
                continue;
            }
            if SWD.contains(&(port, pin)) {
                refuse(
                    &mut out,
                    Check::Swd,
                    f.key(),
                    format!("{at} is an SWD pin, the configuration path"),
                );
                continue;
            }
            if let Some(first) = used.insert((port, pin), f.key()) {
                used.insert((port, pin), first);
                refuse(
                    &mut out,
                    Check::PinTwice,
                    f.key(),
                    format!("{at} is already {}", path_of(first)),
                );
            }
        }
    }
    // Each group: complete or absent, and a complete one routed on this chip.
    let mut instances: HashMap<(Periph, u8), Key> = HashMap::new();
    for g in groups() {
        // A protection input the image does not read is `protect_chip`'s alone (below).
        if !g.pins.iter().all(|(_, role)| image_reads(t, *role)) {
            continue;
        }
        let set: Vec<bool> = g
            .pins
            .iter()
            .map(|(f, _)| state.contains_key(&f.key()))
            .collect();
        let instance = g.instance.map(|(k, p)| (k, p, byte_of(state, k)));
        let stored = set.iter().filter(|s| **s).count();
        let all = g.pick.map_or(stored == set.len(), |n| stored == n)
            && instance.is_none_or(|(_, _, n)| n.is_some());
        let none = stored == 0 && instance.is_none_or(|(_, _, n)| n.is_none());
        // A switched group: on, it is all or none with its switch, so it needs every key; off, its
        // keys may be stored or not, and a whole bus is still routed below.
        let on = g.switch.map(|k| byte_of(state, k).unwrap_or(0) != 0);
        if on == Some(false) && !all {
            continue;
        }
        if none && on != Some(true) {
            continue;
        }
        if !all {
            let head = g
                .pins
                .first()
                .map(|(f, _)| f.key())
                .expect("a group has a pin");
            if let Some(n) = g.pick {
                let paths: Vec<String> = g.pins.iter().map(|(f, _)| path_of(f.key())).collect();
                refuse(
                    &mut out,
                    Check::Group,
                    head,
                    format!(
                        "{n} of {} are stored together, not {stored}",
                        paths.join(", ")
                    ),
                );
                continue;
            }
            let missing = g
                .pins
                .iter()
                .zip(&set)
                .find(|(_, s)| !**s)
                .map(|((f, _), _)| f.key())
                .or(instance.map(|(k, _, _)| k))
                .expect("an incomplete group misses a key");
            refuse(
                &mut out,
                Check::Group,
                head,
                format!(
                    "the group is incomplete: {} is not stored",
                    path_of(missing)
                ),
            );
            continue;
        }
        let n = match instance {
            Some((k, p, Some(n))) => {
                if !p.has(t, n) {
                    refuse(
                        &mut out,
                        Check::Instance,
                        k,
                        format!("the {} has no {} {n}", chip.name(), p.name()),
                    );
                    continue;
                }
                if let Some(first) = instances.insert((p, n), k) {
                    instances.insert((p, n), first);
                    refuse(
                        &mut out,
                        Check::InstanceTwice,
                        k,
                        format!("{} {n} is already {}", p.name(), path_of(first)),
                    );
                }
                Some(n)
            }
            _ => g.via.and_then(|k| byte_of(state, k)),
        };
        let remap = g.remap.map_or(0, |k| byte_of(state, k).unwrap_or(0));
        for (f, role) in &g.pins {
            let Some(b) = pin_of(state, *f) else { continue };
            let (port, pin) = (b[0], b[1]);
            let at = pin_text(port, pin);
            if matches!(role, Role::Adc | Role::DcLink) {
                let want = t.adc_channel(port, pin);
                if want != Some(b[2]) {
                    let msg = match want {
                        Some(c) => format!("{at} is ADC channel {c}, not {}", b[2]),
                        None => format!("{at} is not an ADC input on the {}", chip.name()),
                    };
                    refuse(&mut out, Check::Adc, f.key(), msg);
                } else if *role == Role::DcLink && !t.feeds(port, pin, ADC2) {
                    let msg = format!(
                        "{at} does not feed ADC2, whose inserted group converts the DC-link \
                         inputs"
                    );
                    refuse(&mut out, Check::Adc, f.key(), msg);
                }
                continue;
            }
            let Some(func) = n.and_then(|n| role.func(n)) else {
                continue;
            };
            let sel = t.sel(port, pin, func);
            if chip.remaps() {
                if sel != Some(remap) {
                    let msg = match sel {
                        Some(r) => format!("{at} is {func:?} at remap {r}, not {remap}"),
                        None => format!("{at} cannot carry {func:?} on the {}", chip.name()),
                    };
                    refuse(&mut out, Check::Remap, f.key(), msg);
                }
            } else if sel != Some(b[2]) {
                let msg = match sel {
                    Some(af) => format!("{at} is {func:?} at AF{af}, not AF{}", b[2]),
                    None => format!("{at} cannot carry {func:?} on the {}", chip.name()),
                };
                refuse(&mut out, Check::Af, f.key(), msg);
            }
        }
        if g.instance.is_some_and(|(_, p)| p == Periph::Timer) {
            let m = g.pins[0].0.key().index;
            let dt = byte_of(state, store::MOTOR_DEAD_TIME.at(m).key()).unwrap_or(0);
            if dt < board::DEAD_TIME_MIN_DTG {
                refuse(
                    &mut out,
                    Check::DeadTime,
                    store::MOTOR_DEAD_TIME.at(m).key(),
                    format!(
                        "{dt} is below the floor of {} with the gates configured",
                        board::DEAD_TIME_MIN_DTG
                    ),
                );
            }
        }
    }
    // Each motor's sensed pair is one the chip's image reads.
    for m in 0..store::MOTORS {
        let sensed: Vec<(PinField, Phase)> = SENSE_PHASES
            .iter()
            .map(|(f, p)| (f.at(m), *p))
            .filter(|(f, _)| state.contains_key(&f.key()))
            .collect();
        let [(_, first), (second_field, second)] = sensed[..] else {
            continue;
        };
        if !t.senses(first, second) {
            let pairs: Vec<String> = t
                .sense_pairs
                .iter()
                .map(|[a, b]| format!("{a:?} and {b:?}"))
                .collect();
            refuse(
                &mut out,
                Check::PhasePair,
                second_field.key(),
                format!(
                    "the {} image senses phases {}, not {first:?} and {second:?}",
                    chip.name(),
                    pairs.join(" or ")
                ),
            );
        }
    }
    // Motor 1 runs only beside motor 0, and on a two-motor chip each motor is on its own timer. A
    // motor is stored when its gate group is complete.
    let present: Vec<bool> = (0..store::MOTORS)
        .map(|m| {
            groups()
                .iter()
                .find(|g| g.pins[0].0.key() == store::MOTOR_GATE_HI_A.at(m).key())
                .is_some_and(|g| {
                    g.pins.iter().all(|(f, _)| state.contains_key(&f.key()))
                        && g.instance.is_some_and(|(k, _)| byte_of(state, k).is_some())
                })
        })
        .collect();
    if present[1] && !present[0] {
        refuse(
            &mut out,
            Check::MotorTimer,
            store::MOTOR_GATE_HI_A.at(1).key(),
            "motor 1 is stored without motor 0, whose TIMER0 triggers the ADC and starts motor 1's \
             timer"
                .to_string(),
        );
    }
    if MOTOR_TIMER.iter().all(|&n| t.has_timer(n)) {
        for (m, want) in (0..store::MOTORS).zip(MOTOR_TIMER) {
            let key = store::MOTOR_GATE_TIMER.at(m).key();
            let Some(n) = byte_of(state, key).filter(|n| present[m as usize] && *n != want) else {
                continue;
            };
            refuse(
                &mut out,
                Check::MotorTimer,
                key,
                format!(
                    "motor {m}'s gates are on advanced timer {want} on the {}, not {n}",
                    chip.name()
                ),
            );
        }
    }
    // A protection input is stored only where the chip's image reads it
    // (`specs/optional-protections.md` section 5.2).
    for m in 0..store::MOTORS {
        for (f, role) in [
            (store::MOTOR_DC_LINK_PIN.at(m), Role::DcLink),
            (store::MOTOR_BREAK_PIN.at(m), Role::Break),
        ] {
            if state.contains_key(&f.key()) && !image_reads(t, role) {
                refuse(
                    &mut out,
                    Check::ProtectChip,
                    f.key(),
                    format!("the {} image does not read this input", chip.name()),
                );
            }
        }
    }
    // A remap key is a GD32F10x key.
    if !chip.remaps() {
        let remaps = [store::IMU_REMAP.key()]
            .into_iter()
            .chain((0..store::LINKS).map(|i| store::LINK_REMAP.at(i).key()))
            .chain((0..store::MOTORS).map(|m| store::MOTOR_GATE_REMAP.at(m).key()));
        for k in remaps {
            if state.contains_key(&k) {
                refuse(
                    &mut out,
                    Check::RemapChip,
                    k,
                    format!("the {} has no AFIO remap", chip.name()),
                );
            }
        }
    }
    out
}

/// The `af` rule of a file's `PinAf` values (`specs/config-read-once.md` section 3): given on a
/// chip with a per-pin mux, left out on the GD32F10x, whose `af` byte is not used.
pub fn check_forms(chip: Chip, entries: &[Entry]) -> Vec<Refusal> {
    let mut out = Vec::new();
    for e in entries {
        let Some(row) = crate::names::row_of(e.key) else {
            continue;
        };
        let Json::Object(o) = &e.value else { continue };
        if row.storage != Type::PinAf {
            continue;
        }
        let has = o.contains_key("af");
        let msg = match (chip.remaps(), has) {
            (true, true) => format!(
                "the {} routes by its remap and stores no af: leave af out",
                chip.name()
            ),
            (false, false) => format!("the {} needs the pin's af", chip.name()),
            _ => continue,
        };
        out.push(Refusal {
            check: Check::AfForm,
            key: e.key,
            msg: format!("{}: {msg}", path_of(e.key)),
        });
    }
    out
}

/// Every check that needs no chip: the range table over the arm row, the calibration and the
/// protection settings, each motor's limit above its noise floor, each protection's input, and the
/// IMU frame. The range table's one other field (`wifi.tx_power`) is the forceable one encoding
/// makes.
pub fn check_values(state: &State) -> Vec<Refusal> {
    let mut out = Vec::new();
    for (key, (t, b)) in state {
        let board_check = key.field_id != store::WIFI_TX_POWER.id();
        let Some((lo, hi)) = store::range_of(*key).filter(|_| board_check) else {
            continue;
        };
        let Some(v) = int_of(*t, b) else { continue };
        if !(lo..=hi).contains(&v) {
            out.push(Refusal {
                check: Check::Range,
                key: *key,
                msg: format!("{}: {v} is outside {lo}..{hi}", path_of(*key)),
            });
        }
    }
    // Each motor's limit above its floor, each of the three as the write leaves it or its registry
    // default where it is not stored.
    for m in 0..store::MOTORS {
        let value = |k: Key, default: u32| match state.get(&k) {
            Some((t, b)) => int_of(*t, b).and_then(|v| u32::try_from(v).ok()),
            None => Some(default),
        };
        let limit = store::MOTOR_CURRENT_LIMIT.at(m);
        let cal = store::MOTOR_CURRENT_CAL.at(m);
        let floor = store::MOTOR_NOISE_FLOOR.at(m);
        let (Some(ma), Some(cal), Some(floor)) = (
            value(limit.key(), limit.default()),
            value(cal.key(), cal.default().into()).and_then(|v| u16::try_from(v).ok()),
            value(floor.key(), floor.default().into()).and_then(|v| u16::try_from(v).ok()),
        ) else {
            continue;
        };
        if let Some(why) = limit_floor(ma, cal, floor) {
            out.push(Refusal {
                check: Check::LimitFloor,
                key: limit.key(),
                msg: format!("{}: {why}", path_of(limit.key())),
            });
        }
    }
    // Each protection enabled only beside its input (`specs/optional-protections.md` section 7):
    // a setting as the write leaves it, 0 where it is not stored (each one's default).
    for m in 0..store::MOTORS {
        let set = |k: Key| {
            state
                .get(&k)
                .and_then(|(t, b)| int_of(*t, b))
                .is_some_and(|v| v != 0)
        };
        let checks = [
            (
                store::MOTOR_DC_LINK_LIMIT.at(m).key(),
                state.contains_key(&store::MOTOR_DC_LINK_PIN.at(m).key()),
                path_of(store::MOTOR_DC_LINK_PIN.at(m).key()),
            ),
            (
                store::MOTOR_BREAK_ENABLE.at(m).key(),
                state.contains_key(&store::MOTOR_BREAK_PIN.at(m).key()),
                path_of(store::MOTOR_BREAK_PIN.at(m).key()),
            ),
            (
                store::MOTOR_RAIL_SKIP.at(m).key(),
                senses(state, m),
                format!("a sensed pair (two of motors[{m}].current.phase_a to phase_c)"),
            ),
        ];
        for (key, has_input, input) in checks {
            if set(key) && !has_input {
                out.push(Refusal {
                    check: Check::ProtectInput,
                    key,
                    msg: format!("{}: enabled without its input, {input}", path_of(key)),
                });
            }
        }
    }
    let i32_of = |k: Key| match state.get(&k) {
        Some((Type::I32, b)) => int_of(Type::I32, b).map_or(0, |v| v as i32),
        _ => 0,
    };
    let sign = [0, 1, 2].map(|i| i32_of(store::IMU_AXIS_SIGN.at(i).key()));
    let roles = [0, 1].map(|i| byte_of(state, store::IMU_AXIS_ROLE.at(i).key()).unwrap_or(0));
    if let Some(e) = imu::Config::frame_error(sign, roles) {
        let (key, why) = match e {
            imu::FrameError::Roles => (
                store::IMU_AXIS_ROLE.at(0).key(),
                "the two roles must name two different chip axes",
            ),
            imu::FrameError::Accel => (
                store::IMU_AXIS_SIGN.at(0).key(),
                "the signs read through the roles are a mirror image, not a rotation",
            ),
        };
        out.push(Refusal {
            check: Check::Frame,
            key,
            msg: format!("{}: {why}", path_of(key)),
        });
    }
    out
}

/// Why a motor's limit of `ma` milliamps fails the `limit_floor` check at `cal` counts per amp and a
/// noise floor of `floor` counts, or `None` when it passes (`specs/config-read-once.md` section 4).
/// The arithmetic is the firmware's (`board::limit`): a limit passes when FOC's full scale
/// (`full_scale_counts`, the converted limit less the floor) is above zero, which is a conversion
/// above the floor. The lowest limit that passes is `floor * 1000 / cal + 1` rounded up to the next
/// 100 mA, stepped up by 100 mA while the conversion's rounding still leaves it at the floor.
fn limit_floor(ma: u32, cal: u16, floor: u16) -> Option<String> {
    use board::limit::{
        full_scale_counts, limit_counts, noise_floor_counts, request_counts, CURRENT_CAL_MAX,
        CURRENT_CAL_MIN, LIMIT_REQUEST_CAP_MA,
    };
    let passes = |ma: u32| full_scale_counts(limit_counts(ma, cal, floor), floor) > 0;
    if passes(ma) {
        return None;
    }
    let scale = cal.clamp(CURRENT_CAL_MIN, CURRENT_CAL_MAX) as u32;
    let floor_counts = noise_floor_counts(floor) as u32;
    let start = (floor_counts * 1_000 / scale + 1).div_ceil(100) * 100;
    let lowest = (start..=LIMIT_REQUEST_CAP_MA)
        .step_by(100)
        .find(|ma| passes(*ma));
    let fix = match lowest {
        Some(l) => format!("the lowest limit that passes is {l} mA"),
        None => "no limit passes a floor at the top of the limit window".to_string(),
    };
    Some(format!(
        "{ma} mA is {} counts at a current.cal of {scale}, at or below the noise floor of \
         {floor_counts} counts, so FOC's full scale is 0; {fix}",
        request_counts(ma, cal)
    ))
}

/// The keys whose write can change [`check_leader`]'s verdict: `machine.mode`, `machine.frame` and
/// `control.leader`. No other write runs it, so a machine staged before the field existed does not
/// block unrelated writes.
pub const LEADER_KEYS: [Key; 3] = [
    store::CONTROL_MODE.key(),
    store::CONTROL_FRAME.key(),
    store::CONTROL_LEADER.key(),
];

/// Whether writing `key` runs the leader check.
pub fn changes_leader(key: Key) -> bool {
    LEADER_KEYS.contains(&key)
}

/// The leader check (`specs/rigid-leader.md` section 6.1) over every motor controller of the
/// machine, each by its address and its stored values as the write would leave it: when any of
/// them stores `machine.mode` balance and `machine.frame` rigid, exactly one stores
/// `control.leader` set. Any other mode or frame passes, whatever the flags hold.
pub fn check_leader(boards: &[(u8, &State)]) -> Option<Refusal> {
    let is = |s: &State, key: Key, v: u8| byte_of(s, key).unwrap_or(0) == v;
    let rigid_balance = boards
        .iter()
        .any(|(_, s)| is(s, store::CONTROL_MODE.key(), 1) && is(s, store::CONTROL_FRAME.key(), 1));
    if !rigid_balance {
        return None;
    }
    let set: Vec<u8> = boards
        .iter()
        .filter(|(_, s)| !is(s, store::CONTROL_LEADER.key(), 0))
        .map(|(a, _)| *a)
        .collect();
    let why = match set.as_slice() {
        [_] => return None,
        [] => "none is set".to_string(),
        many => {
            let names: Vec<String> = many.iter().map(|a| format!("{a:#04x}")).collect();
            let (last, rest) = names.split_last().expect("two or more");
            let verb = if many.len() == 2 { "both" } else { "all" };
            format!("{} and {last} are {verb} set", rest.join(", "))
        }
    };
    Some(Refusal {
        check: Check::Leader,
        key: store::CONTROL_LEADER.key(),
        msg: format!(
            "{}: a balance machine in the rigid frame needs exactly one board set as leader; {why}",
            path_of(store::CONTROL_LEADER.key())
        ),
    })
}

/// The first pin `entries` carry: its `af` form is checked against the chip whether or not the
/// write changes it ([`check_forms`]).
pub fn first_pin(entries: &[Entry]) -> Option<Key> {
    entries.iter().map(|e| e.key).find(|k| {
        crate::names::row_of(*k)
            .is_some_and(|r| matches!(r.storage, Type::Pin | Type::PinAf | Type::PinAdc))
    })
}

/// Whether writing `key` changes what the pin checks see: a pin, instance, remap, dead-time or
/// `imu.model` key.
pub fn needs_chip(key: Key) -> bool {
    let pin = crate::names::row_of(key)
        .is_some_and(|r| matches!(r.storage, Type::Pin | Type::PinAf | Type::PinAdc));
    let extra = [
        store::IMU_I2C.id(),
        store::IMU_MODEL.id(),
        store::IMU_REMAP.id(),
        store::LINK_USART.id(),
        store::LINK_REMAP.id(),
        store::MOTOR_GATE_TIMER.id(),
        store::MOTOR_GATE_REMAP.id(),
        store::MOTOR_DEAD_TIME.id(),
    ];
    pin || extra.contains(&key.field_id)
}

/// The chip a `TARGET_ID` names (`espsvc::TargetId`'s family and flash size), or `None` for a part
/// the HAL builds no image for.
pub fn chip_of(family: u8, flash_kib: u16) -> Option<Chip> {
    Some(match (family, flash_kib) {
        (espsvc::FAMILY_F10X, 64) => Chip::Gd32f103c8,
        (espsvc::FAMILY_F10X, 256) => Chip::Gd32f103rc,
        (espsvc::FAMILY_F1X0, 64) => Chip::Gd32f130c8,
        (espsvc::FAMILY_MM32SPIN05, 32) => Chip::Mm32spin05,
        _ => return None,
    })
}

#[cfg(test)]
mod tests;
