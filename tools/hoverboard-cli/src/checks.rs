//! The checks a board's configuration passes before it is written (`specs/config-read-once.md`
//! section 4). The firmware applies what is stored and refuses nothing, so these are the only
//! guard between a configuration and the registers: every `board apply`, `board replace`,
//! `board set` and `machine apply` runs them on the board as it would be after the write, and
//! writes nothing if one refuses. `--force` overrides none of them.
//!
//! Two kinds. The pin checks need the chip, which the CLI reads from `TARGET_ID` through the
//! board's bridge, and test the pin, instance and remap keys against `hal::pincap`'s table of that
//! chip. The value checks need no chip: the arm row and the calibration against the store's range
//! table (`store::RANGES`), and the IMU frame against `imu::Config::frame_error`.
//!
//! `protocol-kotlin` ports the same checks for the app. The two are held together by the shared
//! check cases (`shared/config-checks/`), which this module's tests and the Kotlin tests both run:
//! every [`Check`] has a passing and a refused case there.

use std::collections::{BTreeMap, HashMap};

use hal::pincap::{Chip, Func, Table, SWD};
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
    /// An arm-row or calibration value is in its range.
    Range,
    /// The IMU axis signs and roles form a proper rotation.
    Frame,
}

impl Check {
    /// Every check.
    pub const ALL: [Check; 14] = [
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
        Check::Range,
        Check::Frame,
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
            Check::Range => "range",
            Check::Frame => "frame",
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
            Role::Plain | Role::Adc => return None,
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
/// its pins and instance may be stored or not.
struct Group {
    pins: Vec<(PinField, Role)>,
    instance: Option<(Key, Periph)>,
    remap: Option<Key>,
    switch: Option<Key>,
}

impl Group {
    fn single(f: PinField, role: Role) -> Group {
        Group {
            pins: vec![(f, role)],
            instance: None,
            remap: None,
            switch: None,
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
        });
        g.push(Group {
            pins: vec![
                (store::MOTOR_PHASE_A.at(m), Role::Adc),
                (store::MOTOR_PHASE_B.at(m), Role::Adc),
            ],
            instance: None,
            remap: None,
            switch: None,
        });
    }
    g
}

/// The stored bytes of a pin field, or its fleet default.
fn pin_of(state: &State, f: PinField) -> Option<Vec<u8>> {
    match state.get(&f.key()) {
        Some((t, b)) if *t == f.kind() && b.len() == f.kind().fixed_len().unwrap_or(0) => {
            Some(b.clone())
        }
        Some(_) => None,
        None => f.default().map(|(port, pin)| vec![port, pin, 0]),
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
    // Every pin that is set, stored or by its fleet default.
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
        let set: Vec<bool> = g
            .pins
            .iter()
            .map(|(f, _)| state.contains_key(&f.key()) || f.default().is_some())
            .collect();
        let instance = g.instance.map(|(k, p)| (k, p, byte_of(state, k)));
        let all = set.iter().all(|s| *s) && instance.is_none_or(|(_, _, n)| n.is_some());
        let none = !set.iter().any(|s| *s) && instance.is_none_or(|(_, _, n)| n.is_none());
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
            let missing = g
                .pins
                .iter()
                .zip(&set)
                .find(|(_, s)| !**s)
                .map(|((f, _), _)| f.key())
                .or(instance.map(|(k, _, _)| k))
                .expect("an incomplete group misses a key");
            let head = g
                .pins
                .first()
                .map(|(f, _)| f.key())
                .expect("a group has a pin");
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
            _ => None,
        };
        let remap = g.remap.map_or(0, |k| byte_of(state, k).unwrap_or(0));
        for (f, role) in &g.pins {
            let Some(b) = pin_of(state, *f) else { continue };
            let (port, pin) = (b[0], b[1]);
            let at = pin_text(port, pin);
            if *role == Role::Adc {
                let want = t.adc_channel(port, pin);
                if want != Some(b[2]) {
                    let msg = match want {
                        Some(c) => format!("{at} is ADC channel {c}, not {}", b[2]),
                        None => format!("{at} is not an ADC input on the {}", chip.name()),
                    };
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

/// Every check that needs no chip: the range table over the arm row and the calibration, and the
/// IMU frame. Another field's range (`wifi.tx_power`) is the forceable one encoding makes.
pub fn check_values(state: &State) -> Vec<Refusal> {
    let mut out = Vec::new();
    for (key, (t, b)) in state {
        let board_check = store::lookup_key(*key).is_some_and(|d| d.effect == store::Effect::Arm)
            || store::CALIBRATION.contains(&key.field_id);
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
