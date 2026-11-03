//! The boot read of the board's pins and peripherals (`specs/config-read-once.md` section 5): the
//! pin, instance and remap keys in one store pass, assembled into the groups the bring-ups write
//! into the registers.
//!
//! The values are the register model (port, pin, alternate function, ADC channel, instance) and
//! nothing here checks them: the CLI and the app check a configuration before they write it
//! (section 4). A group with a key missing is not brought up; that is the only test. A record of
//! another type than its key's (an older image's record under a reused id) reads as not stored,
//! the store's own rule for a typed read.
//!
//! This half is pure and host-tested over a real `Store`; the HAL calls that write the values are
//! in `main.rs`'s and `motor.rs`'s target-only bring-ups.

// The callers are the target-only boot in `main.rs`; the host build compiles this half for its
// tests alone.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use crate::motor::COUNT;
use store::{Flash, Key, PinField, Store, Type};

/// A pin: port (`0` = A), pin number.
pub type Pin = [u8; 2];
/// A pin and its alternate function, or its ADC channel: port, pin, af or channel.
pub type Pin3 = [u8; 3];

/// One motor's configuration, present when its halls, its six gates and its gate timer are all
/// stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Motor {
    /// The hall inputs a, b, c.
    pub halls: [Pin; 3],
    /// The gates hi a, b, c, then lo a, b, c, with their alternate functions.
    pub gates: [Pin3; 6],
    /// The advanced timer the gates use (`0` = TIMER0, `7` = TIMER7).
    pub timer: u8,
    /// The timer's AFIO remap (GD32F10x).
    pub remap: u8,
    /// The dead time, DTG ticks.
    pub dead_time: u8,
    /// `current.cal_state` = `low_sides_on`: the zeros are measured with the low sides on
    /// (`specs/twelve-fet.md` section 5.5); `false` (`floating`, the default) with every FET off.
    pub cal_low_sides: bool,
    /// The sensed phase pair with their ADC channels, `None` when the motor has no current sense.
    pub phase: Option<Sense>,
}

/// Which two of the three phases a motor's shunts sense (`specs/twelve-fet.md` section 5.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pair {
    /// Phases A and B, every one-motor board's pair.
    Ab,
    /// Phases B and C: the 12-FET's motor 0 (EFeru's right motor).
    Bc,
}

/// A motor's current sense: the two sensed pins with their ADC channels, in phase order (the order
/// the injected group converts them), and which phases they are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sense {
    /// The two sensed phases' pins and channels, the lower phase first.
    pub pins: [Pin3; 2],
    /// Which phases they are.
    pub pair: Pair,
}

/// The sensed pair from the three phase pins as stored: current sense is fitted when exactly two
/// are, the group rule (`specs/twelve-fet.md` section 5.3). A and C is on no board and the CLI
/// refuses it (`phase_pair`); read here it is no current sense. A one-motor image never reads
/// `phase_c`, so its `c` is always `None` and the rule is the A and B pair it always was.
fn sense(a: Option<Pin3>, b: Option<Pin3>, c: Option<Pin3>) -> Option<Sense> {
    match (a, b, c) {
        (Some(a), Some(b), None) => Some(Sense {
            pins: [a, b],
            pair: Pair::Ab,
        }),
        (None, Some(b), Some(c)) => Some(Sense {
            pins: [b, c],
            pair: Pair::Bc,
        }),
        _ => None,
    }
}

/// One link's configuration, present when its two pins and its USART are stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Link {
    /// Transmit, with its alternate function.
    pub tx: Pin3,
    /// Receive, with its alternate function.
    pub rx: Pin3,
    /// The USART (the HAL's label number).
    pub usart: u8,
    /// The USART's AFIO remap (GD32F10x).
    pub remap: u8,
}

/// The IMU, present when `imu.model` is non-zero and the bus's two pins and its I2C instance are
/// stored: `imu.model` is the switch, so a board can keep its IMU pins with the IMU off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuBus {
    /// The stored `imu.model`, non-zero.
    pub model: u8,
    /// The clock, with its alternate function.
    pub scl: Pin3,
    /// The data, with its alternate function.
    pub sda: Pin3,
    /// The I2C instance (the HAL's label number).
    pub i2c: u8,
    /// The I2C's AFIO remap (GD32F10x).
    pub remap: u8,
}

/// Everything the boot brings up from the store's pin, instance and remap keys.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    /// The power latch, when stored: an unstored latch pin is not touched.
    pub self_hold: Option<Pin>,
    /// The battery-sense input and its ADC channel.
    pub battery: Option<Pin3>,
    /// The button, pad A and pad B.
    pub inputs: [Option<Pin>; 3],
    /// The motors this image drives, by index (`motor::COUNT`, `specs/twelve-fet.md` section 4.2).
    pub motors: [Option<Motor>; COUNT],
    /// The two links.
    pub links: [Option<Link>; store::LINKS as usize],
    /// The IMU.
    pub imu: Option<ImuBus>,
    /// The stored `imu.model` (0 when not stored): whether the store asks for an IMU at all.
    pub imu_model: u8,
}

/// The slots of the pass: one per key it reads, in [`KEYS`] order. One group of wheel keys per
/// motor first, motor `m`'s at `WHEEL * m` in [`WHEEL_KEYS`] order, then the board's keys from
/// [`BOARD`]. The wheel groups lead because they are read the most: their slots then sit at the
/// small offsets the short load encodings reach.
const BOARD: usize = WHEEL * COUNT;
const SELF_HOLD: usize = BOARD;
const BATTERY: usize = BOARD + 1;
const BUTTON: usize = BOARD + 2;
const PAD_A: usize = BOARD + 3;
const PAD_B: usize = BOARD + 4;
const SCL: usize = BOARD + 5;
const SDA: usize = BOARD + 6;
const I2C: usize = BOARD + 7;
const IMU_REMAP: usize = BOARD + 8;
/// Link `i`'s TX, RX, USART and remap are at `LINK + 4 * i`.
const LINK: usize = BOARD + 9;
/// `imu.model`, after the links.
const IMU_MODEL: usize = LINK + 4 * store::LINKS as usize;
/// How many board keys the pass reads.
const BOARD_N: usize = IMU_MODEL + 1 - BOARD;

/// Within a motor's group: the halls, the gates, the gate timer, its remap, the dead time, the
/// calibration state, the phase pins.
const HALLS: usize = 0;
const GATES: usize = 3;
const GATE_TIMER: usize = 9;
const GATE_REMAP: usize = 10;
const DEAD_TIME: usize = 11;
const CAL_STATE: usize = 12;
const PHASE: usize = 13;
/// A motor's group: every wheel key, with `phase_c` only where the image reads it, which is where
/// it drives two motors (`specs/twelve-fet.md` section 5.3: the one-motor images do not carry it).
const WHEEL: usize = if COUNT > 1 { 16 } else { 15 };
const N: usize = BOARD + BOARD_N;

/// The board's keys of the pass, slots `BOARD..N`.
const BOARD_KEYS: [(Key, Type); BOARD_N] = [
    pin(store::BOARD_SELF_HOLD),
    pin(store::BOARD_BATTERY_PIN),
    pin(store::BOARD_BUTTON),
    pin(store::PAD_A),
    pin(store::PAD_B),
    pin(store::IMU_SCL_PIN),
    pin(store::IMU_SDA_PIN),
    (store::IMU_I2C.key(), Type::U8),
    (store::IMU_REMAP.key(), Type::U8),
    pin(store::LINK_TX.at(0)),
    pin(store::LINK_RX.at(0)),
    (store::LINK_USART.at(0).key(), Type::U8),
    (store::LINK_REMAP.at(0).key(), Type::U8),
    pin(store::LINK_TX.at(1)),
    pin(store::LINK_RX.at(1)),
    (store::LINK_USART.at(1).key(), Type::U8),
    (store::LINK_REMAP.at(1).key(), Type::U8),
    (store::IMU_MODEL.key(), Type::U8),
];

/// One motor's wheel keys at index 0, in group order; the pass takes the first [`WHEEL`] of them
/// at each motor's index.
const WHEEL_KEYS: [(Key, Type); 16] = [
    pin(store::MOTOR_HALL_A),
    pin(store::MOTOR_HALL_B),
    pin(store::MOTOR_HALL_C),
    pin(store::MOTOR_GATE_HI_A),
    pin(store::MOTOR_GATE_HI_B),
    pin(store::MOTOR_GATE_HI_C),
    pin(store::MOTOR_GATE_LO_A),
    pin(store::MOTOR_GATE_LO_B),
    pin(store::MOTOR_GATE_LO_C),
    (store::MOTOR_GATE_TIMER.key(), Type::U8),
    (store::MOTOR_GATE_REMAP.key(), Type::U8),
    (store::MOTOR_DEAD_TIME.key(), Type::U8),
    (store::MOTOR_CAL_STATE.key(), Type::U8),
    pin(store::MOTOR_PHASE_A),
    pin(store::MOTOR_PHASE_B),
    pin(store::MOTOR_PHASE_C),
];

/// The keys of the pass, slot by slot, and the type each must be stored as: [`WHEEL_KEYS`] at each
/// motor's index, then [`BOARD_KEYS`].
const KEYS: [(Key, Type); N] = {
    let mut k = [(
        Key {
            field_id: 0,
            index: 0,
        },
        Type::U8,
    ); N];
    let mut i = 0;
    while i < BOARD_N {
        k[BOARD + i] = BOARD_KEYS[i];
        i += 1;
    }
    let mut m = 0;
    while m < COUNT {
        let mut j = 0;
        while j < WHEEL {
            let (key, kind) = WHEEL_KEYS[j];
            k[WHEEL * m + j] = (
                Key {
                    field_id: key.field_id,
                    index: m as u8,
                },
                kind,
            );
            j += 1;
        }
        m += 1;
    }
    k
};

/// [`KEYS`]' keys alone, the filter the pass takes.
const KEY_LIST: [Key; N] = {
    let mut k = [Key {
        field_id: 0,
        index: 0,
    }; N];
    let mut i = 0;
    while i < N {
        k[i] = KEYS[i].0;
        i += 1;
    }
    k
};

const fn pin(f: PinField) -> (Key, Type) {
    (f.key(), f.kind())
}

/// What the pass handed back for one key: its newest record's bytes, up to three.
#[derive(Clone, Copy)]
struct Slot {
    stored: bool,
    b: [u8; 3],
}

/// Read every key of [`KEYS`] in one store pass (`Store::for_each_in`) and assemble the groups.
///
/// `#[inline(never)]`: one call, at boot, and its slot array is this frame's, gone before the
/// bring-ups run.
#[inline(never)]
pub fn read<F: Flash>(s: &Store<F>) -> Layout {
    let mut slots = [Slot {
        stored: false,
        b: [0; 3],
    }; N];
    s.for_each_in(&KEY_LIST, |key, tag, bytes| {
        if let Some(i) = KEY_LIST.iter().position(|k| *k == key) {
            let kind = KEYS[i].1;
            let slot = &mut slots[i];
            // Newest wins by log order; a tombstone or a record of another type un-stores.
            slot.stored = tag == kind.tag() && Some(bytes.len()) == kind.fixed_len();
            if slot.stored {
                slot.b[..bytes.len()].copy_from_slice(bytes);
            }
        }
    });
    let p = |i: usize| -> Option<Pin> { slots[i].stored.then(|| [slots[i].b[0], slots[i].b[1]]) };
    let p3 = |i: usize| -> Option<Pin3> { slots[i].stored.then_some(slots[i].b) };
    let byte = |i: usize| -> Option<u8> { slots[i].stored.then_some(slots[i].b[0]) };

    let motor = |m: usize| -> Option<Motor> {
        let at = WHEEL * m;
        Some(Motor {
            halls: [p(at + HALLS)?, p(at + HALLS + 1)?, p(at + HALLS + 2)?],
            gates: [
                p3(at + GATES)?,
                p3(at + GATES + 1)?,
                p3(at + GATES + 2)?,
                p3(at + GATES + 3)?,
                p3(at + GATES + 4)?,
                p3(at + GATES + 5)?,
            ],
            timer: byte(at + GATE_TIMER)?,
            remap: byte(at + GATE_REMAP).unwrap_or(0),
            dead_time: byte(at + DEAD_TIME).unwrap_or(0),
            cal_low_sides: byte(at + CAL_STATE) == Some(1),
            phase: sense(
                p3(at + PHASE),
                p3(at + PHASE + 1),
                if WHEEL > PHASE + 2 {
                    p3(at + PHASE + 2)
                } else {
                    None
                },
            ),
        })
    };
    let mut motors = [None; COUNT];
    for (m, slot) in motors.iter_mut().enumerate() {
        *slot = motor(m);
    }
    let link = |i: usize| -> Option<Link> {
        let at = LINK + 4 * i;
        Some(Link {
            tx: p3(at)?,
            rx: p3(at + 1)?,
            usart: byte(at + 2)?,
            remap: byte(at + 3).unwrap_or(0),
        })
    };
    let imu_model = byte(IMU_MODEL).unwrap_or(0);
    let imu = (|| {
        Some(ImuBus {
            model: Some(imu_model).filter(|m| *m != 0)?,
            scl: p3(SCL)?,
            sda: p3(SDA)?,
            i2c: byte(I2C)?,
            remap: byte(IMU_REMAP).unwrap_or(0),
        })
    })();
    Layout {
        self_hold: p(SELF_HOLD),
        battery: p3(BATTERY),
        inputs: [p(BUTTON), p(PAD_A), p(PAD_B)],
        motors,
        links: [link(0), link(1)],
        imu,
        imu_model,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_flash::TestFlash;
    use store::Value;

    fn with(stage: impl FnOnce(&mut Store<TestFlash>)) -> Layout {
        let mut flash = TestFlash::erased();
        let mut s = Store::mount(&mut flash).unwrap();
        stage(&mut s);
        read(&s)
    }

    fn set(s: &mut Store<TestFlash>, k: Key, v: Value) {
        s.set_value(k, v).unwrap();
    }

    /// The offroad master's motor 0: halls PC13/PA1/PC14, gates PA8-10 and PB13-15 at AF2 on
    /// TIMER0, dead time 28, phase sense PA0/PB0 on channels 0 and 8.
    fn stage_motor(s: &mut Store<TestFlash>) {
        for (f, v) in [
            (store::MOTOR_HALL_A, Value::Pin([2, 13])),
            (store::MOTOR_HALL_B, Value::Pin([0, 1])),
            (store::MOTOR_HALL_C, Value::Pin([2, 14])),
            (store::MOTOR_GATE_HI_A, Value::PinAf([0, 8, 2])),
            (store::MOTOR_GATE_HI_B, Value::PinAf([0, 9, 2])),
            (store::MOTOR_GATE_HI_C, Value::PinAf([0, 10, 2])),
            (store::MOTOR_GATE_LO_A, Value::PinAf([1, 13, 2])),
            (store::MOTOR_GATE_LO_B, Value::PinAf([1, 14, 2])),
            (store::MOTOR_GATE_LO_C, Value::PinAf([1, 15, 2])),
            (store::MOTOR_PHASE_A, Value::PinAdc([0, 0, 0])),
            (store::MOTOR_PHASE_B, Value::PinAdc([1, 0, 8])),
        ] {
            set(s, f.key(), v);
        }
        s.set(store::MOTOR_GATE_TIMER, 0).unwrap();
        s.set(store::MOTOR_DEAD_TIME, 28).unwrap();
    }

    /// `specs/no-fleet-pins.md`: with nothing stored there is no latch pin either, so the boot
    /// leaves PB12 (and every other pin) unconfigured.
    #[test]
    fn nothing_stored_is_nothing_at_all() {
        let l = with(|_| {});
        assert_eq!(
            (
                l.self_hold,
                l.battery,
                l.inputs,
                l.motors,
                l.links,
                l.imu,
                l.imu_model
            ),
            (None, None, [None; 3], [None; COUNT], [None; 2], None, 0)
        );
    }

    /// A stored latch is the one handed on, and an un-stored one (a tombstone) reads as none, not
    /// as some fallback pin.
    #[test]
    fn the_latch_is_the_stored_pin_or_none() {
        let l = with(|s| set(s, store::BOARD_SELF_HOLD.key(), Value::Pin([1, 12])));
        assert_eq!(l.self_hold, Some([1, 12]));
        let l = with(|s| {
            set(s, store::BOARD_SELF_HOLD.key(), Value::Pin([1, 2]));
            s.erase_value(store::BOARD_SELF_HOLD.key()).unwrap();
        });
        assert_eq!(l.self_hold, None);
    }

    #[test]
    fn the_values_handed_on_are_the_stored_bytes() {
        let l = with(stage_motor);
        let m = l.motors[0].unwrap();
        assert_eq!(m.halls, [[2, 13], [0, 1], [2, 14]]);
        assert_eq!(m.gates[0], [0, 8, 2]);
        assert_eq!(m.gates[5], [1, 15, 2]);
        assert_eq!((m.timer, m.remap, m.dead_time), (0, 0, 28));
        assert!(!m.cal_low_sides, "floating when not stored");
        let l = with(|s| {
            stage_motor(s);
            s.set(store::MOTOR_CAL_STATE, 1).unwrap();
        });
        assert!(l.motors[0].unwrap().cal_low_sides);
        assert_eq!(
            m.phase,
            Some(Sense {
                pins: [[0, 0, 0], [1, 0, 8]],
                pair: Pair::Ab
            })
        );
        let l = with(|s| {
            set(s, store::LINK_TX.at(1).key(), Value::PinAf([0, 2, 1]));
            set(s, store::LINK_RX.at(1).key(), Value::PinAf([0, 3, 1]));
            s.set(store::LINK_USART.at(1), 1).unwrap();
            set(s, store::IMU_SCL_PIN.key(), Value::PinAf([1, 10, 1]));
            set(s, store::IMU_SDA_PIN.key(), Value::PinAf([1, 11, 1]));
            s.set(store::IMU_I2C, 1).unwrap();
            s.set(store::IMU_MODEL, 2).unwrap();
            set(s, store::BOARD_SELF_HOLD.key(), Value::Pin([2, 13]));
            set(s, store::BOARD_BATTERY_PIN.key(), Value::PinAdc([0, 4, 4]));
            set(s, store::PAD_B.key(), Value::Pin([2, 15]));
        });
        assert_eq!(l.links[0], None);
        assert_eq!(
            l.links[1],
            Some(Link {
                tx: [0, 2, 1],
                rx: [0, 3, 1],
                usart: 1,
                remap: 0
            })
        );
        assert_eq!(
            l.imu,
            Some(ImuBus {
                model: 2,
                scl: [1, 10, 1],
                sda: [1, 11, 1],
                i2c: 1,
                remap: 0
            })
        );
        assert_eq!(l.self_hold, Some([2, 13]));
        assert_eq!(l.battery, Some([0, 4, 4]));
        assert_eq!(l.inputs, [None, None, Some([2, 15])]);
    }

    #[test]
    fn a_group_with_a_key_missing_is_not_brought_up() {
        // A gate group without its timer, a link without its USART, an IMU bus without SDA.
        let l = with(|s| {
            stage_motor(s);
            s.erase_value(store::MOTOR_GATE_TIMER.key()).unwrap();
            set(s, store::LINK_TX.key(), Value::PinAf([0, 2, 1]));
            set(s, store::LINK_RX.key(), Value::PinAf([0, 3, 1]));
            set(s, store::IMU_SCL_PIN.key(), Value::PinAf([1, 6, 1]));
            s.set(store::IMU_I2C, 0).unwrap();
        });
        assert_eq!((l.motors[0], l.links[0], l.imu), (None, None, None));
        // A motor without the phase pair is a motor with no current sense.
        let l = with(|s| {
            stage_motor(s);
            s.erase_value(store::MOTOR_PHASE_B.key()).unwrap();
        });
        assert_eq!(l.motors[0].unwrap().phase, None);
    }

    /// The 12-FET's second motor (`specs/twelve-fet.md` section 9): its wheel keys at index 1 are
    /// motor 1, read in the same pass as motor 0's, and its sensed pair is A and B where motor 0's
    /// is B and C.
    #[test]
    fn motor_1_is_its_keys_at_index_1_and_a_pair_is_the_two_phases_stored() {
        let l = with(|s| {
            stage_motor(s);
            // Motor 0 senses B and C (PC4, PC5 on channels 14 and 15).
            s.erase_value(store::MOTOR_PHASE_A.key()).unwrap();
            set(s, store::MOTOR_PHASE_B.key(), Value::PinAdc([2, 4, 14]));
            set(s, store::MOTOR_PHASE_C.key(), Value::PinAdc([2, 5, 15]));
            for (f, v) in [
                (store::MOTOR_HALL_A, Value::Pin([1, 5])),
                (store::MOTOR_HALL_B, Value::Pin([1, 6])),
                (store::MOTOR_HALL_C, Value::Pin([1, 7])),
                (store::MOTOR_GATE_HI_A, Value::PinAf([2, 6, 0])),
                (store::MOTOR_GATE_HI_B, Value::PinAf([2, 7, 0])),
                (store::MOTOR_GATE_HI_C, Value::PinAf([2, 8, 0])),
                (store::MOTOR_GATE_LO_A, Value::PinAf([0, 7, 0])),
                (store::MOTOR_GATE_LO_B, Value::PinAf([1, 0, 0])),
                (store::MOTOR_GATE_LO_C, Value::PinAf([1, 1, 0])),
                (store::MOTOR_PHASE_A, Value::PinAdc([0, 0, 0])),
                (store::MOTOR_PHASE_B, Value::PinAdc([2, 3, 13])),
            ] {
                set(s, f.at(1).key(), v);
            }
            s.set(store::MOTOR_GATE_TIMER.at(1), 7).unwrap();
            s.set(store::MOTOR_DEAD_TIME.at(1), 32).unwrap();
        });
        let [m0, m1] = l.motors;
        let (m0, m1) = (m0.unwrap(), m1.unwrap());
        assert_eq!(
            m0.phase,
            Some(Sense {
                pins: [[2, 4, 14], [2, 5, 15]],
                pair: Pair::Bc
            })
        );
        assert_eq!((m0.timer, m0.dead_time), (0, 28), "motor 0 untouched");
        assert_eq!(m1.halls, [[1, 5], [1, 6], [1, 7]]);
        assert_eq!(m1.gates[3], [0, 7, 0]);
        assert_eq!((m1.timer, m1.remap, m1.dead_time), (7, 0, 32));
        assert_eq!(
            m1.phase,
            Some(Sense {
                pins: [[0, 0, 0], [2, 3, 13]],
                pair: Pair::Ab
            })
        );
    }

    /// Current sense is fitted when exactly two phase pins are stored, and A with C is no pair.
    #[test]
    fn three_phase_pins_or_a_and_c_are_no_current_sense() {
        let l = with(|s| {
            stage_motor(s);
            set(s, store::MOTOR_PHASE_C.key(), Value::PinAdc([2, 5, 15]));
        });
        assert_eq!(l.motors[0].unwrap().phase, None, "three stored");
        let l = with(|s| {
            stage_motor(s);
            s.erase_value(store::MOTOR_PHASE_B.key()).unwrap();
            set(s, store::MOTOR_PHASE_C.key(), Value::PinAdc([2, 5, 15]));
        });
        assert_eq!(l.motors[0].unwrap().phase, None, "A and C");
    }

    /// `imu.model` is the switch: the bus stored whole with no model is no IMU (the rover slave),
    /// and a model with the bus incomplete is none either.
    #[test]
    fn the_imu_needs_its_model_and_its_whole_bus() {
        let bus = |s: &mut Store<TestFlash>| {
            set(s, store::IMU_SCL_PIN.key(), Value::PinAf([1, 6, 1]));
            set(s, store::IMU_SDA_PIN.key(), Value::PinAf([1, 7, 1]));
            s.set(store::IMU_I2C, 0).unwrap();
        };
        let l = with(bus);
        assert_eq!((l.imu, l.imu_model), (None, 0));
        let l = with(|s| {
            bus(s);
            s.set(store::IMU_MODEL, 2).unwrap();
        });
        assert_eq!(l.imu.map(|b| (b.model, b.i2c)), Some((2, 0)));
        let l = with(|s| {
            bus(s);
            s.set(store::IMU_MODEL, 2).unwrap();
            s.erase_value(store::IMU_I2C.key()).unwrap();
        });
        assert_eq!((l.imu, l.imu_model), (None, 2));
    }

    #[test]
    fn a_record_of_another_type_reads_as_not_stored_and_the_newest_wins() {
        let l = with(|s| {
            // An older image's packed pin byte under the button's id.
            s.set(store::Field::<u8>::new(store::BOARD_BUTTON.id(), 0), 0x05)
                .unwrap();
        });
        assert_eq!(l.inputs[0], None);
        let l = with(|s| {
            set(s, store::PAD_A.key(), Value::Pin([0, 11]));
            set(s, store::PAD_A.key(), Value::Pin([0, 12]));
        });
        assert_eq!(l.inputs[1], Some([0, 12]));
    }

    /// The pass reads exactly the registry's pin keys, the groups' instance and remap keys and the
    /// IMU's switch, at every motor index below `COUNT` and every link, and its filter
    /// ([`KEY_LIST`]) is those keys. `phase_c` is read only where the image drives two motors.
    #[test]
    fn the_pass_reads_every_pin_instance_and_remap_key() {
        let pins = store::REGISTRY
            .iter()
            .filter(|e| matches!(e.kind, Type::Pin | Type::PinAf | Type::PinAdc))
            .filter(|e| e.effect == store::Effect::Boot)
            .filter(|e| COUNT > 1 || e.field_id != store::MOTOR_PHASE_C.id())
            .map(|e| e.key());
        let others = [
            store::MOTOR_GATE_TIMER.at(0).key(),
            store::MOTOR_GATE_REMAP.at(0).key(),
            store::IMU_I2C.key(),
            store::IMU_REMAP.key(),
            store::IMU_MODEL.key(),
            store::LINK_USART.at(0).key(),
            store::LINK_REMAP.at(0).key(),
            store::LINK_USART.at(1).key(),
            store::LINK_REMAP.at(1).key(),
        ];
        let base: Vec<Key> = pins.chain(others).collect();
        let mut want = Vec::new();
        for k in &base {
            // A wheel key is read at every motor's index.
            if WHEEL_KEYS.iter().any(|(w, _)| w.field_id == k.field_id) {
                for m in 0..COUNT as u8 {
                    want.push(Key {
                        field_id: k.field_id,
                        index: m,
                    });
                }
            } else {
                want.push(*k);
            }
        }
        for k in &want {
            assert!(KEYS.iter().any(|(key, _)| key == k), "{k:?} not in KEYS");
            assert!(KEY_LIST.contains(k), "{k:?} not in the pass's filter");
        }
        assert_eq!(
            KEY_LIST.len(),
            want.len() + 2 * COUNT,
            "only these, the dead time and the calibration state"
        );
        for m in 0..COUNT as u8 {
            assert!(KEY_LIST.contains(&store::MOTOR_DEAD_TIME.at(m).key()));
            assert!(KEY_LIST.contains(&store::MOTOR_CAL_STATE.at(m).key()));
        }
        for (k, t) in KEYS {
            let d = store::lookup_key(k).unwrap();
            assert_eq!(d.kind, t, "{k:?}");
        }
    }
}
