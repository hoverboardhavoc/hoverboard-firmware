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
    /// The phase-current pair with their ADC channels, `None` when not stored (no current sense).
    pub phase: Option<[Pin3; 2]>,
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
    /// Motor 0, the one a board drives.
    pub motor: Option<Motor>,
    /// The two links.
    pub links: [Option<Link>; store::LINKS as usize],
    /// The IMU.
    pub imu: Option<ImuBus>,
    /// The stored `imu.model` (0 when not stored): whether the store asks for an IMU at all.
    pub imu_model: u8,
}

/// The slots of the pass: one per key it reads, in [`KEYS`] order.
const SELF_HOLD: usize = 0;
const BATTERY: usize = 1;
const BUTTON: usize = 2;
const PAD_A: usize = 3;
const PAD_B: usize = 4;
const HALLS: usize = 5;
const GATES: usize = 8;
const GATE_TIMER: usize = 14;
const GATE_REMAP: usize = 15;
const DEAD_TIME: usize = 16;
const PHASE: usize = 17;
const SCL: usize = 19;
const SDA: usize = 20;
const I2C: usize = 21;
const IMU_REMAP: usize = 22;
/// Link `i`'s TX, RX, USART and remap are at `LINK + 4 * i`.
const LINK: usize = 23;
/// `imu.model`, after the links.
const IMU_MODEL: usize = LINK + 4 * store::LINKS as usize;
const N: usize = IMU_MODEL + 1;

/// The keys of the pass, slot by slot, and the type each must be stored as.
const KEYS: [(Key, Type); N] = [
    pin(store::BOARD_SELF_HOLD),
    pin(store::BOARD_BATTERY_PIN),
    pin(store::BOARD_BUTTON),
    pin(store::PAD_A),
    pin(store::PAD_B),
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
    pin(store::MOTOR_PHASE_A),
    pin(store::MOTOR_PHASE_B),
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

    let motor = (|| {
        Some(Motor {
            halls: [p(HALLS)?, p(HALLS + 1)?, p(HALLS + 2)?],
            gates: [
                p3(GATES)?,
                p3(GATES + 1)?,
                p3(GATES + 2)?,
                p3(GATES + 3)?,
                p3(GATES + 4)?,
                p3(GATES + 5)?,
            ],
            timer: byte(GATE_TIMER)?,
            remap: byte(GATE_REMAP).unwrap_or(0),
            dead_time: byte(DEAD_TIME).unwrap_or(0),
            phase: p3(PHASE).zip(p3(PHASE + 1)).map(|(a, b)| [a, b]),
        })
    })();
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
        motor,
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
                l.motor,
                l.links,
                l.imu,
                l.imu_model
            ),
            (None, None, [None; 3], None, [None; 2], None, 0)
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
        let m = l.motor.unwrap();
        assert_eq!(m.halls, [[2, 13], [0, 1], [2, 14]]);
        assert_eq!(m.gates[0], [0, 8, 2]);
        assert_eq!(m.gates[5], [1, 15, 2]);
        assert_eq!((m.timer, m.remap, m.dead_time), (0, 0, 28));
        assert_eq!(m.phase, Some([[0, 0, 0], [1, 0, 8]]));
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
        assert_eq!((l.motor, l.links[0], l.imu), (None, None, None));
        // A motor without the phase pair is a motor with no current sense.
        let l = with(|s| {
            stage_motor(s);
            s.erase_value(store::MOTOR_PHASE_B.key()).unwrap();
        });
        assert_eq!(l.motor.unwrap().phase, None);
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
    /// IMU's switch, at motor 0 and every link, and its filter ([`KEY_LIST`]) is those keys.
    #[test]
    fn the_pass_reads_every_pin_instance_and_remap_key() {
        let pins = store::REGISTRY
            .iter()
            .filter(|e| matches!(e.kind, Type::Pin | Type::PinAf | Type::PinAdc))
            .filter(|e| e.effect == store::Effect::Boot)
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
        let want: Vec<Key> = pins.chain(others).collect();
        for k in &want {
            assert!(KEYS.iter().any(|(key, _)| key == k), "{k:?} not in KEYS");
            assert!(KEY_LIST.contains(k), "{k:?} not in the pass's filter");
        }
        assert_eq!(
            KEY_LIST.len(),
            want.len() + 1,
            "only these and the dead time"
        );
        assert!(KEY_LIST.contains(&store::MOTOR_DEAD_TIME.at(0).key()));
        for (k, t) in KEYS {
            let d = store::lookup_key(k).unwrap();
            assert_eq!(d.kind, t, "{k:?}");
        }
    }
}
