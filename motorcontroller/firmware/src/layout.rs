//! Each subsystem reads its own layout fields (`specs/configured-links.md` section 3).
//!
//! There is no whole-layout validator. A bring-up reads the fields of the function it brings up,
//! checks only what it cannot run without, and asks the HAL for the rest; a failure leaves that
//! function absent and adds an entry to the boot status record ([`StatusRecord`]), and never costs
//! another function or the boot. Nothing is compared across functions: a pin given to two
//! functions is configured twice and the later bring-up owns it.
//!
//! What a bring-up still checks here, because it cannot run otherwise:
//!
//! - **every stored byte decodes to a pin** ([`Reason::NotAPin`]);
//! - **a group needs all its pins** ([`group`]): none set is silent and the function is absent;
//!   some set is recorded as [`Reason::Incomplete`];
//! - **the chip routes it**: the gate timer behind six gate pins and the ADC channel behind an
//!   analog pin ([`read_motor`], [`read_adc_input`]), answered by the HAL through the closures the
//!   caller passes ([`Reason::NotRouted`]); the I2C instance and the USART are asked where the
//!   peripheral is built, in `main.rs`;
//! - **the dead-time floor**, in the motor's gate group ([`Reason::DeadTime`]).
//!
//! This half is pure and host-tested over a real `Store`; the HAL calls that follow it are in
//! `main.rs`'s target-only bring-up.

// The callers are the target-only boot in `main.rs`; the host build compiles this half for its
// tests alone.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use board::status::{Reason, StatusRecord};
use board::{Parsed, Pin, DEAD_TIME_MIN_DTG};
use store::{Field, Flash, Store};

/// One motor's hall inputs (`hall.a`/`b`/`c`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HallSet {
    pub a: Pin,
    pub b: Pin,
    pub c: Pin,
}

/// One motor's gate group: the six pins, the dead time it is held to, and the advanced timer the
/// HAL's gate-set query derived from the pins (0 = TIMER0, 1 = TIMER7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GateSet {
    pub hi: [Pin; 3],
    pub lo: [Pin; 3],
    pub dead_time: u8,
    pub timer: u8,
}

/// An analog input and the ADC channel behind its pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcInput {
    pub pin: Pin,
    pub channel: u8,
}

/// One motor's phase-current sense pair: the two pins and their ADC channels, in injected-rank
/// order (0 = phase A, 1 = phase B).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseCurrentSet {
    pub pins: [Pin; 2],
    pub channels: [u8; 2],
}

/// What the motor bring-up takes: the groups that came up (absent ones are `None`) and the motor's
/// carried config facts. The facts are carried, not checked: `direction` and `align_offset` feed
/// every arm, `current_cal` the limit conversion, which clamps it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct MotorPlan {
    pub halls: Option<HallSet>,
    pub gates: Option<GateSet>,
    /// Drive direction: `false` = Forward, `true` = Reverse.
    pub direction: bool,
    /// Hall-to-phase align offset (0..5 sectors, taken mod 6 by the commutation crate).
    pub align_offset: u8,
    /// The current-sense calibration, stock counts per amp (`store::MOTOR_CURRENT_CAL`).
    pub current_cal: u16,
    /// The phase-current sense group, or `None` where the motor has no current sense.
    pub phase_current: Option<PhaseCurrentSet>,
}

/// The IMU's staged pin pair and model, when `imu.model` asks for an IMU and both pins are set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuPins {
    pub scl: Pin,
    pub sda: Pin,
    pub model: u8,
}

/// Read a group of pin fields. All set: the pins. None set: `None`, silently (the function is
/// absent). Some set: `None`, recorded as [`Reason::Incomplete`] with the first unset field's id. A
/// byte that is no pin encoding: `None`, recorded as [`Reason::NotAPin`] with the byte.
///
/// Every entry names the group by its FIRST field and that field's index, which is how the status
/// record names a function (`specs/configured-links.md` section 5).
pub fn group<F: Flash, const N: usize>(
    s: &Store<F>,
    fields: [Field<u8>; N],
    status: &mut StatusRecord,
) -> Option<[Pin; N]> {
    let head = *fields.first()?;
    let (id, index) = (head.id(), head.key().index);
    let mut pins: [Option<Pin>; N] = [None; N];
    for (slot, f) in pins.iter_mut().zip(fields) {
        let raw = s.get(f);
        match Pin::parse(raw) {
            Parsed::Invalid => {
                status.record(id, index, Reason::NotAPin, raw);
                return None;
            }
            Parsed::Absent => {}
            Parsed::Valid(p) => *slot = Some(p),
        }
    }
    let set = pins.iter().flatten().count();
    if set == 0 {
        return None;
    }
    if let Some((_, unset)) = pins.iter().zip(fields).find(|(p, _)| p.is_none()) {
        status.record(id, index, Reason::Incomplete, unset.id());
        return None;
    }
    // Every slot is set here; the first one stands in for the `None` no slot holds.
    let first = pins.iter().flatten().next().copied()?;
    Some(pins.map(|p| p.unwrap_or(first)))
}

/// Read one pin field: [`group`] of one.
pub fn pin<F: Flash>(s: &Store<F>, field: Field<u8>, status: &mut StatusRecord) -> Option<Pin> {
    group(s, [field], status).map(|[p]| p)
}

/// Read an analog input (`board.battery.pin`): the pin, and the ADC channel `adc_channel` answers
/// for it. No channel: `None`, recorded as [`Reason::NotRouted`] with the pin.
pub fn read_adc_input<F: Flash>(
    s: &Store<F>,
    field: Field<u8>,
    adc_channel: impl Fn(u8) -> Option<u8>,
    status: &mut StatusRecord,
) -> Option<AdcInput> {
    let p = pin(s, field, status)?;
    match adc_channel(p.packed()) {
        Some(channel) => Some(AdcInput { pin: p, channel }),
        None => {
            status.record(field.id(), field.key().index, Reason::NotRouted, p.packed());
            None
        }
    }
}

/// Read motor `m`'s layout: the hall group, the gate group held to the dead-time floor and routed
/// to an advanced timer (`gate_timer`), and the phase-current pair, fitted exactly when both its
/// pins are set and routed to ADC channels (`adc_channel`).
///
/// A group that is absent or refused is `None` in the plan, and the motor bring-up then leaves the
/// motor absent. A gate group below [`DEAD_TIME_MIN_DTG`] is refused: below the floor both FETs of a
/// leg conduct through the transition on any board.
pub fn read_motor<F: Flash>(
    s: &Store<F>,
    m: u8,
    gate_timer: impl Fn([u8; 3], [u8; 3]) -> Option<u8>,
    adc_channel: impl Fn(u8) -> Option<u8>,
    status: &mut StatusRecord,
) -> MotorPlan {
    let halls = group(
        s,
        [
            store::MOTOR_HALL_A.at(m),
            store::MOTOR_HALL_B.at(m),
            store::MOTOR_HALL_C.at(m),
        ],
        status,
    )
    .map(|[a, b, c]| HallSet { a, b, c });

    let gate_id = store::MOTOR_GATE_HI_A.id();
    let gates = group(
        s,
        [
            store::MOTOR_GATE_HI_A.at(m),
            store::MOTOR_GATE_HI_B.at(m),
            store::MOTOR_GATE_HI_C.at(m),
            store::MOTOR_GATE_LO_A.at(m),
            store::MOTOR_GATE_LO_B.at(m),
            store::MOTOR_GATE_LO_C.at(m),
        ],
        status,
    )
    .and_then(|[ha, hb, hc, la, lb, lc]| {
        let dead_time = s.get(store::MOTOR_DEAD_TIME.at(m));
        if dead_time < DEAD_TIME_MIN_DTG {
            status.record(gate_id, m, Reason::DeadTime, dead_time);
            return None;
        }
        let (hi, lo) = ([ha, hb, hc], [la, lb, lc]);
        match gate_timer(hi.map(|p| p.packed()), lo.map(|p| p.packed())) {
            Some(timer) => Some(GateSet {
                hi,
                lo,
                dead_time,
                timer,
            }),
            None => {
                status.record(gate_id, m, Reason::NotRouted, ha.packed());
                None
            }
        }
    });

    let phase_current = read_phase_current(s, m, adc_channel, status);

    MotorPlan {
        halls,
        gates,
        direction: s.get(store::MOTOR_DIRECTION.at(m)) != 0,
        align_offset: s.get(store::MOTOR_ALIGN_OFFSET.at(m)),
        current_cal: s.get(store::MOTOR_CURRENT_CAL.at(m)),
        phase_current,
    }
}

/// The phase-current pair of motor `m`, a group like the others (`specs/machine-settings.md`
/// section 3.3): phase sense is fitted exactly when both pins are set, neither is no sense and
/// nothing is recorded, one of the two is recorded as incomplete, and each pin needs an ADC
/// channel.
fn read_phase_current<F: Flash>(
    s: &Store<F>,
    m: u8,
    adc_channel: impl Fn(u8) -> Option<u8>,
    status: &mut StatusRecord,
) -> Option<PhaseCurrentSet> {
    let fields = [store::MOTOR_PHASE_A.at(m), store::MOTOR_PHASE_B.at(m)];
    let id = store::MOTOR_PHASE_A.id();
    let pins = group(s, fields, status)?;
    let mut channels = [0u8; 2];
    for (c, p) in channels.iter_mut().zip(pins) {
        match adc_channel(p.packed()) {
            Some(ch) => *c = ch,
            None => {
                status.record(id, m, Reason::NotRouted, p.packed());
                return None;
            }
        }
    }
    Some(PhaseCurrentSet { pins, channels })
}

/// Read the IMU group: `imu.model` is its enable (0 = no IMU fitted, the pins inert and not read),
/// and an enabled group needs both pins. Whether the pair routes to an I2C instance is asked where
/// the bus is built.
pub fn read_imu<F: Flash>(s: &Store<F>, status: &mut StatusRecord) -> Option<ImuPins> {
    let model = s.get(store::IMU_MODEL);
    if model == 0 {
        return None;
    }
    let fields = [store::IMU_SCL_PIN, store::IMU_SDA_PIN];
    let Some([scl, sda]) = group(s, fields, status) else {
        if fields.iter().all(|f| s.get(*f) == board::ABSENT) {
            status.record(
                store::IMU_SCL_PIN.id(),
                0,
                Reason::Incomplete,
                store::IMU_SCL_PIN.id(),
            );
        }
        return None;
    };
    Some(ImuPins { scl, sda, model })
}

/// Read link `i`'s pin pair (`links[i].tx`, `links[i].rx`): both set is a configured link, neither is no
/// link (its pins are not touched), one of the two is recorded as incomplete.
pub fn read_link<F: Flash>(s: &Store<F>, i: u8, status: &mut StatusRecord) -> Option<[Pin; 2]> {
    group(s, [store::LINK_TX.at(i), store::LINK_RX.at(i)], status)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_flash::TestFlash;
    use board::status::{decode, Entry};
    use std::vec::Vec;

    /// Run `f` over a fresh store staged by `stage`, returning what `f` returned and the entries
    /// it recorded.
    fn with<T>(
        stage: impl FnOnce(&mut Store<TestFlash>),
        f: impl FnOnce(&Store<TestFlash>, &mut StatusRecord) -> T,
    ) -> (T, Vec<Entry>) {
        let mut flash = TestFlash::erased();
        let mut s = Store::mount(&mut flash).unwrap();
        stage(&mut s);
        let mut status = StatusRecord::new();
        let out = f(&s, &mut status);
        let entries = decode(status.entries()).collect();
        (out, entries)
    }

    fn entry(field: u8, index: u8, reason: Reason, detail: u8) -> Entry {
        Entry {
            field,
            index,
            reason: reason as u8,
            detail,
        }
    }

    /// The bench 6-FET master's motor 0: halls PC13/PA1/PC14, gates PA8/9/10 + PB13/14/15, dead
    /// time 25, phase sense PB0/PA0.
    fn stage_motor(s: &mut Store<TestFlash>) {
        for (f, v) in [
            (store::MOTOR_HALL_A, 0x2D),
            (store::MOTOR_HALL_B, 0x01),
            (store::MOTOR_HALL_C, 0x2E),
            (store::MOTOR_GATE_HI_A, 0x08),
            (store::MOTOR_GATE_HI_B, 0x09),
            (store::MOTOR_GATE_HI_C, 0x0A),
            (store::MOTOR_GATE_LO_A, 0x1D),
            (store::MOTOR_GATE_LO_B, 0x1E),
            (store::MOTOR_GATE_LO_C, 0x1F),
            (store::MOTOR_DEAD_TIME, 25),
            (store::MOTOR_PHASE_A, 0x10),
            (store::MOTOR_PHASE_B, 0x00),
        ] {
            s.set(f.at(0), v).unwrap();
        }
    }

    /// The fleet's routing answers, as test data: the six bench gate pins are TIMER0's, PB0 and PA0
    /// are ADC channels 8 and 0, PA4 is channel 4.
    fn gate_timer(hi: [u8; 3], lo: [u8; 3]) -> Option<u8> {
        (hi == [0x08, 0x09, 0x0A] && lo == [0x1D, 0x1E, 0x1F]).then_some(0)
    }
    fn adc(p: u8) -> Option<u8> {
        match p {
            0x10 => Some(8),
            0x00 => Some(0),
            0x04 => Some(4),
            _ => None,
        }
    }

    #[test]
    fn an_unset_group_is_absent_and_silent() {
        let (plan, entries) = with(|_| {}, |s, st| read_motor(s, 0, gate_timer, adc, st));
        assert_eq!(plan.halls, None);
        assert_eq!(plan.gates, None);
        assert_eq!(plan.phase_current, None);
        assert_eq!(entries, []);
        let (link, entries) = with(|_| {}, |s, st| read_link(s, 1, st));
        assert_eq!(link, None);
        assert_eq!(
            entries,
            [],
            "an unset link touches nothing and says nothing"
        );
    }

    #[test]
    fn a_configured_motor_reads_whole() {
        let (plan, entries) = with(stage_motor, |s, st| read_motor(s, 0, gate_timer, adc, st));
        assert_eq!(entries, []);
        let gates = plan.gates.unwrap();
        assert_eq!((gates.dead_time, gates.timer), (25, 0));
        assert_eq!(gates.lo[2].packed(), 0x1F);
        assert_eq!(plan.halls.unwrap().b.packed(), 0x01);
        assert_eq!(plan.phase_current.unwrap().channels, [8, 0]);
    }

    #[test]
    fn a_partial_group_is_recorded_with_its_first_unset_field() {
        let (plan, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_HALL_B.at(0), board::ABSENT).unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert_eq!(plan.halls, None);
        assert!(plan.gates.is_some(), "a hall problem costs only the halls");
        assert_eq!(
            entries,
            [entry(
                store::MOTOR_HALL_A.id(),
                0,
                Reason::Incomplete,
                store::MOTOR_HALL_B.id()
            )]
        );
    }

    #[test]
    fn a_dead_time_below_the_floor_is_recorded_and_the_gates_are_absent() {
        for dt in [0, 1, 10, DEAD_TIME_MIN_DTG - 1] {
            let (plan, entries) = with(
                |s| {
                    stage_motor(s);
                    s.set(store::MOTOR_DEAD_TIME.at(0), dt).unwrap();
                },
                |s, st| read_motor(s, 0, gate_timer, adc, st),
            );
            assert_eq!(plan.gates, None, "dead time {dt}");
            assert_eq!(
                entries,
                [entry(store::MOTOR_GATE_HI_A.id(), 0, Reason::DeadTime, dt)]
            );
        }
        let (plan, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_DEAD_TIME.at(0), DEAD_TIME_MIN_DTG)
                    .unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert!(plan.gates.is_some(), "the floor itself is allowed");
        assert_eq!(entries, []);
    }

    #[test]
    fn a_gate_set_the_chip_does_not_route_is_recorded() {
        let (plan, entries) = with(stage_motor, |s, st| read_motor(s, 0, |_, _| None, adc, st));
        assert_eq!(plan.gates, None);
        assert_eq!(
            entries,
            [entry(
                store::MOTOR_GATE_HI_A.id(),
                0,
                Reason::NotRouted,
                0x08
            )]
        );
    }

    #[test]
    fn a_byte_that_is_no_pin_is_recorded_with_the_byte() {
        let (plan, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_GATE_LO_B.at(0), 0x4C).unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert_eq!(plan.gates, None);
        assert_eq!(
            entries,
            [entry(store::MOTOR_GATE_HI_A.id(), 0, Reason::NotAPin, 0x4C)]
        );
    }

    #[test]
    fn the_phase_pair_is_a_group_of_two_pins() {
        // Neither pin: no current sense, nothing to say.
        let (plan, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_PHASE_A.at(0), board::ABSENT).unwrap();
                s.set(store::MOTOR_PHASE_B.at(0), board::ABSENT).unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert_eq!(plan.phase_current, None);
        assert_eq!(entries, []);
        // One of the two: incomplete at the unset one.
        let a = store::MOTOR_PHASE_A.id();
        let (plan, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_PHASE_B.at(0), board::ABSENT).unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert_eq!(plan.phase_current, None);
        assert_eq!(
            entries,
            [entry(a, 0, Reason::Incomplete, store::MOTOR_PHASE_B.id())]
        );
        // A phase pin with no ADC channel.
        let (_, entries) = with(
            |s| {
                stage_motor(s);
                s.set(store::MOTOR_PHASE_B.at(0), 0x1A).unwrap();
            },
            |s, st| read_motor(s, 0, gate_timer, adc, st),
        );
        assert_eq!(entries, [entry(a, 0, Reason::NotRouted, 0x1A)]);
    }

    #[test]
    fn the_battery_pin_needs_an_adc_channel() {
        let (v, entries) = with(
            |s| s.set(store::BOARD_BATTERY_PIN, 0x04).unwrap(),
            |s, st| read_adc_input(s, store::BOARD_BATTERY_PIN, adc, st),
        );
        assert_eq!(v.map(|v| v.channel), Some(4));
        assert_eq!(entries, []);
        let (v, entries) = with(
            |s| s.set(store::BOARD_BATTERY_PIN, 0x1C).unwrap(),
            |s, st| read_adc_input(s, store::BOARD_BATTERY_PIN, adc, st),
        );
        assert_eq!(v, None);
        assert_eq!(
            entries,
            [entry(
                store::BOARD_BATTERY_PIN.id(),
                0,
                Reason::NotRouted,
                0x1C
            )]
        );
    }

    #[test]
    fn the_imu_group_is_enabled_by_its_model() {
        // Model 0: the pins are inert whatever they hold.
        let (imu, entries) = with(|s| s.set(store::IMU_SCL_PIN, 0x16).unwrap(), read_imu);
        assert_eq!((imu, entries.len()), (None, 0));
        // A model with no pins: incomplete at imu.scl.
        let scl = store::IMU_SCL_PIN.id();
        let (imu, entries) = with(|s| s.set(store::IMU_MODEL, 2).unwrap(), read_imu);
        assert_eq!(imu, None);
        assert_eq!(entries, [entry(scl, 0, Reason::Incomplete, scl)]);
        // A model with one pin: incomplete at the missing one.
        let (_, entries) = with(
            |s| {
                s.set(store::IMU_MODEL, 2).unwrap();
                s.set(store::IMU_SCL_PIN, 0x16).unwrap();
            },
            read_imu,
        );
        assert_eq!(
            entries,
            [entry(scl, 0, Reason::Incomplete, store::IMU_SDA_PIN.id())]
        );
        let (imu, entries) = with(
            |s| {
                s.set(store::IMU_MODEL, 2).unwrap();
                s.set(store::IMU_SCL_PIN, 0x1A).unwrap();
                s.set(store::IMU_SDA_PIN, 0x1B).unwrap();
            },
            read_imu,
        );
        let imu = imu.unwrap();
        assert_eq!(
            (imu.scl.packed(), imu.sda.packed(), imu.model),
            (0x1A, 0x1B, 2)
        );
        assert_eq!(entries, []);
    }

    /// The link loop's reading half (`specs/configured-links.md` section 4): each link is read by
    /// its own index, one of two pins set is recorded against that link, and a problem on one link
    /// costs nothing on the other.
    #[test]
    fn each_link_is_read_on_its_own() {
        let (links, entries) = with(
            |s| {
                s.set(store::LINK_TX.at(0), 0x02).unwrap();
                s.set(store::LINK_RX.at(0), 0x03).unwrap();
                s.set(store::LINK_TX.at(1), 0x16).unwrap();
            },
            |s, st| [read_link(s, 0, st), read_link(s, 1, st)],
        );
        assert_eq!(links[0].map(|p| p.map(|p| p.packed())), Some([0x02, 0x03]));
        assert_eq!(links[1], None);
        assert_eq!(
            entries,
            [entry(
                store::LINK_TX.id(),
                1,
                Reason::Incomplete,
                store::LINK_RX.id()
            )]
        );
    }

    #[test]
    fn a_single_pin_field_is_a_group_of_one() {
        let (p, entries) = with(
            |s| s.set(store::PAD_A, 0x5F).unwrap(),
            |s, st| pin(s, store::PAD_A, st),
        );
        assert_eq!(p.map(|p| p.packed()), Some(0x5F));
        assert_eq!(entries, []);
        let (p, entries) = with(
            |s| s.set(store::PAD_A, 0x40).unwrap(),
            |s, st| pin(s, store::PAD_A, st),
        );
        assert_eq!(p, None);
        assert_eq!(
            entries,
            [entry(store::PAD_A.id(), 0, Reason::NotAPin, 0x40)]
        );
        // The registered default of an unstaged field is read like any value: PB12 for the latch.
        let (p, _) = with(|_| {}, |s, st| pin(s, store::BOARD_SELF_HOLD, st));
        assert_eq!(p.map(|p| p.packed()), Some(0x1C));
    }
}
