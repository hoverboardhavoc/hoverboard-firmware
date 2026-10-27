//! The plumbing (`specs/board-model.md`, slicing item 3): the fields -> validator read path, the
//! reserved-set computation, and the `BOARD_OBS` record layout. Nothing here touches the live
//! boot path; the boot wiring (and the real-chip `Capabilities` adapter over runtime-hal's R-CAP
//! queries) lives in `crates/firmware` (slicing item 4).

use crate::{BoardError, BoardErrorKind, BoardField, BoardFields, MotorFields};
use store::{Flash, Store};

/// Read the registered board-layout fields (`specs/board-model.md`, "The field vocabulary", plus
/// the per-motor carried config facts folded in by `specs/motor-integration.md`: direction,
/// align_offset, current_sense + its calibration + the phase-current pins) from the store, through the registry defaults (the defaults'
/// single owner: an absent key
/// reads its registered default, so a blank board yields the benign fleet plan and absent motor
/// groups). Per-motor fields read via `Key.index`.
pub fn read_fields<F: Flash>(s: &Store<F>) -> BoardFields {
    let motor = |m: u8| MotorFields {
        hall_a: s.get(store::MOTOR_HALL_A.at(m)),
        hall_b: s.get(store::MOTOR_HALL_B.at(m)),
        hall_c: s.get(store::MOTOR_HALL_C.at(m)),
        gate_hi_a: s.get(store::MOTOR_GATE_HI_A.at(m)),
        gate_hi_b: s.get(store::MOTOR_GATE_HI_B.at(m)),
        gate_hi_c: s.get(store::MOTOR_GATE_HI_C.at(m)),
        gate_lo_a: s.get(store::MOTOR_GATE_LO_A.at(m)),
        gate_lo_b: s.get(store::MOTOR_GATE_LO_B.at(m)),
        gate_lo_c: s.get(store::MOTOR_GATE_LO_C.at(m)),
        dead_time: s.get(store::MOTOR_DEAD_TIME.at(m)),
        direction: s.get(store::MOTOR_DIRECTION.at(m)),
        align_offset: s.get(store::MOTOR_ALIGN_OFFSET.at(m)),
        current_sense: s.get(store::MOTOR_CURRENT_SENSE.at(m)),
        current_cal: s.get(store::MOTOR_CURRENT_CAL.at(m)),
        phase_a: s.get(store::MOTOR_PHASE_A.at(m)),
        phase_b: s.get(store::MOTOR_PHASE_B.at(m)),
    };
    BoardFields {
        self_hold: s.get(store::BOARD_SELF_HOLD),
        vbatt: s.get(store::BOARD_VBATT),
        buzzer: s.get(store::BOARD_BUZZER),
        led_green: s.get(store::LED_GREEN),
        led_orange: s.get(store::LED_ORANGE),
        led_red: s.get(store::LED_RED),
        pad_a: s.get(store::PAD_A),
        pad_b: s.get(store::PAD_B),
        button: s.get(store::BOARD_BUTTON),
        imu_scl: s.get(store::IMU_SCL_PIN),
        imu_sda: s.get(store::IMU_SDA_PIN),
        imu_model: s.get(store::IMU_MODEL),
        motors: [motor(0), motor(1)],
    }
}

/// One safe-USART allowlist entry as the FIRMWARE owns it (`specs/l3.md`; the compiled constant
/// stays the firmware's, passed in).
#[derive(Clone, Copy, Debug)]
pub struct AllowlistPort {
    /// The port's bit in the persisted `LINK_SET` mask: the identity of this WIRING, unique per
    /// allowlist entry and stable for the life of a board's config.
    pub link_set_bit: u8,
    /// The port's two pins, packed.
    pub pins: [u8; 2],
    /// Can the HAL actually bring this entry up, on THIS chip, this boot?
    ///
    /// The HAL pin model's own per-boot answer (the caller asks it: does this pin pair map to a
    /// USART instance on the detected family, and can that instance receive), never a compiled
    /// literal and never a family test written out here.
    pub routable: bool,
}

/// The SWD pins (PA13/PA14), always reserved on every fleet part.
pub const SWD_PINS: [u8; 2] = [0x0D, 0x0E];

/// The computed reserved set [`crate::validate`] consumes (fixed capacity: SWD + up to 7
/// allowlist ports).
#[derive(Clone, Copy, Debug)]
pub struct ReservedSet {
    pins: [u8; 16],
    len: usize,
}

impl ReservedSet {
    /// The reserved pins, packed, for `validate`'s `reserved` argument.
    pub fn as_slice(&self) -> &[u8] {
        &self.pins[..self.len]
    }
}

/// Is this allowlist port one the link layer may claim its pins for on THIS boot?
///
/// The predicate [`reserved_set`] answers through. Two conditions:
///
/// - **`LINK_SET` says so.** `link_set == 0` means UNCONFIGURED: the board will probe every
///   allowlisted port, so every one is a claimant. A nonzero `link_set` means configured, and only
///   the ports whose bit IS set are live link ports; the clear-bit ports are freed for other
///   functions (an IMU on the freed port's pins is the motivating case, both families).
/// - **The HAL can bring it up.** A port this silicon cannot route (`routable` false) will never be
///   configured whatever the mask says, so it claims nothing. Reserving its pins anyway would
///   reserve them against a phantom: a probe that cannot happen.
fn claims_pins(port: &AllowlistPort, link_set: u8) -> bool {
    port.routable && (link_set == 0 || (link_set & (1 << port.link_set_bit)) != 0)
}

/// Compute the validator's reserved set (`specs/board-model.md`, check 3): the pins the link layer
/// may claim this boot ([`claims_pins`]), plus SWD.
pub fn reserved_set(allowlist: &[AllowlistPort], link_set: u8) -> ReservedSet {
    let mut out = ReservedSet {
        pins: [0; 16],
        len: 0,
    };
    for p in SWD_PINS {
        out.pins[out.len] = p;
        out.len += 1;
    }
    for port in allowlist {
        if claims_pins(port, link_set) {
            for p in port.pins {
                out.pins[out.len] = p;
                out.len += 1;
            }
        }
    }
    out
}

/// The `BOARD_OBS` magic (`"BRDV"` little-endian).
pub const BOARD_OBS_MAGIC: u32 = 0x5644_5242;

/// Result code: the layout validated.
pub const OBS_OK: u8 = 0;

/// The `BOARD_OBS` RAM record (`specs/board-model.md`, "The apply contract"): the validator
/// outcome, readable over the SWD mailbox path. This crate defines the LAYOUT and the
/// constructors; `crates/firmware` places it in RAM (the `#[no_mangle]` `BOARD_OBS` static,
/// written once at boot).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoardObs {
    /// [`BOARD_OBS_MAGIC`].
    pub magic: u32,
    /// [`OBS_OK`] on success, else the failure kind's code (1..=11, see [`BoardObs::failure`]).
    pub result: u8,
    /// The offending field's REGISTRY id (`store`'s field ids; 0 on success).
    pub field_id: u8,
    /// The offending field's `Key.index` (the motor index; 0 for singletons and on success).
    /// One exception: an IMU frame refusal (result 11) leaves this 0 even though `IMU_AXIS_SIGN`
    /// is an indexed field; the refused triple is identified by [`BoardObs::detail`] instead.
    pub index: u8,
    /// The power-latch pin this boot ACTUALLY drove high, packed, or [`crate::ABSENT`] (`0xFF`)
    /// if none was driven.
    ///
    /// Present on the success record as well as the failure one, and deliberately so: the whole
    /// point of the field is that a bench read can tell the pin the board DROVE from the pin it
    /// was told to drive, without inferring one from the other. A staged `board.self_hold` that
    /// does not appear here did not reach hardware.
    pub self_hold: u8,
    /// Kind-specific detail: the packed pin byte (or the raw byte for a bad encoding); 0 where
    /// no pin is involved. For an IMU frame refusal (result 11) it is the refused triple's first
    /// `IMU_AXIS_SIGN` index instead: 0 = accel, 3 = gyro (and 0 for a role refusal, which names
    /// `IMU_AXIS_ROLE` in `field_id`).
    pub detail: u32,
}

/// The registry id behind a [`BoardField`] (via the store handles, their single owner).
fn field_id(f: BoardField) -> u8 {
    match f {
        BoardField::SelfHold => store::BOARD_SELF_HOLD.id(),
        BoardField::Vbatt => store::BOARD_VBATT.id(),
        BoardField::Buzzer => store::BOARD_BUZZER.id(),
        BoardField::LedGreen => store::LED_GREEN.id(),
        BoardField::LedOrange => store::LED_ORANGE.id(),
        BoardField::LedRed => store::LED_RED.id(),
        BoardField::PadA => store::PAD_A.id(),
        BoardField::PadB => store::PAD_B.id(),
        BoardField::Button => store::BOARD_BUTTON.id(),
        BoardField::ImuScl => store::IMU_SCL_PIN.id(),
        BoardField::ImuSda => store::IMU_SDA_PIN.id(),
        BoardField::ImuModel => store::IMU_MODEL.id(),
        BoardField::ImuAxisSign => store::IMU_AXIS_SIGN.id(),
        BoardField::ImuAxisRole => store::IMU_AXIS_ROLE.id(),
        BoardField::HallA => store::MOTOR_HALL_A.id(),
        BoardField::HallB => store::MOTOR_HALL_B.id(),
        BoardField::HallC => store::MOTOR_HALL_C.id(),
        BoardField::GateHiA => store::MOTOR_GATE_HI_A.id(),
        BoardField::GateHiB => store::MOTOR_GATE_HI_B.id(),
        BoardField::GateHiC => store::MOTOR_GATE_HI_C.id(),
        BoardField::GateLoA => store::MOTOR_GATE_LO_A.id(),
        BoardField::GateLoB => store::MOTOR_GATE_LO_B.id(),
        BoardField::GateLoC => store::MOTOR_GATE_LO_C.id(),
        BoardField::DeadTime => store::MOTOR_DEAD_TIME.id(),
        BoardField::PhaseA => store::MOTOR_PHASE_A.id(),
        BoardField::PhaseB => store::MOTOR_PHASE_B.id(),
        BoardField::CurrentSense => store::MOTOR_CURRENT_SENSE.id(),
    }
}

impl BoardObs {
    /// The success record: the layout validated, the plan is in force. `self_hold` is the packed
    /// latch pin the boot drove (or [`crate::ABSENT`]).
    pub fn success(self_hold: u8) -> Self {
        BoardObs {
            magic: BOARD_OBS_MAGIC,
            result: OBS_OK,
            field_id: 0,
            index: 0,
            self_hold,
            detail: 0,
        }
    }

    /// The failure record: the first validator failure, naming the offending field by its
    /// REGISTRY id + index and carrying the kind-specific detail. `self_hold` is the packed latch
    /// pin the boot drove (or [`crate::ABSENT`]): a layout can fail on an unrelated field with the
    /// latch correctly up, which is the state that keeps a battery board reachable.
    pub fn failure(err: &BoardError, self_hold: u8) -> Self {
        let (result, detail) = match err.kind {
            BoardErrorKind::BadEncoding(raw) => (1, raw as u32),
            BoardErrorKind::IncompleteGroup => (2, 0),
            BoardErrorKind::DeadTimeBelowFloor => (3, 0),
            BoardErrorKind::DuplicatePin(p) => (4, p.packed() as u32),
            BoardErrorKind::ReservedPin(p) => (5, p.packed() as u32),
            BoardErrorKind::UnknownPin(p) => (6, p.packed() as u32),
            BoardErrorKind::GateCapableMisused(p) => (7, p.packed() as u32),
            BoardErrorKind::InvalidGateSet => (8, 0),
            BoardErrorKind::NotAdcCapable(p) => (9, p.packed() as u32),
            BoardErrorKind::NotI2cPair => (10, 0),
            BoardErrorKind::ImuFrame(first) => (11, first as u32),
        };
        BoardObs {
            magic: BOARD_OBS_MAGIC,
            result,
            field_id: field_id(err.field.field),
            index: err.field.motor.unwrap_or(0),
            self_hold,
            detail,
        }
    }
}
