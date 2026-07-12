//! Host tests, slice 1 (`specs/board-model.md`, "The boot validator" checks 1-3).

extern crate std;

use super::*;

/// The compiled reserved set the firmware will pass: the safe-USART allowlist pins (PB6/PB7,
/// PA2/PA3, PB10/PB11; `specs/l3.md`) + SWD (PA13/PA14). Test data mirroring the caller's fact.
const RESERVED: &[u8] = &[0x16, 0x17, 0x02, 0x03, 0x1A, 0x1B, 0x0D, 0x0E];
/// The compiled pre-mount self-hold assert pin (PB12).
const BOOT_SELF_HOLD: Option<u8> = Some(0x1C);

/// The registry's benign fleet defaults, as the slice-3 plumbing would read them off a blank
/// board (mirrored here as TEST DATA; the store registry owns the real defaults).
fn blank_board() -> BoardFields {
    BoardFields {
        self_hold: 0x1C,  // PB12
        vbatt: 0x04,      // PA4
        buzzer: 0x19,     // PB9
        led_green: 0x13,  // PB3
        led_orange: 0x0F, // PA15
        led_red: 0x14,    // PB4
        pad_a: 0x0B,      // PA11
        pad_b: 0x2F,      // PC15
        imu_scl: ABSENT,
        imu_sda: ABSENT,
        imu_model: 0,
        motors: [MotorFields::ABSENT; 2],
    }
}

/// The bench 6-FET preset's motor-0 wiring (the spec's preset example).
fn bench_motor0() -> MotorFields {
    MotorFields {
        hall_a: 0x2D,    // PC13
        hall_b: 0x01,    // PA1
        hall_c: 0x2E,    // PC14
        gate_hi_a: 0x08, // PA8
        gate_hi_b: 0x09, // PA9
        gate_hi_c: 0x0A, // PA10
        gate_lo_a: 0x1D, // PB13
        gate_lo_b: 0x1E, // PB14
        gate_lo_c: 0x1F, // PB15
        dead_time: 25,
    }
}

/// Unwrap a known-valid packed byte to its Pin (test helper).
fn pin(raw: u8) -> Pin {
    match Pin::parse(raw) {
        Parsed::Valid(p) => p,
        other => panic!("{raw:#04x}: {other:?}"),
    }
}

fn sref(f: BoardField) -> FieldRef {
    FieldRef {
        field: f,
        motor: None,
    }
}

fn mref(f: BoardField, m: u8) -> FieldRef {
    FieldRef {
        field: f,
        motor: Some(m),
    }
}

// ---------------------------------------------------------------------------------------------
// The packed-pin parse (encoding rules).
// ---------------------------------------------------------------------------------------------

#[test]
fn parse_accepts_the_encoding_and_rejects_the_rest() {
    // Valid ports A/B/C/D/F, all pin numbers.
    for (port, byte) in [(0u8, 0x00u8), (1, 0x1F), (2, 0x2D), (3, 0x30), (5, 0x5A)] {
        let p = match Pin::parse(byte) {
            Parsed::Valid(p) => p,
            other => panic!("byte {byte:#04x}: expected Valid, got {other:?}"),
        };
        assert_eq!(p.port(), port);
        assert_eq!(p.pin(), byte & 0x0F);
        assert_eq!(p.packed(), byte);
    }
    // 0xFF is absent, not an error.
    assert_eq!(Pin::parse(ABSENT), Parsed::Absent);
    // Port E (4) does not exist in the encoding; neither does anything above F (6..=14; 15 with
    // a non-0xF low nibble is also port 15 = invalid).
    for byte in [0x40u8, 0x4C, 0x60, 0x7F, 0x90, 0xA1, 0xE0, 0xF0, 0xFE] {
        assert_eq!(
            Pin::parse(byte),
            Parsed::Invalid,
            "byte {byte:#04x} must be encoding-invalid"
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Whole-set validation: the blank board and the configured bench board.
// ---------------------------------------------------------------------------------------------

#[test]
fn blank_board_validates_to_the_benign_plan() {
    let plan = validate(&blank_board(), RESERVED, BOOT_SELF_HOLD).unwrap();
    assert_eq!(plan.self_hold.unwrap().packed(), 0x1C);
    assert_eq!(plan.vbatt.unwrap().packed(), 0x04);
    assert_eq!(plan.buzzer.unwrap().packed(), 0x19);
    assert!(plan.imu.is_none(), "no IMU configured on a blank board");
    for m in plan.motors {
        assert_eq!(
            m,
            MotorPlan::default(),
            "motor groups absent on a blank board"
        );
    }
}

#[test]
fn all_absent_board_is_a_valid_empty_plan() {
    let fields = BoardFields {
        self_hold: ABSENT,
        vbatt: ABSENT,
        buzzer: ABSENT,
        led_green: ABSENT,
        led_orange: ABSENT,
        led_red: ABSENT,
        pad_a: ABSENT,
        pad_b: ABSENT,
        imu_scl: ABSENT,
        imu_sda: ABSENT,
        imu_model: 0,
        motors: [MotorFields::ABSENT; 2],
    };
    let plan = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap();
    assert_eq!(plan, BoardPlan::default());
}

#[test]
fn bench_preset_board_validates_fully() {
    // The standard-family IMU wiring IS PB6/PB7, which sits on the compiled allowlist; l3.md
    // frees an allowlisted pin wired to a non-link peripheral via LINK_SET, so the CALLER
    // computes the effective reserved set (allowlist minus LINK_SET-freed ports) and this crate
    // just enforces what it is handed (specs/board-model.md check 3). Both halves tested:
    // (1) with the FULL allowlist reserved (an unconfigured board), the IMU-on-PB6/PB7 layout is
    // refused at the IMU field...
    let mut fields = blank_board();
    fields.imu_scl = 0x16; // PB6
    fields.imu_sda = 0x17; // PB7
    fields.imu_model = 2;
    fields.motors[0] = bench_motor0();
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuScl));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x16)));

    // ...(2) with the caller having freed the PB6/PB7 port per LINK_SET, the same layout
    // validates fully.
    let freed: std::vec::Vec<u8> = RESERVED
        .iter()
        .copied()
        .filter(|p| *p != 0x16 && *p != 0x17)
        .collect();
    let plan = validate(&fields, &freed, BOOT_SELF_HOLD).unwrap();
    let imu = plan.imu.unwrap();
    assert_eq!(imu.model, 2);
    let m0 = plan.motors[0];
    assert_eq!(m0.halls.unwrap().a.packed(), 0x2D);
    let gates = m0.gates.unwrap();
    assert_eq!(gates.dead_time, 25);
    assert_eq!(gates.hi[2].packed(), 0x0A);
    assert_eq!(gates.lo[0].packed(), 0x1D);
    assert!(plan.motors[1].gates.is_none());
}

// ---------------------------------------------------------------------------------------------
// Check 1: parse validity named per field.
// ---------------------------------------------------------------------------------------------

#[test]
fn bad_encoding_names_the_field() {
    let mut fields = blank_board();
    fields.led_red = 0x4B; // port E: encoding-invalid
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::LedRed));
    assert_eq!(err.kind, BoardErrorKind::BadEncoding(0x4B));

    let mut fields = blank_board();
    fields.motors[1].hall_b = 0x90;
    fields.motors[1].hall_a = 0x2B;
    fields.motors[1].hall_c = 0x2C;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallB, 1));
    assert_eq!(err.kind, BoardErrorKind::BadEncoding(0x90));
}

// ---------------------------------------------------------------------------------------------
// Check 2: group completeness (halls, gates, IMU, the dead-time rule).
// ---------------------------------------------------------------------------------------------

#[test]
fn partial_hall_group_is_invalid_not_absent() {
    let mut fields = blank_board();
    fields.motors[0].hall_a = 0x2D;
    fields.motors[0].hall_c = 0x2E; // hall_b missing
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallB, 0));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
}

#[test]
fn partial_gate_group_is_invalid_not_absent() {
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].gate_lo_b = ABSENT;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::GateLoB, 0));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
}

#[test]
fn configured_gate_group_requires_nonzero_dead_time() {
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].dead_time = 0;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::DeadTime, 0));
    assert_eq!(err.kind, BoardErrorKind::MissingDeadTime);
    // Halls-only (no gates) needs no dead-time.
    let mut fields = blank_board();
    fields.motors[0].hall_a = 0x2D;
    fields.motors[0].hall_b = 0x01;
    fields.motors[0].hall_c = 0x2E;
    validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap();
}

#[test]
fn imu_group_is_all_or_none_including_the_model() {
    // Pins without a model.
    let mut fields = blank_board();
    fields.imu_scl = 0x28;
    fields.imu_sda = 0x29;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuModel));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
    // A model without pins.
    let mut fields = blank_board();
    fields.imu_model = 1;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuScl));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
    // One pin missing.
    let mut fields = blank_board();
    fields.imu_scl = 0x28;
    fields.imu_model = 1;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuSda));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
}

// ---------------------------------------------------------------------------------------------
// Check 3: duplicates + the reserved set.
// ---------------------------------------------------------------------------------------------

#[test]
fn duplicate_pin_names_the_second_claimant() {
    let mut fields = blank_board();
    fields.pad_b = fields.led_green; // PB3 twice; pad_b is validated after led_green
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadB));
    assert_eq!(err.kind, BoardErrorKind::DuplicatePin(pin(0x13)));
    // Across motors too.
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[1] = bench_motor0(); // motor 1 reuses every motor-0 pin
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallA, 1));
    assert_eq!(err.kind, BoardErrorKind::DuplicatePin(pin(0x2D)));
}

#[test]
fn reserved_pins_refuse_every_field() {
    // An allowlist pin (PA2) in a LED field.
    let mut fields = blank_board();
    fields.led_orange = 0x02;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::LedOrange));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x02)));
    // SWD (PA13) in a gate field: reserved fires (this slice checks the compiled set; the
    // gate-capability rules are the next slice).
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].gate_hi_a = 0x0D;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::GateHiA, 0));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x0D)));
}

#[test]
fn boot_self_hold_pin_is_reserved_except_for_self_hold_itself() {
    // The blank board's self_hold IS PB12: valid (the field that owns it).
    validate(&blank_board(), RESERVED, BOOT_SELF_HOLD).unwrap();
    // Another field claiming PB12 is refused.
    let mut fields = blank_board();
    fields.self_hold = ABSENT;
    fields.buzzer = 0x1C;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::Buzzer));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x1C)));
    // With no compiled boot assert (None), PB12 is an ordinary pin.
    validate(&fields, RESERVED, None).unwrap();
}

#[test]
fn first_failure_wins_in_field_order() {
    // Two failures planted: a bad encoding on an early field (vbatt) and a reserved hit on a
    // later one (pad_a). The early one is reported.
    let mut fields = blank_board();
    fields.vbatt = 0x60;
    fields.pad_a = 0x02;
    let err = validate(&fields, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::Vbatt));
    assert_eq!(err.kind, BoardErrorKind::BadEncoding(0x60));
}
