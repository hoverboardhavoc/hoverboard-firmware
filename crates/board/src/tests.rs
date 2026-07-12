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
// Mock capability tables (the Capabilities seam's host implementations). These model the answers
// the R-CAP runtime-hal integration will provide for the fleet's parts; family differences the
// fleet actually has are modeled: the F103C8's PD0/PD1 (no port F) vs the F130C8's PF0/PF1 (no
// port D); I2C pair indices per family; the 6-FET TIMER0 gate set on both 48-pin parts, and the
// 12-FET F103RC's TIMER0 + TIMER7/TIM8 pair with the full LQFP64 port C.
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum MockChip {
    F103C8,
    F130C8,
    F103RC, // the 12-FET part
}

/// The 6-FET gate map (both 48-pin families): TIMER0 hi PA8/PA9/PA10, lo PB13/PB14/PB15.
const GATES_T0_HI: [u8; 3] = [0x08, 0x09, 0x0A];
const GATES_T0_LO: [u8; 3] = [0x1D, 0x1E, 0x1F];
/// The 12-FET second motor (F103RC): TIMER7/TIM8 hi PC6/PC7/PC8, lo PA7/PB0/PB1.
const GATES_T8_HI: [u8; 3] = [0x26, 0x27, 0x28];
const GATES_T8_LO: [u8; 3] = [0x07, 0x10, 0x11];

impl MockChip {
    fn packed(p: Pin) -> u8 {
        p.packed()
    }

    fn set_matches(hi: [Pin; 3], lo: [Pin; 3], want_hi: [u8; 3], want_lo: [u8; 3]) -> bool {
        hi.map(Self::packed) == want_hi && lo.map(Self::packed) == want_lo
    }
}

impl Capabilities for MockChip {
    fn pin_exists(&self, pin: Pin) -> bool {
        let (port, n) = (pin.port(), pin.pin());
        match self {
            // F103C8 (LQFP48): PA0-15, PB0-15, PC13-15, PD0-1, no port F.
            MockChip::F103C8 => match port {
                0 | 1 => true,
                2 => n >= 13,
                3 => n <= 1,
                _ => false,
            },
            // F130C8 (LQFP48): PA0-15, PB0-15, PC13-15, PF0-1, no port D.
            MockChip::F130C8 => match port {
                0 | 1 => true,
                2 => n >= 13,
                5 => n <= 1,
                _ => false,
            },
            // F103RC (LQFP64): PA, PB, PC full, PD0-2, no port F.
            MockChip::F103RC => match port {
                0..=2 => true,
                3 => n <= 2,
                _ => false,
            },
        }
    }

    fn gate_capable(&self, pin: Pin) -> bool {
        let b = pin.packed();
        let t0 = GATES_T0_HI.contains(&b) || GATES_T0_LO.contains(&b);
        match self {
            MockChip::F103C8 | MockChip::F130C8 => t0,
            MockChip::F103RC => t0 || GATES_T8_HI.contains(&b) || GATES_T8_LO.contains(&b),
        }
    }

    fn gate_set(&self, hi: [Pin; 3], lo: [Pin; 3]) -> Option<u8> {
        if Self::set_matches(hi, lo, GATES_T0_HI, GATES_T0_LO) {
            return Some(0); // TIMER0, every fleet part
        }
        if matches!(self, MockChip::F103RC) && Self::set_matches(hi, lo, GATES_T8_HI, GATES_T8_LO) {
            return Some(1); // TIMER7/TIM8, the 12-FET only
        }
        None
    }

    fn adc_channel(&self, pin: Pin) -> Option<u8> {
        // The standard F1-class analog map: PA0-7 = ch 0-7, PB0-1 = ch 8-9, PC0-5 = ch 10-15
        // (port C analog only where the pins exist, i.e. the RC part).
        match (pin.port(), pin.pin()) {
            (0, n) if n <= 7 => Some(n),
            (1, n) if n <= 1 => Some(8 + n),
            (2, n) if n <= 5 && self.pin_exists(pin) => Some(10 + n),
            _ => None,
        }
    }

    fn i2c_pair(&self, scl: Pin, sda: Pin) -> Option<u8> {
        let pair = (scl.packed(), sda.packed());
        match self {
            // F130: I2C0 on PB6/PB7, I2C1 on PB10/PB11.
            MockChip::F130C8 => match pair {
                (0x16, 0x17) => Some(0),
                (0x1A, 0x1B) => Some(1),
                _ => None,
            },
            // F103: I2C1 on PB6/PB7 (index 1 in the family's own numbering), I2C2 on PB10/PB11.
            MockChip::F103C8 | MockChip::F103RC => match pair {
                (0x16, 0x17) => Some(1),
                (0x1A, 0x1B) => Some(2),
                _ => None,
            },
        }
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
    let plan = validate(&blank_board(), &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap();
    assert_eq!(plan.self_hold.unwrap().packed(), 0x1C);
    assert_eq!(plan.vbatt.unwrap().pin.packed(), 0x04);
    assert_eq!(
        plan.vbatt.unwrap().channel,
        4,
        "PA4 = ADC channel 4, derived"
    );
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
    let plan = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap();
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
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuScl));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x16)));

    // ...(2) with the caller having freed the PB6/PB7 port per LINK_SET, the same layout
    // validates fully.
    let freed: std::vec::Vec<u8> = RESERVED
        .iter()
        .copied()
        .filter(|p| *p != 0x16 && *p != 0x17)
        .collect();
    let plan = validate(&fields, &MockChip::F103C8, &freed, BOOT_SELF_HOLD).unwrap();
    let imu = plan.imu.unwrap();
    assert_eq!(imu.model, 2);
    assert_eq!(imu.bus, 1, "PB6/PB7 = the F103 family's I2C1, derived");
    let m0 = plan.motors[0];
    assert_eq!(m0.halls.unwrap().a.packed(), 0x2D);
    let gates = m0.gates.unwrap();
    assert_eq!(gates.dead_time, 25);
    assert_eq!(gates.timer, 0, "the 6-FET map is TIMER0, derived");
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
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::LedRed));
    assert_eq!(err.kind, BoardErrorKind::BadEncoding(0x4B));

    let mut fields = blank_board();
    fields.motors[1].hall_b = 0x90;
    fields.motors[1].hall_a = 0x05; // PA5 (exists; encoding-valid)
    fields.motors[1].hall_c = 0x06; // PA6
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
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
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallB, 0));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
}

#[test]
fn partial_gate_group_is_invalid_not_absent() {
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].gate_lo_b = ABSENT;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::GateLoB, 0));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
}

#[test]
fn configured_gate_group_requires_nonzero_dead_time() {
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].dead_time = 0;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::DeadTime, 0));
    assert_eq!(err.kind, BoardErrorKind::MissingDeadTime);
    // Halls-only (no gates) needs no dead-time.
    let mut fields = blank_board();
    fields.motors[0].hall_a = 0x2D;
    fields.motors[0].hall_b = 0x01;
    fields.motors[0].hall_c = 0x2E;
    validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap();
}

#[test]
fn imu_group_is_all_or_none_including_the_model() {
    // Pins without a model.
    let mut fields = blank_board();
    fields.imu_scl = 0x05; // PA5 (exists; the group check fires before any capability check)
    fields.imu_sda = 0x06; // PA6
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuModel));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
    // A model without pins.
    let mut fields = blank_board();
    fields.imu_model = 1;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuScl));
    assert_eq!(err.kind, BoardErrorKind::IncompleteGroup);
    // One pin missing.
    let mut fields = blank_board();
    fields.imu_scl = 0x05;
    fields.imu_model = 1;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
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
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadB));
    assert_eq!(err.kind, BoardErrorKind::DuplicatePin(pin(0x13)));
    // Across motors too.
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[1] = bench_motor0(); // motor 1 reuses every motor-0 pin
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallA, 1));
    assert_eq!(err.kind, BoardErrorKind::DuplicatePin(pin(0x2D)));
}

#[test]
fn reserved_pins_refuse_every_field() {
    // An allowlist pin (PA2) in a LED field.
    let mut fields = blank_board();
    fields.led_orange = 0x02;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::LedOrange));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x02)));
    // SWD (PA13) in a gate field: reserved fires (this slice checks the compiled set; the
    // gate-capability rules are the next slice).
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].gate_hi_a = 0x0D;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::GateHiA, 0));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x0D)));
}

#[test]
fn boot_self_hold_pin_is_reserved_except_for_self_hold_itself() {
    // The blank board's self_hold IS PB12: valid (the field that owns it).
    validate(&blank_board(), &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap();
    // Another field claiming PB12 is refused.
    let mut fields = blank_board();
    fields.self_hold = ABSENT;
    fields.buzzer = 0x1C;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::Buzzer));
    assert_eq!(err.kind, BoardErrorKind::ReservedPin(pin(0x1C)));
    // With no compiled boot assert (None), PB12 is an ordinary pin.
    validate(&fields, &MockChip::F103C8, RESERVED, None).unwrap();
}

#[test]
fn first_failure_wins_in_field_order() {
    // Two failures planted: a bad encoding on an early field (vbatt) and a reserved hit on a
    // later one (pad_a). The early one is reported.
    let mut fields = blank_board();
    fields.vbatt = 0x60;
    fields.pad_a = 0x02;
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::Vbatt));
    assert_eq!(err.kind, BoardErrorKind::BadEncoding(0x60));
}

// ---------------------------------------------------------------------------------------------
// Slice 2: the capability stage (check 4 + check 1's existence half) against the mock tables.
// ---------------------------------------------------------------------------------------------

/// The bench 6-FET board with its IMU on the standard-family PB6/PB7 wiring, reserved set
/// already freed for that port (the LINK_SET rule).
fn bench_board() -> (BoardFields, std::vec::Vec<u8>) {
    let mut fields = blank_board();
    fields.imu_scl = 0x16;
    fields.imu_sda = 0x17;
    fields.imu_model = 2;
    fields.motors[0] = bench_motor0();
    let freed: std::vec::Vec<u8> = RESERVED
        .iter()
        .copied()
        .filter(|p| *p != 0x16 && *p != 0x17)
        .collect();
    (fields, freed)
}

#[test]
fn bench_preset_validates_on_both_families_it_fits() {
    // The 6-FET split board exists as an F103 master and an F130 slave; the same layout must
    // validate on both family tables, with the family-correct derived facts.
    let (fields, freed) = bench_board();
    for (chip, want_bus) in [(MockChip::F103C8, 1u8), (MockChip::F130C8, 0u8)] {
        let plan = validate(&fields, &chip, &freed, BOOT_SELF_HOLD).unwrap();
        assert_eq!(
            plan.imu.unwrap().bus,
            want_bus,
            "the family's own I2C instance"
        );
        let gates = plan.motors[0].gates.unwrap();
        assert_eq!(gates.timer, 0, "TIMER0 on both families");
        assert_eq!(plan.vbatt.unwrap().channel, 4, "PA4 = channel 4 on both");
    }
}

#[test]
fn twelve_fet_map_validates_only_where_its_timers_exist() {
    // The 12-FET dual-motor map: motor 0 = the TIMER0 set, motor 1 = the TIM8 set (PC6/7/8 +
    // PA7/PB0/PB1) with its PC10/PC11/PC12 halls (the 12-FET contract; those port-C pins exist
    // only on the RC part). The contract's OTHER hall set (PB5/6/7) overlaps the allowlist and
    // is a reserved-set/LINK_SET question, not this test's: this vector exercises the
    // timers-exist-per-family rule.
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[1] = MotorFields {
        hall_a: 0x2A,    // PC10
        hall_b: 0x2B,    // PC11
        hall_c: 0x2C,    // PC12
        gate_hi_a: 0x26, // PC6
        gate_hi_b: 0x27, // PC7
        gate_hi_c: 0x28, // PC8
        gate_lo_a: 0x07, // PA7
        gate_lo_b: 0x10, // PB0
        gate_lo_c: 0x11, // PB1
        dead_time: 32,
    };

    // On the RC part: both motors validate, on distinct advanced timers.
    let plan = validate(&fields, &MockChip::F103RC, RESERVED, BOOT_SELF_HOLD).unwrap();
    assert_eq!(plan.motors[0].gates.unwrap().timer, 0);
    assert_eq!(
        plan.motors[1].gates.unwrap().timer,
        1,
        "TIM8, the 12-FET second motor"
    );

    // On the 48-pin part the second motor's pins do not exist: refused at the first such field.
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::HallA, 1));
    assert_eq!(err.kind, BoardErrorKind::UnknownPin(pin(0x2A)));
}

#[test]
fn wrong_family_pins_are_unknown() {
    // PF0 exists on the F130, not the F103...
    let mut fields = blank_board();
    fields.pad_b = 0x50; // PF0
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadB));
    assert_eq!(err.kind, BoardErrorKind::UnknownPin(pin(0x50)));
    validate(&fields, &MockChip::F130C8, RESERVED, BOOT_SELF_HOLD).unwrap();

    // ...and PD0 exists on the F103, not the F130.
    let mut fields = blank_board();
    fields.pad_b = 0x30; // PD0
    let err = validate(&fields, &MockChip::F130C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadB));
    assert_eq!(err.kind, BoardErrorKind::UnknownPin(pin(0x30)));
    validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap();
}

#[test]
fn imu_on_a_non_i2c_pair_is_refused() {
    // A complete IMU group on existing, unreserved, non-I2C pins: the pair derivation refuses
    // (the software-I2C variant is not built; link-only boot until it exists).
    let mut fields = blank_board();
    fields.imu_scl = 0x05; // PA5
    fields.imu_sda = 0x06; // PA6
    fields.imu_model = 1;
    let err = validate(&fields, &MockChip::F130C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::ImuScl));
    assert_eq!(err.kind, BoardErrorKind::NotI2cPair);
}

#[test]
fn scrambled_gate_set_is_refused() {
    // The right six pins in an electrically implausible assignment (a low-side pin in a
    // high-side slot and vice versa): pins all exist, no duplicates, but no advanced timer has
    // this complementary shape.
    let mut fields = blank_board();
    fields.motors[0] = bench_motor0();
    fields.motors[0].gate_hi_c = 0x1D; // PB13 (a low-side pin)
    fields.motors[0].gate_lo_a = 0x0A; // PA10 (a high-side pin)
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, mref(BoardField::GateHiA, 0));
    assert_eq!(err.kind, BoardErrorKind::InvalidGateSet);
}

#[test]
fn gate_capable_pins_refuse_non_gate_functions() {
    // PA8 (TIMER0 CH0, gate-capable) in a LED field: the denylist rule fires even though the
    // pin exists and is unreserved.
    let mut fields = blank_board();
    fields.led_green = 0x08; // PA8
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::LedGreen));
    assert_eq!(err.kind, BoardErrorKind::GateCapableMisused(pin(0x08)));
    // The same pin in its gate slot is of course fine (the bench preset covers it); a low-side
    // gate pin in a pad field is refused too.
    let mut fields = blank_board();
    fields.pad_a = 0x1E; // PB14
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadA));
    assert_eq!(err.kind, BoardErrorKind::GateCapableMisused(pin(0x1E)));
}

#[test]
fn vbatt_must_be_adc_capable() {
    let mut fields = blank_board();
    fields.vbatt = 0x2D; // PC13: exists, no ADC channel behind it
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::Vbatt));
    assert_eq!(err.kind, BoardErrorKind::NotAdcCapable(pin(0x2D)));
}

#[test]
fn capability_stage_runs_after_the_set_level_checks() {
    // Two failures planted: a capability failure on an EARLY field (vbatt on a non-ADC pin) and
    // a set-level failure on a LATE field (a duplicate pad). The set-level failure wins: the
    // capability stage runs only over a fully coherent set.
    let mut fields = blank_board();
    fields.vbatt = 0x2D; // capability failure (NotAdcCapable), field order EARLY
    fields.pad_b = fields.pad_a; // duplicate, field order LATE
    let err = validate(&fields, &MockChip::F103C8, RESERVED, BOOT_SELF_HOLD).unwrap_err();
    assert_eq!(err.field, sref(BoardField::PadB));
    assert_eq!(err.kind, BoardErrorKind::DuplicatePin(pin(0x0B)));
}
