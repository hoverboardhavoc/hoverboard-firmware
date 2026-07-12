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
            // F130C8 (LQFP48): PA0-15, PB0-15, PC13-15, PF0/PF1 + PF6/PF7, no port D.
            // (GD32F130xx Datasheet Rev3.7 Figure 2-3 / Table 2-1: LQFP48 DOES bond PF6/PF7,
            // pins 35/36, and its GPIO count is 39 = 32 + 3 + 4; the earlier PF0-1-only table
            // under-modeled the part.)
            MockChip::F130C8 => match port {
                0 | 1 => true,
                2 => n >= 13,
                5 => matches!(n, 0 | 1 | 6 | 7),
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
        // IDENTICAL on both families in the GD/PeriphLabel numbering (datasheet-verified:
        // PB6/PB7 = I2C0 @ 0x4000_5400, PB10/PB11 = I2C1 @ 0x4000_5800 on the F103 AND the
        // F130; the F130 is NOT single-I2C, its I2C1 sits at 0x4000_5800 with PB10 = I2C1_SCL).
        let _ = self;
        match (scl.packed(), sda.packed()) {
            (0x16, 0x17) => Some(0), // I2C0
            (0x1A, 0x1B) => Some(1), // I2C1
            _ => None,
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
    assert_eq!(imu.bus, 0, "PB6/PB7 = I2C0 (GD numbering), derived");
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
    for chip in [MockChip::F103C8, MockChip::F130C8] {
        let plan = validate(&fields, &chip, &freed, BOOT_SELF_HOLD).unwrap();
        // Asserting SAMENESS is the correct family fact: PB6/PB7 = I2C0 = 0 on every fleet
        // part (GD numbering); family variance stays exercised via port existence + timers.
        assert_eq!(plan.imu.unwrap().bus, 0, "PB6/PB7 = I2C0 on both families");
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

// ---------------------------------------------------------------------------------------------
// Slice 3: the plumbing (read path over a mock flash store, the reserved-set seam, BOARD_OBS).
// ---------------------------------------------------------------------------------------------

mod plumbing_tests {
    use super::*;
    use crate::plumbing::{
        read_fields, reserved_set, AllowlistPort, BoardObs, BOARD_OBS_MAGIC, OBS_OK, SWD_PINS,
    };
    use base::error::FlashError;
    use store::{Flash, Store};

    /// A minimal in-RAM [`Flash`] for the plumbing tests (the `net` walk-tests precedent; the
    /// store's own `MockFlash` is crate-internal).
    struct TestFlash {
        page_size: usize,
        bytes: std::vec::Vec<u8>,
    }

    impl TestFlash {
        fn erased() -> Self {
            TestFlash {
                page_size: 1024,
                bytes: std::vec![0xFFu8; 2 * 1024],
            }
        }
    }

    impl Flash for TestFlash {
        fn page_size(&self) -> usize {
            self.page_size
        }
        fn as_bytes(&self) -> &[u8] {
            &self.bytes
        }
        fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
            let start = page * self.page_size;
            let end = start + self.page_size;
            if end > self.bytes.len() {
                return Err(FlashError::OutOfBounds);
            }
            for b in &mut self.bytes[start..end] {
                *b = 0xFF;
            }
            Ok(())
        }
        fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
            if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
                return Err(FlashError::Misaligned);
            }
            if off + bytes.len() > self.bytes.len() {
                return Err(FlashError::OutOfBounds);
            }
            for (i, &b) in bytes.iter().enumerate() {
                if self.bytes[off + i] != 0xFF && b != self.bytes[off + i] {
                    return Err(FlashError::ProgramFailed);
                }
            }
            self.bytes[off..off + bytes.len()].copy_from_slice(bytes);
            Ok(())
        }
    }

    /// The firmware-owned allowlist, as the caller will pass it (`specs/l3.md`: port 1 = the
    /// inter-board UART PA2/PA3, port 2 = the BLE module PB10/PB11, plus the USART0-remap
    /// PB6/PB7 port; bit numbering = the firmware's link-port indices). Test data mirroring the
    /// caller's compiled fact.
    const ALLOWLIST: &[AllowlistPort] = &[
        AllowlistPort {
            link_set_bit: 1,
            pins: [0x02, 0x03], // PA2/PA3
        },
        AllowlistPort {
            link_set_bit: 2,
            pins: [0x1A, 0x1B], // PB10/PB11
        },
        AllowlistPort {
            link_set_bit: 3,
            pins: [0x16, 0x17], // PB6/PB7 (USART0-remap)
        },
    ];

    #[test]
    fn blank_store_reads_the_registry_defaults() {
        // The defaults' single owner is the registry: a blank store yields the benign fleet
        // pins, absent IMU, absent motor groups (specs/board-model.md defaults post-fold).
        let mut flash = TestFlash::erased();
        let s = Store::mount(&mut flash).unwrap();
        let f = read_fields(&s);
        assert_eq!(f.self_hold, 0x1C); // PB12
        assert_eq!(f.vbatt, 0x04); // PA4
        assert_eq!(f.buzzer, 0x19); // PB9
        assert_eq!((f.led_green, f.led_orange, f.led_red), (0x13, 0x0F, 0x14));
        assert_eq!((f.pad_a, f.pad_b), (0x0B, 0x2F));
        assert_eq!((f.imu_scl, f.imu_sda, f.imu_model), (ABSENT, ABSENT, 0));
        for m in &f.motors {
            assert_eq!(m.hall_a, ABSENT);
            assert_eq!(m.gate_hi_a, ABSENT);
            assert_eq!(m.gate_lo_c, ABSENT);
            assert_eq!(m.dead_time, 0);
        }
    }

    #[test]
    fn written_fields_read_back_including_per_motor_indexing() {
        let mut flash = TestFlash::erased();
        let mut s = Store::mount(&mut flash).unwrap();
        s.set(store::IMU_SCL_PIN, 0x16).unwrap();
        s.set(store::IMU_SDA_PIN, 0x17).unwrap();
        s.set(store::IMU_MODEL, 2).unwrap();
        // Motor 1 configured, motor 0 left absent: the index seam.
        s.set(store::MOTOR_HALL_A.at(1), 0x2A).unwrap();
        s.set(store::MOTOR_DEAD_TIME.at(1), 32).unwrap();
        let f = read_fields(&s);
        assert_eq!((f.imu_scl, f.imu_sda, f.imu_model), (0x16, 0x17, 2));
        assert_eq!(f.motors[0].hall_a, ABSENT, "motor 0 untouched");
        assert_eq!(f.motors[0].dead_time, 0);
        assert_eq!(f.motors[1].hall_a, 0x2A);
        assert_eq!(f.motors[1].dead_time, 32);
    }

    #[test]
    fn reserved_set_frees_only_link_set_cleared_ports() {
        // Unconfigured (link_set == 0): every allowlisted pin reserved, plus SWD always.
        let r = reserved_set(ALLOWLIST, 0);
        for p in [
            0x02u8,
            0x03,
            0x1A,
            0x1B,
            0x16,
            0x17,
            SWD_PINS[0],
            SWD_PINS[1],
        ] {
            assert!(
                r.as_slice().contains(&p),
                "{p:#04x} reserved when unconfigured"
            );
        }
        // Configured with bits 1+2 live (inter-board + BLE): the PB6/PB7 port (bit 3, clear) is
        // FREED; the live ports stay reserved; SWD always.
        let r = reserved_set(ALLOWLIST, 0b0110);
        assert!(!r.as_slice().contains(&0x16), "PB6 freed for the IMU");
        assert!(!r.as_slice().contains(&0x17), "PB7 freed");
        for p in [0x02u8, 0x03, 0x1A, 0x1B, SWD_PINS[0], SWD_PINS[1]] {
            assert!(r.as_slice().contains(&p), "{p:#04x} stays reserved");
        }
    }

    #[test]
    fn board_obs_records_success_and_failure() {
        let ok = BoardObs::success();
        assert_eq!(ok.magic, BOARD_OBS_MAGIC);
        assert_eq!(ok.result, OBS_OK);
        assert_eq!((ok.field_id, ok.index, ok.detail), (0, 0, 0));

        // A failure names the offending field by its REGISTRY id + index and carries the pin.
        let err = BoardError {
            field: FieldRef {
                field: BoardField::GateLoB,
                motor: Some(1),
            },
            kind: BoardErrorKind::UnknownPin(pin(0x2A)),
        };
        let obs = BoardObs::failure(&err);
        assert_eq!(obs.magic, BOARD_OBS_MAGIC);
        assert_eq!(obs.result, 6);
        assert_eq!(obs.field_id, store::MOTOR_GATE_LO_B.id()); // 0x51, via the handle
        assert_eq!(obs.field_id, 0x51);
        assert_eq!(obs.index, 1);
        assert_eq!(obs.detail, 0x2A);
    }

    #[test]
    fn end_to_end_store_to_validate_to_obs() {
        // The full slice-3 chain over a mock store + the mock caps: a configured standard-family
        // board (IMU on the LINK_SET-freed PB6/PB7 port) validates and yields the success
        // record; then one bad write flips it to the named failure record.
        let mut flash = TestFlash::erased();
        let mut s = Store::mount(&mut flash).unwrap();
        s.set(store::LINK_SET, 0b0110).unwrap(); // inter-board + BLE live; PB6/PB7 freed
        s.set(store::IMU_SCL_PIN, 0x16).unwrap();
        s.set(store::IMU_SDA_PIN, 0x17).unwrap();
        s.set(store::IMU_MODEL, 2).unwrap();

        let link_set: u8 = s.get(store::LINK_SET);
        let reserved = reserved_set(ALLOWLIST, link_set);
        let fields = read_fields(&s);
        let obs = match validate(
            &fields,
            &MockChip::F130C8,
            reserved.as_slice(),
            BOOT_SELF_HOLD,
        ) {
            Ok(_plan) => BoardObs::success(),
            Err(e) => BoardObs::failure(&e),
        };
        assert_eq!(obs, BoardObs::success());

        // One bad write: vbatt moved to a non-ADC pin. The failure record names it.
        s.set(store::BOARD_VBATT, 0x2D).unwrap(); // PC13
        let fields = read_fields(&s);
        let obs = match validate(
            &fields,
            &MockChip::F130C8,
            reserved.as_slice(),
            BOOT_SELF_HOLD,
        ) {
            Ok(_plan) => BoardObs::success(),
            Err(e) => BoardObs::failure(&e),
        };
        assert_eq!(obs.result, 9, "NotAdcCapable");
        assert_eq!(obs.field_id, store::BOARD_VBATT.id());
        assert_eq!(obs.detail, 0x2D);
    }
}

// ---------------------------------------------------------------------------------------------
// Slice 4: the R-CAP agreement suite. The mock capability tables above are what the validator
// logic was proven against; runtime-hal's REAL pin-capability queries (its
// specs/pin-capability.md) are what the firmware adapter answers from at boot. The two MUST
// agree on every vector either side can express, so they cannot drift apart. runtime-hal is a
// dev-dependency here only (the shipped lib stays HAL-free).
// ---------------------------------------------------------------------------------------------

mod rcap_agreement {
    use super::*;
    use runtime_hal::chip::Chip;
    use runtime_hal::detect::probe::Detected;
    use runtime_hal::pincap;
    use runtime_hal::{descriptor_f103, descriptor_f130, synthesize, Family, PeriphLabel};

    /// The real [`Capabilities`] implementation over runtime-hal's R-CAP queries, mirroring the
    /// firmware's `HalCaps` adapter (`crates/firmware`): packed bytes across the seam, the named
    /// advanced timer mapped to the trait's zero-based index.
    struct RealCaps {
        chip: Chip,
    }

    impl Capabilities for RealCaps {
        fn pin_exists(&self, pin: Pin) -> bool {
            pincap::pin_exists(&self.chip, pin.packed())
        }
        fn gate_capable(&self, pin: Pin) -> bool {
            pincap::gate_capable(&self.chip, pin.packed())
        }
        fn gate_set(&self, hi: [Pin; 3], lo: [Pin; 3]) -> Option<u8> {
            pincap::gate_set(&self.chip, hi.map(|p| p.packed()), lo.map(|p| p.packed())).map(|t| {
                if t == PeriphLabel::Timer7 {
                    1
                } else {
                    0
                }
            })
        }
        fn adc_channel(&self, pin: Pin) -> Option<u8> {
            pincap::adc_channel(&self.chip, pin.packed())
        }
        fn i2c_pair(&self, scl: Pin, sda: Pin) -> Option<u8> {
            pincap::i2c_pair(&self.chip, scl.packed(), sda.packed())
        }
    }

    /// The (mock, real) chip pairs under agreement: the three fleet parts. The real F103C8 /
    /// F130C8 are the HAL's bench reference descriptors; the 12-FET GD32F103RC is synthesized
    /// through the same detection path (256 KiB, two advanced timers, three ADCs measured).
    fn pairs() -> [(MockChip, RealCaps, &'static str); 3] {
        [
            (
                MockChip::F103C8,
                RealCaps {
                    chip: Chip::from_descriptor(descriptor_f103()),
                },
                "F103C8",
            ),
            (
                MockChip::F130C8,
                RealCaps {
                    chip: Chip::from_descriptor(descriptor_f130()),
                },
                "F130C8",
            ),
            (
                MockChip::F103RC,
                RealCaps {
                    chip: Chip::from_descriptor(synthesize(&Detected {
                        family: Family::F10x,
                        flash_kib: 256,
                        adv_timers: 2,
                        adc_count: 3,
                    })),
                },
                "F103RC",
            ),
        ]
    }

    /// Every encoding-valid pin (ports A..D, F; pins 0..15).
    fn all_pins() -> std::vec::Vec<Pin> {
        (0u8..=0xFE)
            .filter_map(|raw| match Pin::parse(raw) {
                Parsed::Valid(p) => Some(p),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn per_pin_queries_agree_on_every_encodable_pin() {
        for (mock, real, part) in pairs() {
            for p in all_pins() {
                assert_eq!(
                    mock.pin_exists(p),
                    real.pin_exists(p),
                    "{part} pin_exists({:#04x})",
                    p.packed()
                );
                assert_eq!(
                    mock.gate_capable(p),
                    real.gate_capable(p),
                    "{part} gate_capable({:#04x})",
                    p.packed()
                );
                assert_eq!(
                    mock.adc_channel(p),
                    real.adc_channel(p),
                    "{part} adc_channel({:#04x})",
                    p.packed()
                );
            }
        }
    }

    #[test]
    fn gate_set_agrees_on_the_fleet_maps_and_their_mutations() {
        let t0_hi = GATES_T0_HI.map(pin);
        let t0_lo = GATES_T0_LO.map(pin);
        let t8_hi = GATES_T8_HI.map(pin);
        let t8_lo = GATES_T8_LO.map(pin);
        // The two fleet maps, their swap, a scramble (the scrambled_gate_set vector), and a
        // rotation: both implementations must place each identically on every part.
        let mut scrambled_hi = t0_hi;
        scrambled_hi[2] = t0_lo[0]; // a low-side pin in a high-side slot
        let mut scrambled_lo = t0_lo;
        scrambled_lo[0] = t0_hi[2];
        let rotated_hi = [t0_hi[1], t0_hi[2], t0_hi[0]];
        let vectors: [([Pin; 3], [Pin; 3]); 6] = [
            (t0_hi, t0_lo),
            (t8_hi, t8_lo),
            (t0_lo, t0_hi),
            (scrambled_hi, scrambled_lo),
            (rotated_hi, t0_lo),
            (t0_hi, t8_lo),
        ];
        for (mock, real, part) in pairs() {
            for (hi, lo) in vectors {
                assert_eq!(
                    mock.gate_set(hi, lo),
                    real.gate_set(hi, lo),
                    "{part} gate_set({:?}, {:?})",
                    hi.map(|p| p.packed()),
                    lo.map(|p| p.packed())
                );
            }
        }
        // And the expected derivations themselves, so agreement is never two matching wrongs:
        // TIMER0 = index 0 everywhere, the TIM8 set = index 1 on the 12-FET part only.
        for (_, real, part) in pairs() {
            assert_eq!(real.gate_set(t0_hi, t0_lo), Some(0), "{part} TIMER0");
        }
        let [_, _, (_, rc, _)] = pairs();
        assert_eq!(rc.gate_set(t8_hi, t8_lo), Some(1), "RC TIM8");
    }

    #[test]
    fn i2c_pair_agrees_on_every_encodable_pair() {
        // The full ordered-pair sweep (80 x 80 per part): covers the two instances, reversed
        // pairs, and every non-pair.
        for (mock, real, part) in pairs() {
            for scl in all_pins() {
                for sda in all_pins() {
                    assert_eq!(
                        mock.i2c_pair(scl, sda),
                        real.i2c_pair(scl, sda),
                        "{part} i2c_pair({:#04x}, {:#04x})",
                        scl.packed(),
                        sda.packed()
                    );
                }
            }
        }
        // The expected derivations (not just agreement): the GD zero-based numbering.
        for (_, real, part) in pairs() {
            assert_eq!(real.i2c_pair(pin(0x16), pin(0x17)), Some(0), "{part} I2C0");
            assert_eq!(real.i2c_pair(pin(0x1A), pin(0x1B)), Some(1), "{part} I2C1");
        }
    }

    #[test]
    fn whole_validator_runs_identically_over_mock_and_real() {
        // End to end: the bench 6-FET layout (IMU on the LINK_SET-freed PB6/PB7) through
        // `validate` over the REAL capabilities produces the same fully-derived plan the mock
        // run produces, on every fleet part it fits.
        let (fields, freed) = bench_board();
        for (mock, real, part) in pairs() {
            let mock_plan = validate(&fields, &mock, &freed, BOOT_SELF_HOLD).unwrap();
            let real_plan = validate(&fields, &real, &freed, BOOT_SELF_HOLD).unwrap();
            assert_eq!(mock_plan, real_plan, "{part} plan");
            assert_eq!(real_plan.imu.unwrap().bus, 0, "{part} I2C0 derived");
            assert_eq!(real_plan.motors[0].gates.unwrap().timer, 0, "{part} TIMER0");
        }
    }
}
