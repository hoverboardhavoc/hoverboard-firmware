//! The board-layout boot validator (`specs/board-model.md`), slice 1: the field vocabulary,
//! the packed port|pin parse, and the set-level coherence checks that need no chip-capability
//! data (parse validity, group completeness incl. the dead-time rule, duplicates, reserved-pin
//! collisions). The `Capabilities`-dependent checks (chip pin existence, gate-set/ADC/I2C
//! capability) are the next slice.
//!
//! **HAL-free by contract**: field values arrive as plain bytes (the slice-3 plumbing reads them
//! from the store registry, whose defaults are the single owner; this crate carries NO default
//! values), the compiled reserved set arrives as a parameter (its owner is the firmware's
//! safe-USART allowlist, `specs/l3.md`), and a validated [`BoardPlan`] leaves as data. Fail-loud:
//! the FIRST failure wins and names the offending field; any failure means link-only boot with
//! no partial bring-up (the caller's contract).
//!
//! Pin encoding (`specs/board-model.md`, "The field vocabulary"): one byte, `(port << 4) | pin`,
//! port A = 0, B = 1, C = 2, D = 3, F = 5 (the encoding defines no port E and nothing above F);
//! `0xFF` = unset = the function is absent, a valid state. Whether a chip HAS a decoded port/pin
//! is the next slice's capability question; this slice rejects only encoding-invalid bytes.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target.

#![no_std]

/// The unset sentinel: the function is absent on this board (matches `store::PIN_ABSENT`).
pub const ABSENT: u8 = 0xFF;

/// The three outcomes of parsing a packed field byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Parsed {
    /// `0xFF`: the function is absent (valid).
    Absent,
    /// An encoding-valid pin.
    Valid(Pin),
    /// A byte the encoding does not define.
    Invalid,
}

/// A parsed, encoding-valid pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    port: u8,
    pin: u8,
}

impl Pin {
    /// Parse a packed field byte: [`Parsed::Absent`] for `0xFF`, [`Parsed::Valid`] for a byte
    /// whose port nibble the encoding defines, [`Parsed::Invalid`] otherwise (4 = the
    /// nonexistent port E, or anything above F). The pin nibble is 0..15 by construction.
    pub fn parse(raw: u8) -> Parsed {
        if raw == ABSENT {
            return Parsed::Absent;
        }
        let port = raw >> 4;
        match port {
            0..=3 | 5 => Parsed::Valid(Pin {
                port,
                pin: raw & 0x0F,
            }),
            _ => Parsed::Invalid,
        }
    }

    /// The port index (A = 0, B = 1, C = 2, D = 3, F = 5).
    pub const fn port(&self) -> u8 {
        self.port
    }

    /// The pin number within the port (0..15).
    pub const fn pin(&self) -> u8 {
        self.pin
    }

    /// The packed byte back.
    pub const fn packed(&self) -> u8 {
        (self.port << 4) | self.pin
    }
}

/// Which field a validator failure names (`specs/board-model.md`: "the failure names the
/// offending field"). Motor-scoped fields carry the motor index in [`FieldRef`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardField {
    SelfHold,
    Vbatt,
    Buzzer,
    LedGreen,
    LedOrange,
    LedRed,
    PadA,
    PadB,
    ImuScl,
    ImuSda,
    ImuModel,
    HallA,
    HallB,
    HallC,
    GateHiA,
    GateHiB,
    GateHiC,
    GateLoA,
    GateLoB,
    GateLoC,
    DeadTime,
}

/// The offending field of a failure: which field, and for the per-motor fields, which motor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldRef {
    pub field: BoardField,
    /// `Some(motor index)` for the per-motor fields; `None` for singletons.
    pub motor: Option<u8>,
}

/// What went wrong (slice-1 checks; the capability kinds arrive next slice).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardErrorKind {
    /// The byte is not a valid pin encoding (carries the raw byte).
    BadEncoding(u8),
    /// A function group is partially present (all-or-none rule).
    IncompleteGroup,
    /// A configured gate group has `motor.dead_time == 0`.
    MissingDeadTime,
    /// The pin is already assigned to another field (carries the colliding pin).
    DuplicatePin(Pin),
    /// The pin collides with the compiled reserved set (allowlist / SWD / the boot-asserted
    /// self-hold default).
    ReservedPin(Pin),
}

/// A validator failure: the first failing check, naming the offending field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoardError {
    pub field: FieldRef,
    pub kind: BoardErrorKind,
}

/// The raw per-motor field values, as read from the registry (index = the motor).
#[derive(Clone, Copy, Debug)]
pub struct MotorFields {
    pub hall_a: u8,
    pub hall_b: u8,
    pub hall_c: u8,
    pub gate_hi_a: u8,
    pub gate_hi_b: u8,
    pub gate_hi_c: u8,
    pub gate_lo_a: u8,
    pub gate_lo_b: u8,
    pub gate_lo_c: u8,
    /// Raw DTG; 0 = unset. Required nonzero iff the gate group is configured.
    pub dead_time: u8,
}

impl MotorFields {
    /// An all-absent motor (no halls, no gates, no dead-time).
    pub const ABSENT: MotorFields = MotorFields {
        hall_a: ABSENT,
        hall_b: ABSENT,
        hall_c: ABSENT,
        gate_hi_a: ABSENT,
        gate_hi_b: ABSENT,
        gate_hi_c: ABSENT,
        gate_lo_a: ABSENT,
        gate_lo_b: ABSENT,
        gate_lo_c: ABSENT,
        dead_time: 0,
    };
}

/// The raw board field set, as read from the registry (defaults already applied by the store's
/// `get`; this crate carries no defaults of its own).
#[derive(Clone, Copy, Debug)]
pub struct BoardFields {
    pub self_hold: u8,
    pub vbatt: u8,
    pub buzzer: u8,
    pub led_green: u8,
    pub led_orange: u8,
    pub led_red: u8,
    pub pad_a: u8,
    pub pad_b: u8,
    pub imu_scl: u8,
    pub imu_sda: u8,
    /// `specs/imu.md`: 0 = no IMU fitted; nonzero = the imu crate's model index.
    pub imu_model: u8,
    pub motors: [MotorFields; 2],
}

/// The validated hall group of one motor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HallSet {
    pub a: Pin,
    pub b: Pin,
    pub c: Pin,
}

/// The validated gate group of one motor (with its required dead-time).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GateSet {
    pub hi: [Pin; 3],
    pub lo: [Pin; 3],
    pub dead_time: u8,
}

/// The validated IMU group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuPlan {
    pub scl: Pin,
    pub sda: Pin,
    pub model: u8,
}

/// One motor's validated plan (absent groups are absent functions).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct MotorPlan {
    pub halls: Option<HallSet>,
    pub gates: Option<GateSet>,
}

/// The validated board plan: the coherent, capability-unchecked layout (this slice; the next
/// slice's capability pass consumes the same plan). Absent = the function does not exist on this
/// board.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct BoardPlan {
    pub self_hold: Option<Pin>,
    pub vbatt: Option<Pin>,
    pub buzzer: Option<Pin>,
    pub led_green: Option<Pin>,
    pub led_orange: Option<Pin>,
    pub led_red: Option<Pin>,
    pub pad_a: Option<Pin>,
    pub pad_b: Option<Pin>,
    pub imu: Option<ImuPlan>,
    pub motors: [MotorPlan; 2],
}

/// The set-level coherence validation (checks 1-3 of `specs/board-model.md`, "The boot
/// validator"): parse validity, group completeness (incl. the configured-gate-group-requires-
/// nonzero-dead-time rule), duplicates across the whole set, and reserved-pin collisions.
///
/// - `reserved`: the COMPILED reserved pins no field may claim (the caller owns them: the
///   safe-USART allowlist pins + SWD, `specs/l3.md`), packed bytes.
/// - `boot_self_hold`: the compiled pre-mount self-hold assert pin (reserved against every field
///   EXCEPT `self_hold` itself, which may legitimately name it), packed byte or `None`.
///
/// First failure wins; `Ok` yields the [`BoardPlan`] the capability pass (next slice) and the
/// integration bring-up consume.
pub fn validate(
    fields: &BoardFields,
    reserved: &[u8],
    boot_self_hold: Option<u8>,
) -> Result<BoardPlan, BoardError> {
    let mut plan = BoardPlan::default();

    // A small claims table for the duplicate check: every assigned pin, in field order, so the
    // SECOND claimant is the named offender. 8 singleton pin fields + 2 IMU + 2 motors * 9.
    let mut claimed: [Option<(Pin, FieldRef)>; 28] = [None; 28];
    let mut n_claimed = 0usize;

    // --- Checks 1 + 3 for one field: parse, then reserved, then duplicates. ---
    let take = |raw: u8,
                fref: FieldRef,
                claimed: &mut [Option<(Pin, FieldRef)>; 28],
                n_claimed: &mut usize|
     -> Result<Option<Pin>, BoardError> {
        let pin = match Pin::parse(raw) {
            Parsed::Invalid => {
                return Err(BoardError {
                    field: fref,
                    kind: BoardErrorKind::BadEncoding(raw),
                })
            }
            Parsed::Absent => return Ok(None),
            Parsed::Valid(p) => p,
        };
        // Reserved: the compiled set applies to every field; the boot self-hold pin applies to
        // every field except self_hold itself.
        let hits_reserved = reserved.contains(&pin.packed())
            || (boot_self_hold == Some(pin.packed()) && fref.field != BoardField::SelfHold);
        if hits_reserved {
            return Err(BoardError {
                field: fref,
                kind: BoardErrorKind::ReservedPin(pin),
            });
        }
        // Duplicates: the second claimant is the offender.
        for (other, _) in claimed.iter().take(*n_claimed).flatten() {
            if *other == pin {
                return Err(BoardError {
                    field: fref,
                    kind: BoardErrorKind::DuplicatePin(pin),
                });
            }
        }
        claimed[*n_claimed] = Some((pin, fref));
        *n_claimed += 1;
        Ok(Some(pin))
    };

    let single = |f: BoardField| FieldRef {
        field: f,
        motor: None,
    };

    // Singletons.
    plan.self_hold = take(
        fields.self_hold,
        single(BoardField::SelfHold),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.vbatt = take(
        fields.vbatt,
        single(BoardField::Vbatt),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.buzzer = take(
        fields.buzzer,
        single(BoardField::Buzzer),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.led_green = take(
        fields.led_green,
        single(BoardField::LedGreen),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.led_orange = take(
        fields.led_orange,
        single(BoardField::LedOrange),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.led_red = take(
        fields.led_red,
        single(BoardField::LedRed),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.pad_a = take(
        fields.pad_a,
        single(BoardField::PadA),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.pad_b = take(
        fields.pad_b,
        single(BoardField::PadB),
        &mut claimed,
        &mut n_claimed,
    )?;

    // The IMU group (check 2: both pins + a nonzero model all-or-none).
    let imu_scl = take(
        fields.imu_scl,
        single(BoardField::ImuScl),
        &mut claimed,
        &mut n_claimed,
    )?;
    let imu_sda = take(
        fields.imu_sda,
        single(BoardField::ImuSda),
        &mut claimed,
        &mut n_claimed,
    )?;
    plan.imu = match (imu_scl, imu_sda, fields.imu_model) {
        (None, None, 0) => None,
        (Some(scl), Some(sda), model) if model != 0 => Some(ImuPlan { scl, sda, model }),
        // Partially present: name the first absent/odd member of the group.
        (None, _, _) => {
            return Err(BoardError {
                field: single(BoardField::ImuScl),
                kind: BoardErrorKind::IncompleteGroup,
            })
        }
        (_, None, _) => {
            return Err(BoardError {
                field: single(BoardField::ImuSda),
                kind: BoardErrorKind::IncompleteGroup,
            })
        }
        (_, _, _) => {
            return Err(BoardError {
                field: single(BoardField::ImuModel),
                kind: BoardErrorKind::IncompleteGroup,
            })
        }
    };

    // The motor groups (check 2: halls all-or-none; gates all-or-none; configured gates require
    // a nonzero dead-time).
    for (m, mf) in fields.motors.iter().enumerate() {
        let mref = |f: BoardField| FieldRef {
            field: f,
            motor: Some(m as u8),
        };

        let halls = [
            (mf.hall_a, BoardField::HallA),
            (mf.hall_b, BoardField::HallB),
            (mf.hall_c, BoardField::HallC),
        ];
        let mut hall_pins = [None; 3];
        for (i, (raw, f)) in halls.iter().enumerate() {
            hall_pins[i] = take(*raw, mref(*f), &mut claimed, &mut n_claimed)?;
        }
        let n_halls = hall_pins.iter().filter(|p| p.is_some()).count();
        let hall_set = match n_halls {
            0 => None,
            3 => Some(HallSet {
                a: hall_pins[0].unwrap(),
                b: hall_pins[1].unwrap(),
                c: hall_pins[2].unwrap(),
            }),
            _ => {
                // Name the first ABSENT member (the missing piece of the group).
                let missing = hall_pins.iter().position(|p| p.is_none()).unwrap();
                return Err(BoardError {
                    field: mref(halls[missing].1),
                    kind: BoardErrorKind::IncompleteGroup,
                });
            }
        };

        let gates = [
            (mf.gate_hi_a, BoardField::GateHiA),
            (mf.gate_hi_b, BoardField::GateHiB),
            (mf.gate_hi_c, BoardField::GateHiC),
            (mf.gate_lo_a, BoardField::GateLoA),
            (mf.gate_lo_b, BoardField::GateLoB),
            (mf.gate_lo_c, BoardField::GateLoC),
        ];
        let mut gate_pins = [None; 6];
        for (i, (raw, f)) in gates.iter().enumerate() {
            gate_pins[i] = take(*raw, mref(*f), &mut claimed, &mut n_claimed)?;
        }
        let n_gates = gate_pins.iter().filter(|p| p.is_some()).count();
        let gate_set = match n_gates {
            0 => None,
            6 => {
                // The table's rule, carried in check 2: a configured gate group requires a
                // nonzero dead-time.
                if mf.dead_time == 0 {
                    return Err(BoardError {
                        field: mref(BoardField::DeadTime),
                        kind: BoardErrorKind::MissingDeadTime,
                    });
                }
                Some(GateSet {
                    hi: [
                        gate_pins[0].unwrap(),
                        gate_pins[1].unwrap(),
                        gate_pins[2].unwrap(),
                    ],
                    lo: [
                        gate_pins[3].unwrap(),
                        gate_pins[4].unwrap(),
                        gate_pins[5].unwrap(),
                    ],
                    dead_time: mf.dead_time,
                })
            }
            _ => {
                let missing = gate_pins.iter().position(|p| p.is_none()).unwrap();
                return Err(BoardError {
                    field: mref(gates[missing].1),
                    kind: BoardErrorKind::IncompleteGroup,
                });
            }
        };

        plan.motors[m] = MotorPlan {
            halls: hall_set,
            gates: gate_set,
        };
    }

    Ok(plan)
}

#[cfg(test)]
mod tests;
