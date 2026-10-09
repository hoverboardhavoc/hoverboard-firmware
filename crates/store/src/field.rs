//! Typed field handles, the firmware's compile-checked front door, and the curated field set.
//!
//! Each registered field is a typed `const` handle whose Rust type is the field's storage type, so
//! misuse does not compile: `get` only accepts a [`Field<T>`] and yields `T`, `get_text` only accepts a
//! [`StrField`], `get_bytes` only a [`BlobField`]. A scalar getter on a string field, the wrong scalar
//! width, or a `STR` write to a `BLOB` field are all *type errors*, never a runtime `None`. So the
//! typed path has no `TypeMismatch` and no `UnknownKey`.
//!
//! The handle is the **single source of truth**: each field's `id`, storage type, and typed default
//! are each written in exactly one place, on the handle. There is no parallel `FieldDef`/`REGISTRY`
//! table to keep in agreement.

use crate::key::{Key, Scalar, Type};
use crate::value::Value;

/// A scalar field handle (`T` = `u32`, `i32`, `bool`, ...), carrying its `field_id` and typed
/// `default`. Its storage type is `<T as Scalar>::KIND`.
#[derive(Clone, Copy)]
pub struct Field<T: Scalar> {
    field_id: u8,
    index: u8,
    default: T,
}

impl<T: Scalar> Field<T> {
    /// Declare a scalar field with its permanent `id` and typed `default`. `const` so the field set
    /// is a table of `const` handles.
    pub const fn new(id: u8, default: T) -> Self {
        Self {
            field_id: id,
            index: 0,
            default,
        }
    }

    /// Select an instance (motor 0/1). Returns the same handle with its `index` set; a singleton
    /// reads without it (`index = 0`).
    pub const fn at(self, index: u8) -> Self {
        Self { index, ..self }
    }

    /// This field's permanent id.
    pub const fn id(self) -> u8 {
        self.field_id
    }

    /// The raw `Key` (the on-flash / on-wire form) this handle resolves to.
    pub const fn key(self) -> Key {
        Key {
            field_id: self.field_id,
            index: self.index,
        }
    }

    /// The storage type tag (`<T as Scalar>::KIND`).
    pub const fn kind(self) -> Type {
        T::KIND
    }

    /// The typed default, read when the field is absent.
    pub const fn default(self) -> T {
        self.default
    }
}

/// An index family whose default VARIES BY INDEX: `defaults[i]` is index `i`'s default.
///
/// The ordinary index family ([`IMU_GYRO_BIAS`], [`ATTITUDE_LEVEL_TRIM`]) shares one default across
/// its indices and is a plain [`Field`]. This exists for the family that cannot: the balance-PID
/// gain triples ([`CONTROL_GAIN_A`] / [`CONTROL_GAIN_B`]), whose three indices default to three
/// different stock constants. The alternative was a sentinel default plus a fallback table in the
/// consumer, i.e. the field's real default living somewhere other than the field.
///
/// It is a SEPARATE type rather than an option on [`Field`] for a measured reason: `Field` is
/// passed by value to every typed `get`, and widening it by a defaults pointer cost **736 B** of
/// image across the 22 non-test call sites that link (74 counting tests; measured 2026-08-13,
/// `cargo image` span, and independently reconstructed at +752 B). Resolving
/// [`Self::at`] to a plain `Field` keeps that handle four bytes and confines the extra width to the
/// two consts that need it.
#[derive(Clone, Copy)]
pub struct IndexedField<T: Scalar, const N: usize> {
    field_id: u8,
    defaults: [T; N],
}

impl<T: Scalar, const N: usize> IndexedField<T, N> {
    /// Declare an index family with its permanent `id` and one default per index.
    pub const fn new(id: u8, defaults: [T; N]) -> Self {
        Self {
            field_id: id,
            defaults,
        }
    }

    /// This family's permanent id (shared by every index).
    pub const fn id(self) -> u8 {
        self.field_id
    }

    /// How many indices the family declares. Sizes the [`REGISTRY`] entries it contributes.
    pub const fn len(self) -> usize {
        N
    }

    /// Whether the family declares no indices at all (never true for a declared field; present
    /// because `len` without it reads as an oversight).
    pub const fn is_empty(self) -> bool {
        N == 0
    }

    /// One index as an ordinary [`Field`] handle, carrying THAT index's default: the form every
    /// typed `get` / `set` takes. An index past the declared end resolves to index 0, the same
    /// fallback [`lookup_key`] applies on the dynamic path, so no path can panic on a stray index.
    pub const fn at(self, index: u8) -> Field<T> {
        let i = if (index as usize) < N { index } else { 0 };
        Field {
            field_id: self.field_id,
            index: i,
            default: self.defaults[i as usize],
        }
    }
}

// `Field<T>::def()` is NOT here. It is emitted per concrete scalar by `impl_scalar_int!` in `key.rs`,
// beside the `Scalar` impl that already owns the `T -> Type -> Value` mapping, because a generic
// `def()` cannot be `const`: it would have to call a trait method to lift the typed default into a
// `Value`, and const trait methods do not exist on this toolchain. `const` is the whole point, since
// it is what makes [`REGISTRY`] a `static`.

/// A `STR` field handle, carrying a `&'static str` default: the declaration writes the default as a
/// string literal, which is how it reads best (`StrField::new(0x10, "Hoverboard")`).
///
/// STR and BLOB are byte-identical on flash and, since shrink round 2
/// (`specs/decision-flash-budget.md`, item 4), on the READ as well: the board no longer validates a
/// `STR` record as UTF-8, so this differs from [`BlobField`] only in the [`Type`] tag it writes. The
/// literal is therefore converted with the const [`str::as_bytes`] wherever bytes are what is
/// wanted: [`StrField::default`] and the [`FieldDef`] this handle lifts itself into.
#[derive(Clone, Copy)]
pub struct StrField {
    field_id: u8,
    index: u8,
    default: &'static str,
}

impl StrField {
    /// Declare a `STR` field with its permanent `id` and `&'static str` default.
    pub const fn new(id: u8, default: &'static str) -> Self {
        Self {
            field_id: id,
            index: 0,
            default,
        }
    }

    /// Select an instance.
    pub const fn at(self, index: u8) -> Self {
        Self { index, ..self }
    }

    pub const fn id(self) -> u8 {
        self.field_id
    }

    pub const fn key(self) -> Key {
        Key {
            field_id: self.field_id,
            index: self.index,
        }
    }

    /// The declared default, as BYTES - the same type `Store::get_text` returns, so the value and
    /// its fallback are one type at every call site. The declaration still carries a `&'static str`
    /// literal; this is its const `as_bytes`.
    pub const fn default(self) -> &'static [u8] {
        self.default.as_bytes()
    }

    /// This field's runtime [`FieldDef`]. `const`, so it can build [`REGISTRY`] in flash.
    pub const fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            index: self.index,
            kind: Type::Str,
            default: Value::Str(self.default.as_bytes()),
        }
    }
}

/// A `BLOB` field handle, carrying a `&'static [u8]` default.
#[derive(Clone, Copy)]
pub struct BlobField {
    field_id: u8,
    index: u8,
    default: &'static [u8],
}

impl BlobField {
    /// Declare a `BLOB` field with its permanent `id` and `&'static [u8]` default.
    pub const fn new(id: u8, default: &'static [u8]) -> Self {
        Self {
            field_id: id,
            index: 0,
            default,
        }
    }

    /// Select an instance.
    pub const fn at(self, index: u8) -> Self {
        Self { index, ..self }
    }

    pub const fn id(self) -> u8 {
        self.field_id
    }

    pub const fn key(self) -> Key {
        Key {
            field_id: self.field_id,
            index: self.index,
        }
    }

    pub const fn default(self) -> &'static [u8] {
        self.default
    }

    /// This field's runtime [`FieldDef`]. `const`, so it can build [`REGISTRY`] in flash.
    pub const fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            index: self.index,
            kind: Type::Blob,
            default: Value::Bytes(self.default),
        }
    }
}

// ---------------------------------------------------------------------------
// The field set: the curated, minimal set of genuine tunables, single source of truth.
//
// Each id is written once, on its handle. `field_ids!` collects the ids into a const array AND
// emits a build-time uniqueness assertion (a duplicate id would collide on flash). The assertion is
// a `const` evaluated at compile time, so a duplicate is a *compile error*, not a runtime check.
// ---------------------------------------------------------------------------

/// Collect the declared field ids into [`FIELD_IDS`] and assert at compile time that they are
/// unique. A duplicate id fails the const eval ([`assert_unique_ids`]) and so fails the build.
macro_rules! field_ids {
    ($($id:expr),+ $(,)?) => {
        /// Every declared `field_id`, the input to the build-time uniqueness assertion.
        pub const FIELD_IDS: &[u8] = &[$($id),+];

        // Force the const assertion: referencing this associated const evaluates it at compile time.
        const _: () = assert_unique_ids(FIELD_IDS);
    };
}

/// `const` uniqueness check over the declared ids. Panics in const context (a compile error) on a
/// duplicate. O(n^2), fine for a small curated set.
const fn assert_unique_ids(ids: &[u8]) {
    let mut i = 0;
    while i < ids.len() {
        let mut j = i + 1;
        while j < ids.len() {
            if ids[i] == ids[j] {
                panic!("duplicate field_id in the store field set");
            }
            j += 1;
        }
        i += 1;
    }
}

// The genuine tunables. (Sem/name and arity are deliberately NOT here, see the spec "What the
// field set deliberately does NOT carry". The board-LAYOUT fields are a distinct class, below.)
pub const MOTOR_CURRENT_LIMIT: Field<u32> = Field::new(0x20, 10_000);
pub const MOTOR_METHOD: Field<u8> = Field::new(0x21, 0);
/// The runtime control mode (`specs/control.md` (b), the `MOTOR_METHOD` precedent): `0 =
/// Throttle` (default: works on every board, no IMU required; balancing is an opt-in), `1 =
/// Balance`. Consumed by the control crate's mode dispatch (its `ControlMode::from_u8` maps
/// unknown values to Throttle); changes apply while disarmed only, at the config-apply seam.
pub const CONTROL_MODE: Field<u8> = Field::new(0x22, 0);
/// Whether balance mode requires a rider (`specs/control.md` (i)): `1 = required` (the default: an
/// unconfigured board engages only with a rider, exactly as before the field existed), `0 = not
/// required` (the rover: the engage conjunction runs without the rider term, the step-off wind-down
/// is held clear and the profile is A). Any nonzero value reads as required. A machine-type setting
/// beside [`CONTROL_MODE`], boot-read into the control dispatch, applied at the next boot; NOT on the
/// tune lane.
pub const CONTROL_RIDER_REQUIRED: Field<u8> = Field::new(0x23, 1);
/// The low-battery floor (`specs/sensing-and-safety.md`, "The low-battery floor"), centivolts: a
/// balance ENGAGE is refused while the effective battery word is below it; never a disengage (on a
/// balancing rover a refused engage is safe and a disengage is a fall, so a run that started above
/// the floor is not ended by sag under it). **`<= 0` = no floor** (the UNKNOWN-word refusal still
/// applies). Default 2400 cV: 3.0 V per cell on eight cells. No clamp beyond the type. A
/// machine-type setting beside [`CONTROL_RIDER_REQUIRED`], boot-read into the control dispatch,
/// applied at the next boot; NOT on the tune lane.
pub const CONTROL_BATTERY_FLOOR: Field<i16> = Field::new(0x24, 2400);
pub const DEVICE_NAME: StrField = StrField::new(0x10, "Hoverboard");
pub const SOME_BLOB: BlobField = BlobField::new(0x30, &[]);

/// The board's persistent L3 node address (`specs/l3.md`, "Addressing"): assigned once by the walk's
/// `ASSIGN` and persisted to flash, reported on every boot, survives reboot. `0x00` = no address yet.
/// The same field a `CONFIG_WRITE` of this key would touch; `ASSIGN` is the bootstrap path that reaches
/// it by relay before the board has an address.
pub const NODE_ADDRESS: Field<u8> = Field::new(0x01, 0x00);

/// The L3 **link-set** (`specs/l3.md`, "Unconfigured bring-up"; `specs/storage-layer.md`): a bitmask
/// of the local ports that came up live (found a module or a peer) during discovery, persisted
/// alongside [`NODE_ADDRESS`]. `0x00` (the default) means **unconfigured** -> the firmware runs the
/// whitelist BT-probe + link-listen; a non-zero mask means **configured** -> bring up exactly those
/// ports, never re-probing the whitelist.
pub const LINK_SET: Field<u8> = Field::new(0x02, 0x00);

// ---------------------------------------------------------------------------
// The board-layout fields (`specs/board-model.md`): the per-pin store fields (packed
// `(port << 4) | pin` bytes, `0xFF` = unset = function absent) plus the non-pin board facts the
// boot validator consumes. Registered here per the spec's registered-at-landing decision: the
// validator (`crates/board`) is their first consumer; motor.current_sense/direction/align_offset
// are now registered too (the `board::MotorPlan` fold-back of `specs/motor-integration.md` carries
// them at boot for the motor bring-up). motor.pole_pairs stays enumerated-only, unregistered, until
// its consumer lands (the Phase-D speed-unit conversion; its model home is a board-model.md open
// question). Read at boot only, through the validator; a config
// write never re-pins before reboot. The BENIGN functions carry the fleet-uniform defaults; the
// motor groups and dead-time default to ABSENT (drive is an explicit configuration act).
// ---------------------------------------------------------------------------

/// `0xFF` = unset = the function is absent (`specs/board-model.md`, "The field vocabulary").
pub const PIN_ABSENT: u8 = 0xFF;

/// The power-latch pin (fleet default PB12; also asserted pre-mount as the compiled early-boot
/// value of this same default).
pub const BOARD_SELF_HOLD: Field<u8> = Field::new(0x40, 0x1C);
/// Battery-sense pin. Default ABSENT (`specs/sensing-and-safety.md`, "The battery word"): the pin
/// is fleet-uniform (PA4) but the SENSE is master-only, and a slave's PA4 reads stuck near 2.0 V,
/// which through the divider is a fictitious 62 V. The firmware has no role fact to tell the two
/// apart, so a defaulted pin would make every slave sense garbage; the master is CONFIGURED to
/// sense (stage `0x04`), a non-sensing board relays its peer's word.
pub const BOARD_VBATT: Field<u8> = Field::new(0x41, PIN_ABSENT);
/// Buzzer pin (fleet default PB9).
pub const BOARD_BUZZER: Field<u8> = Field::new(0x42, 0x19);
/// Indicator LEDs (fleet defaults PB3 / PA15 / PB4).
pub const LED_GREEN: Field<u8> = Field::new(0x43, 0x13);
pub const LED_ORANGE: Field<u8> = Field::new(0x44, 0x0F);
pub const LED_RED: Field<u8> = Field::new(0x45, 0x14);
/// Foot-pad rider-detection inputs (fleet defaults PA11 / PC15).
pub const PAD_A: Field<u8> = Field::new(0x46, 0x0B);
pub const PAD_B: Field<u8> = Field::new(0x47, 0x2F);
/// IMU bus pins (VARIANT function: no safe fleet default; absent until configured).
pub const IMU_SCL_PIN: Field<u8> = Field::new(0x48, PIN_ABSENT);
pub const IMU_SDA_PIN: Field<u8> = Field::new(0x49, PIN_ABSENT);
/// Hall inputs, per-motor via `Key.index` (motor groups are CONFIGURED, never defaulted).
pub const MOTOR_HALL_A: Field<u8> = Field::new(0x4A, PIN_ABSENT);
pub const MOTOR_HALL_B: Field<u8> = Field::new(0x4B, PIN_ABSENT);
pub const MOTOR_HALL_C: Field<u8> = Field::new(0x4C, PIN_ABSENT);
/// The advanced-timer gate set, per-motor via `Key.index` (configured, never defaulted).
pub const MOTOR_GATE_HI_A: Field<u8> = Field::new(0x4D, PIN_ABSENT);
pub const MOTOR_GATE_HI_B: Field<u8> = Field::new(0x4E, PIN_ABSENT);
pub const MOTOR_GATE_HI_C: Field<u8> = Field::new(0x4F, PIN_ABSENT);
pub const MOTOR_GATE_LO_A: Field<u8> = Field::new(0x50, PIN_ABSENT);
pub const MOTOR_GATE_LO_B: Field<u8> = Field::new(0x51, PIN_ABSENT);
pub const MOTOR_GATE_LO_C: Field<u8> = Field::new(0x52, PIN_ABSENT);
/// The two phase-current sense pins, per-motor via `Key.index` (configured, never defaulted).
///
/// `specs/motor-integration.md` bring-up step 5: the injected ADC group is programmed with these
/// two pins' ADC channels as its two ranks, so the CHANNELS are per-board data and cannot be a
/// compiled constant (the bench pair proves it: the F103 master senses on PB0/PA0 and the F130
/// slave on PB0/PB1). The boot validator derives each pin's ADC channel through the same
/// `Capabilities::adc_channel` query `board.vbatt` uses. Ordered A then B, matching the injected
/// rank order (rank 0 = phase A, rank 1 = phase B) and the gate channel order (CH0 = phase A).
///
/// Group rule (the `motor.dead_time` precedent): the pair is all-or-none, and it is present
/// exactly when [`MOTOR_CURRENT_SENSE`] is nonzero -- the capability declaration and its pin
/// realization may not disagree, so a board cannot claim current sense with no channels behind it
/// (or carry channels nothing declares).
pub const MOTOR_PHASE_A: Field<u8> = Field::new(0x54, PIN_ABSENT);
pub const MOTOR_PHASE_B: Field<u8> = Field::new(0x55, PIN_ABSENT);
/// The power-button sense pin (`specs/board-model.md` `board.button`; the `power_request`
/// producer, `specs/integration.md`'s input task). No fleet default is pinned yet, so unset
/// until configured.
pub const BOARD_BUTTON: Field<u8> = Field::new(0x53, PIN_ABSENT);
/// The IMU model index (`specs/imu.md`: 0 = no IMU fitted; the imu crate owns the numbering).
pub const IMU_MODEL: Field<u8> = Field::new(0x60, 0);
/// Per-axis zero-rate gyro bias, raw counts, indexed 0/1/2 = x/y/z (`specs/imu.md`, "Board-config
/// fields": the bench-captured calibration staged into `imu::Config.gyro_bias` at bring-up;
/// default 0 = uncalibrated). i32: the imu crate's bias word (counts fit i16, the type matches
/// the consumer).
pub const IMU_GYRO_BIAS: Field<i32> = Field::new(0x61, 0);
/// Per-axis IMU sign map, indexed 0..5 = `[ax, ay, az, gx, gy, gz]` (`specs/imu.md`, section 7.1;
/// staged into `imu::Config.sign` at bring-up beside [`IMU_GYRO_BIAS`]).
///
/// **Why this is a field and not a constant.** The sign map is the rotation between the IMU chip's
/// axes and the board frame: a per-board MOUNTING fact, exactly like the gyro bias next to it. It
/// was carried as a compiled default recovered from the stock board's mount (a 180-degree rotation
/// about Y), which is correct for that mount and wrong for any other. Both bench boards proved it
/// wrong for theirs on 2026-07-31: level and right way up, the conditioned up-axis read -0.970 g
/// (master) and -0.982 g (slave) and the attitude filter reported roll ~180 degrees, i.e. the
/// firmware believed both boards were upside down. An exercised per-board fact belongs in the
/// model that owns per-board configuration, not in a constant that cannot be right for two mounts
/// at once.
///
/// **0 = unset**, and the bring-up then falls back to that index of the compiled reference map
/// (`imu::Config::default().sign`). 0 is not a valid sign, so it is an unambiguous "not
/// configured" marker and the per-index defaults stay expressible through a single-default handle.
///
/// The map must be a proper ROTATION (determinant +1). A per-axis flip that is not, e.g. negating
/// only the up axis, yields a left-handed frame, and the fusion's accel-error cross product then
/// pushes the gyro integration the wrong way about some axis.
pub const IMU_AXIS_SIGN: Field<i32> = Field::new(0x65, 0);
/// The IMU axis ROLES, indexed `0 = UP`, `1 = PITCH_RATE` (`specs/imu.md`, `IMU_AXIS_ROLE`; staged
/// into `imu::Config.roles` at bring-up beside [`IMU_AXIS_SIGN`] and [`IMU_GYRO_BIAS`]): which CHIP
/// axis plays each body role, `1 = X`, `2 = Y`, `3 = Z`. The third role, FORWARD, is the remaining
/// axis.
///
/// **Why a second field beside the sign map.** A board standing on edge (the rover's wall mount) is a
/// 90 degree rotation from the flat stock mount: a signed axis PERMUTATION, which six signs alone
/// cannot hold. The roles plus the signs together are that signed permutation, factored into two
/// shapes; no rotation matrix is needed. The imu crate applies the roles to the whole sample, after
/// sign, bias and clamp, so every consumer reads body order.
///
/// **0 = unset**, and the bring-up falls back to that index of the compiled roles (UP = Z,
/// PITCH_RATE = Y, the stock flat mount, the identity), the [`IMU_AXIS_SIGN`] unset rule: a board with
/// nothing staged decodes exactly as before the field existed. The resolved frame must be a proper
/// rotation with distinct roles; `imu::Config::staged` refuses anything else at boot and the IMU is
/// not brought up (`board::BoardErrorKind::ImuFrame` in `BOARD_OBS`).
pub const IMU_AXIS_ROLE: Field<u8> = Field::new(0x68, 0);
/// Per-motor dead-time (raw DTG; 0 = unset; a configured gate group requires it nonzero).
pub const MOTOR_DEAD_TIME: Field<u8> = Field::new(0x64, 0);
/// Per-motor drive direction (`specs/commutation.md` six-step `Direction`; a board-mounting fact).
/// `0 = Forward`, nonzero = Reverse. Carried into `board::MotorPlan` at boot (the fold-back of
/// `specs/motor-integration.md`); the bring-up (slice 3) consumes it. Per-motor via `Key.index`.
pub const MOTOR_DIRECTION: Field<u8> = Field::new(0x62, 0);
/// Per-motor six-step align offset (`specs/commutation.md`; bench-swept 0..5, baked per the
/// tuning-into-code rule). Carried into `board::MotorPlan` at boot. Per-motor via `Key.index`.
pub const MOTOR_ALIGN_OFFSET: Field<u8> = Field::new(0x63, 0);
/// Per-motor phase-current-sense capability (`specs/commutation.md`, the FOC capability gate:
/// FOC is selectable only where phase-current sense exists). `0 = none`, nonzero = present.
/// Carried into `board::MotorPlan` at boot. **id 0x66, NOT the 0x61 the board-model.md field
/// table originally proposed: 0x61 was later claimed by [`IMU_GYRO_BIAS`], so current_sense moved
/// to the next free non-pin-block id (both specs folded to 0x66).** Per-motor via `Key.index`.
pub const MOTOR_CURRENT_SENSE: Field<u8> = Field::new(0x66, 0);

/// Per-motor current-sense CALIBRATION, stock current counts per amp of phase current
/// (`specs/motor-integration.md`, "The current-sense calibration"): what a `motor_current` count
/// MEANS, beside [`MOTOR_CURRENT_SENSE`] (0x66), which says where that current is sensed. The last
/// id of the motor-facts block 0x60..0x67.
///
/// **Why a field.** It is a property of the shunt and amplifier chain fitted to a particular board,
/// the [`BOARD_VBATT_CAL`] class, and it was the only analog scale left in a `const`:
/// `COUNTS_PER_AMP = 800` in `crates/firmware/src/motor.rs`, EFeru's `A2BIT_CONV = 50` ADC LSB per
/// amp times 16 counts per LSB, written for a class of mainboard and measured on none of ours.
///
/// **The default is 455**, the safer end of the 2026-10-09 energised bench gate
/// (`specs/bench-evidence/2026-10-09/current-limit/`: two sessions at the same operating point gave
/// 459 and 455, both UPPER BOUNDS because `peak` is a window maximum over noisy samples). A LOWER
/// coded scale yields fewer counts per real amp, so the limiter acts EARLIER in real current; 455
/// errs early where 800 errs late by about 1.8x, and erring late is the unsafe direction for a
/// backstop.
///
/// Read at boot into `board::MotorPlan`, never on the live tune lane. Range enforcement is NOT here
/// (the store validates type only): the boot seam (`firmware::motor::limit_counts`) clamps it to
/// 100..=819, the upper bound being what keeps `CURRENT_LIMIT_CEILING_MA * cal / 1000` inside the
/// `i16` the limit comparison holds. Per-motor via `Key.index`.
pub const MOTOR_CURRENT_CAL: Field<u16> = Field::new(0x67, 455);

/// The battery-sense calibration, counts to centivolts (`specs/sensing-and-safety.md`, "The field:
/// `board.vbatt_cal`"), indexed `0 = slope` (microvolts of rail per 12-bit count) and `1 = offset`
/// (centivolts, added): `centivolts = raw12 * slope / 10_000 + offset`.
///
/// The defaults ARE the 2026-10-08 four-point bench fit (`tools/vbatt-calibrate.py`: 25.200085 mV
/// per count, -47.6 mV), rounded to the field's units. The two standard master families measured
/// 31.17x and 31.27x dividers, so one number serves both; the field exists for the board whose
/// divider differs. A per-board CALIBRATION read at boot (the [`ATTITUDE_LEVEL_TRIM`] class), never
/// on the live tune lane. An [`IndexedField`] because its two indices default differently.
///
/// Range enforcement is NOT here (the store validates type only): the boot seam
/// (`orchestrator::battery::VbattCal::new`) clamps slope to 10000..32000 and offset to -500..500.
/// Consumer: the battery word's local sense, on boards whose plan carries `board.vbatt`.
pub const BOARD_VBATT_CAL: IndexedField<i16, 2> = IndexedField::new(0x69, [25200, -5]);

/// Per-board attitude LEVEL TRIM, centidegrees, indexed `0 = pitch`, `1 = roll`
/// (`specs/attitude.md`, "Output IIR and level trims"): the angle this board reads when it is
/// physically level, SUBTRACTED from the smoothed output before publish, so the balance loop's
/// zero is the board's own level and not the IMU's mounting error. Default 0 = untrimmed.
///
/// **Per board, not per fleet**, and the strongest evidence in the field set for it: stock kept
/// exactly this quantity in its 16-byte per-board cal page (`0x0800fc00` idx 6, centidegrees,
/// subtracted, with a live "level-here" command writing it), and the recovered pair differs by
/// 5.71 degrees (master `+305`, slave `-266`) because the two halves are MIRROR-MOUNTED, which is
/// how both our pairs are mounted too. The two stock images are byte-identical everywhere ELSE, so
/// the mirror mounting is absorbed entirely here rather than by any code difference
/// (`BalanceAgain/findings/attitude_constants.md`).
///
/// Note the asymmetry with the filter GAIN, which is deliberately NOT a field: the same recovery
/// shows `Kp` and the fusion gyro bias identical on both halves, i.e. stock stored the trims per
/// board and hardcoded the gain. This field follows the evidence, not a general urge to make
/// tuning configurable.
///
/// Centidegrees rather than degrees because that is the unit stock stored, the unit its
/// level-here command rounds to, and an integer the host tool can stage exactly; the consumer
/// divides by 100 into its fixed-point output type. i16 covers the full +/-180 range at that
/// scale with the same width stock used. Index 1 (roll) has no stock counterpart (the stock roll
/// channel carried no trim; its idx 7 is a different accel-inclination quantity), so it is our own
/// per-unit trim on the fused roll, defaulting to 0.
pub const ATTITUDE_LEVEL_TRIM: Field<i16> = Field::new(0x70, 0);

/// The balance-PID gain triple of profile A, rider-present (`specs/rider-ui.md` section 4),
/// indexed `0 = kp`, `1 = bk` (the battery/rate coefficient), `2 = pr` (the derivative rate word).
///
/// **Why these are fields when the filter gain next door is not.** [`ATTITUDE_LEVEL_TRIM`] argues
/// that the field set follows the stock evidence, and stock hardcoded its gains; that stance is
/// overridden HERE and only here, by an owner decision (`specs/rider-ui.md` D2/D3): the two RUN
/// profiles become rider-tunable from the app, with a live RAM lane in front of them. The
/// engagement seeds stock also carried (`control::STANDBY_SET`, `control::ARMING_SEED_ORIENT_NZ`)
/// stay compile-time constants: they are engagement safety seeds, not tuning targets.
///
/// The defaults ARE today's compiled constants (`control::RUN_PROFILE_A`), so a board that has
/// never been tuned behaves exactly as it did before this field existed. `store`'s own tests pin
/// that equality against the control crate through a dev-dependency, so the two cannot drift.
///
/// i16 because the stock values (6000/2000/40) and the seam's ranges fit it with room, and the
/// wire echoes a fixed 2-byte value. Range enforcement is NOT here (the store validates type
/// only): it lives at the tune seam, `control::GainShadow`, which every writer goes through.
pub const CONTROL_GAIN_A: IndexedField<i16, 3> = IndexedField::new(0x71, [6000, 2000, 40]);
/// The balance-PID gain triple of profile B (`control::PROFILE_B`), same indices as
/// [`CONTROL_GAIN_A`]. Selected when the rider level is clear (`control::select_profile`).
pub const CONTROL_GAIN_B: IndexedField<i16, 3> = IndexedField::new(0x72, [3000, 1000, 30]);

/// The balance-mode drive input's bound and rate (`specs/control.md` (h)), indexed `0 = lean_max`
/// (centidegrees of equilibrium pitch at full stick) and `1 = lean_slew` (centidegrees per tick).
///
/// **Default `lean_max` 0 = disabled**: the drive value is discarded exactly as it was before this
/// field existed, so an unstaged board is byte-identical in behaviour; the rover stages a nonzero
/// value. `lean_slew` defaults to 4 (10 degrees/s at 250 Hz). Stated in physical units, never in
/// the shaper's `off` units, because `off` scales with `kp` and `kp` is tuned live.
///
/// A boot-read protection parameter, deliberately NOT on the live tune lane (whose allowlist is
/// exactly [`CONTROL_GAIN_A`] / [`CONTROL_GAIN_B`]). Range enforcement is NOT here (the store
/// validates type only): the boot seam (`control::DriveLean::new`) clamps `lean_max` to 0..1500
/// and `lean_slew` to 1..100. An [`IndexedField`] because its two indices default differently.
pub const CONTROL_DRIVE_LEAN: IndexedField<i16, 2> = IndexedField::new(0x73, [0, 4]);

/// The inclusive upper bound each balance-PID gain index accepts (`specs/rider-ui.md` section 4,
/// "Ranges"), indexed like the gain triples (`0 = kp`, `1 = bk`, `2 = pr`) and shared by both
/// profiles; the lower bound is the constant `control::GAIN_MIN` (0).
///
/// A field rather than a constant because the stock-x3 table was derived before there was a
/// rover plant to check it against, so the owner moves the maxima at runtime. The defaults ARE
/// `control::DEFAULT_GAIN_MAX` (pinned by `store`'s tests). Boot-read into `control::GainShadow`
/// (each index taken as `max(0, value)`; a stored gain above its maximum clamps on the way in),
/// so a written maximum applies from the next power-cycle. NOT on the live tune lane, whose
/// allowlist is exactly [`CONTROL_GAIN_A`] / [`CONTROL_GAIN_B`]; the lane refuses against the
/// boot-read maxima. An [`IndexedField`] because its three indices default differently.
pub const CONTROL_GAIN_MAX: IndexedField<i16, 3> = IndexedField::new(0x74, [20000, 10000, 1000]);

// The store-test fields, value consts, and scenario ids are gated behind `test-fields` (off by
// default) so they do NOT compile into a production build: the production field set is exactly the
// genuine tunables above. The `store-test` firmware, the emulator-runner store scenarios, and the
// store's own host tests enable the feature.
//
// The STR variable-value round-trip reuses `DEVICE_NAME` (its "Hoverboard" default differs from the
// test literal `T_STR_VAL`, so the no-write negative control still distinguishes a real write from
// the default), so there is no dedicated test STR field.

/// The store-test scalar field (drives every tier; see the spec "store test function"). A reserved
/// U32 field exposed as a typed handle; [`T_VAL`] is the planted value the host re-derives.
#[cfg(feature = "test-fields")]
pub const T_KEY: Field<u32> = Field::new(0xFE, 0);
/// The scalar value the persist/recovery scenarios set and the host re-derives.
#[cfg(feature = "test-fields")]
pub const T_VAL: u32 = 0x00C0_FFEE;

/// The STR value the variable-value scenario writes to [`DEVICE_NAME`] and the host re-derives. It
/// differs from `DEVICE_NAME`'s "Hoverboard" default so the no-write negative control is detectable.
#[cfg(feature = "test-fields")]
pub const T_STR_VAL: &str = "hoverboard-x1";

/// Reserved test BLOB field for the variable-value round-trip scenario (device-written test blob).
/// Kept dedicated because no genuine tunable has a non-empty-distinguishable default (`SOME_BLOB`'s
/// default is `&[]`).
#[cfg(feature = "test-fields")]
pub const T_BLOB: BlobField = BlobField::new(0xFD, &[]);
/// The BLOB value the variable-value scenario sets and the host re-derives.
#[cfg(feature = "test-fields")]
pub const T_BLOB_VAL: &[u8] = &[0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03];

// One scenario id per store-test scenario (the host packs `(scenario << 16) | phase`). The host
// drives the whole scenario x phase matrix over `CMD_ADDR`; adding a case is a new scenario arm.

/// Persist-survives-reboot: phase 0 sets `T_KEY = T_VAL`, phase 1 cold-mounts and reads it back.
#[cfg(feature = "test-fields")]
pub const PERSIST: u32 = 0;
/// Variable-value round trip (device-written): phase 0 `set_str`(DEVICE_NAME) + `set_bytes`(T_BLOB);
/// phase 1 reads each back into `TestResult.buf`/`len`. The phase's low bit picks STR (1) vs BLOB (2).
#[cfg(feature = "test-fields")]
pub const VAR_VALUE: u32 = 1;
/// Compaction-preserves-keys: the host plants a multi-record region, the device cold-mounts and the
/// host checks every latest-per-key survives (read via the scalar/variable readback).
#[cfg(feature = "test-fields")]
pub const COMPACT: u32 = 2;
/// Torn-payload recovery: host plants a half-written payload, the device cold-mounts and reads the
/// last good `T_KEY` value (which must equal `T_VAL`).
#[cfg(feature = "test-fields")]
pub const TORN_PAYLOAD: u32 = 3;
/// Torn-header auto-compaction: host plants a torn header, the device cold-mounts (auto-compacts) and
/// reads the surviving `T_KEY` value.
#[cfg(feature = "test-fields")]
pub const TORN_HEADER: u32 = 4;
/// Full -> compact -> retry: host plants a near-full active page, the device sets `T_KEY` (which
/// returns `Full`), compacts, retries, and reads it back.
#[cfg(feature = "test-fields")]
pub const FULL: u32 = 5;
/// The DYNAMIC `Key`/[`Value`] path, the one L3's `CONFIG_WRITE` / `CONFIG_READ` actually calls:
/// phase 0 `set_value(T_KEY.key(), Value::U32(T_VAL))`, phase 1 cold-mounts and `get_value`s it back.
///
/// It is a distinct scenario from [`PERSIST`] rather than a variant of it because the two differ in
/// exactly one thing: the dynamic path goes through [`lookup`] and the typed path does not. That makes
/// the PAIR a controlled measurement of what the registry lookup costs the stack, which is the whole
/// point of `dynamic_config_write_costs_no_extra_stack_chip1k` in the emulator suite. Until this
/// existed, no tier-2 or tier-3 test drove `set_value` / `get_value` at all, which is how a 920 B
/// `lookup` frame reached silicon unnoticed (`specs/bench-evidence/2026-08-13/negative-control.md`).
#[cfg(feature = "test-fields")]
pub const DYN_VALUE: u32 = 6;

// The uniqueness assertion must cover exactly the ids that actually compile. With `test-fields` the
// reserved test ids are included and still collision-checked; without it they are absent.
#[cfg(not(feature = "test-fields"))]
field_ids! {
    0x01, // NODE_ADDRESS
    0x02, // LINK_SET
    0x10, // DEVICE_NAME
    0x20, // MOTOR_CURRENT_LIMIT
    0x21, // MOTOR_METHOD
    0x22, // CONTROL_MODE
    0x23, // CONTROL_RIDER_REQUIRED
    0x24, // CONTROL_BATTERY_FLOOR
    0x30, // SOME_BLOB
    0x40, // BOARD_SELF_HOLD
    0x41, // BOARD_VBATT
    0x42, // BOARD_BUZZER
    0x43, // LED_GREEN
    0x44, // LED_ORANGE
    0x45, // LED_RED
    0x46, // PAD_A
    0x47, // PAD_B
    0x48, // IMU_SCL_PIN
    0x49, // IMU_SDA_PIN
    0x4A, // MOTOR_HALL_A
    0x4B, // MOTOR_HALL_B
    0x4C, // MOTOR_HALL_C
    0x4D, // MOTOR_GATE_HI_A
    0x4E, // MOTOR_GATE_HI_B
    0x4F, // MOTOR_GATE_HI_C
    0x50, // MOTOR_GATE_LO_A
    0x51, // MOTOR_GATE_LO_B
    0x52, // MOTOR_GATE_LO_C
    0x53, // BOARD_BUTTON
    0x54, // MOTOR_PHASE_A
    0x55, // MOTOR_PHASE_B
    0x60, // IMU_MODEL
    0x61, // IMU_GYRO_BIAS
    0x65, // IMU_AXIS_SIGN
    0x68, // IMU_AXIS_ROLE
    0x62, // MOTOR_DIRECTION
    0x63, // MOTOR_ALIGN_OFFSET
    0x64, // MOTOR_DEAD_TIME
    0x66, // MOTOR_CURRENT_SENSE
    0x67, // MOTOR_CURRENT_CAL
    0x69, // BOARD_VBATT_CAL
    0x70, // ATTITUDE_LEVEL_TRIM
    0x71, // CONTROL_GAIN_A
    0x72, // CONTROL_GAIN_B
    0x73, // CONTROL_DRIVE_LEAN
    0x74, // CONTROL_GAIN_MAX
}

#[cfg(feature = "test-fields")]
field_ids! {
    0x01, // NODE_ADDRESS
    0x02, // LINK_SET
    0x10, // DEVICE_NAME
    0x20, // MOTOR_CURRENT_LIMIT
    0x21, // MOTOR_METHOD
    0x22, // CONTROL_MODE
    0x23, // CONTROL_RIDER_REQUIRED
    0x24, // CONTROL_BATTERY_FLOOR
    0x30, // SOME_BLOB
    0x40, // BOARD_SELF_HOLD
    0x41, // BOARD_VBATT
    0x42, // BOARD_BUZZER
    0x43, // LED_GREEN
    0x44, // LED_ORANGE
    0x45, // LED_RED
    0x46, // PAD_A
    0x47, // PAD_B
    0x48, // IMU_SCL_PIN
    0x49, // IMU_SDA_PIN
    0x4A, // MOTOR_HALL_A
    0x4B, // MOTOR_HALL_B
    0x4C, // MOTOR_HALL_C
    0x4D, // MOTOR_GATE_HI_A
    0x4E, // MOTOR_GATE_HI_B
    0x4F, // MOTOR_GATE_HI_C
    0x50, // MOTOR_GATE_LO_A
    0x51, // MOTOR_GATE_LO_B
    0x52, // MOTOR_GATE_LO_C
    0x53, // BOARD_BUTTON
    0x54, // MOTOR_PHASE_A
    0x55, // MOTOR_PHASE_B
    0x60, // IMU_MODEL
    0x61, // IMU_GYRO_BIAS
    0x65, // IMU_AXIS_SIGN
    0x68, // IMU_AXIS_ROLE
    0x62, // MOTOR_DIRECTION
    0x63, // MOTOR_ALIGN_OFFSET
    0x64, // MOTOR_DEAD_TIME
    0x66, // MOTOR_CURRENT_SENSE
    0x67, // MOTOR_CURRENT_CAL
    0x69, // BOARD_VBATT_CAL
    0x70, // ATTITUDE_LEVEL_TRIM
    0x71, // CONTROL_GAIN_A
    0x72, // CONTROL_GAIN_B
    0x73, // CONTROL_DRIVE_LEAN
    0x74, // CONTROL_GAIN_MAX
    0xFD, // T_BLOB (store-test reserved)
    0xFE, // T_KEY  (store-test reserved)
}

// ---------------------------------------------------------------------------
// The enumerable registry: the runtime `field_id -> (Type, default)` view of the field set, derived
// from the typed handles so the handle stays the single source of truth (no parallel data table to
// drift). This is the deferred Layer-3 dependency, un-deferred for `net`'s `CONFIG_*`: a schema-less
// controller looks a field up by raw `field_id` to learn its `Type` (to decode a value and validate a
// write) and its default (returned when the key is absent). See `specs/storage-layer.md`.
// ---------------------------------------------------------------------------

/// One field's runtime descriptor: its permanent `field_id`, storage [`Type`], and default [`Value`].
/// Built from a typed handle via its `def()` (so a field's id/type/default are still written once).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldDef {
    /// The field's permanent id.
    pub field_id: u8,
    /// The index this entry describes. 0 for every ordinary field (one entry per id); an index
    /// family declared with [`IndexedField::new`] contributes one entry per index, because its
    /// default differs by index. It costs nothing: the byte sits in padding the struct already
    /// carried.
    pub index: u8,
    /// The field's storage type (decodes a stored value; validates a `CONFIG_WRITE` tag).
    pub kind: Type,
    /// This entry's default, returned when the key is absent.
    pub default: Value<'static>,
}

/// The number of ENTRIES in the registry, which is the declared field count plus the extra
/// per-index entries the [`IndexedField`] families contribute (the two gain families and
/// [`CONTROL_GAIN_MAX`]: one id, three defaults each, so two extra entries each; [`BOARD_VBATT_CAL`] and [`CONTROL_DRIVE_LEAN`]:
/// one id, two defaults, so one extra each). Tracks the field set under each `test-fields` configuration.
#[cfg(not(feature = "test-fields"))]
pub const REGISTRY_LEN: usize = 46 + 8;
/// The number of registry entries (with the reserved store-test fields); see the non-test twin.
#[cfg(feature = "test-fields")]
pub const REGISTRY_LEN: usize = 48 + 8;

/// The full field registry, derived from the typed handles. Enumerable (iterate it) and the basis for
/// [`lookup`].
///
/// A `static` in flash, deliberately, and this is a **stack** decision rather than a flash one. It was
/// a `fn registry() -> [FieldDef; REGISTRY_LEN]` returning the array BY VALUE, so every call
/// materialized all 37 x 24 B of it into the caller's frame. `lookup` calls it on every dynamic
/// `get_value` / `set_value`, which put a **920 B** frame at the bottom of the deepest chain in the
/// image (`service_loop -> ingest -> apply_write -> set_value -> lookup`) and took the measured margin
/// on the bench master to 128 B of a 2,284 B painted stack, under the 250 B floor
/// (`specs/bench-evidence/2026-08-13/negative-control.md`). Held in flash and scanned by reference, the
/// same table costs the stack nothing.
///
/// The handles stay the single source of truth: each entry is that handle's own `def()`.
pub static REGISTRY: [FieldDef; REGISTRY_LEN] = [
    NODE_ADDRESS.def(),
    LINK_SET.def(),
    DEVICE_NAME.def(),
    MOTOR_CURRENT_LIMIT.def(),
    MOTOR_METHOD.def(),
    CONTROL_MODE.def(),
    CONTROL_RIDER_REQUIRED.def(),
    CONTROL_BATTERY_FLOOR.def(),
    SOME_BLOB.def(),
    BOARD_SELF_HOLD.def(),
    BOARD_VBATT.def(),
    BOARD_BUZZER.def(),
    LED_GREEN.def(),
    LED_ORANGE.def(),
    LED_RED.def(),
    PAD_A.def(),
    PAD_B.def(),
    IMU_SCL_PIN.def(),
    IMU_SDA_PIN.def(),
    MOTOR_HALL_A.def(),
    MOTOR_HALL_B.def(),
    MOTOR_HALL_C.def(),
    MOTOR_GATE_HI_A.def(),
    MOTOR_GATE_HI_B.def(),
    MOTOR_GATE_HI_C.def(),
    MOTOR_GATE_LO_A.def(),
    MOTOR_GATE_LO_B.def(),
    MOTOR_GATE_LO_C.def(),
    MOTOR_PHASE_A.def(),
    MOTOR_PHASE_B.def(),
    BOARD_BUTTON.def(),
    IMU_MODEL.def(),
    IMU_GYRO_BIAS.def(),
    IMU_AXIS_SIGN.def(),
    IMU_AXIS_ROLE.def(),
    MOTOR_DIRECTION.def(),
    MOTOR_ALIGN_OFFSET.def(),
    MOTOR_DEAD_TIME.def(),
    MOTOR_CURRENT_SENSE.def(),
    MOTOR_CURRENT_CAL.def(),
    ATTITUDE_LEVEL_TRIM.def(),
    // The index families whose default differs per index (`IndexedField`): one entry each, so an
    // absent key reads ITS index's default on the dynamic path as well as the typed one.
    BOARD_VBATT_CAL.at(0).def(),
    BOARD_VBATT_CAL.at(1).def(),
    CONTROL_GAIN_A.at(0).def(),
    CONTROL_GAIN_A.at(1).def(),
    CONTROL_GAIN_A.at(2).def(),
    CONTROL_GAIN_B.at(0).def(),
    CONTROL_GAIN_B.at(1).def(),
    CONTROL_GAIN_B.at(2).def(),
    CONTROL_DRIVE_LEAN.at(0).def(),
    CONTROL_DRIVE_LEAN.at(1).def(),
    CONTROL_GAIN_MAX.at(0).def(),
    CONTROL_GAIN_MAX.at(1).def(),
    CONTROL_GAIN_MAX.at(2).def(),
    #[cfg(feature = "test-fields")]
    T_BLOB.def(),
    #[cfg(feature = "test-fields")]
    T_KEY.def(),
];

/// Look a field up by its raw `field_id`, or `None` if no field declares it (an `UnknownKey` on the
/// dynamic path). Linear over the small registry.
///
/// It scans [`REGISTRY`] **by reference** and copies out only the matching 24 B [`FieldDef`]. Iterating
/// by value (`REGISTRY.into_iter()`, or the old `registry()` call) would first copy all 888 B of the
/// table into this frame, which is the regression this shape exists to prevent. The gate on that is
/// `dynamic_config_write_costs_no_extra_stack_chip1k` in `crates/emulator-runner`, which measures the
/// real image's stack excursion under Unicorn; there is no host-side gate, and
/// `crates/store/src/tests.rs` records why one is not possible.
pub fn lookup(field_id: u8) -> Option<FieldDef> {
    lookup_key(Key { field_id, index: 0 })
}

/// Look a field up by its full [`Key`]: the entry for THIS index when the field declares one
/// ([`IndexedField`]), else the field's base entry. `None` if no field declares the id.
///
/// The index only ever selects a different DEFAULT; type and id are per-field. This is the form
/// the dynamic `get_value` / `set_value` path uses, because an absent key must read the default of
/// the index that was asked for, not of index 0. Both passes scan [`REGISTRY`] by reference and
/// copy out only the matching 24 B [`FieldDef`], for the reason [`lookup`] records.
pub fn lookup_key(key: Key) -> Option<FieldDef> {
    let mut base = None;
    for d in REGISTRY.iter() {
        if d.field_id != key.field_id {
            continue;
        }
        if d.index == key.index {
            return Some(*d);
        }
        if d.index == 0 {
            base = Some(*d);
        }
    }
    base
}

#[cfg(test)]
mod registry_tests {
    use super::*;

    #[test]
    fn registry_has_every_declared_field_with_its_handle_type_and_default() {
        let reg = &REGISTRY;
        assert_eq!(reg.len(), REGISTRY_LEN);
        // One entry per declared id, plus the extra per-index entries the `indexed` families add.
        let extra = (CONTROL_GAIN_A.len() - 1)
            + (CONTROL_GAIN_B.len() - 1)
            + (BOARD_VBATT_CAL.len() - 1)
            + (CONTROL_DRIVE_LEAN.len() - 1)
            + (CONTROL_GAIN_MAX.len() - 1);
        assert_eq!(reg.len(), FIELD_IDS.len() + extra);
        // Every declared id is present, and no entry carries an id nothing declares.
        for id in FIELD_IDS {
            assert!(
                lookup(*id).is_some(),
                "declared id {id:#04x} absent from REGISTRY"
            );
        }
        for d in reg {
            assert!(
                FIELD_IDS.contains(&d.field_id),
                "REGISTRY entry {:#04x} is not a declared id",
                d.field_id
            );
        }
        // Spot-check the genuine tunables: id + kind + default come straight from the handle.
        let m = lookup(MOTOR_CURRENT_LIMIT.id()).unwrap();
        assert_eq!(m.kind, Type::U32);
        assert_eq!(m.default, Value::U32(10_000));
        let n = lookup(DEVICE_NAME.id()).unwrap();
        assert_eq!(n.kind, Type::Str);
        assert_eq!(n.default, Value::Str(b"Hoverboard"));
        let b = lookup(SOME_BLOB.id()).unwrap();
        assert_eq!(b.kind, Type::Blob);
        assert_eq!(b.default, Value::Bytes(&[]));
        // The rider requirement: a u8 beside CONTROL_MODE, default 1 = required.
        let r = lookup(CONTROL_RIDER_REQUIRED.id()).unwrap();
        assert_eq!(CONTROL_RIDER_REQUIRED.id(), 0x23);
        assert_eq!(r.kind, Type::U8);
        assert_eq!(r.default, Value::U8(1));
        // The current-sense calibration (0x67), the field set's first u16: counts per amp, the
        // 2026-10-09 bench figure as its default (`specs/motor-integration.md`).
        let c = lookup(MOTOR_CURRENT_CAL.id()).unwrap();
        assert_eq!(MOTOR_CURRENT_CAL.id(), 0x67);
        assert_eq!(c.kind, Type::U16);
        assert_eq!(c.default, Value::U16(455));
        // The low-battery floor: an i16 beside it, default 2400 cV.
        let f = lookup(CONTROL_BATTERY_FLOOR.id()).unwrap();
        assert_eq!(CONTROL_BATTERY_FLOOR.id(), 0x24);
        assert_eq!(f.kind, Type::I16);
        assert_eq!(f.default, Value::I16(2400));
    }

    #[test]
    fn lookup_of_an_undeclared_id_is_none() {
        assert!(lookup(0x99).is_none());
    }

    #[test]
    fn every_registry_key_is_unique() {
        let reg = &REGISTRY;
        for (i, a) in reg.iter().enumerate() {
            for b in &reg[i + 1..] {
                assert_ne!(
                    (a.field_id, a.index),
                    (b.field_id, b.index),
                    "two REGISTRY entries describe the same key"
                );
            }
        }
    }

    /// The per-index defaults resolve per index on BOTH paths, and an index the family does not
    /// declare falls back to the base entry (the ordinary families' single-default behaviour).
    #[test]
    fn an_indexed_family_resolves_its_default_per_index() {
        // Typed path (the boot seam's form).
        assert_eq!(CONTROL_GAIN_A.at(0).default(), 6000);
        assert_eq!(CONTROL_GAIN_A.at(1).default(), 2000);
        assert_eq!(CONTROL_GAIN_A.at(2).default(), 40);
        assert_eq!(CONTROL_GAIN_B.at(1).default(), 1000);
        // Dynamic path (what CONFIG_READ of an unwritten key returns).
        let d = |f: u8, i: u8| {
            lookup_key(Key {
                field_id: f,
                index: i,
            })
            .unwrap()
            .default
        };
        assert_eq!(d(0x71, 0), Value::I16(6000));
        assert_eq!(d(0x71, 1), Value::I16(2000));
        assert_eq!(d(0x71, 2), Value::I16(40));
        assert_eq!(d(0x72, 0), Value::I16(3000));
        assert_eq!(d(0x72, 1), Value::I16(1000));
        assert_eq!(d(0x72, 2), Value::I16(30));
        // The battery-sense calibration: slope then offset, the bench fit's defaults.
        assert_eq!(BOARD_VBATT_CAL.at(0).default(), 25200);
        assert_eq!(BOARD_VBATT_CAL.at(1).default(), -5);
        assert_eq!(d(0x69, 0), Value::I16(25200));
        assert_eq!(d(0x69, 1), Value::I16(-5));
        // The drive lean: lean_max 0 (disabled), lean_slew 4.
        assert_eq!(CONTROL_DRIVE_LEAN.at(0).default(), 0);
        assert_eq!(CONTROL_DRIVE_LEAN.at(1).default(), 4);
        assert_eq!(d(0x73, 0), Value::I16(0));
        assert_eq!(d(0x73, 1), Value::I16(4));
        // The gain maxima: kp 20000, bk 10000, pr 1000.
        assert_eq!(CONTROL_GAIN_MAX.at(2).default(), 1000);
        assert_eq!(d(0x74, 0), Value::I16(20000));
        assert_eq!(d(0x74, 1), Value::I16(10000));
        assert_eq!(d(0x74, 2), Value::I16(1000));
        // Past the declared end, and an ordinary family at any index: the base default.
        assert_eq!(d(0x71, 7), Value::I16(6000));
        assert_eq!(CONTROL_GAIN_A.at(7).default(), 6000);
        assert_eq!(CONTROL_GAIN_A.at(7).key().index, 0);
        assert_eq!(d(IMU_GYRO_BIAS.id(), 2), Value::I32(0));
        assert_eq!(IMU_GYRO_BIAS.at(2).default(), 0);
        // The current-sense calibration: per-motor with ONE default, so BOTH motors read 455
        // from an unstaged 0x67 (the per-motor single-default class, not an `IndexedField`).
        assert_eq!(d(0x67, 0), Value::U16(455));
        assert_eq!(d(0x67, 1), Value::U16(455));
        assert_eq!(MOTOR_CURRENT_CAL.at(1).default(), 455);
        assert_eq!(MOTOR_CURRENT_CAL.at(1).key().index, 1);
        // The IMU axis roles: one u8 default (0 = unset) across both indices.
        assert_eq!(d(0x68, 0), Value::U8(0));
        assert_eq!(d(0x68, 1), Value::U8(0));
        assert_eq!(IMU_AXIS_ROLE.at(1).default(), 0);
        assert_eq!(IMU_AXIS_ROLE.at(1).key().index, 1);
    }
}
