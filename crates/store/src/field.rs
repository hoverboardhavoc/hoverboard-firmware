//! Typed field handles, the firmware's compile-checked front door, and the curated field set.
//!
//! Each registered field is a typed `const` handle whose Rust type is the field's storage type, so
//! misuse does not compile: `get` only accepts a [`Field<T>`] and yields `T`, `get_str` only accepts a
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

// `Field<T>::def()` is NOT here. It is emitted per concrete scalar by `impl_scalar_int!` in `key.rs`,
// beside the `Scalar` impl that already owns the `T -> Type -> Value` mapping, because a generic
// `def()` cannot be `const`: it would have to call a trait method to lift the typed default into a
// `Value`, and const trait methods do not exist on this toolchain. `const` is the whole point, since
// it is what makes [`REGISTRY`] a `static`.

/// A `STR` field handle, carrying a `&'static str` default. STR and BLOB are byte-identical on
/// flash; this differs from [`BlobField`] only in the return type and the UTF-8 check on read.
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

    pub const fn default(self) -> &'static str {
        self.default
    }

    /// This field's runtime [`FieldDef`]. `const`, so it can build [`REGISTRY`] in flash.
    pub const fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            kind: Type::Str,
            default: Value::Str(self.default),
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
/// Battery-sense pin (fleet default PA4; masters sense, slaves read the link).
pub const BOARD_VBATT: Field<u8> = Field::new(0x41, 0x04);
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
    0x62, // MOTOR_DIRECTION
    0x63, // MOTOR_ALIGN_OFFSET
    0x64, // MOTOR_DEAD_TIME
    0x66, // MOTOR_CURRENT_SENSE
    0x70, // ATTITUDE_LEVEL_TRIM
}

#[cfg(feature = "test-fields")]
field_ids! {
    0x01, // NODE_ADDRESS
    0x02, // LINK_SET
    0x10, // DEVICE_NAME
    0x20, // MOTOR_CURRENT_LIMIT
    0x21, // MOTOR_METHOD
    0x22, // CONTROL_MODE
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
    0x62, // MOTOR_DIRECTION
    0x63, // MOTOR_ALIGN_OFFSET
    0x64, // MOTOR_DEAD_TIME
    0x66, // MOTOR_CURRENT_SENSE
    0x70, // ATTITUDE_LEVEL_TRIM
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
    /// The field's storage type (decodes a stored value; validates a `CONFIG_WRITE` tag).
    pub kind: Type,
    /// The field's default, returned when the key is absent.
    pub default: Value<'static>,
}

/// The number of fields in the registry. Tracks the field set under each `test-fields` configuration.
#[cfg(not(feature = "test-fields"))]
pub const REGISTRY_LEN: usize = 37;
/// The number of fields in the registry (with the reserved store-test fields).
#[cfg(feature = "test-fields")]
pub const REGISTRY_LEN: usize = 39;

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
    MOTOR_DIRECTION.def(),
    MOTOR_ALIGN_OFFSET.def(),
    MOTOR_DEAD_TIME.def(),
    MOTOR_CURRENT_SENSE.def(),
    ATTITUDE_LEVEL_TRIM.def(),
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
/// table into this frame, which is the regression this shape exists to prevent; the
/// `lookup_costs_no_more_than_one_fielddef_of_stack` test measures that it does not.
pub fn lookup(field_id: u8) -> Option<FieldDef> {
    REGISTRY.iter().find(|d| d.field_id == field_id).copied()
}

#[cfg(test)]
mod registry_tests {
    use super::*;

    #[test]
    fn registry_has_every_declared_field_with_its_handle_type_and_default() {
        let reg = &REGISTRY;
        assert_eq!(reg.len(), REGISTRY_LEN);
        assert_eq!(reg.len(), FIELD_IDS.len()); // one entry per declared id
                                                // Spot-check the genuine tunables: id + kind + default come straight from the handle.
        let m = lookup(MOTOR_CURRENT_LIMIT.id()).unwrap();
        assert_eq!(m.kind, Type::U32);
        assert_eq!(m.default, Value::U32(10_000));
        let n = lookup(DEVICE_NAME.id()).unwrap();
        assert_eq!(n.kind, Type::Str);
        assert_eq!(n.default, Value::Str("Hoverboard"));
        let b = lookup(SOME_BLOB.id()).unwrap();
        assert_eq!(b.kind, Type::Blob);
        assert_eq!(b.default, Value::Bytes(&[]));
    }

    #[test]
    fn lookup_of_an_undeclared_id_is_none() {
        assert!(lookup(0x99).is_none());
    }

    #[test]
    fn every_registry_id_is_unique() {
        let reg = &REGISTRY;
        for (i, a) in reg.iter().enumerate() {
            for b in &reg[i + 1..] {
                assert_ne!(a.field_id, b.field_id);
            }
        }
    }
}
