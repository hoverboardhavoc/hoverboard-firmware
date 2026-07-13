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
    pub fn default(self) -> T {
        self.default
    }

    /// The default as a dynamic [`Value`] (for the registry / the `CONFIG_*` path).
    pub fn default_value(self) -> Value<'static> {
        self.default.to_value()
    }

    /// This field's runtime [`FieldDef`] (id + storage type + default value), derived from the handle
    /// - the handle stays the single source of truth.
    pub fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            kind: T::KIND,
            default: self.default_value(),
        }
    }
}

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

    /// The default as a dynamic [`Value`].
    pub fn default_value(self) -> Value<'static> {
        Value::Str(self.default)
    }

    /// This field's runtime [`FieldDef`].
    pub fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            kind: Type::Str,
            default: self.default_value(),
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

    /// The default as a dynamic [`Value`].
    pub fn default_value(self) -> Value<'static> {
        Value::Bytes(self.default)
    }

    /// This field's runtime [`FieldDef`].
    pub fn def(self) -> FieldDef {
        FieldDef {
            field_id: self.field_id,
            kind: Type::Blob,
            default: self.default_value(),
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
pub const DEVICE_NAME: StrField = StrField::new(0x10, "hoverboard");
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
// validator (`crates/board`) is their first consumer; the vocabulary rows the validator does NOT
// consume (motor.current_sense/direction/align_offset/pole_pairs) stay enumerated in the spec,
// unregistered, until their consumers land. Read at boot only, through the validator; a config
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
/// The IMU model index (`specs/imu.md`: 0 = no IMU fitted; the imu crate owns the numbering).
pub const IMU_MODEL: Field<u8> = Field::new(0x60, 0);
/// Per-motor dead-time (raw DTG; 0 = unset; a configured gate group requires it nonzero).
pub const MOTOR_DEAD_TIME: Field<u8> = Field::new(0x64, 0);

// The store-test fields, value consts, and scenario ids are gated behind `test-fields` (off by
// default) so they do NOT compile into a production build: the production field set is exactly the
// genuine tunables above. The `store-test` firmware, the emulator-runner store scenarios, and the
// store's own host tests enable the feature.
//
// The STR variable-value round-trip reuses `DEVICE_NAME` (its "hoverboard" default differs from the
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
/// differs from `DEVICE_NAME`'s "hoverboard" default so the no-write negative control is detectable.
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
    0x60, // IMU_MODEL
    0x64, // MOTOR_DEAD_TIME
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
    0x60, // IMU_MODEL
    0x64, // MOTOR_DEAD_TIME
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
pub const REGISTRY_LEN: usize = 28;
/// The number of fields in the registry (with the reserved store-test fields).
#[cfg(feature = "test-fields")]
pub const REGISTRY_LEN: usize = 30;

/// The full field registry, derived from the typed handles. Enumerable (iterate the returned array)
/// and the basis for [`lookup`]. A function (not a `const`) because a handle's typed default is lifted
/// into a `Value` at runtime; the array is small and the call is cheap.
pub fn registry() -> [FieldDef; REGISTRY_LEN] {
    [
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
        IMU_MODEL.def(),
        MOTOR_DEAD_TIME.def(),
        #[cfg(feature = "test-fields")]
        T_BLOB.def(),
        #[cfg(feature = "test-fields")]
        T_KEY.def(),
    ]
}

/// Look a field up by its raw `field_id`, or `None` if no field declares it (an `UnknownKey` on the
/// dynamic path). Linear over the small registry.
pub fn lookup(field_id: u8) -> Option<FieldDef> {
    registry().into_iter().find(|d| d.field_id == field_id)
}

#[cfg(test)]
mod registry_tests {
    use super::*;

    #[test]
    fn registry_has_every_declared_field_with_its_handle_type_and_default() {
        let reg = registry();
        assert_eq!(reg.len(), REGISTRY_LEN);
        assert_eq!(reg.len(), FIELD_IDS.len()); // one entry per declared id
                                                // Spot-check the genuine tunables: id + kind + default come straight from the handle.
        let m = lookup(MOTOR_CURRENT_LIMIT.id()).unwrap();
        assert_eq!(m.kind, Type::U32);
        assert_eq!(m.default, Value::U32(10_000));
        let n = lookup(DEVICE_NAME.id()).unwrap();
        assert_eq!(n.kind, Type::Str);
        assert_eq!(n.default, Value::Str("hoverboard"));
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
        let reg = registry();
        for (i, a) in reg.iter().enumerate() {
            for b in &reg[i + 1..] {
                assert_ne!(a.field_id, b.field_id);
            }
        }
    }
}
