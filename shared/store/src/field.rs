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
/// its indices and is a plain [`Field`]. This exists for the families that cannot, such as the
/// gain maxima ([`CONTROL_GAIN_MAX`]), whose four indices default to four different bounds. The
/// alternative was a sentinel default plus a fallback table in the consumer, i.e. the field's real
/// default living somewhere other than the field.
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

    /// This field's [`REGISTRY`] row. `const`, so the table is a `static` in flash.
    ///
    /// `slot` is this field's entry in [`BYTES_DEFAULTS`], which is where a row's bytes live; a
    /// row carries the slot rather than the slice so every row is 8 B.
    pub const fn entry(self, slot: u32, effect: Effect) -> FieldEntry {
        FieldEntry::new(self.field_id, self.index, Type::Str, slot, effect)
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

    /// This field's [`REGISTRY`] row; `slot` is its entry in [`BYTES_DEFAULTS`], as
    /// [`StrField::entry`].
    pub const fn entry(self, slot: u32, effect: Effect) -> FieldEntry {
        FieldEntry::new(self.field_id, self.index, Type::Blob, slot, effect)
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
pub const MOTOR_CURRENT_LIMIT: Field<u32> = Field::new(0x4E, 10_000);
/// The commutation method (`specs/commutation.md`, "The mode model"): `0 = SixStep` (the default,
/// which needs no current sensing and is therefore the method any board can run), `1 = Sine`
/// (open-loop sinusoidal modulation off the interpolated hall angle), `2 = Foc` (hall-sensored
/// field-oriented control in torque mode, closed on the injected-ADC phase currents). The byte IS
/// `commutation::CommutationMethod`'s discriminant, and unknown values read as six-step. In the
/// arm-time value row (`specs/integration.md`, "When a stored value takes effect: the arm-time
/// re-read"), so a write while disarmed takes effect at the next ARM.
///
/// `Foc` is a CAPABILITY request, like Balance in [`CONTROL_MODE`]: it is honoured only on a board
/// that senses phase current (both phase pins set, [`MOTOR_PHASE_A`] and [`MOTOR_PHASE_B`]) and
/// whose boot offset calibration was accepted. Elsewhere it is demoted to six-step with the
/// method-demotion fault (`firmware::motor::running_method`), never run on no current.
///
/// **What `Foc` makes of the demand word** (`specs/foc-torque.md`): a q-axis CURRENT reference, not
/// a voltage. Two PIs regulate `iq` to `clamp(demand, +-28500) * full_scale / 28500` and `id` to
/// zero, where `full_scale` is [`MOTOR_CURRENT_LIMIT`] in counts less [`MOTOR_NOISE_FLOOR`]'s
/// (no field of its own: the limit is the one owner, and the floor keeps the reference under the
/// chop). Zero demand is zero current, driven, so the wheel turns freely. The balance law stops
/// normalizing its word by the battery on a board that runs it, so both boards of a machine stage
/// the same method.
///
/// **Selectable is not confirmed, and that lands on `Foc` alone.** The arm has not turned a wheel
/// on silicon: its frame, the sense polarity and the start gains are checked by that spec's bench
/// gates before anything balances on it. The loop is also scaled by [`MOTOR_CURRENT_CAL`], whose
/// 455 counts per amp is an upper bound off a window maximum over ADC noise rather than a fit
/// (`specs/motor-integration.md`, "The current-sense calibration": the two available two-point
/// slopes disagree, 81 and 306, and the sweep that would settle it has not run). An over-stated
/// scale under-states the measured current, so the loop's gain and the amps a demand word asks for
/// are both off by that factor. Six-step and sine are open-loop and do not care.
pub const MOTOR_METHOD: Field<u8> = Field::new(0x23, 0);
/// The runtime control mode (`specs/control.md` (b), the `MOTOR_METHOD` precedent): `0 =
/// Throttle` (default: works on every board, no IMU required; balancing is an opt-in), `1 =
/// Balance`. Consumed by the control crate's mode dispatch (its `ControlMode::from_u8` maps
/// unknown values to Throttle), which demotes a Balance request on a board with no IMU to Throttle
/// and raises the mode fault. In the arm-time value row (`specs/integration.md`, "When a stored
/// value takes effect: the arm-time re-read"), so a write while disarmed takes effect at the next
/// ARM, which replaces the outgoing controller's producer records with it.
pub const CONTROL_MODE: Field<u8> = Field::new(0x20, 0);
/// Whether balance mode requires a rider (`specs/control.md` (i)): `1 = required` (the default: an
/// unconfigured board engages only with a rider, exactly as before the field existed), `0 = not
/// required` (the rover: the engage conjunction runs without the rider term, the step-off wind-down
/// is held clear and the profile is A). Any nonzero value reads as required. A machine-type setting
/// beside [`CONTROL_MODE`] and in the same arm-time value row, read into the control dispatch at
/// boot and again at every arm; NOT on the tune lane.
pub const CONTROL_RIDER_REQUIRED: Field<u8> = Field::new(0x22, 1);
/// The low-battery floor (`specs/sensing-and-safety.md`, "The low-battery floor"), centivolts: a
/// balance ENGAGE is refused while the effective battery word is below it; never a disengage (on a
/// balancing rover a refused engage is safe and a disengage is a fall, so a run that started above
/// the floor is not ended by sag under it). **`<= 0` = no floor** (the UNKNOWN-word refusal still
/// applies). Default 2400 cV, which is 3.0 V per cell on an eight-cell pack. No clamp beyond the
/// type.
///
/// An ABSOLUTE threshold the operator sets, not a per-cell one the board works out: it carries a
/// disabled state no cell count can express, and a machine may want its engages refused well above
/// a cell's floor. What the pack IS lives in [`BOARD_BATTERY_CELLS`], which is what a client reads to
/// say how full it is; the two are separate facts and neither derives the other. A
/// machine-type setting beside [`CONTROL_RIDER_REQUIRED`] and in the same arm-time value row, read
/// into the control dispatch at boot and again at every arm; NOT on the tune lane.
pub const BOARD_BATTERY_FLOOR: Field<i16> = Field::new(0x26, 2400);
/// The machine's frame (`specs/control.md` (k), `specs/rover-one-imu.md`): `0 = pivoting`
/// platforms (the default, byte-identical to the image before the field: each board balances its
/// own platform on its own IMU), `1 = rigid` body (one balance loop for the machine; the board with
/// a configured IMU leads and puts its torque on the wire, the board without follows and applies
/// it). Consumed by the control crate's mode-selection seam beside [`CONTROL_MODE`] (its
/// `Frame::from_u8` maps unknown values to pivoting). Staged the SAME on both boards of a machine;
/// a pair that disagrees is caught on the wire by the leader and follower flags. A machine-type
/// setting in the arm-time value row (`specs/integration.md`), NOT on the tune lane.
pub const CONTROL_FRAME: Field<u8> = Field::new(0x21, 0);
/// Which wheel of a rigid machine this board drives (`specs/rover-one-imu.md` section 5): `0 =
/// left` (the default), `1 = right`; any other byte reads as left, the [`CONTROL_FRAME`] rule.
/// This is `BlockWords::role_right`'s owner at last: the block word has existed and been read in
/// two places since the throttle producer was built, and never written, and the steering split is
/// the first consumer that needs an answer.
///
/// It also sets the sign of the state-feedback law's position and speed terms on a balancing board
/// (`control::wheel_forward`): the leader reads its own wheel through it and the peer's through the
/// opposite side, so a right wheel left at the default is a sign error the disarmed gate 1 of
/// `specs/state-feedback.md` exists to catch.
///
/// **Per motor** (`Key.index` = the motor, `specs/machine-settings.md` section 3.3): a wheel field,
/// since a two-motor board drives two wheels with two sides. A board drives motor 0 in this
/// version, so the firmware reads index 0. The two boards of a machine hold DIFFERENT values, 1 on
/// the right wheel's motor. Only the LEADER's copy reaches the split, because the leader computes
/// both wheel demands and the follower applies what it is given; the follower's copy is written all
/// the same, because the side is that wheel's own fact and the leader is chosen by IMU presence, so
/// fitting an IMU to the other board swaps the roles and the split then reads the other copy. Read
/// at boot and at every arm (`firmware::arm::ARM_ROW`), NOT on the tune lane.
pub const CONTROL_SIDE: Field<u8> = Field::new(0x50, 0);
pub const DEVICE_NAME: StrField = StrField::new(0x02, "Hoverboard");
/// The Wi-Fi network a bridge joins (`specs/bridge-onboarding.md` section 2.1): 1 to 32 bytes, read
/// once at boot. Empty or unstored (the default), the bridge does not start its Wi-Fi station. A
/// bridge refuses a longer one (`net::walk::bridge_value_usable`); a board stores it like any key
/// and never reads it. Index 1 is reserved for a second network and nothing reads it.
pub const WIFI_SSID: StrField = StrField::new(0x03, "");
/// The password of [`WIFI_SSID`]'s network: empty for an open network, else 8 to 63 bytes (a
/// 64-digit hex PSK is not accepted). A [`SECRET`]: no node reads it back over a link.
pub const WIFI_PASS: StrField = StrField::new(0x04, "");
/// The bridge's Wi-Fi transmit power cap in quarter-dBm (`specs/bridge-onboarding.md` section
/// 2.4), read once at boot with [`WIFI_SSID`] and applied before every join. The default, 34 (8 dBm
/// on the C3), is the cap at which the SuperMinis from one batch join: at full power they stop
/// acknowledging the access point's authentication response. A bridge
/// refuses a write outside [`WIFI_TX_POWER_RANGE`] (`net::walk::bridge_value_usable`), and a stored
/// value outside it (written by an image without the check) is not applied: the bridge uses the
/// default. A board stores it like any key and never reads it.
pub const WIFI_TX_POWER: Field<u8> = Field::new(0x05, 34);
/// The values [`WIFI_TX_POWER`] takes: what `esp_wifi_set_max_tx_power` accepts. The radio applies
/// one of its own discrete levels from it: on the C3 the range spans 2 to 20 dBm, 34 applies 8 dBm,
/// and 80 to 84 all apply 20 dBm (the ESP-IDF header's mapping table, `esp-wifi-sys-esp32c3`
/// `include.rs`).
pub const WIFI_TX_POWER_RANGE: core::ops::RangeInclusive<u8> = 8..=84;

/// The board's persistent L3 node address (`specs/l3.md`, "Addressing"): assigned once by the walk's
/// `ASSIGN` and persisted to flash, reported on every boot, survives reboot. `0x00` = no address yet.
/// The same field a `CONFIG_WRITE` of this key would touch; `ASSIGN` is the bootstrap path that reaches
/// it by relay before the board has an address.
pub const NODE_ADDRESS: Field<u8> = Field::new(0x01, 0x00);

/// The number of inter-board links a board can have (`specs/configured-links.md` section 2): the
/// index of [`LINK_TX`] / [`LINK_RX`]. Two is the most a chain needs (end boards use one link,
/// middle boards two) and the most an F1x0 part has (USART0 and USART1). Every per-link array is
/// sized by it, so the count has one owner.
pub const LINKS: u8 = 2;

/// Each inter-board link's transmit pin, index = the link (`specs/configured-links.md` section 2).
/// A link is configured when both its pins are set; unset (the default), the link does not exist
/// and its pins are not touched, so a board with nothing stored brings up its SWD mailbox and no
/// UART. Which USART a pair drives is the HAL pin model's answer (`pincap::usart_pins`).
pub const LINK_TX: IndexedField<u8, 2> = IndexedField::new(0x0F, [PIN_ABSENT, PIN_ABSENT]);
/// Each inter-board link's receive pin, index = the link (see [`LINK_TX`]).
pub const LINK_RX: IndexedField<u8, 2> = IndexedField::new(0x10, [PIN_ABSENT, PIN_ABSENT]);
// The two handles are sized by [`LINKS`]: written as a literal so the Kotlin mirror's drift gate
// reads them, and held to the count here.
const _: () = assert!(LINK_TX.len() == LINKS as usize && LINK_RX.len() == LINKS as usize);

// ---------------------------------------------------------------------------
// The board-layout fields (`specs/board-model.md`): the per-pin store fields (packed
// `(port << 4) | pin` bytes, `0xFF` = unset = function absent) plus the non-pin board facts the
// boot validator consumes. Registered here per the spec's registered-at-landing decision: the
// validator (`motorcontroller/board`) is their first consumer; motor.current_sense/direction/align_offset
// are now registered too (the `board::MotorPlan` fold-back of `specs/motor-integration.md` carries
// them at boot for the motor bring-up). motor.pole_pairs and wheel.diameter_mm are registered too,
// both defaulting to UNSET: they are the two facts a controller needs to turn counted hall edges
// into a distance, and neither has a figure measured on a board of this fleet. Neither has a
// firmware consumer, so neither is read at boot. Everything else here is read at boot only, through
// the validator; a config
// write never re-pins before reboot. The BENIGN functions carry the fleet-uniform defaults; the
// motor groups and dead-time default to ABSENT (drive is an explicit configuration act), and so do
// the three fields whose PIN is fleet-uniform but whose FUNCTION is not fitted on every board: the
// battery sense and the two foot pads, each of which would otherwise read a fiction off a line
// nothing drives.
// ---------------------------------------------------------------------------

/// `0xFF` = unset = the function is absent (`specs/board-model.md`, "The field vocabulary").
pub const PIN_ABSENT: u8 = 0xFF;

/// The power-latch pin (fleet default PB12; also asserted pre-mount as the compiled early-boot
/// value of this same default).
pub const BOARD_SELF_HOLD: Field<u8> = Field::new(0x06, 0x1C);
/// Battery-sense pin. Default ABSENT (`specs/sensing-and-safety.md`, "The battery word"): the pin
/// is fleet-uniform (PA4) but the SENSE is master-only, and a slave's PA4 reads stuck near 2.0 V,
/// which through the divider is a fictitious 62 V. The firmware has no role fact to tell the two
/// apart, so a defaulted pin would make every slave sense garbage; the master is CONFIGURED to
/// sense (stage `0x04`), a non-sensing board relays its peer's word.
pub const BOARD_BATTERY_PIN: Field<u8> = Field::new(0x07, PIN_ABSENT);
/// Buzzer pin (fleet default PB9).
pub const BOARD_BUZZER: Field<u8> = Field::new(0x08, 0x19);
/// Indicator LEDs (fleet defaults PB3 / PA15 / PB4).
pub const LED_GREEN: Field<u8> = Field::new(0x09, 0x13);
pub const LED_ORANGE: Field<u8> = Field::new(0x0A, 0x0F);
pub const LED_RED: Field<u8> = Field::new(0x0B, 0x14);
/// Foot-pad rider-detection inputs. Default ABSENT, for [`BOARD_BATTERY_PIN`]'s reason one step further
/// on: the pins are fleet-uniform (PA11 / PC15) but the PADS are not, and no board in this fleet
/// has pad hardware fitted. A defaulted pin leaves the line a floating input (the `InputGroup`
/// reader configures no pull and the reset state of both families is a floating input), and the
/// pad debouncer is active-high and engages on ONE high sample while releasing only on two, so a
/// floating line that picks up a single high sample asserts a foot. Two of those assert a rider,
/// which selects the RUN gain set and holds an engagement the rider is supposed to be able to end
/// by stepping off. So a board with pads is CONFIGURED to have them (stage `0x0B` / `0x2F`), and a
/// board without them reads no rider, which is the safe direction and the true one.
pub const PAD_A: Field<u8> = Field::new(0x0C, PIN_ABSENT);
pub const PAD_B: Field<u8> = Field::new(0x0D, PIN_ABSENT);
/// IMU bus pins (VARIANT function: no safe fleet default; absent until configured).
pub const IMU_SCL_PIN: Field<u8> = Field::new(0x11, PIN_ABSENT);
pub const IMU_SDA_PIN: Field<u8> = Field::new(0x12, PIN_ABSENT);
/// Hall inputs, per-motor via `Key.index` (motor groups are CONFIGURED, never defaulted).
pub const MOTOR_HALL_A: Field<u8> = Field::new(0x40, PIN_ABSENT);
pub const MOTOR_HALL_B: Field<u8> = Field::new(0x41, PIN_ABSENT);
pub const MOTOR_HALL_C: Field<u8> = Field::new(0x42, PIN_ABSENT);
/// The advanced-timer gate set, per-motor via `Key.index` (configured, never defaulted).
pub const MOTOR_GATE_HI_A: Field<u8> = Field::new(0x43, PIN_ABSENT);
pub const MOTOR_GATE_HI_B: Field<u8> = Field::new(0x44, PIN_ABSENT);
pub const MOTOR_GATE_HI_C: Field<u8> = Field::new(0x45, PIN_ABSENT);
pub const MOTOR_GATE_LO_A: Field<u8> = Field::new(0x46, PIN_ABSENT);
pub const MOTOR_GATE_LO_B: Field<u8> = Field::new(0x47, PIN_ABSENT);
pub const MOTOR_GATE_LO_C: Field<u8> = Field::new(0x48, PIN_ABSENT);
/// The two phase-current sense pins, per-motor via `Key.index` (configured, never defaulted).
///
/// `specs/motor-integration.md` bring-up step 5: the injected ADC group is programmed with these
/// two pins' ADC channels as its two ranks, so the CHANNELS are per-board data and cannot be a
/// compiled constant (the bench pair proves it: the F103 master senses on PB0/PA0 and the F130
/// slave on PB0/PB1). The boot validator derives each pin's ADC channel through the same
/// `Capabilities::adc_channel` query `battery_sense.pin` uses. Ordered A then B, matching the injected
/// rank order (rank 0 = phase A, rank 1 = phase B) and the gate channel order (CH0 = phase A).
///
/// Group rule: the pair is all-or-none, and phase sense is fitted exactly when both pins are set
/// (`specs/machine-settings.md` section 3.3); one of the two is recorded as incomplete.
pub const MOTOR_PHASE_A: Field<u8> = Field::new(0x49, PIN_ABSENT);
pub const MOTOR_PHASE_B: Field<u8> = Field::new(0x4A, PIN_ABSENT);
/// The power-button sense pin (`specs/board-model.md` `power.button`; the `power_request`
/// producer, `specs/integration.md`'s input task). No fleet default is pinned yet, so unset
/// until configured.
pub const BOARD_BUTTON: Field<u8> = Field::new(0x0E, PIN_ABSENT);
/// The IMU model index (`specs/imu.md`: 0 = no IMU fitted; the imu crate owns the numbering).
pub const IMU_MODEL: Field<u8> = Field::new(0x13, 0);
/// Per-axis zero-rate gyro bias, raw counts, indexed 0/1/2 = x/y/z (`specs/imu.md`, "Board-config
/// fields": the bench-captured calibration staged into `imu::Config.gyro_bias` at bring-up;
/// default 0 = uncalibrated). i32: the imu crate's bias word (counts fit i16, the type matches
/// the consumer).
pub const IMU_GYRO_BIAS: IndexedField<i32, 3> = IndexedField::new(0x14, [0, 0, 0]);
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
pub const IMU_AXIS_SIGN: IndexedField<i32, 3> = IndexedField::new(0x15, [0, 0, 0]);
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
pub const IMU_AXIS_ROLE: IndexedField<u8, 2> = IndexedField::new(0x16, [0, 0]);
/// Per-motor dead-time (raw DTG; 0 = unset; a configured gate group requires it nonzero).
pub const MOTOR_DEAD_TIME: Field<u8> = Field::new(0x4B, 0);
/// Per-motor drive direction (`commutation::Direction`, consumed by every arm: six-step as the
/// source/sink swap, sine and FOC as a half-turn of the angle; a board-mounting fact, staged with
/// [`MOTOR_ALIGN_OFFSET`] as the motor's alignment).
/// `0 = Forward`, nonzero = Reverse, where Forward names the un-mirrored electrical sequence and
/// not a body direction: the demand word's forward is NEGATIVE (`specs/control.md` (a), "Which
/// sign is forward"), so on a balancing board this byte is determined by the wheel's wiring and
/// mounting rather than chosen, and the two boards of one machine need not hold the same value.
///
/// A VALUE-ROW field (`specs/integration.md`, "When a stored
/// value takes effect: the arm-time re-read"): the boot plan carries it as the first value the
/// bring-up runs on (`board::MotorPlan`, the fold-back of `specs/motor-integration.md`), and every
/// ARM re-reads it and installs it again, so a disarmed write takes effect at the next arm with no
/// reboot (the bench measured the boot-read it replaced, 2026-10-20). Per-motor via `Key.index`.
pub const MOTOR_DIRECTION: Field<u8> = Field::new(0x4F, 0);
/// Per-motor align offset: where this motor's hall sensors sit relative to its phase windings, as
/// a 0..5 sector rotation (60 electrical degrees a unit; `specs/commutation.md`, "The alignment
/// every method consumes"; bench-swept and baked per the tuning-into-code rule once found).
///
/// **Consumed by every method**, with [`MOTOR_DIRECTION`], out of the one decode an arm builds
/// from the two bytes: six-step rotates its state assignment by it and the angle path adds the
/// same rotation to the angle sine and FOC index (`commutation::angle_path_offset`). It reached
/// six-step alone until 2026-10-23, which left a staged board driving a different vector under
/// sine and FOC than under six-step.
///
/// A VALUE-ROW field beside [`MOTOR_DIRECTION`] (`specs/integration.md`, "When a stored value
/// takes effect: the arm-time re-read"): the boot plan carries it as the first value the bring-up
/// runs on (`board::MotorPlan`), and every ARM re-reads it and installs it again, so the sweep
/// itself runs without a reboot per value. Per-motor via `Key.index`.
pub const MOTOR_ALIGN_OFFSET: Field<u8> = Field::new(0x51, 0);

/// Per-motor current-sense CALIBRATION, stock current counts per amp of phase current
/// (`specs/motor-integration.md`, "The current-sense calibration"): what a `motor_current` count
/// MEANS, beside the phase pins ([`MOTOR_PHASE_A`], [`MOTOR_PHASE_B`]), which say where that current
/// is sensed.
///
/// **Why a field.** It is a property of the shunt and amplifier chain fitted to a particular board,
/// the [`BOARD_BATTERY_CAL`] class, and it was the only analog scale left in a `const`:
/// `COUNTS_PER_AMP = 800` in `motorcontroller/firmware/src/motor.rs`, EFeru's `A2BIT_CONV = 50` ADC LSB per
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
/// 100..=819, a plausibility band that bounds how far a typoed scale can move the ENFORCED limit
/// away from the labelled one (the converted COUNT is bounded separately, below by this board's
/// [`MOTOR_NOISE_FLOOR`] and above by `firmware::motor::MAX_LIMIT_COUNTS`). Per-motor via
/// `Key.index`.
pub const MOTOR_CURRENT_CAL: Field<u16> = Field::new(0x4C, 455);

/// Per-motor phase-current NOISE FLOOR, stock current counts: the MINIMUM soft limit this board's
/// sense chain can be held to (`specs/store-field-audit.md`, the 2026-10-09 board-specifics sweep,
/// Tier 1 item 1). A milliamp limit that converts below it is clamped UP to it
/// (`firmware::motor::limit_counts`), so it is the smallest current this board enforces.
///
/// **Why a field.** It is the rest noise of the shunt and amplifier chain fitted, the
/// [`MOTOR_CURRENT_CAL`] class and the same units, and it was a compiled
/// `MIN_LIMIT_COUNTS = 2_100` measured on ONE board in ONE session (the high-water of five
/// rest-floor reads on 2026-10-09: 1,444 / 1,863 / 1,910 / 1,941 / 2,097). One number for every
/// chain is wrong in both directions and invisible in both: too low and the limiter chops on the
/// chain's own noise, which is a board that trips at rest; too high and the minimum enforceable
/// current rises silently (2,100 counts is 4.6 A at the default scale, so a staged 2 A becomes
/// 4.6 A with nothing said).
///
/// **The default is 2,100**, the constant it replaces, so a board with no record enforces exactly
/// what the image enforced before the field existed. A board's own floor is a disarmed bench read:
/// gate 1 of `tools/climit-session.py` reports the rest-noise maximum, and that reading is what
/// belongs here rather than the fleet default.
///
/// Range enforcement is NOT here (the store validates type only): the boot seam
/// (`firmware::motor::noise_floor_counts`) clamps it to `1_000..=16_383`, a plausibility band the
/// [`MOTOR_CURRENT_CAL`] class uses too. Below 1,000 a chain would claim a floor under the quietest
/// rest reading any bench board has produced (1,054 median, 1,444 worst of five); at the top of the
/// band the limit has no range left, because that is `firmware::motor::MAX_LIMIT_COUNTS`.
///
/// Boot-read, into the bring-up's `BootFixed` and carried from there into every arm-time
/// re-derivation, so one floor serves the whole boot (`specs/integration.md`, "When a stored value
/// takes effect": the third group, values the loop consumes that stay boot-read). Per-motor via
/// `Key.index`, because the chain is per motor; it cannot share [`MOTOR_CURRENT_CAL`]'s key, whose
/// index already carries the motor.
pub const MOTOR_NOISE_FLOOR: Field<u16> = Field::new(0x4D, 2_100);

/// This wheel's lifetime travel, in ABSOLUTE hall edges (`specs/odometer.md`): every counted
/// commutation edge adds 1, whichever way the wheel turned, so the total never winds back.
///
/// **Counts, not a distance, deliberately.** Converting needs a pole-pair count and a wheel
/// diameter ([`MOTOR_POLE_PAIRS`] and [`WHEEL_DIAMETER_MM`]), both registered, both defaulting to
/// UNSET, and both per-machine facts until somebody measures them on the board in question.
/// Storing the count means a later correction to either changes what a controller prints and
/// leaves every recorded total valid.
///
/// **The only field here the FIRMWARE writes to on its own.** Every other record is a config
/// change somebody typed; this one is appended once per ride, on a service-loop pass that finds
/// the board disarmed and the wheel stopped (`motorcontroller/firmware/src/odometer.rs`). The store carries
/// no arm state and never will (`specs/storage-layer.md`, "Who enforces writes only while
/// disarmed"), so that gate is the caller's, exactly as it is for the L3 `CONFIG_WRITE` path.
///
/// Per motor via `Key.index`, because one wheel's travel is one wheel's. Only index 0 has a
/// writer, because only motor 0 is brought up.
pub const MOTOR_ODOMETER: Field<u32> = Field::new(0x54, 0);

/// This motor's POLE-PAIR count: how many ELECTRICAL revolutions the rotor turns per mechanical
/// one (`specs/board-model.md`, the `motor.pole_pairs` row). With the six hall edges per electrical
/// revolution, it is what turns a counted edge into a fraction of a wheel turn: `6 * pole_pairs`
/// edges make one revolution of the wheel.
///
/// **The default is 0, meaning UNSET, and that is the decision here.** 15 is the count the fleet's
/// 6.5 inch hubs are believed to have and the number every existing derivation in this tree
/// assumes, but it has never been counted on a board of this fleet. A default of 15 makes every
/// board print a distance at a scale nobody checked, and the pole-pair count is the one factor a
/// reader cannot sanity-check from the result: a 20-pole-pair hub reads 25 percent long and still
/// looks plausible. A wrong distance stated confidently is worse than no distance. 0 is not a
/// motor, so it is an unambiguous sentinel, and it is the convention the field set already uses for
/// a fact with no safe fleet value ([`IMU_MODEL`], [`IMU_AXIS_ROLE`]).
///
/// **The value is a measurement.** Turn the wheel one full revolution, read the delta on this
/// motor's [`MOTOR_ODOMETER`], and divide by six.
///
/// **No firmware consumer, and there is no seam to go looking for.** Nothing in the image converts
/// anything to metres, so there is no boot read, no arm-time re-read (it is not in
/// `firmware::arm::ARM_ROW`) and no clamp. The store is this fact's canonical home and a controller
/// reads it over `CONFIG_READ`. Per motor via `Key.index`, because two wheels may carry different
/// hubs; only index 0 has a consumer, because only motor 0 is brought up.
pub const MOTOR_POLE_PAIRS: Field<u8> = Field::new(0x52, 0);

/// This wheel's ROLLING diameter, millimetres (`specs/board-model.md`): the other half of the
/// edges-to-distance conversion, beside [`MOTOR_POLE_PAIRS`]. One wheel revolution covers
/// `PI * diameter_mm`.
///
/// **The default is 0, meaning UNSET.** The rover, the offroad chassis and a stock hoverboard do
/// not share a wheel size, so there is no fleet default to carry here, and 0 mm is not a wheel. The
/// figure wanted is the ROLLING diameter under load, not the tyre's marked size, which differs from
/// it by a few percent.
///
/// **No firmware consumer**, exactly as [`MOTOR_POLE_PAIRS`] has none: no boot read, no `ARM_ROW`
/// entry, no clamp seam, because nothing in the image converts anything to metres. The store is the
/// canonical home and a controller reads it over `CONFIG_READ`. Per motor via `Key.index`, because
/// a machine can carry two different wheels; only index 0 has a consumer.
pub const WHEEL_DIAMETER_MM: Field<u16> = Field::new(0x53, 0);

/// The battery-sense calibration, counts to centivolts (`specs/sensing-and-safety.md`, "The field:
/// `battery_sense.cal`"), indexed `0 = slope` (microvolts of rail per 12-bit count) and `1 = offset`
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
/// Consumer: the battery word's local sense, on boards whose plan carries `battery_sense.pin`.
pub const BOARD_BATTERY_CAL: IndexedField<i16, 2> = IndexedField::new(0x18, [25200, -5]);

/// What the pack IS: the number of cells in series (`specs/sensing-and-safety.md`, "The pack").
/// **0 = not stated**, which is the default and is not a guess at eight.
///
/// The machine's battery is one object, and before this field it was described twice and named
/// nowhere: [`BOARD_BATTERY_FLOOR`]'s 2400 cV default is 3.0 V across eight cells, and the rider
/// app's charge curve carried two more voltages of its own. Neither states the pack, and neither
/// can be derived from the other, so a client that wants to say how full a pack is had to assume
/// one. This is where the pack is stated, and a consumer that needs a per-cell voltage multiplies
/// it rather than carrying a pack-shaped constant.
///
/// **Written the same to BOTH boards of a machine**, the [`CONTROL_FRAME`] rule: one pack feeds
/// both, and each board answers for itself over the link.
///
/// The firmware does not read it. The low-battery floor stays an absolute threshold the operator
/// sets ([`BOARD_BATTERY_FLOOR`], which also has a disabled state this cannot express), so the
/// one consumer is a client turning a pack voltage into a state of charge: 0 means it has nothing
/// to turn it into and says so instead of drawing a gauge at an assumed pack.
pub const BOARD_BATTERY_CELLS: Field<u8> = Field::new(0x24, 0);

/// What KIND of cell the pack is (`specs/remote-ui.md` 3.6d): `0` = li_ion (Li-ion NMC, the
/// hoverboard 18650; the default), `1` = lev50 (Lithium Energy Japan LEV50), `2` = lto (lithium
/// titanate). It picks the resting-voltage table a client reads a state of charge from, per cell,
/// and [`BOARD_BATTERY_CELLS`] multiplies it.
///
/// The default is a chemistry and not "not stated": the cell count already carries the not-stated
/// case, and a stated count with the default draws the gauge a hoverboard pack expects. Written the
/// same to both boards, the cell count's rule. The firmware does not read it; an unknown byte is
/// the client's to refuse.
pub const BOARD_BATTERY_CHEMISTRY: Field<u8> = Field::new(0x25, 0);

/// Which electrical level means ASSERTED on each of the board's three discrete input lines, one
/// bit per line: `b0` button, `b1` pad A, `b2` pad B, a SET bit meaning active high
/// (`specs/board-model.md`, the field table). Bits 3..7 are reserved and inert: `inputs::asserted`
/// masks the word to the three lines, so a stray bit cannot quietly acquire a meaning later.
///
/// **A property of the input wiring fitted, not of the firmware**, and a compiled literal until
/// now: `motorcontroller/inputs/src/debounce.rs` has always said that "which pin / polarity each line maps
/// to is board data resolved by the caller", and the caller resolved the PIN from a field
/// ([`BOARD_BUTTON`], [`PAD_A`], [`PAD_B`]) and the polarity from a literal. ONE packed field
/// rather than three, by the owner's choice: one id, one seam, one editor row, and the three are
/// the same kind of fact about the same connector.
///
/// **The default `0b110` is today's compiled behaviour exactly**, so no board changes: the button
/// idles high through a pull-up and a press pulls it down (active low, bit clear), and the foot
/// pads read high with a foot on (active high, bits set). It is not a measurement of one board,
/// which is why it is a default and not an unset sentinel: there is no sentinel available in a
/// bitmask whose every value is a legal wiring, and a board with no record must still read its
/// inputs. No fleet board is known to differ.
///
/// Getting it wrong is visible rather than silent on the button, and NOT silent-safe on the pads:
/// an inverted pad reads "rider on both pads" on an empty machine, which steps the balance loop
/// from the standby gains to the RUN set, about 120x the proportional authority
/// (`motorcontroller/inputs/src/pad.rs`).
pub const BOARD_INPUT_POLARITY: Field<u8> = Field::new(0x19, 0b110);

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
pub const ATTITUDE_LEVEL_TRIM: IndexedField<i16, 2> = IndexedField::new(0x17, [0, 0]);

/// The rigid frame's turn input bound and rate (`specs/rover-one-imu.md` section 5), indexed
/// `0 = turn_max` (the differential torque in stock counts at full stick) and `1 = turn_slew`
/// (counts per control tick that the turn may move).
///
/// **Default `turn_max` 0 = disabled**: the machine drives straight and a board is byte-identical
/// in behaviour to one before the field existed; the rover stages 1000 to 3000 for first attempts.
/// `turn_slew`'s 24 traverses a staged 3000 in 125 ticks (0.5 s), matching the lean's own traverse,
/// because a step in differential torque is a yaw impulse and on a rigid frame a yaw impulse
/// reaches the pitch loop as a disturbance through the steel.
///
/// Read at boot and again at every arm (`motorcontroller/firmware/src/arm.rs`, `ARM_ROW`), so a write
/// applies from the next arm and costs no power-cycle. NOT on the live tune lane: a bound on how
/// much authority the stick has is not changed under a moving machine, and a config write is
/// refused while armed. The remote app offers it on the Tune screen all the same
/// (`specs/remote-ui.md` 3.3b).
/// Range enforcement is NOT here (the store validates type only): the seam
/// (`control::DriveTurn::new`) clamps `turn_max` to 0..28500 and `turn_slew` to 1..1000. An
/// [`IndexedField`] because its two indices default differently.
pub const CONTROL_DRIVE_TURN: IndexedField<i16, 2> = IndexedField::new(0x2B, [0, 24]);

/// The state-feedback balance law's four gains (`specs/state-feedback.md` sections 2 and 6),
/// indexed `0 = gp` (numerator counts per degree of pitch), `1 = gr` (per 0.1 rad/s of pitch
/// rate), `2 = gx` (per hall edge of position error) and `3 = gv` (per sixteenth of a count of
/// speed error). Each gain multiplies its own measured term and nothing else.
///
/// **Default `[0, 0, 0, 0]` = inert**: an unstaged board in balance mode produces a zero word, so
/// armed it applies no torque and the tilt cutout trips as it falls. The rover is staged from the
/// app with the section-7 start set `[3491, 4002, 197, 926]`. The defaults are `control`'s
/// unstaged `GainShadow` (pinned by `store`'s tests).
///
/// A new id rather than a reuse of the retired PID profile ids (0x71/0x72): the rover's store
/// holds records under those at their old meanings, and a reused id would read them as these
/// gains. Records under retired ids are ignored, because nothing registered names them.
///
/// **ON the live tune lane** (`TUNE_WRITE`/`TUNE_READ`), whose allowlist is exactly this field's
/// four indices and [`CONTROL_REFERENCE`]'s two. Range enforcement is NOT here (the store
/// validates type only): it lives at the tune seam, `control::GainShadow`, which every writer goes
/// through, against [`CONTROL_GAIN_MAX`]. In RUN the law ramps its live copy toward these under
/// fixed per-pass caps (`control::config::sf::RAMP_CAPS`).
pub const CONTROL_SF_GAINS: IndexedField<i16, 4> = IndexedField::new(0x27, [0, 0, 0, 0]);

/// The inclusive upper bound of each [`CONTROL_SF_GAINS`] index (`specs/state-feedback.md` section
/// 6), indexed like the gains; the lower bound is the constant `control::GAIN_MIN` (0).
///
/// A field rather than a constant because the LQR's torque scale (assumption A1) is uncertain by a
/// factor near one, so the owner may need to move the maxima. The defaults ARE
/// `control::DEFAULT_GAIN_MAX` (pinned by `store`'s tests), four to ten times above the start
/// gains. Read at boot and again at every arm (`motorcontroller/firmware/src/arm.rs`, `ARM_ROW`) into
/// `control::GainShadow` (each index taken as `max(0, value)`; a live gain above a shrunk maximum
/// clamps). NOT on the live tune lane.
///
/// A new id rather than the retired 0x74, the three-index PID maxima, for [`CONTROL_SF_GAINS`]'s
/// reason.
pub const CONTROL_GAIN_MAX: IndexedField<i16, 4> =
    IndexedField::new(0x28, [20000, 20000, 2000, 10000]);

/// The state-feedback law's stick reference (`specs/state-feedback.md` section 4), indexed
/// `0 = speed_max` (sixteenths of a wheel-speed count at full stick) and `1 = speed_slew` (1/4096
/// count per control tick, the stick's acceleration and the stop ramp of a stale link).
///
/// **Default `[0, 41]`**: `speed_max` 0 keeps the stick dead (the machine holds position) until it
/// is staged; `speed_slew` 41 is 0.72 m/s^2. The defaults are `control::config::sf`'s
/// `REFERENCE_DEFAULT` (pinned by `store`'s tests).
///
/// **ON the live tune lane** beside [`CONTROL_SF_GAINS`]. Range enforcement is the lane's seam
/// (`control::GainShadow`) against COMPILED bounds (`control::config::sf::REFERENCE_MIN` /
/// `REFERENCE_MAX`, `0..=512` and `1..=4096`), which bound the reference's arithmetic.
pub const CONTROL_REFERENCE: IndexedField<i16, 2> = IndexedField::new(0x29, [0, 41]);

/// The state-feedback law's position clamp, hall edges (`specs/state-feedback.md` section 4): the
/// bound on the position error, so a machine pushed further than this comes back exactly this far
/// and no further. **Default 45** (0.26 m on the rover's wheel); 0 disables the position term.
///
/// Read at boot and again at every arm (`motorcontroller/firmware/src/arm.rs`, `ARM_ROW`), so a write
/// applies from the next arm. NOT on the live tune lane: a bound on how much correction a held
/// push can wind is not changed under a moving machine. Range enforcement is NOT here: the seam
/// (`control::sf::pos_clamp`) clamps it to `0..=450` (`control::sf::POS_CLAMP_MAX`).
pub const CONTROL_POS_CLAMP: Field<i16> = Field::new(0x2A, 45);

/// The number of motor indices (`specs/cli-config.md` section 3, "Motor count"): a per-motor field's
/// `Key.index` is the motor, `0..MOTORS`. The board crate's per-motor arrays are sized by it, so the
/// count has one owner. Which fields are per motor is the `NAMES` table's index kind; how many
/// motors a board has FITTED is its own layout.
pub const MOTORS: u8 = 2;

/// The ids the board or the walk writes, not the operator (`specs/cli-config.md` section 3,
/// "Board-owned fields"): [`NODE_ADDRESS`] (assigned by the walk) and [`MOTOR_ODOMETER`] (written
/// after each ride). A configuration tool leaves them out of a dump and never un-stores them.
pub const BOARD_OWNED: &[u8] = &[NODE_ADDRESS.id(), MOTOR_ODOMETER.id()];

/// The ids whose default a node may answer itself instead of the registry's (`specs/cli-config.md`
/// sections 4.3 and 6.2): [`DEVICE_NAME`], which a bridge answers unstored with its MAC-based name
/// (`net`'s `Responder::set_default_name`). A configuration tool cannot know that value before the
/// key is un-stored, so it does not predict one.
pub const NODE_DEFAULTS: &[u8] = &[DEVICE_NAME.id()];

/// The ids no node reads back over a link (`specs/bridge-onboarding.md` section 2.2): [`WIFI_PASS`].
/// A `CONFIG_READ` of a stored one answers `CFG_SECRET` with no value (`net`'s `read_value`); a
/// configuration tool never prints one, leaves it out of a dump, counts it as changed whenever a
/// file gives it, and never un-stores it unless a file says `null`.
pub const SECRET: &[u8] = &[WIFI_PASS.id()];

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
/// Kept dedicated because no genuine tunable is a blob.
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
    0x02, // DEVICE_NAME
    0x03, // WIFI_SSID
    0x04, // WIFI_PASS
    0x05, // WIFI_TX_POWER
    0x06, // BOARD_SELF_HOLD
    0x07, // BOARD_BATTERY_PIN
    0x08, // BOARD_BUZZER
    0x09, // LED_GREEN
    0x0A, // LED_ORANGE
    0x0B, // LED_RED
    0x0C, // PAD_A
    0x0D, // PAD_B
    0x0E, // BOARD_BUTTON
    0x0F, // LINK_TX
    0x10, // LINK_RX
    0x11, // IMU_SCL_PIN
    0x12, // IMU_SDA_PIN
    0x13, // IMU_MODEL
    0x14, // IMU_GYRO_BIAS
    0x15, // IMU_AXIS_SIGN
    0x16, // IMU_AXIS_ROLE
    0x17, // ATTITUDE_LEVEL_TRIM
    0x18, // BOARD_BATTERY_CAL
    0x19, // BOARD_INPUT_POLARITY
    0x20, // CONTROL_MODE
    0x21, // CONTROL_FRAME
    0x22, // CONTROL_RIDER_REQUIRED
    0x23, // MOTOR_METHOD
    0x24, // BOARD_BATTERY_CELLS
    0x25, // BOARD_BATTERY_CHEMISTRY
    0x26, // BOARD_BATTERY_FLOOR
    0x27, // CONTROL_SF_GAINS
    0x28, // CONTROL_GAIN_MAX
    0x29, // CONTROL_REFERENCE
    0x2A, // CONTROL_POS_CLAMP
    0x2B, // CONTROL_DRIVE_TURN
    0x40, // MOTOR_HALL_A
    0x41, // MOTOR_HALL_B
    0x42, // MOTOR_HALL_C
    0x43, // MOTOR_GATE_HI_A
    0x44, // MOTOR_GATE_HI_B
    0x45, // MOTOR_GATE_HI_C
    0x46, // MOTOR_GATE_LO_A
    0x47, // MOTOR_GATE_LO_B
    0x48, // MOTOR_GATE_LO_C
    0x49, // MOTOR_PHASE_A
    0x4A, // MOTOR_PHASE_B
    0x4B, // MOTOR_DEAD_TIME
    0x4C, // MOTOR_CURRENT_CAL
    0x4D, // MOTOR_NOISE_FLOOR
    0x4E, // MOTOR_CURRENT_LIMIT
    0x4F, // MOTOR_DIRECTION
    0x50, // CONTROL_SIDE
    0x51, // MOTOR_ALIGN_OFFSET
    0x52, // MOTOR_POLE_PAIRS
    0x53, // WHEEL_DIAMETER_MM
    0x54, // MOTOR_ODOMETER
}

#[cfg(feature = "test-fields")]
field_ids! {
    0x01, // NODE_ADDRESS
    0x02, // DEVICE_NAME
    0x03, // WIFI_SSID
    0x04, // WIFI_PASS
    0x05, // WIFI_TX_POWER
    0x06, // BOARD_SELF_HOLD
    0x07, // BOARD_BATTERY_PIN
    0x08, // BOARD_BUZZER
    0x09, // LED_GREEN
    0x0A, // LED_ORANGE
    0x0B, // LED_RED
    0x0C, // PAD_A
    0x0D, // PAD_B
    0x0E, // BOARD_BUTTON
    0x0F, // LINK_TX
    0x10, // LINK_RX
    0x11, // IMU_SCL_PIN
    0x12, // IMU_SDA_PIN
    0x13, // IMU_MODEL
    0x14, // IMU_GYRO_BIAS
    0x15, // IMU_AXIS_SIGN
    0x16, // IMU_AXIS_ROLE
    0x17, // ATTITUDE_LEVEL_TRIM
    0x18, // BOARD_BATTERY_CAL
    0x19, // BOARD_INPUT_POLARITY
    0x20, // CONTROL_MODE
    0x21, // CONTROL_FRAME
    0x22, // CONTROL_RIDER_REQUIRED
    0x23, // MOTOR_METHOD
    0x24, // BOARD_BATTERY_CELLS
    0x25, // BOARD_BATTERY_CHEMISTRY
    0x26, // BOARD_BATTERY_FLOOR
    0x27, // CONTROL_SF_GAINS
    0x28, // CONTROL_GAIN_MAX
    0x29, // CONTROL_REFERENCE
    0x2A, // CONTROL_POS_CLAMP
    0x2B, // CONTROL_DRIVE_TURN
    0x40, // MOTOR_HALL_A
    0x41, // MOTOR_HALL_B
    0x42, // MOTOR_HALL_C
    0x43, // MOTOR_GATE_HI_A
    0x44, // MOTOR_GATE_HI_B
    0x45, // MOTOR_GATE_HI_C
    0x46, // MOTOR_GATE_LO_A
    0x47, // MOTOR_GATE_LO_B
    0x48, // MOTOR_GATE_LO_C
    0x49, // MOTOR_PHASE_A
    0x4A, // MOTOR_PHASE_B
    0x4B, // MOTOR_DEAD_TIME
    0x4C, // MOTOR_CURRENT_CAL
    0x4D, // MOTOR_NOISE_FLOOR
    0x4E, // MOTOR_CURRENT_LIMIT
    0x4F, // MOTOR_DIRECTION
    0x50, // CONTROL_SIDE
    0x51, // MOTOR_ALIGN_OFFSET
    0x52, // MOTOR_POLE_PAIRS
    0x53, // WHEEL_DIAMETER_MM
    0x54, // MOTOR_ODOMETER
    0xFD, // T_BLOB (store-test reserved)
    0xFE, // T_KEY  (store-test reserved)
}

// ---------------------------------------------------------------------------
// What a field describes and when a written value takes effect (`specs/machine-settings.md`
// sections 2 and 5).
// ---------------------------------------------------------------------------

/// What a field describes (`specs/machine-settings.md` section 2): its scope is the block its id
/// sits in, so the id map is the one owner and nothing restates it per field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    /// This circuit board or bridge and what is wired to it (`0x01..=0x1F`): identity, pins,
    /// calibration, links. Never compared across boards.
    Node,
    /// The vehicle as a whole (`0x20..=0x3F`): held as a copy on every motor controller, and the
    /// copies must match (the machine digest, section 4).
    Machine,
    /// One motor and its wheel (`0x40..=0x7F`), `Key.index` = the motor.
    Wheel,
}

impl Scope {
    /// The scope of `field_id`, or `None` for an id outside the three blocks (the store-test
    /// fields at `0xFD`/`0xFE`).
    pub const fn of(field_id: u8) -> Option<Scope> {
        match field_id {
            0x01..=0x1F => Some(Scope::Node),
            0x20..=0x3F => Some(Scope::Machine),
            0x40..=0x7F => Some(Scope::Wheel),
            _ => None,
        }
    }
}

/// When a written value takes effect on a board (`specs/machine-settings.md` section 5), a
/// registry attribute of every row.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Effect {
    /// Applied as it is written: the tune lane's fields, whose persisted write the gain shadow
    /// re-reads.
    Live,
    /// Read at boot and again at every arm: exactly the keys of the firmware's arm-time row
    /// (`firmware::arm::ARM_ROW`, which a test holds to this attribute).
    Arm,
    /// Read once at boot.
    Boot,
    /// No firmware consumer: a client's fact, stored on the board.
    Client,
}

// ---------------------------------------------------------------------------
// The enumerable registry: the runtime `field_id -> (Type, default)` view of the field set, derived
// from the typed handles so the handle stays the single source of truth (no parallel data table to
// drift). This is the deferred Layer-3 dependency, un-deferred for `net`'s `CONFIG_*`: a schema-less
// controller looks a field up by raw `field_id` to learn its `Type` (to decode a value and validate a
// write) and its default (returned when the key is absent). See `specs/storage-layer.md`.
// ---------------------------------------------------------------------------

/// One field's runtime descriptor: its permanent `field_id`, storage [`Type`], default [`Value`]
/// and [`Effect`]. Built from a registry row via [`FieldEntry::def`], which is itself a typed
/// handle's own `entry()` (so a field's id/type/default are still written once).
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
    /// When a written value takes effect on a board.
    pub effect: Effect,
}

/// The number of ENTRIES in the registry: the declared field count plus the extra per-index
/// entries the [`IndexedField`] families contribute (the link pins, the IMU's bias, signs and
/// roles, the level trim, the battery-sense calibration, the gains and their maxima, the
/// reference and the turn bound). Tracks the field set under each `test-fields` configuration.
#[cfg(not(feature = "test-fields"))]
pub const REGISTRY_LEN: usize = 58 + 17;
/// The number of registry entries (with the reserved store-test fields); see the non-test twin.
#[cfg(feature = "test-fields")]
pub const REGISTRY_LEN: usize = 60 + 17;

/// One registry ROW: the same five facts [`FieldDef`] carries, packed to 8 B.
///
/// [`FieldDef`] holds its default as a [`Value`], whose `U64`/`I64` variants give it an 8-byte
/// alignment and make the descriptor 24 B. That alignment is flash spent on padding and on a
/// pointer-sized slot that no registered field needs: every scalar field in the set is 32 bits or
/// narrower, and the only `Str`/`Blob` defaults are one literal and the empty slice. A row
/// therefore carries the default as a 32-bit `bits` word (measured: -928 B of flashed span,
/// `specs/decision-flash-budget.md`, shrink round 4).
///
/// The handles stay the single source of truth: every row is a handle's own `entry()`, and
/// [`FieldEntry::def`] is the one place a [`FieldDef`] is built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldEntry {
    /// The field's permanent id.
    pub field_id: u8,
    /// The index this row describes, as [`FieldDef::index`].
    pub index: u8,
    /// The field's storage type.
    pub kind: Type,
    /// The default, encoded for this row's `kind`: a scalar's bit pattern (sign-extended for the
    /// signed types, which `def` reverses exactly), or, for `Str`/`Blob`, the slot in
    /// [`BYTES_DEFAULTS`].
    bits: u32,
    /// As [`FieldDef::effect`]. It sits in the byte of padding the row already carried, so the row
    /// stays 8 B.
    pub effect: Effect,
}

/// Every `STR`/`BLOB` default the registry declares, by slot, so a row can name its bytes in 4
/// bytes instead of carrying a pointer and a length.
///
/// It is not a second place to declare a default: each slot is the handle's own `default()`, and
/// `every_bytes_row_resolves_to_its_handles_default` walks the registry and fails if a row's slot
/// stops agreeing with the handle it belongs to.
static BYTES_DEFAULTS: [&[u8]; 2] = [DEVICE_NAME.default(), &[]];

/// [`DEVICE_NAME`]'s slot in [`BYTES_DEFAULTS`].
const BYTES_DEVICE_NAME: u32 = 0;
/// The empty default, shared by every `STR` and `BLOB` field that declares one.
const BYTES_EMPTY: u32 = 1;

impl FieldEntry {
    /// Build a row. Called by the handles' `entry()`, which is what fixes `bits`' meaning for the
    /// row's `kind`. Every row names its [`Effect`]: there is no default to forget.
    pub(crate) const fn new(
        field_id: u8,
        index: u8,
        kind: Type,
        bits: u32,
        effect: Effect,
    ) -> Self {
        Self {
            field_id,
            index,
            kind,
            bits,
            effect,
        }
    }

    /// This row's key.
    pub const fn key(&self) -> Key {
        Key {
            field_id: self.field_id,
            index: self.index,
        }
    }

    /// This row's default as its stored value's bytes: a scalar's little-endian bytes, of its
    /// type's width (taken out of `scratch`), or a `Str`/`Blob` default's own bytes. The
    /// bytes [`Self::def`]'s default encodes to, without building the [`Value`].
    pub fn default_bytes<'a>(&self, scratch: &'a mut [u8; 4]) -> &'a [u8] {
        match self.kind.fixed_len() {
            Some(w) => {
                *scratch = self.bits.to_le_bytes();
                &scratch[..w.min(4)]
            }
            None => BYTES_DEFAULTS[self.bits as usize],
        }
    }

    /// This row as the runtime [`FieldDef`] every consumer reads: the one place the descriptor is
    /// built, so the packing above has a single inverse.
    ///
    /// The `U64`/`I64` arms are reachable only through a row that does not exist: `entry()` is
    /// emitted for the 32-bit-and-narrower scalars alone, so a `Field<u64>` cannot be registered
    /// at all (it is a compile error, not a truncation). They widen what the row holds, which is
    /// the only answer consistent with the packing.
    pub const fn def(self) -> FieldDef {
        let default = match self.kind {
            Type::U8 => Value::U8(self.bits as u8),
            Type::U16 => Value::U16(self.bits as u16),
            Type::U32 => Value::U32(self.bits),
            Type::U64 => Value::U64(self.bits as u64),
            Type::I16 => Value::I16(self.bits as i16),
            Type::I32 => Value::I32(self.bits as i32),
            Type::I64 => Value::I64(self.bits as i32 as i64),
            Type::Bool => Value::Bool(self.bits != 0),
            Type::Str => Value::Str(BYTES_DEFAULTS[self.bits as usize]),
            Type::Blob => Value::Bytes(BYTES_DEFAULTS[self.bits as usize]),
        };
        FieldDef {
            field_id: self.field_id,
            index: self.index,
            kind: self.kind,
            default,
            effect: self.effect,
        }
    }
}

use Effect::{Arm, Boot, Client, Live};

/// The full field registry, derived from the typed handles, in `(field_id, index)` order. Enumerable
/// (iterate it, taking each row's [`FieldEntry::def`]) and the basis for [`lookup`]. Each row
/// states its field's [`Effect`] (`specs/machine-settings.md` section 5): the section-3 tables,
/// checked against each reader in the firmware.
///
/// A per-motor field has one row, index 0: a board drives one motor in this version, and every
/// motor index reads that row's default.
///
/// A `static` in flash, deliberately, and this is a **stack** decision rather than a flash one. It was
/// a `fn registry() -> [FieldDef; REGISTRY_LEN]` returning the array BY VALUE, so every call
/// materialized the whole table into the caller's frame. `lookup` calls it on every dynamic
/// `get_value` / `set_value`, which put a **920 B** frame at the bottom of the deepest chain in the
/// image (`service_loop -> ingest -> apply_write -> set_value -> lookup`) and took the measured margin
/// on the bench master to 128 B of a 2,284 B painted stack, under the 250 B floor
/// (`specs/bench-evidence/2026-08-13/negative-control.md`). Held in flash and scanned by reference, the
/// same table costs the stack nothing.
pub static REGISTRY: [FieldEntry; REGISTRY_LEN] = [
    // Node (`0x01..=0x1F`).
    NODE_ADDRESS.entry(Boot),
    DEVICE_NAME.entry(BYTES_DEVICE_NAME, Boot),
    WIFI_SSID.entry(BYTES_EMPTY, Boot),
    WIFI_PASS.entry(BYTES_EMPTY, Boot),
    WIFI_TX_POWER.entry(Boot),
    BOARD_SELF_HOLD.entry(Boot),
    BOARD_BATTERY_PIN.entry(Boot),
    // No firmware reader drives the buzzer or the LEDs yet: client only until one does.
    BOARD_BUZZER.entry(Client),
    LED_GREEN.entry(Client),
    LED_ORANGE.entry(Client),
    LED_RED.entry(Client),
    PAD_A.entry(Boot),
    PAD_B.entry(Boot),
    BOARD_BUTTON.entry(Boot),
    LINK_TX.at(0).entry(Boot),
    LINK_TX.at(1).entry(Boot),
    LINK_RX.at(0).entry(Boot),
    LINK_RX.at(1).entry(Boot),
    IMU_SCL_PIN.entry(Boot),
    IMU_SDA_PIN.entry(Boot),
    IMU_MODEL.entry(Boot),
    IMU_GYRO_BIAS.at(0).entry(Boot),
    IMU_GYRO_BIAS.at(1).entry(Boot),
    IMU_GYRO_BIAS.at(2).entry(Boot),
    IMU_AXIS_SIGN.at(0).entry(Arm),
    IMU_AXIS_SIGN.at(1).entry(Arm),
    IMU_AXIS_SIGN.at(2).entry(Arm),
    IMU_AXIS_ROLE.at(0).entry(Arm),
    IMU_AXIS_ROLE.at(1).entry(Arm),
    ATTITUDE_LEVEL_TRIM.at(0).entry(Boot),
    ATTITUDE_LEVEL_TRIM.at(1).entry(Boot),
    BOARD_BATTERY_CAL.at(0).entry(Boot),
    BOARD_BATTERY_CAL.at(1).entry(Boot),
    BOARD_INPUT_POLARITY.entry(Boot),
    // Machine (`0x20..=0x3F`).
    CONTROL_MODE.entry(Arm),
    CONTROL_FRAME.entry(Arm),
    CONTROL_RIDER_REQUIRED.entry(Arm),
    MOTOR_METHOD.entry(Arm),
    BOARD_BATTERY_CELLS.entry(Client),
    BOARD_BATTERY_CHEMISTRY.entry(Client),
    BOARD_BATTERY_FLOOR.entry(Arm),
    CONTROL_SF_GAINS.at(0).entry(Live),
    CONTROL_SF_GAINS.at(1).entry(Live),
    CONTROL_SF_GAINS.at(2).entry(Live),
    CONTROL_SF_GAINS.at(3).entry(Live),
    CONTROL_GAIN_MAX.at(0).entry(Arm),
    CONTROL_GAIN_MAX.at(1).entry(Arm),
    CONTROL_GAIN_MAX.at(2).entry(Arm),
    CONTROL_GAIN_MAX.at(3).entry(Arm),
    CONTROL_REFERENCE.at(0).entry(Live),
    CONTROL_REFERENCE.at(1).entry(Live),
    CONTROL_POS_CLAMP.entry(Arm),
    CONTROL_DRIVE_TURN.at(0).entry(Arm),
    CONTROL_DRIVE_TURN.at(1).entry(Arm),
    // Wheel (`0x40..=0x7F`), one row per field at motor 0.
    MOTOR_HALL_A.entry(Boot),
    MOTOR_HALL_B.entry(Boot),
    MOTOR_HALL_C.entry(Boot),
    MOTOR_GATE_HI_A.entry(Boot),
    MOTOR_GATE_HI_B.entry(Boot),
    MOTOR_GATE_HI_C.entry(Boot),
    MOTOR_GATE_LO_A.entry(Boot),
    MOTOR_GATE_LO_B.entry(Boot),
    MOTOR_GATE_LO_C.entry(Boot),
    MOTOR_PHASE_A.entry(Boot),
    MOTOR_PHASE_B.entry(Boot),
    MOTOR_DEAD_TIME.entry(Boot),
    MOTOR_CURRENT_CAL.entry(Arm),
    MOTOR_NOISE_FLOOR.entry(Boot),
    MOTOR_CURRENT_LIMIT.entry(Arm),
    MOTOR_DIRECTION.entry(Arm),
    CONTROL_SIDE.entry(Arm),
    MOTOR_ALIGN_OFFSET.entry(Arm),
    MOTOR_POLE_PAIRS.entry(Client),
    WHEEL_DIAMETER_MM.entry(Client),
    MOTOR_ODOMETER.entry(Boot),
    #[cfg(feature = "test-fields")]
    T_BLOB.entry(BYTES_EMPTY, Client),
    #[cfg(feature = "test-fields")]
    T_KEY.entry(Client),
];

/// Look a field up by its raw `field_id`, or `None` if no field declares it (an `UnknownKey` on the
/// dynamic path). Linear over the small registry.
///
/// It scans [`REGISTRY`] **by reference** and builds a [`FieldDef`] from the one matching 8 B row.
/// Iterating by value (`REGISTRY.into_iter()`, or the old `registry()` call) would first copy the
/// whole table into this frame, which is the regression this shape exists to prevent. The gate on that is
/// `dynamic_config_write_costs_no_extra_stack_chip1k` in `tools/emulator-runner`, which measures the
/// real image's stack excursion under Unicorn; there is no host-side gate, and
/// `shared/store/src/tests.rs` records why one is not possible.
pub fn lookup(field_id: u8) -> Option<FieldDef> {
    lookup_key(Key { field_id, index: 0 })
}

/// Look a field up by its full [`Key`]: the entry for THIS index when the field declares one
/// ([`IndexedField`]), else the field's base entry. `None` if no field declares the id.
///
/// The index only ever selects a different DEFAULT; type and id are per-field. This is the form
/// the dynamic `get_value` / `set_value` path uses, because an absent key must read the default of
/// the index that was asked for, not of index 0. Both passes scan [`REGISTRY`] by reference and
/// build the descriptor from the one matching row, for the reason [`lookup`] records.
pub fn lookup_key(key: Key) -> Option<FieldDef> {
    let mut base = None;
    for d in REGISTRY.iter() {
        if d.field_id != key.field_id {
            continue;
        }
        if d.index == key.index {
            return Some(d.def());
        }
        if d.index == 0 {
            base = Some(d.def());
        }
    }
    base
}

// ---------------------------------------------------------------------------
// The name table (`specs/cli-config.md` section 3): every field's human-readable name, index kind
// and display kind, for the host tools. HOST-ONLY: behind the `names` feature (and the store's own
// tests), so no name reaches the firmware image. Each row takes the handle itself, so a row cannot
// name an id or a storage type its handle does not have, and the completeness test below holds it
// to `REGISTRY` both ways.
// ---------------------------------------------------------------------------

/// The name table and its vocabulary (`specs/cli-config.md` section 3).
#[cfg(any(test, feature = "names"))]
pub mod names {
    use super::*;

    /// How many values a field has and what its index means (`specs/cli-config.md` section 2.1).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Index {
        /// One value, index 0.
        Single,
        /// A fixed family of `n` values, indices `0..n`.
        Array(u8),
        /// One value per motor, the index being the motor (`0..`[`MOTORS`]), in a file under the
        /// top-level `motors` array.
        Motor,
        /// One value per inter-board link, the index being the link (`0..`[`LINKS`]), in a file
        /// under the top-level `links` array (`specs/machine-settings.md` section 3.1).
        Link,
    }

    impl Index {
        /// How many indices the table names.
        pub const fn count(self) -> u8 {
            match self {
                Index::Single => 1,
                Index::Array(n) => n,
                Index::Motor => MOTORS,
                Index::Link => LINKS,
            }
        }

        /// The top-level array an object-array field sits in (`motors`, `links`), or `None` for a
        /// field that is a value at its own path.
        pub const fn array(self) -> Option<&'static str> {
            match self {
                Index::Motor => Some("motors"),
                Index::Link => Some("links"),
                Index::Single | Index::Array(_) => None,
            }
        }
    }

    /// How a value is written in a file (`specs/cli-config.md` section 2.2). The storage type is
    /// the handle's; this is what the tools make of it.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Kind {
        /// A packed `(port << 4) | pin` byte, `"PC13"`, or `"none"` for `0xFF`.
        Pin,
        /// An integer, range-checked against the storage type.
        Int,
        /// `0` and `1` as `false` and `true`.
        Bool,
        /// A label per value.
        Enum(&'static [(u8, &'static str)]),
        /// The names of the set bits, by bit number.
        Bits(&'static [(u8, &'static str)]),
        /// A string.
        Str,
        /// Bytes, as hex.
        Blob,
    }

    /// One row of [`NAMES`]: a field's id and storage type (from its handle), its path, index kind
    /// and display kind.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct FieldName {
        /// The field's id.
        pub id: u8,
        /// The field's storage type.
        pub storage: Type,
        /// The dotted name (`hall.a`, `battery_sense.cal`, `machine.mode`).
        pub path: &'static str,
        /// How many values, and what the index means.
        pub index: Index,
        /// How a value is written.
        pub kind: Kind,
        /// An inclusive range the table states beyond the storage type (`align_offset`'s `0..5`),
        /// which a tool refuses without `--force`.
        pub range: Option<(i64, i64)>,
    }

    impl FieldName {
        const fn new(id: u8, storage: Type, path: &'static str, index: Index, kind: Kind) -> Self {
            FieldName {
                id,
                storage,
                path,
                index,
                kind,
                range: None,
            }
        }

        /// A field with one value.
        pub const fn single<T: Scalar>(f: Field<T>, path: &'static str, kind: Kind) -> Self {
            Self::new(f.id(), T::KIND, path, Index::Single, kind)
        }

        /// A field with one value per motor.
        pub const fn motor<T: Scalar>(f: Field<T>, path: &'static str, kind: Kind) -> Self {
            Self::new(f.id(), T::KIND, path, Index::Motor, kind)
        }

        /// A fixed family whose count is the handle's own.
        pub const fn array<T: Scalar, const N: usize>(
            f: IndexedField<T, N>,
            path: &'static str,
            kind: Kind,
        ) -> Self {
            Self::new(f.id(), T::KIND, path, Index::Array(N as u8), kind)
        }

        /// A field with one value per link, whose count is the handle's own and [`LINKS`].
        pub const fn link<T: Scalar>(
            f: IndexedField<T, { LINKS as usize }>,
            path: &'static str,
            kind: Kind,
        ) -> Self {
            Self::new(f.id(), T::KIND, path, Index::Link, kind)
        }

        /// A `STR` field.
        pub const fn text(f: StrField, path: &'static str) -> Self {
            Self::new(f.id(), Type::Str, path, Index::Single, Kind::Str)
        }

        /// A `BLOB` field.
        #[cfg(feature = "test-fields")]
        pub const fn blob(f: BlobField, path: &'static str) -> Self {
            Self::new(f.id(), Type::Blob, path, Index::Single, Kind::Blob)
        }

        /// This row with the inclusive range `lo..=hi`.
        pub const fn range(self, lo: i64, hi: i64) -> Self {
            FieldName {
                range: Some((lo, hi)),
                ..self
            }
        }

        /// Whether the board or the walk writes this field ([`BOARD_OWNED`]).
        pub fn board_owned(&self) -> bool {
            BOARD_OWNED.contains(&self.id)
        }

        /// Whether a node may answer this field's default itself ([`NODE_DEFAULTS`]).
        pub fn node_default(&self) -> bool {
            NODE_DEFAULTS.contains(&self.id)
        }

        /// Whether no node reads this field back ([`SECRET`]).
        pub fn secret(&self) -> bool {
            SECRET.contains(&self.id)
        }

        /// What the field describes ([`Scope::of`]).
        pub fn scope(&self) -> Option<Scope> {
            Scope::of(self.id)
        }

        /// When a written value takes effect on a board: the registry's attribute.
        pub fn effect(&self) -> Effect {
            lookup(self.id).map_or(Effect::Client, |d| d.effect)
        }
    }

    const METHOD: Kind = Kind::Enum(&[(0, "six_step"), (1, "sine"), (2, "foc")]);
    const MODE: Kind = Kind::Enum(&[(0, "throttle"), (1, "balance")]);
    const FRAME: Kind = Kind::Enum(&[(0, "pivoting"), (1, "rigid")]);
    const SIDE: Kind = Kind::Enum(&[(0, "left"), (1, "right")]);
    const DIRECTION: Kind = Kind::Enum(&[(0, "forward"), (1, "reverse")]);
    const AXIS: Kind = Kind::Enum(&[(1, "x"), (2, "y"), (3, "z")]);
    const CHEMISTRY: Kind = Kind::Enum(&[(0, "li_ion"), (1, "lev50"), (2, "lto")]);
    /// `active_high`'s bits name the input pins they describe, by path
    /// (`specs/machine-settings.md` section 3.1).
    const ACTIVE_HIGH: Kind = Kind::Bits(&[(0, "power.button"), (1, "pads.a"), (2, "pads.b")]);

    /// Every registered field's name (`specs/machine-settings.md` section 3), in id order.
    pub static NAMES: &[FieldName] = &[
        // Node.
        FieldName::single(NODE_ADDRESS, "node.address", Kind::Int),
        FieldName::text(DEVICE_NAME, "device.name"),
        FieldName::text(WIFI_SSID, "wifi.ssid"),
        FieldName::text(WIFI_PASS, "wifi.pass"),
        FieldName::single(WIFI_TX_POWER, "wifi.tx_power", Kind::Int).range(
            *WIFI_TX_POWER_RANGE.start() as i64,
            *WIFI_TX_POWER_RANGE.end() as i64,
        ),
        FieldName::single(BOARD_SELF_HOLD, "power.self_hold", Kind::Pin),
        FieldName::single(BOARD_BATTERY_PIN, "battery_sense.pin", Kind::Pin),
        FieldName::single(BOARD_BUZZER, "buzzer", Kind::Pin),
        FieldName::single(LED_GREEN, "leds.green", Kind::Pin),
        FieldName::single(LED_ORANGE, "leds.orange", Kind::Pin),
        FieldName::single(LED_RED, "leds.red", Kind::Pin),
        FieldName::single(PAD_A, "pads.a", Kind::Pin),
        FieldName::single(PAD_B, "pads.b", Kind::Pin),
        FieldName::single(BOARD_BUTTON, "power.button", Kind::Pin),
        FieldName::link(LINK_TX, "tx", Kind::Pin),
        FieldName::link(LINK_RX, "rx", Kind::Pin),
        FieldName::single(IMU_SCL_PIN, "imu.scl", Kind::Pin),
        FieldName::single(IMU_SDA_PIN, "imu.sda", Kind::Pin),
        FieldName::single(IMU_MODEL, "imu.model", Kind::Int),
        FieldName::array(IMU_GYRO_BIAS, "imu.gyro_bias", Kind::Int),
        FieldName::array(IMU_AXIS_SIGN, "imu.axis_sign", Kind::Int),
        FieldName::array(IMU_AXIS_ROLE, "imu.axis_role", AXIS),
        FieldName::array(ATTITUDE_LEVEL_TRIM, "attitude.level_trim", Kind::Int),
        FieldName::array(BOARD_BATTERY_CAL, "battery_sense.cal", Kind::Int),
        FieldName::single(BOARD_INPUT_POLARITY, "active_high", ACTIVE_HIGH),
        // Machine.
        FieldName::single(CONTROL_MODE, "machine.mode", MODE),
        FieldName::single(CONTROL_FRAME, "machine.frame", FRAME),
        FieldName::single(CONTROL_RIDER_REQUIRED, "machine.rider_required", Kind::Bool),
        FieldName::single(MOTOR_METHOD, "machine.method", METHOD),
        FieldName::single(BOARD_BATTERY_CELLS, "machine.battery.cells", Kind::Int),
        FieldName::single(
            BOARD_BATTERY_CHEMISTRY,
            "machine.battery.chemistry",
            CHEMISTRY,
        ),
        FieldName::single(BOARD_BATTERY_FLOOR, "machine.battery.floor", Kind::Int),
        FieldName::array(CONTROL_SF_GAINS, "machine.balance.gains", Kind::Int),
        FieldName::array(CONTROL_GAIN_MAX, "machine.balance.gain_max", Kind::Int),
        FieldName::array(CONTROL_REFERENCE, "machine.balance.reference", Kind::Int),
        FieldName::single(CONTROL_POS_CLAMP, "machine.balance.pos_clamp", Kind::Int),
        FieldName::array(CONTROL_DRIVE_TURN, "machine.turn", Kind::Int),
        // Wheel.
        FieldName::motor(MOTOR_HALL_A, "hall.a", Kind::Pin),
        FieldName::motor(MOTOR_HALL_B, "hall.b", Kind::Pin),
        FieldName::motor(MOTOR_HALL_C, "hall.c", Kind::Pin),
        FieldName::motor(MOTOR_GATE_HI_A, "gate.hi_a", Kind::Pin),
        FieldName::motor(MOTOR_GATE_HI_B, "gate.hi_b", Kind::Pin),
        FieldName::motor(MOTOR_GATE_HI_C, "gate.hi_c", Kind::Pin),
        FieldName::motor(MOTOR_GATE_LO_A, "gate.lo_a", Kind::Pin),
        FieldName::motor(MOTOR_GATE_LO_B, "gate.lo_b", Kind::Pin),
        FieldName::motor(MOTOR_GATE_LO_C, "gate.lo_c", Kind::Pin),
        FieldName::motor(MOTOR_PHASE_A, "current.phase_a", Kind::Pin),
        FieldName::motor(MOTOR_PHASE_B, "current.phase_b", Kind::Pin),
        FieldName::motor(MOTOR_DEAD_TIME, "gate.dead_time", Kind::Int),
        FieldName::motor(MOTOR_CURRENT_CAL, "current.cal", Kind::Int),
        FieldName::motor(MOTOR_NOISE_FLOOR, "current.noise_floor", Kind::Int),
        FieldName::motor(MOTOR_CURRENT_LIMIT, "current.limit", Kind::Int),
        FieldName::motor(MOTOR_DIRECTION, "direction", DIRECTION),
        FieldName::motor(CONTROL_SIDE, "side", SIDE),
        FieldName::motor(MOTOR_ALIGN_OFFSET, "align_offset", Kind::Int).range(0, 5),
        FieldName::motor(MOTOR_POLE_PAIRS, "pole_pairs", Kind::Int),
        FieldName::motor(WHEEL_DIAMETER_MM, "wheel_diameter_mm", Kind::Int),
        FieldName::motor(MOTOR_ODOMETER, "odometer", Kind::Int),
        #[cfg(feature = "test-fields")]
        FieldName::blob(T_BLOB, "test.blob"),
        #[cfg(feature = "test-fields")]
        FieldName::single(T_KEY, "test.key", Kind::Int),
    ];

    /// The row for `id`, if the table names it.
    pub fn by_id(id: u8) -> Option<&'static FieldName> {
        NAMES.iter().find(|r| r.id == id)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::string::{String, ToString};
        use std::vec::Vec;

        /// Every path a row produces, in the file's namespaces: an object-array row under its
        /// array (`motors`, `links`), every other row at the top level.
        fn paths() -> Vec<(Option<&'static str>, &'static str)> {
            NAMES.iter().map(|r| (r.index.array(), r.path)).collect()
        }

        #[test]
        fn every_registry_id_has_exactly_one_row_and_every_row_is_registered() {
            for id in FIELD_IDS {
                let n = NAMES.iter().filter(|r| r.id == *id).count();
                assert_eq!(n, 1, "{id:#04x} has {n} rows in NAMES");
            }
            for r in NAMES {
                assert!(
                    REGISTRY.iter().any(|e| e.field_id == r.id),
                    "{} names {:#04x}, which REGISTRY does not declare",
                    r.path,
                    r.id
                );
            }
        }

        #[test]
        fn no_two_rows_produce_the_same_path_or_nest_inside_another() {
            let all = paths();
            for (i, (arr_a, a)) in all.iter().enumerate() {
                // `motors` and `links` are the object arrays' own keys.
                let head = a.split('.').next();
                assert!(
                    arr_a.is_some() || (head != Some("motors") && head != Some("links")),
                    "{a}"
                );
                for (arr_b, b) in &all[i + 1..] {
                    if arr_a != arr_b {
                        continue;
                    }
                    assert_ne!(a, b, "two rows produce {a}");
                    let (pa, pb): (String, String) = (a.to_string() + ".", b.to_string() + ".");
                    assert!(
                        !pa.starts_with(&pb) && !pb.starts_with(&pa),
                        "{a} and {b} cannot both be a value and an object"
                    );
                }
            }
        }

        /// Machine paths are exactly the machine block (`specs/machine-settings.md` section 3.2),
        /// so a machine file's `machine` object holds machine fields and nothing else.
        #[test]
        fn the_machine_paths_are_the_machine_scope() {
            for r in NAMES {
                assert_eq!(
                    r.path.starts_with("machine."),
                    r.scope() == Some(Scope::Machine),
                    "{}",
                    r.path
                );
                assert_eq!(
                    r.index == Index::Motor,
                    r.scope() == Some(Scope::Wheel),
                    "{}",
                    r.path
                );
            }
        }

        #[test]
        fn every_row_agrees_with_its_registry_type_and_kind() {
            for r in NAMES {
                let def = lookup(r.id).unwrap();
                assert_eq!(r.storage, def.kind, "{}", r.path);
                let ok = match r.kind {
                    Kind::Pin | Kind::Bool | Kind::Enum(_) | Kind::Bits(_) => r.storage == Type::U8,
                    Kind::Int => r.storage.fixed_len().is_some() && r.storage != Type::Bool,
                    Kind::Str => r.storage == Type::Str,
                    Kind::Blob => r.storage == Type::Blob,
                };
                assert!(ok, "{} is {:?} over {:?}", r.path, r.kind, r.storage);
                if let Kind::Enum(labels) | Kind::Bits(labels) = r.kind {
                    for (i, (v, l)) in labels.iter().enumerate() {
                        for (w, m) in &labels[i + 1..] {
                            assert!(v != w && l != m, "{}: {l} and {m}", r.path);
                        }
                    }
                }
                if let Kind::Bits(bits) = r.kind {
                    assert!(bits.iter().all(|(b, _)| *b < 8), "{}", r.path);
                }
            }
        }

        /// An array's count is the registry's: one row per index, each carrying its default.
        #[test]
        fn an_array_rows_count_is_its_registry_rows() {
            for r in NAMES {
                let rows = REGISTRY.iter().filter(|e| e.field_id == r.id).count();
                let want = match r.index {
                    Index::Array(n) => n as usize,
                    Index::Link => LINKS as usize,
                    Index::Single | Index::Motor => 1,
                };
                assert_eq!(rows, want, "{}", r.path);
            }
        }

        /// `active_high`'s bit labels are the paths of the pins they describe.
        #[test]
        fn active_high_names_input_pin_paths() {
            let Kind::Bits(bits) = by_id(BOARD_INPUT_POLARITY.id()).unwrap().kind else {
                panic!("active_high is a bits field");
            };
            for (_, path) in bits {
                let row = NAMES.iter().find(|r| r.path == *path).unwrap();
                assert_eq!(row.kind, Kind::Pin, "{path}");
            }
        }

        #[test]
        fn the_board_owned_node_default_and_secret_marks() {
            let marked = |f: fn(&FieldName) -> bool| -> Vec<&str> {
                NAMES.iter().filter(|r| f(r)).map(|r| r.path).collect()
            };
            assert_eq!(marked(FieldName::board_owned), ["node.address", "odometer"]);
            assert_eq!(marked(FieldName::node_default), ["device.name"]);
            assert_eq!(marked(FieldName::secret), ["wifi.pass"]);
        }
    }
}

#[cfg(test)]
mod registry_tests {
    use super::*;
    use std::vec::Vec;

    #[test]
    fn registry_has_every_declared_field_with_its_handle_type_and_default() {
        let reg = &REGISTRY;
        assert_eq!(reg.len(), REGISTRY_LEN);
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
        // Spot-checks: id + kind + default come straight from the handle.
        let m = lookup(MOTOR_CURRENT_LIMIT.id()).unwrap();
        assert_eq!(
            (m.field_id, m.kind, m.default),
            (0x4E, Type::U32, Value::U32(10_000))
        );
        let n = lookup(DEVICE_NAME.id()).unwrap();
        assert_eq!(
            (n.field_id, n.kind, n.default),
            (0x02, Type::Str, Value::Str(b"Hoverboard"))
        );
        for (f, id) in [(WIFI_SSID, 0x03), (WIFI_PASS, 0x04)] {
            assert_eq!(f.id(), id);
            let d = lookup(id).unwrap();
            assert_eq!((d.kind, d.default), (Type::Str, Value::Str(b"")));
        }
        assert_eq!(SECRET, [WIFI_PASS.id()]);
        let t = lookup(WIFI_TX_POWER.id()).unwrap();
        assert_eq!(
            (t.field_id, t.kind, t.default),
            (0x05, Type::U8, Value::U8(34))
        );
        assert!(WIFI_TX_POWER_RANGE.contains(&WIFI_TX_POWER.default()));
        let r = lookup(CONTROL_RIDER_REQUIRED.id()).unwrap();
        assert_eq!((r.field_id, r.default), (0x22, Value::U8(1)));
        let c = lookup(MOTOR_CURRENT_CAL.id()).unwrap();
        assert_eq!((c.field_id, c.default), (0x4C, Value::U16(455)));
        let nf = lookup(MOTOR_NOISE_FLOOR.id()).unwrap();
        assert_eq!((nf.field_id, nf.default), (0x4D, Value::U16(2_100)));
        let f = lookup(BOARD_BATTERY_FLOOR.id()).unwrap();
        assert_eq!(
            (f.field_id, f.kind, f.default),
            (0x26, Type::I16, Value::I16(2400))
        );
        let sd = lookup(CONTROL_SIDE.id()).unwrap();
        assert_eq!((sd.field_id, sd.default), (0x50, Value::U8(0)));
        let ch = lookup(BOARD_BATTERY_CHEMISTRY.id()).unwrap();
        assert_eq!((ch.field_id, ch.default), (0x25, Value::U8(0)));
        assert_eq!(core::mem::size_of::<FieldEntry>(), 8);
    }

    /// The id map of `specs/machine-settings.md` section 3, as `(id, handle id)`.
    #[test]
    fn the_id_map() {
        let map: &[(u8, u8)] = &[
            (0x01, NODE_ADDRESS.id()),
            (0x02, DEVICE_NAME.id()),
            (0x03, WIFI_SSID.id()),
            (0x04, WIFI_PASS.id()),
            (0x05, WIFI_TX_POWER.id()),
            (0x06, BOARD_SELF_HOLD.id()),
            (0x07, BOARD_BATTERY_PIN.id()),
            (0x08, BOARD_BUZZER.id()),
            (0x09, LED_GREEN.id()),
            (0x0A, LED_ORANGE.id()),
            (0x0B, LED_RED.id()),
            (0x0C, PAD_A.id()),
            (0x0D, PAD_B.id()),
            (0x0E, BOARD_BUTTON.id()),
            (0x0F, LINK_TX.id()),
            (0x10, LINK_RX.id()),
            (0x11, IMU_SCL_PIN.id()),
            (0x12, IMU_SDA_PIN.id()),
            (0x13, IMU_MODEL.id()),
            (0x14, IMU_GYRO_BIAS.id()),
            (0x15, IMU_AXIS_SIGN.id()),
            (0x16, IMU_AXIS_ROLE.id()),
            (0x17, ATTITUDE_LEVEL_TRIM.id()),
            (0x18, BOARD_BATTERY_CAL.id()),
            (0x19, BOARD_INPUT_POLARITY.id()),
            (0x20, CONTROL_MODE.id()),
            (0x21, CONTROL_FRAME.id()),
            (0x22, CONTROL_RIDER_REQUIRED.id()),
            (0x23, MOTOR_METHOD.id()),
            (0x24, BOARD_BATTERY_CELLS.id()),
            (0x25, BOARD_BATTERY_CHEMISTRY.id()),
            (0x26, BOARD_BATTERY_FLOOR.id()),
            (0x27, CONTROL_SF_GAINS.id()),
            (0x28, CONTROL_GAIN_MAX.id()),
            (0x29, CONTROL_REFERENCE.id()),
            (0x2A, CONTROL_POS_CLAMP.id()),
            (0x2B, CONTROL_DRIVE_TURN.id()),
            (0x40, MOTOR_HALL_A.id()),
            (0x41, MOTOR_HALL_B.id()),
            (0x42, MOTOR_HALL_C.id()),
            (0x43, MOTOR_GATE_HI_A.id()),
            (0x44, MOTOR_GATE_HI_B.id()),
            (0x45, MOTOR_GATE_HI_C.id()),
            (0x46, MOTOR_GATE_LO_A.id()),
            (0x47, MOTOR_GATE_LO_B.id()),
            (0x48, MOTOR_GATE_LO_C.id()),
            (0x49, MOTOR_PHASE_A.id()),
            (0x4A, MOTOR_PHASE_B.id()),
            (0x4B, MOTOR_DEAD_TIME.id()),
            (0x4C, MOTOR_CURRENT_CAL.id()),
            (0x4D, MOTOR_NOISE_FLOOR.id()),
            (0x4E, MOTOR_CURRENT_LIMIT.id()),
            (0x4F, MOTOR_DIRECTION.id()),
            (0x50, CONTROL_SIDE.id()),
            (0x51, MOTOR_ALIGN_OFFSET.id()),
            (0x52, MOTOR_POLE_PAIRS.id()),
            (0x53, WHEEL_DIAMETER_MM.id()),
            (0x54, MOTOR_ODOMETER.id()),
        ];
        for (want, got) in map {
            assert_eq!(want, got);
        }
        #[cfg(not(feature = "test-fields"))]
        assert_eq!(map.len(), FIELD_IDS.len());
    }

    /// `specs/machine-settings.md` section 9, test 7: every registered id falls in the block of
    /// its scope, and no block has a free id below its last registered one.
    #[test]
    fn scope_blocks_are_dense() {
        for (scope, first) in [
            (Scope::Node, 0x01u8),
            (Scope::Machine, 0x20),
            (Scope::Wheel, 0x40),
        ] {
            let ids: Vec<u8> = FIELD_IDS
                .iter()
                .copied()
                .filter(|id| Scope::of(*id) == Some(scope))
                .collect();
            let last = *ids.iter().max().unwrap();
            for id in first..=last {
                assert!(ids.contains(&id), "{scope:?} has a free id {id:#04x}");
            }
        }
        assert_eq!(Scope::of(0x1F), Some(Scope::Node));
        assert_eq!(Scope::of(0x20), Some(Scope::Machine));
        assert_eq!(Scope::of(0x3F), Some(Scope::Machine));
        assert_eq!(Scope::of(0x40), Some(Scope::Wheel));
        assert_eq!(Scope::of(0x7F), Some(Scope::Wheel));
        assert_eq!(Scope::of(0x00), None);
        assert_eq!(Scope::of(0xFD), None);
        assert_eq!(Scope::of(0xFE), None);
        for id in FIELD_IDS {
            if !matches!(id, 0xFD | 0xFE) {
                assert!(Scope::of(*id).is_some(), "{id:#04x} has no scope");
            }
        }
    }

    /// The registry is in `(field_id, index)` order: the machine digest is over the machine rows
    /// in id order, and the digest reads them in registry order.
    #[test]
    fn the_registry_is_in_key_order() {
        for w in REGISTRY.windows(2) {
            assert!(
                (w[0].field_id, w[0].index) < (w[1].field_id, w[1].index),
                "{:#04x}.{} before {:#04x}.{}",
                w[0].field_id,
                w[0].index,
                w[1].field_id,
                w[1].index
            );
        }
    }

    /// The effects of `specs/machine-settings.md` section 3, by class.
    #[test]
    fn the_effects() {
        let with = |e: Effect| -> Vec<(u8, u8)> {
            REGISTRY
                .iter()
                .filter(|r| r.effect == e)
                .map(|r| (r.field_id, r.index))
                .collect()
        };
        assert_eq!(
            with(Effect::Live),
            [
                (0x27, 0),
                (0x27, 1),
                (0x27, 2),
                (0x27, 3),
                (0x29, 0),
                (0x29, 1)
            ]
        );
        assert_eq!(
            with(Effect::Client),
            [
                (0x08, 0),
                (0x09, 0),
                (0x0A, 0),
                (0x0B, 0),
                (0x24, 0),
                (0x25, 0),
                (0x52, 0),
                (0x53, 0),
            ]
            .into_iter()
            .chain(
                [(0xFD, 0), (0xFE, 0)]
                    .into_iter()
                    .filter(|_| cfg!(feature = "test-fields"))
            )
            .collect::<Vec<_>>()
        );
        assert_eq!(
            with(Effect::Arm),
            [
                (0x15, 0),
                (0x15, 1),
                (0x15, 2),
                (0x16, 0),
                (0x16, 1),
                (0x20, 0),
                (0x21, 0),
                (0x22, 0),
                (0x23, 0),
                (0x26, 0),
                (0x28, 0),
                (0x28, 1),
                (0x28, 2),
                (0x28, 3),
                (0x2A, 0),
                (0x2B, 0),
                (0x2B, 1),
                (0x4C, 0),
                (0x4E, 0),
                (0x4F, 0),
                (0x50, 0),
                (0x51, 0),
            ]
        );
        // Everything else is read at boot.
        for r in REGISTRY.iter() {
            let classed = [Effect::Live, Effect::Client, Effect::Arm].contains(&r.effect);
            assert!(classed || r.effect == Effect::Boot);
        }
    }

    #[test]
    fn lookup_of_an_undeclared_id_is_none() {
        assert!(lookup(0x99).is_none());
        // The retired ids are gone with their fields.
        for id in [0x30u8, 0x66, 0x78] {
            assert!(lookup(id).is_none(), "{id:#04x}");
        }
    }

    /// The per-index defaults resolve per index on BOTH paths, and an index the family does not
    /// declare falls back to the base entry.
    #[test]
    fn an_indexed_family_resolves_its_default_per_index() {
        assert_eq!(CONTROL_GAIN_MAX.at(0).default(), 20000);
        assert_eq!(CONTROL_GAIN_MAX.at(2).default(), 2000);
        assert_eq!(CONTROL_GAIN_MAX.at(3).default(), 10000);
        let d = |f: u8, i: u8| {
            lookup_key(Key {
                field_id: f,
                index: i,
            })
            .unwrap()
            .default
        };
        for i in 0..4 {
            assert_eq!(d(0x27, i), Value::I16(0));
        }
        assert_eq!(d(0x28, 0), Value::I16(20000));
        assert_eq!(d(0x28, 2), Value::I16(2000));
        assert_eq!(d(0x28, 3), Value::I16(10000));
        assert_eq!(d(0x29, 1), Value::I16(41));
        assert_eq!(d(0x2A, 0), Value::I16(45));
        assert_eq!(d(0x18, 0), Value::I16(25200));
        assert_eq!(d(0x18, 1), Value::I16(-5));
        assert_eq!(d(0x2B, 1), Value::I16(24));
        // Past the declared end: the base default.
        assert_eq!(d(0x28, 7), Value::I16(20000));
        assert_eq!(CONTROL_GAIN_MAX.at(7).key().index, 0);
        // A per-motor field: one row, every motor index reads its default.
        assert_eq!(d(0x4C, 1), Value::U16(455));
        assert_eq!(MOTOR_CURRENT_CAL.at(1).key().index, 1);
        // The IMU's families: three signs, two roles, three biases.
        assert_eq!(
            (
                IMU_AXIS_SIGN.len(),
                IMU_AXIS_ROLE.len(),
                IMU_GYRO_BIAS.len()
            ),
            (3, 2, 3)
        );
        assert_eq!(ATTITUDE_LEVEL_TRIM.len(), 2);
    }

    /// The inter-board link pins: two indices each, both unset by default.
    #[test]
    fn the_link_pins_are_two_unset_pin_families() {
        assert_eq!((LINK_TX.id(), LINK_RX.id(), LINKS), (0x0F, 0x10, 2));
        for f in [LINK_TX, LINK_RX] {
            for i in 0..LINKS {
                let d = lookup_key(f.at(i).key()).unwrap();
                assert_eq!(
                    (d.index, d.kind, d.default),
                    (i, Type::U8, Value::U8(PIN_ABSENT))
                );
            }
        }
    }

    /// The pin on [`BYTES_DEFAULTS`]: every `Str`/`Blob` row is checked against the handle it
    /// belongs to, and the count is checked too.
    #[test]
    fn every_bytes_row_resolves_to_its_handles_default() {
        let expected: &[(u8, &[u8])] = &[
            (DEVICE_NAME.id(), DEVICE_NAME.default()),
            (WIFI_SSID.id(), WIFI_SSID.default()),
            (WIFI_PASS.id(), WIFI_PASS.default()),
            #[cfg(feature = "test-fields")]
            (T_BLOB.id(), T_BLOB.default()),
        ];
        let mut seen = 0usize;
        for row in REGISTRY.iter() {
            if !matches!(row.kind, Type::Str | Type::Blob) {
                continue;
            }
            seen += 1;
            let (_, want) = expected
                .iter()
                .find(|(id, _)| *id == row.field_id)
                .unwrap_or_else(|| panic!("unlisted byte-defaulted field {:#04x}", row.field_id));
            let got = match row.def().default {
                Value::Str(b) | Value::Bytes(b) => b,
                other => panic!("{:#04x} is {:?}, not bytes", row.field_id, other),
            };
            assert_eq!(got, *want, "field {:#04x}", row.field_id);
        }
        assert_eq!(seen, expected.len());
    }
}
