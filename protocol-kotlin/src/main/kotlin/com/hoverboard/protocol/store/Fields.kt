package com.hoverboard.protocol.store

import com.hoverboard.protocol.board.PIN_ABSENT

/**
 * One registered store field as the firmware declares it: its permanent `field_id`, its storage
 * [type] and its typed [default] (what a board that never staged the field reads). A mirror of one
 * `Field<T>` / `StrField` handle in `crates/store/src/field.rs`, which is the single source of truth
 * for all three facts; `RustSourceDriftTest` pins every entry of [Fields.ALL] against it.
 *
 * How many indices a field has is NOT here: the Rust handle does not carry it (an indexed field is a
 * plain `Field<T>` whose index range lives in its doc comment and its consumer), so a client that
 * names an index takes it from the field's own documentation.
 *
 * [readOnly] mirrors a registry row marked `.read_only()`: the firmware's own fact, never stored, and
 * a `CONFIG_WRITE` of it is refused with [com.hoverboard.protocol.l3.Walk.CFG_READ_ONLY].
 */
data class FieldDef(val id: Int, val type: Type, val default: Value, val readOnly: Boolean = false) {
    init {
        require(default.kind() == type) { "default ${default.kind()} does not match type $type" }
    }

    /** The [Key] naming instance [index] of this field (a singleton uses 0). */
    fun key(index: Int = 0) = Key(id, index)
}

/**
 * One registered INDEXED store field whose indices default differently: a mirror of one
 * `IndexedField<T, N>` handle in `crates/store/src/field.rs`, carrying the id, the storage [type]
 * and one default per index ([defaults], so its size is the field's `N`). `RustSourceDriftTest` pins
 * every entry of [Fields.INDEXED] against it.
 */
data class IndexedFieldDef(val id: Int, val type: Type, val defaults: List<Value>) {
    init {
        require(defaults.all { it.kind() == type }) { "a default of $defaults does not match type $type" }
    }

    /** Instance [index] as a plain field with that index's default (the mirror of `IndexedField::at`). */
    fun at(index: Int): FieldDef = FieldDef(id, type, defaults[index])
}

/**
 * The registered store fields a settings client edits or displays (`specs/rider-ui.md` section 3.4),
 * mirrored from `crates/store/src/field.rs`, plus the value vocabularies the firmware gives the
 * byte-valued ones.
 *
 * Not every registered field is here: `SOME_BLOB` and the test fields belong to nobody. The live
 * tune lane's keys, ranges and ramp are in [Gains]. A field is added when a client exercises it,
 * which is why the board-layout block (0x40-0x55 plus the per-motor facts) is here now: the layout
 * editor's mirrored validator reads it (`specs/rider-ui.md` section 3.5,
 * [com.hoverboard.protocol.board.validate]).
 */
object Fields {
    /** The board's persistent L3 node address (walk-owned; displayed, never edited). */
    val NODE_ADDRESS = FieldDef(0x01, Type.U8, Value.U8(0))

    /** The L3 link-set bitmask (walk-owned; displayed, never edited). */
    val LINK_SET = FieldDef(0x02, Type.U8, Value.U8(0))

    /**
     * What a node is (`specs/l3.md`, "Node class"): `0x01` = motor controller, `0x02` = bridge.
     * Read-only; a board running this firmware reads `0x01`.
     */
    val NODE_CLASS = FieldDef(0x03, Type.U8, Value.U8(0x01), readOnly = true)

    /** The advertised device name. */
    val DEVICE_NAME = FieldDef(0x10, Type.Str, Value.Str("Hoverboard"))

    /** The motor current limit, milliamps. The firmware converts it against the board's own counts per amp at bring-up, then clamps the COUNT into the window the comparison admits (see [CURRENT_LIMIT_MA]). */
    val MOTOR_CURRENT_LIMIT = FieldDef(0x20, Type.U32, Value.U32(10_000))

    /** The requested commutation method ([MotorMethod]). */
    val MOTOR_METHOD = FieldDef(0x21, Type.U8, Value.U8(0))

    /** The runtime control mode ([ControlMode]). */
    val CONTROL_MODE = FieldDef(0x22, Type.U8, Value.U8(0))

    /**
     * Whether balance mode requires a rider ([RiderRequired]); the firmware reads any nonzero byte
     * as required (`specs/control.md` (i)).
     */
    val CONTROL_RIDER_REQUIRED = FieldDef(0x23, Type.U8, Value.U8(1))

    /**
     * The low-battery floor, centivolts: a balance engage is refused while the effective battery
     * word is below it, never a disengage (`specs/sensing-and-safety.md`, the battery floor). `<= 0` = no floor; no
     * clamp beyond the type.
     */
    val CONTROL_BATTERY_FLOOR = FieldDef(0x24, Type.I16, Value.I16(2400))

    /**
     * The machine's frame ([ControlFrame]): pivoting platforms, or one rigid body balanced from the
     * one board that has an IMU (`specs/rover-one-imu.md`, the one-IMU rover). Written the SAME to both boards of a
     * machine; a Balance board resolves it with its IMU fact into leader or follower.
     */
    val CONTROL_FRAME = FieldDef(0x25, Type.U8, Value.U8(0))

    /**
     * Which wheel of a rigid machine this board drives ([ControlSide]), the owner of the firmware
     * control block's `role_right` word (`specs/rover-one-imu.md` section 5). Written to BOTH
     * boards with DIFFERENT values, 1 on the right wheel's board; only the leader's copy reaches
     * the steering split, and the follower's is written because the side is that board's own fact
     * and the roles follow which board carries the IMU. Unknown bytes read as [ControlSide.LEFT].
     */
    val CONTROL_SIDE = FieldDef(0x26, Type.U8, Value.U8(0))

    /**
     * The board-layout pin block (`specs/board-model.md`, "The field vocabulary"): one `u8` per
     * function, packed `(port << 4) | pin`, and [com.hoverboard.protocol.board.PIN_ABSENT] (`0xFF`)
     * meaning the function is ABSENT on this board, which is a valid state and not an error.
     *
     * The defaults split by stakes, and the split is the firmware's, mirrored here: the benign
     * functions (the latch, the buzzer, the LEDs) default to the fleet-uniform pin, so a blank board
     * behaves like the pre-layout firmware, while the MOTOR groups, the IMU pair, the battery sense
     * and the foot pads default to ABSENT. A default that described a complete valid gate set would hand
     * a blank board a drivable motor layout, and the drive path has to be an explicit configuration
     * act; the IMU pair has no safe fleet default at all, because one family's I2C bus is the
     * other's BLE USART.
     */
    val BOARD_SELF_HOLD = FieldDef(0x40, Type.U8, Value.U8(0x1C))

    /** Battery sense. ABSENT by default: the pin is fleet-uniform but the SENSE is master-only. */
    val BOARD_VBATT = FieldDef(0x41, Type.U8, Value.U8(PIN_ABSENT))
    val BOARD_BUZZER = FieldDef(0x42, Type.U8, Value.U8(0x19))
    val LED_GREEN = FieldDef(0x43, Type.U8, Value.U8(0x13))
    val LED_ORANGE = FieldDef(0x44, Type.U8, Value.U8(0x0F))
    val LED_RED = FieldDef(0x45, Type.U8, Value.U8(0x14))
    /**
     * The foot pads. ABSENT by default, for [BOARD_VBATT]'s reason one step further on: the pins are
     * fleet-uniform (PA11 / PC15) but the PADS are not, and no board in this fleet has pad hardware
     * fitted. A defaulted pin leaves a floating line that the active-high debouncer reads as a foot,
     * which selects the RUN gain set and holds an engagement.
     */
    val PAD_A = FieldDef(0x46, Type.U8, Value.U8(PIN_ABSENT))
    val PAD_B = FieldDef(0x47, Type.U8, Value.U8(PIN_ABSENT))

    /** The IMU bus pins; no safe fleet default exists, so both are ABSENT until staged. */
    val IMU_SCL_PIN = FieldDef(0x48, Type.U8, Value.U8(PIN_ABSENT))
    val IMU_SDA_PIN = FieldDef(0x49, Type.U8, Value.U8(PIN_ABSENT))

    /** The shared front-end's hall inputs, per motor via `Key.index`. Configured, never defaulted. */
    val MOTOR_HALL_A = FieldDef(0x4A, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_HALL_B = FieldDef(0x4B, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_HALL_C = FieldDef(0x4C, Type.U8, Value.U8(PIN_ABSENT))

    /** The advanced timer's high-side set, per motor. */
    val MOTOR_GATE_HI_A = FieldDef(0x4D, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_GATE_HI_B = FieldDef(0x4E, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_GATE_HI_C = FieldDef(0x4F, Type.U8, Value.U8(PIN_ABSENT))

    /** The complementary low-side set, per motor. */
    val MOTOR_GATE_LO_A = FieldDef(0x50, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_GATE_LO_B = FieldDef(0x51, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_GATE_LO_C = FieldDef(0x52, Type.U8, Value.U8(PIN_ABSENT))

    /** The power-button sense; no fleet default is pinned yet, so unset until configured. */
    val BOARD_BUTTON = FieldDef(0x53, Type.U8, Value.U8(PIN_ABSENT))

    /** The two phase-current sense pins, per motor: the injected ADC group's ranks, in order. */
    val MOTOR_PHASE_A = FieldDef(0x54, Type.U8, Value.U8(PIN_ABSENT))
    val MOTOR_PHASE_B = FieldDef(0x55, Type.U8, Value.U8(PIN_ABSENT))

    /**
     * What the pack IS: the number of cells in series (`crates/store/src/field.rs`,
     * `BOARD_PACK_CELLS`). 0 = not stated, which is the default and is not a guess at eight.
     *
     * Written the SAME to both boards of a machine, the [CONTROL_FRAME] rule: one pack feeds both.
     *
     * The firmware does not read it. [CONTROL_BATTERY_FLOOR] stays an absolute threshold the
     * operator sets, so the one consumer is a client turning a pack voltage into a state of
     * charge, which multiplies the count by a per-cell voltage rather than carrying a pack-shaped
     * constant of its own. 0 is a client with nothing to turn a voltage into, which says so
     * instead of drawing a gauge at an assumed pack.
     */
    val BOARD_PACK_CELLS = FieldDef(0x6A, Type.U8, Value.U8(0))

    /**
     * Which electrical level means ASSERTED on the board's three discrete input lines
     * (`crates/store/src/field.rs`, `BOARD_INPUT_POLARITY`): b0 button, b1 pad A, b2 pad B, a set
     * bit meaning active high. Bits 3..7 are reserved and inert (`crates/inputs/src/lib.rs`,
     * `asserted`, masks them).
     *
     * A property of the input wiring fitted, not of the firmware, and a compiled literal until
     * 2026-10-24. The default 0b110 IS the behaviour the firmware held as three literals: the
     * button idles high through a pull-up and a press pulls it down, the pads read high with a
     * foot on. No fleet board is known to differ, so a board with no record behaves as before.
     */
    val BOARD_INPUT_POLARITY = FieldDef(0x6C, Type.U8, Value.U8(0b110))

    /** The IMU model index ([ImuModel]). */
    val IMU_MODEL = FieldDef(0x60, Type.U8, Value.U8(0))

    /** Per-axis zero-rate gyro bias, raw counts, indices 0/1/2 = x/y/z. */
    val IMU_GYRO_BIAS = FieldDef(0x61, Type.I32, Value.I32(0))

    /** Per-motor drive direction: 0 = forward, nonzero = reverse. */
    val MOTOR_DIRECTION = FieldDef(0x62, Type.U8, Value.U8(0))

    /** Per-motor six-step align offset, a sector rotation 0..5 (taken mod 6 by the decoder). */
    val MOTOR_ALIGN_OFFSET = FieldDef(0x63, Type.U8, Value.U8(0))

    /** Per-motor dead time, raw DTG; 0 = unset. */
    val MOTOR_DEAD_TIME = FieldDef(0x64, Type.U8, Value.U8(0))

    /**
     * Per-motor phase-current sense DECLARATION: 0 = none. The capability half of the
     * phase-current group, which the boot validator holds to agreeing with [MOTOR_PHASE_A] /
     * [MOTOR_PHASE_B], so a board cannot declare current sense with no channels behind it.
     */
    val MOTOR_CURRENT_SENSE = FieldDef(0x66, Type.U8, Value.U8(0))

    /**
     * Per-motor current-sense calibration, counts per amp: what a `motor_current` count MEANS,
     * beside [MOTOR_CURRENT_SENSE], which says where it is sensed. A property of the shunt and
     * amplifier chain fitted, so per-board data rather than a crate constant.
     */
    val MOTOR_CURRENT_CAL = FieldDef(0x67, Type.U16, Value.U16(455))

    /**
     * Per-motor POLE-PAIR count: how many ELECTRICAL revolutions the rotor turns per mechanical
     * one (`crates/store/src/field.rs`, `MOTOR_POLE_PAIRS`). With six hall edges per electrical
     * revolution, `6 * pole_pairs` edges make one revolution of the wheel, which is half of what
     * turns a [MOTOR_ODOMETER] count into a distance; [WHEEL_DIAMETER_MM] is the other half.
     *
     * **The default is 0, meaning UNSET, and a client must treat it as unset rather than guess.**
     * 15 is the count the fleet's 6.5 inch hubs are believed to have and the number every existing
     * derivation assumes, but it has been counted on no board of this fleet. Defaulting to 15
     * would make every board show a distance at a scale nobody checked, and the pole-pair count is
     * the one factor a reader cannot sanity-check from the result: a 20-pole-pair hub reads 25
     * percent long and still looks plausible. A wrong distance stated confidently is worse than no
     * distance, so a client with no count says it has none.
     *
     * **No client offers an editor for it.** The value is a MEASUREMENT, taken at a bench and
     * written with the host config tool: turn the wheel one full revolution, read the delta on
     * that motor's [MOTOR_ODOMETER] and divide by six. A typo here produces a confidently wrong
     * distance, and nothing on a phone can tell one from a right one.
     *
     * No firmware consumer either: nothing in the image converts anything to metres, so the store
     * is the fact's home and a client reads it over `CONFIG_READ`. Not in [ARM_TIME].
     */
    val MOTOR_POLE_PAIRS = FieldDef(0x6E, Type.U8, Value.U8(0))

    /**
     * Per-motor phase-current NOISE FLOOR, stock current counts: the MINIMUM soft limit the board's
     * sense chain can be held to. A staged [MOTOR_CURRENT_LIMIT] that converts below it is clamped
     * UP to it at bring-up, so it is the smallest current that board enforces.
     *
     * Per-board for the same reason as [MOTOR_CURRENT_CAL] and in the same units: it is the rest
     * noise of the shunt and amplifier chain fitted, and it was a compiled firmware constant
     * measured on one board in one session. The default is that constant, so a board with no record
     * enforces what the image always did. The firmware clamps it at boot ([NOISE_FLOOR]).
     *
     * **No client offers an editor for it.** A floor set too low chops on the chain's own noise and
     * a floor set too high raises the minimum enforceable current, and neither is visible from an
     * app: the value comes from a disarmed bench measurement (gate 1 of `tools/climit-session.py`
     * reports the rest-noise maximum), which is the diagnostic that belongs beside the write.
     */
    val MOTOR_NOISE_FLOOR = FieldDef(0x6B, Type.U16, Value.U16(2_100))

    /**
     * Per-motor LIFETIME TRAVEL, in absolute hall edges (`crates/store/src/field.rs`,
     * `MOTOR_ODOMETER`): every counted commutation edge adds 1 whichever way the wheel turned, so
     * the total never winds back. The live value is also on the wire, in the appended block
     * (`crates/linkctl/src/lib.rs`, `CyclicObs`).
     *
     * **Counts, not a distance.** Converting needs a pole-pair count and a wheel diameter
     * ([MOTOR_POLE_PAIRS] and [WHEEL_DIAMETER_MM]), both registered and both defaulting to unset,
     * so a client converts only for a board that has been told both and otherwise displays edges
     * and says they are edges rather than printing a guessed distance.
     *
     * **READ-ONLY to a client, and no client offers an editor for it.** It is the one field the
     * FIRMWARE writes on its own, once per ride, on a service-loop pass that finds the board
     * disarmed and the wheel stopped (`Odometer` in `crates/firmware/src/odometer.rs`). A client write would
     * either be overwritten by that pass or silently falsify a machine's recorded travel, and
     * there is no reading an operator could take that makes one correct.
     */
    val MOTOR_ODOMETER = FieldDef(0x6D, Type.U32, Value.U32(0))

    /**
     * This wheel's ROLLING diameter, millimetres (`crates/store/src/field.rs`,
     * `WHEEL_DIAMETER_MM`): the other half of the edges-to-distance conversion, beside
     * [MOTOR_POLE_PAIRS]. One wheel revolution covers `PI * diameter_mm`.
     *
     * **The default is 0, meaning UNSET.** The rover, the offroad chassis and a stock hoverboard
     * do not share a wheel size, so there is no fleet value to fall back on and 0 mm is not a
     * wheel. The figure wanted is the ROLLING diameter under load rather than the tyre's marked
     * size, which differs from it by a few percent.
     *
     * **No client offers an editor for it**, for [MOTOR_POLE_PAIRS]' reason: it is measured on the
     * machine and written at a bench with the host config tool, and a mistyped wheel scales every
     * distance the client prints with nothing to show it happened. No firmware consumer, and not
     * in [ARM_TIME].
     */
    val WHEEL_DIAMETER_MM = FieldDef(0x6F, Type.U16, Value.U16(0))

    /**
     * Per-axis IMU sign map, indices 0..5 = `ax, ay, az, gx, gy, gz`; 0 = unset (that index falls
     * back to the reference map). See [com.hoverboard.protocol.imu.Orientation].
     */
    val IMU_AXIS_SIGN = FieldDef(0x65, Type.I32, Value.I32(0))

    /**
     * The IMU axis roles, indices 0 = UP, 1 = PITCH_RATE: which chip axis plays each body role
     * (`1 = X`, `2 = Y`, `3 = Z`); 0 = unset (that index falls back to the compiled role). Half of
     * one frame with [IMU_AXIS_SIGN]. See [com.hoverboard.protocol.imu.Orientation].
     */
    val IMU_AXIS_ROLE = FieldDef(0x68, Type.U8, Value.U8(0))

    /** Per-board attitude level trim, centidegrees, indices 0 = pitch, 1 = roll. */
    val ATTITUDE_LEVEL_TRIM = FieldDef(0x70, Type.I16, Value.I16(0))

    /**
     * The battery-sense calibration, indices 0 = slope (microvolts of rail per 12-bit count) and
     * 1 = offset (centivolts): `centivolts = raw12 * slope / 10_000 + offset`. The firmware clamps
     * each at boot ([VBATT_SLOPE], [VBATT_OFFSET]).
     */
    val BOARD_VBATT_CAL = IndexedFieldDef(0x69, Type.I16, listOf(Value.I16(25_200), Value.I16(-5)))

    /**
     * The rigid frame's turn input bound and rate (`crates/store/src/field.rs`,
     * `CONTROL_DRIVE_TURN`), indices 0 = `turn_max`, the differential torque in stock counts at
     * full stick, and 1 = `turn_slew`, the counts per control tick that the turn may move.
     *
     * `turn_max` defaults to 0, which is DISABLED: the machine drives straight and a client
     * showing 0 is showing "steering is off", not "no turn". `turn_slew`'s 24 traverses a staged
     * 3000 in half a second at the 250 Hz control tick, because a step in differential torque is a
     * yaw impulse that reaches the pitch loop through the frame.
     *
     * An arm-time parameter rather than a live-tune one (the tune lane carries [CONTROL_SF_GAINS]
     * and [CONTROL_REFERENCE] only), so a write takes effect at the next arm ([ARM_TIME]). Both
     * indices are clamped by the firmware seam (`crates/control/src/drive.rs`, `DriveTurn`), since
     * the store validates type and nothing else: [TURN_MAX] and [TURN_SLEW] mirror that clamp.
     */
    val CONTROL_DRIVE_TURN = IndexedFieldDef(0x75, Type.I16, listOf(Value.I16(0), Value.I16(24)))

    /**
     * How the rider app drives the machine ([MachineDriveMode]; `specs/rider-ui.md` 3.2): written
     * with the same value to every motor controller of a machine. The firmware stores it and never
     * reads it, so it is in no boot seam and not in [ARM_TIME]; the app is its only consumer.
     */
    val MACHINE_DRIVE_MODE = FieldDef(0x78, Type.U8, Value.U8(0))

    /**
     * The state-feedback balance law's four gains (`crates/store/src/field.rs`, `CONTROL_SF_GAINS`;
     * `specs/state-feedback.md` sections 2 and 6), indices [Gains.GP] / [Gains.GR] / [Gains.GX] /
     * [Gains.GV]. Default zero, the inert board: a zero word in balance mode. ON the live tune lane
     * ([Gains]), so NOT in [ARM_TIME]; bounded by [CONTROL_GAIN_MAX].
     */
    val CONTROL_SF_GAINS = IndexedFieldDef(0x79, Type.I16, Gains.DEFAULT.map { Value.I16(it) })

    /**
     * The gains' inclusive upper bounds, indexed like [CONTROL_SF_GAINS]
     * (`crates/store/src/field.rs`, `CONTROL_GAIN_MAX`). Re-read at every arm ([ARM_TIME]).
     * Defaults [Gains.DEFAULT_MAX].
     */
    val CONTROL_GAIN_MAX = IndexedFieldDef(0x7A, Type.I16, Gains.DEFAULT_MAX.map { Value.I16(it) })

    /**
     * The stick reference (`crates/store/src/field.rs`, `CONTROL_REFERENCE`;
     * `specs/state-feedback.md` section 4), indices [Gains.SPEED_MAX] (sixteenths of a count at
     * full stick; 0 = the stick does nothing and the machine holds position) and [Gains.SPEED_SLEW]
     * (1/4096 count per tick). ON the live tune lane, against compiled bounds
     * ([Gains.REFERENCE_MINIMA], [Gains.REFERENCE_MAXIMA]).
     */
    val CONTROL_REFERENCE =
        IndexedFieldDef(0x7B, Type.I16, Gains.REFERENCE_DEFAULT.map { Value.I16(it) })

    /**
     * The law's position clamp, hall edges (`crates/store/src/field.rs`, `CONTROL_POS_CLAMP`;
     * `specs/state-feedback.md` section 4): a machine pushed further than this comes back exactly
     * this far. Default 45 (0.26 m on the rover); 0 disables the position term. An arm-time
     * parameter ([ARM_TIME]); the firmware seam (`control::sf::pos_clamp`) clamps it into
     * [POS_CLAMP], since the store validates type and nothing else.
     */
    val CONTROL_POS_CLAMP = FieldDef(0x7C, Type.I16, Value.I16(45))

    /** Every field above by its Rust handle name: the set the drift gate pins. */
    val ALL: Map<String, FieldDef> = mapOf(
        "NODE_ADDRESS" to NODE_ADDRESS,
        "LINK_SET" to LINK_SET,
        "NODE_CLASS" to NODE_CLASS,
        "DEVICE_NAME" to DEVICE_NAME,
        "MOTOR_CURRENT_LIMIT" to MOTOR_CURRENT_LIMIT,
        "MOTOR_METHOD" to MOTOR_METHOD,
        "CONTROL_MODE" to CONTROL_MODE,
        "CONTROL_RIDER_REQUIRED" to CONTROL_RIDER_REQUIRED,
        "CONTROL_BATTERY_FLOOR" to CONTROL_BATTERY_FLOOR,
        "CONTROL_FRAME" to CONTROL_FRAME,
        "CONTROL_SIDE" to CONTROL_SIDE,
        "BOARD_SELF_HOLD" to BOARD_SELF_HOLD,
        "BOARD_VBATT" to BOARD_VBATT,
        "BOARD_BUZZER" to BOARD_BUZZER,
        "LED_GREEN" to LED_GREEN,
        "LED_ORANGE" to LED_ORANGE,
        "LED_RED" to LED_RED,
        "PAD_A" to PAD_A,
        "PAD_B" to PAD_B,
        "IMU_SCL_PIN" to IMU_SCL_PIN,
        "IMU_SDA_PIN" to IMU_SDA_PIN,
        "MOTOR_HALL_A" to MOTOR_HALL_A,
        "MOTOR_HALL_B" to MOTOR_HALL_B,
        "MOTOR_HALL_C" to MOTOR_HALL_C,
        "MOTOR_GATE_HI_A" to MOTOR_GATE_HI_A,
        "MOTOR_GATE_HI_B" to MOTOR_GATE_HI_B,
        "MOTOR_GATE_HI_C" to MOTOR_GATE_HI_C,
        "MOTOR_GATE_LO_A" to MOTOR_GATE_LO_A,
        "MOTOR_GATE_LO_B" to MOTOR_GATE_LO_B,
        "MOTOR_GATE_LO_C" to MOTOR_GATE_LO_C,
        "BOARD_BUTTON" to BOARD_BUTTON,
        "MOTOR_PHASE_A" to MOTOR_PHASE_A,
        "MOTOR_PHASE_B" to MOTOR_PHASE_B,
        "IMU_MODEL" to IMU_MODEL,
        "IMU_GYRO_BIAS" to IMU_GYRO_BIAS,
        "MOTOR_DIRECTION" to MOTOR_DIRECTION,
        "MOTOR_ALIGN_OFFSET" to MOTOR_ALIGN_OFFSET,
        "MOTOR_DEAD_TIME" to MOTOR_DEAD_TIME,
        "MOTOR_CURRENT_SENSE" to MOTOR_CURRENT_SENSE,
        "MOTOR_CURRENT_CAL" to MOTOR_CURRENT_CAL,
        "MOTOR_POLE_PAIRS" to MOTOR_POLE_PAIRS,
        "MOTOR_NOISE_FLOOR" to MOTOR_NOISE_FLOOR,
        "MOTOR_ODOMETER" to MOTOR_ODOMETER,
        "WHEEL_DIAMETER_MM" to WHEEL_DIAMETER_MM,
        "IMU_AXIS_SIGN" to IMU_AXIS_SIGN,
        "IMU_AXIS_ROLE" to IMU_AXIS_ROLE,
        "ATTITUDE_LEVEL_TRIM" to ATTITUDE_LEVEL_TRIM,
        "BOARD_PACK_CELLS" to BOARD_PACK_CELLS,
        "BOARD_INPUT_POLARITY" to BOARD_INPUT_POLARITY,
        "MACHINE_DRIVE_MODE" to MACHINE_DRIVE_MODE,
        "CONTROL_POS_CLAMP" to CONTROL_POS_CLAMP,
    )

    /** Every indexed field above by its Rust handle name: the set the drift gate pins. */
    val INDEXED: Map<String, IndexedFieldDef> = mapOf(
        "BOARD_VBATT_CAL" to BOARD_VBATT_CAL,
        "CONTROL_DRIVE_TURN" to CONTROL_DRIVE_TURN,
        "CONTROL_SF_GAINS" to CONTROL_SF_GAINS,
        "CONTROL_GAIN_MAX" to CONTROL_GAIN_MAX,
        "CONTROL_REFERENCE" to CONTROL_REFERENCE,
    )

    /**
     * The field ids the firmware re-reads every time a board ARMS, so a value written while
     * disarmed runs from the next arm with no restart (`crates/firmware/src/arm.rs`, `ARM_ROW`;
     * `specs/integration.md`, "When a stored value takes effect: the arm-time re-read"). Every
     * other registered field is read once, during boot.
     *
     * **It is mirrored here, and pinned, because a client that states the class gets it wrong
     * otherwise.** The class is a firmware fact with no presence on the wire, so nothing a client
     * reads from a board reveals it, and it has gone stale twice in the rider app: `control.mode`
     * and then `motor.direction` / `motor.align_offset` moved into the row while the screen went
     * on telling the owner to power-cycle. `RustSourceDriftTest` compares this set against the row
     * itself, so a field that moves between the two classes fails there rather than at the bench.
     *
     * Ids and not keys: the row names `IMU_AXIS_SIGN` index by index, but no field has one index
     * in the row and another outside it.
     */
    val ARM_TIME: Set<Int> = setOf(
        MOTOR_CURRENT_LIMIT.id,
        MOTOR_METHOD.id,
        CONTROL_MODE.id,
        CONTROL_RIDER_REQUIRED.id,
        CONTROL_BATTERY_FLOOR.id,
        CONTROL_FRAME.id,
        CONTROL_SIDE.id,
        MOTOR_DIRECTION.id,
        MOTOR_ALIGN_OFFSET.id,
        MOTOR_CURRENT_CAL.id,
        IMU_AXIS_SIGN.id,
        IMU_AXIS_ROLE.id,
        CONTROL_GAIN_MAX.id,
        CONTROL_DRIVE_TURN.id,
        CONTROL_POS_CLAMP.id,
    )

    /**
     * The inclusive range the firmware clamps a staged [BOARD_VBATT_CAL] slope into at boot
     * (`orchestrator::battery::SLOPE_MIN` / `SLOPE_MAX`).
     */
    val VBATT_SLOPE = 10_000L..32_000L

    /** The same for the offset (`orchestrator::battery::OFFSET_MIN` / `OFFSET_MAX`). */
    val VBATT_OFFSET = -500L..500L

    /**
     * The inclusive band the firmware clamps a staged [MOTOR_NOISE_FLOOR] into at boot
     * (`firmware::motor::noise_floor_counts`: `NOISE_FLOOR_MIN` up to `MAX_LIMIT_COUNTS`, the
     * limit window's own ceiling).
     *
     * Below the bottom a board would claim a floor under the quietest rest reading any bench board
     * has produced, which is a typo rather than a measurement; at the top the limit window is one
     * value wide and has no range left. A plausibility band, like [VBATT_SLOPE]: the store validates
     * type only, so this is the seam a hand-poked word stops at, and a client reporting what a board
     * will DO applies the same clamp to what it read.
     */
    val NOISE_FLOOR = 1_000L..16_383L

    /**
     * The inclusive range a settings client offers for a staged [MOTOR_CURRENT_LIMIT], milliamps.
     *
     * **Both bounds are this client's own, and neither mirrors a firmware constant.** The firmware
     * bounds the limit in COUNTS, not milliamps (`firmware::motor::MIN_LIMIT_COUNTS` and
     * `MAX_LIMIT_COUNTS`), because what a milliamp buys in counts is per-board data
     * (`store::MOTOR_CURRENT_CAL`) and a milliamp bound is therefore a bound at one assumed scale.
     * A staged value outside the window the board's own scale admits is clamped into it at
     * bring-up, DOWN to that board's ceiling or UP to its noise floor, never refused, so this range
     * is a UI offer rather than a validity rule: 40 A is above the ceiling of every scale the
     * calibration seam admits (36.0 A at the measured 455 counts per amp, 20.0 A at the top of the
     * seam), and 1 A is this client refusing a sub-amp request it knows will be floored
     * (`specs/motor-integration.md`, "The current limit").
     */
    val CURRENT_LIMIT_MA = 1_000L..40_000L

    /**
     * The inclusive range a settings client offers for a staged [BOARD_PACK_CELLS], series cells,
     * plus the 0 that means not stated.
     *
     * This client's own, like [CURRENT_LIMIT_MA] and for the same reason: the firmware never reads
     * the field, so there is no constant to mirror. One cell is not a pack any board in this fleet
     * runs on, and sixteen is 67 V fully charged, above the rating of every part here, so the
     * offer is what a plausible machine states rather than what the byte holds.
     */
    val PACK_CELLS = 2L..16L

    /**
     * The inclusive range the firmware clamps a staged [CONTROL_POS_CLAMP] into, hall edges: 0 (the
     * position term off; a negative word reads as 0) up to `control::sf::POS_CLAMP_MAX`, five wheel
     * turns. Mirrored the way [VBATT_SLOPE] is: the store validates type only, so the seam is where
     * an out-of-range word stops.
     */
    val POS_CLAMP = 0L..450L

    /**
     * The inclusive range the firmware clamps a staged [CONTROL_DRIVE_TURN] index 0 (`turn_max`)
     * into, differential torque counts at full stick: 0 (steering off) up to
     * `base::TORQUE_FULL_SCALE`, the whole torque word one wheel can be asked for.
     */
    val TURN_MAX = 0L..28_500L

    /**
     * The same for index 1 (`turn_slew`), counts per control tick: `TURN_SLEW_MIN` up to
     * `TURN_SLEW_MAX`, which crosses the whole torque word in 29 ticks. The floor is 1 and not 0,
     * so a staged `turn_max` is never dead behind a rate of nothing.
     */
    val TURN_SLEW = 1L..1_000L

    /** [CONTROL_MODE]'s vocabulary (`control::ControlMode`); unknown bytes run as [THROTTLE]. */
    object ControlMode {
        const val THROTTLE = 0
        const val BALANCE = 1
    }

    /** [NODE_CLASS]'s vocabulary (`specs/l3.md`, "Node class"). */
    object NodeClass {
        const val MOTOR_CONTROLLER = 0x01
        const val BRIDGE = 0x02
    }

    /**
     * [MACHINE_DRIVE_MODE]'s vocabulary, written down in the field's own doc comment in
     * `crates/store/src/field.rs` (the firmware has no enum for it, since it never reads the field).
     */
    object MachineDriveMode {
        const val SINGLE = 0
        const val BOUND = 1
        const val DIFFERENTIAL = 2
        const val RIGID = 3
    }

    /** [CONTROL_FRAME]'s vocabulary (`control::Frame`); unknown bytes run as [PIVOTING]. */
    object ControlFrame {
        const val PIVOTING = 0
        const val RIGID = 1
    }

    /**
     * [CONTROL_SIDE]'s vocabulary (`side_is_right` in `crates/orchestrator/src/dispatch.rs`,
     * which reads `1` as the right
     * wheel and every other byte as [LEFT]).
     */
    object ControlSide {
        const val LEFT = 0
        const val RIGHT = 1
    }

    /**
     * [CONTROL_RIDER_REQUIRED]'s vocabulary (`ControlDispatch::new` in `crates/control/src/mode.rs`
     * decodes `!= 0`, so every
     * byte other than [NOT_REQUIRED] reads as required).
     */
    object RiderRequired {
        const val NOT_REQUIRED = 0
        const val REQUIRED = 1
    }

    /**
     * [MOTOR_METHOD]'s vocabulary (`commutation::CommutationMethod`); unknown bytes request
     * [SIX_STEP]. What the firmware RUNS is a separate question: `firmware::motor::running_method`
     * clamps every request to six-step today.
     */
    object MotorMethod {
        const val SIX_STEP = 0
        const val SINE = 1
        const val FOC = 2
    }

    /** [IMU_MODEL]'s vocabulary (`imu::model_from_index`): 0 = no IMU fitted. */
    object ImuModel {
        const val NONE = 0
        const val MPU6050 = 1
        const val CLONE_2E = 2
    }
}
