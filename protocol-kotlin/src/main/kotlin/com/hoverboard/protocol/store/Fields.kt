package com.hoverboard.protocol.store

/**
 * One registered store field as the firmware declares it: its permanent `field_id`, its storage
 * [type] and its typed [default] (what a board that never staged the field reads). A mirror of one
 * `Field<T>` / `StrField` handle in `crates/store/src/field.rs`, which is the single source of truth
 * for all three facts; `RustSourceDriftTest` pins every entry of [Fields.ALL] against it.
 *
 * How many indices a field has is NOT here: the Rust handle does not carry it (an indexed field is a
 * plain `Field<T>` whose index range lives in its doc comment and its consumer), so a client that
 * names an index takes it from the field's own documentation.
 */
data class FieldDef(val id: Int, val type: Type, val default: Value) {
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
 * Not every registered field is here. The board-layout pin fields (0x40-0x55) belong to the layout
 * editor (`specs/rider-ui.md` section 3.5), the gain families to [Gains], and `SOME_BLOB` and the
 * test fields to nobody; a field is added here when a client exercises it.
 */
object Fields {
    /** The board's persistent L3 node address (walk-owned; displayed, never edited). */
    val NODE_ADDRESS = FieldDef(0x01, Type.U8, Value.U8(0))

    /** The L3 link-set bitmask (walk-owned; displayed, never edited). */
    val LINK_SET = FieldDef(0x02, Type.U8, Value.U8(0))

    /** The advertised device name. */
    val DEVICE_NAME = FieldDef(0x10, Type.Str, Value.Str("Hoverboard"))

    /** The motor current limit, milliamps. The firmware clamps it at bring-up ([CURRENT_LIMIT_MA]). */
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
     * word is below it, never a disengage (`specs/sensing-and-safety.md`). `<= 0` = no floor; no
     * clamp beyond the type.
     */
    val CONTROL_BATTERY_FLOOR = FieldDef(0x24, Type.I16, Value.I16(2400))

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

    /** Every field above by its Rust handle name: the set the drift gate pins. */
    val ALL: Map<String, FieldDef> = mapOf(
        "NODE_ADDRESS" to NODE_ADDRESS,
        "LINK_SET" to LINK_SET,
        "DEVICE_NAME" to DEVICE_NAME,
        "MOTOR_CURRENT_LIMIT" to MOTOR_CURRENT_LIMIT,
        "MOTOR_METHOD" to MOTOR_METHOD,
        "CONTROL_MODE" to CONTROL_MODE,
        "CONTROL_RIDER_REQUIRED" to CONTROL_RIDER_REQUIRED,
        "CONTROL_BATTERY_FLOOR" to CONTROL_BATTERY_FLOOR,
        "IMU_MODEL" to IMU_MODEL,
        "IMU_GYRO_BIAS" to IMU_GYRO_BIAS,
        "MOTOR_DIRECTION" to MOTOR_DIRECTION,
        "MOTOR_ALIGN_OFFSET" to MOTOR_ALIGN_OFFSET,
        "MOTOR_DEAD_TIME" to MOTOR_DEAD_TIME,
        "IMU_AXIS_SIGN" to IMU_AXIS_SIGN,
        "IMU_AXIS_ROLE" to IMU_AXIS_ROLE,
        "ATTITUDE_LEVEL_TRIM" to ATTITUDE_LEVEL_TRIM,
    )

    /** Every indexed field above by its Rust handle name: the set the drift gate pins. */
    val INDEXED: Map<String, IndexedFieldDef> = mapOf(
        "BOARD_VBATT_CAL" to BOARD_VBATT_CAL,
    )

    /**
     * The inclusive range the firmware clamps a staged [BOARD_VBATT_CAL] slope into at boot
     * (`orchestrator::battery::SLOPE_MIN` / `SLOPE_MAX`).
     */
    val VBATT_SLOPE = 10_000L..32_000L

    /** The same for the offset (`orchestrator::battery::OFFSET_MIN` / `OFFSET_MAX`). */
    val VBATT_OFFSET = -500L..500L

    /**
     * The inclusive range the firmware clamps a staged [MOTOR_CURRENT_LIMIT] into at bring-up,
     * milliamps (`firmware::motor::CURRENT_LIMIT_FLOOR_MA` / `CURRENT_LIMIT_CEILING_MA`).
     */
    val CURRENT_LIMIT_MA = 1_000L..40_000L

    /** [CONTROL_MODE]'s vocabulary (`control::ControlMode`); unknown bytes run as [THROTTLE]. */
    object ControlMode {
        const val THROTTLE = 0
        const val BALANCE = 1
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
