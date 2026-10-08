package com.hoverboard.remote.model

import androidx.annotation.StringRes
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.R

/** The Setup screen's groups (`specs/rider-ui.md` section 3.4). The pin block is not one of them. */
enum class SetupGroup(@StringRes val title: Int) {
    IDENTITY(R.string.setup_group_identity),
    DRIVE(R.string.setup_group_drive),
    IMU(R.string.setup_group_imu),
    CALIBRATION(R.string.setup_group_calibration),
}

/** One choice a byte-valued field offers: the byte, and what to call it. */
data class Choice(val value: Int, @StringRes val label: Int)

/**
 * How a field is edited: the render hint the embedded `FieldDef` deliberately does not carry
 * (`specs/rider-ui.md` section 7, "the registry-driven UI").
 */
sealed interface Editor {
    /** Displayed, never edited. */
    data object ReadOnly : Editor

    /** One of a fixed set of byte values. */
    data class Chips(val choices: List<Choice>) : Editor

    /** A number in an inclusive client-side range. */
    data class Range(val range: LongRange) : Editor

    /**
     * A number in an inclusive range that only a flow writes, never the row itself: the row is
     * displayed read-only and the value reaches the basket through the flow that owns it (the
     * orientation presets for `IMU_AXIS_ROLE`, which `specs/imu.md` keeps off any bare picker).
     */
    data class Flow(val range: LongRange) : Editor

    /**
     * No hint beyond the label: rendered from the value's own type tag (the schema-less fallback),
     * a text field for a string and a number in the type's natural range for an integer.
     */
    data object Generic : Editor
}

/**
 * One editable instance on the Setup screen: a registered field ([def], mirrored and drift-pinned in
 * `protocol-kotlin`), the [index] this row edits, and the render hint.
 *
 * @param note a standing caveat shown under the control, or null.
 * @param advanced whether the row sits behind ADVANCED (the raw editors the flows replace).
 */
data class SetupField(
    val def: FieldDef,
    val index: Int,
    @StringRes val label: Int,
    val group: SetupGroup,
    val editor: Editor,
    @StringRes val note: Int? = null,
    val advanced: Boolean = false,
) {
    val key: Key get() = def.key(index)

    /**
     * Whether this row takes [value]: the field's own type, inside the editor's range, and short
     * enough for one `CONFIG_WRITE` to carry and verify ([fitsOneWrite]).
     */
    fun accepts(value: Value): Boolean {
        if (value.kind() != def.type || !value.fitsOneWrite()) return false
        return when (editor) {
            Editor.ReadOnly -> false
            is Editor.Chips -> value.asLong()?.let { v -> editor.choices.any { it.value.toLong() == v } } == true
            is Editor.Range -> value.asLong()?.let { it in editor.range } == true
            is Editor.Flow -> value.asLong()?.let { it in editor.range } == true
            Editor.Generic -> value !is Value.Bytes
        }
    }

    /** Parse what the user typed into this row's value, or null when it is not one the row takes. */
    fun parse(text: String): Value? {
        val v = if (def.type == Type.Str) {
            Value.Str(text)
        } else {
            text.trim().toLongOrNull()?.let { n -> integer(def.type, n) }
        }
        return v?.takeIf { accepts(it) }
    }
}

/**
 * Whether one `CONFIG_WRITE` can carry this value and its echo verify it: at most
 * [CONFIG_VALUE_MAX] encoded bytes (UTF-8 bytes for a string, not characters). Every scalar fits.
 */
fun Value.fitsOneWrite(): Boolean = encode().size <= CONFIG_VALUE_MAX

/** The value as a Long when it is an integer type, else null. */
fun Value.asLong(): Long? = when (this) {
    is Value.U8 -> v.toLong()
    is Value.U16 -> v.toLong()
    is Value.U32 -> v
    is Value.U64 -> v
    is Value.I16 -> v.toLong()
    is Value.I32 -> v.toLong()
    is Value.I64 -> v
    is Value.Bool, is Value.Str, is Value.Bytes -> null
}

/** How a value reads on screen. */
fun Value.display(): String = when (this) {
    is Value.Str -> v
    is Value.Bool -> v.toString()
    is Value.Bytes -> v.joinToString(" ") { "%02x".format(it) }
    else -> asLong().toString()
}

/** The value of integer [type] holding [n], or null when [n] is outside the type's natural range. */
private fun integer(type: Type, n: Long): Value? {
    if (n !in naturalRange(type) ?: return null) return null
    return when (type) {
        Type.U8 -> Value.U8(n.toInt())
        Type.U16 -> Value.U16(n.toInt())
        Type.U32 -> Value.U32(n)
        Type.U64 -> Value.U64(n)
        Type.I16 -> Value.I16(n.toInt())
        Type.I32 -> Value.I32(n.toInt())
        Type.I64 -> Value.I64(n)
        Type.Bool, Type.Str, Type.Blob -> null
    }
}

/** The values an integer [type] can hold (U64 as far as a Long reaches), or null for a non-integer. */
private fun naturalRange(type: Type): LongRange? = when (type) {
    Type.U8 -> 0..U8_MAX
    Type.U16 -> 0..U16_MAX
    Type.U32 -> 0..U32_MAX
    Type.U64 -> 0..Long.MAX_VALUE
    Type.I16 -> Short.MIN_VALUE.toLong()..Short.MAX_VALUE.toLong()
    Type.I32 -> Int.MIN_VALUE.toLong()..Int.MAX_VALUE.toLong()
    Type.I64 -> Long.MIN_VALUE..Long.MAX_VALUE
    Type.Bool, Type.Str, Type.Blob -> null
}

private const val U8_MAX = 0xFFL
private const val U16_MAX = 0xFFFFL
private const val U32_MAX = 0xFFFF_FFFFL

/**
 * The Setup screen's rows (`specs/rider-ui.md` section 3.4, minus the pin block of 3.5): the hint
 * table over the drift-pinned field mirror. Motor rows edit motor index 0 only; index 1 is
 * unconsumed by the firmware and hidden, as are `SOME_BLOB` and the test fields.
 */
object SetupFields {
    private val motorMethods = listOf(
        Choice(Fields.MotorMethod.SIX_STEP, R.string.setup_choice_six_step),
        Choice(Fields.MotorMethod.SINE, R.string.setup_choice_sine),
        Choice(Fields.MotorMethod.FOC, R.string.setup_choice_foc),
    )

    val DEVICE_NAME = SetupField(
        Fields.DEVICE_NAME, 0, R.string.setup_field_device_name, SetupGroup.IDENTITY, Editor.Generic,
        note = R.string.setup_note_device_name,
    )
    val NODE_ADDRESS = SetupField(
        Fields.NODE_ADDRESS, 0, R.string.setup_field_node_address, SetupGroup.IDENTITY, Editor.ReadOnly,
        note = R.string.setup_note_walk_owned,
    )
    val LINK_SET = SetupField(
        Fields.LINK_SET, 0, R.string.setup_field_link_set, SetupGroup.IDENTITY, Editor.ReadOnly,
        note = R.string.setup_note_walk_owned,
    )
    val CONTROL_MODE = SetupField(
        Fields.CONTROL_MODE, 0, R.string.setup_field_control_mode, SetupGroup.DRIVE,
        Editor.Chips(
            listOf(
                Choice(Fields.ControlMode.THROTTLE, R.string.setup_choice_throttle),
                Choice(Fields.ControlMode.BALANCE, R.string.setup_choice_balance),
            ),
        ),
    )
    val RIDER_REQUIRED = SetupField(
        Fields.CONTROL_RIDER_REQUIRED, 0, R.string.setup_field_rider_required, SetupGroup.DRIVE,
        Editor.Chips(
            listOf(
                Choice(Fields.RiderRequired.REQUIRED, R.string.setup_choice_required),
                Choice(Fields.RiderRequired.NOT_REQUIRED, R.string.setup_choice_not_required),
            ),
        ),
        note = R.string.setup_note_rider_required,
    )
    val MOTOR_METHOD = SetupField(
        Fields.MOTOR_METHOD, 0, R.string.setup_field_motor_method, SetupGroup.DRIVE,
        Editor.Chips(motorMethods), note = R.string.setup_note_motor_method,
    )
    val MOTOR_DIRECTION = SetupField(
        Fields.MOTOR_DIRECTION, 0, R.string.setup_field_motor_direction, SetupGroup.DRIVE,
        Editor.Chips(
            listOf(
                Choice(DIRECTION_FORWARD, R.string.setup_choice_forward),
                Choice(DIRECTION_REVERSE, R.string.setup_choice_reverse),
            ),
        ),
    )
    val MOTOR_ALIGN_OFFSET = SetupField(
        Fields.MOTOR_ALIGN_OFFSET, 0, R.string.setup_field_align_offset, SetupGroup.DRIVE,
        Editor.Range(0L..ALIGN_OFFSET_MAX),
    )
    val MOTOR_DEAD_TIME = SetupField(
        Fields.MOTOR_DEAD_TIME, 0, R.string.setup_field_dead_time, SetupGroup.DRIVE,
        Editor.Range(0L..U8_MAX), note = R.string.setup_note_dead_time,
    )
    val MOTOR_CURRENT_LIMIT = SetupField(
        Fields.MOTOR_CURRENT_LIMIT, 0, R.string.setup_field_current_limit, SetupGroup.DRIVE,
        Editor.Range(Fields.CURRENT_LIMIT_MA), note = R.string.setup_note_current_limit,
    )
    val IMU_MODEL = SetupField(
        Fields.IMU_MODEL, 0, R.string.setup_field_imu_model, SetupGroup.IMU,
        Editor.Chips(
            listOf(
                Choice(Fields.ImuModel.NONE, R.string.setup_choice_imu_none),
                Choice(Fields.ImuModel.MPU6050, R.string.setup_choice_imu_mpu6050),
                Choice(Fields.ImuModel.CLONE_2E, R.string.setup_choice_imu_clone),
            ),
        ),
    )

    /** `ATTITUDE_LEVEL_TRIM` index 0 (pitch) and 1 (roll); the "set level" flow writes both. */
    val LEVEL_TRIM: List<SetupField> = listOf(
        R.string.setup_field_trim_pitch,
        R.string.setup_field_trim_roll,
    ).mapIndexed { i, label ->
        SetupField(Fields.ATTITUDE_LEVEL_TRIM, i, label, SetupGroup.IMU, Editor.Generic, advanced = true)
    }

    /** `IMU_GYRO_BIAS` indices 0/1/2 = x/y/z, raw counts. */
    val GYRO_BIAS: List<SetupField> = listOf(
        R.string.setup_field_gyro_bias_x,
        R.string.setup_field_gyro_bias_y,
        R.string.setup_field_gyro_bias_z,
    ).mapIndexed { i, label ->
        SetupField(Fields.IMU_GYRO_BIAS, i, label, SetupGroup.IMU, Editor.Generic, advanced = true)
    }

    /**
     * `IMU_AXIS_SIGN` indices 0..5 = `ax, ay, az, gx, gy, gz`, each -1, 0 (unset) or +1. The raw
     * editors behind ADVANCED; the orientation flow stages all six at once, and every Apply that
     * touches one is checked as a whole frame, with the roles, first ([Orientation.check]).
     */
    val AXIS_SIGN: List<SetupField> = listOf(
        R.string.setup_field_sign_ax,
        R.string.setup_field_sign_ay,
        R.string.setup_field_sign_az,
        R.string.setup_field_sign_gx,
        R.string.setup_field_sign_gy,
        R.string.setup_field_sign_gz,
    ).also { check(it.size == Orientation.INDICES) }.mapIndexed { i, label ->
        SetupField(
            Fields.IMU_AXIS_SIGN, i, label, SetupGroup.IMU, Editor.Range(-1L..1L),
            note = if (i == 0) R.string.setup_note_axis_sign else null, advanced = true,
        )
    }

    /**
     * `IMU_AXIS_ROLE` indices 0 = UP, 1 = PITCH_RATE, each 1..3 = chip X/Y/Z or 0 (unset). Read with
     * the rest so the frame is judged by the roles the board holds; written only by the orientation
     * flow, as half of one frame with [AXIS_SIGN] (`specs/imu.md`: never a bare picker).
     */
    val AXIS_ROLE: List<SetupField> = listOf(
        R.string.setup_field_role_up,
        R.string.setup_field_role_pitch,
    ).mapIndexed { i, label ->
        SetupField(
            Fields.IMU_AXIS_ROLE, i, label, SetupGroup.IMU, Editor.Flow(0L..ROLE_MAX),
            note = if (i == 0) R.string.setup_note_axis_role else null, advanced = true,
        )
    }

    /**
     * `BOARD_VBATT_CAL` index 0 (slope) and 1 (offset), raw, behind ADVANCED, each offered over the
     * range the firmware clamps it into at boot.
     */
    val VBATT_CAL: List<SetupField> = listOf(
        Triple(R.string.setup_field_vbatt_slope, Fields.VBATT_SLOPE, R.string.setup_note_vbatt_cal),
        Triple(R.string.setup_field_vbatt_offset, Fields.VBATT_OFFSET, null),
    ).mapIndexed { i, (label, range, note) ->
        SetupField(
            Fields.BOARD_VBATT_CAL.at(i), i, label, SetupGroup.CALIBRATION, Editor.Range(range),
            note = note, advanced = true,
        )
    }

    /** The rows that together are one IMU frame: the six signs and the two roles. */
    val FRAME: List<SetupField> = AXIS_SIGN + AXIS_ROLE

    /** Every row, in screen order within each group. */
    val ALL: List<SetupField> = listOf(
        DEVICE_NAME, NODE_ADDRESS, LINK_SET,
        CONTROL_MODE, RIDER_REQUIRED, MOTOR_METHOD, MOTOR_DIRECTION, MOTOR_ALIGN_OFFSET, MOTOR_DEAD_TIME,
        MOTOR_CURRENT_LIMIT,
        IMU_MODEL,
    ) + LEVEL_TRIM + GYRO_BIAS + AXIS_SIGN + AXIS_ROLE + VBATT_CAL

    private val byKey: Map<Key, SetupField> = ALL.associateBy { it.key }

    /** The row editing [key], or null when the screen has none. */
    fun forKey(key: Key): SetupField? = byKey[key]

    /** `MOTOR_DIRECTION`'s bytes: 0 = forward, and the firmware reads any nonzero byte as reverse. */
    const val DIRECTION_FORWARD = 0
    const val DIRECTION_REVERSE = 1

    /** The six-step align offset is a sector rotation, 0..5 (`commutation::sixstep`, taken mod 6). */
    const val ALIGN_OFFSET_MAX = 5L

    /** The largest axis role, chip Z (`1 = X`, `2 = Y`, `3 = Z`). */
    const val ROLE_MAX = 3L
}

/**
 * One orientation preset (`specs/rider-ui.md` 3.4): a whole frame, the axis roles AND the six signs,
 * staged together, with where it was derived from shown beside it so a stale derivation is visible.
 *
 * @param roles `[UP, PITCH_RATE]`, the `IMU_AXIS_ROLE` values.
 * @param signs the six `IMU_AXIS_SIGN` values.
 * @param source where the frame comes from, shown beside the preset.
 */
data class OrientationPreset(
    @StringRes val label: Int,
    val roles: List<Int>,
    val signs: List<Int>,
    @StringRes val source: Int,
) {
    init {
        require(Orientation.check(signs, roles) == null) { "a preset must be a proper rotation" }
    }
}

/**
 * The orientation presets. One today: the stock flat mount. The rover's on-edge presets are not
 * here because the specs do not give their signs; `specs/imu.md` says to derive them from the
 * rover's mechanical model at staging time, and the app does not carry that derivation yet.
 */
object OrientationPresets {
    /** The stock board's flat mount: the compiled reference map under the compiled roles. */
    val STOCK_FLAT = OrientationPreset(
        label = R.string.setup_preset_stock_flat,
        roles = Orientation.DEFAULT_ROLES,
        signs = Orientation.REFERENCE,
        source = R.string.setup_preset_stock_flat_source,
    )

    val ALL: List<OrientationPreset> = listOf(STOCK_FLAT)
}
