package com.hoverboard.remote.model

import androidx.annotation.StringRes
import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
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
    TUNING(R.string.setup_group_tuning),
}

/**
 * When a value the board already stores starts being used (`specs/integration.md`, "When a stored
 * value takes effect: the arm-time re-read"), which is what a Setup row has to tell the operator.
 *
 * Two cases, where that decision names three groups. The arm path re-reads exactly one row of
 * fields (`crates/firmware/src/arm.rs`, `read_arm_values`); everything else is derived once in the
 * boot sequence, whether it configures a peripheral (the pins, the timer, `motor.dead_time`) or is
 * a value the boot path simply owns (`attitude.level_trim`, `board.vbatt_cal`, `imu.gyro_bias`,
 * `imu.model`, `link.link_set`). Those two groups differ in why they wait for a boot, not in what
 * the operator has to do about it, so the screen asks one question and gets two answers.
 *
 * Which of those two a row is in is NOT declared here: [SetupField.takesEffect] reads it off
 * [Fields.ARM_TIME], the drift-pinned mirror of the firmware's own arm row. A list retyped beside
 * the rows went stale twice. [APP_ONLY] is the exception and says why.
 */
enum class TakesEffect {
    /** Re-read in the arm path, so a value written while disarmed runs from the next ARM. */
    NEXT_ARM,

    /** Derived once at boot, so a value written now runs from the board's next RESTART. */
    RESTART,

    /**
     * Nothing on the board reads it, so there is no board-side event to wait for: the app uses it
     * as soon as the write is verified. The store carries facts about the machine as well as
     * settings the firmware acts on, and `board.pack_cells` is the first of the former with a row.
     */
    APP_ONLY,
}

/**
 * When a value written to the field [fieldId] starts being used: re-read in the arm path, so it
 * runs from the next ARM, or derived once at boot, so it runs from the next RESTART.
 *
 * Read off [Fields.ARM_TIME], the mirror `RustSourceDriftTest` pins against the firmware's own arm
 * row, rather than declared per row: this is a firmware fact and not a screen decision, and a
 * hand-set flag went stale twice, telling the owner to power-cycle for a value the board had
 * already picked up (`specs/integration.md`, "When a stored value takes effect: the arm-time
 * re-read").
 *
 * Two screens state it, so it has one home: every Setup row ([SetupField.takesEffect]) and the
 * Tune screen's four drive bounds, which are written to flash and have the same question to answer.
 */
fun takesEffectOf(fieldId: Int): TakesEffect =
    if (fieldId in Fields.ARM_TIME) TakesEffect.NEXT_ARM else TakesEffect.RESTART

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

    /**
     * A number in an inclusive client-side range, plus [unset]: the one value outside that range
     * the field also takes, where its unset state is not the bottom of its valid one
     * (`motor.dead_time`, whose 0 means no gates while its configured values start at a safety
     * floor).
     */
    data class Range(val range: LongRange, val unset: Long? = null) : Editor {
        /** Whether this editor takes [v]. */
        fun accepts(v: Long): Boolean = v in range || v == unset
    }

    /**
     * A number in an inclusive range that only a flow writes, never the row itself: the row is
     * displayed read-only and the value reaches the basket through the flow that owns it (the
     * orientation pose picker for `IMU_AXIS_ROLE`, kept off any bare picker by `specs/imu.md`,
     * the board-config fields).
     */
    data class Flow(val range: LongRange) : Editor

    /**
     * No hint beyond the label: rendered from the value's own type tag (the schema-less fallback),
     * a text field for a string and a number in the type's natural range for an integer.
     */
    data object Generic : Editor
}

/**
 * One editable instance of a registered field ([def], mirrored and drift-pinned in
 * `protocol-kotlin`): the [index] it edits and the render hint.
 *
 * The Setup screen's rows are these, and so are the Tune screen's four drive bounds
 * (`specs/rider-ui.md` 3.3b), which are the same thing on another screen: a field, a range, a
 * label, and the take-effect class read off the firmware's own arm row.
 *
 * @param group which Setup group the row sits in, or null for a row another screen owns and Setup
 *   therefore does not list (the Tune screen's drive bounds).
 * @param note a standing caveat shown under the control, or null.
 * @param noteArgs the note's format arguments, where it states a number the model owns.
 * @param advanced whether the row sits behind ADVANCED (the raw editors the flows replace).
 * @param showDefault whether the row shows its field's default beside the stored value (a bound
 *   whose stock value is the reference point an edit departs from).
 * @param appOnly whether the firmware reads this field at all. Declared per row and not read off a
 *   mirror, unlike the arm-or-boot split below, because "no consumer on the board" is not a class
 *   the firmware's own tables name: the field's Rust documentation states it and nothing else can.
 */
data class SetupField(
    val def: FieldDef,
    val index: Int,
    @StringRes val label: Int,
    val group: SetupGroup?,
    val editor: Editor,
    @StringRes val note: Int? = null,
    val noteArgs: List<Any> = emptyList(),
    val advanced: Boolean = false,
    val showDefault: Boolean = false,
    val appOnly: Boolean = false,
) {
    val key: Key get() = def.key(index)

    /**
     * When the board starts using a value written here, which the row states ([TakesEffect]), or
     * null for a row nothing writes ([Editor.ReadOnly]) and which therefore has no edit to place in
     * time. [takesEffectOf] is where it is decided, so the screen and the firmware cannot disagree
     * about a field.
     */
    val takesEffect: TakesEffect?
        get() = when {
            editor == Editor.ReadOnly -> null
            appOnly -> TakesEffect.APP_ONLY
            else -> takesEffectOf(def.id)
        }

    /**
     * Whether this row takes [value]: the field's own type, inside the editor's range, and short
     * enough for one `CONFIG_WRITE` to carry and verify ([fitsOneWrite]).
     */
    fun accepts(value: Value): Boolean {
        if (value.kind() != def.type || !value.fitsOneWrite()) return false
        return when (editor) {
            Editor.ReadOnly -> false
            is Editor.Chips -> value.asLong()?.let { v -> editor.choices.any { it.value.toLong() == v } } == true
            is Editor.Range -> value.asLong()?.let { editor.accepts(it) } == true
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
 * The Setup screen's rows (`specs/rider-ui.md` sections 3.4 and 3.6a, minus the pin block of 3.5):
 * the hint table over the drift-pinned field mirror. Motor rows edit motor index 0 only; index 1 is
 * unconsumed by the firmware and hidden, as are `SOME_BLOB` and the test fields.
 */
object SetupFields {
    /**
     * The control tick, passes per second (`Gains.Ramp.PASS_HZ`, the mirror of the firmware's
     * `TICK_HZ`): the rate the two slew rows are stated per, so a caption can say what a per-tick
     * number does in a second without retyping the rate.
     */
    val CONTROL_HZ: Int = Gains.Ramp.PASS_HZ

    private val motorMethods = listOf(
        Choice(Fields.MotorMethod.SIX_STEP, R.string.setup_choice_six_step),
        Choice(Fields.MotorMethod.SINE, R.string.setup_choice_sine),
        Choice(Fields.MotorMethod.FOC, R.string.setup_choice_foc),
    )

    val DEVICE_NAME = SetupField(
        Fields.DEVICE_NAME, 0, R.string.setup_field_device_name, SetupGroup.IDENTITY, Editor.Generic,
        note = R.string.setup_note_device_name,
    )
    /**
     * `node.class` (`specs/l3.md`, "Node class"): what the node is. Read-only on every node, a
     * bridge's included: the value is the node's own firmware's.
     */
    val NODE_CLASS = SetupField(
        Fields.NODE_CLASS, 0, R.string.setup_field_node_class, SetupGroup.IDENTITY, Editor.ReadOnly,
        note = R.string.setup_note_node_class,
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

    /**
     * `BOARD_PACK_CELLS`, the series cell count of the pack this machine runs: what turns a pack
     * voltage into a state of charge ([batteryReading]), and the one place the pack is stated.
     * Offered over [Fields.PACK_CELLS], with 0 as the unset value the field defaults to.
     *
     * Read on BOTH boards with the ride facts, like [MACHINE], because the Ride screen's gauge
     * needs it whether or not Setup is ever opened, and written to both: one pack feeds both
     * boards.
     */
    val PACK_CELLS = SetupField(
        Fields.BOARD_PACK_CELLS, 0, R.string.setup_field_pack_cells, SetupGroup.DRIVE,
        Editor.Range(Fields.PACK_CELLS, unset = 0L), note = R.string.setup_note_pack_cells,
        appOnly = true,
    )

    /**
     * `CONTROL_BATTERY_FLOOR`, centivolts: any i16 (the firmware takes it as written; `<= 0` = no
     * floor), so the generic editor over the type's natural range.
     */
    val BATTERY_FLOOR = SetupField(
        Fields.CONTROL_BATTERY_FLOOR, 0, R.string.setup_field_battery_floor, SetupGroup.DRIVE,
        Editor.Generic, note = R.string.setup_note_battery_floor,
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
    /**
     * `motor.dead_time`, the SECOND editor of the field the layout screen also offers. The board
     * refuses a configured gate group below [DEAD_TIME_MIN_DTG] (shoot-through), and 0 only with
     * the gates unset, so this row offers exactly what the validator takes rather than the whole
     * byte.
     */
    val MOTOR_DEAD_TIME = SetupField(
        Fields.MOTOR_DEAD_TIME, 0, R.string.setup_field_dead_time, SetupGroup.DRIVE,
        Editor.Range(DEAD_TIME_MIN_DTG.toLong()..U8_MAX, unset = 0L),
        note = R.string.setup_note_dead_time, noteArgs = listOf(DEAD_TIME_MIN_DTG),
    )
    val MOTOR_CURRENT_LIMIT = SetupField(
        Fields.MOTOR_CURRENT_LIMIT, 0, R.string.setup_field_current_limit, SetupGroup.DRIVE,
        Editor.Range(Fields.CURRENT_LIMIT_MA), note = R.string.setup_note_current_limit,
    )
    /**
     * `control.frame`, the machine's frame (`specs/rider-ui.md` 3.7a). The Machine section's chips
     * are the primary control, because the value belongs to the machine and goes to both boards;
     * this is the raw row behind ADVANCED, the escape hatch for a bench with one board powered.
     */
    val CONTROL_FRAME = SetupField(
        Fields.CONTROL_FRAME, 0, R.string.setup_field_control_frame, SetupGroup.DRIVE,
        Editor.Chips(
            listOf(
                Choice(Fields.ControlFrame.PIVOTING, R.string.setup_choice_frame_pivoting),
                Choice(Fields.ControlFrame.RIGID, R.string.setup_choice_frame_rigid),
            ),
        ),
        note = R.string.setup_note_control_frame, advanced = true,
    )

    /**
     * `machine.drive_mode`, how the app drives this machine (`specs/rider-ui.md` 3.2). Nothing on the
     * board reads it ([SetupField.appOnly]). The Machine section's chips are the primary control,
     * because the value belongs to the machine and goes to every motor controller; this is the raw
     * row behind ADVANCED, as for [CONTROL_FRAME].
     */
    val DRIVE_MODE = SetupField(
        Fields.MACHINE_DRIVE_MODE, 0, R.string.setup_field_drive_mode, SetupGroup.DRIVE,
        Editor.Chips(DriveMode.entries.map { Choice(it.byte, driveModeLabel(it)) }),
        note = R.string.setup_note_drive_mode, advanced = true, appOnly = true,
    )

    /**
     * `control.side`, which wheel of a rigid machine this board drives (`specs/rider-ui.md` 3.7a).
     * The raw row, like [CONTROL_FRAME]: the Machine section's Right wheel picker is the control
     * that cannot express a pair with two right wheels, and this one can.
     */
    val CONTROL_SIDE = SetupField(
        Fields.CONTROL_SIDE, 0, R.string.setup_field_control_side, SetupGroup.DRIVE,
        Editor.Chips(
            listOf(
                Choice(Fields.ControlSide.LEFT, R.string.setup_choice_side_left),
                Choice(Fields.ControlSide.RIGHT, R.string.setup_choice_side_right),
            ),
        ),
        note = R.string.setup_note_control_side, advanced = true,
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
        SetupField(
            Fields.ATTITUDE_LEVEL_TRIM, i, label, SetupGroup.IMU, Editor.Generic, advanced = true,
        )
    }

    /** `IMU_GYRO_BIAS` indices 0/1/2 = x/y/z, raw counts. */
    val GYRO_BIAS: List<SetupField> = listOf(
        R.string.setup_field_gyro_bias_x,
        R.string.setup_field_gyro_bias_y,
        R.string.setup_field_gyro_bias_z,
    ).mapIndexed { i, label ->
        SetupField(
            Fields.IMU_GYRO_BIAS, i, label, SetupGroup.IMU, Editor.Generic, advanced = true,
        )
    }

    /**
     * `IMU_AXIS_SIGN` indices 0..5 = `ax, ay, az, gx, gy, gz`, each -1, 0 (unset) or +1. Displayed
     * behind ADVANCED, written only by the orientation pose picker, which writes all six with the
     * roles as one checked frame ([Orientation.check]).
     *
     * [Editor.Flow] rather than a per-index editor because the screen writes on edit
     * (`specs/rider-ui.md` 3.6a) and there is no basket to compose a frame in: flipping ONE sign of
     * a rotation always gives a mirrored frame, which the board refuses, so a bare sign editor could
     * only ever store something the board will not run. The frame is one object with an invariant
     * across its fields, which is the layout editor's shape rather than the rest of Setup's.
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
            Fields.IMU_AXIS_SIGN, i, label, SetupGroup.IMU, Editor.Flow(-1L..1L),
            note = if (i == 0) R.string.setup_note_axis_sign else null, advanced = true,
        )
    }

    /**
     * `IMU_AXIS_ROLE` indices 0 = UP, 1 = PITCH_RATE, each 1..3 = chip X/Y/Z or 0 (unset). Read with
     * the rest so the frame is judged by the roles the board holds; written only by the orientation
     * flow, as half of one frame with [AXIS_SIGN] (`specs/imu.md`: never a bare picker). The two
     * indices are separate records, so the pair is written back to back and can hold a duplicate in
     * between (`specs/rider-ui.md` 3.6a).
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

    /**
     * `CONTROL_GAIN_MAX` indices gp / gr / gx / gv: the inclusive upper bound each state-feedback
     * gain accepts (`specs/state-feedback.md` section 6), behind ADVANCED with the default shown.
     * Offered over `0..i16::MAX`: the firmware reads a negative maximum as 0, so a negative one says
     * nothing a 0 does not.
     */
    val GAIN_MAX: List<SetupField> = listOf(
        R.string.setup_field_gain_max_gp,
        R.string.setup_field_gain_max_gr,
        R.string.setup_field_gain_max_gx,
        R.string.setup_field_gain_max_gv,
    ).also { check(it.size == Gains.COUNT) }.mapIndexed { i, label ->
        SetupField(
            Fields.CONTROL_GAIN_MAX.at(i), i, label, SetupGroup.TUNING,
            Editor.Range(Gains.MIN.toLong()..Short.MAX_VALUE.toLong()),
            note = if (i == 0) R.string.setup_note_gain_max else null, advanced = true, showDefault = true,
        )
    }

    /**
     * `CONTROL_DRIVE_TURN` index 0 (`turn_max`): how hard the two wheels are driven apart with the
     * stick at full travel, in the counts the wheels take torque in. 0 drives straight, and only a
     * machine set to one rigid frame steers at all.
     *
     * One of [DRIVE_BOUNDS], so it has no Setup group: the Tune screen owns these rows
     * (`specs/rider-ui.md` 3.3b). The range is the clamp the firmware applies as it reads the word,
     * so a value the board would silently narrow cannot be sent, and the default is shown because
     * it is the shipped state the first edit departs from.
     */
    val TURN_MAX = SetupField(
        Fields.CONTROL_DRIVE_TURN.at(0), 0, R.string.tune_bound_turn_max, null,
        Editor.Range(Fields.TURN_MAX), note = R.string.tune_bound_turn_max_note, showDefault = true,
    )

    /** `CONTROL_DRIVE_TURN` index 1 (`turn_slew`): counts the turn may move per control tick. */
    val TURN_SLEW = SetupField(
        Fields.CONTROL_DRIVE_TURN.at(1), 1, R.string.tune_bound_turn_slew, null,
        Editor.Range(Fields.TURN_SLEW), note = R.string.tune_bound_turn_slew_note,
        noteArgs = listOf(CONTROL_HZ), showDefault = true,
    )

    /**
     * `CONTROL_POS_CLAMP` (`pos_clamp`): how far, in hall edges, the balance law's position term
     * pulls the machine back (`specs/state-feedback.md` section 4). Pushed further than this, the
     * machine returns exactly this far; 0 turns the position term off. [TURN_MAX]'s kind of row: a
     * stored value read at the next arm, over the range the firmware clamps it into.
     */
    val POS_CLAMP = SetupField(
        Fields.CONTROL_POS_CLAMP, 0, R.string.tune_bound_pos_clamp, null,
        Editor.Range(Fields.POS_CLAMP), note = R.string.tune_bound_pos_clamp_note, showDefault = true,
    )

    /** The rows that together are one IMU frame: the six signs and the two roles. */
    val FRAME: List<SetupField> = AXIS_SIGN + AXIS_ROLE

    /**
     * The rows the Machine section owns (`specs/rider-ui.md` 3.2 and 3.7a). Read on every motor
     * controller once per attached session with the ride facts ([MOTOR_FACTS]), because the arm
     * control and the drive state what they come to; their raw rows above are the per-board escape
     * hatch.
     */
    val MACHINE: List<SetupField> = listOf(DRIVE_MODE, CONTROL_FRAME, CONTROL_SIDE)

    /**
     * The rows the attach-time ride-facts read takes from EVERY motor controller: [MACHINE]'s three,
     * the pack, and the rider requirement. Each is a fact a screen states before Setup has ever been
     * opened, so every copy is read and a machine that disagrees is visible.
     */
    val MOTOR_FACTS: List<SetupField> = MACHINE + PACK_CELLS + RIDER_REQUIRED

    /**
     * The rows the attach-time read takes from EVERY node, before anything else: what each node is
     * and what it is called (`specs/rider-ui.md` section 2). Nothing is read from a node as a motor
     * controller until its class says it is one.
     */
    val IDENTITY: List<SetupField> = listOf(NODE_CLASS, DEVICE_NAME)

    /**
     * A bridge's rows (`specs/hoverboard-bridge.md`, "The bridge as a node"): its class, its name and its link
     * set. Also what a node whose class is not yet read shows, since those keys are every node's.
     */
    val BRIDGE: List<SetupField> = listOf(DEVICE_NAME, NODE_CLASS, LINK_SET)

    /**
     * The three drive bounds, in screen order: the turn limit and its rate, then the position clamp
     * (`specs/rider-ui.md` 3.3b, `specs/state-feedback.md` section 10).
     *
     * The TUNE screen's rows, not Setup's, by the owner's decision of 2026-10-23: they are set
     * while a machine is being tuned, beside the gains and the stick, and the alternative was a
     * debug probe. So they are absent from [ALL], which is the Setup screen's list and what it
     * reads from the board; the Tune model reads and writes these three itself.
     */
    val DRIVE_BOUNDS: List<SetupField> = listOf(TURN_MAX, TURN_SLEW, POS_CLAMP)

    /** Every row, in screen order within each group. */
    val ALL: List<SetupField> = listOf(
        DEVICE_NAME, NODE_CLASS, NODE_ADDRESS, LINK_SET,
        CONTROL_MODE, RIDER_REQUIRED, PACK_CELLS, BATTERY_FLOOR, MOTOR_METHOD, MOTOR_DIRECTION,
        MOTOR_ALIGN_OFFSET, MOTOR_DEAD_TIME, MOTOR_CURRENT_LIMIT, DRIVE_MODE, CONTROL_FRAME, CONTROL_SIDE,
        IMU_MODEL,
    ) + LEVEL_TRIM + GYRO_BIAS + AXIS_SIGN + AXIS_ROLE + VBATT_CAL + GAIN_MAX

    private val byKey: Map<Key, SetupField> = (ALL + DRIVE_BOUNDS).associateBy { it.key }

    /**
     * The row editing [key], or null when no screen has one. The Tune screen's [DRIVE_BOUNDS] are
     * in here as well as Setup's [ALL], because what a key is CALLED is the same question on both
     * screens; what [ALL] decides is which rows Setup lists and reads.
     */
    fun forKey(key: Key): SetupField? = byKey[key]

    /**
     * The rows Setup lists and reads for a node of [nodeClass]: every row for a motor controller,
     * and [BRIDGE]'s for a bridge or a node whose class is not read yet.
     */
    fun rowsFor(nodeClass: Int?): List<SetupField> =
        if (nodeClass == Fields.NodeClass.MOTOR_CONTROLLER) ALL else BRIDGE

    /** `MOTOR_DIRECTION`'s bytes: 0 = forward, and the firmware reads any nonzero byte as reverse. */
    const val DIRECTION_FORWARD = 0
    const val DIRECTION_REVERSE = 1

    /** The six-step align offset is a sector rotation, 0..5 (`commutation::sixstep`, taken mod 6). */
    const val ALIGN_OFFSET_MAX = 5L

    /** The largest axis role, chip Z (`1 = X`, `2 = Y`, `3 = Z`). */
    const val ROLE_MAX = 3L

}

/** The label a drive mode goes by, on the Machine chips, the raw row and the Ride strip. */
@StringRes
fun driveModeLabel(mode: DriveMode): Int = when (mode) {
    DriveMode.SINGLE -> R.string.drive_mode_single
    DriveMode.BOUND -> R.string.drive_mode_bound
    DriveMode.DIFFERENTIAL -> R.string.drive_mode_differential
    DriveMode.RIGID -> R.string.drive_mode_rigid
}
