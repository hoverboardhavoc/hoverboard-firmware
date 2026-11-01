package com.hoverboard.remote.model

import androidx.annotation.StringRes
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.R

/** The layout editor's sections (`specs/remote-ui.md` section 3.5). */
enum class LayoutGroup(@StringRes val title: Int) {
    BOARD(R.string.layout_group_board),
    IMU(R.string.layout_group_imu),

    /** One section per motor; the title takes the motor index. */
    MOTOR(R.string.layout_group_motor),
}

/** How one layout field is edited. */
sealed interface LayoutEditor {
    /** A pin, chosen from the pins the picker offers for this slot. */
    data object Pin : LayoutEditor

    /** One of a fixed set of byte values. */
    data class Choices(val choices: List<Choice>) : LayoutEditor

    /**
     * A number in an inclusive range, plus [unset]: the one value outside that range the field
     * also takes, where its unset state is not the bottom of its valid one (`motor.dead_time`,
     * whose 0 means no gates while its configured values start at a safety floor).
     */
    data class Number(val range: LongRange, val unset: Long? = null) : LayoutEditor {
        /** Whether this editor takes [v]. */
        fun accepts(v: Long): Boolean = v in range || v == unset
    }

    /** Shown, never edited here. */
    data object ReadOnly : LayoutEditor
}

/**
 * One row of the layout editor: a field of the layout, what to call it, and how it is edited.
 *
 * @param note a standing rule shown under the control, or null.
 * @param noteArgs the note's format arguments, where it states a number the model owns.
 */
data class LayoutRow(
    val slot: LayoutSlot,
    @StringRes val label: Int,
    val group: LayoutGroup,
    val editor: LayoutEditor,
    @StringRes val note: Int? = null,
    val noteArgs: List<Any> = emptyList(),
)

/**
 * The editor's rows: the pins of a layout and the few facts that configure them, with their labels.
 *
 * Not every field of a layout has a row. The three carried per-motor facts (direction, align offset,
 * counts per amp) are read with the rest, because a layout is one object, but the Setup screen
 * already edits the two that anyone edits, so a row for them here would be a second editor for a
 * field that is not this screen's subject.
 *
 * The power latch has a row and no editor, which is the one deliberate exception
 * (`specs/remote-ui.md` section 3.5, "Safety rails"): on battery that pin is what holds the board's
 * own rail up, so a wrong one powers the board off at boot and only SWD recovers it. It is shown so
 * an operator can SEE what the board latches through, and it is not offered in the picker.
 */
object LayoutRows {

    private val imuModels = listOf(
        Choice(Fields.ImuModel.NONE, R.string.layout_choice_imu_none),
        Choice(Fields.ImuModel.MPU6050, R.string.layout_choice_imu_mpu6050),
        Choice(Fields.ImuModel.CLONE_2E, R.string.layout_choice_imu_clone),
    )

    /** Every row, in screen order. */
    val ALL: List<LayoutRow> = buildList {
        fun row(
            field: FieldDef,
            motor: Int?,
            @StringRes label: Int,
            group: LayoutGroup,
            editor: LayoutEditor = LayoutEditor.Pin,
            @StringRes note: Int? = null,
            noteArgs: List<Any> = emptyList(),
        ) = add(
            LayoutRow(
                slot = checkNotNull(Layout.forField(field, motor)) { "${field.id} has no layout slot" },
                label = label,
                group = group,
                editor = editor,
                note = note,
                noteArgs = noteArgs,
            ),
        )

        // The power latch: shown, and the one row with no editor (Layout.LATCH says why).
        add(
            LayoutRow(
                slot = Layout.LATCH,
                label = R.string.layout_field_self_hold,
                group = LayoutGroup.BOARD,
                editor = LayoutEditor.ReadOnly,
                note = R.string.layout_note_self_hold,
            ),
        )
        row(Fields.BOARD_BATTERY_PIN, null, R.string.layout_field_vbatt, LayoutGroup.BOARD)
        row(Fields.BOARD_BUZZER, null, R.string.layout_field_buzzer, LayoutGroup.BOARD)
        row(Fields.LED_GREEN, null, R.string.layout_field_led_green, LayoutGroup.BOARD)
        row(Fields.LED_ORANGE, null, R.string.layout_field_led_orange, LayoutGroup.BOARD)
        row(Fields.LED_RED, null, R.string.layout_field_led_red, LayoutGroup.BOARD)
        row(Fields.PAD_A, null, R.string.layout_field_pad_a, LayoutGroup.BOARD)
        row(Fields.PAD_B, null, R.string.layout_field_pad_b, LayoutGroup.BOARD)
        row(Fields.BOARD_BUTTON, null, R.string.layout_field_button, LayoutGroup.BOARD)

        row(Fields.IMU_SCL_PIN, null, R.string.layout_field_imu_scl, LayoutGroup.IMU, note = R.string.layout_note_imu)
        row(Fields.IMU_SDA_PIN, null, R.string.layout_field_imu_sda, LayoutGroup.IMU)
        row(
            Fields.IMU_MODEL, null, R.string.layout_field_imu_model, LayoutGroup.IMU,
            editor = LayoutEditor.Choices(imuModels),
        )

        for (m in 0 until BoardFields.MOTORS) {
            row(
                Fields.MOTOR_HALL_A, m, R.string.layout_field_hall_a, LayoutGroup.MOTOR,
                note = R.string.layout_note_halls,
            )
            row(Fields.MOTOR_HALL_B, m, R.string.layout_field_hall_b, LayoutGroup.MOTOR)
            row(Fields.MOTOR_HALL_C, m, R.string.layout_field_hall_c, LayoutGroup.MOTOR)
            row(
                Fields.MOTOR_GATE_HI_A, m, R.string.layout_field_gate_hi_a, LayoutGroup.MOTOR,
                note = R.string.layout_note_gates,
            )
            row(Fields.MOTOR_GATE_HI_B, m, R.string.layout_field_gate_hi_b, LayoutGroup.MOTOR)
            row(Fields.MOTOR_GATE_HI_C, m, R.string.layout_field_gate_hi_c, LayoutGroup.MOTOR)
            row(Fields.MOTOR_GATE_LO_A, m, R.string.layout_field_gate_lo_a, LayoutGroup.MOTOR)
            row(Fields.MOTOR_GATE_LO_B, m, R.string.layout_field_gate_lo_b, LayoutGroup.MOTOR)
            row(Fields.MOTOR_GATE_LO_C, m, R.string.layout_field_gate_lo_c, LayoutGroup.MOTOR)
            row(
                Fields.MOTOR_DEAD_TIME, m, R.string.layout_field_dead_time, LayoutGroup.MOTOR,
                editor = LayoutEditor.Number(DEAD_TIME_MIN_DTG.toLong()..DEAD_TIME_MAX, unset = DEAD_TIME_UNSET),
                note = R.string.layout_note_dead_time, noteArgs = listOf(DEAD_TIME_MIN_DTG),
            )
            row(
                Fields.MOTOR_PHASE_A, m, R.string.layout_field_phase_a, LayoutGroup.MOTOR,
                note = R.string.layout_note_phase,
            )
            row(Fields.MOTOR_PHASE_B, m, R.string.layout_field_phase_b, LayoutGroup.MOTOR)
        }
    }

    /** The rows of [group], for [motor] where the group is per-motor. */
    fun of(group: LayoutGroup, motor: Int? = null): List<LayoutRow> =
        ALL.filter { it.group == group && (motor == null || it.slot.motor == motor) }

    /** The row editing [slot], or null when the screen has none. */
    fun forSlot(slot: LayoutSlot): LayoutRow? = ALL.firstOrNull { it.slot.key == slot.key }

    /** The dead time is a raw DTG byte, so the type is its only upper bound. */
    const val DEAD_TIME_MAX = 255L

    /**
     * The dead time with no gate group behind it. The motor bring-up takes 0 only with the gates
     * unset, and a configured gate group at or above [DEAD_TIME_MIN_DTG], so the row offers
     * the floor upwards plus this one value (`protocol-kotlin`'s `DEAD_TIME_MIN_DTG` owns why).
     */
    const val DEAD_TIME_UNSET = 0L
}
