package com.hoverboard.remote

import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** The Setup screen's hint table against the field mirror it decorates. */
class SetupFieldsTest {

    @Test
    fun everyEditableRowTakesItsFieldsDefault() {
        for (f in SetupFields.ALL.filter { it.editor != Editor.ReadOnly }) {
            assertTrue(f.accepts(f.def.default), "${f.key} refuses its own default ${f.def.default}")
        }
    }

    @Test
    fun everyRowEditsADistinctKey() {
        assertEquals(SetupFields.ALL.size, SetupFields.ALL.map { it.key }.toSet().size)
    }

    @Test
    fun parsingFollowsTheFieldTypeAndTheRowRange() {
        assertEquals(Value.U32(15_000), SetupFields.MOTOR_CURRENT_LIMIT.parse("15000"))
        assertNull(SetupFields.MOTOR_CURRENT_LIMIT.parse("999"), "under the firmware clamp")
        assertNull(SetupFields.MOTOR_ALIGN_OFFSET.parse("6"))
        assertNull(SetupFields.MOTOR_DEAD_TIME.parse("256"), "outside a u8")
        // The dead-time floor: the Setup screen is the second editor of the field the layout
        // screen also offers, and the board refuses a configured gate group below it.
        assertNull(SetupFields.MOTOR_DEAD_TIME.parse("1"), "below the dead-time floor")
        assertNull(SetupFields.MOTOR_DEAD_TIME.parse("${DEAD_TIME_MIN_DTG - 1}"), "below the dead-time floor")
        assertEquals(Value.U8(DEAD_TIME_MIN_DTG), SetupFields.MOTOR_DEAD_TIME.parse("$DEAD_TIME_MIN_DTG"))
        assertEquals(Value.U8(0), SetupFields.MOTOR_DEAD_TIME.parse("0"), "0 = the gates unset")
        assertEquals(Value.I32(-88), SetupFields.GYRO_BIAS[2].parse(" -88 "))
        assertEquals(Value.I16(-266), SetupFields.LEVEL_TRIM[0].parse("-266"))
        assertNull(SetupFields.LEVEL_TRIM[0].parse("40000"), "outside an i16")
        assertEquals(Value.Str("rover-left"), SetupFields.DEVICE_NAME.parse("rover-left"))
        assertNull(SetupFields.AXIS_SIGN[0].parse("2"))
        assertEquals(Value.I32(-1), SetupFields.AXIS_SIGN[0].parse("-1"), "the pose picker's own value")
        assertNull(SetupFields.CONTROL_MODE.parse("2"), "not one of the choices")
        assertEquals(Value.U8(0), SetupFields.RIDER_REQUIRED.parse("0"))
        assertNull(SetupFields.RIDER_REQUIRED.parse("2"), "any nonzero byte reads as required, but only 1 is offered")
        assertEquals(Value.I16(2400), SetupFields.BATTERY_FLOOR.parse("2400"))
        assertEquals(Value.I16(-1), SetupFields.BATTERY_FLOOR.parse("-1"), "no clamp beyond the type")
        assertNull(SetupFields.BATTERY_FLOOR.parse("32768"), "outside an i16")
        assertEquals(Value.I16(30_000), SetupFields.GAIN_MAX[0].parse("30000"))
        assertEquals(Value.I16(0), SetupFields.GAIN_MAX[2].parse("0"))
        assertNull(SetupFields.GAIN_MAX[1].parse("-1"), "a negative maximum reads as 0 on the board")
        assertEquals(Value.I16(31_170), SetupFields.VBATT_CAL[0].parse("31170"))
        assertNull(SetupFields.VBATT_CAL[0].parse("9999"), "under the boot clamp")
        assertNull(SetupFields.VBATT_CAL[1].parse("501"), "over the boot clamp")
        assertEquals(Value.I16(-500), SetupFields.VBATT_CAL[1].parse("-500"))
        assertEquals(Value.U8(3), SetupFields.AXIS_ROLE[0].parse("3"))
        assertNull(SetupFields.AXIS_ROLE[0].parse("4"))
    }

    /**
     * The takes-effect classification, which no row declares: [SetupField.takesEffect] reads
     * [com.hoverboard.protocol.store.Fields.ARM_TIME], the mirror `RustSourceDriftTest` pins
     * against the firmware's own arm row (`crates/firmware/src/arm.rs`, `read_arm_values`, the
     * table in `specs/integration.md`). Every other row is boot-read, whether it configures a
     * peripheral or is a value the boot path owns.
     *
     * This names both sets so that a field moving between them is a failure HERE as well as in the
     * drift gate: the gate watches the Kotlin mirror against the Rust, and this watches what the
     * screen then tells the owner. `motor.current_cal` is in that firmware row and has no row on
     * any screen, so it is absent rather than misclassified; `control.drive_lean` and
     * `control.drive_turn` are the Tune screen's rows and are checked in `TuneBoundTest`.
     */
    @Test
    fun theArmTimeRowIsExactlyTheFieldsTheBoardReReadsWhenItArms() {
        val armTime = SetupFields.ALL.filter { it.takesEffect == TakesEffect.NEXT_ARM }.map { it.key }.toSet()
        val expected = listOf(
            SetupFields.MOTOR_METHOD,
            SetupFields.MOTOR_CURRENT_LIMIT,
            SetupFields.MOTOR_DIRECTION,
            SetupFields.MOTOR_ALIGN_OFFSET,
            SetupFields.CONTROL_MODE,
            SetupFields.RIDER_REQUIRED,
            SetupFields.BATTERY_FLOOR,
            SetupFields.CONTROL_FRAME,
            SetupFields.CONTROL_SIDE,
        ).map { it.key } + (SetupFields.AXIS_SIGN + SetupFields.AXIS_ROLE + SetupFields.GAIN_MAX).map { it.key }
        assertEquals(expected.toSet(), armTime)

        val boot = SetupFields.ALL.filter { it.takesEffect == TakesEffect.RESTART }.map { it.key }.toSet()
        val restartNamed = listOf(
            SetupFields.DEVICE_NAME,
            SetupFields.MOTOR_DEAD_TIME,
            SetupFields.IMU_MODEL,
        ).map { it.key } + (SetupFields.LEVEL_TRIM + SetupFields.GYRO_BIAS + SetupFields.VBATT_CAL).map { it.key }
        assertEquals(restartNamed.toSet(), boot)

        // The third class, which is not a firmware read class at all: the board stores the pack
        // for the app and never acts on it, so there is no board-side event to wait for.
        val appOnly = SetupFields.ALL.filter { it.takesEffect == TakesEffect.APP_ONLY }.map { it.key }.toSet()
        assertEquals(setOf(SetupFields.PACK_CELLS.key), appOnly)
    }

    /** A row the screen can write states when the board takes it up; a read-only row has nothing to say. */
    @Test
    fun onlyTheRowsNothingWritesLackATakesEffectGroup() {
        for (f in SetupFields.ALL) {
            assertEquals(
                f.editor == Editor.ReadOnly,
                f.takesEffect == null,
                "${f.key}: a takes-effect group that does not match what the row can do",
            )
        }
    }

    @Test
    fun walkOwnedFieldsTakeNothing() {
        assertFalse(SetupFields.NODE_ADDRESS.accepts(Value.U8(2)))
        assertFalse(SetupFields.LINK_SET.accepts(Value.U8(1)))
    }

    /** P2-8: a name longer than one `CONFIG_WRITE` and its echo carry is refused, counted in UTF-8 bytes. */
    @Test
    fun theDeviceNameIsBoundedByOneConfigWrite() {
        val f = SetupFields.DEVICE_NAME
        assertEquals(Value.Str("n".repeat(CONFIG_VALUE_MAX)), f.parse("n".repeat(CONFIG_VALUE_MAX)))
        assertNull(f.parse("n".repeat(CONFIG_VALUE_MAX + 1)))
        // Two bytes per character in UTF-8: half as many characters fit.
        assertNull(f.parse("\u00e9".repeat(CONFIG_VALUE_MAX / 2 + 1)))
        assertFalse(f.accepts(Value.Str("n".repeat(CONFIG_VALUE_MAX + 1))))
    }
}
