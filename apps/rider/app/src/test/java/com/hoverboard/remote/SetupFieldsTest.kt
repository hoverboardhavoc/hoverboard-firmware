package com.hoverboard.remote

import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupFields
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
