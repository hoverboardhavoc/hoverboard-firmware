package com.hoverboard.remote

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
        assertEquals(Value.I32(-88), SetupFields.GYRO_BIAS[2].parse(" -88 "))
        assertEquals(Value.I16(-266), SetupFields.LEVEL_TRIM[0].parse("-266"))
        assertNull(SetupFields.LEVEL_TRIM[0].parse("40000"), "outside an i16")
        assertEquals(Value.Str("rover-left"), SetupFields.DEVICE_NAME.parse("rover-left"))
        assertNull(SetupFields.AXIS_SIGN[0].parse("2"))
        assertNull(SetupFields.CONTROL_MODE.parse("2"), "not one of the choices")
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
