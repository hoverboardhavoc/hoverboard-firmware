package com.hoverboard.protocol.board

import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** What a client may offer for one layout slot: the chip's table, less SWD and taken pins. */
class PinPickerTest {

    private fun slot(def: FieldDef, index: Int? = null) = checkNotNull(Layout.forField(def, index))

    private val blank = BoardFields()

    private fun offered(def: FieldDef, chip: Chip, index: Int? = null, fields: BoardFields = blank) =
        candidates(slot(def, index), fields, chip)

    @Test
    fun aHeldPinIsOfferedToItsOwnSlotAndToNoOther() {
        // The buzzer and the latch hold their fleet pins, PB9 and PB12, without storing them.
        assertTrue(Value.Pin(1, 9) in offered(Fields.BOARD_BUZZER, Chip.GD32F103C8), "its own value")
        assertFalse(Value.Pin(1, 9) in offered(Fields.LED_GREEN, Chip.GD32F103C8), "the buzzer holds it")
        assertFalse(Value.Pin(1, 12) in offered(Fields.LED_GREEN, Chip.GD32F103C8), "the power latch holds it")
        val pads = blank.with(slot(Fields.PAD_A), Value.Pin(0, 11))
        assertFalse(Value.Pin(0, 11) in offered(Fields.PAD_B, Chip.GD32F103C8, fields = pads), "a stored pin is held")
    }

    @Test
    fun theSwdPinsAndUnbondedPinsAreNeverOffered() {
        val offer = offered(Fields.PAD_B, Chip.GD32F103C8)
        assertFalse(Value.Pin(0, 13) in offer)
        assertFalse(Value.Pin(0, 14) in offer)
        assertFalse(Value.Pin(5, 0) in offer, "PF0 is not bonded on the F103C8")
        assertTrue(Value.Pin(5, 0) in offered(Fields.PAD_B, Chip.GD32F130C8))
    }

    @Test
    fun anAnalogSlotOffersItsPinsWithTheirChannels() {
        val offer = offered(Fields.BOARD_BATTERY_PIN, Chip.GD32F103C8)
        assertTrue(Value.PinAdc(0, 4, 4) in offer)
        assertTrue(Value.PinAdc(1, 0, 8) in offer)
        assertFalse(offer.any { it is Value.PinAdc && it.port == 2 }, "PC0-5 are not bonded on the LQFP48")
    }

    @Test
    fun aGateOffersTheRoutesOfItsFunctionWithTheirAfOrZeroOnARemapChip() {
        assertEquals(listOf(Value.PinAf(0, 8, 2)), offered(Fields.MOTOR_GATE_HI_A, Chip.GD32F130C8, 0))
        assertEquals(listOf(Value.PinAf(0, 8, 0)), offered(Fields.MOTOR_GATE_HI_A, Chip.GD32F103C8, 0))
        assertEquals(
            listOf(Value.PinAf(0, 7, 0), Value.PinAf(1, 13, 0)),
            offered(Fields.MOTOR_GATE_LO_A, Chip.GD32F103RC, 0),
            "TIMER7's CH0N and TIMER0's",
        )
        assertEquals(
            listOf(Value.PinAf(1, 6, 1), Value.PinAf(1, 10, 1)),
            offered(Fields.IMU_SCL_PIN, Chip.GD32F130C8),
        )
    }

    @Test
    fun aSlotThatHoldsNoPinOffersNothing() {
        for (def in listOf(Fields.IMU_I2C, Fields.MOTOR_DEAD_TIME, Fields.MOTOR_CURRENT_CAL)) {
            val s = checkNotNull(Layout.forField(def, 0))
            assertFalse(s.isPin)
            assertEquals(emptyList<Value>(), candidates(s, blank, Chip.GD32F130C8))
        }
    }
}
