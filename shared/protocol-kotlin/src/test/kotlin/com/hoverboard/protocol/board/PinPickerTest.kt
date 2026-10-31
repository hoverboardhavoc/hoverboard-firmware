package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.FieldDef
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** What a client may offer for one layout slot: the part's bonded pins, less SWD and taken pins. */
class PinPickerTest {

    private val chip = ChipFamily.F103C8

    private fun slot(def: FieldDef, motor: Int? = null) = checkNotNull(Layout.forField(def, motor))

    private val board = BoardFields(
        selfHold = 0x1C,
        vbatt = 0x04,
        buzzer = 0x19,
        ledGreen = 0x13,
        ledOrange = 0x0F,
        ledRed = 0x14,
        padA = 0x0B,
        padB = 0x2F,
    )

    private fun offered(def: FieldDef, motor: Int? = null, fields: BoardFields = board, part: ChipFamily = chip) =
        candidatePins(slot(def, motor), fields, part).map { it.packed }

    @Test
    fun aClaimedPinIsOfferedToItsOwnSlotAndToNoOther() {
        assertTrue(0x19 in offered(Fields.BOARD_BUZZER), "its own value, so the row can show it")
        assertFalse(0x19 in offered(Fields.LED_GREEN), "the buzzer holds it")
        assertFalse(0x1C in offered(Fields.LED_GREEN), "the power latch holds it")
    }

    @Test
    fun theSwdPinsAreNeverOffered() {
        for (raw in SWD_PINS) assertFalse(raw in offered(Fields.BOARD_BUZZER), "pin ${raw.toString(16)}")
        assertEquals(listOf(0x0D, 0x0E), SWD_PINS, "PA13 and PA14")
    }

    @Test
    fun aPinThisPartDoesNotBondIsNeverOffered() {
        assertFalse(0x50 in offered(Fields.PAD_B), "PF0 is not bonded on the F103C8")
        assertTrue(0x50 in offered(Fields.PAD_B, part = ChipFamily.F130C8))
    }

    @Test
    fun noFunctionIsNarrowedToWhatTheChipRoutes() {
        // PA7/PB0/PB1 drive the low side on some F130 layouts (robo-2-1-3) and PB13 lights an LED
        // on others: what the chip can route to a function is the board's own answer at boot.
        val f130 = ChipFamily.F130C8
        for (raw in listOf(0x07, 0x10, 0x11)) assertTrue(raw in offered(Fields.MOTOR_GATE_LO_A, 0, part = f130))
        assertTrue(0x1D in offered(Fields.LED_GREEN), "PB13 is offered to an LED")
        assertTrue(0x2D in offered(Fields.BOARD_BATTERY_PIN), "no analog narrowing either")
    }

    @Test
    fun aNonPinSlotOffersNoPins() {
        // The model, the dead time and the current-sense declaration are choices, not pins.
        for (key in listOf(Fields.IMU_MODEL.key(), Fields.MOTOR_DEAD_TIME.key(0), Fields.MOTOR_CURRENT_SENSE.key(0))) {
            val s = checkNotNull(Layout.forKey(key))
            assertFalse(s.isPin)
            assertEquals(emptyList<Pin>(), candidatePins(s, board, chip))
        }
    }
}
