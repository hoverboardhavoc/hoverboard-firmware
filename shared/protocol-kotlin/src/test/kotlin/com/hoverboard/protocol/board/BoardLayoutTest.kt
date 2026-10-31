package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The pin vocabulary against the Rust crate's own vector (`parse_accepts_the_encoding_and_rejects_the_rest`
 * in `motorcontroller/board/src/tests.rs`), and the layout as one object: how the editor reads a board's fields into one value and writes the
 * difference back.
 */
class BoardLayoutTest {

    /** The benign fleet defaults, with vbatt on PA4. */
    private fun blankBoard() = BoardFields(
        selfHold = 0x1C, // PB12
        vbatt = 0x04, // PA4
        buzzer = 0x19, // PB9
        ledGreen = 0x13, // PB3
        ledOrange = 0x0F, // PA15
        ledRed = 0x14, // PB4
        padA = 0x0B, // PA11
        padB = 0x2F, // PC15
        button = PIN_ABSENT,
        imuScl = PIN_ABSENT,
        imuSda = PIN_ABSENT,
        imuModel = 0,
    )

    /** The bench 6-FET preset's motor-0 wiring. */
    private fun benchMotor0() = MotorFields(
        hallA = 0x2D, // PC13
        hallB = 0x01, // PA1
        hallC = 0x2E, // PC14
        gateHiA = 0x08, // PA8
        gateHiB = 0x09, // PA9
        gateHiC = 0x0A, // PA10
        gateLoA = 0x1D, // PB13
        gateLoB = 0x1E, // PB14
        gateLoC = 0x1F, // PB15
        deadTime = 25,
        direction = 0,
        alignOffset = 3,
        currentSense = 1,
        currentCal = 455,
        phaseA = 0x10, // PB0
        phaseB = 0x00, // PA0
    )

    private fun BoardFields.withMotor(m: Int, mf: MotorFields) =
        copy(motors = motors.mapIndexed { i, old -> if (i == m) mf else old })

    private fun pin(raw: Int): Pin = (Pin.parse(raw) as Parsed.Valid).pin

    // --- the encoding: `parse_accepts_the_encoding_and_rejects_the_rest` in
    // motorcontroller/board/src/tests.rs ----------------------------------------------------------

    @Test
    fun parseAcceptsTheEncodingAndRejectsTheRest() {
        for ((port, byte) in listOf(0 to 0x00, 1 to 0x1F, 2 to 0x2D, 3 to 0x30, 5 to 0x5A)) {
            val p = (Pin.parse(byte) as Parsed.Valid).pin
            assertEquals(port, p.port, "port of ${byte.toString(16)}")
            assertEquals(byte and 0x0F, p.pin)
            assertEquals(byte, p.packed)
        }
        assertEquals(Parsed.Absent, Pin.parse(PIN_ABSENT))
        // Port E (4) is not in the encoding, and neither is anything above F.
        for (byte in listOf(0x40, 0x4C, 0x60, 0x7F, 0x90, 0xA1, 0xE0, 0xF0, 0xFE)) {
            assertEquals(Parsed.Invalid, Pin.parse(byte), "byte ${byte.toString(16)} must be invalid")
        }
    }

    @Test
    fun aPinSpellsItselfTheWayAPresetWritesIt() {
        // The client-side spelling the preset contract uses, and its inverse.
        assertEquals("PB12", pin(0x1C).name)
        assertEquals("PA0", pin(0x00).name)
        assertEquals("PF7", pin(0x57).name)
        assertEquals(pin(0x1C), Pin.byName("PB12"))
        assertEquals(pin(0x2E), Pin.byName("PC14"))
        assertNull(Pin.byName("PE0"), "port E is not in the encoding")
        assertNull(Pin.byName("PB16"), "16 is not a pin number")
        assertNull(Pin.byName("nonsense"))
    }

    // --- the layout as one object ---------------------------------------------------------------

    @Test
    fun aLayoutReadsBackAsTheFieldsItWasBuiltFrom() {
        val fields = blankBoard()
            .copy(imuScl = 0x16, imuSda = 0x17, imuModel = 2)
            .withMotor(0, benchMotor0())
        val stored = Layout.SLOTS.associate { it.key to it.value(it.of(fields)) }
        assertEquals(emptyList<LayoutSlot>(), Layout.missing(stored))
        assertEquals(fields, Layout.fieldsFrom(stored))
    }

    @Test
    fun aPartialReadIsNoLayoutAtAll() {
        val fields = blankBoard()
        val stored = Layout.SLOTS.associate { it.key to it.value(it.of(fields)) } - Fields.PAD_B.key()
        assertNull(Layout.fieldsFrom(stored), "a layout is one object: a gap is not a layout")
        assertEquals(listOf(Fields.PAD_B.key()), Layout.missing(stored).map { it.key })
    }

    @Test
    fun aWrongTypeReadsAsMissingRatherThanAsAValue() {
        val stored = Layout.SLOTS.associate { it.key to Value.I32(0) }
        assertNull(Layout.fieldsFrom(stored))
        assertEquals(Layout.SLOTS.size, Layout.missing(stored).size)
    }

    @Test
    fun theDeltaIsTheWritesAndNothingElse() {
        val from = blankBoard()
        val to = from.copy(buzzer = 0x15).withMotor(1, MotorFields(deadTime = 32))
        assertEquals(
            listOf(Fields.BOARD_BUZZER.key(), Fields.MOTOR_DEAD_TIME.key(1)),
            Layout.writes(from, to).map { it.first },
            "one write per changed slot, in slot order",
        )
        assertEquals(listOf(Value.U8(0x15), Value.U8(32)), Layout.writes(from, to).map { it.second })
        assertEquals(emptyList<Pair<*, *>>(), Layout.writes(from, from), "nothing to write for no change")
    }

    @Test
    fun aSlotIsFoundByItsFieldAndMotor() {
        assertEquals(Fields.BOARD_BUZZER.key(), Layout.forField(Fields.BOARD_BUZZER)?.key)
        assertEquals(Fields.MOTOR_HALL_B.key(1), Layout.forField(Fields.MOTOR_HALL_B, 1)?.key)
        assertEquals(Fields.MOTOR_HALL_B.key(0), Layout.forField(Fields.MOTOR_HALL_B)?.key)
        assertEquals(Fields.BOARD_SELF_HOLD.key(), Layout.LATCH.key)
        assertNull(Layout.forField(Fields.IMU_AXIS_SIGN), "the frame is not a layout field")
    }
}
