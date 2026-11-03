package com.hoverboard.protocol.board

import com.hoverboard.protocol.checks.Checks
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/** The layout as one object: how the editor reads a board's keys into one value and writes the difference back. */
class BoardLayoutTest {

    /** Every slot read, the board storing only [stored]: the rest answered with their defaults. */
    private fun read(stored: Map<Key, Value> = emptyMap()): Map<Key, Value?> =
        Layout.SLOTS.associate { it.key to stored[it.key] }

    private val gates = mapOf(
        Fields.MOTOR_GATE_HI_A.key(0) to Value.PinAf(0, 8, 2),
        Fields.MOTOR_GATE_TIMER.key(0) to Value.U8(0),
        Fields.MOTOR_DEAD_TIME.key(0) to Value.U8(28),
    )

    @Test
    fun aLayoutHoldsWhatTheBoardStoresAndNothingItAnsweredWithItsDefault() {
        val fields = checkNotNull(Layout.fieldsFrom(read(gates)))
        assertEquals(gates, fields.values)
        assertEquals(emptyList<LayoutSlot>(), Layout.missing(read(gates)))
        assertNull(Layout.LATCH.of(fields), "the latch is not stored")
    }

    /** `specs/no-fleet-pins.md` section 2: an unstored latch, buzzer or LED reads as absent, not as a pin. */
    @Test
    fun anUnstoredLatchBuzzerOrLedReadsAsAbsent() {
        val fields = checkNotNull(Layout.fieldsFrom(read()))
        val unstored = listOf(
            Fields.BOARD_SELF_HOLD, Fields.BOARD_BUZZER, Fields.LED_GREEN, Fields.LED_ORANGE, Fields.LED_RED,
        )
        for (def in unstored) {
            val slot = checkNotNull(Layout.forField(def))
            assertNull(slot.of(fields), "${def.id} is absent")
            assertNull(Checks.pinAt(fields.values, slot.key), "${def.id} holds no pin")
            assertEquals(Value.Pin(0xFF, 0xFF), def.default, "${def.id}'s default is the absent bytes")
        }
    }

    @Test
    fun aPartialReadIsNoLayoutAtAll() {
        val partial = read() - Fields.PAD_B.key()
        assertNull(Layout.fieldsFrom(partial), "a layout is one object: a gap is not a layout")
        assertEquals(listOf(Fields.PAD_B.key()), Layout.missing(partial).map { it.key })
    }

    @Test
    fun aWrongTypeReadsAsMissingRatherThanAsAValue() {
        val wrong = Layout.SLOTS.associate { it.key to Value.I32(0) }
        assertNull(Layout.fieldsFrom(wrong))
        assertEquals(Layout.SLOTS.size, Layout.missing(wrong).size)
    }

    @Test
    fun theDeltaIsTheWritesAndNothingElse() {
        val from = checkNotNull(Layout.fieldsFrom(read()))
        val buzzer = checkNotNull(Layout.forField(Fields.BOARD_BUZZER))
        val deadTime = checkNotNull(Layout.forField(Fields.MOTOR_DEAD_TIME, 1))
        val to = from.with(buzzer, Value.Pin(1, 5)).with(deadTime, Value.U8(32))
        assertEquals(
            listOf(Fields.BOARD_BUZZER.key() to Value.Pin(1, 5), Fields.MOTOR_DEAD_TIME.key(1) to Value.U8(32)),
            Layout.writes(from, to),
            "one write per changed slot, in slot order",
        )
        assertEquals(emptyList<Pair<*, *>>(), Layout.writes(from, from), "nothing to write for no change")
    }

    @Test
    fun aSlotIsFoundByItsFieldAndIndex() {
        assertEquals(Fields.BOARD_BUZZER.key(), Layout.forField(Fields.BOARD_BUZZER)?.key)
        assertEquals(Fields.MOTOR_HALL_B.key(1), Layout.forField(Fields.MOTOR_HALL_B, 1)?.key)
        assertEquals(1, Layout.forField(Fields.MOTOR_HALL_B, 1)?.motor)
        assertEquals(Key(Fields.LINK_USART.id, 1), Layout.forField(Fields.LINK_USART.at(1), 1)?.key)
        assertNull(Layout.forField(Fields.LINK_TX, 1)?.motor, "a link is not a motor")
        assertEquals(Fields.BOARD_SELF_HOLD.key(), Layout.LATCH.key)
        assertNull(Layout.forField(Fields.IMU_AXIS_SIGN.at(0)), "the frame is not a layout field")
    }
}
