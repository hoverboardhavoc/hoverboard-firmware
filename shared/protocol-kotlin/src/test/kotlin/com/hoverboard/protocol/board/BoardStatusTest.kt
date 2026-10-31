package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The `board.status` decode, against the record's encoding (`StatusRecord` in
 * `motorcontroller/board/src/status.rs`) and the crate's own vectors for it
 * (`motorcontroller/board/src/tests.rs` (entries_encode_as_four_bytes_each_in_record_order,
 * a_trailing_partial_entry_is_ignored)).
 */
class BoardStatusTest {

    private fun bytes(vararg b: Int) = ByteArray(b.size) { b[it].toByte() }

    @Test
    fun anEmptyValueIsABoardWhoseFunctionsAllCameUp() {
        assertEquals(emptyList<BoardStatus.Entry>(), BoardStatus.decode(ByteArray(0)))
        assertEquals(emptyList<BoardStatus.Entry>(), BoardStatus.decode(Fields.BOARD_STATUS.default))
    }

    @Test
    fun entriesDecodeFourBytesEachInRecordOrder() {
        val entries = BoardStatus.decode(
            bytes(
                0x56, 1, 3, 0x16, // link[1]: not routed, PB6
                0x4D, 0, 5, 10, // motors[0] gate: dead time 10
                0x48, 0, 6, 0, // IMU: did not answer its probe
            ),
        )
        assertEquals(
            listOf(
                BoardStatus.Entry(Fields.LINK_TX.id, 1, 3, 0x16),
                BoardStatus.Entry(Fields.MOTOR_GATE_HI_A.id, 0, 5, 10),
                BoardStatus.Entry(Fields.IMU_SCL_PIN.id, 0, 6, 0),
            ),
            entries,
        )
        assertEquals(
            listOf(BoardStatus.Reason.NOT_ROUTED, BoardStatus.Reason.DEAD_TIME, BoardStatus.Reason.NOT_UP),
            entries.map { it.reason },
        )
    }

    @Test
    fun aTrailingPartialEntryIsIgnored() {
        val entries = BoardStatus.decode(bytes(0x41, 0, 1, 0x40, 0x4A, 0))
        assertEquals(listOf(BoardStatus.Entry(Fields.BOARD_BATTERY_PIN.id, 0, 1, 0x40)), entries)
    }

    @Test
    fun bytesAboveSevenBitsReadUnsigned() {
        val e = BoardStatus.decode(bytes(0xFE, 0x80, 0xFF, 0xC8)).single()
        assertEquals(BoardStatus.Entry(0xFE, 0x80, 0xFF, 0xC8), e)
        assertNull(e.reason, "a byte no reason has")
    }

    @Test
    fun aValueThatIsNotABlobDecodesToNothing() {
        assertNull(BoardStatus.decode(Value.U8(0)))
        assertEquals(1, BoardStatus.decode(Value.Bytes(bytes(0x40, 0, 4, 0x0D)))?.size)
    }

    @Test
    fun theReasonsAreTheSevenCodesInOrder() {
        assertEquals((1..7).toList(), BoardStatus.Reason.entries.map { it.code })
        for (r in BoardStatus.Reason.entries) assertEquals(r, BoardStatus.Reason.of(r.code))
        assertNull(BoardStatus.Reason.of(0))
        assertNull(BoardStatus.Reason.of(8))
    }

    @Test
    fun aNamePrintsItsIndexTheWayTheCliDoes() {
        assertEquals("motors[1].hall.c", BoardStatus.NAMES.getValue(Fields.MOTOR_HALL_C.id).at(1))
        assertEquals("link.rx[0]", BoardStatus.NAMES.getValue(Fields.LINK_RX.id).at(0))
        assertEquals("pad.a", BoardStatus.NAMES.getValue(Fields.PAD_A.id).at(0))
    }

    @Test
    fun everyLayoutFieldHasAName() {
        // A detail of INCOMPLETE names any member of a group, which is a layout field or a link pin.
        val unnamed = Layout.SLOTS.map { it.def.id }.distinct().filter { it !in BoardStatus.NAMES } -
            setOf(Fields.MOTOR_DIRECTION.id, Fields.MOTOR_ALIGN_OFFSET.id, Fields.MOTOR_CURRENT_CAL.id)
        assertEquals(emptyList<Int>(), unnamed)
    }
}
