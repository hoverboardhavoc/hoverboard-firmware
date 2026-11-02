package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The `STATUS` decode, against the record's encoding (`StatusRecord` in
 * `motorcontroller/board/src/status.rs`) and the crate's own vectors for it
 * (`motorcontroller/board/src/tests.rs, every_reason_round_trips_through_its_byte`).
 */
class BoardStatusTest {

    private fun bytes(vararg b: Int) = ByteArray(b.size) { b[it].toByte() }

    @Test
    fun anEmptyValueIsABoardWhoseFunctionsAllCameUp() {
        assertEquals(emptyList<BoardStatus.Entry>(), BoardStatus.decode(ByteArray(0)))
    }

    @Test
    fun entriesDecodeFourBytesEachInRecordOrder() {
        val entries = BoardStatus.decode(
            bytes(
                0x0F, 1, 6, 3, // links[1]: the HAL refused it, code 3
                0x11, 0, 6, 0, // IMU: did not answer its probe
            ),
        )
        assertEquals(
            listOf(BoardStatus.Entry(Fields.LINK_TX.id, 1, 6, 3), BoardStatus.Entry(Fields.IMU_SCL_PIN.id, 0, 6, 0)),
            entries,
        )
        assertEquals(listOf(BoardStatus.Reason.NOT_UP, BoardStatus.Reason.NOT_UP), entries.map { it.reason })
    }

    @Test
    fun aTrailingPartialEntryIsIgnored() {
        val entries = BoardStatus.decode(bytes(0x07, 0, 6, 0x40, 0x4A, 0))
        assertEquals(listOf(BoardStatus.Entry(Fields.BOARD_BATTERY_PIN.id, 0, 6, 0x40)), entries)
    }

    @Test
    fun bytesAboveSevenBitsReadUnsigned() {
        val e = BoardStatus.decode(bytes(0xFE, 0x80, 0xFF, 0xC8)).single()
        assertEquals(BoardStatus.Entry(0xFE, 0x80, 0xFF, 0xC8), e)
        assertNull(e.reason, "a byte no reason has")
    }

    @Test
    fun notUpIsTheOneReasonAndKeepsItsNumber() {
        assertEquals(listOf(6), BoardStatus.Reason.entries.map { it.code })
        assertEquals(BoardStatus.Reason.NOT_UP, BoardStatus.Reason.of(6))
        for (gone in listOf(1, 2, 3, 4, 5, 7)) assertNull(BoardStatus.Reason.of(gone), "reason $gone went with the firmware's checks")
    }
}
