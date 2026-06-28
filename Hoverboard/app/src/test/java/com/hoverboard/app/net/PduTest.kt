package com.hoverboard.app.net

import com.hoverboard.app.net.l3.HEADER_LEN
import com.hoverboard.app.net.l3.Opcode
import com.hoverboard.app.net.l3.Pdu
import com.hoverboard.app.net.l3.PduException
import com.hoverboard.app.net.l3.isBoard
import com.hoverboard.app.net.l3.isController
import com.hoverboard.app.net.l3.isUnicast
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** Mirror of `crates/net/src/pdu.rs`'s codec tests: the PDU wire form must match byte-for-byte. */
class PduTest {

    @Test
    fun roundTripsEveryKnownOpcode() {
        val payload = byteArrayOf(0xDE.toByte(), 0xAD.toByte(), 0xBE.toByte(), 0xEF.toByte())
        for (op in Opcode.entries) {
            val pdu = Pdu.of(op, 0x01, 0x80, payload)
            val bytes = pdu.encode()
            assertEquals(HEADER_LEN + payload.size, bytes.size)
            // The header bytes are laid out exactly [opcode][src][dst].
            assertEquals(op.value, bytes[0].toInt() and 0xFF)
            assertEquals(0x01, bytes[1].toInt() and 0xFF)
            assertEquals(0x80, bytes[2].toInt() and 0xFF)
            val got = Pdu.decode(bytes)
            assertEquals(pdu, got)
            assertEquals(op, got.known())
        }
    }

    @Test
    fun emptyPayloadRoundTrips() {
        val pdu = Pdu.of(Opcode.ProbePorts, 0x80, 0x01, ByteArray(0))
        val bytes = pdu.encode()
        assertEquals(HEADER_LEN, bytes.size)
        val got = Pdu.decode(bytes)
        assertArrayEquals(ByteArray(0), got.payload)
        assertEquals(pdu, got)
    }

    @Test
    fun unknownOpcodeDecodesToIgnoreNotError() {
        // An opcode L3 does not interpret (an L7 0x10) decodes fine; known() is null (forward/ignore).
        val buf = byteArrayOf(0x10, 0x02, 0x03, 0x99.toByte())
        val got = Pdu.decode(buf)
        assertEquals(0x10, got.opcode)
        assertNull(got.known())
        assertEquals(0x02, got.src)
        assertEquals(0x03, got.dst)
        assertArrayEquals(byteArrayOf(0x99.toByte()), got.payload)
    }

    @Test
    fun opcode0x00And0xffAreRejected() {
        assertThrows(PduException.InvalidOpcode::class.java) { Pdu.decode(byteArrayOf(0x00, 1, 2)) }
        assertThrows(PduException.InvalidOpcode::class.java) { Pdu.decode(byteArrayOf(0xFF.toByte(), 1, 2)) }
        assertThrows(PduException.InvalidOpcode::class.java) { Pdu(0xFF, 1, 2, ByteArray(0)).encode() }
    }

    @Test
    fun tooShortBufferIsRejected() {
        assertThrows(PduException.TooShort::class.java) { Pdu.decode(ByteArray(0)) }
        assertThrows(PduException.TooShort::class.java) { Pdu.decode(byteArrayOf(0x01)) }
        assertThrows(PduException.TooShort::class.java) { Pdu.decode(byteArrayOf(0x01, 0x02)) }
        // decodeOrNull swallows it.
        assertNull(Pdu.decodeOrNull(byteArrayOf(0x01)))
    }

    @Test
    fun addressingRangeHelpers() {
        assertTrue(isBoard(0x01) && isBoard(0x7F))
        assertFalse(isBoard(0x00) || isBoard(0x80))
        assertTrue(isController(0x80) && isController(0xFE))
        assertFalse(isController(0x7F) || isController(0xFF))
        assertTrue(isUnicast(0x01) && isUnicast(0xFE))
        assertFalse(isUnicast(0x00) || isUnicast(0xFF))
    }
}
