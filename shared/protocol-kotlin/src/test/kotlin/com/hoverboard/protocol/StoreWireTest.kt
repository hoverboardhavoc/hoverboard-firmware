package com.hoverboard.protocol

import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Names
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/** Mirror of `shared/store/src/{key,value}.rs`: the Type tags + little-endian Value forms. */
class StoreWireTest {

    @Test
    fun typeTagsMatchTheFirmware() {
        assertEquals(0x01, Type.U8.tag)
        assertEquals(0x02, Type.U16.tag)
        assertEquals(0x03, Type.U32.tag)
        assertEquals(0x04, Type.U64.tag)
        assertEquals(0x05, Type.I16.tag)
        assertEquals(0x06, Type.I32.tag)
        assertEquals(0x07, Type.I64.tag)
        assertEquals(0x08, Type.Bool.tag)
        assertEquals(0x09, Type.Blob.tag)
        assertEquals(0x0A, Type.Str.tag)
        assertEquals(0x0B, Type.Pin.tag)
        assertEquals(0x0C, Type.PinAf.tag)
        assertEquals(0x0D, Type.PinAdc.tag)
        for (t in Type.entries) assertEquals(t, Type.fromTag(t.tag))
    }

    @Test
    fun everyValueTypeEncodeDecodeRoundTrips() {
        val values = listOf(
            Value.U8(0xAB),
            Value.U16(0xBEEF),
            Value.U32(0xDEAD_BEEFL),
            Value.U64(0x0102_0304_0506_0708L),
            Value.I16(-12345),
            Value.I32(-2_000_000_000),
            Value.I64(-9_000_000_000_000_000L),
            Value.Bool(true),
            Value.Bool(false),
            Value.Str("hoverboard"),
            Value.Bytes(byteArrayOf(0xDE.toByte(), 0xAD.toByte(), 0xBE.toByte(), 0xEF.toByte())),
            Value.Pin(1, 12),
            Value.PinAf(0, 8, 2),
            Value.PinAdc(0, 4, 4),
        )
        for (v in values) {
            val bytes = v.encode()
            val back = Value.decode(v.kind(), bytes)
            assertEquals(v, back, "round-trip failed for $v")
        }
    }

    @Test
    fun littleEndianLayoutIsExact() {
        // U32 0xDEADBEEF -> EF BE AD DE on the wire.
        assertArrayEquals(
            byteArrayOf(0xEF.toByte(), 0xBE.toByte(), 0xAD.toByte(), 0xDE.toByte()),
            Value.U32(0xDEAD_BEEFL).encode(),
        )
        assertArrayEquals(byteArrayOf(0xAB.toByte()), Value.U8(0xAB).encode())
        assertArrayEquals(byteArrayOf(1), Value.Bool(true).encode())
        // A pin is its register model in order: port, pin, then the af or the ADC channel.
        assertArrayEquals(byteArrayOf(0, 8, 2), Value.PinAf(0, 8, 2).encode())
        assertArrayEquals(byteArrayOf(2, 13), Value.Pin(2, 13).encode())
    }

    @Test
    fun decodeRejectsWidthMismatchAndBadUtf8() {
        assertNull(Value.decode(Type.U32, byteArrayOf(1, 2))) // U32 needs 4 bytes
        assertNull(Value.decode(Type.U8, ByteArray(0)))
        assertNull(Value.decode(Type.PinAf, byteArrayOf(0, 8))) // a short pin record
        // The absent default a board answers for an unstored pin is the bytes 0xFF.
        assertEquals(Value.PinAdc(0xFF, 0xFF, 0xFF), Value.decode(Type.PinAdc, byteArrayOf(-1, -1, -1)))
        assertNull(Value.decode(Type.Str, byteArrayOf(0xFF.toByte(), 0xFE.toByte()))) // invalid UTF-8
        // A Blob accepts any bytes.
        assertEquals(Value.Bytes(byteArrayOf(0xFF.toByte(), 0xFE.toByte())), Value.decode(Type.Blob, byteArrayOf(0xFF.toByte(), 0xFE.toByte())))
    }

    /**
     * A per-motor family's path (`specs/flux-observer.md`, "Fields"), the CLI's `path_of`: motor
     * m's element e is key index `m * n + e`, printed under that motor.
     */
    @Test
    fun aPerMotorFamilyPrintsUnderItsMotor() {
        assertEquals("motors[1].observer.shadow[4]", Names.pathOf(Key(0x61, 11)))
        assertEquals("motors[0].observer.model[1]", Names.pathOf(Key(0x60, 1)))
        assertEquals("motors[1].observer.model[0]", Names.pathOf(Key(0x60, 4)))
    }
}
