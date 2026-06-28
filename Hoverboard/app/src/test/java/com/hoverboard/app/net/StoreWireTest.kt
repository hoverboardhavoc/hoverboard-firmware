package com.hoverboard.app.net

import com.hoverboard.app.net.store.Type
import com.hoverboard.app.net.store.Value
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/** Mirror of `crates/store/src/{key,value}.rs`: the Type tags + little-endian Value forms. */
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
    }

    @Test
    fun decodeRejectsWidthMismatchAndBadUtf8() {
        assertNull(Value.decode(Type.U32, byteArrayOf(1, 2))) // U32 needs 4 bytes
        assertNull(Value.decode(Type.U8, ByteArray(0)))
        assertNull(Value.decode(Type.Str, byteArrayOf(0xFF.toByte(), 0xFE.toByte()))) // invalid UTF-8
        // A Blob accepts any bytes.
        assertEquals(Value.Bytes(byteArrayOf(0xFF.toByte(), 0xFE.toByte())), Value.decode(Type.Blob, byteArrayOf(0xFF.toByte(), 0xFE.toByte())))
    }
}
