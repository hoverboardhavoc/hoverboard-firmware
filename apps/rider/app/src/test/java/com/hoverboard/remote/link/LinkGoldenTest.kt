package com.hoverboard.remote.link

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Cross-repo golden-vector contract. Reads `golden_frames.txt` from test resources (a byte-for-byte
 * copy of crates/link/tests/golden_frames.txt, the Rust source of truth) and asserts that the
 * Kotlin [Frame.encode] reproduces every committed `frame_hex` exactly and [Frame.decode] recovers
 * the header fields. This is the durable Kotlin <-> Rust byte agreement.
 *
 * File format, one vector per line: name|opcode_hex|src|dst|payload_hex|frame_hex
 * CRC anchor lines: crc|<input_hex>|<crc16_modbus_hex>. Lines starting with `#` are comments.
 */
class LinkGoldenTest {

    private data class Vector(
        val name: String,
        val opcodeHex: Int,
        val src: Int,
        val dst: Int,
        val payload: ByteArray,
        val frame: ByteArray,
    )

    private fun readGolden(): String {
        val stream = javaClass.classLoader!!.getResourceAsStream("golden_frames.txt")
        assertNotNull(stream, "golden_frames.txt missing from test resources")
        return stream!!.bufferedReader().use { it.readText() }
    }

    private fun hexToBytes(hex: String): ByteArray {
        if (hex.isEmpty()) return ByteArray(0)
        return ByteArray(hex.length / 2) {
            ((Character.digit(hex[it * 2], 16) shl 4) or Character.digit(hex[it * 2 + 1], 16)).toByte()
        }
    }

    private fun bytesToHex(b: ByteArray): String = b.joinToString("") { "%02x".format(it) }

    @Test
    fun `golden frames encode byte-for-byte and decode fields`() {
        val lines = readGolden().lineSequence()
            .map { it.trim() }
            .filter { it.isNotEmpty() && !it.startsWith("#") && !it.startsWith("crc|") }
            .toList()
        assertTrue(lines.size >= 4, "expected at least 4 frame vectors, got ${lines.size}")

        for (line in lines) {
            val f = line.split("|")
            assertEquals(6, f.size, "malformed golden line: $line")
            val v = Vector(
                name = f[0],
                opcodeHex = f[1].toInt(16),
                src = f[2].toInt(),
                dst = f[3].toInt(),
                payload = hexToBytes(f[4]),
                frame = hexToBytes(f[5]),
            )
            val opcode = Opcode.fromByte(v.opcodeHex)
            assertNotNull(opcode, "unknown opcode in golden vector ${v.name}")

            // Encode must equal the committed frame hex byte-for-byte.
            val encoded = Frame.encode(opcode!!, v.src, v.dst, v.payload)
            assertEquals(bytesToHex(v.frame), bytesToHex(encoded), "encode mismatch for ${v.name}")

            // Decode must recover the header fields and payload.
            val decoded = Frame.decode(v.frame)
            assertNotNull(decoded, "decode returned null for ${v.name}")
            assertEquals(LinkConstants.PROTO_VER.toInt() and 0xFF, decoded!!.header.ver, v.name)
            assertEquals(v.opcodeHex, decoded.header.opcode, "opcode field for ${v.name}")
            assertEquals(v.src, decoded.header.src, "src for ${v.name}")
            assertEquals(v.dst, decoded.header.dst, "dst for ${v.name}")
            assertEquals(v.payload.size, decoded.header.len, "len for ${v.name}")
            assertEquals(bytesToHex(v.payload), bytesToHex(decoded.payload), "payload for ${v.name}")
        }
    }

    @Test
    fun `golden crc anchors match Crc16Modbus`() {
        val anchors = readGolden().lineSequence()
            .map { it.trim() }
            .filter { it.startsWith("crc|") }
            .toList()
        assertEquals(3, anchors.size, "expected 3 crc anchor lines")

        for (line in anchors) {
            val f = line.split("|")
            assertEquals(3, f.size, "malformed crc line: $line")
            val input = hexToBytes(f[1])
            val expected = f[2].toInt(16)
            assertEquals(expected, Crc16Modbus.compute(input), "crc anchor for input ${f[1]}")
        }
    }
}
