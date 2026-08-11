package com.hoverboard.remote.link

/**
 * The parsed 6-byte header. `opcode` is the raw opcode byte (0..255) so unknown opcodes still
 * round-trip; use [Opcode.fromByte] to resolve it to a known variant.
 */
data class FrameHeader(
    val ver: Int,
    val opcode: Int,
    val src: Int,
    val dst: Int,
    val len: Int,
)

/** A decoded frame: the header plus a copy of the payload bytes. */
class DecodedFrame(val header: FrameHeader, val payload: ByteArray) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is DecodedFrame) return false
        return header == other.header && payload.contentEquals(other.payload)
    }

    override fun hashCode(): Int = 31 * header.hashCode() + payload.contentHashCode()

    override fun toString(): String =
        "DecodedFrame(header=$header, payload=${payload.joinToString(" ") { "%02x".format(it) }})"
}

/**
 * Frame encode/decode. Mirrors crates/link/src/frame.rs byte-for-byte.
 *
 * Layout: SOF, ver, opcode, src, dst, len(=payload.size), payload, then CRC-16/MODBUS (LE) over
 * bytes 0..(6+len).
 */
object Frame {

    /**
     * Encode `opcode` + header ids + `payload` into a fresh frame byte array. The written `len`
     * byte equals `payload.size`. The CRC covers the header and payload (bytes 0..6+len) and is
     * appended little-endian. Throws [IllegalArgumentException] if the payload exceeds
     * [LinkConstants.MAX_PAYLOAD].
     */
    fun encode(opcode: Opcode, src: Int, dst: Int, payload: ByteArray): ByteArray {
        require(payload.size <= LinkConstants.MAX_PAYLOAD) {
            "payload too long: ${payload.size} > ${LinkConstants.MAX_PAYLOAD}"
        }
        val len = payload.size
        val total = LinkConstants.HEADER_LEN + len + LinkConstants.CRC_LEN
        val out = ByteArray(total)
        out[0] = LinkConstants.SOF
        out[1] = LinkConstants.PROTO_VER
        out[2] = (opcode.value and 0xFF).toByte()
        out[3] = (src and 0xFF).toByte()
        out[4] = (dst and 0xFF).toByte()
        out[5] = (len and 0xFF).toByte()
        payload.copyInto(out, LinkConstants.HEADER_LEN)

        val crc = Crc16Modbus.compute(out, 0, LinkConstants.HEADER_LEN + len)
        out[LinkConstants.HEADER_LEN + len] = (crc and 0xFF).toByte()
        out[LinkConstants.HEADER_LEN + len + 1] = ((crc ushr 8) and 0xFF).toByte()
        return out
    }

    /**
     * Decode a single frame from the front of `bytes`. Returns null on any of BadSof, BadVersion,
     * TooShort, LenMismatch, or CrcMismatch (mirroring the Rust `DecodeError` cases). Trailing bytes
     * beyond the declared frame are ignored (only the first 6 + len + 2 bytes are inspected), so a
     * coalesced buffer decodes its first frame.
     */
    fun decode(bytes: ByteArray): DecodedFrame? {
        if (bytes.size < LinkConstants.HEADER_LEN) return null // TooShort
        if (bytes[0] != LinkConstants.SOF) return null // BadSof
        if (bytes[1] != LinkConstants.PROTO_VER) return null // BadVersion
        val len = bytes[5].toInt() and 0xFF
        val total = LinkConstants.HEADER_LEN + len + LinkConstants.CRC_LEN
        if (bytes.size < total) return null // LenMismatch

        val crcCalc = Crc16Modbus.compute(bytes, 0, LinkConstants.HEADER_LEN + len)
        val crcWire = (bytes[LinkConstants.HEADER_LEN + len].toInt() and 0xFF) or
            ((bytes[LinkConstants.HEADER_LEN + len + 1].toInt() and 0xFF) shl 8)
        if (crcCalc != crcWire) return null // CrcMismatch

        val header = FrameHeader(
            ver = bytes[1].toInt() and 0xFF,
            opcode = bytes[2].toInt() and 0xFF,
            src = bytes[3].toInt() and 0xFF,
            dst = bytes[4].toInt() and 0xFF,
            len = len,
        )
        val payload = bytes.copyOfRange(LinkConstants.HEADER_LEN, LinkConstants.HEADER_LEN + len)
        return DecodedFrame(header, payload)
    }
}
