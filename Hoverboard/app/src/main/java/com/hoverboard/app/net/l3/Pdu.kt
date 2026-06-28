package com.hoverboard.app.net.l3

/**
 * The L3 PDU codec, a byte-for-byte mirror of `crates/net/src/pdu.rs`. One PDU rides inside one L2
 * packet:
 *
 * ```text
 * [ opcode : 1 ][ src : 1 ][ dst : 1 ][ payload : ... ]
 * ```
 *
 * L2 owns framing/length/integrity, so the PDU has no SOF, len, CRC, or version byte. `0x00` and
 * `0xFF` are never valid opcodes; an unknown opcode is ignored (not an error), so later opcodes are
 * forward-compatible.
 */

/** `dst = 0xFF` is broadcast. */
const val BROADCAST = 0xFF

/** `0x00` = "no address yet" (an unassigned board's src, or "the one peer" on a point-to-point link). */
const val NO_ADDRESS = 0x00

/** The fixed PDU header length (`opcode` + `src` + `dst`). */
const val HEADER_LEN = 3

/** Is `a` a board address (0x01..=0x7F, persistent, assigned once)? */
fun isBoard(a: Int): Boolean = a in 0x01..0x7F

/** Is `a` a controller / guest address (0x80..=0xFE, transient, session-only)? */
fun isController(a: Int): Boolean = a in 0x80..0xFE

/** Is `a` a unicast, routable, learnable address (0x01..=0xFE)? Excludes 0x00 and 0xFF. */
fun isUnicast(a: Int): Boolean = isBoard(a) || isController(a)

/** The L3 opcodes this layer interprets (the `specs/l3.md` opcode table). */
enum class Opcode(val value: Int) {
    NodeHello(0x01),
    ProbePorts(0x02),
    Ports(0x03),
    Assign(0x06),
    AssignAck(0x07),
    ConfigRead(0x30),
    ConfigWrite(0x31),
    ConfigResp(0x32),
    ConfigWriteMulti(0x33),
    ;

    companion object {
        /** Decode a known L3 opcode, or null for one L3 does not interpret (forward-by-dst / ignore). */
        fun fromU8(b: Int): Opcode? = entries.firstOrNull { it.value == b }
    }
}

/** Why [Pdu.decode] / [Pdu.encode] failed. */
sealed class PduException(message: String) : Exception(message) {
    object TooShort : PduException("fewer than $HEADER_LEN header bytes")
    object InvalidOpcode : PduException("opcode 0x00 / 0xFF is never valid")
}

/** One decoded PDU: the header fields plus the opaque L7 payload (L3 never interprets the payload). */
data class Pdu(val opcode: Int, val src: Int, val dst: Int, val payload: ByteArray) {

    /** The known L3 opcode this PDU carries, or null if L3 does not interpret it. */
    fun known(): Opcode? = Opcode.fromU8(opcode)

    /** Encode the PDU, returning the bytes (`HEADER_LEN + payload`). Throws on a 0x00/0xFF opcode. */
    fun encode(): ByteArray {
        if (opcode == 0x00 || opcode == 0xFF) throw PduException.InvalidOpcode
        val out = ByteArray(HEADER_LEN + payload.size)
        out[0] = opcode.toByte()
        out[1] = src.toByte()
        out[2] = dst.toByte()
        System.arraycopy(payload, 0, out, HEADER_LEN, payload.size)
        return out
    }

    override fun equals(other: Any?): Boolean =
        other is Pdu && opcode == other.opcode && src == other.src && dst == other.dst &&
            payload.contentEquals(other.payload)

    override fun hashCode(): Int =
        ((opcode * 31 + src) * 31 + dst) * 31 + payload.contentHashCode()

    companion object {
        /** Build a PDU from a known [Opcode]. */
        fun of(opcode: Opcode, src: Int, dst: Int, payload: ByteArray): Pdu =
            Pdu(opcode.value, src, dst, payload)

        /** Decode a PDU from one L2 packet. Throws [PduException] on a too-short buffer or 0x00/0xFF. */
        fun decode(buf: ByteArray): Pdu {
            if (buf.size < HEADER_LEN) throw PduException.TooShort
            val opcode = buf[0].toInt() and 0xFF
            if (opcode == 0x00 || opcode == 0xFF) throw PduException.InvalidOpcode
            return Pdu(
                opcode = opcode,
                src = buf[1].toInt() and 0xFF,
                dst = buf[2].toInt() and 0xFF,
                payload = buf.copyOfRange(HEADER_LEN, buf.size),
            )
        }

        /** Decode, returning null instead of throwing (for a "try and ignore" path). */
        fun decodeOrNull(buf: ByteArray): Pdu? = try {
            decode(buf)
        } catch (_: PduException) {
            null
        }
    }
}
