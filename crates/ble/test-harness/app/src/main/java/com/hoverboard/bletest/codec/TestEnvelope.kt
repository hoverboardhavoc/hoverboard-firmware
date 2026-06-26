package com.hoverboard.bletest.codec

/**
 * The harness's own minimal **test envelope**, owned by the harness purely for measurement and
 * deliberately distinct from the `link` frame format (`specs/ble.md`, "Where it lives"). This works
 * BELOW `link`: it carries no opcodes, no addressing, no link CRC. It exists only so the app (the active
 * party and the oracle) can send packets with an increasing `seq` and a known byte pattern, and, because
 * the board echoes the raw bytes verbatim, recover and score exactly what should come back.
 *
 * Wire layout (all multi-byte fields little-endian, like the rest of the project):
 * ```
 *  off  field    size  notes
 *  0    MARKER   1     0xBE  (distinct from the link SOF 0x5A, so the two never alias)
 *  1    seq      2     u16 LE, increasing per sent packet
 *  3    len      1     payload length (0..MAX_PAYLOAD)
 *  4..  payload  len   the known pattern for this seq (see [patternByte])
 *  4+len check   1     XOR self-check over bytes 0..(3+len) (i.e. everything before the check byte)
 * ```
 *
 * Overhead is 5 bytes (marker + seq + len + check). The payload is a *known* pattern derived from `seq`,
 * so a recovered packet can be verified byte-for-byte without the receiver having stored what was sent.
 */
object TestEnvelope {
    const val MARKER: Int = 0xBE
    const val HEADER_LEN: Int = 4 // marker(1) + seq(2) + len(1)
    const val OVERHEAD: Int = HEADER_LEN + 1 // + check(1)

    /** Max payload that fits the 1-byte length field. */
    const val MAX_PAYLOAD: Int = 255

    /**
     * The known payload byte for a given (`seq`, offset). A simple, position-dependent pattern so a
     * single-byte corruption or a misalignment shows up as a mismatch: `(seq + index) mod 256`.
     */
    fun patternByte(seq: Int, index: Int): Byte = ((seq + index) and 0xFF).toByte()

    /** Build the payload pattern for a packet of [len] bytes at sequence [seq]. */
    fun pattern(seq: Int, len: Int): ByteArray = ByteArray(len) { i -> patternByte(seq, i) }

    /**
     * Encode one test packet. [seq] is masked to u16; [payload] must be <= [MAX_PAYLOAD]. The self-check
     * is an XOR over every byte before it.
     */
    fun encode(seq: Int, payload: ByteArray): ByteArray {
        require(payload.size <= MAX_PAYLOAD) { "payload too long: ${payload.size}" }
        val out = ByteArray(OVERHEAD + payload.size)
        out[0] = MARKER.toByte()
        out[1] = (seq and 0xFF).toByte()
        out[2] = ((seq ushr 8) and 0xFF).toByte()
        out[3] = payload.size.toByte()
        System.arraycopy(payload, 0, out, HEADER_LEN, payload.size)
        var check = 0
        for (i in 0 until out.size - 1) check = check xor (out[i].toInt() and 0xFF)
        out[out.size - 1] = check.toByte()
        return out
    }

    /** Convenience: encode the known pattern packet for [seq] of [len] payload bytes. */
    fun encodePattern(seq: Int, len: Int): ByteArray = encode(seq, pattern(seq, len))
}

/** A successfully recovered packet, with the verdict the oracle assigns it. */
data class RecoveredPacket(
    val seq: Int,
    val payload: ByteArray,
    /** True if the payload matched the known pattern for this seq AND the self-check verified. */
    val intact: Boolean,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is RecoveredPacket) return false
        return seq == other.seq && intact == other.intact && payload.contentEquals(other.payload)
    }

    override fun hashCode(): Int = (seq * 31 + intact.hashCode()) * 31 + payload.contentHashCode()
}

/**
 * A streaming parser that recovers test packets from a split / coalesced byte stream, the same job
 * `StreamFramer` does for `link` but for the harness envelope. BLE notifications arrive in arbitrary
 * chunks (the carrier is byte-oriented), so the parser must resync on the [TestEnvelope.MARKER], tolerate
 * a chunk boundary mid-packet, and recover following packets after a corrupted one.
 *
 * On a self-check failure or a pattern mismatch it still emits the packet (so corruption is *counted*,
 * not silently dropped) with `intact = false`; a structurally unparseable byte run is skipped to the next
 * marker (a resync), and resync events are reported via [resyncs].
 */
class StreamRecoverer {
    private val buf = ArrayDeque<Int>() // pending bytes (0..255), oldest first
    var resyncs: Int = 0
        private set

    /** Feed a chunk of received bytes; return every packet now fully recoverable, in order. */
    fun push(chunk: ByteArray): List<RecoveredPacket> {
        for (b in chunk) buf.addLast(b.toInt() and 0xFF)
        val out = ArrayList<RecoveredPacket>()
        while (true) {
            val pkt = tryRecoverOne() ?: break
            out.add(pkt)
        }
        return out
    }

    /** Try to recover one packet from the head of the buffer; null if not enough bytes yet. */
    private fun tryRecoverOne(): RecoveredPacket? {
        // Resync: discard bytes until the head is the marker.
        while (buf.isNotEmpty() && buf.first() != TestEnvelope.MARKER) {
            buf.removeFirst()
            resyncs++
        }
        if (buf.size < TestEnvelope.HEADER_LEN) return null
        val len = buf.elementAt(3) // the length byte (header is marker, seq lo, seq hi, len)
        val total = TestEnvelope.OVERHEAD + len
        if (buf.size < total) return null // wait for the rest of the packet

        // We have a full candidate packet. Pop it.
        val bytes = IntArray(total)
        for (i in 0 until total) bytes[i] = buf.removeFirst()

        val seq = bytes[1] or (bytes[2] shl 8)
        val payload = ByteArray(len) { i -> bytes[TestEnvelope.HEADER_LEN + i].toByte() }

        var check = 0
        for (i in 0 until total - 1) check = check xor bytes[i]
        val checkOk = (check and 0xFF) == bytes[total - 1]

        val patternOk = (0 until len).all { i ->
            (payload[i].toInt() and 0xFF) == (TestEnvelope.patternByte(seq, i).toInt() and 0xFF)
        }

        return RecoveredPacket(seq = seq, payload = payload, intact = checkOk && patternOk)
    }
}
