package com.hoverboard.app.net.l2

/**
 * Byte-stream L2 framing, a mirror of `crates/link/src/framer.rs`. A raw stream (the BLE
 * transparent-UART bridge, the inter-board UART) has no boundaries and no integrity, so L2 supplies
 * both:
 *
 * ```text
 * [ SOF : 1 = 0x5A ][ len : 1 ][ frag-hdr : 1 ][ chunk : len-1 ][ CRC-16 : 2 ]
 * ```
 *
 * - `len` = bytes from `frag-hdr` through end of `chunk` (so `len == 1 + chunk.size`); the inner
 *   `[ frag-hdr ][ chunk ]` is the L2 frame.
 * - CRC-16/MODBUS over `SOF..chunk`, little-endian on the wire.
 */
object StreamFrame {
    /** Start-of-frame marker. */
    const val SOF = 0x5A

    /** Fixed leading bytes before the L2 frame: `SOF` then `len`. */
    const val STREAM_HEADER_LEN = 2

    /** Trailing CRC length. */
    const val STREAM_CRC_LEN = 2

    /** Largest `len` value (the field is one byte), i.e. the largest inner L2 frame. */
    const val MAX_L2_LEN = 255

    /** Largest total stream frame on the wire. */
    const val MAX_STREAM_FRAME = STREAM_HEADER_LEN + MAX_L2_LEN + STREAM_CRC_LEN

    /**
     * Wrap one L2 frame (`[ frag-hdr ][ chunk ]`) into a stream frame, appending a little-endian
     * CRC-16/MODBUS over `SOF..chunk`. Throws [IllegalArgumentException] on an empty or oversize L2
     * frame (the firmware's `FrameError::BadLen`).
     */
    fun encode(l2: ByteArray): ByteArray {
        val len = l2.size
        require(len in 1..MAX_L2_LEN) { "L2 frame length $len out of 1..$MAX_L2_LEN" }
        val out = ByteArray(STREAM_HEADER_LEN + len + STREAM_CRC_LEN)
        out[0] = SOF.toByte()
        out[1] = len.toByte()
        System.arraycopy(l2, 0, out, STREAM_HEADER_LEN, len)
        val crc = Crc16.modbus(out, 0, STREAM_HEADER_LEN + len)
        out[STREAM_HEADER_LEN + len] = (crc and 0x00FF).toByte()
        out[STREAM_HEADER_LEN + len + 1] = (crc ushr 8).toByte()
        return out
    }
}

/**
 * A resyncing stream framer over a growable buffer (a mirror of `framer.rs`'s `StreamFramer`). Eats
 * arbitrary byte chunks, resyncs on `SOF`, validates `len` + CRC, and calls `sink` once per whole
 * CRC-valid L2 frame (`[ frag-hdr ][ chunk ]`), in order. A bad CRC drops that frame and the framer
 * resyncs at the next `SOF`. Resync is an iterative bounded loop (drop one byte, re-scan), never
 * recursion.
 */
class StreamFramer {
    private val buf = ArrayList<Byte>()

    /** Reset to the hunt state, dropping any partial frame. */
    fun reset() = buf.clear()

    /** Feed an arbitrary chunk of bytes, calling `sink` once per complete CRC-valid L2 frame. */
    fun feed(bytes: ByteArray, sink: (ByteArray) -> Unit) {
        for (b in bytes) {
            buf.add(b)
            process(sink)
        }
    }

    private fun byte(i: Int): Int = buf[i].toInt() and 0xFF

    private fun discardFront(n: Int) {
        // ArrayList.subList().clear() shifts the remainder to the front in one shot.
        buf.subList(0, n).clear()
    }

    private fun process(sink: (ByteArray) -> Unit) {
        while (true) {
            // Hunt: drop leading non-SOF bytes in one shift so garbage never accumulates.
            var hunt = 0
            while (hunt < buf.size && byte(hunt) != StreamFrame.SOF) hunt++
            if (hunt > 0) discardFront(hunt)

            // Need at least SOF + len to know the target length.
            if (buf.size < StreamFrame.STREAM_HEADER_LEN) return

            // A real frame always carries a frag-hdr, so len >= 1; len == 0 is a false SOF.
            val len = byte(1)
            if (len == 0) {
                discardFront(1)
                continue
            }

            val total = StreamFrame.STREAM_HEADER_LEN + len + StreamFrame.STREAM_CRC_LEN
            if (buf.size < total) return // need more bytes for this candidate

            // A full candidate frame. Validate the CRC over SOF..chunk.
            val crcOff = StreamFrame.STREAM_HEADER_LEN + len
            val frame = ByteArray(crcOff)
            for (i in 0 until crcOff) frame[i] = buf[i]
            val crcCalc = Crc16.modbus(frame, 0, crcOff)
            val crcWire = byte(crcOff) or (byte(crcOff + 1) shl 8)
            if (crcCalc == crcWire) {
                val l2 = ByteArray(len)
                for (i in 0 until len) l2[i] = buf[StreamFrame.STREAM_HEADER_LEN + i]
                sink(l2)
                discardFront(total)
            } else {
                // CRC failure: drop just the candidate SOF and re-scan (resync past the false start).
                discardFront(1)
            }
        }
    }
}
