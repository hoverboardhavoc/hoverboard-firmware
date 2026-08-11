package com.hoverboard.remote.link

/**
 * Resyncing stream framer. Mirrors crates/link/src/framer.rs exactly.
 *
 * Consumes arbitrary split / coalesced byte chunks (the BLE case: notifications chop and merge
 * frames) and emits zero or more complete, CRC-valid frames. There is no idle-line assumption.
 *
 * Resync rule: on a CRC failure or any structural error, the whole buffer is NOT discarded. The
 * candidate SOF at buf[0] is dropped and the remaining buffered bytes are replayed through the hunt
 * logic, so a false 0x5A inside garbage still resyncs to the next real frame.
 */
class StreamFramer {

    // Bytes accumulated since the current candidate SOF. buf[0] is always the candidate SOF once
    // non-empty. Bounded at MAX_FRAME, like the heapless Vec on the Rust side.
    private val buf = ArrayList<Byte>(LinkConstants.MAX_FRAME)

    /** Reset to the hunt state, dropping any partial frame. */
    fun reset() {
        buf.clear()
    }

    /** Feed an arbitrary chunk, calling [onFrame] once per complete CRC-valid frame, in order. */
    fun feed(chunk: ByteArray, onFrame: (DecodedFrame) -> Unit) {
        for (b in chunk) {
            pushByte(b, onFrame)
        }
    }

    private fun pushByte(b: Byte, onFrame: (DecodedFrame) -> Unit) {
        // Hunting: buffer empty. Drop any non-SOF byte; on SOF start a candidate.
        if (buf.isEmpty()) {
            if (b != LinkConstants.SOF) return
            buf.add(b)
            return
        }

        // Non-empty: buf[0] is the candidate SOF. Append the new byte. The buffer never overflows
        // because once HEADER_LEN bytes are present the exact target length is known and the
        // candidate is processed/resynced as soon as it is reached; the only unbounded case (len
        // byte not yet seen) is capped at HEADER_LEN bytes.
        if (buf.size >= LinkConstants.MAX_FRAME) {
            // Defensive: should be unreachable given length-driven processing below.
            resyncAfterSof(onFrame)
            return
        }
        buf.add(b)

        // Validate the version byte (offset 1) the instant it arrives: a wrong version means this
        // is not a real frame start, so resync immediately rather than waiting for a full length.
        if (buf.size == 2 && buf[1] != LinkConstants.PROTO_VER) {
            resyncAfterSof(onFrame)
            return
        }

        // Until the full header is present the target length is unknown.
        if (buf.size < LinkConstants.HEADER_LEN) return

        // Header present: target total = HEADER_LEN + len + CRC_LEN.
        val len = buf[5].toInt() and 0xFF
        val total = LinkConstants.HEADER_LEN + len + LinkConstants.CRC_LEN
        if (buf.size < total) return // need more bytes

        // A full candidate frame's worth of bytes. Decode exactly `total` bytes (buf.size == total
        // here, since we process the instant the target is reached).
        val candidate = ByteArray(total) { buf[it] }
        val decoded = Frame.decode(candidate)
        if (decoded != null) {
            onFrame(decoded)
            // Consume the whole frame; keep hunting (coalesced frames re-enter byte by byte).
            buf.clear()
        } else {
            // CRC or structural failure on a full-length candidate. Resync from the byte after the
            // presumed SOF so a false 0x5A still locks onto the next real frame.
            resyncAfterSof(onFrame)
        }
    }

    /**
     * Drop the candidate SOF at buf[0] and replay the remaining buffered bytes through the hunt
     * logic, so an embedded real SOF is found instead of discarding the whole buffer.
     */
    private fun resyncAfterSof(onFrame: (DecodedFrame) -> Unit) {
        // Copy everything after the candidate SOF, clear, then replay.
        val tail = ByteArray(buf.size - 1) { buf[it + 1] }
        buf.clear()
        for (t in tail) {
            pushByte(t, onFrame)
        }
    }
}
