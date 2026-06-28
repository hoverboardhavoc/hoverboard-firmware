package com.hoverboard.app.net.l2

/** Why [fragment] failed (mirror of `reasm.rs`'s `FragError`). */
sealed class FragException(message: String) : Exception(message) {
    /** The packet needs more than [FragHdr.MAX_FRAGMENTS] fragments at this link's chunk capacity. */
    object PacketTooLarge : FragException("packet exceeds ${FragHdr.MAX_FRAGMENTS} fragments")

    /** `chunkCap` was 0 (a frame capacity of <= 1 leaves no room for a chunk). */
    object ZeroChunkCap : FragException("chunk capacity is 0")
}

/**
 * Split `packet` into fragments at `chunkCap` bytes per chunk, calling `emit(fragHdr, chunk)` once
 * per fragment in order (a mirror of `reasm.rs::fragment`). All fragments carry `pid`; `FRAG_IDX`
 * runs 0..; `MORE` is set on all but the last. An empty packet yields exactly one fragment with an
 * empty chunk (`MORE=0, FRAG_IDX=0`), preserving the one-byte-overhead single-frame case.
 */
fun fragment(packet: ByteArray, chunkCap: Int, pid: Int, emit: (FragHdr, ByteArray) -> Unit) {
    if (chunkCap == 0) throw FragException.ZeroChunkCap
    val p = pid and FragHdr.MAX_PID

    if (packet.isEmpty()) {
        emit(FragHdr(more = false, pid = p, fragIdx = 0), ByteArray(0))
        return
    }

    val nFrags = (packet.size + chunkCap - 1) / chunkCap // div_ceil
    if (nFrags > FragHdr.MAX_FRAGMENTS) throw FragException.PacketTooLarge
    var i = 0
    var off = 0
    while (off < packet.size) {
        val end = minOf(off + chunkCap, packet.size)
        val chunk = packet.copyOfRange(off, end)
        val more = i + 1 < nFrags
        emit(FragHdr(more = more, pid = p, fragIdx = i), chunk)
        i++
        off = end
    }
}

/**
 * Reassembles fragments into whole packets under the atomic-or-discard rule (a mirror of `reasm.rs`'s
 * `Reassembler`). All fragments of a packet share one `PID`; `FRAG_IDX` runs 0,1,2,...; `MORE` is set
 * on every fragment except the last. A packet is delivered only when every fragment of the same
 * packet has arrived; a torn set (a dropped fragment, a `FRAG_IDX` skip, or interleaved packets) is
 * discarded whole.
 */
class Reassembler {
    private var active = false
    private var pid = 0
    private var nextIdx = 0
    private val buf = ArrayList<Byte>()

    /** Drop any set in progress and return to idle. */
    fun reset() {
        active = false
        nextIdx = 0
        buf.clear()
    }

    private fun take(): ByteArray {
        val out = ByteArray(buf.size)
        for (i in buf.indices) out[i] = buf[i]
        return out
    }

    /**
     * Feed one L2 frame's `frag-hdr` byte and its chunk. Returns the completed packet exactly when
     * this fragment completes a packet, otherwise null (more fragments expected, or the frame was
     * discarded as torn/stray).
     */
    fun push(hdrByte: Int, chunk: ByteArray): ByteArray? {
        val h = FragHdr.decode(hdrByte)

        if (h.fragIdx == 0) {
            // FRAG_IDX 0 always starts a fresh set, discarding any set in progress.
            buf.clear()
            for (b in chunk) buf.add(b)
            pid = h.pid
            if (h.more) {
                active = true
                nextIdx = 1
                return null
            }
            // Single-fragment packet: complete immediately.
            active = false
            nextIdx = 0
            return take()
        }

        // FRAG_IDX > 0: a continuation. Without an active set it is stray; drop it.
        if (!active) return null
        // A different PID, or a skipped index, means the set in progress is torn: discard it whole.
        if (h.pid != pid || h.fragIdx != nextIdx) {
            reset()
            return null
        }
        for (b in chunk) buf.add(b)
        nextIdx++
        return if (h.more) {
            null
        } else {
            active = false
            nextIdx = 0
            take()
        }
    }
}
