package com.hoverboard.bletest.transport

import android.util.Log

/**
 * L2 (`specs/l2.md`) framing over any raw [Transport], for the Tier-3 BLE validation. This makes the
 * phone an L2 peer so the master <-> Android path carries fragmented packets, not the raw throughput
 * envelope alone.
 *
 * Send: fragment a packet into `<=20`-byte `[ len ][ frag-hdr ][ chunk ]` frames and write each as one
 * ATT write. The **in-band length delimiter** (`len` = the frag-hdr..chunk byte count) is required: the
 * CC2541 **coalesces** back-to-back forwarded writes across the bridge (measured Tier-3: the master saw
 * bursts up to 64 B merging four 20 B frames), so the ATT/idle boundary alone does NOT delimit frames -
 * the length does.
 *
 * Receive: a notification may carry one frame or several coalesced ones, so it is **split on the length
 * byte** and each `[ frag-hdr ][ chunk ]` frame is pushed to a reassembler mirroring `link::Reassembler`
 * (atomic-or-discard, PID/FRAG_IDX torn detection); a completed packet is handed upstream to the scorer.
 */
class L2FragmentedTransport(private val inner: Transport) : Transport {
    private var txPid = 0
    private val reasm = L2Reassembler()
    private val rxStream = java.io.ByteArrayOutputStream()
    private var notifyCount = 0
    private var oversize = 0
    private var framesParsed = 0

    override fun connect() = inner.connect()

    override fun close() {
        Log.i(TAG, "l2 summary notifications=$notifyCount framesParsed=$framesParsed oversize(>20B)=$oversize")
        inner.close()
    }

    override fun send(bytes: ByteArray) {
        val frames = fragment(bytes, txPid)
        txPid = (txPid + 1) and 0x7
        for (f in frames) inner.send(f)
    }

    override fun onReceive(sink: (ByteArray) -> Unit) {
        inner.onReceive { notif ->
            notifyCount++
            if (notif.size > 20) oversize++
            val head = notif.take(4).joinToString(" ") { (it.toInt() and 0xFF).toString(16) }
            Log.d(TAG, "notify #$notifyCount len=${notif.size} head=[$head]")
            // The CC2541 bridge re-chunks the master's UART output into arbitrary notifications that
            // SPLIT frames (measured Tier-3: a 20 B frame arrived as 18 B + 2 B across two notifies). So
            // the notification stream is treated as a continuous BYTE STREAM: append, then peel off each
            // complete [ len ][ frag-hdr ][ chunk ] frame, leaving any partial frame buffered.
            rxStream.write(notif)
            val buf = rxStream.toByteArray()
            var pos = 0
            while (pos < buf.size) {
                val flen = buf[pos].toInt() and 0xFF // frag-hdr + chunk
                if (flen == 0 || flen > 19) {
                    pos++ // desync (lost byte): drop one and re-scan
                    continue
                }
                if (pos + 1 + flen > buf.size) break // partial frame: wait for more
                val frame = buf.copyOfRange(pos + 1, pos + 1 + flen)
                framesParsed++
                reasm.push(frame)?.let { sink(it) }
                pos += 1 + flen
            }
            // Retain only the unconsumed tail.
            rxStream.reset()
            if (pos < buf.size) rxStream.write(buf, pos, buf.size - pos)
        }
    }

    companion object {
        const val TAG = "BLE_TPUT"

        /** Usable chunk per BLE frame: 20-byte ATT payload minus 1 len byte and 1 frag-hdr byte. */
        private const val CHUNK_CAP = 18

        /** frag-hdr: bit7 MORE, bits6..4 PID (0..7), bits3..0 FRAG_IDX (0..15). */
        private fun fragHdr(more: Boolean, pid: Int, idx: Int): Byte =
            (((if (more) 0x80 else 0) or ((pid and 0x7) shl 4) or (idx and 0xF)).toByte())

        /**
         * Fragment `packet` into `[ len ][ frag-hdr ][ chunk ]` frames sharing `pid`; `len` =
         * frag-hdr..chunk. Empty packet -> one frame carrying just the frag-hdr.
         */
        fun fragment(packet: ByteArray, pid: Int): List<ByteArray> {
            fun frame(more: Boolean, idx: Int, chunk: ByteArray, cOff: Int, cLen: Int): ByteArray {
                val f = ByteArray(2 + cLen)
                f[0] = (1 + cLen).toByte() // len = frag-hdr + chunk
                f[1] = fragHdr(more, pid, idx)
                System.arraycopy(chunk, cOff, f, 2, cLen)
                return f
            }
            if (packet.isEmpty()) return listOf(frame(false, 0, packet, 0, 0))
            val frames = ArrayList<ByteArray>()
            var off = 0
            var idx = 0
            while (off < packet.size) {
                val end = minOf(off + CHUNK_CAP, packet.size)
                frames.add(frame(end < packet.size, idx, packet, off, end - off))
                off = end
                idx++
            }
            return frames
        }
    }
}

/**
 * Faithful mirror of `link::Reassembler` (`crates/link/src/reasm.rs`): one frame per [push],
 * atomic-or-discard, FRAG_IDX must run 0,1,2,... within one PID, a PID change or index skip discards the
 * set in progress. FRAG_IDX 0 always starts a fresh set. The frame here is `[ frag-hdr ][ chunk ]` (the
 * length byte was already stripped by the caller's split).
 */
private class L2Reassembler {
    private var active = false
    private var pid = 0
    private var nextIdx = 0
    private val buf = java.io.ByteArrayOutputStream()

    /** Push one L2 frame (`[frag-hdr][chunk]`); returns the packet when it completes, else null. */
    fun push(frame: ByteArray): ByteArray? {
        if (frame.isEmpty()) return null
        val hdr = frame[0].toInt() and 0xFF
        val more = (hdr and 0x80) != 0
        val framePid = (hdr shr 4) and 0x7
        val fragIdx = hdr and 0xF
        val chunkOff = 1

        if (fragIdx == 0) {
            buf.reset()
            buf.write(frame, chunkOff, frame.size - chunkOff)
            pid = framePid
            return if (more) {
                active = true; nextIdx = 1; null
            } else {
                active = false; nextIdx = 0; buf.toByteArray()
            }
        }
        if (!active) return null
        if (framePid != pid || fragIdx != nextIdx) {
            reset(); return null
        }
        buf.write(frame, chunkOff, frame.size - chunkOff)
        nextIdx++
        return if (more) null else {
            active = false; nextIdx = 0; buf.toByteArray()
        }
    }

    private fun reset() {
        active = false; nextIdx = 0; buf.reset()
    }
}
