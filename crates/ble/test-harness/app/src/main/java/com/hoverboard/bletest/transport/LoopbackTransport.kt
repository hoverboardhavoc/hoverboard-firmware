package com.hoverboard.bletest.transport

/**
 * A local software byte-echo transport (`specs/ble.md`, Tier 2: "loopback, no board"). It loops every
 * sent byte straight back to the receive sink, mirroring the board-side byte-loopback firmware, so the
 * measurement plumbing (seq tracking, loss accounting, the result file, the scorer) can be proven WITHOUT
 * a radio or a board.
 *
 * It optionally models the carrier's imperfections so the loss/corruption accounting can be exercised:
 * [chunkSize] splits the echo into chunks (proving the [com.hoverboard.bletest.codec.StreamRecoverer]
 * reassembles a coalesced/split stream), and [dropEvery] drops every Nth chunk (proving loss accounting).
 * With both at their defaults (no split, no drop) it is a perfect echo.
 */
class LoopbackTransport(
    private val chunkSize: Int = 0, // 0 = echo each send as one chunk
    private val dropEvery: Int = 0, // 0 = never drop
) : Transport {
    private var sink: ((ByteArray) -> Unit)? = null
    private var chunkCount = 0
    private var connected = false

    override fun connect() {
        connected = true
    }

    override fun onReceive(sink: (ByteArray) -> Unit) {
        this.sink = sink
    }

    override fun send(bytes: ByteArray) {
        check(connected) { "send before connect" }
        val s = sink ?: return
        val size = if (chunkSize <= 0) bytes.size else chunkSize
        var off = 0
        while (off < bytes.size) {
            val end = minOf(off + size, bytes.size)
            val chunk = bytes.copyOfRange(off, end)
            off = end
            chunkCount++
            // Drop every Nth chunk to model a lossy carrier (off by default).
            if (dropEvery > 0 && chunkCount % dropEvery == 0) continue
            s(chunk)
        }
    }

    override fun close() {
        connected = false
    }
}
