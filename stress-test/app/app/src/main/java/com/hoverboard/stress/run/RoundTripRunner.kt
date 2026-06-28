package com.hoverboard.stress.run

import android.util.Log
import com.hoverboard.stress.ble.BleBytePipe
import com.hoverboard.stress.ble.BleStressTransport
import com.hoverboard.stress.net.l2.BleStreamTransport
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.ClosedReceiveChannelException
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onCompletion
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.withTimeoutOrNull

/** The outcome of one round-trip run: counters + latency + connection stability. */
data class RoundTripResult(
    val sent: Int,
    val echoed: Int,
    val lost: Int,
    val rttCount: Int,
    val rttMinNs: Long,
    val rttMeanNs: Long,
    val rttMaxNs: Long,
    val durationMs: Long,
    /** True if the GATT link dropped mid-run (a fresh session generation or a closed notify stream). */
    val dropped: Boolean,
    /** Frames successfully echoed before the first drop (== [echoed] if it never dropped). */
    val framesBeforeDrop: Int,
    /** ms the link stayed up before the drop, or the whole run duration if it held. */
    val connectedMs: Long,
) {
    val lossFraction: Double get() = if (sent == 0) 0.0 else lost.toDouble() / sent
    fun throughputFps(): Double = if (durationMs == 0L) 0.0 else echoed * 1000.0 / durationMs
}

/**
 * Round-trip mode (spec "Modes"): send a numbered fixed-size frame (4-byte big-endian seq + filler),
 * `StreamFramer`-encoded to a ~20-byte ATT write, await its byte-faithful echo, verify seq + payload
 * (CRC is verified implicitly: [BleStreamTransport]'s framer only surfaces CRC-valid frames), and time
 * the round trip. Repeat N frames / a fixed duration.
 *
 * The notify stream is collected into a [Channel] so this runner is the sole owner of the (non
 * thread-safe) [BleStreamTransport] framer. A closed channel (the notify flow ending) is the drop
 * signal. The runner also watches [BleStressTransport.sessionGeneration] to catch a reconnect.
 */
class RoundTripRunner(
    private val transport: BleStressTransport,
    private val cfg: RunConfig,
) {
    private val framer = BleStreamTransport()

    suspend fun run(pipe: BleBytePipe, scope: CoroutineScope): RoundTripResult {
        val startGen = transport.sessionGeneration
        val rx = Channel<ByteArray>(Channel.UNLIMITED)
        val rxJob = pipe.incoming
            .onEach { rx.trySend(it) }
            .onCompletion { rx.close() } // notify stream ended -> link dropped
            .launchIn(scope)

        var sent = 0
        var echoed = 0
        var lost = 0
        var rttCount = 0
        var rttSum = 0L
        var rttMin = Long.MAX_VALUE
        var rttMax = 0L
        var dropped = false
        var framesBeforeDrop = 0

        val runStart = System.nanoTime()
        val perFrameTimeoutNs = PER_FRAME_TIMEOUT_MS * 1_000_000L
        val intervalNs = if (cfg.rate > 0) 1_000_000_000L / cfg.rate else 0L
        val durLimitNs = if (cfg.durSec > 0) cfg.durSec * 1_000_000_000L else Long.MAX_VALUE

        var seq = 0
        runLoop@ while (true) {
            val elapsed = System.nanoTime() - runStart
            if (cfg.durSec > 0) {
                if (elapsed >= durLimitNs) break
            } else if (seq >= cfg.n) {
                break
            }

            if (!isAlive(startGen)) {
                dropped = true
                break
            }

            val frameStart = System.nanoTime()
            val wire = encodeFrame(seq, cfg.chunk)
            try {
                pipe.write(wire)
            } catch (e: Exception) {
                Log.w(TAG, "write failed at seq=$seq (link gone): ${e.message}")
                dropped = true
                break
            }
            sent++

            when (val rtt = awaitEcho(rx, seq, frameStart, perFrameTimeoutNs, startGen)) {
                ECHO_DROPPED -> {
                    dropped = true
                    break@runLoop
                }
                ECHO_TIMEOUT -> {
                    lost++
                    Log.d(TAG, "seq=$seq timed out (no echo within ${PER_FRAME_TIMEOUT_MS}ms)")
                }
                else -> {
                    echoed++
                    if (!dropped) framesBeforeDrop = echoed
                    rttCount++
                    rttSum += rtt
                    if (rtt < rttMin) rttMin = rtt
                    if (rtt > rttMax) rttMax = rtt
                }
            }

            seq++

            if (intervalNs > 0) {
                val spent = System.nanoTime() - frameStart
                val waitNs = intervalNs - spent
                if (waitNs > 0) delay(waitNs / 1_000_000L)
            }
        }

        rxJob.cancel()

        val durationMs = (System.nanoTime() - runStart) / 1_000_000L
        val connectedMs = when {
            dropped && transport.disconnectedAtMs > 0 && transport.connectedAtMs > 0 ->
                transport.disconnectedAtMs - transport.connectedAtMs
            else -> durationMs
        }
        return RoundTripResult(
            sent = sent,
            echoed = echoed,
            lost = lost,
            rttCount = rttCount,
            rttMinNs = if (rttCount == 0) 0 else rttMin,
            rttMeanNs = if (rttCount == 0) 0 else rttSum / rttCount,
            rttMaxNs = rttMax,
            durationMs = durationMs,
            dropped = dropped,
            framesBeforeDrop = framesBeforeDrop,
            connectedMs = connectedMs,
        )
    }

    /**
     * Wait for the echo of [expectedSeq]. Returns the round-trip latency (ns), or [ECHO_TIMEOUT] /
     * [ECHO_DROPPED]. Stale lower-seq frames (a prior frame's late echo) are dropped and the wait
     * continues.
     */
    private suspend fun awaitEcho(
        rx: Channel<ByteArray>,
        expectedSeq: Int,
        startNs: Long,
        timeoutNs: Long,
        startGen: Int,
    ): Long {
        val deadline = startNs + timeoutNs
        while (true) {
            val l2 = framer.recvL2Frame()
            if (l2 != null) {
                val seq = decodeSeq(l2)
                if (seq == expectedSeq && payloadMatches(l2, expectedSeq)) {
                    return System.nanoTime() - startNs
                }
                continue // stale / mismatched frame; keep waiting for ours
            }
            if (!isAlive(startGen)) return ECHO_DROPPED
            val remainingNs = deadline - System.nanoTime()
            if (remainingNs <= 0) return ECHO_TIMEOUT
            val waitMs = (remainingNs / 1_000_000L).coerceIn(1, POLL_MS)
            val raw = try {
                withTimeoutOrNull(waitMs) { rx.receive() }
            } catch (e: ClosedReceiveChannelException) {
                return ECHO_DROPPED // notify stream closed -> link dropped
            }
            if (raw != null) framer.onReceive(raw)
        }
    }

    private fun isAlive(startGen: Int): Boolean =
        transport.sessionGeneration == startGen && transport.pipe.value != null

    /** Build the wire bytes: L2 frame = `[frag-hdr=0x00][4-byte BE seq][filler]`, SOF/len/CRC-wrapped. */
    private fun encodeFrame(seq: Int, chunkSize: Int): ByteArray {
        val l2 = ByteArray(1 + chunkSize)
        l2[0] = 0x00 // frag-hdr: single un-fragmented frame (MORE=0, PID=0, FRAG_IDX=0)
        l2[1] = (seq ushr 24).toByte()
        l2[2] = (seq ushr 16).toByte()
        l2[3] = (seq ushr 8).toByte()
        l2[4] = seq.toByte()
        for (i in 5 until l2.size) l2[i] = FILLER[(i - 5) % FILLER.size]
        framer.sendL2Frame(l2)
        return framer.drainOutgoing() ?: ByteArray(0)
    }

    /** The 4-byte big-endian seq carried in the echoed L2 frame's chunk, or -1 if too short. */
    private fun decodeSeq(l2: ByteArray): Int {
        if (l2.size < 5) return -1 // frag-hdr + 4 seq bytes
        return ((l2[1].toInt() and 0xFF) shl 24) or
            ((l2[2].toInt() and 0xFF) shl 16) or
            ((l2[3].toInt() and 0xFF) shl 8) or
            (l2[4].toInt() and 0xFF)
    }

    /** Confirm the echoed frame is byte-identical to what was sent for [expectedSeq]. */
    private fun payloadMatches(l2: ByteArray, expectedSeq: Int): Boolean {
        if (l2.size != 1 + cfg.chunk) return false
        if (l2[0].toInt() != 0x00) return false
        for (i in 5 until l2.size) {
            if (l2[i] != FILLER[(i - 5) % FILLER.size]) return false
        }
        return decodeSeq(l2) == expectedSeq
    }

    private companion object {
        const val TAG = "BLE_STRESS"

        /** Per-frame echo timeout. 9600 baud * 20 B each way ~= 40 ms wire; 1 s covers BLE jitter. */
        const val PER_FRAME_TIMEOUT_MS = 1_000L

        /** Max single wait on the notify channel before re-checking liveness/deadline. */
        const val POLL_MS = 25L

        /** Filler byte pattern after the 4-byte seq (a recognizable ramp). */
        val FILLER = byteArrayOf(
            0xA5.toByte(), 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
            0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
        )

        const val ECHO_TIMEOUT = -1L
        const val ECHO_DROPPED = -2L
    }
}
