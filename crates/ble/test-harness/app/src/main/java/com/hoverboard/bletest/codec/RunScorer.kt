package com.hoverboard.bletest.codec

/**
 * The host-oracle scorer (`specs/ble.md`, "Measurement model (host-oracle)"). The app sends test-envelope
 * packets with increasing `seq`; the board echoes the raw bytes verbatim, so the app knows exactly what
 * should come back. This class tracks what was sent, ingests the echoed byte stream via a
 * [StreamRecoverer], and derives the per-run metrics.
 *
 * Round-trip loss is the core metric (`specs/ble.md`): it answers "how much can we reliably send back and
 * forth." Per-direction attribution is out of scope for the raw loopback (a raw loopback cannot, on its
 * own, say which direction dropped) and is deferred per the spec's open questions.
 */
class RunScorer {
    private val recoverer = StreamRecoverer()

    /** seq -> the wall-clock time (ns) the packet was sent. */
    private val sentAt = LinkedHashMap<Int, Long>()
    private val deliveredSeqs = HashSet<Int>()
    private val rttsNs = ArrayList<Long>()

    var corrupted: Int = 0
        private set

    /** Record that packet [seq] was sent at [txNanos]. */
    fun onSent(seq: Int, txNanos: Long) {
        sentAt[seq] = txNanos
    }

    /** Ingest a chunk of echoed bytes received at [rxNanos]; score every packet it completes. */
    fun onReceived(chunk: ByteArray, rxNanos: Long) {
        for (pkt in recoverer.push(chunk)) {
            val tx = sentAt[pkt.seq] ?: continue // an echo for a seq we never sent: ignore
            if (deliveredSeqs.add(pkt.seq)) {
                rttsNs.add(rxNanos - tx)
            }
            if (!pkt.intact) corrupted++
        }
    }

    /** Compute the run result over the packets sent so far. */
    fun result(): RunResult {
        val sent = sentAt.size
        val delivered = deliveredSeqs.size
        val missing = sentAt.keys.filter { it !in deliveredSeqs }.sorted()
        return RunResult(
            sent = sent,
            delivered = delivered,
            lost = missing.size,
            missingSeqs = missing,
            corrupted = corrupted,
            resyncs = recoverer.resyncs,
            latencyNs = Latency.of(rttsNs),
        )
    }
}

/** Per-`seq` round-trip latency summary (ns): min/mean/p50/p95/max. */
data class Latency(
    val count: Int,
    val minNs: Long,
    val meanNs: Long,
    val p50Ns: Long,
    val p95Ns: Long,
    val maxNs: Long,
) {
    companion object {
        fun of(samples: List<Long>): Latency {
            if (samples.isEmpty()) return Latency(0, 0, 0, 0, 0, 0)
            val s = samples.sorted()
            fun pct(p: Double): Long = s[((p * (s.size - 1)).toInt()).coerceIn(0, s.size - 1)]
            return Latency(
                count = s.size,
                minNs = s.first(),
                meanNs = s.sum() / s.size,
                p50Ns = pct(0.50),
                p95Ns = pct(0.95),
                maxNs = s.last(),
            )
        }
    }
}

/** The result of one fixed-config run, the unit a sweep collects. */
data class RunResult(
    val sent: Int,
    val delivered: Int,
    val lost: Int,
    val missingSeqs: List<Int>,
    val corrupted: Int,
    val resyncs: Int,
    val latencyNs: Latency,
) {
    /** Round-trip loss fraction in [0,1]; 0 when nothing was sent. */
    val lossFraction: Double get() = if (sent == 0) 0.0 else lost.toDouble() / sent

    /** Goodput in bytes/sec round-tripped, given the [payloadLen] each packet carried and the [durSec]. */
    fun goodputBytesPerSec(payloadLen: Int, durSec: Double): Double =
        if (durSec <= 0.0) 0.0 else (delivered.toLong() * payloadLen) / durSec
}
