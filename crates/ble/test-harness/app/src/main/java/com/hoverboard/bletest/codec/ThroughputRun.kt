package com.hoverboard.bletest.codec

import com.hoverboard.bletest.transport.Transport

/**
 * Drives ONE fixed-config throughput run over any [Transport] (the real BLE central on Tier 3, the local
 * fake echo on Tier 2). It is plain Kotlin with no Android dependency, so the same code that runs on a
 * phone is exercised by the JVM Tier-1/Tier-2 tests.
 *
 * The app is the active party and the oracle: it sends test-envelope packets with increasing `seq` at the
 * offered rate, and because the board echoes the raw bytes verbatim, the [RunScorer] knows exactly what
 * should come back and derives delivered/loss/latency/goodput/corruption.
 *
 * Timing is injected ([nowNanos] / [sleepUntil]) so a test can drive it deterministically with a virtual
 * clock; on the phone these are `System.nanoTime` and a real sleep.
 */
class ThroughputRun(
    private val transport: Transport,
    private val nowNanos: () -> Long,
    private val sleepUntil: (Long) -> Unit,
) {
    fun run(config: RunConfig): RunResult {
        val scorer = RunScorer()
        transport.onReceive { chunk -> scorer.onReceived(chunk, nowNanos()) }
        transport.connect()

        val gapNs = if (config.rate <= 0) 0L else 1_000_000_000L / config.rate
        val start = nowNanos()
        val deadline = start + config.durSec.toLong() * 1_000_000_000L
        var seq = 0
        var next = start
        while (nowNanos() < deadline) {
            val pkt = TestEnvelope.encodePattern(seq, config.payload)
            val tx = nowNanos()
            scorer.onSent(seq, tx)
            transport.send(pkt)
            seq++
            next += gapNs
            if (gapNs > 0) sleepUntil(next)
        }
        // A short drain window so in-flight echoes are scored before the result is taken.
        transport.close()
        return scorer.result()
    }
}
