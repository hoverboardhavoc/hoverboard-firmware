package com.hoverboard.bletest

import com.hoverboard.bletest.codec.ConnPriority
import com.hoverboard.bletest.codec.RunConfig
import com.hoverboard.bletest.codec.ThroughputRun
import com.hoverboard.bletest.codec.WriteType
import com.hoverboard.bletest.transport.LoopbackTransport
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tier 2 (loopback, no board): drive the harness run against a software byte echo to prove the
 * measurement plumbing (`specs/ble.md`, "Tiers"): seq tracking, loss accounting, the result, and the
 * knee-detection logic the sweep relies on. No radio, no board. A virtual clock makes it deterministic.
 */
class LoopbackRunTest {

    /** A virtual clock: time only advances when [ThroughputRun] sleeps to the next send instant. */
    private class VClock {
        var now = 0L
        fun nowNanos(): Long = now
        fun sleepUntil(target: Long) { if (target > now) now = target }
    }

    private fun cfg(rate: Int, payload: Int = 16, dur: Int = 1, mode: String = "fake") = RunConfig(
        device = "OnePlus", name = "test", mode = mode, payload = payload, rate = rate, durSec = dur,
        write = WriteType.NO_RESPONSE, mtu = 247, priority = ConnPriority.HIGH, out = "t.json",
    )

    @Test
    fun perfect_echo_has_no_loss() {
        val clk = VClock()
        val run = ThroughputRun(LoopbackTransport(), clk::nowNanos, clk::sleepUntil)
        val r = run.run(cfg(rate = 100, dur = 1))
        assertTrue("should have sent some packets", r.sent > 0)
        assertEquals("a perfect echo loses nothing", 0, r.lost)
        assertEquals(r.sent, r.delivered)
        assertEquals(0, r.corrupted)
        assertEquals(0.0, r.lossFraction, 0.0001)
    }

    @Test
    fun dropped_chunks_show_as_loss() {
        val clk = VClock()
        // Drop every 3rd chunk; each packet is one chunk here (chunkSize 0), so ~1/3 are lost.
        val run = ThroughputRun(LoopbackTransport(dropEvery = 3), clk::nowNanos, clk::sleepUntil)
        val r = run.run(cfg(rate = 90, dur = 1))
        assertTrue("loss must be accounted", r.lost > 0)
        assertEquals(r.sent - r.delivered, r.lost)
        assertTrue("about a third should drop", r.lossFraction in 0.25..0.40)
    }

    @Test
    fun split_echo_still_reassembles_with_no_loss() {
        val clk = VClock()
        // Echo each send in 5-byte chunks: the StreamRecoverer must reassemble, no loss.
        val run = ThroughputRun(LoopbackTransport(chunkSize = 5), clk::nowNanos, clk::sleepUntil)
        val r = run.run(cfg(rate = 50, payload = 32, dur = 1))
        assertEquals(0, r.lost)
        assertEquals(r.sent, r.delivered)
    }

    @Test
    fun goodput_and_latency_are_derived() {
        val clk = VClock()
        val run = ThroughputRun(LoopbackTransport(), clk::nowNanos, clk::sleepUntil)
        val c = cfg(rate = 100, payload = 16, dur = 1)
        val r = run.run(c)
        assertTrue(r.latencyNs.count > 0)
        // Instant virtual echo => zero RTT, but the summary is still populated.
        assertEquals(r.delivered, r.latencyNs.count)
        assertTrue(r.goodputBytesPerSec(c.payload, c.durSec.toDouble()) > 0.0)
    }

    @Test
    fun knee_detection_finds_the_first_lossy_rate() {
        // Mirror the sweep's knee logic: ramp the offered rate and report the first rate with loss. Here
        // the fake drops every 4th chunk only above a threshold, modeled by enabling dropEvery past a rate.
        fun lossAt(rate: Int): Double {
            val clk = VClock()
            val transport = if (rate >= 80) LoopbackTransport(dropEvery = 4) else LoopbackTransport()
            val run = ThroughputRun(transport, clk::nowNanos, clk::sleepUntil)
            return run.run(cfg(rate = rate, dur = 1)).lossFraction
        }
        val rates = listOf(20, 40, 60, 80, 100)
        val knee = rates.firstOrNull { lossAt(it) > 0.01 }
        assertEquals("the knee is the first rate where loss appears", 80, knee)
    }
}
