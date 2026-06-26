package com.hoverboard.bletest

import com.hoverboard.bletest.codec.RecoveredPacket
import com.hoverboard.bletest.codec.StreamRecoverer
import com.hoverboard.bletest.codec.TestEnvelope
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tier 1 (host, no hardware) unit tests of the harness's **test-envelope** codec (`specs/ble.md`,
 * "Tiers"): build/parse a packet, recover packets from a split/coalesced byte stream, detect a corrupted
 * pattern. Proves the measurement layer before any radio. This is the harness's OWN envelope, not the
 * `link` codec. Runnable on any JVM with `./gradlew test` (no Android device).
 */
class TestEnvelopeTest {

    @Test
    fun encode_then_recover_roundtrips() {
        val pkt = TestEnvelope.encodePattern(seq = 7, len = 16)
        // Structural: marker, seq LE, len, and the 5-byte overhead.
        assertEquals(TestEnvelope.MARKER, pkt[0].toInt() and 0xFF)
        assertEquals(7, (pkt[1].toInt() and 0xFF) or ((pkt[2].toInt() and 0xFF) shl 8))
        assertEquals(16, pkt[3].toInt() and 0xFF)
        assertEquals(TestEnvelope.OVERHEAD + 16, pkt.size)

        val rec = StreamRecoverer().push(pkt)
        assertEquals(1, rec.size)
        assertEquals(7, rec[0].seq)
        assertTrue("a clean packet must be intact", rec[0].intact)
        assertEquals(16, rec[0].payload.size)
    }

    @Test
    fun recovers_packets_from_a_split_stream() {
        // Two packets concatenated, then sliced at awkward boundaries (a chunk boundary mid-packet).
        val a = TestEnvelope.encodePattern(seq = 1, len = 8)
        val b = TestEnvelope.encodePattern(seq = 2, len = 4)
        val whole = a + b

        val r = StreamRecoverer()
        val got = ArrayList<RecoveredPacket>()
        // Feed 3 bytes at a time, deliberately splitting packets across chunks.
        var off = 0
        while (off < whole.size) {
            val end = minOf(off + 3, whole.size)
            got.addAll(r.push(whole.copyOfRange(off, end)))
            off = end
        }
        assertEquals(2, got.size)
        assertEquals(1, got[0].seq); assertTrue(got[0].intact)
        assertEquals(2, got[1].seq); assertTrue(got[1].intact)
    }

    @Test
    fun recovers_packets_from_a_coalesced_stream() {
        // Many packets delivered as ONE big chunk (coalesced notifications).
        val whole = (0 until 5).fold(ByteArray(0)) { acc, s ->
            acc + TestEnvelope.encodePattern(seq = s, len = 32)
        }
        val got = StreamRecoverer().push(whole)
        assertEquals(5, got.size)
        got.forEachIndexed { i, p -> assertEquals(i, p.seq); assertTrue(p.intact) }
    }

    @Test
    fun detects_a_corrupted_pattern() {
        val pkt = TestEnvelope.encodePattern(seq = 9, len = 16)
        // Flip a payload byte (offset 6 = inside the payload), leave the self-check stale -> caught.
        pkt[6] = (pkt[6].toInt() xor 0xFF).toByte()
        val got = StreamRecoverer().push(pkt)
        assertEquals(1, got.size)
        assertEquals(9, got[0].seq)
        assertFalse("a corrupted payload must NOT be reported intact", got[0].intact)
    }

    @Test
    fun resyncs_past_leading_garbage() {
        val pkt = TestEnvelope.encodePattern(seq = 3, len = 8)
        // Garbage bytes (none equal to the marker) prepended; the recoverer must resync to the marker.
        val garbage = byteArrayOf(0x00, 0x11, 0x5A, 0x22) // includes 0x5A (link SOF), still not our marker
        val r = StreamRecoverer()
        val got = r.push(garbage + pkt)
        assertEquals(1, got.size)
        assertEquals(3, got[0].seq)
        assertTrue(got[0].intact)
        assertTrue("garbage before the marker must register as resyncs", r.resyncs >= garbage.size)
    }

    @Test
    fun empty_and_zero_length_payloads_roundtrip() {
        val pkt = TestEnvelope.encodePattern(seq = 0, len = 0)
        assertEquals(TestEnvelope.OVERHEAD, pkt.size)
        val got = StreamRecoverer().push(pkt)
        assertEquals(1, got.size)
        assertEquals(0, got[0].seq)
        assertTrue(got[0].intact)
        assertEquals(0, got[0].payload.size)
    }
}
