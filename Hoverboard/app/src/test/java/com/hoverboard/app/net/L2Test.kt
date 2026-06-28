package com.hoverboard.app.net

import com.hoverboard.app.net.l2.Crc16
import com.hoverboard.app.net.l2.FragHdr
import com.hoverboard.app.net.l2.Link
import com.hoverboard.app.net.l2.Reassembler
import com.hoverboard.app.net.l2.SendException
import com.hoverboard.app.net.l2.StreamFrame
import com.hoverboard.app.net.l2.StreamFramer
import com.hoverboard.app.net.l2.Transport
import com.hoverboard.app.net.l2.fragment
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** Mirror of `crates/link`'s framer/frag/reasm/link tests: the L2 wire form must match byte-for-byte. */
class L2Test {

    private fun runFramer(vararg chunks: ByteArray): List<ByteArray> {
        val framer = StreamFramer()
        val got = ArrayList<ByteArray>()
        for (c in chunks) framer.feed(c) { got.add(it) }
        return got
    }

    @Test
    fun crc16ModbusKnownVector() {
        // CRC-16/MODBUS of "123456789" is 0x4B37 (the canonical check value).
        assertEquals(0x4B37, Crc16.modbus("123456789".toByteArray(Charsets.US_ASCII)))
    }

    @Test
    fun fragHdrBitPositions() {
        assertEquals(0x00, FragHdr(more = false, pid = 0, fragIdx = 0).encode())
        assertEquals(0b1000_0000, FragHdr(more = true, pid = 0, fragIdx = 0).encode())
        assertEquals(0b0101_0000, FragHdr(more = false, pid = 0b101, fragIdx = 0).encode())
        assertEquals(0b0000_1011, FragHdr(more = false, pid = 0, fragIdx = 0b1011).encode())
        assertEquals(0b1111_1111, FragHdr(more = true, pid = 7, fragIdx = 15).encode())
        for (b in 0..255) assertEquals(b, FragHdr.decode(b).encode(), "byte $b did not round-trip")
    }

    @Test
    fun streamFrameLayoutAndRoundTrip() {
        // L2 frame = frag-hdr 0x00 + chunk [1,2,3,4]. Stream frame = SOF len hdr chunk CRClo CRChi.
        val l2 = byteArrayOf(0x00, 1, 2, 3, 4)
        val frame = StreamFrame.encode(l2)
        assertEquals(StreamFrame.SOF, frame[0].toInt() and 0xFF)
        assertEquals(l2.size, frame[1].toInt() and 0xFF)
        val got = runFramer(frame)
        assertEquals(1, got.size)
        assertArrayEquals(l2, got[0])
    }

    @Test
    fun streamEmptyChunkIsLoneFragHdr() {
        val l2 = byteArrayOf(0x00) // empty chunk: a lone frag-hdr, len == 1
        val frame = StreamFrame.encode(l2)
        assertEquals(StreamFrame.STREAM_HEADER_LEN + 1 + StreamFrame.STREAM_CRC_LEN, frame.size)
        val got = runFramer(frame)
        assertArrayEquals(l2, got.single())
    }

    @Test
    fun badCrcDropped() {
        val frame = StreamFrame.encode(byteArrayOf(0x00, 0xAA.toByte(), 0xBB.toByte()))
        frame[StreamFrame.STREAM_HEADER_LEN + 1] = (frame[StreamFrame.STREAM_HEADER_LEN + 1].toInt() xor 0xFF).toByte()
        assertTrue(runFramer(frame).isEmpty())
    }

    @Test
    fun splitAcrossReadChunks() {
        val l2 = byteArrayOf(0x00, 9, 8, 7, 6, 5)
        val f = StreamFrame.encode(l2)
        val got = runFramer(
            f.copyOfRange(0, 1), f.copyOfRange(1, 3), f.copyOfRange(3, f.size - 1), f.copyOfRange(f.size - 1, f.size),
        )
        assertArrayEquals(l2, got.single())
    }

    @Test
    fun coalescedFrames() {
        val a = StreamFrame.encode(byteArrayOf(0x00, 1))
        val b = StreamFrame.encode(byteArrayOf(0x11, 2, 3))
        val c = StreamFrame.encode(byteArrayOf(0x22))
        val got = runFramer(a + b + c)
        assertEquals(3, got.size)
        assertArrayEquals(byteArrayOf(0x00, 1), got[0])
        assertArrayEquals(byteArrayOf(0x11, 2, 3), got[1])
        assertArrayEquals(byteArrayOf(0x22), got[2])
    }

    @Test
    fun resyncPastGarbageAndFalseSof() {
        val l2 = byteArrayOf(0x00, 0x11, 0x22)
        // Leading garbage with a stray SOF whose len=1 frame fails CRC, then the real frame.
        val stream = byteArrayOf(0x00, 0xFF.toByte(), StreamFrame.SOF.toByte(), 0x01, 0x77, 0x00, 0x00) +
            StreamFrame.encode(l2)
        assertArrayEquals(l2, runFramer(stream).single())
    }

    @Test
    fun fragmentSingleFrameOneByteOverhead() {
        val packet = byteArrayOf(1, 2, 3, 4)
        val frames = ArrayList<Pair<FragHdr, ByteArray>>()
        fragment(packet, 19, 0) { h, c -> frames.add(h to c) }
        assertEquals(1, frames.size)
        assertEquals(FragHdr(more = false, pid = 0, fragIdx = 0), frames[0].first)
        assertArrayEquals(packet, frames[0].second)
    }

    @Test
    fun fragmentMultiAndReassembleRoundTrip() {
        // 50 bytes over a 19-byte chunk -> 3 fragments (MORE,MORE,last), reassembling to the packet.
        val packet = ByteArray(50) { it.toByte() }
        val frames = ArrayList<Pair<Int, ByteArray>>()
        fragment(packet, 19, 5) { h, c -> frames.add(h.encode() to c) }
        assertEquals(3, frames.size)
        assertEquals(FragHdr(more = true, pid = 5, fragIdx = 0), FragHdr.decode(frames[0].first))
        assertEquals(FragHdr(more = true, pid = 5, fragIdx = 1), FragHdr.decode(frames[1].first))
        assertEquals(FragHdr(more = false, pid = 5, fragIdx = 2), FragHdr.decode(frames[2].first))
        val r = Reassembler()
        var out: ByteArray? = null
        for ((hdr, chunk) in frames) r.push(hdr, chunk)?.let { out = it }
        assertArrayEquals(packet, out)
    }

    @Test
    fun reassemblerDiscardsTornSet() {
        // frag 0 (MORE) then frag 2 (skip past 1): discarded, nothing delivered.
        val r = Reassembler()
        assertNull(r.push(FragHdr(more = true, pid = 2, fragIdx = 0).encode(), byteArrayOf(1, 2, 3)))
        assertNull(r.push(FragHdr(more = false, pid = 2, fragIdx = 2).encode(), byteArrayOf(7, 8, 9)))
    }

    @Test
    fun reassemblerResetDropsPartialSet() {
        val r = Reassembler()
        assertNull(r.push(FragHdr(more = true, pid = 1, fragIdx = 0).encode(), byteArrayOf(0xAA.toByte())))
        r.reset() // drop the in-progress set
        // A fresh single-fragment packet now completes cleanly (the old partial is gone).
        assertArrayEquals(
            byteArrayOf(0xBB.toByte()),
            r.push(FragHdr(more = false, pid = 2, fragIdx = 0).encode(), byteArrayOf(0xBB.toByte())),
        )
    }

    @Test
    fun streamFramerResetDropsPartialFrame() {
        val framer = StreamFramer()
        val real = StreamFrame.encode(byteArrayOf(0x00, 0x77))
        val got = ArrayList<ByteArray>()
        framer.feed(real.copyOfRange(0, 3)) { got.add(it) } // half a frame
        framer.reset()
        framer.feed(real) { got.add(it) } // a whole frame after the reset
        assertEquals(1, got.size)
        assertArrayEquals(byteArrayOf(0x00, 0x77), got[0])
    }

    // --- the Link service over a mock datagram transport ---

    private class Loopback(private val cap: Int) : Transport {
        val wire = ArrayDeque<ByteArray>()
        var maxEmitted = 0
        override fun frameCapacity(): Int = cap
        override fun sendL2Frame(l2: ByteArray) {
            assertTrue(l2.size <= cap, "frame ${l2.size} > capacity $cap")
            maxEmitted = maxOf(maxEmitted, l2.size)
            wire.addLast(l2.copyOf())
        }
        override fun recvL2Frame(): ByteArray? = wire.removeFirstOrNull()
    }

    @Test
    fun linkDatagramRoundTrip() {
        val t = Loopback(20)
        val link = Link(t)
        val packet = byteArrayOf(1, 2, 3, 4, 5)
        link.send(packet)
        assertArrayEquals(packet, link.pollRecv())
        assertEquals(6, t.maxEmitted) // 5-byte packet -> 6-byte frame (one byte of overhead)
    }

    @Test
    fun linkNeverEmitsOverCapacityAndReassembles() {
        val t = Loopback(20)
        val link = Link(t)
        val packet = ByteArray(16 * 19) { it.toByte() } // the BLE max: 304 B
        link.send(packet)
        assertTrue(t.maxEmitted <= 20, "emitted ${t.maxEmitted}")
        assertArrayEquals(packet, link.pollRecv())
    }

    @Test
    fun linkOversizePacketRejected() {
        val link = Link(Loopback(20))
        assertThrows(SendException::class.java) { link.send(ByteArray(16 * 19 + 1)) }
    }

    @Test
    fun mtuHintReflectsCapacity() {
        assertEquals(16 * 19, Link(Loopback(20)).mtuHint())
        assertEquals(16 * 254, Link(Loopback(255)).mtuHint())
    }
}
