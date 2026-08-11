package com.hoverboard.remote.link

import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Pure-JVM codec tests for the com.hoverboard.remote.link package. No Android dependencies.
 *
 * Covers: the three CRC-16/MODBUS anchors; payload round-trips; Frame encode/decode (incl. empty
 * payload, bad SOF / version / length / CRC); and the StreamFramer cases ported from
 * crates/link/src/framer.rs (split, coalesced, resync past garbage, false SOF, trailing tolerance).
 */
class LinkCodecTest {

    // ---- CRC anchors ----

    @Test
    fun `crc modbus anchors`() {
        assertEquals(0x4B37, Crc16Modbus.compute("123456789".toByteArray()))
        assertEquals(0xBB2A, Crc16Modbus.compute(byteArrayOf(1, 2, 3, 4, 5)))
        assertEquals(0xFFFF, Crc16Modbus.compute(ByteArray(0)))
    }

    @Test
    fun `crc honors offset and length`() {
        // CRC of bytes [1,2,3,4,5] embedded in a larger buffer with padding on both ends.
        val padded = byteArrayOf(0x77, 0x77, 1, 2, 3, 4, 5, 0x33)
        assertEquals(0xBB2A, Crc16Modbus.compute(padded, 2, 5))
    }

    // ---- Opcode ----

    @Test
    fun `opcode resolves known and rejects unknown`() {
        assertEquals(Opcode.Inputs, Opcode.fromByte(0x50))
        assertEquals(Opcode.Telemetry, Opcode.fromByte(0x20))
        assertNull(Opcode.fromByte(0x7E))
        assertNull(Opcode.fromByte(0xFF))
    }

    // ---- Payload round-trips ----

    @Test
    fun `inputs round trip incl negative throttle`() {
        val v = Inputs(throttle = -1000, buttons = 0x0F, rider = 1)
        val enc = v.encode()
        assertEquals(Inputs.LEN, enc.size)
        assertEquals(v, Inputs.decode(enc))
    }

    @Test
    fun `drive cmd round trip`() {
        val v = DriveCmd(kind = 2, value = -300, steer = 75)
        val enc = v.encode()
        assertEquals(DriveCmd.LEN, enc.size)
        assertEquals(v, DriveCmd.decode(enc))
    }

    @Test
    fun `telemetry round trip`() {
        val v = Telemetry(motorIndex = 1, batteryMv = 36000, currentCa = -250, speed = 4096, faultCode = 0, flags = 0x80)
        val enc = v.encode()
        assertEquals(Telemetry.LEN, enc.size)
        assertEquals(v, Telemetry.decode(enc))
    }

    @Test
    fun `telemetry ignores trailing bytes`() {
        val v = Telemetry(motorIndex = 0, batteryMv = 100, currentCa = 1, speed = 2, faultCode = 0, flags = 0)
        val enc = v.encode() + byteArrayOf(0xAA.toByte(), 0xBB.toByte(), 0xCC.toByte())
        assertEquals(v, Telemetry.decode(enc))
    }

    @Test
    fun `config read round trip`() {
        val v = ConfigRead(key = 0xBEEF)
        assertEquals(v, ConfigRead.decode(v.encode()))
    }

    @Test
    fun `config write round trip with variable tail`() {
        val v = ConfigWrite(key = 0x1234, value = byteArrayOf(0xDE.toByte(), 0xAD.toByte(), 0xBE.toByte()))
        val enc = v.encode()
        // key LE then value
        assertEquals(0x34.toByte(), enc[0])
        assertEquals(0x12.toByte(), enc[1])
        val back = ConfigWrite.decode(enc)!!
        assertEquals(0x1234, back.key)
        assertArrayEquals(byteArrayOf(0xDE.toByte(), 0xAD.toByte(), 0xBE.toByte()), back.value)
    }

    @Test
    fun `config resp round trip with variable tail`() {
        val bytes = byteArrayOf(0x34, 0x12, 0x00, 0x01, 0x02) // key=0x1234, status=0, value=[01 02]
        val r = ConfigResp.decode(bytes)!!
        assertEquals(0x1234, r.key)
        assertEquals(0, r.status)
        assertArrayEquals(byteArrayOf(0x01, 0x02), r.value)
    }

    @Test
    fun `fault round trip`() {
        val v = Fault(faultCode = 5, action = 1)
        assertEquals(v, Fault.decode(v.encode()))
    }

    @Test
    fun `cyclic state decode only`() {
        // pitch=-1234, roll=567, wheelSpeed=-8, status=0b1011
        val v = CyclicState(pitch = -1234, roll = 567, wheelSpeed = -8, status = 0b1011)
        val bytes = ByteArray(CyclicState.LEN)
        bytes[0] = (-1234 and 0xFF).toByte(); bytes[1] = ((-1234 shr 8) and 0xFF).toByte()
        bytes[2] = (567 and 0xFF).toByte(); bytes[3] = ((567 shr 8) and 0xFF).toByte()
        bytes[4] = (-8 and 0xFF).toByte(); bytes[5] = ((-8 shr 8) and 0xFF).toByte()
        bytes[6] = 0b1011
        assertEquals(v, CyclicState.decode(bytes))
    }

    @Test
    fun `short payloads rejected`() {
        assertNull(Inputs.decode(ByteArray(3)))
        assertNull(DriveCmd.decode(ByteArray(4)))
        assertNull(Telemetry.decode(ByteArray(8)))
        assertNull(CyclicState.decode(byteArrayOf(0, 1, 2)))
        assertNull(ConfigWrite.decode(byteArrayOf(0)))
    }

    // ---- Frame encode/decode ----

    @Test
    fun `frame round trip with payload`() {
        val payload = byteArrayOf(0xDE.toByte(), 0xAD.toByte(), 0xBE.toByte(), 0xEF.toByte())
        val frame = Frame.encode(Opcode.Inputs, src = 0x02, dst = 0x03, payload = payload)
        assertEquals(LinkConstants.HEADER_LEN + payload.size + LinkConstants.CRC_LEN, frame.size)
        val d = Frame.decode(frame)!!
        assertEquals(0x50, d.header.opcode)
        assertEquals(0x02, d.header.src)
        assertEquals(0x03, d.header.dst)
        assertArrayEquals(payload, d.payload)
    }

    @Test
    fun `frame round trip empty payload`() {
        val frame = Frame.encode(Opcode.Fault, src = 1, dst = 2, payload = ByteArray(0))
        assertEquals(LinkConstants.HEADER_LEN + LinkConstants.CRC_LEN, frame.size)
        val d = Frame.decode(frame)!!
        assertEquals(0, d.header.len)
        assertEquals(0, d.payload.size)
    }

    @Test
    fun `decode rejects bad sof`() {
        val frame = Frame.encode(Opcode.DriveCmd, 1, 2, byteArrayOf(1, 2))
        frame[0] = 0x00
        assertNull(Frame.decode(frame))
    }

    @Test
    fun `decode rejects bad version`() {
        val frame = Frame.encode(Opcode.DriveCmd, 1, 2, byteArrayOf(1, 2))
        frame[1] = 0x09
        assertNull(Frame.decode(frame))
    }

    @Test
    fun `decode rejects too short and len mismatch`() {
        assertNull(Frame.decode(byteArrayOf(LinkConstants.SOF, LinkConstants.PROTO_VER, 0x10)))
        // header says len=5 but not enough bytes follow
        assertNull(Frame.decode(byteArrayOf(LinkConstants.SOF, LinkConstants.PROTO_VER, 0x10, 0, 0, 5, 0, 0)))
    }

    @Test
    fun `decode rejects bad crc`() {
        val frame = Frame.encode(Opcode.DriveCmd, 1, 2, byteArrayOf(1, 2, 3))
        frame[LinkConstants.HEADER_LEN] = (frame[LinkConstants.HEADER_LEN].toInt() xor 0xFF).toByte()
        assertNull(Frame.decode(frame))
    }

    @Test
    fun `unknown opcode still decodes structurally`() {
        // Build a frame with opcode 0x7E by hand, fix CRC with the codec helper.
        val payload = byteArrayOf(9, 9)
        val total = LinkConstants.HEADER_LEN + payload.size + LinkConstants.CRC_LEN
        val frame = ByteArray(total)
        frame[0] = LinkConstants.SOF
        frame[1] = LinkConstants.PROTO_VER
        frame[2] = 0x7E
        frame[3] = 1
        frame[4] = 2
        frame[5] = payload.size.toByte()
        payload.copyInto(frame, LinkConstants.HEADER_LEN)
        val crc = Crc16Modbus.compute(frame, 0, LinkConstants.HEADER_LEN + payload.size)
        frame[LinkConstants.HEADER_LEN + payload.size] = (crc and 0xFF).toByte()
        frame[LinkConstants.HEADER_LEN + payload.size + 1] = ((crc ushr 8) and 0xFF).toByte()

        val d = Frame.decode(frame)!!
        assertEquals(0x7E, d.header.opcode)
        assertNull(Opcode.fromByte(d.header.opcode)) // unknown to this build, but framed fine
        assertArrayEquals(payload, d.payload)
    }

    // ---- StreamFramer (ported from framer.rs tests) ----

    private fun makeFrame(opcode: Opcode, payload: ByteArray, src: Int, dst: Int): ByteArray =
        Frame.encode(opcode, src, dst, payload)

    /** Run the framer over the given chunks, collecting (opcode, payload) pairs in order. */
    private fun run(vararg chunks: ByteArray): List<Pair<Int, ByteArray>> {
        val framer = StreamFramer()
        val got = mutableListOf<Pair<Int, ByteArray>>()
        for (c in chunks) {
            framer.feed(c) { f -> got.add(f.header.opcode to f.payload) }
        }
        return got
    }

    @Test
    fun `framer single whole frame`() {
        val f = makeFrame(Opcode.CyclicState, byteArrayOf(1, 2, 3, 4), 1, 2)
        val got = run(f)
        assertEquals(1, got.size)
        assertEquals(Opcode.CyclicState.value, got[0].first)
        assertArrayEquals(byteArrayOf(1, 2, 3, 4), got[0].second)
    }

    @Test
    fun `framer split in two inside header`() {
        val f = makeFrame(Opcode.DriveCmd, byteArrayOf(9, 8, 7), 3, 4)
        val got = run(f.copyOfRange(0, 3), f.copyOfRange(3, f.size))
        assertEquals(1, got.size)
        assertArrayEquals(byteArrayOf(9, 8, 7), got[0].second)
    }

    @Test
    fun `framer split in three incl crc`() {
        val f = makeFrame(Opcode.Telemetry, byteArrayOf(0xAA.toByte(), 0xBB.toByte()), 5, 6)
        // split inside header (1 byte), inside payload, and inside the CRC (last byte alone)
        val got = run(f.copyOfRange(0, 1), f.copyOfRange(1, f.size - 1), f.copyOfRange(f.size - 1, f.size))
        assertEquals(1, got.size)
        assertArrayEquals(byteArrayOf(0xAA.toByte(), 0xBB.toByte()), got[0].second)
    }

    @Test
    fun `framer coalesced two`() {
        val a = makeFrame(Opcode.CyclicState, byteArrayOf(1), 1, 2)
        val b = makeFrame(Opcode.DriveCmd, byteArrayOf(2, 3), 3, 4)
        val got = run(a + b)
        assertEquals(2, got.size)
        assertEquals(Opcode.CyclicState.value, got[0].first)
        assertEquals(Opcode.DriveCmd.value, got[1].first)
    }

    @Test
    fun `framer coalesced three incl empty payload`() {
        val a = makeFrame(Opcode.CyclicState, ByteArray(0), 1, 2)
        val b = makeFrame(Opcode.DriveCmd, byteArrayOf(7), 3, 4)
        val c = makeFrame(Opcode.Fault, byteArrayOf(1, 1), 5, 6)
        val got = run(a + b + c)
        assertEquals(3, got.size)
        assertEquals(Opcode.CyclicState.value, got[0].first)
        assertEquals(Opcode.DriveCmd.value, got[1].first)
        assertEquals(Opcode.Fault.value, got[2].first)
    }

    @Test
    fun `framer crc fail emits nothing`() {
        val f = makeFrame(Opcode.DriveCmd, byteArrayOf(1, 2, 3), 1, 2)
        f[LinkConstants.HEADER_LEN] = (f[LinkConstants.HEADER_LEN].toInt() xor 0xFF).toByte()
        val got = run(f)
        assertTrue(got.isEmpty())
    }

    @Test
    fun `framer resync past garbage and stray sof`() {
        val f = makeFrame(Opcode.CyclicState, byteArrayOf(0x11, 0x22), 1, 2)
        // garbage including a stray 0x5A that is NOT a real frame start, then the real frame
        val stream = byteArrayOf(0x00, 0xFF.toByte(), LinkConstants.SOF, 0x99.toByte(), 0x01, 0x02) + f
        val got = run(stream)
        assertEquals(1, got.size)
        assertArrayEquals(byteArrayOf(0x11, 0x22), got[0].second)
    }

    @Test
    fun `framer resync false sof with bad version`() {
        val f = makeFrame(Opcode.DriveCmd, byteArrayOf(0x42), 1, 2)
        val stream = byteArrayOf(LinkConstants.SOF, 0xEE.toByte(), 0x00) + f // false SOF, bad version
        val got = run(stream)
        assertEquals(1, got.size)
        assertArrayEquals(byteArrayOf(0x42), got[0].second)
    }

    @Test
    fun `framer back to back two in two chunks`() {
        val a = makeFrame(Opcode.CyclicState, byteArrayOf(1, 2), 1, 2)
        val b = makeFrame(Opcode.DriveCmd, byteArrayOf(3, 4), 3, 4)
        // first frame plus head of second, then tail of second
        val first = a + b.copyOfRange(0, 2)
        val got = run(first, b.copyOfRange(2, b.size))
        assertEquals(2, got.size)
        assertEquals(Opcode.CyclicState.value, got[0].first)
        assertEquals(Opcode.DriveCmd.value, got[1].first)
    }

    @Test
    fun `framer reset drops partial`() {
        val f = makeFrame(Opcode.DriveCmd, byteArrayOf(1, 2, 3), 1, 2)
        val framer = StreamFramer()
        val got = mutableListOf<DecodedFrame>()
        framer.feed(f.copyOfRange(0, 4)) { got.add(it) }
        framer.reset()
        framer.feed(f.copyOfRange(4, f.size)) { got.add(it) }
        // the tail alone is not a valid frame after reset; nothing emitted
        assertTrue(got.isEmpty())
        // a fresh whole frame still works
        framer.feed(f) { got.add(it) }
        assertEquals(1, got.size)
    }
}
