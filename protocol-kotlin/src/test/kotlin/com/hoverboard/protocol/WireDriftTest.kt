package com.hoverboard.protocol

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Crc16
import com.hoverboard.protocol.l2.FragHdr
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l2.StreamFrame
import com.hoverboard.protocol.l3.HEADER_LEN
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.DriveCmd
import com.hoverboard.protocol.linkctl.DriveKind
import com.hoverboard.protocol.linkctl.Fault
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_CYCLIC_STATE
import com.hoverboard.protocol.linkctl.OP_DRIVE_CMD
import com.hoverboard.protocol.linkctl.OP_FAULT
import com.hoverboard.protocol.linkctl.OP_INPUTS
import com.hoverboard.protocol.store.Type
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Drift pins: every wire fact this Kotlin mirror asserts about the Rust firmware, with the Rust
 * file:line it was derived from.
 *
 * The point of this file is that a firmware change which outruns the Kotlin fails a build rather
 * than a bench session. Each expectation below was read out of the Rust source, not out of the
 * Kotlin it is checking, so editing the Kotlin to match a changed firmware requires editing this
 * file too, deliberately.
 *
 * The golden byte vectors are copied verbatim from the Rust's own unit tests, so the two languages
 * are pinned to a single shared set of bytes.
 *
 * Repo-relative paths are against the firmware repo root. Line numbers are as of firmware main
 * 59e30b9.
 */
class WireDriftTest {

    // --- Opcode allocation ----------------------------------------------------------------------

    /**
     * L7 control block, `crates/linkctl/src/lib.rs:34,37,40,43`, pinned on the Rust side by
     * `opcode_allocation_pinned` at `crates/linkctl/src/lib.rs:361-371`.
     *
     * These are the values the retired rider protocol got wrong: it had INPUTS at 0x50, a
     * TELEMETRY opcode at 0x20 that does not exist in the firmware, and FAULT at 0x40.
     */
    @Test
    fun linkctlOpcodesMatchTheFirmware() {
        assertEquals(0x10, OP_CYCLIC_STATE, "crates/linkctl/src/lib.rs:34")
        assertEquals(0x11, OP_DRIVE_CMD, "crates/linkctl/src/lib.rs:37")
        assertEquals(0x12, OP_INPUTS, "crates/linkctl/src/lib.rs:40")
        assertEquals(0x13, OP_FAULT, "crates/linkctl/src/lib.rs:43")
    }

    /**
     * The whole control block lives inside the reserved range `0x10..0x2F`, asserted on the Rust
     * side at `crates/linkctl/src/lib.rs:361-371`. A new family added outside the range would be
     * forwarded by L3 but never handed back to the control decoder.
     */
    @Test
    fun theControlBlockStaysInsideItsReservedRange() {
        for (op in listOf(OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS, OP_FAULT)) {
            assertTrue(op in 0x10..0x2F, "opcode 0x${op.toString(16)} left 0x10..0x2F")
        }
    }

    /**
     * L3 opcodes, `crates/net/src/pdu.rs:43-62`. 0x04 and 0x05 are reserved holes (the retired
     * ATTACH / ATTACH_ACK, `specs/l3.md:264`) and must stay unallocated.
     */
    @Test
    fun l3OpcodesMatchTheFirmware() {
        assertEquals(0x01, Opcode.NodeHello.value, "crates/net/src/pdu.rs:45")
        assertEquals(0x02, Opcode.ProbePorts.value, "crates/net/src/pdu.rs:47")
        assertEquals(0x03, Opcode.Ports.value, "crates/net/src/pdu.rs:49")
        assertEquals(0x06, Opcode.Assign.value, "crates/net/src/pdu.rs:51")
        assertEquals(0x07, Opcode.AssignAck.value, "crates/net/src/pdu.rs:53")
        assertEquals(0x30, Opcode.ConfigRead.value, "crates/net/src/pdu.rs:55")
        assertEquals(0x31, Opcode.ConfigWrite.value, "crates/net/src/pdu.rs:57")
        assertEquals(0x32, Opcode.ConfigResp.value, "crates/net/src/pdu.rs:59")
        assertEquals(0x33, Opcode.ConfigWriteMulti.value, "crates/net/src/pdu.rs:61")

        assertNull(Opcode.fromU8(0x04), "0x04 is a reserved hole, specs/l3.md:264")
        assertNull(Opcode.fromU8(0x05), "0x05 is a reserved hole, specs/l3.md:264")
    }

    /**
     * The two opcode namespaces are disjoint layers and must never be merged into one enum: 0x10
     * is a perfectly valid L3 opcode byte that L3 deliberately does not interpret
     * (`crates/net/src/pdu.rs:41`, `:70-71`). Merging them would make L3 reject or mis-handle a
     * control PDU it is only supposed to forward by `dst`.
     */
    @Test
    fun l3DoesNotInterpretTheControlBlock() {
        for (op in listOf(OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS, OP_FAULT)) {
            assertNull(Opcode.fromU8(op), "L3 must not claim control opcode 0x${op.toString(16)}")
        }
    }

    // --- Committed payload lengths --------------------------------------------------------------

    /**
     * Pinned on the Rust side by `committed_lengths_pinned` at `crates/linkctl/src/lib.rs:382`.
     *
     * CyclicState at 11 is the one the retired rider protocol had at 7: it was missing the
     * `battery` u16 inserted mid-struct, and had a single `status` byte where the firmware carries
     * `mode`, `fault` and `flags`.
     */
    @Test
    fun committedPayloadLengthsMatchTheFirmware() {
        assertEquals(11, CyclicState.LEN, "crates/linkctl/src/lib.rs:110")
        assertEquals(5, DriveCmd.LEN, "crates/linkctl/src/lib.rs:188")
        assertEquals(4, Inputs.LEN, "crates/linkctl/src/lib.rs:235")
        assertEquals(2, Fault.LEN, "crates/linkctl/src/lib.rs:291")
    }

    /**
     * The demand word's full scale, hand-copied here as every other value in this file is;
     * `RustSourceDriftTest.theDriveDemandScaleAgreesWithTheRustSource` reads it out of the Rust.
     */
    @Test
    fun theDriveDemandIsFullScaleNotTheThousandDomain() {
        assertEquals(32767, DriveCmd.FULL_SCALE, "crates/control/src/config.rs:168")
    }

    // --- Field order and byte layout, golden vectors from the Rust's own tests --------------------

    /**
     * Copied verbatim from `cyclic_state_wire_layout_is_little_endian`,
     * `crates/linkctl/src/lib.rs:402-418`. This pins field ORDER, not just size: swapping any two
     * fields keeps the length at 11 and only this vector catches it.
     */
    @Test
    fun cyclicStateWireLayoutMatchesTheRustGoldenVector() {
        val sample = CyclicState(
            pitch = -2,
            roll = 0x0102,
            wheelSpeed = -1,
            battery = 0xA1B2,
            mode = 0x03,
            fault = 0x11,
            flags = CyclicState.FLAG_RIDER or CyclicState.FLAG_LOCKDOWN,
        )
        val expected = byteArrayOf(
            0xFE.toByte(), 0xFF.toByte(), // pitch -2
            0x02, 0x01, // roll 0x0102
            0xFF.toByte(), 0xFF.toByte(), // wheelSpeed -1
            0xB2.toByte(), 0xA1.toByte(), // battery 0xA1B2
            0x03, // mode
            0x11, // fault
            0x81.toByte(), // flags: bit0 | bit7
        )
        assertArrayEquals(expected, sample.encode())
        assertEquals(CyclicState.LEN, sample.encode().size)
        assertEquals(sample, CyclicState.decode(expected))
    }

    /** From `drive_cmd_wire_layout_is_little_endian`, `crates/linkctl/src/lib.rs:420-433`. */
    @Test
    fun driveCmdWireLayoutMatchesTheRustGoldenVector() {
        val cmd = DriveCmd(kind = DriveKind.Throttle, value = -300, steer = 0x1234)
        val expected =
            byteArrayOf(0x01, 0xD4.toByte(), 0xFE.toByte(), 0x34, 0x12)
        assertArrayEquals(expected, cmd.encode())
        assertEquals(cmd, DriveCmd.decode(expected))
    }

    /** From `inputs_wire_layout_is_little_endian`, `crates/linkctl/src/lib.rs:435-445`. */
    @Test
    fun inputsWireLayoutMatchesTheRustGoldenVector() {
        val inp = Inputs(
            throttle = 0x7FFF,
            buttons = Inputs.BUTTON_POWER,
            rider = Inputs.RIDER_PRESENT,
        )
        val expected = byteArrayOf(0xFF.toByte(), 0x7F, 0x01, 0x01)
        assertArrayEquals(expected, inp.encode())
        assertEquals(inp, Inputs.decode(expected))
    }

    /** From `fault_wire_layout`, `crates/linkctl/src/lib.rs:447-456`. */
    @Test
    fun faultWireLayoutMatchesTheRustGoldenVector() {
        val f = Fault(code = 0x21, action = Fault.ACTION_STOP_ALL)
        val expected = byteArrayOf(0x21, 0x01)
        assertArrayEquals(expected, f.encode())
        assertEquals(f, Fault.decode(expected))
    }

    /**
     * Flag and action bit values: `crates/linkctl/src/lib.rs:113,118` (CyclicState),
     * `:238,241` (Inputs), `:294,297` (Fault).
     */
    @Test
    fun flagAndActionBitsMatchTheFirmware() {
        assertEquals(0x01, CyclicState.FLAG_RIDER, "crates/linkctl/src/lib.rs:113")
        assertEquals(0x80, CyclicState.FLAG_LOCKDOWN, "crates/linkctl/src/lib.rs:118")
        assertEquals(0x01, Inputs.BUTTON_POWER, "crates/linkctl/src/lib.rs:238")
        assertEquals(0x01, Inputs.RIDER_PRESENT, "crates/linkctl/src/lib.rs:241")
        assertEquals(0, Fault.ACTION_NOTIFY, "crates/linkctl/src/lib.rs:294")
        assertEquals(1, Fault.ACTION_STOP_ALL, "crates/linkctl/src/lib.rs:297")
    }

    /**
     * Supervision timeouts, `crates/linkctl/src/lib.rs:51,56`, pinned on the Rust side at
     * `:373-378`. The controller mirrors these to decide when its own view has gone stale.
     */
    @Test
    fun supervisionTimeoutsMatchTheFirmware() {
        assertEquals(
            25,
            com.hoverboard.protocol.linkctl.CYCLIC_TIMEOUT_TICKS,
            "crates/linkctl/src/lib.rs:51",
        )
        assertEquals(
            50,
            com.hoverboard.protocol.linkctl.DRIVE_TIMEOUT_TICKS,
            "crates/linkctl/src/lib.rs:56",
        )
    }

    // --- L2 frame header shape ------------------------------------------------------------------

    /**
     * The L2 stream frame, `crates/link/src/framer.rs:8` with the encoder at `:44-60`:
     *
     * ```
     * [ SOF 1 = 0x5A ][ len 1 ][ frag-hdr 1 ][ chunk len-1 ][ CRC-lo 1 ][ CRC-hi 1 ]
     * ```
     *
     * The header is TWO bytes and the second one is the LENGTH. The retired rider framer expected
     * a version byte 0x01 at offset 1 (a six-byte SOF/ver/opcode/src/dst/len header), which is the
     * single break that made every current frame unparseable to it.
     *
     * `len` counts the frag-hdr through end of chunk, so `len == 1 + chunk.size`. It is NOT the
     * whole-frame length and does not include SOF, len or CRC (`crates/link/src/framer.rs:11-12`).
     */
    @Test
    fun l2FrameHeaderIsSofThenLengthWithNoVersionByte() {
        assertEquals(0x5A, StreamFrame.SOF, "crates/link/src/framer.rs:23")
        assertEquals(2, StreamFrame.STREAM_HEADER_LEN, "crates/link/src/framer.rs:25")
        assertEquals(2, StreamFrame.STREAM_CRC_LEN, "crates/link/src/framer.rs:27")
        assertEquals(255, StreamFrame.MAX_L2_LEN, "crates/link/src/framer.rs:29")
        assertEquals(259, StreamFrame.MAX_STREAM_FRAME, "crates/link/src/framer.rs:31")

        // An L2 frame of frag-hdr 0x00 plus a 3-byte chunk.
        val l2 = byteArrayOf(0x00, 0xAA.toByte(), 0xBB.toByte(), 0xCC.toByte())
        val wire = StreamFrame.encode(l2)

        assertEquals(0x5A, wire[0].toInt() and 0xFF, "offset 0 is SOF")
        assertEquals(l2.size, wire[1].toInt() and 0xFF, "offset 1 is len == frag-hdr + chunk")
        assertEquals(0x00, wire[2].toInt() and 0xFF, "offset 2 is the frag-hdr, not a version byte")
        assertEquals(2 + l2.size + 2, wire.size, "SOF + len + body + CRC16")
    }

    /**
     * CRC choice and coverage. CRC-16/MODBUS, reflected poly 0xA001, init 0xFFFF, no final XOR
     * (`crates/base/src/crc16.rs:6,17`). It covers SOF and the len byte as well as the body,
     * i.e. everything except itself (`crates/link/src/framer.rs:56` encode, `:188` verify), and is
     * written little-endian, low byte first (`crates/link/src/framer.rs:57-58`).
     *
     * Coverage is the subtle half: a CRC over the body only would still round-trip in isolation
     * and only fail against real firmware.
     */
    @Test
    fun crcIsModbusOverSofAndLengthToo() {
        // Known-answer vectors from crates/base/src/crc16.rs:70,78,84.
        assertEquals(0x4B37, Crc16.modbus("123456789".toByteArray()), "crates/base/src/crc16.rs:70")
        assertEquals(
            0xBB2A,
            Crc16.modbus(byteArrayOf(1, 2, 3, 4, 5)),
            "crates/base/src/crc16.rs:78",
        )
        assertEquals(0xFFFF, Crc16.modbus(ByteArray(0)), "crates/base/src/crc16.rs:84")

        val l2 = byteArrayOf(0x00, 0xAA.toByte(), 0xBB.toByte(), 0xCC.toByte())
        val wire = StreamFrame.encode(l2)
        val bodyEnd = StreamFrame.STREAM_HEADER_LEN + l2.size

        val overWholePrefix = Crc16.modbus(wire, 0, bodyEnd)
        val onWire = (wire[bodyEnd].toInt() and 0xFF) or ((wire[bodyEnd + 1].toInt() and 0xFF) shl 8)
        assertEquals(overWholePrefix, onWire, "CRC covers SOF + len + body, little-endian")

        val overBodyOnly = Crc16.modbus(wire, StreamFrame.STREAM_HEADER_LEN, l2.size)
        assertTrue(overBodyOnly != onWire, "a body-only CRC must not accidentally agree")
    }

    /**
     * The one-byte fragmentation header, `crates/link/src/frag.rs:1-23`:
     * bit7 MORE, bits 6..4 PID (0..7), bits 3..0 FRAG_IDX (0..15).
     */
    @Test
    fun fragHeaderBitPositionsMatchTheFirmware() {
        assertEquals(0b1000_0000, FragHdr.MORE_BIT, "crates/link/src/frag.rs:15")
        assertEquals(0b0000_0111, FragHdr.MAX_PID, "crates/link/src/frag.rs:19")
        assertEquals(0b0000_1111, FragHdr.MAX_FRAG_IDX, "crates/link/src/frag.rs:21")
        assertEquals(16, FragHdr.MAX_FRAGMENTS, "crates/link/src/frag.rs:23")

        // PID sits at bits 6..4, so pid 5 encodes as 0b0101_0000.
        assertEquals(0b0101_0011, FragHdr(more = false, pid = 5, fragIdx = 3).encode())
        assertEquals(0b1101_0011, FragHdr(more = true, pid = 5, fragIdx = 3).encode())
    }

    // --- L3 PDU header shape --------------------------------------------------------------------

    /**
     * The L3 PDU is `[opcode][src][dst][payload...]` with no SOF, len, CRC, version, seq, TTL or
     * hop count: L2 owns framing and integrity (`crates/net/src/pdu.rs:4,7,21`).
     */
    @Test
    fun l3PduHeaderIsThreeBytesAndCarriesNoIntegrity() {
        assertEquals(3, HEADER_LEN, "crates/net/src/pdu.rs:21")

        val payload = byteArrayOf(0x01, 0x02)
        val pdu = Pdu(opcode = OP_CYCLIC_STATE, src = 0x02, dst = 0x00, payload = payload)
        val bytes = pdu.encode()

        assertEquals(HEADER_LEN + payload.size, bytes.size, "header then payload, nothing else")
        assertEquals(OP_CYCLIC_STATE, bytes[0].toInt() and 0xFF)
        assertEquals(0x02, bytes[1].toInt() and 0xFF)
        assertEquals(0x00, bytes[2].toInt() and 0xFF)
    }

    /** Address ranges, `crates/net/src/pdu.rs:14-37`. */
    @Test
    fun addressRangesMatchTheFirmware() {
        assertEquals(0xFF, com.hoverboard.protocol.l3.BROADCAST, "crates/net/src/pdu.rs:15")
        assertEquals(0x00, com.hoverboard.protocol.l3.NO_ADDRESS, "crates/net/src/pdu.rs:18")

        assertTrue(com.hoverboard.protocol.l3.isBoard(0x01) && com.hoverboard.protocol.l3.isBoard(0x7F))
        assertTrue(!com.hoverboard.protocol.l3.isBoard(0x00) && !com.hoverboard.protocol.l3.isBoard(0x80))
        assertTrue(
            com.hoverboard.protocol.l3.isController(0x80) &&
                com.hoverboard.protocol.l3.isController(0xFE),
        )
        assertTrue(!com.hoverboard.protocol.l3.isController(0xFF))
        assertTrue(
            !com.hoverboard.protocol.l3.isUnicast(0x00) && !com.hoverboard.protocol.l3.isUnicast(0xFF),
        )
    }

    // --- Store value type tags ------------------------------------------------------------------

    /** Type tags ride inside CONFIG_*; `crates/store/src/key.rs:48-60`. */
    @Test
    fun storeTypeTagsMatchTheFirmware() {
        assertEquals(0x01, Type.U8.tag)
        assertEquals(0x02, Type.U16.tag)
        assertEquals(0x03, Type.U32.tag)
        assertEquals(0x04, Type.U64.tag)
        assertEquals(0x05, Type.I16.tag)
        assertEquals(0x06, Type.I32.tag)
        assertEquals(0x07, Type.I64.tag)
        assertEquals(0x08, Type.Bool.tag)
        assertEquals(0x09, Type.Blob.tag)
        assertEquals(0x0A, Type.Str.tag)
    }

    // --- Forward-compatibility contract -----------------------------------------------------------

    /**
     * The committed-prefix rule, `crates/linkctl/src/lib.rs:98-102`, pinned on the Rust side by
     * `trailing_bytes_are_ignored_every_family` (`:504-539`) and `short_payloads_are_rejected`
     * (`:543-559`).
     *
     * This is what lets the firmware append a field without breaking a phone in someone's pocket.
     * If the Kotlin ever tightened to an exact-length check, every future firmware would look
     * corrupt to it, so the behaviour is pinned in both directions.
     */
    @Test
    fun trailingBytesAreIgnoredAndShortPayloadsRejected() {
        val cyclic = CyclicState(1, 2, 3, 4, 5, 6, 7)
        val padded = cyclic.encode() + byteArrayOf(0x77, 0x88.toByte())
        assertEquals(cyclic, CyclicState.decode(padded), "a longer future payload still decodes")
        assertNull(CyclicState.decode(cyclic.encode().copyOf(CyclicState.LEN - 1)))

        val inputs = Inputs(-5, 1, 1)
        assertEquals(inputs, Inputs.decode(inputs.encode() + byteArrayOf(0x00)))
        assertNull(Inputs.decode(ByteArray(Inputs.LEN - 1)))

        val fault = Fault(0x21, 1)
        assertEquals(fault, Fault.decode(fault.encode() + byteArrayOf(0x00)))
        assertNull(Fault.decode(ByteArray(Fault.LEN - 1)))

        val drive = DriveCmd(DriveKind.Throttle, 10, 20)
        assertEquals(drive, DriveCmd.decode(drive.encode() + byteArrayOf(0x00)))
        assertNull(DriveCmd.decode(ByteArray(DriveCmd.LEN - 1)))
    }

    /**
     * An unknown DRIVE_CMD kind byte decodes as Neutral rather than failing
     * (`crates/linkctl/src/lib.rs:206-209`, test at `:563-582`). Fail-safe: an unrecognised
     * command must not be read as throttle.
     */
    @Test
    fun anUnknownDriveKindIsNeutralNotAnError() {
        val raw = byteArrayOf(0x7F, 0x10, 0x00, 0x20, 0x00)
        val decoded = DriveCmd.decode(raw)
        assertNotNull(decoded)
        assertEquals(DriveKind.Neutral, decoded!!.kind)
        assertEquals(0x10, decoded.value, "value is carried through even under Neutral")
    }

    // --- The BLE transport budget ---------------------------------------------------------------

    /**
     * A CYCLIC_STATE PDU fits one BLE ATT notification, and does so as a SINGLE fragment.
     *
     * ```
     * CYCLIC_STATE payload            11 B   crates/linkctl/src/lib.rs:110
     * + L3 header                      3 B   crates/net/src/pdu.rs:21
     *                                = 14 B PDU
     * BLE frame_capacity 16 -> usable chunk 15 >= 14, so one fragment
     * L2 frame = frag-hdr 1 + chunk 14 = 15 B
     * wire = SOF 1 + len 1 + 15 + CRC 2 = 19 B <= 20
     * ```
     *
     * BLE frame capacity 16 is `crates/firmware/src/main.rs:149`. The same 19-byte arithmetic is
     * asserted on the Rust side by `stage_of_a_cyclic_state_pdu_is_nineteen_bytes_on_the_ble_wire`
     * on branch feat/ble-telemetry, which adds the 5 Hz CYCLIC_STATE emission to the BLE port.
     *
     * This test is the readiness check for that branch: it does not require the branch, it just
     * pins that when the firmware starts sending these, the rider parses them out of one
     * notification with no re-chunking.
     */
    @Test
    fun aCyclicStatePduIsNineteenBytesOnTheBleWireInOneFragment() {
        assertEquals(16, BleStreamTransport.DEFAULT_FRAME_CAPACITY, "crates/firmware/src/main.rs:149")

        val cyclic = CyclicState(
            pitch = -250,
            roll = 125,
            wheelSpeed = 900,
            battery = 3780,
            mode = 2,
            fault = 0,
            flags = CyclicState.FLAG_RIDER,
        )
        val pdu = Pdu(
            opcode = OP_CYCLIC_STATE,
            src = 0x01,
            dst = com.hoverboard.protocol.l3.NO_ADDRESS,
            payload = cyclic.encode(),
        )
        val pduBytes = pdu.encode()
        assertEquals(14, pduBytes.size, "3 B L3 header + 11 B CYCLIC_STATE")

        val transport = BleStreamTransport()
        Link(transport).send(pduBytes)
        val wire = transport.drainOutgoing()!!

        assertEquals(19, wire.size, "one 19-byte stream frame")
        assertTrue(wire.size <= 20, "must fit one 20-byte ATT notification")
        assertEquals(0x00, wire[2].toInt() and 0xFF, "single fragment: frag-hdr 0, MORE clear")

        // And it round-trips back through the receive path, which is exactly what the rider does
        // with an inbound notification.
        val rxTransport = BleStreamTransport()
        rxTransport.onReceive(wire)
        val recovered = Pdu.decode(Link(rxTransport).pollRecv()!!)
        assertEquals(OP_CYCLIC_STATE, recovered.opcode)
        assertEquals(cyclic, CyclicState.decode(recovered.payload))
    }
}
