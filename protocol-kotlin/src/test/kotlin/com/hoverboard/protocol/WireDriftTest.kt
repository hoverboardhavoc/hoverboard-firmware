package com.hoverboard.protocol

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Crc16
import com.hoverboard.protocol.l2.FragHdr
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l2.StreamFrame
import com.hoverboard.protocol.l3.HEADER_LEN
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
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
 * file and SYMBOL it was derived from.
 *
 * The point of this file is that a firmware change which outruns the Kotlin fails a build rather
 * than a bench session. Each expectation below was read out of the Rust source, not out of the
 * Kotlin it is checking, so editing the Kotlin to match a changed firmware requires editing this
 * file too, deliberately.
 *
 * The golden byte vectors are copied verbatim from the Rust's own unit tests, so the two languages
 * are pinned to a single shared set of bytes.
 *
 * Repo-relative paths are against the firmware repo root, and every citation names the SYMBOL it
 * refers to (`crates/firmware/src/main.rs, BLE_FRAME_CAP`), never a line number.
 *
 * This file is where that rule was learned twice over. It carried 67 line-numbered citations pinned
 * to firmware main 59e30b9, and by the time anyone looked they pointed at whatever had drifted into
 * those positions since; the branch that first wrote this note then rotted thirteen MORE of them in
 * the same commit, by inserting twelve lines into `crates/net/src/pdu.rs`, so the citation for line
 * 45 stopped being `NodeHello = 0x01` and became a doc line about `is_unicast`. A number that a
 * change elsewhere can silently falsify is not a citation, it is a decoration.
 *
 * A symbol survives every edit that does not rename it, and a rename is exactly the moment someone
 * should be re-reading the claim attached to it. Symbols are also CHECKABLE, and are checked:
 * `tools/check-citations.py` resolves every citation in this file against the file it names and
 * fails the build on one that does not, which is what makes the rule above enforced rather than
 * merely stated. It earned its place immediately, by finding a citation here that named a Rust test
 * one word off from the real one (timeouts for constants) and would have read as authoritative
 * forever.
 *
 * A line number is never permitted here, with no exception. This header used to carve one out for
 * citations into frozen external sources (the EFeru dump, the Declassyfied decompile), on the
 * reasoning that nobody edits those so they cannot rot - but the checker rejected that form, this
 * file has never cited one, and a rule stated in prose and enforced nowhere is the thing this file
 * exists to argue against. If such a citation is ever wanted, it comes back with the code that
 * accepts it.
 *
 * One class of citation is checked by a human and not by that script: paths under `specs/`, which
 * this file cites for the two reserved L3 opcode holes. `specs/` is gitignored, so it exists in the
 * primary checkout and nowhere else - not in a worktree, not in a clean clone, not on the runner -
 * and a check that resolved them could only ever pass on one machine. The script skips them and
 * says how many it skipped, so the exemption is visible in its output rather than silent.
 */
class WireDriftTest {

    // --- Opcode allocation ----------------------------------------------------------------------

    /**
     * L7 control block, `crates/linkctl/src/lib.rs` (`OP_CYCLIC_STATE`, `OP_DRIVE_CMD`, `OP_INPUTS`,
     * `OP_FAULT`), pinned on the Rust side by its own `opcode_allocation_pinned`.
     *
     * These are the values the retired rider protocol got wrong: it had INPUTS at 0x50, a
     * TELEMETRY opcode at 0x20 that does not exist in the firmware, and FAULT at 0x40.
     */
    @Test
    fun linkctlOpcodesMatchTheFirmware() {
        assertEquals(0x10, OP_CYCLIC_STATE, "crates/linkctl/src/lib.rs, OP_CYCLIC_STATE")
        assertEquals(0x11, OP_DRIVE_CMD, "crates/linkctl/src/lib.rs, OP_DRIVE_CMD")
        assertEquals(0x12, OP_INPUTS, "crates/linkctl/src/lib.rs, OP_INPUTS")
        assertEquals(0x13, OP_FAULT, "crates/linkctl/src/lib.rs, OP_FAULT")
    }

    /**
     * The whole control block lives inside the reserved range `0x10..0x2F`, asserted on the Rust
     * side by `opcode_allocation_pinned` in `crates/linkctl/src/lib.rs`. A new family added outside
     * the range would be forwarded by L3 but never handed back to the control decoder.
     */
    @Test
    fun theControlBlockStaysInsideItsReservedRange() {
        for (op in listOf(OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS, OP_FAULT)) {
            assertTrue(op in 0x10..0x2F, "opcode 0x${op.toString(16)} left 0x10..0x2F")
        }
    }

    /**
     * L3 opcodes, `crates/net/src/pdu.rs`, `Opcode`. 0x04 and 0x05 are reserved holes (the retired
     * ATTACH / ATTACH_ACK, `specs/l3.md`, "Discovery and address assignment") and must stay
     * unallocated.
     */
    @Test
    fun l3OpcodesMatchTheFirmware() {
        assertEquals(0x01, Opcode.NodeHello.value, "crates/net/src/pdu.rs, Opcode::NodeHello")
        assertEquals(0x02, Opcode.ProbePorts.value, "crates/net/src/pdu.rs, Opcode::ProbePorts")
        assertEquals(0x03, Opcode.Ports.value, "crates/net/src/pdu.rs, Opcode::Ports")
        assertEquals(0x06, Opcode.Assign.value, "crates/net/src/pdu.rs, Opcode::Assign")
        assertEquals(0x07, Opcode.AssignAck.value, "crates/net/src/pdu.rs, Opcode::AssignAck")
        assertEquals(0x30, Opcode.ConfigRead.value, "crates/net/src/pdu.rs, Opcode::ConfigRead")
        assertEquals(0x31, Opcode.ConfigWrite.value, "crates/net/src/pdu.rs, Opcode::ConfigWrite")
        assertEquals(0x32, Opcode.ConfigResp.value, "crates/net/src/pdu.rs, Opcode::ConfigResp")
        assertEquals(0x33, Opcode.ConfigWriteMulti.value, "crates/net/src/pdu.rs, Opcode::ConfigWriteMulti")

        assertNull(Opcode.fromU8(0x04), "0x04 is a reserved hole: specs/l3.md, Discovery and address assignment")
        assertNull(Opcode.fromU8(0x05), "0x05 is a reserved hole: specs/l3.md, Discovery and address assignment")
    }

    /**
     * The two opcode namespaces are disjoint layers and must never be merged into one enum: 0x10
     * is a perfectly valid L3 opcode byte that L3 deliberately does not interpret
     * (`crates/net/src/pdu.rs`, `Opcode` and `Opcode::from_u8`). Merging them would make L3 reject or
     * mis-handle a
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
     * Pinned on the Rust side by `committed_lengths_pinned` in `crates/linkctl/src/lib.rs`.
     *
     * CyclicState at 11 is the one the retired rider protocol had at 7: it was missing the
     * `battery` u16 inserted mid-struct, and had a single `status` byte where the firmware carries
     * `mode`, `fault` and `flags`.
     */
    @Test
    fun committedPayloadLengthsMatchTheFirmware() {
        assertEquals(11, CyclicState.LEN, "crates/linkctl/src/lib.rs, CyclicState::LEN")
        assertEquals(5, DriveCmd.LEN, "crates/linkctl/src/lib.rs, DriveCmd::LEN")
        assertEquals(2, Inputs.LEN, "crates/linkctl/src/lib.rs, Inputs::LEN")
        assertEquals(2, Fault.LEN, "crates/linkctl/src/lib.rs, Fault::LEN")
    }

    /**
     * The appended block's length, and the `CYCLIC_STATE` payload's one on-wire length: 11 is
     * where the block starts, 19 is what the wire carries and the only length a decoder takes.
     *
     * Pinned on the Rust side by `appended_block_lengths_pinned` in `crates/linkctl/src/lib.rs`.
     */
    @Test
    fun theAppendedBlockLengthsMatchTheFirmware() {
        assertEquals(8, CyclicObs.LEN, "crates/linkctl/src/lib.rs, CyclicObs::LEN")
        assertEquals(19, CyclicState.ENCODED_LEN, "crates/linkctl/src/lib.rs, CyclicState::ENCODED_LEN")
    }

    /**
     * The demand word's full scale, hand-copied here as every other value in this file is;
     * `RustSourceDriftTest.theDriveDemandScaleAgreesWithTheRustSource` reads it out of the Rust.
     */
    @Test
    fun theDriveDemandIsFullScaleNotTheThousandDomain() {
        assertEquals(32767, DriveCmd.FULL_SCALE, "crates/control/src/config.rs, FRAME_IN_MAX")
    }

    // --- Field order and byte layout, golden vectors from the Rust's own tests --------------------

    /**
     * Copied verbatim from `cyclic_state_wire_layout_is_little_endian` in
     * `crates/linkctl/src/lib.rs`. This pins field ORDER, not just size: swapping any two fields
     * keeps the length the same and only this vector catches it.
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
            obs = CyclicObs(
                phasePeak = 0x0304,
                phaseMean = -3,
                dutyOn = 0x08C1,
                bootTag = 0x7B,
                chip = ChipTag.F130C8,
            ),
        )
        val expected = byteArrayOf(
            0xFE.toByte(), 0xFF.toByte(), // pitch -2
            0x02, 0x01, // roll 0x0102
            0xFF.toByte(), 0xFF.toByte(), // wheelSpeed -1
            0xB2.toByte(), 0xA1.toByte(), // battery 0xA1B2
            0x03, // mode
            0x11, // fault
            0x81.toByte(), // flags: bit0 | bit7
            // The appended block, from offset 11.
            0x04, 0x03, // phasePeak 0x0304
            0xFD.toByte(), 0xFF.toByte(), // phaseMean -3
            0xC1.toByte(), 0x08, // dutyOn 0x08C1
            0x7B, // bootTag
            0x02, // chip: F130C8
        )
        assertArrayEquals(expected, sample.encode())
        assertEquals(CyclicState.ENCODED_LEN, sample.encode().size)
        assertEquals(sample, CyclicState.decode(expected))
    }

    /**
     * A payload of any length but [CyclicState.ENCODED_LEN] is REFUSED, short or long, from
     * `a_cyclic_payload_short_of_the_whole_is_refused` and
     * `an_over_long_cyclic_payload_is_refused` in `crates/linkctl/src/lib.rs`.
     *
     * The eleven committed bytes decoded here until 2026-10-10, with the block absent, for a
     * board on an image from before the block existed. Both arguments for that are void: nothing
     * is owed an older image before a release (boards and phones are updated together), and a
     * truncated frame never reaches this decoder at all, because L2 drops a frame whose CRC-16
     * fails and resyncs at the next SOF (`specs/l2.md`, "The L2 frame"). Eleven bytes can only
     * be a peer that deliberately sent eleven.
     */
    @Test
    fun aPayloadThatIsNotTheOneLengthIsRefused() {
        val whole = byteArrayOf(
            0xFE.toByte(), 0xFF.toByte(), 0x02, 0x01, 0xFF.toByte(), 0xFF.toByte(),
            0xB2.toByte(), 0xA1.toByte(), 0x03, 0x11, 0x81.toByte(),
            0x04, 0x03, 0xFD.toByte(), 0xFF.toByte(), 0xC1.toByte(), 0x08, 0x7B, 0x02,
        )
        assertEquals(CyclicState.ENCODED_LEN, whole.size)
        assertNotNull(CyclicState.decode(whole), "the one length decodes")

        // Every shorter length, which includes the eleven committed bytes and every partial block
        // above them.
        for (size in 0 until CyclicState.ENCODED_LEN) {
            assertNull(CyclicState.decode(whole.copyOf(size)), "$size bytes is refused")
        }
        // And every longer one: this family reads no prefix out of a payload it does not agree
        // with about the length.
        for (size in CyclicState.ENCODED_LEN + 1..CyclicState.ENCODED_LEN + 4) {
            assertNull(CyclicState.decode(whole.copyOf(size)), "$size bytes is refused")
        }
    }

    /**
     * An unallocated chip byte is [ChipTag.Unknown] rather than an error, the same fail-safe the
     * drive kind has: a client that meets a part this build does not know says so.
     * From `chip_tag_maps_both_ways` in `crates/linkctl/src/lib.rs`.
     */
    @Test
    fun anUnknownChipByteIsUnknownNotAnError() {
        for (byte in listOf(4, 0x7F, 0xFF)) {
            val raw = ByteArray(CyclicState.ENCODED_LEN)
            raw[CyclicState.ENCODED_LEN - 1] = byte.toByte()
            assertEquals(ChipTag.Unknown, CyclicState.decode(raw)!!.obs!!.chip, "byte $byte")
        }
    }

    /** From `drive_cmd_wire_layout_is_little_endian` in `crates/linkctl/src/lib.rs`. */
    @Test
    fun driveCmdWireLayoutMatchesTheRustGoldenVector() {
        val cmd = DriveCmd(kind = DriveKind.Throttle, value = -300, steer = 0x1234)
        val expected =
            byteArrayOf(0x01, 0xD4.toByte(), 0xFE.toByte(), 0x34, 0x12)
        assertArrayEquals(expected, cmd.encode())
        assertEquals(cmd, DriveCmd.decode(expected))
    }

    /** From `inputs_wire_layout_is_little_endian` in `crates/linkctl/src/lib.rs`. */
    @Test
    fun inputsWireLayoutMatchesTheRustGoldenVector() {
        val inp = Inputs(buttons = Inputs.BUTTON_POWER, rider = Inputs.RIDER_PRESENT)
        val expected = byteArrayOf(0x01, 0x01)
        assertArrayEquals(expected, inp.encode())
        assertEquals(inp, Inputs.decode(expected))
    }

    /** From `fault_wire_layout` in `crates/linkctl/src/lib.rs`. */
    @Test
    fun faultWireLayoutMatchesTheRustGoldenVector() {
        val f = Fault(code = 0x21, action = Fault.ACTION_STOP_ALL)
        val expected = byteArrayOf(0x21, 0x01)
        assertArrayEquals(expected, f.encode())
        assertEquals(f, Fault.decode(expected))
    }

    /**
     * Flag and action bit values, all in `crates/linkctl/src/lib.rs`: `CyclicState::FLAG_RIDER` /
     * `CyclicState::FLAG_LOCKDOWN`,
     * `:238,241` (Inputs), `:294,297` (Fault).
     */
    @Test
    fun flagAndActionBitsMatchTheFirmware() {
        assertEquals(0x01, CyclicState.FLAG_RIDER, "crates/linkctl/src/lib.rs, CyclicState::FLAG_RIDER")
        assertEquals(
            0x80,
            CyclicState.FLAG_LOCKDOWN,
            "crates/linkctl/src/lib.rs, CyclicState::FLAG_LOCKDOWN",
        )
        assertEquals(0x01, Inputs.BUTTON_POWER, "crates/linkctl/src/lib.rs, Inputs::BUTTON_POWER")
        assertEquals(0x01, Inputs.RIDER_PRESENT, "crates/linkctl/src/lib.rs, Inputs::RIDER_PRESENT")
        assertEquals(0, Fault.ACTION_NOTIFY, "crates/linkctl/src/lib.rs, Fault::ACTION_NOTIFY")
        assertEquals(1, Fault.ACTION_STOP_ALL, "crates/linkctl/src/lib.rs, Fault::ACTION_STOP_ALL")
    }

    /**
     * Supervision timeouts, `crates/linkctl/src/lib.rs` (`CYCLIC_TIMEOUT_TICKS`,
     * `DRIVE_TIMEOUT_TICKS`, `INPUTS_TIMEOUT_TICKS`), pinned on the Rust side by
     * `supervision_constants_pinned`. The controller mirrors these to decide when its own view has
     * gone stale, and `INPUTS_TIMEOUT_TICKS` additionally bounds how long a gap in its OWN sending
     * a board will tolerate before it drops the arm.
     *
     * The ordering is the safety property and is asserted here as well as in the Rust: the demand
     * decays (200 ms) before the arm mirror is released (1.5 s), so link loss stops the machine
     * first and disarms it second, never the reverse.
     */
    @Test
    fun supervisionTimeoutsMatchTheFirmware() {
        assertEquals(
            25,
            com.hoverboard.protocol.linkctl.CYCLIC_TIMEOUT_TICKS,
            "crates/linkctl/src/lib.rs, CYCLIC_TIMEOUT_TICKS",
        )
        assertEquals(
            50,
            com.hoverboard.protocol.linkctl.DRIVE_TIMEOUT_TICKS,
            "crates/linkctl/src/lib.rs, DRIVE_TIMEOUT_TICKS",
        )
        assertEquals(
            375,
            com.hoverboard.protocol.linkctl.INPUTS_TIMEOUT_TICKS,
            "crates/linkctl/src/lib.rs, INPUTS_TIMEOUT_TICKS",
        )
        assertTrue(
            com.hoverboard.protocol.linkctl.INPUTS_TIMEOUT_TICKS >
                com.hoverboard.protocol.linkctl.DRIVE_TIMEOUT_TICKS,
            "the demand must decay before the arm mirror is released",
        )
    }

    // --- L2 frame header shape ------------------------------------------------------------------

    /**
     * The L2 stream frame, `crates/link/src/framer.rs` (the module doc's diagram, with the encoder in
     * `encode`):
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
     * whole-frame length and does not include SOF, len or CRC (`crates/link/src/framer.rs`, the module doc).
     */
    @Test
    fun l2FrameHeaderIsSofThenLengthWithNoVersionByte() {
        assertEquals(0x5A, StreamFrame.SOF, "crates/link/src/framer.rs, SOF")
        assertEquals(2, StreamFrame.STREAM_HEADER_LEN, "crates/link/src/framer.rs, STREAM_HEADER_LEN")
        assertEquals(2, StreamFrame.STREAM_CRC_LEN, "crates/link/src/framer.rs, STREAM_CRC_LEN")
        assertEquals(255, StreamFrame.MAX_L2_LEN, "crates/link/src/framer.rs, MAX_L2_LEN")
        assertEquals(259, StreamFrame.MAX_STREAM_FRAME, "crates/link/src/framer.rs, MAX_STREAM_FRAME")

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
     * (`crates/base/src/crc16.rs`, `modbus`). It covers SOF and the len byte as well as the body, i.e.
     * everything except itself (`crates/link/src/framer.rs`: `encode` writes it,
     * `StreamFramer::feed_one` verifies it), and is written little-endian, low byte first.
     *
     * Coverage is the subtle half: a CRC over the body only would still round-trip in isolation
     * and only fail against real firmware.
     */
    @Test
    fun crcIsModbusOverSofAndLengthToo() {
        // Known-answer vectors from crates/base/src/crc16.rs: golden_check_value, golden_frozen_pair,
        // empty_is_init.
        assertEquals(0x4B37, Crc16.modbus("123456789".toByteArray()), "crates/base/src/crc16.rs, golden_check_value")
        assertEquals(
            0xBB2A,
            Crc16.modbus(byteArrayOf(1, 2, 3, 4, 5)),
            "crates/base/src/crc16.rs, golden_frozen_pair",
        )
        assertEquals(0xFFFF, Crc16.modbus(ByteArray(0)), "crates/base/src/crc16.rs, empty_is_init")

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
     * The one-byte fragmentation header, `crates/link/src/frag.rs` (its module doc):
     * bit7 MORE, bits 6..4 PID (0..7), bits 3..0 FRAG_IDX (0..15).
     */
    @Test
    fun fragHeaderBitPositionsMatchTheFirmware() {
        assertEquals(0b1000_0000, FragHdr.MORE_BIT, "crates/link/src/frag.rs, MORE_BIT")
        assertEquals(0b0000_0111, FragHdr.MAX_PID, "crates/link/src/frag.rs, MAX_PID")
        assertEquals(0b0000_1111, FragHdr.MAX_FRAG_IDX, "crates/link/src/frag.rs, MAX_FRAG_IDX")
        assertEquals(16, FragHdr.MAX_FRAGMENTS, "crates/link/src/frag.rs, MAX_FRAGMENTS")

        // PID sits at bits 6..4, so pid 5 encodes as 0b0101_0000.
        assertEquals(0b0101_0011, FragHdr(more = false, pid = 5, fragIdx = 3).encode())
        assertEquals(0b1101_0011, FragHdr(more = true, pid = 5, fragIdx = 3).encode())
    }

    // --- L3 PDU header shape --------------------------------------------------------------------

    /**
     * The L3 PDU is `[opcode][src][dst][payload...]` with no SOF, len, CRC, version, seq, TTL or
     * hop count: L2 owns framing and integrity (`crates/net/src/pdu.rs`, the module doc + `HEADER_LEN`).
     */
    @Test
    fun l3PduHeaderIsThreeBytesAndCarriesNoIntegrity() {
        assertEquals(3, HEADER_LEN, "crates/net/src/pdu.rs, HEADER_LEN")

        val payload = byteArrayOf(0x01, 0x02)
        val pdu = Pdu(opcode = OP_CYCLIC_STATE, src = 0x02, dst = 0x00, payload = payload)
        val bytes = pdu.encode()

        assertEquals(HEADER_LEN + payload.size, bytes.size, "header then payload, nothing else")
        assertEquals(OP_CYCLIC_STATE, bytes[0].toInt() and 0xFF)
        assertEquals(0x02, bytes[1].toInt() and 0xFF)
        assertEquals(0x00, bytes[2].toInt() and 0xFF)
    }

    /** Address ranges, `crates/net/src/pdu.rs` (`BROADCAST` through `is_unicast`). */
    @Test
    fun addressRangesMatchTheFirmware() {
        assertEquals(0xFF, com.hoverboard.protocol.l3.BROADCAST, "crates/net/src/pdu.rs, BROADCAST")
        assertEquals(0x00, com.hoverboard.protocol.l3.NO_ADDRESS, "crates/net/src/pdu.rs, NO_ADDRESS")

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

    /** Type tags ride inside CONFIG_*; `crates/store/src/key.rs`, `Type::tag`. */
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
     * The committed-prefix rule (`crates/linkctl/src/lib.rs`, the module doc), pinned on the Rust
     * side by `trailing_bytes_are_ignored_every_append_tolerant_family` and
     * `short_payloads_are_rejected`.
     *
     * This is what lets the firmware append a field to one of these three without breaking a
     * phone in someone's pocket, so the behaviour is pinned in both directions. `CyclicState` is
     * NOT one of them: its length is exact, and [aPayloadThatIsNotTheOneLengthIsRefused] holds
     * that side.
     */
    @Test
    fun trailingBytesAreIgnoredAndShortPayloadsRejected() {
        val inputs = Inputs(1, 1)
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
     * (`crates/linkctl/src/lib.rs`, `DriveCmd::decode`, tested by `unknown_drive_kind_decodes_as_neutral`).
     * Fail-safe: an unrecognised
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
     * A CYCLIC_STATE PDU is TWO BLE stream frames, each inside one ATT notification.
     *
     * ```
     * CYCLIC_STATE payload            19 B   crates/linkctl, CyclicState::ENCODED_LEN
     * + L3 header                      3 B   crates/net, HEADER_LEN
     *                                = 22 B PDU
     * BLE frame_capacity 16 -> usable chunk 15 < 22, so TWO fragments, 15 B + 7 B
     * L2 frames = frag-hdr 1 + 15 = 16 B and frag-hdr 1 + 7 = 8 B
     * wire = SOF 1 + len 1 + body + CRC 2 = 20 B + 12 B = 32 B, each frame <= 20
     * ```
     *
     * It was 14 B, one fragment and 19 B of wire until the payload grew its appended block:
     * crossing the 15 B chunk is what makes eight payload bytes cost thirteen here, and the 5 Hz
     * rate's budget is re-derived from the 32 against the module's ~960 B/s
     * (`crates/orchestrator/src/dispatch.rs`, `BLE_CYCLIC_DIVISOR`).
     *
     * BLE frame capacity 16 is `BLE_FRAME_CAP` in `crates/firmware/src/main.rs`, named rather than
     * cited by line because a line number rots on the first insertion above it (this one said
     * line 149 while the constant had moved to line 145). The same arithmetic is asserted on the
     * Rust side by `stage_of_a_cyclic_state_pdu_is_two_frames_of_thirty_two_bytes_on_the_ble_wire`
     * in `crates/link/src/link.rs`.
     *
     * What this pins is the receiving end: the rider reassembles such a payload from TWO
     * notifications, which is the reason the receive path is a continuous stream rather than one
     * frame per ATT transaction.
     */
    @Test
    fun aCyclicStatePduIsTwoBleFramesOfThirtyTwoBytes() {
        assertEquals(
            16,
            BleStreamTransport.DEFAULT_FRAME_CAPACITY,
            "crates/firmware/src/main.rs, BLE_FRAME_CAP",
        )

        val cyclic = CyclicState(
            pitch = -250,
            roll = 125,
            wheelSpeed = 900,
            battery = 3780,
            mode = 2,
            fault = 0,
            flags = CyclicState.FLAG_RIDER,
            obs = CyclicObs(
                phasePeak = 980,
                phaseMean = 410,
                dutyOn = 1_125,
                bootTag = 7,
                chip = ChipTag.F103C8,
            ),
        )
        val pdu = Pdu(
            opcode = OP_CYCLIC_STATE,
            src = 0x01,
            dst = com.hoverboard.protocol.l3.NO_ADDRESS,
            payload = cyclic.encode(),
        )
        val pduBytes = pdu.encode()
        assertEquals(22, pduBytes.size, "3 B L3 header + 19 B CYCLIC_STATE")

        val transport = BleStreamTransport()
        Link(transport).send(pduBytes)
        val wire = transport.drainOutgoing()!!

        assertEquals(32, wire.size, "two stream frames, 20 B + 12 B")
        // Frame boundaries: SOF, length byte, body, CRC16. The first frame's length byte says 16
        // (frag-hdr + a 15 B chunk), so the second starts at 20.
        assertEquals(16, wire[1].toInt() and 0xFF, "the first frame's body is a full chunk")
        assertEquals(8, wire[21].toInt() and 0xFF, "the second frame carries the 7 B remainder")
        assertTrue(
            20 <= 20 && wire.size - 20 <= 20,
            "each frame must fit one 20-byte ATT notification",
        )

        // And it round-trips back through the receive path, which is exactly what the rider does
        // with an inbound notification.
        val rxTransport = BleStreamTransport()
        rxTransport.onReceive(wire)
        val recovered = Pdu.decode(Link(rxTransport).pollRecv()!!)
        assertEquals(OP_CYCLIC_STATE, recovered.opcode)
        assertEquals(cyclic, CyclicState.decode(recovered.payload))
    }
}
