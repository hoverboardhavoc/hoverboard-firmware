package com.hoverboard.remote

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.linkctl.DriveCmd
import com.hoverboard.protocol.linkctl.DriveKind
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_DRIVE_CMD
import com.hoverboard.protocol.linkctl.OP_INPUTS
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What one tick of rider intent puts on the wire.
 *
 * These are the tests the app never had, and their absence is why it shipped for a bench session
 * unable to move anything: every existing test asserted on `Inputs` objects, so "the app sends the
 * wrong opcode for motion" and "the app never asserts power" were both invisible to the suite.
 */
class RiderCommandTest {

    private val src = 0x81
    private val dst = 0x02

    private fun pdusOf(command: RiderCommand): List<Pdu> =
        command.pdus(src, dst).map { checkNotNull(Pdu.decodeOrNull(it)) { "undecodable PDU" } }

    @Test
    fun `a drive demand is emitted as DRIVE_CMD, and INPUTS carries only levels`() {
        val pdus = pdusOf(RiderCommand.armed(demand = 2_000))

        // The word that moves a wheel is opcode 0x11 (crates/linkctl/src/lib.rs:37). INPUTS cannot
        // carry a demand at all: its throttle word was an ADC mirror with no consumer and is
        // deleted (specs/todo.md part 3), so the payload is two level bytes.
        val drive = pdus.single { it.opcode == OP_DRIVE_CMD }
        val decoded = checkNotNull(DriveCmd.decode(drive.payload))
        assertEquals(DriveKind.Throttle, decoded.kind)
        assertEquals(2_000, decoded.value)
        assertEquals(0, decoded.steer)

        // ... and the INPUTS frame is levels only, so there is exactly one word in this tick that
        // means "go", not two that could disagree.
        val inputsPayload = pdus.single { it.opcode == OP_INPUTS }.payload
        assertEquals(Inputs.LEN, inputsPayload.size)
        val inputs = checkNotNull(Inputs.decode(inputsPayload))
        assertTrue(inputs.powerRequest(), "armed asserts the power-request level")
    }

    @Test
    fun `the demand is on the DRIVE_CMD full scale, not the firmware's thousand-domain`() {
        // The frame-in adapter maps value onto +-CMD_LIMIT by `value * 1000 / FULL_SCALE`
        // (crates/control/src/throttle.rs:138), so full pad travel has to be a fraction of
        // FULL_SCALE. Reading the scale as 1000 would command a thirty-third of what was meant.
        assertEquals(DriveCmd.FULL_SCALE, Throttle.MAX_SPEED)

        val full = checkNotNull(
            DriveCmd.decode(
                pdusOf(RiderCommand.armed(Throttle.MAX_SPEED)).single { it.opcode == OP_DRIVE_CMD }.payload,
            ),
        )
        // Full scale, which the firmware's own arithmetic turns into the full 1000-domain command.
        // Above the +-33 truncation floor and well above the +-590 the engagement gate needs, both
        // re-derived from the Rust by protocol-kotlin's drift gate.
        assertEquals(32_767, full.value)
        assertTrue(full.value > 590, "full travel must be able to engage a stopped machine")
    }

    @Test
    fun `arming sets INPUTS buttons bit0 and releasing clears it`() {
        val armed = checkNotNull(
            Inputs.decode(pdusOf(RiderCommand.armed(0)).single { it.opcode == OP_INPUTS }.payload),
        )
        assertTrue(armed.powerRequest(), "power_request must be asserted while armed")
        assertEquals(Inputs.BUTTON_POWER, armed.buttons and Inputs.BUTTON_POWER)

        val disarmed = checkNotNull(
            Inputs.decode(pdusOf(RiderCommand.DISARMED).single { it.opcode == OP_INPUTS }.payload),
        )
        assertFalse(disarmed.powerRequest(), "power_request must clear on release")
        assertEquals(0, disarmed.buttons)
    }

    /**
     * The app does not claim a rider is aboard.
     *
     * It used to: `rider` was a copy of the arm level, on the argument that the arm control was the
     * rider deadman. The board ORs the mirror with its own foot pads and the peer's flag
     * (`orchestrator::dispatch::rider_level`) and the result picks the PID profile, rider-present
     * selecting the stiff profile A. So an app asserting rider whenever it is armed pins every board
     * to profile A and the pads can never select anything, which is the phone overriding a decision
     * it has no sensor for.
     */
    @Test
    fun `an armed command claims no rider unless the bench toggle asks for one`() {
        val plain = checkNotNull(
            Inputs.decode(pdusOf(RiderCommand.armed(2_000)).single { it.opcode == OP_INPUTS }.payload),
        )
        assertTrue(plain.powerRequest(), "arming is still the app's business")
        assertFalse(plain.riderPresent(), "the app has no pad sensor and must not claim one")
        assertEquals(0, plain.rider)

        val simulated = checkNotNull(
            Inputs.decode(
                pdusOf(RiderCommand.armed(2_000, simulatingRider = true))
                    .single { it.opcode == OP_INPUTS }.payload,
            ),
        )
        assertTrue(simulated.riderPresent(), "the bench toggle is the one way the app asserts rider")
        assertEquals(Inputs.RIDER_PRESENT, simulated.rider and Inputs.RIDER_PRESENT)
    }

    @Test
    fun `the all-stop command claims nothing at all`() {
        val disarmed = checkNotNull(
            Inputs.decode(pdusOf(RiderCommand.DISARMED).single { it.opcode == OP_INPUTS }.payload),
        )
        assertFalse(disarmed.powerRequest())
        assertFalse(disarmed.riderPresent(), "the all-stop frame states nothing, toggle or no toggle")
    }

    @Test
    fun `a disarmed command is inert in both payloads`() {
        val pdus = pdusOf(RiderCommand.DISARMED)
        val drive = checkNotNull(DriveCmd.decode(pdus.single { it.opcode == OP_DRIVE_CMD }.payload))
        // Neutral kind AND a zero value: the firmware reads a Neutral command as a literal (0, 0)
        // whatever it carries (crates/orchestrator/src/dispatch.rs:170-175), and the zero means a
        // decoder that lost the kind byte still reads no demand.
        assertEquals(DriveKind.Neutral, drive.kind)
        assertEquals(0, drive.value)
    }

    @Test
    fun `a disarmed command carrying a demand is not representable`() {
        // The invariant that keeps disarm from ever being a two-frame sequence with a live demand
        // in the middle: there is no way to build one. DISARMED is the only disarmed value.
        assertFalse(RiderCommand.DISARMED.armed)
        assertEquals(0, RiderCommand.DISARMED.demand)
        assertTrue(RiderCommand.armed(1).armed)
    }

    @Test
    fun `both payloads go out every tick, INPUTS first`() {
        // One tick is both halves or neither, and the arm level takes effect before the demand.
        for (command in listOf(RiderCommand.DISARMED, RiderCommand.armed(500))) {
            val opcodes = pdusOf(command).map { it.opcode }
            assertEquals(listOf(OP_INPUTS, OP_DRIVE_CMD), opcodes)
        }
    }

    @Test
    fun `the demand is clamped to the wire scale`() {
        assertEquals(DriveCmd.FULL_SCALE, RiderCommand.armed(Int.MAX_VALUE).demand)
        assertEquals(-DriveCmd.FULL_SCALE, RiderCommand.armed(Int.MIN_VALUE).demand)
    }

    /**
     * The wire sizes the link budget rests on, measured through the real L2 stack rather than
     * asserted in a comment. They were asserted in comments, and were wrong by the frag-hdr byte.
     *
     * They are load-bearing twice: 20 Hz x 13 B + 2 Hz x 10 B = ~280 B/s is what says the armed
     * cruise sits under the module's ~360 B/s overrun ceiling, and 13 <= 20 is what says each frame
     * is one ATT write rather than a split the CC2541 would re-chunk.
     *
     * The INPUTS frame was 12 B until its throttle word was deleted (specs/todo.md part 3), which
     * is where the 2 B went. The budget only got cheaper, and the keepalive was never the binding
     * half of it.
     */
    @Test
    fun `one tick is 10 and 13 bytes on the wire, one ATT write each`() {
        val transport = BleStreamTransport()
        val link = Link(transport)
        val command = RiderCommand.armed(1_000)

        link.send(command.inputsPdu(src, dst))
        val inputsFrame = checkNotNull(transport.drainOutgoing())
        link.send(command.drivePdu(src, dst))
        val driveFrame = checkNotNull(transport.drainOutgoing())

        // SOF + len + frag-hdr + PDU + CRC16 = PDU + 5.
        assertEquals(command.inputsPdu(src, dst).size + 5, inputsFrame.size)
        assertEquals(command.drivePdu(src, dst).size + 5, driveFrame.size)
        assertEquals(10, inputsFrame.size)
        assertEquals(13, driveFrame.size)
        assertTrue(driveFrame.size <= ATT_WRITE, "a tick's frame must fit a single ATT write")
    }

    @Test
    fun `both frames are addressed from the app to the board`() {
        for (pdu in pdusOf(RiderCommand.armed(100))) {
            assertEquals(src, pdu.src)
            assertEquals(dst, pdu.dst)
        }
    }

    private companion object {
        /** Bytes per GATT write, mirroring `BleHoverboardTransport.ATT_CHUNK`. */
        const val ATT_WRITE = 20
    }
}
