package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The Machine section's rows (`specs/rider-ui.md` 3.2 and 3.7a): `machine.drive_mode` and
 * `control.frame` written to every motor controller with the same value, `control.side` written with
 * different values and in an order that cannot leave two right wheels, the verdicts the screen
 * shows, and the statements the arm control makes about them.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupMachineTest {

    /** The board on the phone's link, and the one the walk found behind it. */
    private val b1 = 0x01
    private val b2 = 0x02
    private val frame = SetupFields.CONTROL_FRAME.key
    private val side = SetupFields.CONTROL_SIDE.key
    private val mode = SetupFields.DRIVE_MODE.key
    private val rigid = Value.U8(Fields.ControlFrame.RIGID)
    private val left = Value.U8(Fields.ControlSide.LEFT)
    private val right = Value.U8(Fields.ControlSide.RIGHT)

    /** A rig attached to the pair, with each board's store seeded, past its ride-facts read. */
    private fun TestScope.pair(
        store1: Map<Key, Value> = emptyMap(),
        store2: Map<Key, Value> = emptyMap(),
        discovered: Int? = b2,
    ): SetupRig = SetupRig(this).also { rig ->
        store1.forEach { (k, v) -> rig.transport.store[b1 to k] = v }
        store2.forEach { (k, v) -> rig.transport.store[b2 to k] = v }
        rig.transport.setAttachedBoard(b1, discovered)
        runCurrent()
    }

    // --- what the pair reads --------------------------------------------------------------------

    @Test
    fun `the ride facts read the frame and the side on both boards`() = runTest {
        val rig = pair()
        val reads = rig.transport.reads
        for (board in listOf(b1, b2)) {
            assertTrue(board to frame in reads, "the frame was not read on 0x%02X".format(board))
            assertTrue(board to side in reads, "the side was not read on 0x%02X".format(board))
        }
    }

    @Test
    fun `a fresh pair agrees on pivoting and has no right wheel, and the arm says nothing`() = runTest {
        val rig = pair()
        assertEquals(FramePair.Agree(rigid = false), rig.state.machine.frame)
        assertEquals(SidePair.NoneRight, rig.state.machine.side)
        assertEquals(MachineArm.None, rig.state.machine.armStatement)
        assertFalse(rig.state.machine.disagrees, "an unconfigured pair is the factory state, not a disagreement")
    }

    @Test
    fun `a byte that is not 1 reads as pivoting, so a 0 and a 7 do not differ`() = runTest {
        val rig = pair(store1 = mapOf(frame to Value.U8(7)))
        assertEquals(FramePair.Agree(rigid = false), rig.state.machine.frame)
        assertEquals(MachineArm.None, rig.state.machine.armStatement)
    }

    @Test
    fun `a session with one motor controller reads both verdicts unknown and warns at the arm`() = runTest {
        val rig = pair(discovered = null)
        assertEquals(FramePair.Unknown, rig.state.machine.frame)
        assertEquals(SidePair.Unknown, rig.state.machine.side)
        assertEquals(MachineArm.FrameUnread(peerFound = false), rig.state.machine.armStatement)
        assertFalse(rig.state.machine.armStatement.refuses, "an unknown is a warning, never a refusal")
    }

    @Test
    fun `a frame read that times out leaves the verdict unknown, and the warning says read again`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += frame
        rig.transport.setAttachedBoard(b1, b2)
        runCurrent()
        assertEquals(FramePair.Unknown, rig.state.machine.frame)
        assertEquals(MachineArm.FrameUnread(peerFound = true), rig.state.machine.armStatement)
    }

    // --- the arm control --------------------------------------------------------------------

    @Test
    fun `two different frames refuse the arm and name each board's value`() = runTest {
        val rig = pair(store1 = mapOf(frame to rigid))
        assertEquals(FramePair.Differ(mapOf(b1 to true, b2 to false)), rig.state.machine.frame)
        assertEquals(MachineArm.FramesDiffer(mapOf(b1 to true, b2 to false)), rig.state.machine.armStatement)
        assertTrue(rig.state.machine.armStatement.refuses)
        assertTrue(rig.state.machine.disagrees)
    }

    @Test
    fun `a rigid pair with no right wheel warns, and the same pair pivoting says nothing`() = runTest {
        val both = mapOf(frame to rigid)
        val rig = pair(store1 = both, store2 = both)
        assertEquals(MachineArm.SideNotOneRight(SidePair.NoneRight), rig.state.machine.armStatement)
        assertFalse(rig.state.machine.armStatement.refuses, "the side is never a refusal")

        val pivot = pair()
        assertEquals(MachineArm.None, pivot.state.machine.armStatement, "the side means nothing in pivoting mode")
    }

    @Test
    fun `two right wheels are a disagreement whichever frame the pair runs`() = runTest {
        val rig = pair(store1 = mapOf(side to right), store2 = mapOf(side to right))
        assertEquals(SidePair.BothRight, rig.state.machine.side)
        assertTrue(rig.state.machine.disagrees)
        assertEquals(MachineArm.None, rig.state.machine.armStatement, "pivoting, so the arm has nothing to say")
    }

    @Test
    fun `a correctly staged rigid pair says nothing at all`() = runTest {
        val rig = pair(
            store1 = mapOf(frame to rigid, side to right),
            store2 = mapOf(frame to rigid, side to left),
        )
        assertEquals(SidePair.OneRight(b1), rig.state.machine.side)
        assertEquals(MachineArm.None, rig.state.machine.armStatement)
        assertFalse(rig.state.machine.disagrees)
    }

    // --- writing the pair ---------------------------------------------------------------------

    @Test
    fun `the frame goes to every motor controller with the same value`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        flush()

        assertEquals(
            listOf(Triple(b1, frame, rigid), Triple(b2, frame, rigid)),
            rig.transport.writes,
        )
        assertEquals(FramePair.Agree(rigid = true), rig.state.machine.frame)
    }

    @Test
    fun `the right wheel sets every other board to the left FIRST`() = runTest {
        val rig = pair(store2 = mapOf(side to right))
        rig.model.chooseRightWheel(b1)
        flush()

        assertEquals(
            listOf(Triple(b2, side, left), Triple(b1, side, right)),
            rig.transport.writes,
            "the left wheel is written first, so an interrupted write cannot leave two right wheels",
        )
        assertEquals(SidePair.OneRight(b1), rig.state.machine.side)
    }

    /**
     * Re-picking inside the settle window writes what the pair still needs and nothing more: the
     * queue holds the latest choice per board, and a board already on the value it would be given
     * is left alone rather than having a flash record appended for nothing.
     */
    @Test
    fun `re-picking inside the settle window writes only what the pair still needs`() = runTest {
        val rig = pair(store1 = mapOf(side to right), store2 = mapOf(side to right))
        rig.model.chooseRightWheel(b2)
        rig.model.chooseRightWheel(b1)
        flush()

        assertEquals(listOf(Triple(b2, side, left)), rig.transport.writes)
        assertEquals(SidePair.OneRight(b1), rig.state.machine.side)
    }

    @Test
    fun `a side write stopped after the first board leaves no right wheel, not two`() = runTest {
        val rig = pair(store1 = mapOf(side to right), store2 = mapOf(side to left))
        // The second write of the pair is the one that is refused.
        var seen = 0
        rig.transport.writeHook = { k, _ ->
            if (k == side && seen++ == 1) Refused(CfgRefusal.STORE_ERR) else null
        }
        rig.model.chooseRightWheel(b2)
        flush()

        assertEquals(left, rig.transport.store[b1 to side], "board 0x01 was set to the left wheel")
        assertEquals(left, rig.transport.store[b2 to side], "board 0x02's write was refused, so it stayed left")
        assertEquals(SidePair.NoneRight, rig.state.machine.side)
        val refused = WriteState.Refused(CfgRefusal.STORE_ERR)
        assertEquals(refused, rig.state.setupOf(b2).writes[side]?.state, "the refusal is on board 0x02's row")
    }

    /**
     * The ordering guarantee under write-on-change: the chosen board's write FOLLOWS the other
     * board's, so a left-wheel write that did not land holds the right-wheel write back rather
     * than leaving the pair with two right wheels.
     */
    @Test
    fun `the right wheel is held back when the left wheel write did not land`() = runTest {
        val rig = pair(store1 = mapOf(side to right), store2 = mapOf(side to left))
        rig.transport.writeHook = { k, v -> if (k == side && v == left) Refused(CfgRefusal.STORE_ERR) else null }
        rig.model.chooseRightWheel(b2)
        flush()

        assertEquals(
            listOf(Triple(b1, side, left)),
            rig.transport.writes,
            "the right wheel went out although the left wheel had not landed",
        )
        assertEquals(right, rig.transport.store[b1 to side], "board 0x01 still holds the right wheel")
        assertEquals(WriteState.Skipped, rig.state.setupOf(b2).writes[side]?.state)
    }

    /** Each tap is its own write, in the order the taps were made; the side keeps its own order inside that. */
    @Test
    fun `each choice writes as it is made, and the side keeps its internal order`() = runTest {
        val rig = pair(store2 = mapOf(side to right))
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.chooseRightWheel(b1)
        flush()

        assertEquals(listOf(frame, frame, side, side), rig.transport.writes.map { it.second })
        assertEquals(
            listOf(Triple(b2, side, left), Triple(b1, side, right)),
            rig.transport.writes.filter { it.second == side },
        )
    }

    @Test
    fun `with one motor controller the write reaches it and the verdict stays unknown`() = runTest {
        val rig = pair(discovered = null)
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.chooseRightWheel(b1)
        flush()

        assertEquals(
            listOf(Triple(b1, frame, rigid), Triple(b1, side, right)),
            rig.transport.writes,
        )
        assertEquals(FramePair.Unknown, rig.state.machine.frame)
        assertEquals(SidePair.Unknown, rig.state.machine.side)
    }

    @Test
    fun `a board the walk did not find cannot be picked as the right wheel`() = runTest {
        val rig = pair(discovered = null)
        rig.model.chooseRightWheel(b2)
        flush()

        assertTrue(rig.transport.writes.isEmpty())
        assertTrue(rig.state.setupOf(b1).writes.isEmpty())
    }

    @Test
    fun `a machine choice while armed sends nothing and records nothing`() = runTest {
        val rig = pair()
        rig.armed = true
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        flush()

        assertTrue(rig.transport.writes.isEmpty())
        assertTrue(rig.state.setupOf(b1).writes.isEmpty())
        assertTrue(rig.state.setupOf(b2).writes.isEmpty())
    }

    @Test
    fun `a detach leaves both pair verdicts unknown`() = runTest {
        val rig = pair(store1 = mapOf(frame to rigid), store2 = mapOf(frame to rigid))
        assertEquals(FramePair.Agree(rigid = true), rig.state.machine.frame)

        rig.transport.setAttachedBoard(null)
        assertEquals(Machine.UNKNOWN, rig.state.machine)
    }

    /**
     * Both fields are in the arm-time value row, so a verified write is done: it takes effect at
     * the next arm and the screen must not ask for a restart over it.
     */
    @Test
    fun `a written machine field is recorded per board and asks for no restart`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        flush()

        for (node in listOf(b1, b2)) {
            assertEquals(WriteState.Written, rig.state.setupOf(node).writes[frame]?.state)
        }
        assertEquals(TakesEffect.NEXT_ARM, SetupFields.CONTROL_FRAME.takesEffect)
        assertEquals(TakesEffect.NEXT_ARM, SetupFields.CONTROL_SIDE.takesEffect)
    }

    // --- the drive mode (`specs/rider-ui.md` 3.2) ---------------------------------------------

    @Test
    fun `the ride facts read the drive mode on every motor controller`() = runTest {
        val rig = pair()
        for (board in listOf(b1, b2)) {
            assertTrue(board to mode in rig.transport.reads, "the drive mode was not read on 0x%02X".format(board))
        }
        assertEquals(ModePair.Agree(Fields.MachineDriveMode.SINGLE), rig.state.machine.mode)
    }

    @Test
    fun `the drive mode goes to every motor controller with the same value`() = runTest {
        val rig = pair()
        rig.model.chooseDriveMode(Fields.MachineDriveMode.RIGID)
        flush()

        val rigidMode = Value.U8(Fields.MachineDriveMode.RIGID)
        assertEquals(listOf(Triple(b1, mode, rigidMode), Triple(b2, mode, rigidMode)), rig.transport.writes)
        assertEquals(ModePair.Agree(Fields.MachineDriveMode.RIGID), rig.state.machine.mode)
        for (node in listOf(b1, b2)) assertEquals(WriteState.Written, rig.state.setupOf(node).writes[mode]?.state)
        assertEquals(TakesEffect.APP_ONLY, SetupFields.DRIVE_MODE.takesEffect, "nothing on the board reads it")
    }

    @Test
    fun `copies that disagree are badged, and one tap writes either value to all`() = runTest {
        val rig = pair(
            store1 = mapOf(mode to Value.U8(Fields.MachineDriveMode.BOUND)),
            store2 = mapOf(mode to Value.U8(Fields.MachineDriveMode.DIFFERENTIAL)),
        )
        assertEquals(
            ModePair.Differ(mapOf(b1 to Fields.MachineDriveMode.BOUND, b2 to Fields.MachineDriveMode.DIFFERENTIAL)),
            rig.state.machine.mode,
        )
        assertTrue(rig.state.machine.disagrees)

        rig.model.chooseDriveMode(Fields.MachineDriveMode.BOUND)
        flush()

        // Board 0x01 already holds BOUND, so only the other copy is written.
        assertEquals(listOf(Triple(b2, mode, Value.U8(Fields.MachineDriveMode.BOUND))), rig.transport.writes)
        assertEquals(ModePair.Agree(Fields.MachineDriveMode.BOUND), rig.state.machine.mode)
        assertFalse(rig.state.machine.disagrees)
    }

    @Test
    fun `a byte that names no drive mode is not written`() = runTest {
        val rig = pair()
        rig.model.chooseDriveMode(DRIVE_MODE_UNKNOWN)
        flush()
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `a bridge holds no machine setting and is written none`() = runTest {
        // The ESP at 0x01 on the phone's link, the two boards behind it.
        val rig = SetupRig(this)
        rig.transport.makeBridge(b1)
        rig.transport.setAttachedBoard(b1, b2, B3)
        runCurrent()

        assertEquals(listOf(b2, B3), rig.state.motorControllers)
        assertFalse(b1 to mode in rig.transport.reads, "a bridge was asked for a machine setting")
        rig.model.chooseDriveMode(Fields.MachineDriveMode.BOUND)
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.chooseRightWheel(B3)
        flush()

        assertTrue(rig.transport.writes.none { it.first == b1 }, "a bridge was written a machine setting")
        assertEquals(
            listOf(
                Triple(b2, mode, Value.U8(Fields.MachineDriveMode.BOUND)),
                Triple(B3, mode, Value.U8(Fields.MachineDriveMode.BOUND)),
                Triple(b2, frame, rigid),
                Triple(B3, frame, rigid),
                Triple(B3, side, right),
            ),
            rig.transport.writes,
        )
        assertEquals(SidePair.OneRight(B3), rig.state.machine.side)
    }

    private companion object {
        /** A third node, for the machine with an ESP bridge at the attach point. */
        const val B3 = 0x03

        /** A drive-mode byte outside the vocabulary. */
        const val DRIVE_MODE_UNKNOWN = 9
    }
}
