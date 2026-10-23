package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The Machine section's two rows (`specs/rider-ui.md` 3.7a): `control.frame` written to both boards
 * with the same value, `control.side` written to both with different values and in an order that
 * cannot leave two right wheels, the pair verdicts the screen shows, and the statements the arm
 * control makes about them.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupMachineTest {

    private val master = 0x01
    private val slave = 0x02
    private val frame = SetupFields.CONTROL_FRAME.key
    private val side = SetupFields.CONTROL_SIDE.key
    private val rigid = Value.U8(Fields.ControlFrame.RIGID)
    private val left = Value.U8(Fields.ControlSide.LEFT)
    private val right = Value.U8(Fields.ControlSide.RIGHT)

    /** A rig attached to the pair, with each board's store seeded, past its ride-facts read. */
    private fun TestScope.pair(
        masterStore: Map<Key, Value> = emptyMap(),
        slaveStore: Map<Key, Value> = emptyMap(),
        discovered: Int? = slave,
    ): SetupRig = SetupRig(this).also { rig ->
        masterStore.forEach { (k, v) -> rig.transport.store[master to k] = v }
        slaveStore.forEach { (k, v) -> rig.transport.store[slave to k] = v }
        rig.transport.setAttachedBoard(master, discovered)
        runCurrent()
    }

    // --- what the pair reads --------------------------------------------------------------------

    @Test
    fun `the ride facts read the frame and the side on both boards`() = runTest {
        val rig = pair()
        val reads = rig.transport.reads
        for (board in listOf(master, slave)) {
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
        val rig = pair(masterStore = mapOf(frame to Value.U8(7)))
        assertEquals(FramePair.Agree(rigid = false), rig.state.machine.frame)
        assertEquals(MachineArm.None, rig.state.machine.armStatement)
    }

    @Test
    fun `a session with no slave reads both verdicts unknown and warns at the arm`() = runTest {
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
        rig.transport.setAttachedBoard(master, slave)
        runCurrent()
        assertEquals(FramePair.Unknown, rig.state.machine.frame)
        assertEquals(MachineArm.FrameUnread(peerFound = true), rig.state.machine.armStatement)
    }

    // --- the arm control --------------------------------------------------------------------

    @Test
    fun `two different frames refuse the arm and name the master's value`() = runTest {
        val rig = pair(masterStore = mapOf(frame to rigid))
        assertEquals(FramePair.Differ(masterRigid = true), rig.state.machine.frame)
        assertEquals(MachineArm.FramesDiffer(masterRigid = true), rig.state.machine.armStatement)
        assertTrue(rig.state.machine.armStatement.refuses)
        assertTrue(rig.state.machine.disagrees)
    }

    @Test
    fun `a rigid pair with no right wheel warns, and the same pair pivoting says nothing`() = runTest {
        val both = mapOf(frame to rigid)
        val rig = pair(masterStore = both, slaveStore = both)
        assertEquals(MachineArm.SideNotOneRight(SidePair.NoneRight), rig.state.machine.armStatement)
        assertFalse(rig.state.machine.armStatement.refuses, "the side is never a refusal")

        val pivot = pair()
        assertEquals(MachineArm.None, pivot.state.machine.armStatement, "the side means nothing in pivoting mode")
    }

    @Test
    fun `two right wheels are a disagreement whichever frame the pair runs`() = runTest {
        val rig = pair(masterStore = mapOf(side to right), slaveStore = mapOf(side to right))
        assertEquals(SidePair.BothRight, rig.state.machine.side)
        assertTrue(rig.state.machine.disagrees)
        assertEquals(MachineArm.None, rig.state.machine.armStatement, "pivoting, so the arm has nothing to say")
    }

    @Test
    fun `a correctly staged rigid pair says nothing at all`() = runTest {
        val rig = pair(
            masterStore = mapOf(frame to rigid, side to right),
            slaveStore = mapOf(frame to rigid, side to left),
        )
        assertEquals(SidePair.OneRight(Node.MASTER), rig.state.machine.side)
        assertEquals(MachineArm.None, rig.state.machine.armStatement)
        assertFalse(rig.state.machine.disagrees)
    }

    // --- writing the pair ---------------------------------------------------------------------

    @Test
    fun `the frame goes to both boards with the same value`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.writeMachine()
        runCurrent()

        assertEquals(
            listOf(Triple(master, frame, rigid), Triple(slave, frame, rigid)),
            rig.transport.writes,
        )
        assertEquals(FramePair.Agree(rigid = true), rig.state.machine.frame)
        assertFalse(rig.state.machinePending.any, "a written choice is no longer pending")
    }

    @Test
    fun `the right wheel sets the other board to the left FIRST`() = runTest {
        val rig = pair()
        rig.model.chooseRightWheel(Node.MASTER)
        rig.model.writeMachine()
        runCurrent()

        assertEquals(
            listOf(Triple(slave, side, left), Triple(master, side, right)),
            rig.transport.writes,
            "the left wheel is written first, so an interrupted write cannot leave two right wheels",
        )
        assertEquals(SidePair.OneRight(Node.MASTER), rig.state.machine.side)
    }

    @Test
    fun `a side write stopped after the first board leaves no right wheel, not two`() = runTest {
        val rig = pair(masterStore = mapOf(side to right), slaveStore = mapOf(side to left))
        // The second write of the pair is the one that is refused.
        var seen = 0
        rig.transport.writeHook = { k, _ ->
            if (k == side && seen++ == 1) Refused(CfgRefusal.STORE_ERR) else null
        }
        rig.model.chooseRightWheel(Node.SLAVE)
        rig.model.writeMachine()
        runCurrent()

        assertEquals(left, rig.transport.store[master to side], "the master was set to the left wheel")
        assertEquals(left, rig.transport.store[slave to side], "the slave's write was refused, so it stayed left")
        assertEquals(SidePair.NoneRight, rig.state.machine.side)
        assertTrue(rig.state.notice is SetupNotice.BoardRefused, "the refusal is reported")
        assertEquals(Node.SLAVE, rig.state.machinePending.rightWheel, "the unwritten choice is still pending")
    }

    @Test
    fun `the side is written before the frame, the order a rover is configured in`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.chooseRightWheel(Node.MASTER)
        rig.model.writeMachine()
        runCurrent()

        assertEquals(listOf(side, side, frame, frame), rig.transport.writes.map { it.second })
    }

    @Test
    fun `with no slave the write reaches the attached board and the verdict stays unknown`() = runTest {
        val rig = pair(discovered = null)
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.chooseRightWheel(Node.MASTER)
        rig.model.writeMachine()
        runCurrent()

        assertEquals(
            listOf(Triple(master, side, right), Triple(master, frame, rigid)),
            rig.transport.writes,
        )
        assertEquals(FramePair.Unknown, rig.state.machine.frame)
        assertEquals(SidePair.Unknown, rig.state.machine.side)
    }

    @Test
    fun `the slave cannot be picked as the right wheel while discovery has found none`() = runTest {
        val rig = pair(discovered = null)
        rig.model.chooseRightWheel(Node.SLAVE)
        assertEquals(MachineEdit(), rig.state.machinePending)
    }

    @Test
    fun `a machine write is refused while armed, before anything is sent`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.armed = true
        rig.model.writeMachine()
        runCurrent()

        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(SetupNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals(Fields.ControlFrame.RIGID, rig.state.machinePending.frame, "the choice is kept")
    }

    @Test
    fun `a detach drops the unwritten machine choices`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.transport.setAttachedBoard(null)
        assertEquals(MachineEdit(), rig.state.machinePending)
        assertEquals(MachinePair.UNKNOWN, rig.state.machine)
    }

    /**
     * Both fields are in the arm-time value row, so a verified write is done: it takes effect at
     * the next arm and the screen must not ask for a restart over it.
     */
    @Test
    fun `a written machine field is staged but asks for no power-cycle`() = runTest {
        val rig = pair()
        rig.model.chooseFrame(Fields.ControlFrame.RIGID)
        rig.model.writeMachine()
        runCurrent()

        val board = checkNotNull(rig.state.boards[Node.MASTER])
        assertTrue(frame in board.staged, "the write was verified")
        assertTrue(board.stagedNeedingPowerCycle.isEmpty())
        assertFalse(board.awaitingPowerCycle, "nothing here needs a restart")
    }
}
