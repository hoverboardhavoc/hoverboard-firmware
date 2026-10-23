package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [SetupModel] with a second board in the session (`specs/rider-ui.md` 3.7, the Setup board
 * selector): each board is its own store at its own address, with its own basket, staged marks and
 * attitude, and the two must not be read for each other. The scheduler is turned as in
 * [SetupModelTest].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupSlaveTargetTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val limit = SetupFields.MOTOR_CURRENT_LIMIT.key

    /** The one boot-read field here: what the power-cycle instruction is for, the rest being arm-time. */
    private val deadTime = SetupFields.MOTOR_DEAD_TIME.key
    private val trims = SetupFields.LEVEL_TRIM.map { it.key }

    private fun TestScope.shown(): SetupRig = shownRig(board)

    private fun telemetry(rig: SetupRig, pitch: Int, roll: Int, from: Int? = null) =
        rig.transport.emitCyclicState(CyclicState(pitch, roll, 0, 0, 0, 0, 0, 0, QUIET_OBS), from)

    /**
     * The board selector (`specs/rider-ui.md` 3.7): the slave is its own store at its own address,
     * reached through the master, and each board's basket is its own.
     */
    @Test
    fun `selecting the slave reads and writes its own store at its own address, apart from the master's`() = runTest {
        val slave = 0x02
        val rig = shownRig(board, slave)
        rig.transport.store[slave to mode] = Value.U8(1)
        rig.model.stage(mode, Value.U8(1))
        val readsBefore = rig.transport.reads.size

        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        assertEquals(slave, rig.state.board)
        assertEquals(SetupFields.ALL.map { slave to it.key }, rig.transport.reads.drop(readsBefore))
        assertEquals(Value.U8(1), rig.state.values[mode], "the slave's own value")
        assertTrue(rig.state.pending.isEmpty(), "the master's basket showed under the slave")

        rig.model.stage(limit, Value.U32(15_000))
        rig.model.apply()
        runCurrent()
        assertEquals(listOf(Triple(slave, limit, Value.U32(15_000))), rig.transport.writes)
        assertEquals(setOf(limit), rig.state.staged.keys)

        rig.model.selectTarget(Node.MASTER)
        runCurrent()
        assertEquals(board, rig.state.board)
        assertEquals(mapOf(mode to Value.U8(1)), rig.state.pending, "the master's basket was lost")
        assertTrue(rig.state.staged.isEmpty(), "the slave's staged mark showed under the master")
        assertEquals(readsBefore + SetupFields.ALL.size, rig.transport.reads.size, "the master was read again")
    }

    @Test
    fun `the slave cannot be selected until discovery has found one`() = runTest {
        val rig = shown()

        rig.model.selectTarget(Node.SLAVE)

        assertEquals(Node.MASTER, rig.state.target)
        assertEquals(board, rig.state.board)
    }

    /**
     * The slave is levelled and checked off ITS OWN attitude, which reaches the app under its own
     * address. The master is leaning 3 degrees at the same moment and none of that is the slave's:
     * the trim staged here zeroes the slave's reading, and the level check passes on the slave's
     * reading while the master's would have failed it.
     */
    @Test
    fun `set level and the rotation check run for the slave off the slave's own attitude`() = runTest {
        val slave = 0x02
        val rig = shownRig(board, slave = slave)
        telemetry(rig, pitch = 305, roll = -50)
        telemetry(rig, pitch = 12, roll = -4, from = slave)
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        rig.model.setLevel()
        assertEquals(
            mapOf(trims[0] to Value.I16(12), trims[1] to Value.I16(-4)),
            rig.state.pending,
        )

        rig.model.checkLevel()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.level)
    }

    /**
     * A board that has not reported blocks the flows whichever board it is: there is no other
     * board's reading to stand in for it.
     */
    @Test
    fun `set level and the rotation check are refused for a slave that has not reported`() = runTest {
        val rig = shownRig(board, slave = 0x02)
        telemetry(rig, pitch = 305, roll = -50)
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        rig.model.setLevel()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
        assertTrue(rig.state.pending.isEmpty())

        rig.model.checkLevel()
        assertEquals(SetupNotice.CheckUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
        assertNull(rig.state.rotationCheck.level)
    }

    /** The slave's power-cycle leaves the phone link up, so it is taken on the operator's word; the master's is not. */
    @Test
    fun `a slave power-cycle is confirmed without the link dropping`() = runTest {
        val rig = shownRig(board, slave = 0x02)
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()
        rig.model.stage(deadTime, Value.U8(32))
        rig.model.apply()
        runCurrent()
        assertTrue(rig.state.awaitingPowerCycle)
        assertTrue(rig.state.canConfirmPowerCycle)

        rig.model.confirmPowerCycled()
        runCurrent()
        assertTrue(rig.state.staged.isEmpty())
        assertFalse(rig.state.awaitingPowerCycle)

        rig.model.selectTarget(Node.MASTER)
        runCurrent()
        rig.model.stage(deadTime, Value.U8(32))
        rig.model.apply()
        runCurrent()
        assertTrue(rig.state.awaitingPowerCycle)
        assertFalse(rig.state.canConfirmPowerCycle, "the master's confirmation needs the drop")
    }

    @Test
    fun `a reconnect to the same pair keeps the slave's staged marks, and a different slave drops them`() = runTest {
        val rig = shownRig(board, slave = 0x02)
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board, slave = 0x02)
        runCurrent()
        assertEquals(Node.SLAVE, rig.state.target)
        assertEquals(setOf(mode), rig.state.staged.keys)
        assertTrue(rig.state.linkDroppedSinceApply)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board, slave = 0x03)
        runCurrent()
        assertEquals(0x03, rig.state.board)
        assertTrue(rig.state.staged.isEmpty())
        assertEquals(SetupNotice.BoardChanged(0x02), rig.state.notice)
    }

    @Test
    fun `a session that finds no slave falls back to the master`() = runTest {
        val rig = shownRig(board, slave = 0x02)
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()

        assertEquals(Node.MASTER, rig.state.target)
        assertEquals(board, rig.state.board)
    }
}
