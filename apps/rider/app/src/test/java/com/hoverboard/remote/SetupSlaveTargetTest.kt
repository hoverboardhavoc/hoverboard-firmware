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
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [SetupModel] with a second board in the session (`specs/rider-ui.md` 3.7, the Setup board
 * selector): each board is its own store at its own address, with its own write records, boot tag
 * and attitude, and the two must not be read for each other. The scheduler is turned as in
 * [SetupModelTest].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupSlaveTargetTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val limit = SetupFields.MOTOR_CURRENT_LIMIT.key

    /** The one boot-read field here, so its row is the one that waits on a restart. */
    private val deadTime = SetupFields.MOTOR_DEAD_TIME.key
    private val trims = SetupFields.LEVEL_TRIM.map { it.key }

    private fun TestScope.shown(): SetupRig = shownRig(board)

    private fun telemetry(rig: SetupRig, pitch: Int, roll: Int, from: Int? = null) =
        rig.transport.emitCyclicState(CyclicState(pitch, roll, 0, 0, 0, 0, 0, 0, QUIET_OBS), from)

    /**
     * The board selector (`specs/rider-ui.md` 3.7): the slave is its own store at its own address,
     * reached through the master, and each board's write records are its own.
     */
    @Test
    fun `selecting the slave reads and writes its own store at its own address, apart from the master's`() = runTest {
        val slave = 0x02
        val rig = shownRig(board, slave)
        rig.transport.store[slave to mode] = Value.U8(1)
        rig.model.edit(mode, Value.U8(1))
        flush()
        assertEquals(listOf(Triple(board, mode, Value.U8(1))), rig.transport.writes)
        val readsBefore = rig.transport.reads.size

        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        assertEquals(slave, rig.state.board)
        assertEquals(SetupFields.ALL.map { slave to it.key }, rig.transport.reads.drop(readsBefore))
        assertEquals(Value.U8(1), rig.state.values[mode], "the slave's own value")
        assertTrue(rig.state.writes.isEmpty(), "the master's write record showed under the slave")

        rig.model.edit(limit, Value.U32(15_000))
        flush()
        assertEquals(Triple(slave, limit, Value.U32(15_000)), rig.transport.writes.last())
        assertEquals(setOf(limit), rig.state.writes.keys)

        rig.model.selectTarget(Node.MASTER)
        runCurrent()
        assertEquals(board, rig.state.board)
        assertEquals(setOf(mode), rig.state.writes.keys, "the master's write record was lost")
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
        flush()
        assertEquals(
            listOf(Triple(slave, trims[0], Value.I16(12)), Triple(slave, trims[1], Value.I16(-4))),
            rig.transport.writes,
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
        flush()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())

        rig.model.checkLevel()
        assertEquals(SetupNotice.CheckUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
        assertNull(rig.state.rotationCheck.level)
    }

    /**
     * Each board's restart is its own fact, read off the boot tag its own address keys (the peer
     * forward, `specs/link-control.md`, the BLE-port forward). The slave's restart puts the
     * slave's writes into effect and leaves the master's record exactly where it was.
     */
    @Test
    fun `a slave restart is read off the slave's own boot tag and leaves the master's record`() = runTest {
        val slave = 0x02
        val rig = shownRig(board, slave)
        rig.emitBoot(bootTag = 4)
        rig.emitBoot(bootTag = 9, from = slave)
        runCurrent()
        rig.model.edit(deadTime, Value.U8(32))
        flush()

        rig.model.selectTarget(Node.SLAVE)
        runCurrent()
        rig.model.edit(deadTime, Value.U8(32))
        flush()
        assertEquals(WriteState.Written, rig.state.writes[deadTime]?.state)

        rig.emitBoot(bootTag = 10, from = slave)
        runCurrent()

        assertTrue(rig.state.setupOf(Node.SLAVE).writes.isEmpty(), "the slave's restart left its own record")
        val master = rig.state.setupOf(Node.MASTER)
        assertEquals(WriteState.Written, master.writes[deadTime]?.state, "the master's record went with the slave's")
    }

    @Test
    fun `a reconnect to the same pair keeps the slave's write records, and a different slave drops them`() =
        runTest {
            val rig = shownRig(board, slave = 0x02)
            rig.model.selectTarget(Node.SLAVE)
            runCurrent()
            rig.model.edit(mode, Value.U8(1))
            flush()
            assertEquals(WriteState.Written, rig.state.writes[mode]?.state)

            rig.transport.setAttachedBoard(null)
            rig.transport.setAttachedBoard(board, slave = 0x02)
            runCurrent()
            assertEquals(Node.SLAVE, rig.state.target)
            assertEquals(setOf(mode), rig.state.writes.keys)

            rig.transport.setAttachedBoard(null)
            rig.transport.setAttachedBoard(board, slave = 0x03)
            runCurrent()
            assertEquals(0x03, rig.state.board)
            assertTrue(rig.state.writes.isEmpty())
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
