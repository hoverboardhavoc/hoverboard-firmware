package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [SetupModel] over a session with more than one node (`specs/rider-ui.md` section 2 and 3.4): the
 * node list read on attach, the selector, and each node as its own store at its own address, with
 * its own write records, boot tag and attitude, never read for another. The scheduler is turned as
 * in [SetupModelTest].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupNodeTargetTest {

    private val board = 0x01
    private val other = 0x02
    private val mode = SetupFields.CONTROL_MODE.key
    private val limit = SetupFields.MOTOR_CURRENT_LIMIT.key

    /** The one boot-read field here, so its row is the one that waits on a restart. */
    private val deadTime = SetupFields.MOTOR_DEAD_TIME.key
    private val trims = SetupFields.LEVEL_TRIM.map { it.key }
    private val nodeClass = SetupFields.NODE_CLASS.key
    private val name = SetupFields.DEVICE_NAME.key

    private fun telemetry(rig: SetupRig, pitch: Int, roll: Int, from: Int? = null) =
        rig.transport.emitCyclicState(CyclicState(pitch, roll, 0, 0, 0, 0, 0, 0, 0, QUIET_OBS, 0), from)

    // --- the node list (`specs/rider-ui.md` section 2) -----------------------------------------

    /**
     * The connect sequence through a bridge: the walk found a bridge on the phone's link and two motor
     * controllers behind it. Every node's class and name come first, and only the nodes that answered
     * motor controller are then asked for the machine's facts. The bridge is asked nothing else.
     */
    @Test
    fun `the attach read takes class and name from every node, then motor facts from motor controllers only`() =
        runTest {
            val rig = SetupRig(this)
            rig.transport.makeBridge(0x01)
            rig.transport.store[0x01 to name] = Value.Str("hb-bridge-1a2b")
            rig.transport.store[0x02 to name] = Value.Str("rover-r")
            rig.transport.store[0x03 to name] = Value.Str("rover-l")
            rig.transport.setAttachedBoard(0x01, 0x02, 0x03)
            runCurrent()

            val reads = rig.transport.reads
            assertEquals(
                listOf(0x01, 0x02, 0x03).flatMap { listOf(it to nodeClass, it to name) },
                reads.take(6),
                "what each node is, before anything else",
            )
            assertEquals(listOf(0x01 to nodeClass, 0x01 to name), reads.filter { it.first == 0x01 })
            for (field in SetupFields.MOTOR_FACTS) {
                val asked = reads.filter { it.second == field.key }.map { it.first }
                assertEquals(listOf(0x02, 0x03), asked, field.key.toString())
            }
            assertEquals(
                listOf(
                    NodeInfo(0x01, Fields.NodeClass.BRIDGE, "hb-bridge-1a2b"),
                    NodeInfo(0x02, Fields.NodeClass.MOTOR_CONTROLLER, "rover-r"),
                    NodeInfo(0x03, Fields.NodeClass.MOTOR_CONTROLLER, "rover-l"),
                ),
                rig.state.nodes,
            )
            assertEquals(listOf(0x02, 0x03), rig.state.motorControllers)
            assertEquals(0x02, rig.state.selectedMotor, "the bridge is not a motor controller to select")
        }

    /** The CC2541 path: the node on the phone's link is a motor controller, and stays the default. */
    @Test
    fun `through a board's own module the attached motor controller is the default selection`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(0x02, 0x01)
        runCurrent()

        assertEquals(listOf(0x01, 0x02), rig.state.motorControllers)
        assertEquals(0x02, rig.state.selectedMotor)
        assertEquals(0x02, rig.state.board)
        assertEquals(
            listOf(0x01, 0x02).flatMap { listOf(it to nodeClass, it to name) },
            rig.transport.reads.take(4),
        )
    }

    /** A node that does not answer its class shows its address alone and is asked nothing more. */
    @Test
    fun `a node whose class read has not answered shows its address alone`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += nodeClass
        rig.transport.setAttachedBoard(board, other)
        runCurrent()

        assertEquals(listOf(NodeInfo(board, null, null), NodeInfo(other, null, null)), rig.state.nodes)
        assertTrue(rig.transport.reads.none { it.second == name }, "a silent node is not asked its name")
        assertTrue(rig.state.motorControllers.isEmpty())
        assertNull(rig.state.selectedMotor)
    }

    // --- a bridge's Setup (`specs/rider-ui.md` 3.4) --------------------------------------------

    /**
     * A bridge shows the keys every node has: its name, its class read-only, and its link set. Its
     * read pass asks for nothing else, and the class row takes no edit.
     */
    @Test
    fun `a bridge shows and reads only the keys every node has`() = runTest {
        val rig = SetupRig(this)
        rig.transport.makeBridge(0x01)
        rig.transport.setAttachedBoard(0x01, 0x02)
        rig.model.onShown()
        rig.model.selectTarget(0x01)
        runCurrent()

        assertEquals(0x01, rig.state.board)
        assertEquals(SetupFields.BRIDGE, rig.state.rows)
        val bridgeReads = rig.transport.reads.filter { it.first == 0x01 }.map { it.second }.toSet()
        assertEquals(SetupFields.BRIDGE.map { it.key }.toSet(), bridgeReads)

        rig.model.edit(nodeClass, Value.U8(Fields.NodeClass.MOTOR_CONTROLLER))
        flush()
        assertTrue(rig.transport.writes.isEmpty(), "node.class is read-only on every node")
        assertEquals(0x02, rig.state.selectedMotor, "picking the bridge leaves the motor controller selected")
    }

    // --- the selector (`specs/rider-ui.md` 3.4) ------------------------------------------------

    @Test
    fun `selecting another node reads and writes its own store at its own address`() = runTest {
        val rig = shownRig(board, other)
        rig.transport.store[other to mode] = Value.U8(1)
        rig.model.edit(mode, Value.U8(1))
        flush()
        assertEquals(listOf(Triple(board, mode, Value.U8(1))), rig.transport.writes)
        val readsBefore = rig.transport.reads.size

        rig.model.selectTarget(other)
        runCurrent()

        assertEquals(other, rig.state.board)
        assertEquals(other, rig.state.selectedMotor, "picking a motor controller makes it the one SINGLE drives")
        assertEquals(SetupFields.ALL.map { other to it.key }, rig.transport.reads.drop(readsBefore))
        assertEquals(Value.U8(1), rig.state.values[mode], "the other board's own value")
        assertTrue(rig.state.writes.isEmpty(), "the first board's write record showed under the other")

        rig.model.edit(limit, Value.U32(15_000))
        flush()
        assertEquals(Triple(other, limit, Value.U32(15_000)), rig.transport.writes.last())

        rig.model.selectTarget(board)
        runCurrent()
        assertEquals(board, rig.state.board)
        assertEquals(setOf(mode), rig.state.writes.keys, "the first board's write record was lost")
        assertEquals(readsBefore + SetupFields.ALL.size, rig.transport.reads.size, "the first board was read again")
    }

    @Test
    fun `a node the walk did not find cannot be selected`() = runTest {
        val rig = shownRig(board)

        rig.model.selectTarget(other)

        assertNull(rig.state.target)
        assertEquals(board, rig.state.board)
    }

    /**
     * The other board is levelled and checked off ITS OWN attitude, which reaches the app under its
     * own address. The attached board is leaning 3 degrees at the same moment and none of that is the
     * other's.
     */
    @Test
    fun `set level and the rotation check run off the selected board's own attitude`() = runTest {
        val rig = shownRig(board, other)
        telemetry(rig, pitch = 305, roll = -50)
        telemetry(rig, pitch = 12, roll = -4, from = other)
        rig.model.selectTarget(other)
        runCurrent()

        rig.model.setLevel()
        flush()
        assertEquals(
            listOf(Triple(other, trims[0], Value.I16(12)), Triple(other, trims[1], Value.I16(-4))),
            rig.transport.writes,
        )

        rig.model.checkLevel()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.level)
    }

    @Test
    fun `set level and the rotation check are refused for a board that has not reported`() = runTest {
        val rig = shownRig(board, other)
        telemetry(rig, pitch = 305, roll = -50)
        rig.model.selectTarget(other)
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
     * Each board's restart is its own fact, read off the boot tag its own address keys. The other
     * board's restart puts its writes into effect and leaves the first board's record where it was.
     */
    @Test
    fun `a restart is read off each board's own boot tag`() = runTest {
        val rig = shownRig(board, other)
        rig.emitBoot(bootTag = 4)
        rig.emitBoot(bootTag = 9, from = other)
        runCurrent()
        rig.model.edit(deadTime, Value.U8(32))
        flush()

        rig.model.selectTarget(other)
        runCurrent()
        rig.model.edit(deadTime, Value.U8(32))
        flush()
        assertEquals(WriteState.Written, rig.state.writes[deadTime]?.state)

        rig.emitBoot(bootTag = 10, from = other)
        runCurrent()

        assertTrue(rig.state.setupOf(other).writes.isEmpty(), "the restart left its own record")
        assertEquals(WriteState.Written, rig.state.setupOf(board).writes[deadTime]?.state)
    }

    /** Write records are the board's at its address, and a reconnect finds them there. */
    @Test
    fun `a reconnect keeps each board's write records at its address`() = runTest {
        val rig = shownRig(board, other)
        rig.model.selectTarget(other)
        runCurrent()
        rig.model.edit(mode, Value.U8(1))
        flush()
        assertEquals(WriteState.Written, rig.state.writes[mode]?.state)

        rig.transport.setAttachedBoard(null)
        assertTrue(rig.state.nodes.isEmpty())
        rig.transport.setAttachedBoard(board, other)
        runCurrent()
        assertEquals(other, rig.state.board, "the selection outlives the drop")
        assertEquals(setOf(mode), rig.state.writes.keys)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board, 0x03)
        runCurrent()
        assertEquals(board, rig.state.board, "the node picked is gone, so the attached board is edited")
        assertFalse(rig.state.writes.containsKey(mode), "0x03 never had 0x02's record")
    }
}
