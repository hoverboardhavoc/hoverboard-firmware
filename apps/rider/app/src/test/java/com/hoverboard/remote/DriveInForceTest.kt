package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.RiderCommand
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

/**
 * The mode-in-force rules of `specs/rider-ui.md` 3.2 and 3.1c, on [DriveInForce.of] alone: the
 * stored mode when every copy agrees and its precondition holds, else a refusal with the reason and
 * no boards, keeping the stored mode where one is known; and the motor controllers each mode
 * addresses. A bridge is never among them.
 */
class DriveInForceTest {

    private val mc = Fields.NodeClass.MOTOR_CONTROLLER
    private val pair = listOf(NodeInfo(0x01, mc, "r"), NodeInfo(0x02, mc, "l"))

    private fun of(
        nodes: List<NodeInfo> = pair,
        modes: Map<Int, Int?> = mapOf(0x01 to 0, 0x02 to 0),
        sides: Map<Int, Int?> = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to Fields.ControlSide.LEFT),
        leaders: Set<Int> = emptySet(),
        selected: Int? = 0x01,
    ) = DriveInForce.of(nodes, { modes[it] }, { sides[it] }, leaders, selected)

    private fun both(mode: DriveMode) = mapOf(0x01 to mode.byte, 0x02 to mode.byte)

    @Test
    fun `agreeing copies whose precondition holds are in force`() {
        assertEquals(DriveTargets(DriveMode.SINGLE, listOf(0x01)), of().targets)
        assertEquals(DriveTargets(DriveMode.BOUND, listOf(0x01, 0x02)), of(modes = both(DriveMode.BOUND)).targets)
        assertEquals(
            DriveTargets(DriveMode.DIFFERENTIAL, listOf(0x01, 0x02), right = 0x01),
            of(modes = both(DriveMode.DIFFERENTIAL)).targets,
        )
        val rigid = of(modes = both(DriveMode.RIGID), leaders = setOf(0x02))
        assertEquals(DriveTargets(DriveMode.RIGID, listOf(0x01, 0x02)), rigid.targets)
        for (mode in DriveMode.entries) {
            val inForce = of(modes = both(mode), leaders = setOf(0x01))
            assertNull(inForce.refusal, "$mode")
            assertEquals(mode, inForce.stored)
        }
    }

    @Test
    fun `single drives the selected motor controller and nothing that is not one`() {
        assertEquals(listOf(0x02), of(selected = 0x02).targets.boards)
        assertEquals(emptyList<Int>(), of(selected = null).targets.boards)
        val withBridge =
            listOf(NodeInfo(0x01, Fields.NodeClass.BRIDGE, "esp")) + pair.map { it.copy(address = it.address + 1) }
        val single = of(nodes = withBridge, modes = mapOf(0x02 to 0, 0x03 to 0), selected = 0x01)
        assertEquals(emptyList<Int>(), single.targets.boards, "a bridge selected is no board to drive")
    }

    @Test
    fun `a disagreement, an unread copy or an unknown byte is a refusal with no boards`() {
        assertEquals(DriveRefusal.DISAGREE, of(modes = mapOf(0x01 to 1, 0x02 to 2)).refusal)
        assertEquals(DriveRefusal.UNREAD, of(modes = mapOf(0x01 to 1, 0x02 to null)).refusal)
        assertEquals(DriveRefusal.UNKNOWN_MODE, of(modes = mapOf(0x01 to 7, 0x02 to 7)).refusal)
        val unreadClass = listOf(NodeInfo(0x01, mc, "r"), NodeInfo(0x02, null, null))
        assertEquals(DriveRefusal.UNREAD, of(nodes = unreadClass).refusal, "a node that may be a motor controller")
        // Nothing is known to be stored, so the pad is SINGLE's, and nothing is armed.
        val unknown = listOf(
            of(modes = mapOf(0x01 to 1, 0x02 to 2)),
            of(nodes = unreadClass),
            of(modes = mapOf(0x01 to 7, 0x02 to 7)),
        )
        for (f in unknown) {
            assertEquals(DriveTargets(DriveMode.SINGLE, emptyList()), f.targets)
            assertNull(f.stored)
        }
    }

    @Test
    fun `each precondition that fails is a refusal that keeps the stored mode for the pad`() {
        val one = pair.take(1)
        for (mode in listOf(DriveMode.BOUND, DriveMode.DIFFERENTIAL, DriveMode.RIGID)) {
            val f = of(nodes = one, modes = mapOf(0x01 to mode.byte), leaders = setOf(0x01))
            assertEquals(DriveRefusal.NOT_TWO, f.refusal, "$mode on one motor controller")
            // The app never drives a mode the machine does not store: no fallback to SINGLE.
            assertEquals(DriveTargets(mode, emptyList()), f.targets, "$mode keeps its pad and arms nothing")
            assertEquals(mode, f.stored)
        }
        val stored = listOf(
            of(modes = both(DriveMode.RIGID)),
            of(modes = both(DriveMode.DIFFERENTIAL), sides = emptyMap()),
        )
        for (f in stored) {
            assertTrue(f.targets.boards.isEmpty())
            assertEquals(f.stored, f.mode)
        }
        assertEquals(DriveRefusal.NO_LEADER, of(modes = both(DriveMode.RIGID)).refusal)
        assertEquals(DriveRefusal.NO_LEADER, of(modes = both(DriveMode.RIGID), leaders = setOf(0x01, 0x02)).refusal)
        val noRight = mapOf(0x01 to Fields.ControlSide.LEFT, 0x02 to Fields.ControlSide.LEFT)
        val twoRight = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to Fields.ControlSide.RIGHT)
        val unreadSide = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to null)
        for (sides in listOf(noRight, twoRight, unreadSide)) {
            assertEquals(DriveRefusal.NO_RIGHT_WHEEL, of(modes = both(DriveMode.DIFFERENTIAL), sides = sides).refusal)
        }
    }

    @Test
    fun `a frame addresses exactly the boards its targets name`() {
        val armed = RiderCommand.armed(1_000)
        val bound = DriveFrame.of(DriveTargets(DriveMode.BOUND, listOf(0x02, 0x03)), armed)
        assertEquals(setOf(0x02, 0x03), bound.commands.keys)
        assertTrue(DriveFrame.of(DriveTargets.NONE, armed).commands.isEmpty())
        // A rigid frame with no leader among its boards drives nobody and still tells both.
        val rigid = DriveFrame.of(DriveTargets(DriveMode.RIGID, listOf(0x02, 0x03)), armed, steer = 5, leader = 0x01)
        assertEquals(setOf(0x02, 0x03), rigid.commands.keys)
        assertTrue(rigid.driven.isEmpty())
    }

    @Test
    fun `targets that cannot be driven are not constructed`() {
        assertThrows<IllegalArgumentException> { DriveTargets(DriveMode.SINGLE, listOf(0x01, 0x02)) }
        assertThrows<IllegalArgumentException> { DriveTargets(DriveMode.DIFFERENTIAL, listOf(0x01, 0x02)) }
        // A refused DIFFERENTIAL names its mode and addresses nobody, so it needs no right wheel.
        assertTrue(DriveTargets(DriveMode.DIFFERENTIAL, emptyList()).boards.isEmpty())
    }
}
