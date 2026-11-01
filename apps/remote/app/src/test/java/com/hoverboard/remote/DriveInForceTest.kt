package com.hoverboard.remote

import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.RiderCommand
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

/**
 * The derived drive mode (`specs/machine-settings.md` section 7 and section 9, test 6) on
 * [DriveInForce.of] alone: RIGID is `machine.mode` balance with `machine.frame` rigid; SINGLE is one
 * motor controller; with two and not rigid, BOUND without the turn axis and DIFFERENTIAL with it. A
 * precondition that fails, or a copy unread or in disagreement, is a refusal with the reason and no
 * boards, keeping the derived mode where one is known. A bridge is never among the boards.
 */
class DriveInForceTest {

    private val mc = Walk.CLASS_MOTOR
    private val pair = listOf(NodeInfo(0x01, mc, "r"), NodeInfo(0x02, mc, "l"))
    private val throttle = Fields.ControlMode.THROTTLE
    private val balance = Fields.ControlMode.BALANCE
    private val pivoting = Fields.ControlFrame.PIVOTING
    private val rigid = Fields.ControlFrame.RIGID

    private fun of(
        nodes: List<NodeInfo> = pair,
        modes: Map<Int, Int?> = mapOf(0x01 to throttle, 0x02 to throttle),
        frames: Map<Int, Int?> = mapOf(0x01 to pivoting, 0x02 to pivoting),
        sides: Map<Int, Int?> = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to Fields.ControlSide.LEFT),
        leaders: Set<Int> = emptySet(),
        turnAxis: Boolean = false,
    ) = DriveInForce.of(nodes, { modes[it] }, { frames[it] }, { sides[it] }, leaders, turnAxis)

    private fun both(v: Int) = mapOf(0x01 to v, 0x02 to v)

    @Test
    fun `the mode is derived from the frame, the control mode, the count and the turn axis`() {
        assertEquals(DriveTargets(DriveMode.BOUND, listOf(0x01, 0x02)), of().targets, "two, pivoting, no turn axis")
        assertEquals(
            DriveTargets(DriveMode.DIFFERENTIAL, listOf(0x01, 0x02), right = 0x01),
            of(turnAxis = true).targets,
            "two, pivoting, the turn axis used",
        )
        val rigidBalance = of(modes = both(balance), frames = both(rigid), leaders = setOf(0x02))
        assertEquals(DriveTargets(DriveMode.RIGID, listOf(0x01, 0x02)), rigidBalance.targets)
        assertEquals(DriveMode.RIGID, of(modes = both(balance), frames = both(rigid), turnAxis = true).derived)
        // A rigid frame in throttle mode is not one rigid machine: the firmware resolves the frame
        // only in balance, so the two wheels are driven as a pair.
        assertEquals(DriveMode.BOUND, of(frames = both(rigid)).derived)
        assertEquals(DriveMode.BOUND, of(modes = both(balance)).derived, "balance on pivoting platforms")
        val one = of(nodes = pair.take(1), modes = mapOf(0x01 to balance), turnAxis = true)
        assertEquals(DriveTargets(DriveMode.SINGLE, listOf(0x01)), one.targets, "one motor controller")
        for (f in listOf(of(), of(turnAxis = true), rigidBalance, one)) assertNull(f.refusal)
    }

    @Test
    fun `single drives the one motor controller and never a bridge`() {
        val withBridge = listOf(NodeInfo(0x01, Walk.CLASS_BRIDGE, "esp"), NodeInfo(0x02, mc, "r"))
        val single = of(nodes = withBridge, modes = mapOf(0x02 to throttle), frames = mapOf(0x02 to pivoting))
        assertEquals(DriveTargets(DriveMode.SINGLE, listOf(0x02)), single.targets)
    }

    @Test
    fun `a disagreement or an unread copy is a refusal with no boards`() {
        assertEquals(DriveRefusal.DISAGREE, of(modes = mapOf(0x01 to balance, 0x02 to throttle)).refusal)
        assertEquals(DriveRefusal.DISAGREE, of(frames = mapOf(0x01 to rigid, 0x02 to pivoting)).refusal)
        assertEquals(DriveRefusal.UNREAD, of(modes = mapOf(0x01 to throttle, 0x02 to null)).refusal)
        assertEquals(DriveRefusal.UNREAD, of(frames = mapOf(0x01 to null, 0x02 to pivoting)).refusal)
        val unreadClass = listOf(NodeInfo(0x01, mc, "r"), NodeInfo(0x02, null, null))
        assertEquals(DriveRefusal.UNREAD, of(nodes = unreadClass).refusal, "a node that may be a motor controller")
        // Effective values: a byte the firmware runs as throttle agrees with throttle.
        assertNull(of(modes = mapOf(0x01 to throttle, 0x02 to 7)).refusal)
        val unknown = listOf(
            of(modes = mapOf(0x01 to balance, 0x02 to throttle)),
            of(nodes = unreadClass),
        )
        for (f in unknown) {
            assertEquals(DriveTargets(DriveMode.SINGLE, emptyList()), f.targets)
            assertNull(f.derived)
        }
    }

    @Test
    fun `each precondition that fails is a refusal that keeps the derived mode for the pad`() {
        val three = pair + NodeInfo(0x03, mc, "x")
        val notTwo = of(
            nodes = three,
            modes = both(throttle) + (0x03 to throttle),
            frames = both(pivoting) + (0x03 to pivoting),
        )
        assertEquals(DriveRefusal.NOT_TWO, notTwo.refusal)
        assertEquals(DriveTargets(DriveMode.BOUND, emptyList()), notTwo.targets)

        val noLeader = of(modes = both(balance), frames = both(rigid))
        assertEquals(DriveRefusal.NO_LEADER, noLeader.refusal)
        assertEquals(DriveMode.RIGID, noLeader.mode, "the rigid pad, and nothing armed")
        assertTrue(noLeader.targets.boards.isEmpty())
        val twoLeaders = of(modes = both(balance), frames = both(rigid), leaders = setOf(0x01, 0x02))
        assertEquals(DriveRefusal.NO_LEADER, twoLeaders.refusal)
        val noRight = mapOf(0x01 to Fields.ControlSide.LEFT, 0x02 to Fields.ControlSide.LEFT)
        val twoRight = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to Fields.ControlSide.RIGHT)
        val unreadSide = mapOf(0x01 to Fields.ControlSide.RIGHT, 0x02 to null)
        for (sides in listOf(noRight, twoRight, unreadSide)) {
            val f = of(sides = sides, turnAxis = true)
            assertEquals(DriveRefusal.NO_RIGHT_WHEEL, f.refusal)
            assertEquals(DriveMode.DIFFERENTIAL, f.derived)
        }
    }

    @Test
    fun `a frame addresses exactly the boards its targets name`() {
        val armed = RiderCommand.armed(1_000)
        val bound = DriveFrame.of(DriveTargets(DriveMode.BOUND, listOf(0x02, 0x03)), armed)
        assertEquals(setOf(0x02, 0x03), bound.commands.keys)
        assertTrue(DriveFrame.of(DriveTargets.NONE, armed).commands.isEmpty())
        // A rigid frame with no leader among its boards drives nobody and still tells both.
        val rigidTargets = DriveTargets(DriveMode.RIGID, listOf(0x02, 0x03))
        val rigidFrame = DriveFrame.of(rigidTargets, armed, steer = 5, leader = 0x01)
        assertEquals(setOf(0x02, 0x03), rigidFrame.commands.keys)
        assertTrue(rigidFrame.driven.isEmpty())
    }

    @Test
    fun `targets that cannot be driven are not constructed`() {
        assertThrows<IllegalArgumentException> { DriveTargets(DriveMode.SINGLE, listOf(0x01, 0x02)) }
        assertThrows<IllegalArgumentException> { DriveTargets(DriveMode.DIFFERENTIAL, listOf(0x01, 0x02)) }
        // A refused DIFFERENTIAL names its mode and addresses nobody, so it needs no right wheel.
        assertTrue(DriveTargets(DriveMode.DIFFERENTIAL, emptyList()).boards.isEmpty())
    }
}
