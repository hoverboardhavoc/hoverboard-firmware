package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.DriveMix
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What each drive mode puts on the wire (`specs/remote-ui.md` section 2 and 3.2,
 * `specs/rover-one-imu.md` section 7): the mode in force, DERIVED from what the machine stores and
 * the Ride screen's turn-axis setting (`specs/machine-settings.md` section 7 and section 9, test 6),
 * which motor controllers it addresses and arms, which of them it drives, and where the turn goes.
 * Never a bridge.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class DriveModeTest : RiderVmTest() {

    /**
     * Make the pair derive [mode]: RIGID is balance on a rigid frame; DIFFERENTIAL and BOUND are a
     * pivoting pair with the turn axis on or off. 0x01 is the right wheel.
     */
    /** The boards store [mode]'s mode and frame; RIGID's [leader] stores `control.leader` set. */
    private fun derive(mode: DriveMode, vararg boards: Int = intArrayOf(0x01, 0x02), leader: Int? = 0x01) {
        val rigid = mode == DriveMode.RIGID
        for (b in boards) {
            val control = if (rigid) Fields.ControlMode.BALANCE else Fields.ControlMode.THROTTLE
            val frame = if (rigid) Fields.ControlFrame.RIGID else Fields.ControlFrame.PIVOTING
            transport.store[b to Fields.CONTROL_MODE.key()] = Value.U8(control)
            transport.store[b to Fields.CONTROL_FRAME.key()] = Value.U8(frame)
            if (rigid) transport.store[b to Fields.CONTROL_LEADER.key()] = Value.U8(if (b == leader) 1 else 0)
        }
        settings.setTurnAxis(mode == DriveMode.DIFFERENTIAL)
        transport.store[0x01 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.RIGHT)
        transport.store[0x02 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.LEFT)
    }

    /** Connected, attached to the board at 0x01 with the one at 0x02 found by the walk. */
    private fun connectPair() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        reporting(0x01, 0x02)
    }

    @Test
    fun `one motor controller is single, whatever the turn axis`() = runTest(dispatcher) {
        settings.setTurnAxis(true)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        reporting(0x01)
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)

        val state = currentState()
        assertEquals(DriveMode.SINGLE, state.driveMode)
        assertNull(state.drive.refusal)
        assertFalse(state.turnAxisApplies, "one board has nowhere to spend a turn")
        assertEquals(setOf(0x01), transport.frames.last().commands.keys)
    }

    @Test
    fun `a pivoting pair is bound by default and differential with the turn axis`() = runTest(dispatcher) {
        transport.store[0x01 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.RIGHT)
        connectPair()
        var state = currentState()
        assertEquals(DriveMode.BOUND, state.driveMode, "the turn axis is off until set")
        assertTrue(state.turnAxisApplies)

        viewModel.setTurnAxis(true)
        runCurrent()
        state = currentState()
        assertTrue(state.turnAxis)
        assertEquals(DriveMode.DIFFERENTIAL, state.driveMode)
        assertTrue(settings.turnAxis.value, "kept by the app with its link settings")
    }

    /** The setting survives the app: a fresh [LinkSettings] over the same store reads it back. */
    @Test
    fun `the turn axis is kept with the link settings`() {
        val prefs = FakeSharedPreferences()
        LinkSettings(prefs).setTurnAxis(true)
        assertTrue(LinkSettings(prefs).turnAxis.value)
    }

    /** The arm latches its boards and mode: the setting is refused while armed, and a change under it moves nothing. */
    @Test
    fun `the arm latches the boards and the mode it armed`() = runTest(dispatcher) {
        derive(DriveMode.BOUND)
        connectPair()
        viewModel.onArmToggle()
        viewModel.setTurnAxis(true)
        assertFalse(settings.turnAxis.value, "the setting is refused while armed")

        settings.setTurnAxis(true)
        viewModel.onThrottleMove(y = 0.25f * h, height = h)
        val armed = currentState()
        assertEquals(DriveMode.BOUND, armed.driveMode, "the armed mode holds")
        val frame = transport.frames.last()
        assertEquals(frame.commands.getValue(0x01), frame.commands.getValue(0x02))

        viewModel.onArmToggle()
        runCurrent()
        assertEquals(DriveMode.DIFFERENTIAL, currentState().driveMode, "disarmed, the setting is in force")
    }

    @Test
    fun `bound sends the same demand to both boards, steer 0 to each`() = runTest(dispatcher) {
        derive(DriveMode.BOUND)
        connectPair()
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0.25f * h, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(0x01, 0x02), frame.commands.keys)
        val one = frame.commands.getValue(0x01)
        assertEquals(one, frame.commands.getValue(0x02))
        assertTrue(one.armed)
        assertEquals(Throttle.speedFor(0.25f * h, h), one.demand)
        assertEquals(DriveMode.BOUND, currentState().driveMode)

        // Release: zero to both, still armed; disarm: the disarmed command to both.
        viewModel.onThrottleRelease()
        assertTrue(transport.frames.last().commands.values.all { it.armed && it.demand == 0 })
        viewModel.onArmToggle()
        assertTrue(transport.frames.last().commands.values.all { it == RiderCommand.DISARMED })
    }

    /**
     * DIFFERENTIAL sends a PAIR: each board its own wheel's demand, the right wheel to the board whose
     * own motor's `side` says it is the right wheel. A turn on the spot is the clearest case, because
     * the two values are then opposite and nothing else could produce them.
     */
    @Test
    fun `differential sends each board its own wheel`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        connectPair()
        viewModel.onArmToggle()
        // Centre vertically (no travel), three quarters across (half a turn to the right).
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.5f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(0x01, 0x02), frame.commands.keys)
        val right = frame.commands.getValue(0x01)
        val left = frame.commands.getValue(0x02)
        val steer = Throttle.steerFor(0.75f * w, w)
        assertTrue(steer > 0, "the right half of the pad is a turn to the right")
        assertEquals(DriveMix.of(forward = 0, steer = steer).right, right.demand)
        assertEquals(DriveMix.of(forward = 0, steer = steer).left, left.demand)
        assertEquals(steer, left.demand - right.demand, "the pair differs by the whole turn")
        assertTrue(right.armed && left.armed)
        assertTrue(right.drive.steer == 0 && left.drive.steer == 0)
    }

    /** The right wheel is each board's own setting: swap the sides and the wheels swap with them. */
    @Test
    fun `differential follows the side, not the attach point`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        transport.store[0x01 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.LEFT)
        transport.store[0x02 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.RIGHT)
        connectPair()
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.5f * h, width = w, height = h)

        val frame = transport.frames.last()
        val steer = Throttle.steerFor(0.75f * w, w)
        assertEquals(DriveMix.of(forward = 0, steer = steer).right, frame.commands.getValue(0x02).demand)
        assertEquals(DriveMix.of(forward = 0, steer = steer).left, frame.commands.getValue(0x01).demand)
    }

    /**
     * The clamp, through the ViewModel rather than the mix alone: full forward and a full turn cannot
     * be given to both wheels, and what gives is the travel.
     */
    @Test
    fun `a saturating differential stick keeps the turn and gives up the travel`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        connectPair()
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = w, y = 0f, width = w, height = h)

        val frame = transport.frames.last()
        val right = frame.commands.getValue(0x01).demand
        val left = frame.commands.getValue(0x02).demand
        val asked = 2 * (Throttle.MAX_SPEED / 2)
        assertEquals(asked, left - right, "the turn survives the clamp")
        assertEquals(Throttle.MAX_SPEED, left, "the outer wheel is on the rail")
        assertTrue(right < Throttle.MAX_SPEED / 2, "the inner wheel gave up the travel")
    }

    @Test
    fun `arming in differential takes the arm level to both boards`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        connectPair()
        viewModel.onArmToggle()

        val armedFrame = transport.frames.last()
        assertEquals(setOf(0x01, 0x02), armedFrame.commands.keys)
        assertTrue(armedFrame.commands.values.all { it.armed && it.demand == 0 })

        viewModel.onArmToggle()
        val disarmedFrame = transport.frames.last()
        assertEquals(setOf(0x01, 0x02), disarmedFrame.commands.keys)
        assertTrue(disarmedFrame.commands.values.all { it == RiderCommand.DISARMED })
    }

    @Test
    fun `releasing the stick zeroes both axes and leaves the machine armed`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        connectPair()
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = w, y = 0f, width = w, height = h)
        assertTrue(currentState().steer > 0)

        viewModel.onThrottleRelease()
        val state = currentState()
        assertEquals(0, state.steer)
        assertEquals(0, state.throttleSpeed)
        assertFalse(state.engaged)
        assertTrue(transport.frames.last().commands.values.all { it.armed && it.demand == 0 })
    }

    @Test
    fun `an unarmed stick commands nothing on either axis`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        connectPair()
        viewModel.onJoystickMove(x = w, y = 0f, width = w, height = h)

        val state = currentState()
        assertEquals(0, state.steer)
        assertEquals(0, state.throttleSpeed)
        assertTrue(transport.frames.last().commands.values.all { it == RiderCommand.DISARMED })
    }

    /**
     * The rule 3.2 says must survive any edit: the app never sends a nonzero steer while also
     * deciding what each wheel gets, or the correction is applied twice. Checked over every frame a
     * session of every two-board mode produced, each reached the way the owner reaches it: the
     * machine's control mode and frame written to both boards through the Setup model, and the turn
     * axis through the Ride screen's setting.
     */
    @Test
    fun `no frame both mixes a pair and states a turn`() = runTest(dispatcher) {
        derive(DriveMode.BOUND)
        // The rigid frame's leader, so the Setup model's leader check lets RIGID in.
        transport.store[0x01 to Fields.CONTROL_LEADER.key()] = Value.U8(1)
        connectPair()
        for (mode in listOf(DriveMode.BOUND, DriveMode.DIFFERENTIAL, DriveMode.RIGID)) {
            val rigid = mode == DriveMode.RIGID
            viewModel.setup.edit(
                Fields.CONTROL_MODE.key(),
                Value.U8(if (rigid) Fields.ControlMode.BALANCE else Fields.ControlMode.THROTTLE),
            )
            viewModel.setup.chooseFrame(if (rigid) Fields.ControlFrame.RIGID else Fields.ControlFrame.PIVOTING)
            viewModel.setTurnAxis(mode == DriveMode.DIFFERENTIAL)
            flush()
            assertEquals(mode, currentState().driveMode, "$mode derived from what both boards store")
            viewModel.onArmToggle()
            for (y in listOf(0f, 0.3f * h, 0.5f * h, 0.8f * h, h)) viewModel.onThrottleMove(y = y, height = h)
            for (x in listOf(0f, 0.3f * w, 0.5f * w, 0.8f * w, w)) {
                viewModel.onJoystickMove(x = x, y = 0.2f * h, width = w, height = h)
            }
            viewModel.onThrottleRelease()
            viewModel.onArmToggle()
        }
        assertTrue(transport.frames.isNotEmpty())
        for (frame in transport.frames) {
            val turning = frame.commands.values.filter { it.drive.steer != 0 }
            assertTrue(turning.isEmpty() || frame.driven.size == 1, "a mixed pair states no turn")
        }
        assertTrue(transport.frames.any { 0x02 in it.commands })
        assertTrue(
            transport.frames.any { f -> f.commands.values.any { it.drive.steer != 0 } },
            "the rigid mode does put a turn on the wire, so the rule above is not vacuous",
        )
    }

    // --- the rigid drive mode (`specs/rover-one-imu.md` section 7) ----------------------------

    /** The board at [leader] reports itself the rigid frame's leader, and the other the follower. */
    private fun leaderIs(leader: Int) {
        for (b in listOf(0x01, 0x02)) {
            val flag = if (b == leader) CyclicState.FLAG_LEADER else CyclicState.FLAG_FOLLOWER
            transport.emitCyclicState(rigid(flag), from = b)
        }
        dispatcher.scheduler.runCurrent()
    }

    /** A resting board whose cyclic carries [flags]. */
    private fun rigid(flags: Int) =
        CyclicState(0, 0, 0, 3_600, 0, 0, flags, 0, 0, QUIET_OBS, 0, 0)

    /**
     * The whole point of the mode: ONE command, to the leader, carrying both axes on the wire. The
     * follower is addressed so it holds its own arm level and is driven by nothing.
     */
    @Test
    fun `rigid drives the leader alone and puts both axes on the wire`() = runTest(dispatcher) {
        // The 2026-11-02 case (`specs/rigid-leader.md` section 7): no board's telemetry says it
        // leads (both ran throttle since their last arm), and the stored flag names the leader.
        derive(DriveMode.RIGID, leader = 0x01)
        connectPair()
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.25f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(0x01, 0x02), frame.commands.keys)
        assertEquals(setOf(0x01), frame.driven, "only the leader is driven")

        val leader = frame.commands.getValue(0x01).drive
        assertEquals(Throttle.speedFor(0.25f * h, h), leader.value, "the lean is the stick's y")
        assertEquals(Throttle.steerFor(0.75f * w, w), leader.steer, "the turn is the stick's x")

        val follower = frame.commands.getValue(0x02)
        assertTrue(follower.armed, "the follower runs its own mode machine and holds its own level")
        assertEquals(0, follower.demand)
        assertEquals(0, follower.steer)
    }

    /**
     * The leader is read off the store, not the wire (`specs/rigid-leader.md` section 7): the board
     * stored as leader is the board driven, whatever the telemetry flags say.
     */
    @Test
    fun `rigid drives whichever board is stored as leader`() = runTest(dispatcher) {
        derive(DriveMode.RIGID, leader = 0x02)
        connectPair()
        leaderIs(0x01)
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.25f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(0x02), frame.driven)
        assertEquals(0x02, currentState().leader)
        assertTrue(frame.commands.getValue(0x02).drive.steer > 0)
        assertEquals(0, frame.commands.getValue(0x01).drive.steer)
    }

    /**
     * A machine stored with no leader is refused and says why (`specs/remote-ui.md` 3.1c,
     * `specs/rigid-leader.md` section 7), whatever the telemetry says. The pad is RIGID's, the mode
     * the machine stores, and nothing is armed.
     */
    @Test
    fun `rigid is refused while no board is stored as leader`() = runTest(dispatcher) {
        derive(DriveMode.RIGID, leader = null)
        connectPair()
        val state = currentState()
        assertEquals(DriveMode.RIGID, state.driveMode, "the derived mode's pad")
        assertEquals(DriveRefusal.NO_LEADER, state.drive.refusal)
        assertEquals(DriveMode.RIGID, state.drive.derived)
        assertTrue(state.drive.targets.boards.isEmpty(), "no board stored as leader")
        assertFalse(state.canArm)

        leaderIs(0x01)
        assertEquals(DriveRefusal.NO_LEADER, currentState().drive.refusal, "a telemetry flag names no leader")
    }

    // --- the mode in force (`specs/remote-ui.md` 3.2) ------------------------------------------

    /**
     * Copies that disagree are seen once a board reports it (`EV_MACHINE`): the machine fields are
     * then read from each board, and the app refuses the arm and says why.
     */
    @Test
    fun `copies that disagree refuse the arm and say so`() = runTest(dispatcher) {
        derive(DriveMode.RIGID, 0x01)
        derive(DriveMode.BOUND, 0x02)
        connectPair()
        assertEquals(DriveMode.RIGID, currentState().drive.derived, "read once, from the first board")

        transport.emitCyclicState(QUIET_STATE.copy(fault = CyclicState.EV_MACHINE), from = 0x02)
        runCurrent()
        viewModel.onArmToggle()

        val state = currentState()
        assertEquals(DriveRefusal.DISAGREE, state.drive.refusal)
        assertFalse(state.armed, "the app never drives a mode the machine does not agree on")
        assertTrue(transport.frames.last().commands.isEmpty())
    }

    @Test
    fun `a copy that has not answered refuses the arm`() = runTest(dispatcher) {
        transport.unreadable += Fields.CONTROL_MODE.key()
        connectPair()

        val state = currentState()
        assertEquals(DriveMode.SINGLE, state.driveMode)
        assertEquals(DriveRefusal.UNREAD, state.drive.refusal)
        assertFalse(state.canArm)
    }

    /** A rigid balance machine of which the walk found one board. */
    @Test
    fun `a rigid machine with one motor controller refuses the arm`() = runTest(dispatcher) {
        derive(DriveMode.RIGID, 0x01)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        viewModel.onArmToggle()

        val state = currentState()
        assertEquals(DriveMode.RIGID, state.driveMode, "the derived mode's pad")
        assertEquals(DriveRefusal.NOT_TWO, state.drive.refusal)
        assertFalse(state.armed)
        assertTrue(transport.frames.last().commands.isEmpty())
    }

    @Test
    fun `differential with no right wheel refuses the arm on the stick`() = runTest(dispatcher) {
        derive(DriveMode.DIFFERENTIAL)
        transport.store[0x01 to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.LEFT)
        connectPair()

        val state = currentState()
        assertEquals(DriveRefusal.NO_RIGHT_WHEEL, state.drive.refusal)
        assertEquals(DriveMode.DIFFERENTIAL, state.driveMode)
        assertFalse(state.canArm)
    }

    /**
     * Nothing is addressed and nothing can be armed until a node has answered that it is a motor
     * controller: a node whose `IDENTITY` has not answered may be a bridge.
     */
    @Test
    fun `a node whose identity has not answered is never addressed`() = runTest(dispatcher) {
        transport.unidentifiable += listOf(0x01, 0x02)
        connectPair()

        val state = currentState()
        assertTrue(state.motorControllers.isEmpty())
        assertEquals(DriveRefusal.UNREAD, state.drive.refusal)
        assertFalse(state.canArm)
        viewModel.onArmToggle()
        assertFalse(currentState().armed)
        assertTrue(transport.frames.all { it.commands.isEmpty() })
    }

    /**
     * Through a bridge: the node on the phone's link is a bridge and the motor controllers are behind
     * it (`specs/hoverboard-bridge.md`, "The bridge as a node"). The bridge is never addressed, driven or armed;
     * a pivoting pair behind it is BOUND, and one motor controller SINGLE.
     */
    @Test
    fun `a bridge at the attach point is never armed`() = runTest(dispatcher) {
        transport.makeBridge(0x01)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02, 0x03)
        reporting(0x02, 0x03)
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0.25f * h, height = h)

        val state = currentState()
        assertEquals(listOf(0x02, 0x03), state.motorControllers.map { it.address })
        assertEquals(DriveMode.BOUND, state.driveMode)
        assertEquals(listOf(0x02, 0x03), state.driving.boards)
        assertTrue(transport.frames.none { 0x01 in it.commands }, "the bridge is never told anything")
        assertEquals(setOf(0x02, 0x03), transport.frames.last().commands.keys)
        assertTrue(transport.frames.last().commands.values.all { it.armed })
    }

    @Test
    fun `single through a bridge drives the one motor controller`() = runTest(dispatcher) {
        transport.makeBridge(0x01)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        reporting(0x02)
        viewModel.onArmToggle()

        assertEquals(DriveMode.SINGLE, currentState().driveMode)
        assertEquals(setOf(0x02), transport.frames.last().commands.keys)
    }

    /** The mode is derived again on a reconnect, from the boards and the app's own setting. */
    @Test
    fun `the drive mode comes back with the boards after a reconnect`() = runTest(dispatcher) {
        derive(DriveMode.RIGID)
        connectPair()
        assertEquals(DriveMode.RIGID, currentState().driveMode)

        transport.setConnectionState(ConnectionState.SCANNING)
        transport.setAttachedBoard(null)
        assertEquals(DriveMode.SINGLE, currentState().driveMode, "no node attached, nothing in force")
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        assertEquals(DriveMode.RIGID, currentState().driveMode)
    }
}
