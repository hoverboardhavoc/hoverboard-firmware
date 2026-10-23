package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveMix
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What each drive mode puts on the wire (`specs/rider-ui.md` 3.2 and `specs/rover-one-imu.md`
 * section 7): which boards a mode addresses, which of them it drives, and where the turn goes.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class DriveModeTest : RiderVmTest() {

    /** Connected, attached to the master at 0x01 with the slave at 0x02 discovered. */
    private fun connectPair() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, slave = 0x02)
    }

    @Test
    fun `single is the default and names the master only`() = runTest(dispatcher) {
        connectPair()
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)

        assertEquals(DriveMode.SINGLE, currentState().driveMode)
        assertEquals(setOf(Node.MASTER), transport.frames.last().commands.keys)
    }

    @Test
    fun `bound sends the same demand to both boards, steer 0 to each`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.BOUND)
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0.25f * h, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), frame.commands.keys)
        assertEquals(frame.master, frame.commands.getValue(Node.SLAVE))
        assertTrue(frame.master.armed)
        assertEquals(Throttle.speedFor(0.25f * h, h), frame.master.demand)
        val state = currentState()
        assertEquals(DriveMode.BOUND, state.driveMode)
        assertEquals(0x02, state.slaveBoard)

        // Release: zero to both, still armed; disarm: the disarmed command to both.
        viewModel.onThrottleRelease()
        assertTrue(transport.frames.last().commands.values.all { it.armed && it.demand == 0 })
        viewModel.onArmToggle()
        assertTrue(transport.frames.last().commands.values.all { it == RiderCommand.DISARMED })
    }

    /**
     * DIFFERENTIAL sends a PAIR: each board its own wheel's demand, and the master is the right wheel
     * (the rover's build fixes the sides, `specs/rider-ui.md` 3.2). A turn on the spot is the clearest
     * case, because the two values are then opposite and nothing else could produce them.
     */
    @Test
    fun `differential sends each board its own wheel`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.DIFFERENTIAL)
        viewModel.onArmToggle()
        // Centre vertically (no travel), three quarters across (half a turn to the right).
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.5f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), frame.commands.keys)
        val master = frame.commands.getValue(Node.MASTER)
        val slave = frame.commands.getValue(Node.SLAVE)
        val steer = Throttle.steerFor(0.75f * w, w)
        assertTrue(steer > 0, "the right half of the pad is a turn to the right")
        assertEquals(DriveMix.of(forward = 0, steer = steer).right, master.demand)
        assertEquals(DriveMix.of(forward = 0, steer = steer).left, slave.demand)
        assertEquals(steer, slave.demand - master.demand, "the pair differs by the whole turn")
        assertTrue(master.armed && slave.armed)
        assertTrue(master.drive.steer == 0 && slave.drive.steer == 0)
    }

    /**
     * The clamp, through the ViewModel rather than the mix alone: full forward and a full turn cannot
     * be given to both wheels, and what gives is the travel. The two boards still differ by the whole
     * turn, so the machine turns as hard as it was asked to and runs slower than it was asked to.
     */
    @Test
    fun `a saturating differential stick keeps the turn and gives up the travel`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.DIFFERENTIAL)
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = w, y = 0f, width = w, height = h)

        val frame = transport.frames.last()
        val master = frame.commands.getValue(Node.MASTER).demand
        val slave = frame.commands.getValue(Node.SLAVE).demand
        val asked = 2 * (Throttle.MAX_SPEED / 2)
        assertEquals(asked, slave - master, "the turn survives the clamp")
        assertEquals(Throttle.MAX_SPEED, slave, "the outer wheel is on the rail")
        assertTrue(master < Throttle.MAX_SPEED / 2, "the inner wheel gave up the travel")
    }

    @Test
    fun `arming in differential takes the arm level to both boards`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.DIFFERENTIAL)
        viewModel.onArmToggle()

        val armedFrame = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), armedFrame.commands.keys)
        assertTrue(armedFrame.commands.values.all { it.armed && it.demand == 0 })

        viewModel.onArmToggle()
        val disarmedFrame = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), disarmedFrame.commands.keys)
        assertTrue(disarmedFrame.commands.values.all { it == RiderCommand.DISARMED })
    }

    @Test
    fun `releasing the stick zeroes both axes and leaves the machine armed`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.DIFFERENTIAL)
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
        connectPair()
        viewModel.setDriveMode(DriveMode.DIFFERENTIAL)
        viewModel.onJoystickMove(x = w, y = 0f, width = w, height = h)

        val state = currentState()
        assertEquals(0, state.steer)
        assertEquals(0, state.throttleSpeed)
        assertTrue(transport.frames.last().commands.values.all { it == RiderCommand.DISARMED })
    }

    /**
     * The rule 3.2 says must survive any edit: the app never sends a nonzero steer while also
     * deciding what each wheel gets, or the correction is applied twice. Checked over every frame a
     * session of every mode produced, including the one mode that DOES put a steer on the wire.
     */
    @Test
    fun `no frame both mixes a pair and states a turn`() = runTest(dispatcher) {
        connectPair()
        leaderIsTheMaster()
        for (mode in DriveMode.entries) {
            viewModel.setDriveMode(mode)
            viewModel.onArmToggle()
            for (y in listOf(0f, 0.3f * h, 0.5f * h, 0.8f * h, h)) viewModel.onThrottleMove(y = y, height = h)
            // The turn axis too, which is the one that could reach the wire if the mix ever sent it
            // instead of spending it on the pair.
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
        assertTrue(transport.frames.any { Node.SLAVE in it.commands })
        assertTrue(
            transport.frames.any { f -> f.commands.values.any { it.drive.steer != 0 } },
            "the rigid mode does put a turn on the wire, so the rule above is not vacuous",
        )
    }

    // --- the rigid drive mode (`specs/rover-one-imu.md` section 7) ----------------------------

    /** The attached master reports itself the rigid frame's leader, and the slave the follower. */
    private fun leaderIsTheMaster() {
        transport.emitCyclicState(rigid(CyclicState.FLAG_LEADER), from = 0x01)
        transport.emitCyclicState(rigid(CyclicState.FLAG_FOLLOWER), from = 0x02)
        dispatcher.scheduler.runCurrent()
    }

    /** A resting board whose cyclic carries [flags]. */
    private fun rigid(flags: Int) =
        CyclicState(0, 0, 0, 3_600, 0, 0, flags, 0, QUIET_OBS)

    /**
     * The whole point of the mode: ONE command, to the leader, carrying both axes on the wire. The
     * follower is addressed so it holds its own arm level and is driven by nothing.
     */
    @Test
    fun `rigid drives the leader alone and puts both axes on the wire`() = runTest(dispatcher) {
        connectPair()
        leaderIsTheMaster()
        viewModel.setDriveMode(DriveMode.RIGID)
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.25f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), frame.commands.keys)
        assertEquals(setOf(Node.MASTER), frame.driven, "only the leader is driven")

        val leader = frame.commands.getValue(Node.MASTER).drive
        assertEquals(Throttle.speedFor(0.25f * h, h), leader.value, "the lean is the stick's y")
        assertEquals(Throttle.steerFor(0.75f * w, w), leader.steer, "the turn is the stick's x")
        assertTrue(leader.steer > 0, "the right half of the pad is a turn to the right")

        val follower = frame.commands.getValue(Node.SLAVE)
        assertTrue(follower.armed, "the follower runs its own mode machine and holds its own level")
        assertEquals(0, follower.demand)
        assertEquals(0, follower.steer)
    }

    /** The leader is read off the wire, not assumed: a slave that leads is the board that is driven. */
    @Test
    fun `rigid drives the slave when the slave is the one leading`() = runTest(dispatcher) {
        connectPair()
        transport.emitCyclicState(rigid(CyclicState.FLAG_FOLLOWER), from = 0x01)
        transport.emitCyclicState(rigid(CyclicState.FLAG_LEADER), from = 0x02)
        dispatcher.scheduler.runCurrent()

        viewModel.setDriveMode(DriveMode.RIGID)
        viewModel.onArmToggle()
        viewModel.onJoystickMove(x = 0.75f * w, y = 0.25f * h, width = w, height = h)

        val frame = transport.frames.last()
        assertEquals(setOf(Node.SLAVE), frame.driven)
        assertEquals(Node.SLAVE, currentState().leader)
        assertTrue(frame.commands.getValue(Node.SLAVE).drive.steer > 0)
        assertEquals(0, frame.commands.getValue(Node.MASTER).drive.steer)
    }

    /** Arming and disarming reach both boards, because both run their own mode machine. */
    @Test
    fun `arming in rigid takes the arm level to both boards`() = runTest(dispatcher) {
        connectPair()
        leaderIsTheMaster()
        viewModel.setDriveMode(DriveMode.RIGID)
        viewModel.onArmToggle()

        val armed = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), armed.commands.keys)
        assertTrue(armed.commands.values.all { it.armed })

        viewModel.onArmToggle()
        val disarmed = transport.frames.last()
        assertEquals(setOf(Node.MASTER, Node.SLAVE), disarmed.commands.keys)
        assertTrue(disarmed.commands.values.all { it == RiderCommand.DISARMED })
    }

    /**
     * A role the app cannot see is a board it cannot address, so the mode is refused: with no
     * leader reported at all, and with BOTH boards claiming the lead, which is the staging error
     * the leader's own mode fault refuses to engage on.
     */
    @Test
    fun `rigid is refused until exactly one board reports leading`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.RIGID)
        assertEquals(DriveMode.SINGLE, currentState().driveMode, "no board reports leading")

        transport.emitCyclicState(rigid(CyclicState.FLAG_LEADER), from = 0x01)
        transport.emitCyclicState(rigid(CyclicState.FLAG_LEADER), from = 0x02)
        dispatcher.scheduler.runCurrent()
        viewModel.setDriveMode(DriveMode.RIGID)
        assertEquals(DriveMode.SINGLE, currentState().driveMode, "two leaders name no leader")
        assertFalse(currentState().canSelect(DriveMode.RIGID))

        leaderIsTheMaster()
        viewModel.setDriveMode(DriveMode.RIGID)
        assertEquals(DriveMode.RIGID, currentState().driveMode)
        assertTrue(currentState().canSelect(DriveMode.RIGID))
    }

    @Test
    fun `a mode that drives the slave is refused without one, and no mode changes while armed`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        for (mode in DriveMode.entries.filter { it.needsSlave }) {
            viewModel.setDriveMode(mode)
            assertEquals(DriveMode.SINGLE, currentState().driveMode, "$mode without a slave")
        }

        connectPair()
        viewModel.onArmToggle()
        for (mode in DriveMode.entries.filter { it != DriveMode.SINGLE }) {
            viewModel.setDriveMode(mode)
            assertEquals(DriveMode.SINGLE, currentState().driveMode, "$mode while armed")
        }
        assertFalse(currentState().canChangeDriveMode)
    }

    @Test
    fun `the drive mode outlives a disarm and resets with the link`() = runTest(dispatcher) {
        connectPair()
        viewModel.setDriveMode(DriveMode.BOUND)
        viewModel.onArmToggle()
        viewModel.onArmToggle()
        assertEquals(DriveMode.BOUND, currentState().driveMode)

        transport.setConnectionState(ConnectionState.SCANNING)
        transport.setAttachedBoard(null)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, slave = 0x02)
        assertEquals(DriveMode.SINGLE, currentState().driveMode)
    }
}
