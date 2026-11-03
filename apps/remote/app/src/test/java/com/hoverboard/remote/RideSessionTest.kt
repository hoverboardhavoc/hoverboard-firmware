package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicMotor
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.model.Wheel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What the view model does for the Ride screen after the first drive (`specs/remote-ui.md` 3.1):
 * the motor-controller-off card, the boards a re-walk finds and the lamp test that waits for them,
 * and the wheel that does not turn.
 *
 * Read with [now], never with an idle-advancing read: a session that has found only a bridge walks
 * again every 3 s for as long as it lasts, so the scheduler is never idle.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class RideSessionTest : RiderVmTest() {

    /**
     * A test on the view model's own scope, cancelled when the body ends: a session with only a
     * bridge walks again every 3 s for as long as it lasts, which `runTest` would wait out forever.
     */
    private fun vmTest(body: suspend TestScope.() -> Unit) = runTest(dispatcher) {
        try {
            body()
        } finally {
            viewModel.viewModelScope.cancel()
        }
    }

    private fun TestScope.now(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        runCurrent()
        return viewModel.uiState.value.also { collector.cancel() }
    }

    /** Hold a collector for the whole test, so the card's 1 s runs on the test clock. */
    private fun TestScope.collecting() = backgroundScope.launch { viewModel.uiState.collect {} }

    private fun TestScope.bridgeOnly() {
        transport.makeBridge(0x01)
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
    }

    // --- the motor-controller-off card ---------------------------------------------------------

    @Test
    fun `the card shows 1 s after the connect with no motor controller reporting, and not before`() =
        vmTest {
            collecting()
            bridgeOnly()
            assertFalse(now().boardsOff)
            advanceTimeBy(MainViewModel.BOARDS_OFF_MS - 1)
            assertFalse(now().boardsOff)
            advanceTimeBy(2)
            val state = now()
            assertTrue(state.boardsOff)
            assertFalse(state.canArm)
        }

    @Test
    fun `the card goes on the first frame from a possible motor controller`() = vmTest {
        collecting()
        bridgeOnly()
        advanceTimeBy(MainViewModel.BOARDS_OFF_MS + 1)
        assertTrue(now().boardsOff)

        // A source the walk did not find counts as a possible motor controller.
        reporting(0x02)
        assertFalse(now().boardsOff)
    }

    @Test
    fun `boards that do report keep the card away`() = vmTest {
        collecting()
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        advanceTimeBy(MainViewModel.BOARDS_OFF_MS * 3)
        assertFalse(now().boardsOff)
    }

    /** An app holding the arm level when the card shows drops it: a board powered up again boots to OFF. */
    @Test
    fun `an armed app drops the arm level when the card shows`() = vmTest {
        collecting()
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onArmToggle()
        runCurrent()
        assertTrue(now().armed)

        advanceTimeBy(MainViewModel.BOARDS_OFF_MS + 1)
        val state = now()
        assertTrue(state.boardsOff)
        assertFalse(state.armed)
        assertFalse(state.selfDisarmed, "no statement: the card says why")
        assertEquals(RiderCommand.DISARMED, transport.last)
        viewModel.onArmToggle()
        assertFalse(now().armed, "ARM refuses while the card shows")
    }

    // --- boards that come up after the connect -------------------------------------------------

    /**
     * Only a bridge at the connect: no lamp test and a re-walk every 3 s. A re-walk that finds the
     * boards reads their facts, and the lamp test runs once they are in.
     */
    @Test
    fun `a re-walk finds the boards and the lamp test waits for them`() = vmTest {
        collecting()
        bridgeOnly()
        assertFalse(now().lampTest, "a bridge alone has no lamp to test")
        assertTrue(transport.rewalks.isEmpty())

        advanceTimeBy(com.hoverboard.remote.SetupModel.REWALK_INTERVAL_MS + 1)
        assertEquals(listOf(listOf(0x01)), transport.rewalks, "probed from the attached node")
        assertFalse(now().lampTest)

        transport.rewalkFinds = listOf(0x01, 0x02, 0x03)
        advanceTimeBy(com.hoverboard.remote.SetupModel.REWALK_INTERVAL_MS)
        reporting(0x02, 0x03)
        val state = now()
        assertEquals(listOf(0x02, 0x03), state.motorControllers.map { it.address })
        assertTrue(state.lampTest, "the facts of the boards it found are in")
        assertFalse(state.boardsOff)
        val walks = transport.rewalks.size
        advanceTimeBy(MainViewModel.LAMP_TEST_MS + com.hoverboard.remote.SetupModel.REWALK_INTERVAL_MS * 2)
        assertEquals(walks, transport.rewalks.size, "no cadence once a motor controller is known")
    }

    @Test
    fun `an arm tap ends the lamp test`() = vmTest {
        collecting()
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        assertTrue(now().lampTest)
        viewModel.onArmToggle()
        val state = now()
        assertTrue(state.armed)
        assertFalse(state.lampTest, "the pad is back as the machine arms")
    }

    // --- a wheel that does not turn ------------------------------------------------------------

    /** The pad position that commands exactly [demand]. */
    private fun yFor(demand: Int): Float {
        val near = h / 2 * (1f - demand.toFloat() / Throttle.MAX_SPEED)
        return (-2_000..2_000).map { near + it * 0.0005f }.first { Throttle.speedFor(it, h) == demand }
    }

    private fun TestScope.armedAt(demand: Int) {
        collecting()
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        runCurrent()
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = yFor(demand), height = h)
        runCurrent()
        check(transport.last?.demand == demand)
    }

    @Test
    fun `a demand of 8192 held 1 s with no wheel rate is a wheel that does not turn`() = vmTest {
        armedAt(MainViewModel.NOT_TURNING_DEMAND)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS - 1)
        assertTrue(now().notTurning.isEmpty())
        advanceTimeBy(2)
        val state = now()
        assertEquals(setOf(Wheel(ONE_BOARD, 0)), state.notTurning)
        assertTrue(state.armed, "the app does not disarm for it")
    }

    @Test
    fun `a demand of 8191 is not judged`() = vmTest {
        armedAt(MainViewModel.NOT_TURNING_DEMAND - 1)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS * 3)
        assertTrue(now().notTurning.isEmpty())
    }

    @Test
    fun `a turning wheel, a dropped demand, a silent board and the disarm each clear it`() = vmTest {
        armedAt(MainViewModel.NOT_TURNING_DEMAND * 2)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS + 1)
        assertEquals(setOf(Wheel(ONE_BOARD, 0)), now().notTurning)

        // The wheel reports a rate: cleared, and the second it stays at a rate judges nothing.
        transport.emitCyclicState(QUIET_STATE.copy(wheelRate = 4_096))
        assertTrue(now().notTurning.isEmpty())
        advanceTimeBy(MainViewModel.NOT_TURNING_MS * 2)
        assertTrue(now().notTurning.isEmpty())

        // Back to no rate: judged again after its own second; the demand dropping clears it.
        transport.emitCyclicState(QUIET_STATE)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS + 1)
        assertEquals(setOf(Wheel(ONE_BOARD, 0)), now().notTurning)
        viewModel.onThrottleRelease()
        assertTrue(now().notTurning.isEmpty())

        // Judged again, then the disarm clears it.
        viewModel.onThrottleMove(y = 0f, height = h)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS + 1)
        assertEquals(setOf(Wheel(ONE_BOARD, 0)), now().notTurning)
        viewModel.onArmToggle()
        assertTrue(now().notTurning.isEmpty())
    }

    /** Each board is judged on the demand in its own command: BOUND gives both of them one. */
    @Test
    fun `every board given a demand is judged`() = vmTest {
        collecting()
        // Two motor controllers, pivoting, turn axis off: the derived mode is BOUND.
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        reporting(0x01, 0x02)
        runCurrent()
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS + 1)
        assertEquals(setOf(Wheel(0x01, 0), Wheel(0x02, 0)), now().notTurning, "BOUND gives both the demand")
    }

    /**
     * A 12-FET alone (`specs/twelve-fet.md` section 6.2): each wheel is judged by its own motor's
     * rate, motor 1's from the frame's motor-1 block (`specs/link-control.md`, "The motor-1
     * block"), so the wheel that stands still is the one said and the turning one is not. Its
     * figures come from motor 1's own facts, read on attach.
     */
    @Test
    fun `a two-motor board's wheels are judged and figured each by its own motor`() = vmTest {
        collecting()
        transport.store[0x01 to Fields.MOTOR_GATE_HI_A.key(1)] = Value.PinAf(2, 6, 0)
        transport.store[0x01 to Fields.CONTROL_SIDE.key(0)] = Value.U8(Fields.ControlSide.LEFT)
        transport.store[0x01 to Fields.CONTROL_SIDE.key(1)] = Value.U8(Fields.ControlSide.RIGHT)
        transport.store[0x01 to Fields.MOTOR_CURRENT_CAL.key(1)] = Value.U16(410)
        transport.store[0x01 to Fields.MOTOR_CURRENT_LIMIT.key(1)] = Value.U32(12_000)
        transport.store[0x01 to Fields.MOTOR_POLE_PAIRS.key(1)] = Value.U8(15)
        transport.store[0x01 to Fields.WHEEL_DIAMETER_MM.key(1)] = Value.U16(165)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        val stalled = CyclicMotor(wheelSpeed = 0, phasePeak = 0, phaseMean = 0, dutyOn = 0, wheelRate = 0)
        transport.emitCyclicState(QUIET_STATE.copy(wheelRate = 4_096, motor1 = stalled), from = 0x01)
        runCurrent()

        val facts = now()
        val m0 = Wheel(0x01, 0)
        val m1 = Wheel(0x01, 1)
        assertEquals(listOf(m0, m1), facts.wheels)
        assertEquals(listOf(m0, m1), facts.slots.all.map { it.wheel }, "motor 0 left, motor 1 right")
        assertEquals(410, facts.countsPerAmp[m1])
        assertEquals(12_000, facts.currentLimit[m1])
        assertEquals(listOf(4_096 to 0, 0 to 1), facts.speedFacts.map { it.wheelRateQ12 to it.motor })
        assertEquals(15, facts.speedFacts[1].polePairs)

        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        advanceTimeBy(MainViewModel.NOT_TURNING_MS + 1)
        assertEquals(setOf(m1), now().notTurning, "motor 0 reports a rate, motor 1 none")

        val turning = QUIET_STATE.copy(wheelRate = 4_096, motor1 = stalled.copy(wheelRate = 4_096))
        transport.emitCyclicState(turning, from = 0x01)
        assertTrue(now().notTurning.isEmpty(), "motor 1 reports a rate")
    }
}
