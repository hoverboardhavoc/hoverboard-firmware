package com.hoverboardhavoc.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboardhavoc.remote.model.ArmPhase
import com.hoverboardhavoc.remote.model.ArmRefusal
import com.hoverboardhavoc.remote.model.BoardDisarm
import com.hoverboardhavoc.remote.model.ConnectionState
import com.hoverboardhavoc.remote.model.FaultLamp
import com.hoverboardhavoc.remote.model.RiderCommand
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The arm control shows what the boards confirm (`specs/arm-confirmed.md` section 3, test 3): the
 * four phases and their exits against a telemetry stream the test writes by hand, each board's
 * `FLAG_MOE` included ([FakeHoverboardTransport.boardsFollowArm] off).
 */
@OptIn(ExperimentalCoroutinesApi::class)
class ArmConfirmTest : RiderVmTest() {

    /** The current state, read while a collector keeps the flow turning and virtual time stands. */
    private fun TestScope.now(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        runCurrent()
        return viewModel.uiState.value.also { collector.cancel() }
    }

    /** Connect to [boards] (the first attached), each reporting a quiet frame with `FLAG_MOE` clear. */
    private fun TestScope.connect(vararg boards: Int) {
        transport.boardsFollowArm = false
        backgroundScope.launch { viewModel.uiState.collect {} }
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(boards.first(), *boards.drop(1).toTypedArray())
        reporting(*boards)
        runCurrent()
    }

    /** Arm, and have every one of [boards] confirm it. */
    private fun TestScope.armConfirmed(vararg boards: Int) {
        viewModel.onArmToggle()
        runCurrent()
        transport.setMoe(true, *boards)
        runCurrent()
        check(now().armPhase == ArmPhase.ARMED)
    }

    @Test
    fun `ARM, ARMING until every board confirms, then ARMED`() = runTest(dispatcher) {
        connect(0x01, 0x02)
        assertEquals(ArmPhase.IDLE, now().armPhase)

        viewModel.onArmToggle()
        runCurrent()
        assertEquals(ArmPhase.ARMING, now().armPhase)
        assertFalse(now().armed)
        assertTrue(checkNotNull(transport.last).armed, "the arm level is on the wire while ARMING")

        transport.setMoe(true, 0x01)
        assertEquals(ArmPhase.ARMING, now().armPhase, "one board of two is not the machine")
        transport.setMoe(true, 0x02)
        assertEquals(ArmPhase.ARMED, now().armPhase)
        assertTrue(now().armed)

        // Past the window, a confirmed arm stays armed.
        advanceTimeBy(MainViewModel.ARM_CONFIRM_MS + 1)
        assertEquals(ArmPhase.ARMED, now().armPhase)
        assertNull(now().armRefused)
    }

    @Test
    fun `the throttle stays zero until the arm is confirmed`() = runTest(dispatcher) {
        connect(ONE_BOARD)
        viewModel.onArmToggle()
        runCurrent()
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()
        assertEquals(0, now().throttleSpeed)
        assertEquals(0, checkNotNull(transport.last).demand)

        transport.setMoe(true, ONE_BOARD)
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()
        assertTrue(now().throttleSpeed > 0, "live once confirmed")
        assertTrue(checkNotNull(transport.last).demand > 0)
    }

    @Test
    fun `an arm not confirmed in 1 s is refused and its power request dropped`() = runTest(dispatcher) {
        connect(0x01, 0x02)
        viewModel.onArmToggle()
        runCurrent()
        transport.setMoe(true, 0x01)
        // 0x02 refuses: it reports EV_NO_MOTOR, which appeared after the arm.
        transport.emitCyclicState(QUIET_STATE.copy(fault = CyclicState.EV_NO_MOTOR), from = 0x02)
        runCurrent()

        advanceTimeBy(MainViewModel.ARM_CONFIRM_MS - 1)
        runCurrent()
        assertEquals(ArmPhase.ARMING, now().armPhase, "still inside the window")
        advanceTimeBy(2)
        runCurrent()

        val state = now()
        assertEquals(ArmRefusal(0x02, FaultLamp.NO_MOTOR), state.armRefused)
        assertFalse(state.armPhase.holdsLevel)
        assertEquals(RiderCommand.DISARMED, transport.last, "the power request is dropped")
        // 0x01 still reports MOE: DISARMING until it clears.
        assertEquals(ArmPhase.DISARMING, state.armPhase)
        transport.setMoe(false, 0x01)
        assertEquals(ArmPhase.IDLE, now().armPhase)
        assertEquals(ArmRefusal(0x02, FaultLamp.NO_MOTOR), now().armRefused, "kept until the next ARM tap")
    }

    @Test
    fun `an arm no board answers is refused without a bit to name`() = runTest(dispatcher) {
        connect(ONE_BOARD)
        viewModel.onArmToggle()
        advanceTimeBy(MainViewModel.ARM_CONFIRM_MS + 1)
        runCurrent()
        assertEquals(ArmRefusal(ONE_BOARD, null), now().armRefused)
        assertEquals(ArmPhase.IDLE, now().armPhase, "the board reports MOE clear, so the disarm is confirmed")
        assertEquals(RiderCommand.DISARMED, transport.last)
    }

    @Test
    fun `DISARMING until every board reports MOE clear, then ARM`() = runTest(dispatcher) {
        connect(0x01, 0x02)
        armConfirmed(0x01, 0x02)

        viewModel.onArmToggle()
        runCurrent()
        assertEquals(ArmPhase.DISARMING, now().armPhase)
        assertEquals(RiderCommand.DISARMED, transport.last, "the stop is sent at once, never waited for")
        assertFalse(now().canArm, "no arm until the disarm is confirmed")

        transport.setMoe(false, 0x01)
        assertEquals(ArmPhase.DISARMING, now().armPhase)
        transport.setMoe(false, 0x02)
        assertEquals(ArmPhase.IDLE, now().armPhase)
        assertTrue(now().canArm)
        assertTrue(now().disarmUnconfirmed.isEmpty())
    }

    @Test
    fun `a disarm not confirmed in 1 s is labelled and the stop is still sent`() = runTest(dispatcher) {
        connect(0x01, 0x02)
        armConfirmed(0x01, 0x02)
        val tap = transport.frames.size
        viewModel.onArmToggle()
        transport.setMoe(false, 0x01)

        advanceTimeBy(MainViewModel.ARM_CONFIRM_MS + 1)
        runCurrent()
        val state = now()
        assertEquals(ArmPhase.DISARMING, state.armPhase)
        assertEquals(listOf(0x02), state.disarmUnconfirmed)
        // Every frame since the tap is the stop, to both boards: the label waits, the stop does not.
        val sinceTap = transport.frames.drop(tap)
        assertTrue(sinceTap.isNotEmpty())
        assertTrue(sinceTap.all { f -> f.commands.values.all { it == RiderCommand.DISARMED } })
        assertEquals(listOf(0x01, 0x02), sinceTap.last().boards)

        // The late confirmation ends it.
        transport.setMoe(false, 0x02)
        assertEquals(ArmPhase.IDLE, now().armPhase)
        assertTrue(now().disarmUnconfirmed.isEmpty())
    }

    @Test
    fun `a board that drops MOE while armed disarmed itself, and the app drops the level`() = runTest(dispatcher) {
        connect(0x01, 0x02)
        armConfirmed(0x01, 0x02)

        transport.setMoe(false, 0x02)
        val state = now()
        assertEquals(BoardDisarm(0x02, null), state.boardDisarm)
        assertFalse(state.armPhase.holdsLevel)
        assertEquals(RiderCommand.DISARMED, transport.last)
        assertEquals(ArmPhase.DISARMING, state.armPhase, "0x01 still reports MOE")
        transport.setMoe(false, 0x01)
        assertEquals(ArmPhase.IDLE, now().armPhase)

        // The next arm clears the report.
        viewModel.onArmToggle()
        runCurrent()
        assertNull(now().boardDisarm)
    }

    @Test
    fun `a board that stops reporting keeps the label`() = runTest(dispatcher) {
        connect(ONE_BOARD)
        armConfirmed(ONE_BOARD)
        // The fake never ages a board out; a board that stops reporting is one that sends nothing
        // new, and nothing new says MOE clear. The label stays ARMED.
        advanceTimeBy(MainViewModel.ARM_CONFIRM_MS * 3)
        runCurrent()
        assertEquals(ArmPhase.ARMED, now().armPhase)
        assertNull(now().boardDisarm)
    }

    @Test
    fun `the arm control is disabled while a target reports EV_NO_MOTOR`() = runTest(dispatcher) {
        connect(ONE_BOARD)
        transport.emitCyclicState(QUIET_STATE.copy(fault = CyclicState.EV_NO_MOTOR), from = ONE_BOARD)
        runCurrent()
        val state = now()
        assertTrue(state.armBlocked)
        assertFalse(state.canArm)
        assertTrue(FaultLamp.NO_MOTOR in state.lamps, "the lamp names it")
        assertTrue(state.gridShows)
        viewModel.onArmToggle()
        runCurrent()
        assertEquals(ArmPhase.IDLE, now().armPhase, "the tap is refused")
        assertTrue(transport.sent.none { it.armed })
    }
}
