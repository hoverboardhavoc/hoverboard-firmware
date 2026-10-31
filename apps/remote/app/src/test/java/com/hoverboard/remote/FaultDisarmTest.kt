package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.FaultDisarm
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.RiderCommand
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
 * What the view model does with the boards' fault words (`specs/remote-ui.md` 3.1 and 3.1b): the
 * fault disarm, the latch hold and the lamp test.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class FaultDisarmTest : RiderVmTest() {

    /** A board's frame at rest, carrying [fault]. */
    private fun frame(fault: Int) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = 3_780,
        mode = 3,
        fault = fault,
        flags = 0,
        peerDemand = 0,
        wheelPos = 0,
        wheelRate = 0,
        obs = QUIET_OBS,
    )

    /** The current state, read while a collector keeps the flow turning and virtual time is not advanced. */
    private fun TestScope.now(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        runCurrent()
        return viewModel.uiState.value.also { collector.cancel() }
    }

    private fun TestScope.connectAndArm() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.emitCyclicState(frame(0))
        runCurrent()
        viewModel.onArmToggle()
        runCurrent()
        check(now().armed)
    }

    // --- the fault disarm ----------------------------------------------------------------------

    /**
     * An arm-blocking bit on an armed board drops the arm level on that frame and sets
     * faultDisarm; the bit clearing does not raise the level again; the next ARM tap clears it.
     */
    @Test
    fun `a blocking bit on an armed board disarms on that frame and stays disarmed`() = runTest(dispatcher) {
        connectAndArm()

        transport.emitCyclicState(frame(CyclicState.EV_MOTOR_PERIOD))
        runCurrent()
        var state = now()
        assertFalse(state.armed)
        assertEquals(FaultDisarm(FaultLamp.PERIOD, ONE_BOARD), state.faultDisarm)
        assertEquals(RiderCommand.DISARMED, transport.last, "the level is off the wire")

        // The lamp goes out a frame later: nothing re-arms, and the statement stays.
        transport.emitCyclicState(frame(0))
        runCurrent()
        state = now()
        assertFalse(state.armed)
        assertEquals(FaultDisarm(FaultLamp.PERIOD, ONE_BOARD), state.faultDisarm)
        assertEquals(RiderCommand.DISARMED, transport.last)

        viewModel.onArmToggle()
        runCurrent()
        state = now()
        assertTrue(state.armed)
        assertNull(state.faultDisarm, "the ARM tap clears it")
    }

    /** The power request and the mode readback are in the word and are not arm-blocking. */
    @Test
    fun `bits outside the arm-blocking set do not disarm`() = runTest(dispatcher) {
        connectAndArm()
        for (bit in listOf(CyclicState.EV_POWER_REQUEST, CyclicState.EV_MODE_FAULT, 1 shl 3, 1 shl 12)) {
            transport.emitCyclicState(frame(bit))
            runCurrent()
            assertTrue(now().armed, "bit $bit")
            assertNull(now().faultDisarm)
        }
    }

    /** While a board reports a blocking bit, the arm is refused: that board would refuse it anyway. */
    @Test
    fun `a live blocking bit refuses the arm`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.emitCyclicState(frame(CyclicState.EV_MOTOR_HALL))
        runCurrent()
        assertFalse(now().canArm)
        viewModel.onArmToggle()
        runCurrent()
        assertFalse(now().armed)
    }

    // --- the latch hold ------------------------------------------------------------------------

    /**
     * A LATCH bit seen for one frame stays lit after it clears, per board and per bit, until the
     * next ARM tap, and is dropped on disconnect; ARM is not refused while only a held lamp is lit.
     */
    @Test
    fun `a latch seen once is held until the next arm tap`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        transport.emitCyclicState(frame(CyclicState.EV_LATCH_A), from = 0x01)
        transport.emitCyclicState(frame(0), from = 0x02)
        runCurrent()
        transport.emitCyclicState(frame(0), from = 0x01)
        transport.emitCyclicState(frame(CyclicState.EV_LATCH_B), from = 0x02)
        runCurrent()
        transport.emitCyclicState(frame(0), from = 0x02)
        runCurrent()

        var state = now()
        assertEquals(mapOf(0x01 to CyclicState.EV_LATCH_A, 0x02 to CyclicState.EV_LATCH_B), state.latchHeld)
        assertEquals(CyclicState.EV_LATCH_A, state.lampWords[0x01])
        assertEquals(CyclicState.EV_LATCH_B, state.lampWords[0x02])
        assertTrue(state.gridShows, "a held lamp shows the grid")
        assertTrue(FaultLamp.LATCH_2 in state.lamps, "a held LATCH 2 is a set bit, never hidden")
        assertTrue(state.canArm, "a held lamp refuses nothing")

        viewModel.onArmToggle()
        runCurrent()
        state = now()
        assertTrue(state.armed)
        assertTrue(state.latchHeld.isEmpty(), "the tap that arms clears the hold")
    }

    @Test
    fun `the latch hold is dropped on disconnect`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.emitCyclicState(frame(CyclicState.EV_LATCH_A))
        runCurrent()
        assertEquals(mapOf(ONE_BOARD to CyclicState.EV_LATCH_A), now().latchHeld)

        transport.setConnectionState(ConnectionState.DISCONNECTED)
        assertTrue(now().latchHeld.isEmpty())
    }

    /** A latch on an armed board is also a fault disarm: LATCH is arm-blocking while it is set. */
    @Test
    fun `a latch on an armed board disarms and is held`() = runTest(dispatcher) {
        connectAndArm()
        transport.emitCyclicState(frame(CyclicState.EV_LATCH_A))
        runCurrent()
        transport.emitCyclicState(frame(0))
        runCurrent()

        val state = now()
        assertFalse(state.armed)
        assertEquals(FaultDisarm(FaultLamp.LATCH, ONE_BOARD), state.faultDisarm)
        assertEquals(CyclicState.EV_LATCH_A, state.latchHeld[ONE_BOARD])
        assertTrue(state.canArm, "tapping ARM clears the lamp and arms")
    }

    // --- the lamp test -------------------------------------------------------------------------

    /** Once per connection, 2 s on the test clock, from the ride facts read's first completion. */
    @Test
    fun `the lamp test runs once per connection for two seconds`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertTrue(now().lampTest, "on when the ride facts are in")
        advanceTimeBy(MainViewModel.LAMP_TEST_MS - 1)
        runCurrent()
        assertTrue(now().lampTest)
        advanceTimeBy(2)
        runCurrent()
        assertFalse(now().lampTest)

        // A walk that re-reads the facts within the connection does not run it again.
        transport.setAttachedBoard(null)
        transport.setAttachedBoard(ONE_BOARD)
        runCurrent()
        assertFalse(now().lampTest)

        // A new connection does.
        transport.setConnectionState(ConnectionState.DISCONNECTED)
        transport.setAttachedBoard(null)
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertTrue(now().lampTest)
    }

    @Test
    fun `the lamp test is skipped while a blocking bit is set`() = runTest(dispatcher) {
        transport.emitCyclicState(frame(CyclicState.EV_MOTOR_HALL), from = ONE_BOARD)
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertFalse(now().lampTest)
    }

    @Test
    fun `the lamp test ends the moment a blocking bit appears`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertTrue(now().lampTest)

        transport.emitCyclicState(frame(CyclicState.EV_IMU_LOSS))
        runCurrent()
        assertFalse(now().lampTest)
    }
}
