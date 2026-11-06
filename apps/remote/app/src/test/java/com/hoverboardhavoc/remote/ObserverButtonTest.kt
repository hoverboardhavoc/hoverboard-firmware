package com.hoverboardhavoc.remote

import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboardhavoc.remote.model.ConnectionState
import com.hoverboardhavoc.remote.model.RiderCommand
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The OBS button (`specs/remote-ui.md` 3.1, "The observer button"; `specs/flux-observer.md`, "S4,
 * first slice"): enabled only while armed on a FOC machine, `INPUTS.buttons` bit 1 in every command
 * the pump sends while it is on, and cleared by a tap, a disarm and a disconnect.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class ObserverButtonTest : RiderVmTest() {

    private fun connect(method: Int) {
        transport.store[ONE_BOARD to Fields.MOTOR_METHOD.key()] = Value.U8(method)
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        dispatcher.scheduler.runCurrent()
    }

    private fun observerBit(c: RiderCommand?): Boolean = checkNotNull(c).inputs.buttons and Inputs.BUTTON_OBSERVER != 0

    @Test
    fun `the bit rides every command while on and not after a disarm`() = runTest(dispatcher) {
        connect(Fields.MotorMethod.FOC)
        assertFalse(currentState().canObserve, "disarmed: the button is disabled")
        viewModel.onObserverToggle()
        assertFalse(currentState().observer, "a tap while disarmed does nothing")

        viewModel.onArmToggle()
        assertTrue(currentState().canObserve, "armed on a FOC machine")
        assertFalse(observerBit(transport.last), "armed, not yet pressed")

        viewModel.onObserverToggle()
        assertTrue(currentState().observer)
        val from = transport.sent.size - 1
        assertTrue(observerBit(transport.last), "the press is on the wire")
        assertTrue(checkNotNull(transport.last).inputs.powerRequest(), "beside the arm level")
        viewModel.onThrottleMove(y = 200f, height = h)
        viewModel.onThrottleRelease()
        assertTrue(transport.sent.drop(from).all { observerBit(it) }, "every command while on")

        viewModel.onArmToggle()
        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(observerBit(transport.last), "the disarm releases it in the same frame")
        assertFalse(currentState().observer)

        viewModel.onArmToggle()
        assertFalse(observerBit(transport.last), "a fresh arm starts with it off")
    }

    @Test
    fun `a second tap clears it and a disconnect clears it`() = runTest(dispatcher) {
        connect(Fields.MotorMethod.FOC)
        viewModel.onArmToggle()
        viewModel.onObserverToggle()
        viewModel.onObserverToggle()
        assertFalse(currentState().observer)
        assertFalse(observerBit(transport.last), "tapped off")

        viewModel.onObserverToggle()
        assertTrue(observerBit(transport.last))
        transport.setConnectionState(ConnectionState.DISCONNECTED)
        assertFalse(currentState().observer, "the link going clears it")
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        viewModel.onArmToggle()
        assertFalse(observerBit(transport.last), "and it does not come back with the next arm")
    }

    @Test
    fun `a machine that is not FOC refuses it`() = runTest(dispatcher) {
        connect(Fields.MotorMethod.SIX_STEP)
        viewModel.onArmToggle()
        assertTrue(currentState().armed)
        assertFalse(currentState().focMachine)
        assertFalse(currentState().canObserve)
        viewModel.onObserverToggle()
        assertFalse(currentState().observer)
        assertTrue(transport.sent.none { observerBit(it) })
    }
}
