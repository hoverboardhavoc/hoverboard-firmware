package com.hoverboard.remote

import androidx.lifecycle.ViewModelStore
import app.cash.turbine.test
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

@OptIn(ExperimentalCoroutinesApi::class)
class MainViewModelTest {

    private val dispatcher = StandardTestDispatcher()
    private lateinit var transport: FakeHoverboardTransport
    private lateinit var settings: LinkSettings
    private lateinit var viewModel: MainViewModel

    @BeforeEach
    fun setUp() {
        Dispatchers.setMain(dispatcher)
        transport = FakeHoverboardTransport()
        settings = LinkSettings(FakeSharedPreferences())
        viewModel = MainViewModel(transport, settings)
    }

    @AfterEach
    fun tearDown() {
        Dispatchers.resetMain()
    }

    private val h = 1000f

    /** Connect, and arm by pressing and holding the arm control. */
    private fun connectAndArm() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onArmToggle()
    }

    /**
     * The current [UiState], with the flow actually turned.
     *
     * [MainViewModel.uiState] is a `stateIn(WhileSubscribed)`, so with no collector its upstream
     * never runs and it reports its initial value forever. Reading it with a bare `first()` makes
     * every assertion about a non-default field vacuously true, which is worth exactly nothing in
     * tests whose whole job is to prove the machine ends up disarmed. This subscribes, lets the
     * dispatcher run, and then reads.
     */
    private suspend fun TestScope.currentState(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        advanceUntilIdle()
        return viewModel.uiState.value.also { collector.cancel() }
    }

    // --- arming ---------------------------------------------------------------------------------

    @Test
    fun `boots disarmed, at zero, disconnected`() = runTest(dispatcher) {
        val state = viewModel.uiState.first()
        assertFalse(state.armed)
        assertEquals(0, state.throttleSpeed)
        assertFalse(state.engaged)
        assertEquals(ConnectionState.DISCONNECTED, state.connectionState)
    }

    @Test
    fun `connecting does not arm`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        advanceUntilIdle()
        // Arming has to be an act, not a consequence of the link coming up.
        assertFalse(currentState().armed)
        assertTrue(transport.sent.none { it.armed })
    }

    @Test
    fun `arming asserts power and toggling off clears it`() = runTest(dispatcher) {
        connectAndArm()
        assertTrue(checkNotNull(transport.last).armed)
        assertTrue(checkNotNull(transport.last).inputs.powerRequest())

        viewModel.onArmToggle()
        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(checkNotNull(transport.last).inputs.powerRequest())
    }

    @Test
    fun `the throttle alone never arms and never demands`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0f, height = h) // full forward, unarmed

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertTrue(transport.sent.none { it.armed || it.demand != 0 })
        // The pad must not display travel that is not being commanded.
        assertEquals(0, currentState().throttleSpeed)
    }

    @Test
    fun `arming is refused while the throttle is held`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0f, height = h) // finger down on a deflected throttle
        assertFalse(currentState().canArm)

        viewModel.onArmToggle()
        assertFalse(checkNotNull(transport.last).armed, "must not arm into a deflected throttle")

        // Let go, and the same press is accepted.
        viewModel.onThrottleRelease()
        viewModel.onArmToggle()
        assertTrue(checkNotNull(transport.last).armed)
    }

    @Test
    fun `disarming is never refused, even with the throttle held`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h) // engaged, so ARMING would be refused

        viewModel.onArmToggle()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `armed throttle travel is commanded as a demand`() = runTest(dispatcher) {
        connectAndArm()

        viewModel.onThrottleMove(y = 0f, height = h)
        assertEquals(Throttle.MAX_SPEED, checkNotNull(transport.last).demand)

        viewModel.onThrottleMove(y = h, height = h)
        assertEquals(-Throttle.MAX_SPEED, checkNotNull(transport.last).demand)

        viewModel.onThrottleMove(y = 0.5f * h, height = h)
        assertEquals(0, checkNotNull(transport.last).demand)
    }

    @Test
    fun `releasing the throttle zeroes the demand but stays armed`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        viewModel.onThrottleRelease()
        val last = checkNotNull(transport.last)
        assertEquals(0, last.demand)
        assertTrue(last.armed, "a rider resting at a stop has not stopped riding")
    }

    @Test
    fun `nothing is sent while not connected`() = runTest(dispatcher) {
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        assertTrue(transport.sent.isEmpty())
    }

    // --- the four release paths, each ending disarmed --------------------------------------------

    @Test
    fun `release path 1, tapping the arm control off disarms`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)
        assertTrue(checkNotNull(transport.last).armed)

        viewModel.onArmToggle()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `release path 2, backgrounding the app disarms`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        // No pointer-cancel: Android does not promise one when it takes the window away.
        viewModel.onAppBackgrounded()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `release path 3, disconnecting disarms BEFORE the link is dropped`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        viewModel.disconnect()

        // The disarming command is produced immediately...
        assertEquals(RiderCommand.DISARMED, transport.last)
        val sentBefore = transport.sent.size

        // ... and the link is held open for it. Dropping it first would strand the board armed:
        // the firmware holds the last INPUTS level it was delivered with no staleness at all.
        assertEquals(0, transport.disconnectCalls, "link dropped before the disarm could be sent")

        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS + 1)
        runCurrent()

        assertEquals(1, transport.disconnectCalls)
        assertEquals(sentBefore, transport.sentAtDisconnect)
        assertEquals(RiderCommand.DISARMED, transport.sent[transport.sentAtDisconnect!! - 1])
        assertTrue(
            MainViewModel.DISARM_SETTLE_MS >= 2 * LinkConfig.SEND_INTERVAL_MS,
            "the window must cover more than one pump tick on a link with no retransmit",
        )
    }

    @Test
    fun `release path 4, losing the link disarms the app and requires a fresh press`() =
        runTest(dispatcher) {
            connectAndArm()
            viewModel.onThrottleMove(y = 0f, height = h)
            assertTrue(currentState().armed)

            // The link drops on its own; nothing can be sent from here on.
            transport.setConnectionState(ConnectionState.DISCONNECTED)
            assertFalse(currentState().armed)

            // Reconnecting must not resume the arm level off a finger that never lifted.
            val sentBefore = transport.sent.size
            transport.setConnectionState(ConnectionState.CONNECTED)
            advanceUntilIdle()
            assertEquals(sentBefore, transport.sent.size, "reconnect must not re-assert power")
            assertFalse(currentState().armed)

            viewModel.onArmToggle()
            assertTrue(checkNotNull(transport.last).armed)
        }

    /**
     * The settle window is not a quiet period: the link is still up, the control screen is still on
     * screen, and a finger is still on the glass. If the arm control works inside it, one mis-tap
     * re-arms the board and the delayed drop then leaves it armed with nothing left that could
     * correct it: the firmware latches the last `INPUTS` level with no age at all. The app would
     * meanwhile reset itself to disarmed and render "disconnected", which is the worst possible
     * pair: a live machine that the app believes it has left.
     */
    @Test
    fun `a tap inside the settle window cannot leave the board armed`() = runTest(dispatcher) {
        backgroundScope.launch { viewModel.uiState.collect {} }
        runCurrent()
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()
        assertTrue(viewModel.uiState.value.armed)

        viewModel.disconnect()
        runCurrent()
        val disarmedAt = transport.sent.size
        assertEquals(RiderCommand.DISARMED, transport.last)

        // Half way through the window. Nothing about the UI has changed yet: still CONNECTED, still
        // the control screen, and the drop is still pending.
        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS / 2)
        runCurrent()
        assertEquals(ConnectionState.CONNECTED, viewModel.uiState.value.connectionState)
        assertEquals(0, transport.disconnectCalls)

        // The mis-tap, and a finger that follows it onto the throttle.
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()

        // The window runs out and the link goes.
        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS)
        runCurrent()

        assertEquals(1, transport.disconnectCalls)
        val atDrop = checkNotNull(transport.sentAtDisconnect)
        assertFalse(
            transport.sent[atDrop - 1].armed,
            "the link was dropped with the board armed; the firmware holds that level forever",
        )
        assertTrue(
            transport.sent.subList(disarmedAt, atDrop).none { it.armed },
            "nothing may re-assert power between the disarm and the drop",
        )
        assertFalse(viewModel.uiState.value.armed)
    }

    /**
     * The same refusal as the test above, rendered for the user rather than enforced on the wire.
     * A control that silently ignores a press is a control the rider will press again; the arm
     * button has to go unavailable for the whole window, and come back only with the link.
     */
    @Test
    fun `the arm control is unavailable for the whole settle window`() = runTest(dispatcher) {
        backgroundScope.launch { viewModel.uiState.collect {} }
        runCurrent()
        connectAndArm()
        runCurrent()
        assertTrue(viewModel.uiState.value.canArm)

        viewModel.disconnect()
        runCurrent()
        assertFalse(viewModel.uiState.value.canArm, "refused as soon as the disconnect is asked for")

        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS - 1)
        runCurrent()
        assertEquals(ConnectionState.CONNECTED, viewModel.uiState.value.connectionState)
        assertFalse(viewModel.uiState.value.canArm, "still refused on the last tick of the window")

        // A fresh session arms normally: the refusal is scoped to the disconnect, not sticky.
        advanceTimeBy(2)
        runCurrent()
        transport.setConnectionState(ConnectionState.DISCONNECTED)
        runCurrent()
        assertFalse(viewModel.uiState.value.canArm, "nothing is armable while disconnected")
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertTrue(viewModel.uiState.value.canArm, "a new connection must be armable again")
    }

    /**
     * The settle runs in `viewModelScope`, so a ViewModel cleared inside the window would cancel it
     * with the link still up and the transport still holding a session. The board is already
     * disarmed and pinned there by the pump, so this is a leak rather than a hazard, but the link
     * must not outlive the thing that owns it.
     */
    @Test
    fun `a ViewModel cleared inside the settle window still drops the link`() = runTest(dispatcher) {
        val store = ViewModelStore()
        store.put("main", viewModel)
        connectAndArm()

        viewModel.disconnect()
        runCurrent()
        assertEquals(0, transport.disconnectCalls, "the window has not run out yet")

        // The activity goes away mid-window: clear() cancels viewModelScope, then calls onCleared().
        store.clear()
        advanceUntilIdle()

        assertEquals(1, transport.disconnectCalls, "the link outlived the ViewModel that owned it")
    }

    // --- ui state -------------------------------------------------------------------------------

    @Test
    fun `armed and engaged are both reflected in ui state`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.uiState.test {
            assertFalse(awaitItem().armed)

            viewModel.onArmToggle()
            var state = awaitItem()
            while (!state.armed) state = awaitItem()
            assertFalse(state.engaged)

            viewModel.onThrottleMove(y = 0f, height = h)
            state = awaitItem()
            while (!state.engaged) state = awaitItem()
            assertEquals(Throttle.MAX_SPEED, state.throttleSpeed)
            assertEquals(100, state.throttlePercent)

            viewModel.onArmToggle()
            state = awaitItem()
            while (state.armed) state = awaitItem()
            assertEquals(0, state.throttleSpeed)
            assertFalse(state.engaged)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `connect delegates to transport`() = runTest(dispatcher) {
        viewModel.connect()
        assertEquals(1, transport.connectCalls)
    }

    @Test
    fun `injected telemetry maps into ui state with battery percent`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial

            transport.setConnectionState(ConnectionState.CONNECTED)
            // batt 3555 cV = 35.55 V -> mid pack, not low. CYCLIC_STATE.battery is CENTIvolts
            // (crates/orchestrator/src/dispatch.rs:42,113), not the millivolts this app used to
            // assume; reading it as mV would show a 36 V pack as 3.6 V and trip battery-low.
            transport.emitCyclicState(
                CyclicState(
                    pitch = -250,
                    roll = 125,
                    wheelSpeed = 600,
                    battery = 3_555,
                    mode = 2,
                    fault = 0,
                    flags = CyclicState.FLAG_RIDER,
                ),
            )

            var state = awaitItem()
            while (state.telemetry == null) {
                state = awaitItem()
            }
            assertTrue(state.isConnected)
            val telem = state.telemetry!!
            assertEquals(35.55f, telem.batteryVolts, 0.01f)
            assertEquals(600, telem.speedRaw)
            assertEquals(-2.5f, telem.pitchDegrees, 0.001f)
            assertEquals(1.25f, telem.rollDegrees, 0.001f)
            assertTrue(telem.riderPresent)
            assertFalse(telem.lockdown)
            assertFalse(telem.batteryLow)
            // BatteryCurve maps the pack voltage; sanity-check it is invoked.
            BatteryCurve.percent(telem.batteryVolts)
            cancelAndIgnoreRemainingEvents()
        }
    }

    /**
     * Replaces the old "telemetry merges latest per motor index".
     *
     * There is no per-motor telemetry frame any more: CYCLIC_STATE is one board-level record
     * (`crates/linkctl/src/lib.rs:88-106`) with no motor index and no per-wheel current, and it is
     * best-effort latest-wins (`crates/linkctl/src/lib.rs:104-105`). So the property worth pinning
     * flipped from "keep one entry per motor" to "the newest state wins outright".
     */
    @Test
    fun `cyclic state is latest-wins`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial
            transport.setConnectionState(ConnectionState.CONNECTED)

            transport.emitCyclicState(
                CyclicState(0, 0, 100, 3_600, 1, 0, 0),
            )
            transport.emitCyclicState(
                CyclicState(0, 0, 250, 3_550, 2, 0, CyclicState.FLAG_RIDER),
            )

            var state = awaitItem()
            while (state.telemetry?.cyclic?.wheelSpeed != 250) {
                state = awaitItem()
            }
            val telem = state.telemetry!!
            assertEquals(250, telem.speedRaw)
            assertEquals(35.5f, telem.batteryVolts, 0.01f)
            assertTrue(telem.riderPresent)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `the ui carries the configured board name, defaulting to Hoverboard`() = runTest(dispatcher) {
        // The Scan button labels itself with this, so it has to reach the UI state.
        assertEquals(LinkConfig.DEFAULT_DEVICE_NAME, viewModel.uiState.first().deviceName)
    }

    @Test
    fun `setting the board name reaches both the ui and the store`() = runTest(dispatcher) {
        viewModel.setDeviceName("hb-offroad-m")

        assertEquals("hb-offroad-m", settings.deviceName.value)
        viewModel.uiState.test {
            var state = awaitItem()
            while (state.deviceName != "hb-offroad-m") {
                state = awaitItem()
            }
            assertEquals("hb-offroad-m", state.deviceName)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `every disarming path produces the same single command`() = runTest(dispatcher) {
        // One value, so no path can disarm halfway: there is no representable command that has let
        // go of the arm level while still carrying a demand.
        val paths = listOf<(MainViewModel) -> Unit>(
            { it.onArmToggle() },
            { it.onAppBackgrounded() },
            { it.disconnect() },
        )
        for (path in paths) {
            transport = FakeHoverboardTransport()
            viewModel = MainViewModel(transport, settings)
            connectAndArm()
            viewModel.onThrottleMove(y = 0f, height = h)

            path(viewModel)

            val last = checkNotNull(transport.last)
            assertNotNull(last)
            assertEquals(RiderCommand.DISARMED, last)
            assertFalse(last.inputs.powerRequest())
            assertEquals(0, last.demand)
            advanceUntilIdle()
        }
    }
}
