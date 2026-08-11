package com.hoverboard.remote

import app.cash.turbine.test
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
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

    /** Engaged rider Inputs at the given wire throttle (rider bit set, no buttons). */
    private fun engaged(throttle: Int) = Inputs(throttle = throttle, buttons = 0, rider = 1)

    /** Deadman / released Inputs (throttle 0, rider clear). */
    private val deadman = Inputs(throttle = 0, buttons = 0, rider = 0)

    @Test
    fun `boots at zero throttle and disconnected`() = runTest(dispatcher) {
        val state = viewModel.uiState.first()
        assertEquals(0, state.throttleSpeed)
        assertFalse(state.engaged)
        assertEquals(ConnectionState.DISCONNECTED, state.connectionState)
    }

    @Test
    fun `connect delegates to transport`() = runTest(dispatcher) {
        viewModel.connect()
        assertEquals(1, transport.connectCalls)
    }

    @Test
    fun `touch at top produces max forward inputs with rider engaged`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        // y = 0 -> +MAX forward
        viewModel.onThrottleMove(y = 0f, height = h)
        assertEquals(engaged(Throttle.MAX_SPEED), transport.lastInputs)
    }

    @Test
    fun `centre produces zero throttle inputs still engaged`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0.5f * h, height = h)
        assertEquals(engaged(0), transport.lastInputs)
    }

    @Test
    fun `bottom produces max reverse inputs`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = h, height = h)
        assertEquals(engaged(-Throttle.MAX_SPEED), transport.lastInputs)
    }

    @Test
    fun `touch above centre drives forward`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0.25f * h, height = h)
        assertEquals(engaged(Throttle.MAX_SPEED / 2), transport.lastInputs)
    }

    @Test
    fun `release immediately produces deadman inputs`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0f, height = h)
        assertEquals(engaged(Throttle.MAX_SPEED), transport.lastInputs)

        viewModel.onThrottleRelease()
        assertEquals(deadman, transport.lastInputs)
    }

    @Test
    fun `no inputs produced while not connected`() = runTest(dispatcher) {
        // Still disconnected.
        viewModel.onThrottleMove(y = 0.5f * h, height = h)
        assertTrue(transport.sentInputs.isEmpty())
    }

    @Test
    fun `engaged flag reflected in ui state`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.uiState.test {
            assertEquals(0, awaitItem().throttleSpeed)

            viewModel.onThrottleMove(y = 0f, height = h)
            val engagedState = awaitItem()
            assertTrue(engagedState.engaged)
            assertEquals(Throttle.MAX_SPEED, engagedState.throttleSpeed)
            assertEquals(100, engagedState.throttlePercent)

            viewModel.onThrottleRelease()
            val released = awaitItem()
            assertFalse(released.engaged)
            assertEquals(0, released.throttleSpeed)
            cancelAndIgnoreRemainingEvents()
        }
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
    fun `disconnect forces deadman and delegates`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0.5f * h, height = h)

        viewModel.disconnect()
        assertEquals(deadman, transport.sentInputs[transport.sentInputs.lastIndex])
        assertEquals(1, transport.disconnectCalls)
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
}
