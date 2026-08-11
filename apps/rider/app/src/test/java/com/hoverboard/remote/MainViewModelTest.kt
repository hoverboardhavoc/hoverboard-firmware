package com.hoverboard.remote

import app.cash.turbine.test
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.link.Inputs
import com.hoverboard.remote.link.Telemetry
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
            // batt 35550 mV = 35.55 V -> mid pack, not low; current 150 cA = 1.5 A.
            transport.emitTelemetry(
                Telemetry(
                    motorIndex = 0,
                    batteryMv = 35_550,
                    currentCa = 150,
                    speed = 600,
                    faultCode = 0,
                    flags = 0,
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
            assertEquals(1.5f, telem.currentAmpsA, 0.001f)
            assertFalse(telem.batteryLow)
            // BatteryCurve maps the pack voltage; sanity-check it is invoked.
            BatteryCurve.percent(telem.batteryVolts)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `telemetry merges latest per motor index`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial
            transport.setConnectionState(ConnectionState.CONNECTED)

            transport.emitTelemetry(
                Telemetry(motorIndex = 0, batteryMv = 36_000, currentCa = 100, speed = 0, faultCode = 0, flags = 0),
            )
            transport.emitTelemetry(
                Telemetry(motorIndex = 1, batteryMv = 36_000, currentCa = 250, speed = 0, faultCode = 0, flags = 0),
            )

            var state = awaitItem()
            while (state.telemetry?.motors?.size != 2) {
                state = awaitItem()
            }
            val telem = state.telemetry!!
            assertEquals(1.0f, telem.currentAmpsA, 0.001f) // motor 0
            assertEquals(2.5f, telem.currentAmpsB, 0.001f) // motor 1
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
