package com.hoverboard.remote

import app.cash.turbine.test
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.model.ConnectionState
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/**
 * Which board a reading belongs to, end to end: the transport files frames under the address they
 * arrived from and the ViewModel hands each screen the board it is about.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TelemetryKeyingTest {

    private val dispatcher = StandardTestDispatcher()
    private lateinit var transport: FakeHoverboardTransport
    private lateinit var viewModel: MainViewModel

    @BeforeEach
    fun setUp() {
        Dispatchers.setMain(dispatcher)
        transport = FakeHoverboardTransport(dispatcher.scheduler)
        viewModel = MainViewModel(transport, LinkSettings(FakeSharedPreferences()), FakeBoardScanner())
    }

    @AfterEach
    fun tearDown() {
        Dispatchers.resetMain()
    }

    /**
     * The two streams on the one link are two boards, each forwarded under its own address, so a
     * merged record would have the Ride screen flickering between them twice a second; keyed by
     * address each board's readings stay its own.
     */
    @Test
    fun `each board's state is resolved by its own address`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial
            transport.setConnectionState(ConnectionState.CONNECTED)
            transport.setAttachedBoard(0x01, 0x02)

            transport.emitCyclicState(CyclicState(-250, 0, 0, 3_780, 3, 0, 0, 0, QUIET_OBS))
            transport.emitCyclicState(
                CyclicState(1_100, 0, 0, 3_612, 0, 0, 0, 0, QUIET_OBS),
                from = 0x02,
            )

            var state = awaitItem()
            while (state.telemetryOf(0x02) == null || state.telemetryOf(0x01) == null) {
                state = awaitItem()
            }
            assertEquals(-2.5f, state.telemetryOf(0x01)!!.pitchDegrees, 0.001f)
            assertEquals(37.80f, state.telemetryOf(0x01)!!.batteryVolts!!, 0.01f)
            assertEquals(11f, state.telemetryOf(0x02)!!.pitchDegrees, 0.001f)
            assertEquals(36.12f, state.telemetryOf(0x02)!!.batteryVolts!!, 0.01f)
            cancelAndIgnoreRemainingEvents()
        }
    }

    /**
     * A session whose walk found one board shows one board, whatever arrives under another address:
     * the strip and the panel list the motor controllers the walk found, not every address heard.
     */
    @Test
    fun `a session that found one board shows one board`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial
            transport.setConnectionState(ConnectionState.CONNECTED)
            transport.setAttachedBoard(0x01)

            transport.emitCyclicState(CyclicState(0, 0, 0, 3_780, 3, 0, 0, 0, QUIET_OBS))
            transport.emitCyclicState(
                CyclicState(0, 0, 0, 3_612, 3, 0, 0, 0, QUIET_OBS),
                from = 0x02,
            )

            var state = awaitItem()
            while (state.telemetry == null) {
                state = awaitItem()
            }
            assertEquals(listOf(0x01), state.motorControllers.map { it.address })
            assertEquals(37.80f, state.telemetry!!.batteryVolts!!, 0.01f)
            cancelAndIgnoreRemainingEvents()
        }
    }
}
