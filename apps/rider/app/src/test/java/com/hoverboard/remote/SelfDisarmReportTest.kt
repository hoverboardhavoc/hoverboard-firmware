package com.hoverboard.remote

import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.model.ConnectionState
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/**
 * The app says when it is the thing that disarmed the machine.
 *
 * Backgrounding is the one disarm in [MainViewModel]'s list the rider did not ask for, and a
 * display timing out is an `ON_STOP`, so it fires at moments nobody is watching: three runs on
 * 2026-10-23 went from resume to stop in 300, 306 and 307 s against the phone's 300,000 ms screen
 * timeout, and the machine read disarmed afterwards with nothing said
 * (`specs/ble-reliability.md`, final section, and
 * `specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`, the run log). The disarm itself
 * stays; what these pin is that it is distinguishable from a fault and from a dropped link.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SelfDisarmReportTest {

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

    /** Connect, and arm. */
    private fun connectAndArm() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.emitCyclicState(QUIET_STATE)
        viewModel.onArmToggle()
    }

    /**
     * [MainViewModel.uiState] is a `stateIn(WhileSubscribed)`, so with no collector it reports its
     * initial value forever and every assertion below would be vacuously true. Subscribe, let the
     * dispatcher run, then read.
     */
    private suspend fun TestScope.currentState(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        advanceUntilIdle()
        return viewModel.uiState.value.also { collector.cancel() }
    }

    @Test
    fun `backgrounding an armed machine is reported as the app's own doing`() = runTest(dispatcher) {
        connectAndArm()

        viewModel.onAppBackgrounded()

        assertFalse(currentState().armed)
        assertTrue(currentState().selfDisarmed)
    }

    /** An app backgrounded with nothing armed disarmed nothing, and claims nothing. */
    @Test
    fun `backgrounding a disarmed machine claims no disarm`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)

        viewModel.onAppBackgrounded()

        assertFalse(currentState().selfDisarmed)
    }

    /** Nor does any disarm the rider performed themselves. */
    @Test
    fun `a disarm the rider asked for claims nothing`() = runTest(dispatcher) {
        connectAndArm()

        viewModel.onArmToggle()
        assertFalse(currentState().selfDisarmed)

        viewModel.onArmToggle()
        viewModel.disconnect()
        assertFalse(currentState().selfDisarmed)
        advanceUntilIdle()
    }

    /** The report is answered by the next arm: it describes the last disarm, not this machine. */
    @Test
    fun `arming again clears the app's own-doing report`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onAppBackgrounded()
        assertTrue(currentState().selfDisarmed)

        viewModel.onArmToggle()

        assertTrue(currentState().armed)
        assertFalse(currentState().selfDisarmed)
    }

    /** So is losing the link, which ends the session and is its own, different, explanation. */
    @Test
    fun `losing the link clears the app's own-doing report`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onAppBackgrounded()
        assertTrue(currentState().selfDisarmed)

        transport.setConnectionState(ConnectionState.DISCONNECTED)

        assertFalse(currentState().selfDisarmed)
    }
}
