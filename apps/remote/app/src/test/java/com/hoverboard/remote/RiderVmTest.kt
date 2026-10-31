package com.hoverboard.remote

import com.hoverboard.remote.ble.LinkSettings
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach

/**
 * One [MainViewModel] on a test dispatcher, with its fake transport, settings and scanner: the
 * fixture every ViewModel suite starts from.
 *
 * It exists because there is more than one such suite. The ViewModel answers two separate questions
 * (what the arm control and the link do, and what the drive modes put on the wire), and they are
 * asked by two classes against ONE fixture rather than by one class twice the size or by two copies
 * of the same four lines of setup.
 */
@OptIn(ExperimentalCoroutinesApi::class)
abstract class RiderVmTest {

    protected val dispatcher = StandardTestDispatcher()
    protected lateinit var transport: FakeHoverboardTransport
    protected lateinit var settings: LinkSettings
    protected lateinit var viewModel: MainViewModel

    @BeforeEach
    fun setUpViewModel() {
        Dispatchers.setMain(dispatcher)
        transport = FakeHoverboardTransport(dispatcher.scheduler)
        settings = LinkSettings(FakeSharedPreferences())
        viewModel = MainViewModel(transport, settings, FakeBoardScanner())
    }

    @AfterEach
    fun tearDownViewModel() {
        Dispatchers.resetMain()
    }

    /**
     * Each of [boards] sends a quiet frame, as a powered board does: a session in which no motor
     * controller reports is the boards switched off, and the app drops the arm level 1 s in
     * (`specs/remote-ui.md` 3.1, "Motor controller off"). The fake never ages a board out.
     */
    protected fun reporting(vararg boards: Int) {
        for (b in boards) transport.emitCyclicState(QUIET_STATE, from = b)
    }

    /** The throttle pad's height; its y axis is the demand. */
    protected val h = 1000f

    /** The joystick pad's width; its x axis is the turn, as its height is the travel. */
    protected val w = 800f

    /**
     * The current [UiState], with the flow actually turned.
     *
     * [MainViewModel.uiState] is a `stateIn(WhileSubscribed)`, so with no collector its upstream
     * never runs and it reports its initial value forever. Reading it with a bare `first()` makes
     * every assertion about a non-default field vacuously true, which is worth exactly nothing in
     * tests whose whole job is to prove the machine ends up disarmed. This subscribes, lets the
     * dispatcher run, and then reads.
     */
    protected suspend fun TestScope.currentState(): UiState {
        val collector = launch { viewModel.uiState.collect {} }
        advanceUntilIdle()
        return viewModel.uiState.value.also { collector.cancel() }
    }
}
