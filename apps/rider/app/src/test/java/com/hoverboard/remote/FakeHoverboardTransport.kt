package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.test.TestCoroutineScheduler
import kotlinx.coroutines.test.runCurrent

/**
 * In-memory fake transport for unit tests. Records every [RiderCommand] the ViewModel produced and
 * lets tests drive connection state and inject a [CyclicState], with no Android BLE stack.
 *
 * It records commands rather than wire bytes on purpose. What the app *intends* is this seam's
 * business; how that intent is spelled on the wire is [RiderCommand.pdus]', and the tests that care
 * about opcodes call it directly rather than trusting a re-implementation here.
 *
 * @param scheduler the test scheduler driving the ViewModel, so [setConnectionState] can deliver
 *   each change rather than leaving it to a caller who might not.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class FakeHoverboardTransport(
    private val scheduler: TestCoroutineScheduler,
) : HoverboardTransport {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState

    private val _telemetry = MutableStateFlow<TelemetryUi?>(null)
    override val telemetry: StateFlow<TelemetryUi?> = _telemetry

    /** Every [RiderCommand] the ViewModel produced, in order. */
    val sent: MutableList<RiderCommand> = mutableListOf()

    val last: RiderCommand? get() = sent.lastOrNull()

    var connectCalls: Int = 0
        private set
    var disconnectCalls: Int = 0
        private set

    /**
     * How many commands had been sent by the time [disconnect] was called, or null if it never was.
     *
     * This is what lets a test check ORDER rather than just contents: the guarantee that matters on
     * the disconnect path is that the disarming command went out BEFORE the link was dropped, and a
     * fake that only records the final state cannot tell those two apart.
     */
    var sentAtDisconnect: Int? = null
        private set

    override fun connect() {
        connectCalls++
    }

    override fun disconnect() {
        disconnectCalls++
        sentAtDisconnect = sent.size
    }

    override fun sendCommand(command: RiderCommand) {
        sent.add(command)
    }

    // --- Test driving helpers ---

    /**
     * Move the link to [state] AND let the scheduler deliver it.
     *
     * The advance is the whole point, and it is here rather than left to each caller because
     * forgetting it produces a silent wrong answer rather than a failure. `_connectionState` is a
     * [MutableStateFlow], which CONFLATES AND DEDUPLICATES: a test that sets CONNECTED and then
     * DISCONNECTED with no scheduler turn in between never lets
     * [com.hoverboard.remote.MainViewModel]'s collector observe CONNECTED at all, so when it does
     * resume the value equals the one it last saw and there is NO EMISSION. The force-disarm that
     * a dropped link is supposed to trigger then simply never runs, and the test reads as if the
     * ViewModel ignored the drop.
     *
     * That cost real time once: it looks exactly like a ViewModel bug, and it is not one. Driving
     * the flow through this helper makes every state change observable by construction, so the
     * hazard cannot come back in the next test someone writes.
     *
     * [TestCoroutineScheduler.runCurrent] rather than `advanceUntilIdle`: it is enough to deliver
     * the emission (the collector resumes with no delay) and it cannot fast-forward virtual time
     * through a pending `delay`, which COULD silently collapse windows like
     * [com.hoverboard.remote.MainViewModel.DISARM_SETTLE_MS]. No test today would fail under
     * `advanceUntilIdle` (the one settle-window test sets its connection state after virtual time
     * has already passed the window), so this is a hazard the choice forecloses rather than a
     * failure it currently prevents. It costs nothing to keep it foreclosed.
     */
    fun setConnectionState(state: ConnectionState) {
        _connectionState.value = state
        scheduler.runCurrent()
    }

    /** Inject a [CyclicState], folding it into the telemetry StateFlow. Latest-wins. */
    fun emitCyclicState(state: CyclicState) {
        _telemetry.value = (_telemetry.value ?: TelemetryUi()).merge(state)
    }
}
