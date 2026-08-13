package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * In-memory fake transport for unit tests. Records every [RiderCommand] the ViewModel produced and
 * lets tests drive connection state and inject a [CyclicState], with no Android BLE stack.
 *
 * It records commands rather than wire bytes on purpose. What the app *intends* is this seam's
 * business; how that intent is spelled on the wire is [RiderCommand.pdus]', and the tests that care
 * about opcodes call it directly rather than trusting a re-implementation here.
 */
class FakeHoverboardTransport : HoverboardTransport {

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

    fun setConnectionState(state: ConnectionState) {
        _connectionState.value = state
    }

    /** Inject a [CyclicState], folding it into the telemetry StateFlow. Latest-wins. */
    fun emitCyclicState(state: CyclicState) {
        _telemetry.value = (_telemetry.value ?: TelemetryUi()).merge(state)
    }
}
