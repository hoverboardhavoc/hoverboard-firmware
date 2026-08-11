package com.hoverboard.remote

import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * In-memory fake transport for unit tests. Records every [Inputs] frame the ViewModel produced
 * and lets tests drive connection state and inject a [CyclicState], with no Android BLE stack.
 */
class FakeHoverboardTransport : HoverboardTransport {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState

    private val _telemetry = MutableStateFlow<TelemetryUi?>(null)
    override val telemetry: StateFlow<TelemetryUi?> = _telemetry

    /** Every [Inputs] the ViewModel produced, in order. */
    val sentInputs: MutableList<Inputs> = mutableListOf()

    val lastInputs: Inputs? get() = sentInputs.lastOrNull()

    var connectCalls: Int = 0
        private set
    var disconnectCalls: Int = 0
        private set

    override fun connect() {
        connectCalls++
    }

    override fun disconnect() {
        disconnectCalls++
    }

    override fun sendInputs(inputs: Inputs) {
        sentInputs.add(inputs)
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
