package com.hoverboard.remote

import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.link.Inputs
import com.hoverboard.remote.link.Telemetry
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * In-memory fake transport for unit tests. Records every [Inputs] frame the ViewModel produced
 * and lets tests drive connection state and inject link [Telemetry], with no Android BLE stack.
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

    /** Inject a link [Telemetry] frame, merging it into the telemetry StateFlow by motor index. */
    fun emitTelemetry(telemetry: Telemetry) {
        _telemetry.value = (_telemetry.value ?: TelemetryUi()).merge(telemetry)
    }
}
