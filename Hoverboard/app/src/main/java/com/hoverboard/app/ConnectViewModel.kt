package com.hoverboard.app

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.hoverboard.app.ble.HoverboardTransport
import com.hoverboard.app.model.ConnectionState
import com.hoverboard.app.net.l3.WalkOutcome
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

/** The connect screen's discovery (L3 walk) state. */
sealed interface DiscoveryUiState {
    /** No walk run yet (or reset on disconnect). */
    data object Idle : DiscoveryUiState

    /** A walk is in flight. */
    data object Running : DiscoveryUiState

    /** The walk finished: the discovered fleet. */
    data class Done(val outcome: WalkOutcome) : DiscoveryUiState

    /** The walk could not run (not connected) or failed. */
    data class Failed(val reason: String) : DiscoveryUiState
}

/**
 * Connect-screen ViewModel: a thin MVVM seam over the injected [HoverboardTransport]. It exposes the
 * BLE [ConnectionState] + forwards connect/disconnect, and (slice 4) runs the controller-side L3 walk
 * over the live BLE link via [discover], surfacing the discovered fleet as [DiscoveryUiState].
 */
@HiltViewModel
class ConnectViewModel @Inject constructor(
    private val transport: HoverboardTransport,
) : ViewModel() {

    /** The BLE link lifecycle, driven by the transport. */
    val connectionState: StateFlow<ConnectionState> = transport.connectionState

    private val _discovery = MutableStateFlow<DiscoveryUiState>(DiscoveryUiState.Idle)

    /** The L3 walk result for the connect screen. */
    val discovery: StateFlow<DiscoveryUiState> = _discovery.asStateFlow()

    fun connect() = transport.connect()

    fun disconnect() {
        transport.disconnect()
        _discovery.value = DiscoveryUiState.Idle
    }

    /** Run the controller-side walk over the connected link, publishing the outcome. */
    fun discover() {
        if (_discovery.value is DiscoveryUiState.Running) return
        _discovery.value = DiscoveryUiState.Running
        viewModelScope.launch {
            _discovery.value = runCatching { transport.discover() }
                .fold(
                    onSuccess = { outcome ->
                        if (outcome == null) {
                            DiscoveryUiState.Failed("Not connected")
                        } else {
                            DiscoveryUiState.Done(outcome)
                        }
                    },
                    onFailure = { DiscoveryUiState.Failed(it.message ?: "Walk failed") },
                )
        }
    }
}
