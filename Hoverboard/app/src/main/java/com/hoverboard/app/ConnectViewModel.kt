package com.hoverboard.app

import androidx.lifecycle.ViewModel
import com.hoverboard.app.ble.HoverboardTransport
import com.hoverboard.app.model.ConnectionState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.StateFlow
import javax.inject.Inject

/**
 * Connect-screen ViewModel (slice 1): a thin MVVM seam over the injected [HoverboardTransport].
 * It exposes the BLE [ConnectionState] and forwards connect/disconnect intents. Drive/telemetry
 * state arrives in later slices.
 */
@HiltViewModel
class ConnectViewModel @Inject constructor(
    private val transport: HoverboardTransport,
) : ViewModel() {

    /** The BLE link lifecycle, driven by the transport. */
    val connectionState: StateFlow<ConnectionState> = transport.connectionState

    fun connect() = transport.connect()

    fun disconnect() = transport.disconnect()
}
