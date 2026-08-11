package com.hoverboard.remote

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.link.Inputs
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update

/**
 * UI state surfaced to Compose.
 *
 * @param connectionState BLE link state.
 * @param telemetry latest merged telemetry, or null before the first frame.
 * @param throttleSpeed current commanded wire speed (-MAX..MAX), 0 when not engaged.
 * @param engaged whether the deadman is currently held in the engage zone.
 */
data class UiState(
    val connectionState: ConnectionState = ConnectionState.DISCONNECTED,
    val telemetry: TelemetryUi? = null,
    val throttleSpeed: Int = 0,
    val engaged: Boolean = false,
    val deviceName: String = LinkConfig.DEFAULT_DEVICE_NAME,
) {
    val isConnected: Boolean get() = connectionState == ConnectionState.CONNECTED

    /** Commanded throttle as a percent of MAX_SPEED, signed (-100..100). */
    val throttlePercent: Int get() = (throttleSpeed * PERCENT) / Throttle.MAX_SPEED

    private companion object {
        const val PERCENT = 100
    }
}

/**
 * Owns the deadman throttle + safety logic and bridges the [HoverboardTransport].
 *
 * The app is a virtual-rider node: a held finger produces an engaged [Inputs] frame
 * (rider = 1) carrying the wire throttle; a lift produces the deadman [Inputs] (throttle 0,
 * rider 0) immediately.
 *
 * Safety invariants:
 *  - Boots at speed 0 (engaged = false); motion only on an explicit engaged touch.
 *  - Finger-up ([onThrottleRelease]) immediately produces the deadman Inputs.
 *  - Inputs are only produced while connected; a disconnect resets throttle to 0.
 */
class MainViewModel(
    private val transport: HoverboardTransport,
    private val settings: LinkSettings,
    private val maxSpeed: Int = Throttle.MAX_SPEED,
) : ViewModel() {

    private val local = MutableStateFlow(LocalState())

    val uiState: StateFlow<UiState> =
        combine(
            transport.connectionState,
            transport.telemetry,
            local,
            settings.deviceName,
        ) { conn, telem, l, name ->
            UiState(
                connectionState = conn,
                telemetry = telem,
                throttleSpeed = l.throttleSpeed,
                engaged = l.engaged,
                deviceName = name,
            )
        }.stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(STATE_TIMEOUT_MS),
            initialValue = UiState(),
        )

    /** Begin scanning + connecting to the peripheral. */
    fun connect() = transport.connect()

    /**
     * Retarget the app at a differently-named board. Persisted immediately; the transport reads the
     * name at the start of each scan, so this takes effect on the next connect without a restart.
     */
    fun setDeviceName(name: String) = settings.setDeviceName(name)

    /** Disconnect and force throttle to 0. */
    fun disconnect() {
        forceStop()
        transport.disconnect()
    }

    /**
     * Handle a deadman touch at [y] within a pad of [height] (pixels, y down). Computes the
     * throttle mapping (engage gate + forward/reverse), updates state, and streams an engaged
     * [Inputs] (rider = 1). Call on touch-down and on every move while held.
     */
    fun onThrottleMove(y: Float, height: Float) {
        val engaged = Throttle.isEngaged(height)
        val speed = if (engaged) Throttle.speedFor(y, height, maxSpeed) else 0
        local.update { it.copy(throttleSpeed = speed, engaged = engaged) }
        sendInputs(throttle = speed, rider = if (engaged) RIDER_ENGAGED else RIDER_RELEASED)
    }

    /** Finger-up: immediate deadman Inputs (throttle 0, rider 0). */
    fun onThrottleRelease() = forceStop()

    private fun forceStop() {
        local.update { it.copy(throttleSpeed = 0, engaged = false) }
        sendInputs(throttle = 0, rider = RIDER_RELEASED)
    }

    private fun sendInputs(throttle: Int, rider: Int) {
        if (transport.connectionState.value == ConnectionState.CONNECTED) {
            transport.sendInputs(Inputs(throttle = throttle, buttons = 0, rider = rider))
        }
    }

    private data class LocalState(
        val throttleSpeed: Int = 0,
        val engaged: Boolean = false,
    )

    private companion object {
        const val STATE_TIMEOUT_MS = 5_000L
        const val RIDER_ENGAGED = 1
        const val RIDER_RELEASED = 0
    }
}
