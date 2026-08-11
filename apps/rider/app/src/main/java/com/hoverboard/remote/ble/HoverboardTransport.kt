package com.hoverboard.remote.ble

import com.hoverboard.remote.link.Inputs
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.StateFlow

/**
 * Abstraction over the BLE link to the board's onboard CC2541 module.
 *
 * The app is a virtual-rider node speaking OUR link frame (see [com.hoverboard.remote.link]).
 * It PRODUCES [Inputs] frames (throttle + rider/deadman bit) and CONSUMES [TelemetryUi] /
 * fault frames. The transport seam hides which radio/GATT carries the bytes, NOT which wire
 * protocol: every impl speaks the same link frame.
 *
 * Kept behind an interface so a fake can be injected in tests:
 *  - [BleHoverboardTransport] wraps Nordic Kotlin-BLE for the real app.
 *  - A fake implements this directly in unit tests, with no Android BLE stack.
 *
 * All sends are fire-and-forget (Write Without Response). Telemetry arrives as a hot
 * [StateFlow]; [connectionState] tracks the link lifecycle.
 */
interface HoverboardTransport {

    /** Current BLE link state. */
    val connectionState: StateFlow<ConnectionState>

    /** Latest merged telemetry, or null until the first valid Telemetry frame arrives. */
    val telemetry: StateFlow<TelemetryUi?>

    /** Start scanning for, and connect to, the configured peripheral. Idempotent. */
    fun connect()

    /** Disconnect and stop scanning. Safe to call when already disconnected. */
    fun disconnect()

    /**
     * Stream the latest rider [Inputs] (throttle/buttons/rider). The transport conflates and
     * re-emits the held value at the link cadence to feed the board's no-command timeout. No-op
     * if not [ConnectionState.CONNECTED]. Deadman/finger-up logic lives in the ViewModel; this
     * just transmits.
     */
    fun sendInputs(inputs: Inputs)
}
