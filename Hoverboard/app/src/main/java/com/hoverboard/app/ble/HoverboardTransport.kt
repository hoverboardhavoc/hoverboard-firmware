package com.hoverboard.app.ble

import com.hoverboard.app.model.ConnectionState
import kotlinx.coroutines.flow.StateFlow

/**
 * Abstraction over the BLE link to the board's onboard CC2541 module.
 *
 * For slice 1 this is connect-only: scan for the advertised name, connect, discover the
 * transparent-UART write/notify GATT pair, and surface the [ConnectionState] lifecycle. The L2/L3
 * byte path (sending/receiving protocol frames) is added in slice 2.
 *
 * Kept behind an interface so a fake can be injected in tests, and so the real
 * [BleHoverboardTransport] (Nordic Kotlin-BLE) is swapped at the Hilt module seam.
 */
interface HoverboardTransport {

    /** Current BLE link state. */
    val connectionState: StateFlow<ConnectionState>

    /** Start scanning for, and connect to, the configured peripheral. Idempotent. */
    fun connect()

    /** Disconnect and stop scanning. Safe to call when already disconnected. */
    fun disconnect()
}
