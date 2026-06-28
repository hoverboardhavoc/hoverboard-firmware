package com.hoverboard.app.ble

import com.hoverboard.app.model.ConnectionState
import com.hoverboard.app.net.l3.WalkOutcome
import kotlinx.coroutines.flow.StateFlow

/**
 * Abstraction over the BLE link to the board's onboard CC2541 module.
 *
 * Slice 1 is connect-only: scan for the advertised name, connect, discover the transparent-UART
 * write/notify GATT pair, and surface the [ConnectionState] lifecycle. Slice 4 adds [discover]: once
 * connected, it runs the controller-side L3 walk over the live BLE byte stream and reports the fleet.
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

    /**
     * Run the controller-side walk over the connected BLE byte stream and return the discovered fleet,
     * or null if not currently [ConnectionState.CONNECTED]. Suspends until the walk quiesces.
     */
    suspend fun discover(): WalkOutcome?
}
