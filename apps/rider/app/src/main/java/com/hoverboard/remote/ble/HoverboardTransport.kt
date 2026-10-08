package com.hoverboard.remote.ble

import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.ConfigWriteResult
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.StateFlow

/**
 * Abstraction over the BLE link to the board's onboard CC2541 module.
 *
 * The app is a virtual-rider node speaking OUR link frame (see [com.hoverboard.protocol.linkctl]).
 * It PRODUCES [RiderCommand]s (an arm level plus a drive demand, spelled as an `INPUTS` and a
 * `DRIVE_CMD` payload) and CONSUMES [TelemetryUi] / fault frames. The transport seam hides which
 * radio/GATT carries the bytes, NOT which wire protocol: every impl speaks the same link frame.
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
     * Stream the latest [RiderCommand]. The transport conflates and re-emits the held value at the
     * link cadence, which is what keeps `DRIVE_CMD` inside the firmware's 200 ms decay window. Arm
     * and disarm logic lives in the ViewModel; this just transmits.
     *
     * Both payloads go out on the same tick, in the same order every time, so the board never sees
     * a demand from one command paired with an arm level from another.
     */
    fun sendCommand(command: RiderCommand)

    /**
     * The L3 address of the board this session attached to, or null while no session is attached.
     * It is the only board the app can name as a config target today: the transport attaches to
     * one board and does not run discovery, so the slave's address is not known to it.
     */
    val attachedBoard: StateFlow<Int?>

    /**
     * Read [key] from the board at [target] over the session's config client
     * ([com.hoverboard.protocol.config.ConfigClient]). Null when no session is attached, in which
     * case nothing was sent.
     */
    suspend fun readConfig(key: Key, target: Int): ConfigReadResult?

    /**
     * Write [value] to [key] on the board at [target] and verify the stored value the board echoes.
     * A verified write is a statement about the board's store, never about a running loop. Null when
     * no session is attached, in which case nothing was sent.
     */
    suspend fun writeConfig(key: Key, value: Value, target: Int): ConfigWriteResult?
}
