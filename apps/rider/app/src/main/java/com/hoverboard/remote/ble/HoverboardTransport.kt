package com.hoverboard.remote.ble

import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.ConfigWriteResult
import com.hoverboard.protocol.config.TuneReadResult
import com.hoverboard.protocol.config.TuneWriteResult
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.flow.StateFlow

/**
 * Abstraction over the BLE link to the board's onboard CC2541 module.
 *
 * The app is a virtual-rider node speaking OUR link frame (see [com.hoverboard.protocol.linkctl]).
 * It PRODUCES [RiderCommand]s, one per board it drives (an arm level plus a drive demand, spelled
 * as an `INPUTS` and a `DRIVE_CMD` payload) and CONSUMES [TelemetryUi] / fault frames from the
 * master. The transport seam hides which
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
     * Stream the latest [DriveFrame]: a [RiderCommand] per board. The transport conflates and
     * re-emits the held value at the link cadence, which is what keeps `DRIVE_CMD` inside each
     * board's 200 ms decay window. Arm and disarm logic, and which boards a frame names, live in the
     * ViewModel; this just transmits, addressing [Node.MASTER] as [attachedBoard] and [Node.SLAVE] as
     * [slaveBoard].
     *
     * Each board's payloads go out on the same tick, in the same order every time, so a board never
     * sees a demand from one command paired with an arm level from another.
     */
    fun sendCommand(frame: DriveFrame)

    /**
     * The L3 address of the board this session attached to (the master), or null while no session
     * is attached. It is the board the Setup screen configures.
     */
    val attachedBoard: StateFlow<Int?>

    /**
     * The L3 address of the slave this session's discovery found behind the master, or null: before
     * discovery, when it found no single other board, when the walk was abandoned
     * ([DiscoverOutcome.slave]), and after the session ends. Session-scoped and
     * never persisted (`specs/rider-ui.md` section 2). Published together with [attachedBoard], so a
     * board the app can name is a board it already knows the whole pair of.
     */
    val slaveBoard: StateFlow<Int?>

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

    /**
     * Read [key]'s STAGED value (the board's RAM gain shadow, `TUNE_READ`) from the board at
     * [target] over the session's tune client ([com.hoverboard.protocol.config.TuneClient]). Null
     * when no session is attached, in which case nothing was sent.
     */
    suspend fun readTune(key: Key, target: Int): TuneReadResult?

    /**
     * Stage [value] for [key] on the board at [target] (`TUNE_WRITE`: RAM only, allowed armed) and
     * verify the staged value the board echoes. The running loop ramps toward it; nothing is
     * persisted. Null when no session is attached, in which case nothing was sent.
     */
    suspend fun writeTune(key: Key, value: Int, target: Int): TuneWriteResult?
}
