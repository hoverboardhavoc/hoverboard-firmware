package com.hoverboard.remote.ble

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.Fault
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_CYCLIC_STATE
import com.hoverboard.protocol.linkctl.OP_FAULT
import com.hoverboard.protocol.linkctl.OP_INPUTS
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.withTimeoutOrNull
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.firstOrNull
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import no.nordicsemi.android.kotlin.ble.core.data.GattConnectionState
import no.nordicsemi.android.kotlin.ble.client.main.callback.ClientBleGatt
import no.nordicsemi.android.kotlin.ble.client.main.service.ClientBleGattCharacteristic
import no.nordicsemi.android.kotlin.ble.client.main.service.ClientBleGattService
import no.nordicsemi.android.kotlin.ble.client.main.service.ClientBleGattServices
import no.nordicsemi.android.kotlin.ble.core.data.BleGattConnectOptions
import no.nordicsemi.android.kotlin.ble.core.data.BleGattProperty
import no.nordicsemi.android.kotlin.ble.core.data.BleWriteType
import no.nordicsemi.android.kotlin.ble.core.data.util.DataByteArray
import no.nordicsemi.android.kotlin.ble.core.ServerDevice
import no.nordicsemi.android.kotlin.ble.core.scanner.BleScanResult
import no.nordicsemi.android.kotlin.ble.scanner.BleScanner

/**
 * BLE transport for the onboard CC2541 module on the master hoverboard board.
 *
 * The app is a virtual-rider node speaking the firmware's current L2/L3 stack
 * ([com.hoverboard.protocol]) over the module's transparent BLE<->UART bridge.
 *
 * The stack is two layers, not the single flat frame this app used to build:
 *  - L2 ([Link] over [BleStreamTransport]) owns SOF/len/CRC framing and fragmentation.
 *  - L3 ([Pdu]) owns `[opcode][src][dst][payload]`.
 * A CYCLIC_STATE PDU is 14 bytes, so it rides one fragment in a 19-byte stream frame, inside a
 * single 20-byte ATT notification (pinned by the module's WireDriftTest).
 *
 * Connection flow:
 *  1. Scan for the board named by [LinkSettings.deviceName] (set by the firmware AT+NAME),
 *     preferring the address of the board that name last connected to (see [DeviceSelection]).
 *  2. Connect and discover all services.
 *  3. Find the write-and-notify characteristic by properties, walking every service for the
 *     first characteristic with both WRITE_WITHOUT_RESPONSE (or WRITE) and NOTIFY. The module
 *     is dumb; we don't trust the UUID.
 *  4. Subscribe to notifications, feed each chunk into the L2 transport (one notification is
 *     NOT one frame: the module splits and coalesces).
 *  5. **Attach** ([L3Session.attach]): run the L3 first contact so the board holds an address and the app
 *     holds the guest address the board granted it. Nothing is drivable and no telemetry exists
 *     before this.
 *  6. Outgoing [Inputs] are staged as an INPUTS PDU by [CommandPump] and flushed by the session
 *     loop, which is the link's single writer; incoming packets are dispatched by L3 opcode.
 *
 * The control/safety logic (deadman, finger-up -> 0) lives in the ViewModel.
 *
 * ## The app is a transient controller
 *
 * The firmware emits `CYCLIC_STATE` only while the board holds an assigned L3 address
 * (`crates/orchestrator/src/dispatch.rs`, `ble_cyclic_tx`), and an address arrives exactly one way:
 * a controller performs first contact. So a rider remote that skipped the attach connected, showed
 * its control screen, and waited for telemetry that could never come, while commands appeared to
 * work only because a broadcast `dst` is delivered locally.
 *
 * This app therefore performs the attach itself, as a transient controller for the duration of one
 * connection (`specs/l3.md`, "Addressing"): [BleWalkEngine] sends `NODE_HELLO`, the app adopts the
 * granted `your_addr` as its `src`, and if the board reports `node_id = 0x00` the app assigns it an
 * address. A board that already reports an identity keeps it. The guest address is session-scoped:
 * never persisted, and dropped with the engine on disconnect, so a reconnect attaches cleanly.
 *
 * It stops after the attach leg, and does not walk the tree ([BleWalkEngine.attachOnly]). A rider
 * has one point-to-point BLE link to one board, drives that board and reads its telemetry; the
 * `PROBE_PORTS` half of the walk answers "what is connected to it", which the rider never asks. It
 * would also cost every connect the board's ~500 ms probe window and a multi-frame `PORTS` reply on
 * the same 9600-baud metered port that must then carry telemetry, and it would hand addresses to
 * sideboards from a map the rider does not persist. Mapping and provisioning the fleet is the
 * Hoverboard controller app's job.
 */
class BleHoverboardTransport(
    private val context: Context,
    private val settings: LinkSettings,
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob()),
) : HoverboardTransport {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _telemetry = MutableStateFlow<TelemetryUi?>(null)
    override val telemetry: StateFlow<TelemetryUi?> = _telemetry.asStateFlow()

    private var client: ClientBleGatt? = null
    private var writeCharacteristic: ClientBleGattCharacteristic? = null
    private var notifyCharacteristic: ClientBleGattCharacteristic? = null
    private var sessionJob: Job? = null

    /**
     * The connected session's engine-turning loop, held so EVERY exit path stops it. It is launched
     * in the transport-lifetime [scope], so a session that ends by cancellation (a [disconnect]
     * landing on the park below) or by a rethrown error would otherwise leave it running: an orphan
     * that keeps pumping a dead session's engine and swallowing writes to a dead characteristic,
     * one more per disconnect/reconnect cycle. [tearDownSession] runs on every one of those paths.
     */
    private var serviceJob: Job? = null

    /**
     * The current session's notification collector, held for the same reason as [serviceJob]: it is
     * session-scoped work launched in the transport-lifetime [scope], so nothing but an explicit
     * cancel stops it feeding a torn-down session's engine.
     */
    private var notifyJob: Job? = null
    private var pump: CommandPump? = null

    /**
     * The L3/L2 stack for the current session: one [BleWalkEngine] owning one L2 [Link] over the
     * BLE byte stream. Rebuilt by [resetLink] on every (re)connect, which is also what drops the
     * session's guest address: a new engine starts with a fresh [com.hoverboard.protocol.l3.Controller],
     * an empty framer, no half-reassembled packet and a fresh fragment id.
     *
     * One engine, not an engine plus a private link: a second [Link] over the same byte stream would
     * split reassembly across two objects and restart the fragment id mid-session.
     *
     * Guarded by [linkLock]: notifications arrive on one coroutine while [CommandPump] stages sends
     * on another and the session loop pumps on a third, and the engine's queues and the link's
     * fragment counter are plain mutable state.
     */
    private val linkLock = Any()
    private var engine: BleWalkEngine? = null

    /**
     * The L3 identity this connection attached with, or null until [L3Session.attach] succeeds.
     * Session-scoped by construction (a plain field cleared on teardown, never written to
     * [LinkSettings]).
     */
    private var attachment: Attachment? = null

    /**
     * Did the user ask to stay connected? Set true by [connect], false by [disconnect].
     * Auto-reconnect retries while this is true; a deliberate disconnect clears it so
     * we don't fight the user.
     */
    private var keepConnected: Boolean = false

    @SuppressLint("MissingPermission")
    override fun connect() {
        Log.d(TAG, "connect() state=${_connectionState.value}")
        if (sessionJob?.isActive == true) {
            Log.d(TAG, "connect() ignored: session already running")
            return
        }
        keepConnected = true
        sessionJob = scope.launch { runWithReconnect() }
    }

    private suspend fun runWithReconnect() {
        var attempt = 0
        while (keepConnected && currentCoroutineContext().isActive) {
            if (attempt > 0) {
                val backoff = (RECONNECT_DELAY_MS * attempt).coerceAtMost(RECONNECT_DELAY_MAX_MS)
                Log.d(TAG, "reconnect attempt $attempt in ${backoff}ms")
                delay(backoff)
                if (!keepConnected) break
            }
            attempt++
            try {
                runSession()
            } catch (e: TimeoutCancellationException) {
                // A hung connect/discover times out here (withTimeout). It is a CancellationException
                // subtype, so it MUST be caught before the CancellationException branch, and must NOT
                // be rethrown: we want the reconnect loop to retry, not die.
                Log.w(TAG, "connect/discover timed out (attempt $attempt), retrying")
            } catch (e: CancellationException) {
                throw e
            } catch (e: Throwable) {
                Log.w(TAG, "session error (attempt $attempt): ${e.message}")
            } finally {
                tearDownSession()
            }
            if (!keepConnected) break
            // Successful connect should reset attempt counter, but it doesn't matter
            // much; the backoff caps quickly.
        }
        // A failed attach ends the loop with a diagnosis on screen; do not overwrite it with the
        // generic idle state, which would render as "nothing happened".
        if (_connectionState.value != ConnectionState.ATTACH_FAILED) {
            _connectionState.value = ConnectionState.DISCONNECTED
        }
    }

    @SuppressLint("MissingPermission")
    private suspend fun runSession() {
        // Read the target fresh every session: the name is user-settable and persisted, so a change
        // takes effect on the next scan without restarting the app.
        val targetName = settings.deviceName.value
        val preferredAddress = settings.preferredAddress()
        Log.d(TAG, "runSession start, name=$targetName preferAddress=$preferredAddress")
        _connectionState.value = ConnectionState.SCANNING
        val device = try {
            findDevice(targetName, preferredAddress)
        } catch (e: SecurityException) {
            Log.w(TAG, "scan SecurityException", e)
            failed(e)
            return
        } catch (e: Throwable) {
            Log.w(TAG, "scan failed", e)
            failed(e)
            return
        }
        Log.d(TAG, "scan returned device=$device")
        if (device == null) {
            _connectionState.value = ConnectionState.ERROR
            return
        }

        _connectionState.value = ConnectionState.CONNECTING
        // Let the BLE scan radio settle before a direct connect. Connecting while a scan is still
        // winding down is a common cause of a connect that never gets a callback on Android.
        delay(SCAN_SETTLE_MS)
        try {
            // Bound the connect and the service discovery: the Nordic connect() can suspend forever if
            // the link never establishes (no callback), which strands us in CONNECTING. A timeout
            // surfaces it as TimeoutCancellationException so the reconnect loop retries.
            // autoConnect = true uses Android's queued/opportunistic connect, which succeeds on BLE
            // stacks that fail a direct (autoConnect = false) connect to cheap modules with GATT 133.
            // It can be slower, hence the longer CONNECT_TIMEOUT_MS.
            val gatt = withTimeout(CONNECT_TIMEOUT_MS) {
                ClientBleGatt.connect(
                    context,
                    device,
                    scope,
                    options = BleGattConnectOptions(autoConnect = true),
                )
            }
            client = gatt
            val services = withTimeout(DISCOVER_TIMEOUT_MS) { gatt.discoverServices() }

            // Read User Description (0x2901) for each writable char in custom services
            // so we can see which one the module labels as the UART TX path.
            for (service in services.services) {
                if (service.uuid.isSigStandardMetadata()) continue
                for (ch in service.characteristics) {
                    if (!ch.hasWrite()) continue
                    val ud = ch.descriptors.firstOrNull { it.uuid == USER_DESC_UUID }
                    if (ud != null) {
                        try {
                            val bytes = ud.read().value
                            val text = bytes.toString(Charsets.UTF_8).trimEnd('\u0000', ' ')
                            Log.d(TAG, "char ${ch.uuid} user-description='$text'")
                        } catch (e: Throwable) {
                            Log.d(TAG, "char ${ch.uuid} user-description read failed: ${e.message}")
                        }
                    }
                }
            }

            val (writeChar, notifyChar) = pickIoCharacteristics(services)
            if (writeChar == null || notifyChar == null) {
                Log.w(
                    TAG,
                    "GATT does not expose required write/notify pair: write=$writeChar notify=$notifyChar",
                )
                _connectionState.value = ConnectionState.ERROR
                gatt.disconnect()
                return
            }
            Log.d(
                TAG,
                "picked write=${writeChar.uuid} notify=${notifyChar.uuid}",
            )
            writeCharacteristic = writeChar
            notifyCharacteristic = notifyChar
            val engine = resetLink()
            _telemetry.value = null

            notifyJob = notifyChar.getNotifications()
                .onEach { data -> onNotificationBytes(data.value) }
                .launchIn(scope)

            val writeType = writeChar.preferredWriteType()
            val session = L3Session(
                engine = engine,
                lock = linkLock,
                nowMs = System::currentTimeMillis,
                onPacket = ::dispatchPacket,
                write = { bytes -> writeChunked(writeChar, writeType, bytes) },
            )

            // First contact, before anything is drivable and before any telemetry can exist.
            _connectionState.value = ConnectionState.ATTACHING
            val outcome = session.attach()
            val attached = (outcome as? AttachOutcome.Attached)?.attachment
            if (attached == null) {
                // The link is alive but the board would not take part. Say so and stop: the
                // retransmits already absorbed a dropped frame, so retrying the whole connect would
                // just churn silently, which is the failure mode this whole path exists to remove.
                Log.w(TAG, "L3 attach failed ($outcome); not driving")
                keepConnected = false
                _connectionState.value = ConnectionState.ATTACH_FAILED
                gatt.disconnect()
                return
            }
            attachment = attached
            Log.d(
                TAG,
                "attached: src=0x${Integer.toHexString(attached.guestAddr)} " +
                    "board=0x${Integer.toHexString(attached.boardAddr)}",
            )

            pump = CommandPump(scope, SEND_INTERVAL_MS) { inputs ->
                // Stage one INPUTS PDU per tick; the session loop is the link's single writer and
                // puts it on the wire. The payload is 4 bytes, so 7 as a PDU and 11 on the wire:
                // one fragment, one ATT write. Pump swallows ordinary exceptions and retries, so
                // don't rethrow here; that would kill the coroutine on the first hiccup.
                stageInputs(engine, attached, inputs)
            }.also { it.start() }

            // The session loop: pump the engine (reassemble, answer a probe of our own port),
            // dispatch telemetry, and flush every outgoing byte. Sole writer of the GATT char.
            serviceJob = scope.launch { serviceLoop(session) }

            _connectionState.value = ConnectionState.CONNECTED
            // Remember the board we actually reached, not the one we decided to try: this address is
            // only worth preferring next time because it is known to have worked.
            settings.rememberAddress(device.address)

            // Block here until the underlying GATT link drops. The reconnect loop
            // sees this function return and schedules a fresh session if the user
            // still wants to be connected.
            gatt.connectionState.first { it == GattConnectionState.STATE_DISCONNECTED }
            Log.d(TAG, "GATT link dropped")
        } catch (e: SecurityException) {
            failed(e)
            throw e
        } catch (e: IllegalStateException) {
            failed(e)
            throw e
        }
    }

    /**
     * Scan until a board worth connecting to shows up.
     *
     * With nothing remembered this is exactly the original behaviour: take the FIRST advertisement
     * whose name matches and connect immediately, no added latency.
     *
     * With an address remembered, the scan is given a short grace window to let that specific board
     * appear, because the whole point of remembering it is to pick it over another board answering
     * to the same name, and the right one is not guaranteed to advertise first. Any name match seen
     * during the window is kept as the fallback, so the window costs a delay only when the preferred
     * board is genuinely absent, and never turns a findable board into a failure.
     */
    @SuppressLint("MissingPermission")
    private suspend fun findDevice(targetName: String, preferredAddress: String?): ServerDevice? {
        val scan = BleScanner(context).scan()
        if (preferredAddress == null) {
            return scan.firstOrNull { DeviceSelection.matchesName(it.toCandidate(), targetName) }
                ?.device
        }
        var nameMatch: ServerDevice? = null
        val exact = withTimeoutOrNull(ADDRESS_PREFERENCE_WINDOW_MS) {
            scan.firstOrNull { result ->
                val candidate = result.toCandidate()
                if (nameMatch == null && DeviceSelection.matchesName(candidate, targetName)) {
                    nameMatch = result.device
                }
                DeviceSelection.matchesAddress(candidate, preferredAddress)
            }?.device
        }
        if (exact == null && nameMatch != null) {
            Log.d(TAG, "preferred address $preferredAddress not seen; falling back to name match")
        }
        return exact ?: nameMatch
    }

    private fun tearDownSession() {
        pump?.stop()
        pump = null
        // Stop turning the engine before it is dropped. This runs on every session exit (the
        // reconnect loop's finally, and disconnect()), so the loop cannot outlive its session
        // whether the session ended normally, by cancellation, or by a rethrown error.
        serviceJob?.cancel()
        serviceJob = null
        notifyJob?.cancel()
        notifyJob = null
        try {
            client?.disconnect()
        } catch (_: Throwable) {
            // already-dead links throw; we just want to drop the handle
        }
        client = null
        writeCharacteristic = null
        notifyCharacteristic = null
        // Drop the engine and with it the session's guest address: it is never persisted and never
        // outlives the connection, so a reconnect attaches from scratch (specs/l3.md: a controller
        // is a guest that releases its address on disconnect).
        synchronized(linkLock) {
            engine = null
            attachment = null
        }
        _telemetry.value = null
        if (keepConnected) {
            _connectionState.value = ConnectionState.SCANNING
        }
    }

    /**
     * Rebuild the session's L3/L2 stack and return it, dropping any partial frame, half-reassembled
     * packet, and the previous session's guest address (a fresh engine means a fresh controller).
     */
    private fun resetLink(): BleWalkEngine = synchronized(linkLock) {
        val fresh = BleWalkEngine(
            attachOnly = true,
            replyTimeoutMs = L3Session.REPLY_TIMEOUT_MS,
            nowMs = System::currentTimeMillis,
        )
        engine = fresh
        attachment = null
        fresh
    }

    /**
     * The connected session's service loop. The engine is synchronous, so something has to turn it:
     * [L3Session.turn] drains reassembled packets, answers a probe of the app's own port (a fleet
     * controller walking the tree reaches the rider through the board), dispatches telemetry, and
     * writes every outgoing byte. It is the link's SINGLE writer, which is what keeps GATT to one
     * operation in flight now that [CommandPump] stages rather than writes.
     */
    private suspend fun serviceLoop(session: L3Session) {
        while (currentCoroutineContext().isActive) {
            try {
                session.turn()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Throwable) {
                // A write failing means the GATT link is gone; runSession is already parked on the
                // connection state and will tear the session down. Nothing useful to do here.
                Log.d(TAG, "service loop write failed: ${e.message}")
            }
            delay(L3Session.POLL_IDLE_MS)
        }
    }

    /** Write one outgoing byte run, split to what a single ATT write carries. */
    private suspend fun writeChunked(
        writeChar: ClientBleGattCharacteristic,
        writeType: BleWriteType,
        out: ByteArray,
    ) {
        var i = 0
        while (i < out.size) {
            val end = minOf(i + ATT_CHUNK, out.size)
            writeChar.write(DataByteArray(out.copyOfRange(i, end)), writeType)
            i = end
        }
    }

    /** Stage one INPUTS PDU on the session link, addressed by what first contact settled. */
    private fun stageInputs(engine: BleWalkEngine, at: Attachment, inputs: Inputs) =
        synchronized(linkLock) {
            engine.sendPacket(Pdu(OP_INPUTS, at.guestAddr, at.boardAddr, inputs.encode()).encode())
        }

    /**
     * Feed a notification's bytes into L2 (split/coalesce safe). Reassembly and dispatch happen on
     * the service loop's next turn, so the whole engine is touched from one place.
     */
    private fun onNotificationBytes(bytes: ByteArray) = synchronized(linkLock) {
        engine?.onReceive(bytes)
        Unit
    }

    /**
     * Dispatch a packet the walk engine did not consume, by L3 opcode:
     *  - [OP_CYCLIC_STATE] -> the board's state, which is what feeds the telemetry pane,
     *  - [OP_FAULT] -> a latch-edge stop/fault notification,
     *  - anything else ignored.
     *
     * CYCLIC_STATE is the telemetry source because the firmware has no telemetry opcode at all.
     * The 0x40..0x6F telemetry block is reserved and unimplemented (`crates/net/src/pdu.rs:41`);
     * this app used to decode a TELEMETRY 0x20 that nothing ever sent.
     */
    private fun dispatchPacket(packet: ByteArray) {
        val pdu = Pdu.decodeOrNull(packet) ?: return
        when (pdu.opcode) {
            OP_CYCLIC_STATE -> CyclicState.decode(pdu.payload)?.let { state ->
                _telemetry.value = (_telemetry.value ?: TelemetryUi()).merge(state)
            }
            OP_FAULT -> Fault.decode(pdu.payload)?.let { f ->
                _telemetry.value = (_telemetry.value ?: TelemetryUi()).copy(
                    faultStop = f.stopAll(),
                    faultCode = f.code,
                )
            }
            else -> Unit // not this app's business
        }
    }

    private fun failed(@Suppress("UNUSED_PARAMETER") cause: Throwable) {
        _connectionState.value = ConnectionState.ERROR
    }

    override fun disconnect() {
        keepConnected = false
        sessionJob?.cancel()
        sessionJob = null
        tearDownSession()
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    override fun sendInputs(inputs: Inputs) {
        pump?.set(inputs)
    }

    /** Cancel the transport's coroutine scope. Call when the owning component is destroyed. */
    fun shutdown() {
        disconnect()
        scope.cancel()
    }

    private companion object {

        const val TAG = "PalTransport"

        /**
         * Stream cadence. Firmware's `LOST_CONNECTION_STOP_MILLIS` is 500 ms.
         * - 30 Hz overran the CC2541's BLE→UART buffer (drops + heat).
         * - 5 Hz had 3-drops-in-a-row failure modes around RF dropouts.
         * - 10 Hz is the sweet spot: matches telemetry cadence (which is reliable),
         *   tolerates 4 consecutive drops before tripping the 500 ms timeout.
         */
        const val SEND_INTERVAL_MS = 100L

        /** First reconnect attempt fires after this many ms; subsequent attempts back off. */
        const val RECONNECT_DELAY_MS = 800L
        const val RECONNECT_DELAY_MAX_MS = 5000L

        /** Pause after the scan stops, before a direct connect, so the radio is free. */
        const val SCAN_SETTLE_MS = 600L

        /**
         * How long a scan waits for a REMEMBERED address before settling for a name match. Only
         * applies when an address is remembered; with none, the first name match is taken at once.
         * Long enough to cover a few advertising intervals, short enough that a board that has been
         * swapped out does not stall every reconnect attempt.
         */
        const val ADDRESS_PREFERENCE_WINDOW_MS = 4_000L

        /** Bound on ClientBleGatt.connect() so a never-establishing link retries instead of hanging. */
        const val CONNECT_TIMEOUT_MS = 30_000L

        /** Bound on service discovery (a connected-but-silent GATT also strands CONNECTING). */
        const val DISCOVER_TIMEOUT_MS = 10_000L
        const val LOG_EVERY_WRITES = 30

        /**
         * Bytes per GATT write. The engine hands back a byte stream, which can hold more than one
         * frame if a probe reply and an INPUTS PDU come due together, so it is split rather than
         * written whole and left to trip the ATT MTU.
         */
        const val ATT_CHUNK = 20

        /**
         * Pick the BLE characteristics that carry the transparent UART. Some CC2541-class
         * modules expose **one** characteristic with both write+notify; others split it
         * across **two** (one for writes, one for notifications), and only the notify one
         * gets a standard CCCD descriptor (UUID 0x2902). We walk all services and:
         *
         *  - pick the **write** characteristic as the first one with WRITE_WITHOUT_RESPONSE
         *    (preferred for throughput) or WRITE, and
         *  - pick the **notify** characteristic as the first one with NOTIFY *and* a CCCD
         *    descriptor (so Nordic's `getNotifications()` can write CCCD without throwing
         *    NotificationDescriptorNotFoundException).
         *
         * Same characteristic may fulfil both roles when the module exposes a single pipe.
         */
        fun pickIoCharacteristics(
            services: ClientBleGattServices,
        ): Pair<ClientBleGattCharacteristic?, ClientBleGattCharacteristic?> {
            // Strategy: in a non-SIG service, prefer a characteristic that supports
            // WRITE_WITHOUT_RESPONSE (typical transparent UART pattern). Skip dedicated
            // write-only chars on this revisit. Empirically, on the OEM Offroad
            // CC2541 firmware, only the WRITE_NO_RESPONSE swiss-army char actually
            // forwards bytes to UART; dedicated WRITE-only chars route to the module's
            // own command parser, not the UART.
            for (service in services.services) {
                if (service.uuid.isSigStandardMetadata()) continue
                val notify = service.characteristics.firstOrNull { it.hasNotifyWithCccd() }
                    ?: continue
                val noRespWrite = service.characteristics.firstOrNull {
                    BleGattProperty.PROPERTY_WRITE_NO_RESPONSE in it.properties
                }
                if (noRespWrite != null) return noRespWrite to notify
                val anyWrite = service.characteristics.firstOrNull { it.hasWrite() }
                if (anyWrite != null) return anyWrite to notify
            }
            // Cross-service fallback.
            var write: ClientBleGattCharacteristic? = null
            var notify: ClientBleGattCharacteristic? = null
            for (service in services.services) {
                if (service.uuid.isSigStandardMetadata()) continue
                for (ch in service.characteristics) {
                    if (write == null && ch.hasWrite()) write = ch
                    if (notify == null && ch.hasNotifyWithCccd()) notify = ch
                    if (write != null && notify != null) return write to notify
                }
            }
            return write to notify
        }

        /**
         * Reduce a scan result to the fields selection cares about. Both names are carried: the
         * scan-record local name is what the board is advertising now, the device name is what
         * Android has cached against the MAC and can be stale after a rename.
         */
        fun BleScanResult.toCandidate(): ScanCandidate = ScanCandidate(
            address = device.address,
            advertisedName = data?.scanRecord?.deviceName,
            cachedName = device.name,
        )

        fun java.util.UUID.isSigStandardMetadata(): Boolean {
            // 0x1800 Generic Access, 0x1801 Generic Attribute, 0x180A Device Information
            val short = (mostSignificantBits ushr 32) and 0xFFFFL
            return short == 0x1800L || short == 0x1801L || short == 0x180AL
        }

        fun ClientBleGattCharacteristic.hasWrite(): Boolean =
            BleGattProperty.PROPERTY_WRITE_NO_RESPONSE in properties ||
                BleGattProperty.PROPERTY_WRITE in properties

        fun ClientBleGattCharacteristic.isDedicatedWrite(): Boolean {
            if (!hasWrite()) return false
            return BleGattProperty.PROPERTY_NOTIFY !in properties &&
                BleGattProperty.PROPERTY_INDICATE !in properties &&
                BleGattProperty.PROPERTY_READ !in properties
        }

        fun ClientBleGattCharacteristic.hasNotifyWithCccd(): Boolean {
            if (BleGattProperty.PROPERTY_NOTIFY !in properties) return false
            return descriptors.any { it.uuid == CCCD_UUID }
        }

        fun ClientBleGattCharacteristic.preferredWriteType(): BleWriteType =
            if (BleGattProperty.PROPERTY_WRITE_NO_RESPONSE in properties) {
                BleWriteType.NO_RESPONSE
            } else {
                BleWriteType.DEFAULT
            }

        val CCCD_UUID: java.util.UUID =
            java.util.UUID.fromString("00002902-0000-1000-8000-00805F9B34FB")

        val USER_DESC_UUID: java.util.UUID =
            java.util.UUID.fromString("00002901-0000-1000-8000-00805F9B34FB")

        val AT_MODE_DATA: ByteArray = "AT+MODE=DATA\r\n".toByteArray(Charsets.US_ASCII)
    }
}
