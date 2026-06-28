package com.hoverboard.app.ble

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import com.hoverboard.app.model.ConnectionState
import com.hoverboard.app.net.l3.BleBytePipe
import com.hoverboard.app.net.l3.BlePipeSource
import com.hoverboard.app.net.l3.BleWalkDriver
import com.hoverboard.app.net.l3.WalkOutcome
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.cancel
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.firstOrNull
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.withTimeoutOrNull
import no.nordicsemi.android.kotlin.ble.client.main.callback.ClientBleGatt
import no.nordicsemi.android.kotlin.ble.client.main.service.ClientBleGattCharacteristic
import no.nordicsemi.android.kotlin.ble.client.main.service.ClientBleGattServices
import no.nordicsemi.android.kotlin.ble.core.data.BleGattConnectOptions
import no.nordicsemi.android.kotlin.ble.core.data.BleGattConnectionPriority
import no.nordicsemi.android.kotlin.ble.core.data.BleGattProperty
import no.nordicsemi.android.kotlin.ble.core.data.BleWriteType
import no.nordicsemi.android.kotlin.ble.core.data.GattConnectionState
import no.nordicsemi.android.kotlin.ble.core.data.util.DataByteArray
import no.nordicsemi.android.kotlin.ble.scanner.BleScanner

/**
 * Connect-only BLE transport for the onboard CC2541 module on the master board (slice 1).
 *
 * Flow:
 *  1. Scan for the advertised name from [LinkConfig.deviceName] (set by the firmware AT+NAME).
 *  2. Connect (autoConnect = true — Android's queued/opportunistic connect, which lands on cheap
 *     modules that fail a direct connect with GATT 133) and discover all services.
 *  3. Find the transparent-UART write/notify pair — runtime discovery (the module is dumb, so the
 *     UUID is not trusted): walk every non-SIG service for a NOTIFY-with-CCCD characteristic and a
 *     WRITE_WITHOUT_RESPONSE (or WRITE) characteristic. The bench module exposes 0x1001 / 0x1002.
 *  4. Subscribe to notifications (validates the CCCD path) and report [ConnectionState.CONNECTED].
 *
 * The L2/L3 byte path (encoding frames out the write char, decoding the notify stream) is slice 2;
 * here the notify bytes are only counted, to prove the pipe is live.
 */
class BleHoverboardTransport(
    private val context: Context,
    private val config: LinkConfig = LinkConfig(),
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob()),
) : HoverboardTransport {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private var client: ClientBleGatt? = null
    private var sessionJob: Job? = null

    /**
     * The live byte pipe over the connected write/notify chars; non-null only while CONNECTED. A
     * `StateFlow` so the walk driver can *await* the next live pipe across a reconnect (the session loop
     * sets it on CONNECTED and clears it on teardown).
     */
    private val pipeState = MutableStateFlow<BleBytePipe?>(null)

    /** Did the user ask to stay connected? Auto-reconnect retries while true. */
    private var keepConnected: Boolean = false

    @SuppressLint("MissingPermission")
    override fun connect() {
        Log.d(TAG, "connect() state=${_connectionState.value}")
        if (sessionJob?.isActive == true) {
            Log.d(TAG, "connect() ignored — session already running")
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
                // A hung connect/discover times out here. It is a CancellationException subtype, so
                // it MUST be caught before the CancellationException branch and must NOT be rethrown:
                // the reconnect loop should retry, not die.
                Log.w(TAG, "connect/discover timed out (attempt $attempt), retrying", e)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Throwable) {
                Log.w(TAG, "session error (attempt $attempt): ${e.message}", e)
            } finally {
                tearDownSession()
            }
            if (!keepConnected) break
        }
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    @SuppressLint("MissingPermission")
    private suspend fun runSession() {
        Log.d(TAG, "runSession start, looking for name=${config.deviceName}")
        _connectionState.value = ConnectionState.SCANNING
        val device = try {
            BleScanner(context).scan()
                .firstOrNull { result ->
                    // The CC2541 vendor firmware pads the AT+NAME slot with trailing whitespace, so
                    // the advertised local name comes through as "Pal                " — trim before
                    // equality, both for the scan-record name and the cached device name.
                    val advName = result.data?.scanRecord?.deviceName?.trim()
                    val devName = result.device.name?.trim()
                    // Bench diagnostic: log every named advert seen so the actual module name is
                    // visible in logcat when the scan does not match the configured deviceName.
                    if (advName != null || devName != null) {
                        Log.d(TAG, "scan saw adv='$advName' dev='$devName' addr=${result.device.address}")
                    }
                    advName == config.deviceName || devName == config.deviceName
                }
                ?.device
        } catch (e: SecurityException) {
            Log.w(TAG, "scan SecurityException", e)
            _connectionState.value = ConnectionState.ERROR
            return
        } catch (e: Throwable) {
            Log.w(TAG, "scan failed", e)
            _connectionState.value = ConnectionState.ERROR
            return
        }
        Log.d(TAG, "scan returned device=$device")
        if (device == null) {
            _connectionState.value = ConnectionState.ERROR
            return
        }

        _connectionState.value = ConnectionState.CONNECTING
        // Let the scan radio settle before a direct connect: connecting while a scan is still
        // winding down is a common cause of a connect that never gets a callback on Android.
        delay(SCAN_SETTLE_MS)

        // Bound the connect + discovery: Nordic connect() can suspend forever if the link never
        // establishes (no callback), stranding us in CONNECTING; a timeout surfaces it as
        // TimeoutCancellationException so the reconnect loop retries.
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

        val (writeChar, notifyChar) = pickIoCharacteristics(services)
        if (writeChar == null || notifyChar == null) {
            Log.w(TAG, "GATT lacks a write/notify pair — write=$writeChar notify=$notifyChar")
            _connectionState.value = ConnectionState.ERROR
            gatt.disconnect()
            return
        }
        Log.d(TAG, "picked write=${writeChar.uuid} notify=${notifyChar.uuid}")

        // Subscribe so the CCCD path is exercised before CONNECTED (Nordic enables the CCCD on the
        // first getNotifications()); the byte pipe below re-collects the same notify flow for the walk.
        val notifications = notifyChar.getNotifications()
        var notifyCount = 0
        notifications
            .onEach {
                notifyCount++
                if (notifyCount % LOG_EVERY_NOTIFY == 1) {
                    Log.d(TAG, "notify #$notifyCount (${it.value.size} B)")
                }
            }
            .launchIn(scope)

        // The L2/L3 byte path (slice 4): the controller walk rides this pipe. Write Without Response
        // (the module's transparent-UART property), splitWrite so a >MTU stream burst is chunked; the
        // notify flow's raw bytes feed the inbound framer. The CC2541 re-chunks both ways - L2 framing
        // (SOF/len/CRC) tolerates it, so we never assume one notification per frame.
        pipeState.value = object : BleBytePipe {
            override suspend fun write(bytes: ByteArray) =
                writeChar.splitWrite(DataByteArray(bytes), BleWriteType.NO_RESPONSE)

            override val incoming: Flow<ByteArray> = notifications.map { it.value }
        }

        // Tighten the connection interval for the walk: low-latency request/reply over the 9600-baud
        // bridge, and a shorter interval makes the link less prone to the supervision-timeout drops seen
        // on the bench. (Kept HIGH for the whole connected session; the session is only the walk.)
        runCatching { gatt.requestConnectionPriority(BleGattConnectionPriority.HIGH) }
            .onFailure { Log.w(TAG, "requestConnectionPriority(HIGH) failed: ${it.message}") }

        _connectionState.value = ConnectionState.CONNECTED
        Log.d(TAG, "CONNECTED")

        // Block until the GATT link drops; the reconnect loop then schedules a fresh session if the
        // user still wants to be connected.
        gatt.connectionState.first { it == GattConnectionState.STATE_DISCONNECTED }
        Log.d(TAG, "GATT link dropped")
    }

    private fun tearDownSession() {
        pipeState.value = null
        try {
            client?.disconnect()
        } catch (e: Throwable) {
            Log.d(TAG, "disconnect on teardown threw (already dead): ${e.message}")
        }
        client = null
        if (keepConnected) {
            _connectionState.value = ConnectionState.SCANNING
        }
    }

    /**
     * Run the controller-side walk over the BLE link (slice 4), surviving mid-walk GATT drops by
     * reconnecting and restarting (the bench shows the CC2541 link dropping on a ~5 s supervision
     * timeout). The [BleWalkDriver] drives the connect/drop/reconnect loop against [pipeSource]; the
     * protocol stepping is the host-tested `BleWalkEngine`. Returns null if not even trying to connect.
     */
    override suspend fun discover(): WalkOutcome? {
        if (!keepConnected) {
            Log.w(TAG, "discover() ignored — not connected / not connecting")
            return null
        }
        Log.d(TAG, "discover() walking the fleet over BLE (with reconnect-and-resume)")
        return BleWalkDriver(pipeSource()).discover().also { Log.d(TAG, "discover() -> $it") }
    }

    /**
     * A [BlePipeSource] backed by the session loop: it awaits the next live pipe (the loop scans +
     * connects + reconnects on its own), so a `connect()` after a drop resolves once the loop has
     * re-established the link. Null if no live pipe appears within [PIPE_AWAIT_MS].
     */
    private fun pipeSource(): BlePipeSource = BlePipeSource {
        withTimeoutOrNull(PIPE_AWAIT_MS) { pipeState.filterNotNull().first() }
    }

    override fun disconnect() {
        keepConnected = false
        sessionJob?.cancel()
        sessionJob = null
        tearDownSession()
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    /** Cancel the transport's coroutine scope. Call when the owning component is destroyed. */
    fun shutdown() {
        disconnect()
        scope.cancel()
    }

    private companion object {

        const val TAG = "HoverboardBle"

        /** First reconnect attempt fires after this many ms; subsequent attempts back off. */
        const val RECONNECT_DELAY_MS = 800L
        const val RECONNECT_DELAY_MAX_MS = 5000L

        /** Pause after the scan stops, before a direct connect, so the radio is free. */
        const val SCAN_SETTLE_MS = 600L

        /** Bound on ClientBleGatt.connect() so a never-establishing link retries instead of hanging. */
        const val CONNECT_TIMEOUT_MS = 30_000L

        /** Bound on service discovery (a connected-but-silent GATT also strands CONNECTING). */
        const val DISCOVER_TIMEOUT_MS = 10_000L

        /** How long the walk driver waits for the session loop to (re)establish a live pipe. */
        const val PIPE_AWAIT_MS = 40_000L

        /** Log only every Nth notification so the bench log is readable. */
        const val LOG_EVERY_NOTIFY = 20

        /**
         * Pick the BLE characteristics that carry the transparent UART. Some CC2541-class modules
         * expose ONE characteristic with both write+notify; others split it across TWO (one write,
         * one notify) where only the notify one gets a CCCD (0x2902). Walk all non-SIG services and:
         *  - pick the WRITE char as the first with WRITE_WITHOUT_RESPONSE (preferred) or WRITE, and
         *  - pick the NOTIFY char as the first with NOTIFY and a CCCD descriptor (so Nordic's
         *    getNotifications() can write the CCCD without throwing).
         * The same characteristic may fill both roles when the module exposes a single pipe.
         */
        fun pickIoCharacteristics(
            services: ClientBleGattServices,
        ): Pair<ClientBleGattCharacteristic?, ClientBleGattCharacteristic?> {
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

        fun java.util.UUID.isSigStandardMetadata(): Boolean {
            // 0x1800 Generic Access, 0x1801 Generic Attribute, 0x180A Device Information
            val short = (mostSignificantBits ushr 32) and 0xFFFFL
            return short == 0x1800L || short == 0x1801L || short == 0x180AL
        }

        fun ClientBleGattCharacteristic.hasWrite(): Boolean =
            BleGattProperty.PROPERTY_WRITE_NO_RESPONSE in properties ||
                BleGattProperty.PROPERTY_WRITE in properties

        fun ClientBleGattCharacteristic.hasNotifyWithCccd(): Boolean {
            if (BleGattProperty.PROPERTY_NOTIFY !in properties) return false
            return descriptors.any { it.uuid == CCCD_UUID }
        }

        val CCCD_UUID: java.util.UUID =
            java.util.UUID.fromString("00002902-0000-1000-8000-00805F9B34FB")
    }
}
