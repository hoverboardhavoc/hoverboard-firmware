package com.hoverboard.stress.ble

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import com.hoverboard.stress.model.ConnectionState
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
 * The production BLE path (a lightly-adapted copy of `Hoverboard/.../ble/BleHoverboardTransport.kt`),
 * exposing the live [BleBytePipe] to the stress runner instead of driving the L3 walk. The flow is
 * byte-for-byte the real walk's link, so a result here characterizes the real failing link:
 *
 *  1. Scan for [LinkConfig.deviceName] (set by the firmware's `AT+NAME`).
 *  2. Connect (autoConnect = true — the queued/opportunistic connect cheap CC2541 modules need) and
 *     discover services.
 *  3. Runtime-pick the transparent-UART write (0x1001) / notify (0x1002) pair.
 *  4. Subscribe (validates the CCCD path), `requestConnectionPriority(HIGH)`, publish a live
 *     [BleBytePipe] on [pipe], and report [ConnectionState.CONNECTED] until the GATT link drops.
 *
 * The reconnect loop is kept (the bench shows the CC2541 dropping on a ~5 s supervision timeout): each
 * established session bumps [sessionGeneration] and stamps [connectedAtMs]; a drop clears [pipe] and
 * stamps [disconnectedAtMs], which the runner uses for the connection-stability metric.
 */
class BleStressTransport(
    private val context: Context,
    private val config: LinkConfig = LinkConfig(),
    /** Connection-priority request applied after connect: "none" (don't call it — let the module's own
     *  L2CAP param request stand), "low" (LOW_POWER ~100ms/2s timeout), "balanced" (~45ms/5s),
     *  "high" (~15ms/2s). Diagnostic lever for the conn-param trajectory. */
    private val connPriority: String = "none",
    /** Diagnostic: use WRITE (with ATT response) instead of WRITE_NO_RESPONSE on the write char. A
     *  with-response write that times out/throws proves the connection cannot carry data PDUs at all;
     *  one that returns OK while the firmware still sees 0 bytes means the module drops the UART forward. */
    private val writeWithResponse: Boolean = false,
    /** Diagnostic: createBond() before connecting. A bonded device lets Android cache the GATT DB and
     *  skip service discovery on reconnect — discovery is what triggers Android's fast (7.5ms) interval
     *  burst the slow CC2541 desyncs on. */
    private val bond: Boolean = false,
    /** autoConnect option for ClientBleGatt.connect. true (DEFAULT, matches the production transport) =
     *  Android's opportunistic/background connect, which holds the CC2541 link drop-free; false =
     *  direct/aggressive connect, which fails (5s supervision-timeout drop or connect-timeout). The
     *  uncommitted flip to false was the regression that produced the whole OnePlus "instability" saga. */
    private val autoConnect: Boolean = true,
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob()),
) {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    /** The live byte pipe over the connected write/notify chars; non-null only while CONNECTED. */
    private val pipeState = MutableStateFlow<BleBytePipe?>(null)
    val pipe: StateFlow<BleBytePipe?> = pipeState.asStateFlow()

    /** Monotonic counter bumped each time a fresh CONNECTED session is established (drop detection). */
    @Volatile
    var sessionGeneration: Int = 0
        private set

    /** `System.currentTimeMillis()` of the last CONNECTED transition, or 0. */
    @Volatile
    var connectedAtMs: Long = 0L
        private set

    /** `System.currentTimeMillis()` of the last GATT drop, or 0. */
    @Volatile
    var disconnectedAtMs: Long = 0L
        private set

    private var client: ClientBleGatt? = null
    private var sessionJob: Job? = null
    private var keepConnected: Boolean = false

    @SuppressLint("MissingPermission")
    fun connect() {
        Log.d(TAG, "connect() state=${_connectionState.value}")
        if (sessionJob?.isActive == true) {
            Log.d(TAG, "connect() ignored — session already running")
            return
        }
        keepConnected = true
        sessionJob = scope.launch { runWithReconnect() }
    }

    /** Await the next live pipe (the session loop scans + connects on its own), or null on timeout. */
    suspend fun awaitPipe(timeoutMs: Long): BleBytePipe? =
        withTimeoutOrNull(timeoutMs) { pipeState.filterNotNull().first() }

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
                    val advName = result.data?.scanRecord?.deviceName?.trim()
                    val devName = result.device.name?.trim()
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

        if (bond) {
            ensureBonded(device.address)
        }
        delay(SCAN_SETTLE_MS)

        val gatt = withTimeout(CONNECT_TIMEOUT_MS) {
            ClientBleGatt.connect(
                context,
                device,
                scope,
                options = BleGattConnectOptions(autoConnect = autoConnect),
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
        Log.i(
            WIRE,
            "chars write=${writeChar.uuid} props=${writeChar.properties} (using NO_RESPONSE) " +
                "notify=${notifyChar.uuid} props=${notifyChar.properties}",
        )

        val notifications = notifyChar.getNotifications()

        // Stamp the generation + connect time BEFORE publishing the pipe: awaitPipe() unblocks the
        // instant pipeState goes non-null, and the runner captures sessionGeneration right after, so the
        // increment must already be visible or the runner sees a stale gen and treats it as an instant drop.
        sessionGeneration++
        connectedAtMs = System.currentTimeMillis()

        val writeType = if (writeWithResponse) BleWriteType.DEFAULT else BleWriteType.NO_RESPONSE
        pipeState.value = object : BleBytePipe {
            override suspend fun write(bytes: ByteArray) {
                Log.i(WIRE, "tx ${bytes.size}B -> 0x1001 ($writeType): ${bytes.toHex()}")
                // plain write (single ATT Write Command) — matches the ESP central that bridges OK,
                // vs splitWrite's prepared/long-write path the CC2541 may not forward to UART.
                writeChar.write(DataByteArray(bytes), writeType)
            }

            override val incoming: Flow<ByteArray> = notifications.map { n ->
                Log.i(WIRE, "rx ${n.value.size}B <- 0x1002: ${n.value.toHex()}")
                n.value
            }
        }

        // Diagnostic lever: which connection-priority (if any) keeps the slow CC2541 synced.
        val prio = when (connPriority.lowercase()) {
            "high" -> BleGattConnectionPriority.HIGH
            "balanced" -> BleGattConnectionPriority.BALANCED
            "low" -> BleGattConnectionPriority.LOW_POWER
            else -> null // "none": don't request — let the module's L2CAP param request stand
        }
        if (prio != null) {
            runCatching { gatt.requestConnectionPriority(prio) }
                .onFailure { Log.w(TAG, "requestConnectionPriority($prio) failed: ${it.message}") }
        }
        Log.i(WIRE, "connPriority=$connPriority bond=$bond autoConnect=$autoConnect")

        _connectionState.value = ConnectionState.CONNECTED
        Log.d(TAG, "CONNECTED (gen=$sessionGeneration)")

        gatt.connectionState.first { it == GattConnectionState.STATE_DISCONNECTED }
        disconnectedAtMs = System.currentTimeMillis()
        Log.d(TAG, "GATT link dropped after ${disconnectedAtMs - connectedAtMs}ms")
    }

    /** Bond to the module by MAC, waiting up to ~8 s for BOND_BONDED. No-op if already bonded or if
     *  the module rejects pairing (logged). */
    @SuppressLint("MissingPermission")
    private suspend fun ensureBonded(address: String) {
        val mgr = context.getSystemService(Context.BLUETOOTH_SERVICE) as android.bluetooth.BluetoothManager
        val dev = mgr.adapter.getRemoteDevice(address)
        if (dev.bondState == android.bluetooth.BluetoothDevice.BOND_BONDED) {
            Log.i(WIRE, "already bonded to $address"); return
        }
        Log.i(WIRE, "createBond($address) -> ${dev.createBond()} state=${dev.bondState}")
        val deadline = System.currentTimeMillis() + 8000
        while (System.currentTimeMillis() < deadline) {
            when (dev.bondState) {
                android.bluetooth.BluetoothDevice.BOND_BONDED -> { Log.i(WIRE, "BONDED"); return }
                android.bluetooth.BluetoothDevice.BOND_NONE ->
                    if (System.currentTimeMillis() > deadline - 7000) { Log.w(WIRE, "bond fell to NONE"); return }
            }
            delay(250)
        }
        Log.w(WIRE, "bond timed out state=${dev.bondState}")
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

    fun disconnect() {
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

        const val TAG = "HbStressBle"

        /** Byte-level wire diagnostic tag: every chunk tx'd to / rx'd from the CC2541 GATT chars. */
        const val WIRE = "BleWire"

        fun ByteArray.toHex(): String = joinToString(" ") { "%02x".format(it) }

        const val RECONNECT_DELAY_MS = 800L
        const val RECONNECT_DELAY_MAX_MS = 5000L
        const val SCAN_SETTLE_MS = 600L
        // autoConnect=true on Android 8 can take ~130s to land the opportunistic connection; give it
        // room so a single attempt succeeds instead of timing out at 30s and re-scanning.
        const val CONNECT_TIMEOUT_MS = 150_000L
        const val DISCOVER_TIMEOUT_MS = 10_000L

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
