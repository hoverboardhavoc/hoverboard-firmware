package com.hoverboard.bletest.transport

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * The real Android BLE central (`specs/ble.md`, "Central side" + "Board-side peer"). It connects to the
 * onboard transparent BLE module, discovers the single write+notify characteristic at runtime, and
 * exposes it as a raw byte [Transport]. The module is dumb (a transparent UART bridge), so the central
 * does NOT trust a fixed UUID:
 *
 *  1. scan by the advertised name (the firmware's `AT+NAME`),
 *  2. discover all services,
 *  3. PREFER the 0xFFE0 service / 0xFFE1 characteristic ([com.hoverboard.bletest.codec] gatt hints), else
 *     fall back to walking every service for the first characteristic that has BOTH
 *     `WRITE_WITHOUT_RESPONSE` (or `WRITE`) AND `NOTIFY`.
 *
 * One characteristic carries both directions: write to it = bytes to the board UART; notify on it = the
 * board's UART output. This mirrors the proven `BLEHoverboardRemote` runtime discovery (prior art, not a
 * dependency).
 *
 * The [com.hoverboard.bletest.Devices.autoConnect] quirk is honored: the ASUS ROG (Android 8) only
 * connects with `autoConnect=true`, so it is read from the resolved device and passed to `connectGatt`.
 */
class BleTransport(
    private val context: Context,
    private val deviceName: String,
    private val autoConnect: Boolean,
    private val requestMtu: Int,
    private val writeWithResponse: Boolean,
) : Transport {

    // 16-bit base UUID expansion for the discovery hints (0xFFE0 service / 0xFFE1 characteristic).
    private val serviceHint = uuid16(0xFFE0)
    private val charHint = uuid16(0xFFE1)
    private val cccdUuid = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private val adapter: BluetoothAdapter =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter

    private var gatt: BluetoothGatt? = null
    private var writeChar: BluetoothGattCharacteristic? = null
    private val notifyQueue = ArrayDeque<BluetoothGattCharacteristic>()
    private var sink: ((ByteArray) -> Unit)? = null

    private val scanLatch = CountDownLatch(1)
    private val readyLatch = CountDownLatch(1)
    @Volatile private var found: BluetoothDevice? = null
    @Volatile private var negotiatedMtu = 23

    override fun onReceive(sink: (ByteArray) -> Unit) {
        this.sink = sink
    }

    override fun connect() {
        scanByName()
        val dev = found ?: error("device advertising '$deviceName' not found")
        // The quirk: autoConnect=true for the ASUS ROG; direct connect otherwise.
        gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            dev.connectGatt(context, autoConnect, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } else {
            dev.connectGatt(context, autoConnect, gattCallback)
        }
        // A generous wait covers the ASUS ROG's slow (~133 s) autoConnect path.
        check(readyLatch.await(180, TimeUnit.SECONDS)) { "GATT not ready (connect/discover/MTU/subscribe)" }
    }

    private fun scanByName() {
        val scanner = adapter.bluetoothLeScanner
        val cb = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) {
                // Match on the LIVE advertised local name from the scan record, NOT result.device.name --
                // the latter returns Android's CACHED name for the address, which is stale here (the
                // module persists/changes its name and the OS cache lags, e.g. an old hbloop/RoboBT/Pal).
                // scanRecord.deviceName is what the module is advertising right now.
                // TRIM: the TTC2541 space-pads its advertised name to a fixed width (e.g. "hbk2" comes
                // through as "hbk2              "), so compare trimmed.
                val advName = result.scanRecord?.deviceName?.trim()
                if (advName == deviceName) {
                    found = result.device
                    scanner.stopScan(this)
                    scanLatch.countDown()
                }
            }

            override fun onScanFailed(errorCode: Int) {
                android.util.Log.e("BLE_TPUT", "scan failed to start: errorCode=$errorCode")
            }
        }
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
        scanner.startScan(null, settings, cb)
        if (!scanLatch.await(30, TimeUnit.SECONDS)) {
            scanner.stopScan(cb)
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                g.requestMtu(requestMtu)
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            // DIAGNOSTIC: what MTU did the module actually grant vs what we requested? status 0 = success.
            android.util.Log.i("BLE_TPUT", "onMtuChanged requested=$requestMtu granted=$mtu status=$status")
            negotiatedMtu = mtu
            g.discoverServices()
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            // DIAGNOSTIC: dump the GATT table so we can see the module's actual write/notify char layout.
            for (svc in g.services) {
                for (c in svc.characteristics) {
                    android.util.Log.i(
                        "BLE_TPUT",
                        "gatt svc=${svc.uuid} char=${c.uuid} props=0x${Integer.toHexString(c.properties)}",
                    )
                }
            }
            // Pick the WRITE characteristic, then subscribe to ALL notify-capable characteristics. This
            // module (service 0x1000) WRITES on 0x1001 but NOTIFIES the UART echo on a SEPARATE char
            // 0x1002, so a single-characteristic assumption misses the return path. Subscribing every
            // notify char catches the module's notify whichever char it actually uses.
            writeChar = pickWriteCharacteristic(g) ?: error("no writable characteristic")
            notifyQueue.clear()
            for (svc in g.services) {
                for (c in svc.characteristics) {
                    if (c.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0) {
                        notifyQueue.addLast(c)
                    }
                }
            }
            if (notifyQueue.isEmpty()) error("no notify characteristic")
            subscribeNextNotify(g)
        }

        // Subscribe to notify characteristics one at a time -- Android serializes GATT ops, so writing
        // the next CCCD must wait for the previous onDescriptorWrite. When the queue drains, the link is
        // ready. (Each: enable notifications locally, then write ENABLE_NOTIFICATION to the CCCD.)
        private fun subscribeNextNotify(g: BluetoothGatt) {
            val c = notifyQueue.removeFirstOrNull()
            if (c == null) {
                // DIAGNOSTIC: the effective MTU and the write-chunk size the run will actually use.
                android.util.Log.i(
                    "BLE_TPUT",
                    "ready: negotiatedMtu=$negotiatedMtu writeChunk=${(negotiatedMtu - 3).coerceAtLeast(20)}",
                )
                readyLatch.countDown()
                return
            }
            g.setCharacteristicNotification(c, true)
            val cccd = c.getDescriptor(cccdUuid)
            if (cccd != null) {
                cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                g.writeDescriptor(cccd)
            } else {
                subscribeNextNotify(g)
            }
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            subscribeNextNotify(g)
        }

        @Deprecated("compat for API < 33")
        override fun onCharacteristicChanged(g: BluetoothGatt, ch: BluetoothGattCharacteristic) {
            ch.value?.let { sink?.invoke(it) }
        }

        override fun onCharacteristicChanged(
            g: BluetoothGatt,
            ch: BluetoothGattCharacteristic,
            value: ByteArray,
        ) {
            sink?.invoke(value)
        }
    }

    /**
     * Runtime discovery of the WRITE characteristic (the central->board direction): prefer the 0xFFE0/0xFFE1
     * hint, else the first characteristic with WRITE_WITHOUT_RESPONSE or WRITE ("the module is dumb; we
     * don't trust the UUID"). The board->central (notify) direction may be a DIFFERENT characteristic, so it
     * is handled separately by subscribing to every notify-capable characteristic.
     */
    private fun pickWriteCharacteristic(g: BluetoothGatt): BluetoothGattCharacteristic? {
        val writable = BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE or
            BluetoothGattCharacteristic.PROPERTY_WRITE
        val notify = BluetoothGattCharacteristic.PROPERTY_NOTIFY
        g.getService(serviceHint)?.getCharacteristic(charHint)?.let {
            if (it.properties and writable != 0) return it
        }
        // PREFER a char with BOTH write AND notify: the dumb module's data char (0x1001) advertises both,
        // which distinguishes it from standard GATT chars that are merely writable (e.g. Generic Access
        // 0x2a02 Peripheral Privacy Flag). Only if none has both, fall back to the first writable.
        for (svc in g.services) {
            for (ch in svc.characteristics) {
                if (ch.properties and writable != 0 && ch.properties and notify != 0) return ch
            }
        }
        for (svc in g.services) {
            for (ch in svc.characteristics) {
                if (ch.properties and writable != 0) return ch
            }
        }
        return null
    }

    override fun send(bytes: ByteArray) {
        val g = gatt ?: error("not connected")
        val ch = writeChar ?: error("no characteristic")
        // Split into <= MTU-3 chunks written in order (no length prefix or delimiter, per the spec).
        val chunk = (negotiatedMtu - 3).coerceAtLeast(20)
        val writeType = if (writeWithResponse) {
            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        } else {
            BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        }
        var off = 0
        while (off < bytes.size) {
            val end = minOf(off + chunk, bytes.size)
            val part = bytes.copyOfRange(off, end)
            off = end
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                g.writeCharacteristic(ch, part, writeType)
            } else {
                @Suppress("DEPRECATION")
                ch.writeType = writeType
                @Suppress("DEPRECATION")
                ch.value = part
                @Suppress("DEPRECATION")
                g.writeCharacteristic(ch)
            }
        }
    }

    override fun close() {
        gatt?.disconnect()
        gatt?.close()
        gatt = null
    }

    private fun uuid16(short: Int): UUID =
        UUID.fromString(String.format("0000%04x-0000-1000-8000-00805f9b34fb", short))
}
