package com.hoverboard.remote.ble

import android.annotation.SuppressLint
import android.content.Context
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import no.nordicsemi.android.kotlin.ble.core.scanner.BleScanResult
import no.nordicsemi.android.kotlin.ble.scanner.BleScanner

/**
 * A scan the Connect screen can run on its own, to show the user what is nearby.
 *
 * This is a second seam next to [HoverboardTransport], not a method on it, because the two scans
 * answer different questions. The transport scans to reach ONE board and stops at the first
 * advertisement that satisfies the target; this one never stops on its own, reports everything it
 * hears, and connects to nothing. Folding it into the transport would mean a scan that sometimes
 * connects, which is the kind of overloading the connect path is hard enough to reason about
 * without.
 *
 * Kept behind an interface for the same reason the transport is: the picker's wiring can then be
 * driven by a fake in unit tests, with no Android BLE stack.
 */
interface BoardScanner {

    /**
     * Every advertisement heard, as it arrives, until the collector stops. The same board is
     * reported repeatedly; [ScanList.merge] is what turns that into a list of boards.
     */
    fun scan(): Flow<ScanCandidate>
}

/** The real radio, via the Nordic scanner the transport uses. */
class BleBoardScanner(private val context: Context) : BoardScanner {

    @SuppressLint("MissingPermission")
    override fun scan(): Flow<ScanCandidate> =
        // Unfiltered: the whole point of the picker is to show boards whose name the user does not
        // know, or that are advertising a name the app would not have matched.
        BleScanner(context).scan().map { it.toCandidate() }
}

/**
 * Reduce a Nordic scan result to the fields the app reasons about. Both names are carried: the
 * scan-record local name is what the board is advertising now, the device name is what Android has
 * cached against the MAC and can be stale after a rename.
 *
 * One mapping, used by both scans, so the picker's rows and the connect filter are reading the
 * same advertisement the same way.
 */
@SuppressLint("MissingPermission")
internal fun BleScanResult.toCandidate(): ScanCandidate = ScanCandidate(
    address = device.address,
    advertisedName = data?.scanRecord?.deviceName,
    cachedName = device.name,
    rssi = data?.rssi,
)
