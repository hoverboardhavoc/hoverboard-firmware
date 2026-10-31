package com.hoverboard.remote.ble

/**
 * The list of boards a running scan has turned up, folded from the stream of advertisements the
 * radio delivers. Pure, like [DeviceSelection], so the behaviour the picker depends on is testable
 * without a radio.
 *
 * A BLE scan is not a list of devices, it is a repeating stream of advertisements: the same board
 * is reported again every few hundred milliseconds, and consecutive reports from one board do not
 * carry the same fields. Three consequences shape [merge]:
 *
 *  - **One row per address.** Without this the screen fills with the same board over and over.
 *  - **A later sighting never erases what an earlier one said.** Devices alternate between an
 *    advertisement carrying the local name and one that does not, so taking the newest report
 *    wholesale makes names blink in and out while the user is trying to tap one.
 *  - **Rows keep the order they were first seen in.** The user is aiming a finger at a moving
 *    list; re-sorting it by signal strength as the numbers wobble means tapping the wrong board.
 */
object ScanList {

    /**
     * Fold one fresh advertisement into the rows gathered so far, returning the new list.
     *
     * The list is returned unchanged (same instance) when the sighting says nothing new, so a
     * board re-advertising an identical record does not churn the UI.
     */
    fun merge(seen: List<ScanCandidate>, sighting: ScanCandidate): List<ScanCandidate> {
        val at = seen.indexOfFirst { DeviceSelection.matchesAddress(it, sighting.address) }
        if (at < 0) return seen + sighting
        val known = seen[at]
        val merged = sighting.copy(
            advertisedName = sighting.advertisedName ?: known.advertisedName,
            cachedName = sighting.cachedName ?: known.cachedName,
            rssi = sighting.rssi ?: known.rssi,
        )
        if (merged == known) return seen
        return seen.toMutableList().also { it[at] = merged }
    }
}
