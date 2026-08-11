package com.hoverboard.remote.ble

/**
 * One BLE advertisement seen during a scan, reduced to the three things selection needs.
 *
 * Both names are carried because they are not the same thing and either can be the one that matches:
 * [advertisedName] is the local name in the scan record the board is broadcasting right now;
 * [cachedName] is the name Android has cached against this MAC from an earlier session, which can be
 * stale for a long time after a board is renamed.
 */
data class ScanCandidate(
    val address: String,
    val advertisedName: String?,
    val cachedName: String?,
)

/**
 * Which advertisement to connect to. Pure, so the policy is testable without a radio.
 *
 * Two filters, in priority order:
 *
 *  1. **The remembered address**, if there is one. A MAC is unique; a name is not. Two masters
 *     staged with the same name are indistinguishable by name alone, and Android's cached GAP name
 *     can disagree with what a board is actually advertising after a rename. Preferring the address
 *     of the board that last worked cuts through both.
 *  2. **The configured name**, as the fallback. This is what finds a board the first time, or after
 *     a swap, and it is what the whole flow degrades to when nothing is remembered.
 *
 * The remembered address is stored per name (see [LinkSettings]), so changing the target name
 * discards the preference rather than pinning the app to a board the user just stopped asking for.
 */
object DeviceSelection {

    /**
     * Does this advertisement carry [targetName]?
     *
     * The CC2541 vendor firmware pads the AT+NAME slot with trailing whitespace, so the advertised
     * local name comes through as `"Hoverboard        "`. Both names are trimmed before comparison.
     * The comparison is otherwise EXACT, including case: the firmware's default is lowercase
     * `"hoverboard"` and the app's is `"Hoverboard"`, and quietly matching those two to each other
     * would hide an unstaged board rather than surface it.
     */
    fun matchesName(candidate: ScanCandidate, targetName: String): Boolean {
        val target = targetName.trim()
        if (target.isEmpty()) return false
        return candidate.advertisedName?.trim() == target || candidate.cachedName?.trim() == target
    }

    /** Is this the board whose address we remembered? Android reports MACs uppercase; be lenient. */
    fun matchesAddress(candidate: ScanCandidate, address: String): Boolean =
        candidate.address.equals(address, ignoreCase = true)

    /**
     * Pick from everything seen so far: the remembered address if it is present, else the first
     * name match, else nothing.
     *
     * Note the address wins WITHOUT also having to match the name. That is the point of it: a board
     * whose advertised name the phone has cached wrongly is exactly the case a name filter cannot
     * solve, and the address is the identity that does not drift.
     */
    fun choose(
        seen: List<ScanCandidate>,
        targetName: String,
        preferredAddress: String?,
    ): ScanCandidate? {
        if (preferredAddress != null) {
            seen.firstOrNull { matchesAddress(it, preferredAddress) }?.let { return it }
        }
        return seen.firstOrNull { matchesName(it, targetName) }
    }
}
