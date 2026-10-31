package com.hoverboard.remote.ble

/**
 * One BLE advertisement seen during a scan, reduced to what both the picker and the connect filter
 * need from it.
 *
 * Both names are carried because they are not the same thing and either can be the one that
 * matters: [advertisedName] is the local name in the scan record the board is broadcasting right
 * now; [cachedName] is the name Android has cached against this MAC from an earlier session, which
 * can be stale for a long time after a board is renamed.
 *
 * [rssi] is null when the scan result carried no advertisement data at all (the Nordic scanner's
 * `data` is nullable, and the signal strength lives on it), which is why the picker treats signal
 * strength as something a row may or may not have rather than as a number every row shows.
 */
data class ScanCandidate(
    val address: String,
    val advertisedName: String?,
    val cachedName: String?,
    val rssi: Int?,
) {
    /**
     * What to call this board on screen, or null when it is advertising no name under either
     * spelling and the address is the only thing there is to show.
     *
     * The advertised name wins over the cached one for the same reason the connect filter accepts
     * either: the cached one is what Android remembers, the advertised one is what the board is
     * saying now, and after a rename only one of those is true.
     */
    val displayName: String?
        get() = advertisedName.cleaned() ?: cachedName.cleaned()

    private fun String?.cleaned(): String? = this?.trim()?.takeIf { it.isNotEmpty() }
}

/**
 * Which board a connect attempt is for. Two kinds, because the two ways of naming a board behave
 * differently when the board is not there, and that difference is the whole point of the type.
 */
sealed interface BoardTarget {

    /**
     * A board the user picked from the scan list, identified by its MAC.
     *
     * A MAC is the board's identity: it is unique, and unlike the advertised name it survives a
     * rename, so a pinned board is still the same board after `AT+NAME` changes. Nothing else can
     * satisfy this target. A pin that is not advertising is a board that is out of range or
     * powered down, which is a connect that waits, never a reason to reach for some other board.
     */
    data class Pinned(val address: String) : BoardTarget

    /**
     * A board named by the user in the manual entry box, for when a board will not turn up in a
     * scan. First advertisement answering to the name wins; see [DeviceSelection.matchesName].
     */
    data class ByName(val name: String) : BoardTarget
}

/**
 * Whether an advertisement satisfies the target. Pure, so the policy is testable without a radio.
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

    /** Is this the board whose address was pinned? Android reports MACs uppercase; be lenient. */
    fun matchesAddress(candidate: ScanCandidate, address: String): Boolean =
        candidate.address.equals(address, ignoreCase = true)

    /**
     * Is this the advertisement the connect is waiting for?
     *
     * Note what a [BoardTarget.Pinned] target does NOT do: it does not also check the name, and it
     * does not fall back to a name match when the pinned board is absent. Both are deliberate.
     * A phone can hold a stale cached GAP name against a MAC long after a board was renamed, so
     * requiring the name as well would lose exactly the board the pin exists to hold on to; and
     * falling back would hand the rider a different machine than the one they picked, which is the
     * single worst thing this screen could do.
     */
    fun matches(candidate: ScanCandidate, target: BoardTarget): Boolean = when (target) {
        is BoardTarget.Pinned -> matchesAddress(candidate, target.address)
        is BoardTarget.ByName -> matchesName(candidate, target.name)
    }
}
