package com.hoverboard.remote.ble

import android.content.Context
import android.content.SharedPreferences
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * The persisted answer to "which board is this app for": the advertised name to scan for, and the
 * address of the board that name last connected to.
 *
 * This is the single owner of the target name. [BleHoverboardTransport] reads it at the start of
 * every scan rather than holding a copy, so changing the name in the UI takes effect on the next
 * scan without a restart, and there is no second value that can disagree with it.
 *
 * Backed by [SharedPreferences]. DataStore was the alternative and was not worth it here: the state
 * is two strings read synchronously on one thread at scan time, and DataStore would add a dependency
 * plus a suspending read to the one place (inside the scan) that wants an answer immediately.
 *
 * The remembered address is stored TOGETHER WITH the name it was learned under, and
 * [preferredAddress] only returns it while that name is still the target. Retargeting the app at a
 * different board must not leave it silently preferring the old board's MAC forever.
 */
class LinkSettings(private val prefs: SharedPreferences) {

    constructor(context: Context) : this(
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE),
    )

    private val _deviceName = MutableStateFlow(
        prefs.getString(KEY_DEVICE_NAME, null)?.takeIf { it.isNotBlank() }
            ?: LinkConfig.DEFAULT_DEVICE_NAME,
    )

    /** The advertised name to scan for. Never blank. */
    val deviceName: StateFlow<String> = _deviceName.asStateFlow()

    /**
     * Retarget the app at a differently-named board and persist it immediately.
     *
     * A blank name is ignored rather than stored: it would match nothing and strand the user on a
     * scan screen with no way back other than reinstalling.
     */
    fun setDeviceName(name: String) {
        val trimmed = name.trim()
        if (trimmed.isEmpty() || trimmed == _deviceName.value) return
        prefs.edit().putString(KEY_DEVICE_NAME, trimmed).apply()
        _deviceName.value = trimmed
    }

    /**
     * The address to prefer on the next scan, or null if there is nothing useful to prefer: nothing
     * has connected yet, or what was remembered belongs to a different target name.
     */
    fun preferredAddress(): String? {
        val forName = prefs.getString(KEY_LAST_ADDRESS_NAME, null) ?: return null
        if (forName != _deviceName.value) return null
        return prefs.getString(KEY_LAST_ADDRESS, null)?.takeIf { it.isNotBlank() }
    }

    /**
     * Record the board a session actually reached. Call on a SUCCESSFUL connect, not on picking a
     * device to try: the value of the preference is that it points at hardware known to work, so a
     * failed attempt must not overwrite a good address.
     */
    fun rememberAddress(address: String) {
        if (address.isBlank()) return
        prefs.edit()
            .putString(KEY_LAST_ADDRESS, address)
            .putString(KEY_LAST_ADDRESS_NAME, _deviceName.value)
            .apply()
    }

    private companion object {
        const val PREFS_NAME = "hoverboard_link"
        const val KEY_DEVICE_NAME = "device_name"
        const val KEY_LAST_ADDRESS = "last_address"
        const val KEY_LAST_ADDRESS_NAME = "last_address_name"
    }
}
