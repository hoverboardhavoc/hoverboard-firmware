package com.hoverboard.remote.ble

import android.content.Context
import android.content.SharedPreferences
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * The board the user picked, as it is remembered between runs.
 *
 * [address] is the identity and the only part the connect uses. [name] is the label the board was
 * advertising when it was picked, kept so the Connect screen can say which board it is pointing at
 * without having to find it first. It is display only, and it is allowed to be null (a board can
 * advertise no name at all) and allowed to go stale (the board can be renamed). Nothing may make
 * reconnecting conditional on it: a board that is renamed is still the same board.
 */
data class SelectedBoard(val address: String, val name: String?)

/**
 * The persisted answer to "which board is this app for".
 *
 * Two answers, and only one of them is in force at a time ([target]):
 *
 *  - **The picked board** ([selectedBoard]), keyed by ADDRESS. This is what tapping a row in the
 *    Connect screen's scan list stores, and it is sticky: nothing clears it but the user picking
 *    a different board or retargeting by name. In particular a board that is out of range, or
 *    powered down, or renamed, stays picked, because none of those make it a different board.
 *  - **The advertised name** ([deviceName]), the fallback path for a board that will not show up
 *    in a scan. It is what the app uses until a board has been picked.
 *
 * This replaced a last-connected address stored PER NAME, which had the pick evaporate the moment
 * the target name changed and fall back to whatever else answered to the name. The owner's
 * requirement is the opposite of that: the address is the identity, so the address is the key.
 *
 * [BleHoverboardTransport] reads [target] at the start of every scan rather than holding a copy,
 * so a change takes effect on the next scan without a restart, and there is no second value that
 * can disagree with it.
 *
 * Backed by [SharedPreferences]. DataStore was the alternative and was not worth it here: the
 * state is three strings read synchronously on one thread at scan time, and DataStore would add a
 * dependency plus a suspending read to the one place (inside the scan) that wants an answer
 * immediately.
 */
class LinkSettings(private val prefs: SharedPreferences) {

    constructor(context: Context) : this(
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE),
    )

    private val _deviceName = MutableStateFlow(
        prefs.getString(KEY_DEVICE_NAME, null)?.takeIf { it.isNotBlank() }
            ?: LinkConfig.DEFAULT_DEVICE_NAME,
    )

    /** The advertised name the manual entry path scans for. Never blank. */
    val deviceName: StateFlow<String> = _deviceName.asStateFlow()

    private val _selectedBoard = MutableStateFlow(storedSelection())

    /** The board picked from the scan list, or null while the app has never been pointed at one. */
    val selectedBoard: StateFlow<SelectedBoard?> = _selectedBoard.asStateFlow()

    /**
     * Pin the app to the board at [address], with [name] as its label for the screen.
     *
     * Called when the user taps a row, not when a connect succeeds: the pick is a statement of
     * intent about which machine this phone drives, and it has to survive the board being absent,
     * which is precisely the case where no connect ever succeeds to record it.
     */
    fun selectBoard(address: String, name: String?) {
        if (address.isBlank()) return
        val picked = SelectedBoard(address.trim(), name?.trim()?.takeIf { it.isNotEmpty() })
        if (picked == _selectedBoard.value) return
        prefs.edit()
            .putString(KEY_SELECTED_ADDRESS, picked.address)
            .putString(KEY_SELECTED_NAME, picked.name)
            .apply()
        _selectedBoard.value = picked
    }

    /**
     * Retarget the app at a differently-named board and persist it immediately.
     *
     * This DROPS the pinned board, because it is the one thing besides picking another row that
     * says the user wants a different machine, and so the only way back to the by-name path once a
     * board has been picked. A pin that outranked the name box would leave that box inert.
     *
     * A blank name is ignored rather than stored: it would match nothing and strand the user on a
     * scan screen with no way back other than reinstalling.
     */
    fun setDeviceName(name: String) {
        val trimmed = name.trim()
        if (trimmed.isEmpty() || trimmed == _deviceName.value) return
        prefs.edit()
            .putString(KEY_DEVICE_NAME, trimmed)
            .remove(KEY_SELECTED_ADDRESS)
            .remove(KEY_SELECTED_NAME)
            .apply()
        _deviceName.value = trimmed
        _selectedBoard.value = null
    }

    /** Which board the next scan is for: the pinned one if there is one, else the name. */
    fun target(): BoardTarget =
        _selectedBoard.value?.let { BoardTarget.Pinned(it.address) }
            ?: BoardTarget.ByName(_deviceName.value)

    private fun storedSelection(): SelectedBoard? {
        val address = prefs.getString(KEY_SELECTED_ADDRESS, null)?.takeIf { it.isNotBlank() }
            ?: return null
        return SelectedBoard(address, prefs.getString(KEY_SELECTED_NAME, null))
    }

    private companion object {
        const val PREFS_NAME = "hoverboard_link"
        const val KEY_DEVICE_NAME = "device_name"
        const val KEY_SELECTED_ADDRESS = "selected_address"
        const val KEY_SELECTED_NAME = "selected_name"
    }
}
