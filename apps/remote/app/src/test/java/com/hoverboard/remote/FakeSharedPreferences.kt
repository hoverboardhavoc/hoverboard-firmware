package com.hoverboard.remote

import android.content.SharedPreferences

/**
 * In-memory [SharedPreferences] for the plain (non-Robolectric) unit tests, so a test that is not
 * about persistence does not have to boot an Android runtime to construct a
 * [com.hoverboard.remote.ble.LinkSettings].
 *
 * Deliberately only the string operations are real: that is all LinkSettings uses. The rest return
 * the caller's default rather than throwing, so an accidental new call site shows up as a test
 * behaving oddly rather than as an exception from a stub.
 *
 * That REAL SharedPreferences behaves the same way is not assumed here: `LinkSettingsTest` runs the
 * same store against Robolectric's genuine implementation.
 */
class FakeSharedPreferences : SharedPreferences {

    private val values = mutableMapOf<String, String>()

    override fun getAll(): Map<String, *> = values.toMap()

    override fun getString(key: String, defValue: String?): String? = values[key] ?: defValue

    override fun getStringSet(key: String, defValues: MutableSet<String>?): MutableSet<String>? =
        defValues

    override fun getInt(key: String, defValue: Int): Int = defValue

    override fun getLong(key: String, defValue: Long): Long = defValue

    override fun getFloat(key: String, defValue: Float): Float = defValue

    override fun getBoolean(key: String, defValue: Boolean): Boolean = defValue

    override fun contains(key: String): Boolean = values.containsKey(key)

    override fun edit(): SharedPreferences.Editor = Editor()

    override fun registerOnSharedPreferenceChangeListener(
        listener: SharedPreferences.OnSharedPreferenceChangeListener?,
    ) = Unit

    override fun unregisterOnSharedPreferenceChangeListener(
        listener: SharedPreferences.OnSharedPreferenceChangeListener?,
    ) = Unit

    /** Writes land in the backing map on apply/commit, as the real editor does. */
    private inner class Editor : SharedPreferences.Editor {
        private val pending = mutableMapOf<String, String?>()
        private var clearAll = false

        override fun putString(key: String, value: String?): SharedPreferences.Editor = apply {
            pending[key] = value
        }

        override fun putStringSet(
            key: String,
            values: MutableSet<String>?,
        ): SharedPreferences.Editor = this

        override fun putInt(key: String, value: Int): SharedPreferences.Editor = this

        override fun putLong(key: String, value: Long): SharedPreferences.Editor = this

        override fun putFloat(key: String, value: Float): SharedPreferences.Editor = this

        override fun putBoolean(key: String, value: Boolean): SharedPreferences.Editor = this

        override fun remove(key: String): SharedPreferences.Editor = apply { pending[key] = null }

        override fun clear(): SharedPreferences.Editor = apply { clearAll = true }

        override fun commit(): Boolean {
            apply()
            return true
        }

        override fun apply() {
            if (clearAll) values.clear()
            pending.forEach { (key, value) ->
                if (value == null) values.remove(key) else values[key] = value
            }
            pending.clear()
            clearAll = false
        }
    }
}
