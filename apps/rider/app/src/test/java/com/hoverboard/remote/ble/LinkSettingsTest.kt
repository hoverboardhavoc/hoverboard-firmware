package com.hoverboard.remote.ble

import android.app.Application
import android.content.Context
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * The persisted target name and the remembered board address.
 *
 * Runs against Robolectric's REAL SharedPreferences rather than an in-memory stand-in, because the
 * property under test is that a value written by one instance is visible to the next one, and a fake
 * map would prove only that a map is a map.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class)
class LinkSettingsTest {

    private lateinit var context: Context

    @Before
    fun setUp() {
        context = ApplicationProvider.getApplicationContext()
        // Robolectric keeps prefs per-test, but be explicit: these tests are about what persists.
        context.getSharedPreferences("hoverboard_link", Context.MODE_PRIVATE)
            .edit().clear().commit()
    }

    private fun settings() = LinkSettings(context)

    @Test
    fun `an unconfigured app targets Hoverboard`() {
        assertEquals("Hoverboard", settings().deviceName.value)
        assertEquals("Hoverboard", LinkConfig.DEFAULT_DEVICE_NAME)
    }

    @Test
    fun `a changed name survives into a new instance`() {
        settings().setDeviceName("hb-offroad-m")
        // A fresh instance is what the app gets on the next launch.
        assertEquals("hb-offroad-m", settings().deviceName.value)
    }

    @Test
    fun `the name is trimmed before it is stored`() {
        // Phone keyboards add a trailing space readily, and the scan filter compares exactly.
        settings().setDeviceName("  hb-stress  ")
        assertEquals("hb-stress", settings().deviceName.value)
    }

    @Test
    fun `a blank name is refused rather than stored`() {
        // Clearing the text box must not leave the app targeting a name that can match nothing.
        val store = settings()
        store.setDeviceName("hb-stress")
        store.setDeviceName("   ")
        assertEquals("hb-stress", store.deviceName.value)
        assertEquals("hb-stress", settings().deviceName.value)
    }

    @Test
    fun `nothing is preferred until a board has actually connected`() {
        assertNull(settings().preferredAddress())
    }

    @Test
    fun `a connected board's address is preferred on the next run`() {
        settings().rememberAddress("AA:BB:CC:DD:EE:FF")
        assertEquals("AA:BB:CC:DD:EE:FF", settings().preferredAddress())
    }

    @Test
    fun `retargeting the app at a different name drops the remembered address`() {
        // The remembered address belongs to the board that answered to the OLD name. Preferring it
        // after the user asks for a different board would pin the app to the wrong hardware, and
        // the address preference deliberately outranks the name filter.
        val store = settings()
        store.rememberAddress("AA:BB:CC:DD:EE:FF")
        store.setDeviceName("hb-stress")
        assertNull(store.preferredAddress())
        assertNull(settings().preferredAddress())
    }

    @Test
    fun `going back to the original name restores its remembered address`() {
        val store = settings()
        store.rememberAddress("AA:BB:CC:DD:EE:FF")
        store.setDeviceName("hb-stress")
        store.setDeviceName("Hoverboard")
        assertEquals("AA:BB:CC:DD:EE:FF", store.preferredAddress())
    }

    @Test
    fun `the address is remembered against the name in force when it connected`() {
        val store = settings()
        store.setDeviceName("hb-stress")
        store.rememberAddress("11:22:33:44:55:66")
        assertEquals("11:22:33:44:55:66", store.preferredAddress())
        store.setDeviceName("Hoverboard")
        assertNull(store.preferredAddress())
    }
}
