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
 * The picked board and the fallback target name, as they survive between runs.
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

    // ---- the fallback name -------------------------------------------------------------------

    @Test
    fun `an unconfigured app looks for Hoverboard by name`() {
        assertEquals("Hoverboard", settings().deviceName.value)
        assertEquals("Hoverboard", LinkConfig.DEFAULT_DEVICE_NAME)
        assertEquals(BoardTarget.ByName("Hoverboard"), settings().target())
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

    // ---- the picked board --------------------------------------------------------------------

    @Test
    fun `nothing is picked until the user taps a row`() {
        assertNull(settings().selectedBoard.value)
    }

    @Test
    fun `a picked board survives into a new instance, address and label`() {
        settings().selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        assertEquals(
            SelectedBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m"),
            settings().selectedBoard.value,
        )
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), settings().target())
    }

    @Test
    fun `a picked board outranks the name, so the name no longer decides anything`() {
        val store = settings()
        store.selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), store.target())
    }

    @Test
    fun `a board with no advertised name is picked by address alone`() {
        // The label is for the screen; the address is the identity. A board advertising nothing
        // must still be pickable, and reconnecting to it must not need a name it never had.
        settings().selectBoard("AA:BB:CC:DD:EE:FF", null)
        assertEquals(SelectedBoard("AA:BB:CC:DD:EE:FF", null), settings().selectedBoard.value)
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), settings().target())
    }

    @Test
    fun `picking another board replaces the pick`() {
        val store = settings()
        store.selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        store.selectBoard("11:22:33:44:55:66", "hb-stress")
        assertEquals(SelectedBoard("11:22:33:44:55:66", "hb-stress"), store.selectedBoard.value)
        assertEquals(SelectedBoard("11:22:33:44:55:66", "hb-stress"), settings().selectedBoard.value)
    }

    @Test
    fun `a pick is not disturbed by the name the app happens to be carrying`() {
        // The owner's requirement, as the store sees it: the pick is keyed by address and nothing
        // about a name makes it stale. The old store kept the address PER NAME and dropped it here.
        val store = settings()
        store.setDeviceName("hb-stress")
        store.selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), store.target())
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), settings().target())
    }

    @Test
    fun `typing a different name is the one thing that gives the pick up`() {
        // It has to be: with a pick in force the name decides nothing, so without this the name
        // box would be inert forever once a board had been tapped.
        val store = settings()
        store.selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        store.setDeviceName("hb-stress")
        assertNull(store.selectedBoard.value)
        assertNull(settings().selectedBoard.value)
        assertEquals(BoardTarget.ByName("hb-stress"), settings().target())
    }

    @Test
    fun `retyping the name already in force leaves the pick alone`() {
        // Every keystroke in the name box reaches the store, and the field starts out holding the
        // stored name. Re-submitting that same name is not the user asking for a different board.
        val store = settings()
        store.selectBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        store.setDeviceName(LinkConfig.DEFAULT_DEVICE_NAME)
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), store.target())
    }
}
