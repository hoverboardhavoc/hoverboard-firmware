package com.hoverboard.remote

import android.app.Application
import android.graphics.BitmapFactory
import androidx.compose.material3.Text
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsNotDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithContentDescription
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performTouchInput
import androidx.compose.ui.test.swipeRight
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.github.takahirom.roborazzi.captureRoboImage
import com.hoverboard.remote.ui.screens.AppDestination
import com.hoverboard.remote.ui.screens.ConnectedScreen
import com.hoverboard.remote.ui.screens.DRAWER_ICON_TAG
import com.hoverboard.remote.ui.screens.drawerGesturesEnabled
import com.hoverboard.remote.ui.screens.drawerItemTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode

/**
 * The navigation drawer that replaced the tab row (`specs/rider-ui.md` section 2a).
 *
 * Two properties here are safety properties, not layout ones. The edge swipe that opens a drawer is
 * the same gesture the throttle pad and the joystick claim, so on RIDE the drawer must not take it;
 * and the armed state used to be legible from the Ride screen's tinted surfaces, which a drawer
 * hides, so the drawer icon has to carry the tint on every destination.
 */
@RunWith(AndroidJUnit4::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class DrawerTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int) = context.getString(id)

    /** The connected app with placeholder destinations, starting at [start]. */
    private fun show(start: AppDestination = AppDestination.RIDE, armed: Boolean = false): () -> Int {
        var released = 0
        compose.setContent {
            var destination by remember { mutableStateOf(start) }
            HoverboardRemoteTheme {
                ConnectedScreen(
                    destination = destination,
                    onDestination = { destination = it },
                    onLeaveRide = { released++ },
                    armed = armed,
                    ride = { Text("RIDE-CONTENT") },
                    tune = { Text("TUNE-CONTENT") },
                    setup = { Text("SETUP-CONTENT") },
                    layout = { Text("LAYOUT-CONTENT") },
                )
            }
        }
        return { released }
    }

    @Test
    fun theEdgeSwipeDoesNotOpenTheDrawerOnRide() {
        show(AppDestination.RIDE)

        compose.onNodeWithText("RIDE-CONTENT").performTouchInput { swipeRight() }

        // The sheet is always composed, parked off the left edge, so "shut" is its items being
        // off screen rather than absent. The pad keeps the gesture, which is the point.
        compose.onNodeWithTag(drawerItemTag(AppDestination.TUNE)).assertIsNotDisplayed()
        compose.onNodeWithText("RIDE-CONTENT").assertIsDisplayed()
    }

    @Test
    fun theEdgeSwipeOpensTheDrawerOnTheOtherDestinations() {
        show(AppDestination.TUNE)

        compose.onNodeWithText("TUNE-CONTENT").performTouchInput { swipeRight() }

        compose.onNodeWithTag(drawerItemTag(AppDestination.RIDE)).assertIsDisplayed()
    }

    @Test
    fun theDragIsOfferedEverywhereExceptAClosedDrawerOnRide() {
        assertFalse(drawerGesturesEnabled(AppDestination.RIDE, drawerOpen = false))
        assertTrue(drawerGesturesEnabled(AppDestination.RIDE, drawerOpen = true))
        for (d in AppDestination.entries - AppDestination.RIDE) {
            assertTrue(d.name, drawerGesturesEnabled(d, drawerOpen = false))
        }
    }

    @Test
    fun theIconOpensTheDrawerOnRideAndLeavingRideReleasesTheThrottle() {
        val released = show(AppDestination.RIDE)

        compose.onNodeWithTag(DRAWER_ICON_TAG).performClick()
        compose.onNodeWithTag(drawerItemTag(AppDestination.TUNE)).assertIsDisplayed().performClick()

        compose.onNodeWithText("TUNE-CONTENT").assertIsDisplayed()
        compose.onNodeWithText("RIDE-CONTENT").assertDoesNotExist()
        compose.onNodeWithTag(drawerItemTag(AppDestination.TUNE)).assertIsNotDisplayed()
        assertEquals(1, released())

        compose.onNodeWithTag(DRAWER_ICON_TAG).performClick()
        compose.onNodeWithTag(drawerItemTag(AppDestination.SETUP)).performClick()
        compose.onNodeWithText("SETUP-CONTENT").assertIsDisplayed()
        assertEquals("leaving Tune released the throttle again", 1, released())

        compose.onNodeWithTag(DRAWER_ICON_TAG).performClick()
        compose.onNodeWithTag(drawerItemTag(AppDestination.LAYOUT)).performClick()
        compose.onNodeWithText("LAYOUT-CONTENT").assertIsDisplayed()
    }

    @Test
    fun theDrawerIconCarriesTheArmedTintWhileArmed() {
        show(AppDestination.TUNE, armed = true)

        assertTrue("the armed tint is not on the drawer icon", armedPixels("armed") > 0)
        compose.onNodeWithContentDescription(s(R.string.nav_open_armed)).assertExists()
    }

    @Test
    fun theDrawerIconCarriesNoArmedTintWhileDisarmed() {
        show(AppDestination.TUNE, armed = false)

        assertEquals(0, armedPixels("disarmed"))
        compose.onNodeWithContentDescription(s(R.string.nav_open)).assertExists()
    }

    /**
     * How many pixels of the drawer icon are drawn in the armed colour.
     *
     * Captured through Roborazzi rather than `captureToImage`, which syncs on a real window redraw
     * Robolectric never performs, so the capture times out instead of failing on the colour. These
     * are working captures under `build/`, not screenshot baselines.
     */
    private fun armedPixels(name: String): Int {
        val path = "build/test-captures/drawer_icon_$name.png"
        compose.onNodeWithTag(DRAWER_ICON_TAG).captureRoboImage(path)
        val image = BitmapFactory.decodeFile(path)
        var armed = 0
        for (x in 0 until image.width) {
            for (y in 0 until image.height) {
                if (isArmedTint(image.getPixel(x, y))) armed++
            }
        }
        return armed
    }

    /**
     * Whether [argb] is `AccentRed` as drawn: the icon is antialiased over a dark background, so its
     * edge pixels are the tint at a partial alpha and only the interior matches exactly. Each
     * channel is compared against the tint's own, which is what makes this a test of the TINT and
     * not of the renderer.
     */
    private fun isArmedTint(argb: Int): Boolean {
        val a = (argb ushr ALPHA_SHIFT) and CHANNEL
        val r = (argb ushr RED_SHIFT) and CHANNEL
        val g = (argb ushr GREEN_SHIFT) and CHANNEL
        val b = argb and CHANNEL
        return a > CHANNEL / 2 && r > CHANNEL / 2 && g < ARMED_GREEN + MARGIN && b < ARMED_BLUE + MARGIN
    }

    private companion object {
        const val CHANNEL = 0xFF
        const val ALPHA_SHIFT = 24
        const val RED_SHIFT = 16
        const val GREEN_SHIFT = 8

        /** The green and the blue channel of `AccentRed` (0xFFFF4D4D), which the tint has to match. */
        const val ARMED_GREEN = 0x4D
        const val ARMED_BLUE = 0x4D

        /** How far a channel may sit above the tint's own and still be that tint, compositing aside. */
        const val MARGIN = 0x1A
    }
}
