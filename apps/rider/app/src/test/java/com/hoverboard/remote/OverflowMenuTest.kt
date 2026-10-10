package com.hoverboard.remote

import android.app.Application
import android.graphics.BitmapFactory
import androidx.activity.OnBackPressedDispatcher
import androidx.activity.compose.LocalOnBackPressedDispatcherOwner
import androidx.compose.material3.Text
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.test.assertIsDisplayed
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
import com.hoverboard.remote.ui.screens.OVERFLOW_ICON_TAG
import com.hoverboard.remote.ui.screens.TOP_BAR_TAG
import com.hoverboard.remote.ui.screens.UP_ICON_TAG
import com.hoverboard.remote.ui.screens.menuItemTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode

/**
 * The navigation the tab row became (`specs/rider-ui.md` section 2a): Ride is the root, with the
 * other three pushed onto it from its overflow menu and returning with an up arrow.
 *
 * Three properties here are safety or model properties, not layout ones. Navigation must claim no
 * gesture at all, because the throttle pad and the joystick consume drags and a throttle that loses
 * its gesture mid-drive is a safety problem; a drawer opens on an edge swipe and so needed rules to
 * withhold it, and a menu behind a tap target has no gesture to withhold. A pushed destination
 * carries no menu, so there is no sideways hop between the three and the way out is back to Ride.
 * And the armed state used to be legible from the Ride screen's tinted surfaces, which the pushed
 * destinations do not show, so the top bar SURFACE carries the tint wherever the rider has
 * navigated: the icon it used to sit on is not there on a pushed destination.
 */
@RunWith(AndroidJUnit4::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class OverflowMenuTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    /** What a run can ask the app afterwards: how often the throttle was released, and system back. */
    private class Rig {
        var released = 0
        lateinit var back: OnBackPressedDispatcher
    }

    /** The connected app with placeholder destinations, starting at [start]. */
    private fun show(start: AppDestination = AppDestination.RIDE, armed: Boolean = false): Rig {
        val rig = Rig()
        compose.setContent {
            var destination by remember { mutableStateOf(start) }
            rig.back = checkNotNull(LocalOnBackPressedDispatcherOwner.current).onBackPressedDispatcher
            HoverboardRemoteTheme {
                ConnectedScreen(
                    destination = destination,
                    onDestination = { destination = it },
                    onLeaveRide = { rig.released++ },
                    armed = armed,
                    ride = { Text("RIDE-CONTENT") },
                    tune = { Text("TUNE-CONTENT") },
                    setup = { Text("SETUP-CONTENT") },
                    layout = { Text("LAYOUT-CONTENT") },
                )
            }
        }
        return rig
    }

    @Test
    fun navigationClaimsNoGestureOnRide() {
        show(AppDestination.RIDE)

        compose.onNodeWithText("RIDE-CONTENT").performTouchInput { swipeRight() }

        // Not merely shut: a menu is composed only while it is open, so there is nothing parked at
        // the edge for a swipe to drag in, and the pad keeps the gesture.
        compose.onNodeWithTag(menuItemTag(AppDestination.TUNE)).assertDoesNotExist()
        compose.onNodeWithText("RIDE-CONTENT").assertIsDisplayed()
    }

    @Test
    fun aPushedDestinationOffersAnUpArrowAndNoMenuAtAll() {
        show(AppDestination.TUNE)

        // No menu icon, so no sideways hop and nothing to open: the way out of Tune is back to Ride.
        compose.onNodeWithTag(OVERFLOW_ICON_TAG).assertDoesNotExist()
        compose.onNodeWithTag(menuItemTag(AppDestination.SETUP)).assertDoesNotExist()

        compose.onNodeWithTag(UP_ICON_TAG).assertIsDisplayed().performClick()
        compose.onNodeWithText("RIDE-CONTENT").assertIsDisplayed()
        compose.onNodeWithText("TUNE-CONTENT").assertDoesNotExist()
    }

    @Test
    fun theRootCarriesTheMenuAndNoUpArrow() {
        show(AppDestination.RIDE)

        // Material 3: a top-level destination shows no up button, because there is nothing above it.
        compose.onNodeWithTag(UP_ICON_TAG).assertDoesNotExist()

        compose.onNodeWithTag(OVERFLOW_ICON_TAG).performClick()
        // Three entries: Ride is where the menu IS, so it is not an item in it.
        compose.onNodeWithTag(menuItemTag(AppDestination.TUNE)).assertIsDisplayed()
        compose.onNodeWithTag(menuItemTag(AppDestination.SETUP)).assertIsDisplayed()
        compose.onNodeWithTag(menuItemTag(AppDestination.LAYOUT)).assertIsDisplayed()
        compose.onNodeWithTag(menuItemTag(AppDestination.RIDE)).assertDoesNotExist()
    }

    @Test
    fun everyPushLeavesRideAndReleasesTheThrottle() {
        val rig = show(AppDestination.RIDE)

        compose.onNodeWithTag(OVERFLOW_ICON_TAG).performClick()
        compose.onNodeWithTag(menuItemTag(AppDestination.TUNE)).performClick()
        compose.onNodeWithText("TUNE-CONTENT").assertIsDisplayed()
        compose.onNodeWithText("RIDE-CONTENT").assertDoesNotExist()
        compose.onNodeWithTag(menuItemTag(AppDestination.TUNE)).assertDoesNotExist()
        assertEquals(1, rig.released)

        // Back to Ride and out again is the only route to a second destination, and it releases
        // again: the pad was recomposed in between and could have been under a finger.
        compose.onNodeWithTag(UP_ICON_TAG).performClick()
        compose.onNodeWithTag(OVERFLOW_ICON_TAG).performClick()
        compose.onNodeWithTag(menuItemTag(AppDestination.SETUP)).performClick()
        compose.onNodeWithText("SETUP-CONTENT").assertIsDisplayed()
        assertEquals(2, rig.released)

        compose.onNodeWithTag(UP_ICON_TAG).performClick()
        compose.onNodeWithTag(OVERFLOW_ICON_TAG).performClick()
        compose.onNodeWithTag(menuItemTag(AppDestination.LAYOUT)).performClick()
        compose.onNodeWithText("LAYOUT-CONTENT").assertIsDisplayed()
        assertEquals(3, rig.released)
    }

    @Test
    fun systemBackPopsAPushedDestinationToRide() {
        val rig = show(AppDestination.SETUP)

        assertTrue("system back is not handled on a pushed destination", rig.back.hasEnabledCallbacks())
        compose.runOnUiThread { rig.back.onBackPressed() }

        compose.onNodeWithText("RIDE-CONTENT").assertIsDisplayed()
        compose.onNodeWithText("SETUP-CONTENT").assertDoesNotExist()
    }

    @Test
    fun systemBackOnRideIsLeftToTheSystem() {
        val rig = show(AppDestination.RIDE)

        // Unhandled, so back leaves the app as it did before there was a stack. Asserted as "no
        // callback wants it" rather than by pressing it, which would finish the test's own host.
        assertFalse("Ride swallows system back instead of leaving the app", rig.back.hasEnabledCallbacks())
    }

    @Test
    fun theTopBarSurfaceCarriesTheArmedTintOnAPushedDestination() {
        show(AppDestination.TUNE, armed = true)

        assertTrue("the armed tint is not on the top bar surface", armedPixels("armed") > 0)
        compose.onNodeWithContentDescription(s(R.string.nav_title_armed, s(R.string.nav_tune))).assertExists()
    }

    @Test
    fun theTopBarSurfaceCarriesNoArmedTintWhileDisarmed() {
        show(AppDestination.TUNE, armed = false)

        assertEquals(0, armedPixels("disarmed"))
        compose.onNodeWithText(s(R.string.nav_tune)).assertIsDisplayed()
    }

    /**
     * How many pixels of the top bar are drawn in the armed colour.
     *
     * Captured through Roborazzi rather than `captureToImage`, which syncs on a real window redraw
     * Robolectric never performs, so the capture times out instead of failing on the colour. These
     * are working captures under `build/`, not screenshot baselines.
     */
    private fun armedPixels(name: String): Int {
        val path = "build/test-captures/top_bar_$name.png"
        compose.onNodeWithTag(TOP_BAR_TAG).captureRoboImage(path)
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
     * Whether [argb] is `AccentRed` as drawn: the outline is antialiased over a dark background, so
     * its edge pixels are the tint at a partial alpha and only the interior matches exactly. Each
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
