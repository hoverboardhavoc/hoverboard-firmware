package com.hoverboard.remote

import android.app.Application
import android.view.View
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * The display is held on while a board is attached.
 *
 * This is the counterpart to the `ON_STOP` disarm, and the reason that disarm can be trusted to
 * mean what it says. A display timing out is an `ON_STOP`, so without this the app disarmed every
 * machine it was connected to after the phone's screen timeout, which was measured three times at
 * 300, 306 and 307 seconds (`specs/ble-reliability.md`, final section). The flag is read back off
 * the view the composition runs in, which is the thing the window takes its own
 * `FLAG_KEEP_SCREEN_ON` from (`ViewRootImpl.applyKeepScreenOnFlag`).
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class KeepScreenOnTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    @Test
    fun theDisplayIsHeldOnWhileConnectedAndReleasedWhenTheLinkGoes() {
        val connected = mutableStateOf(false)
        var view: View? = null
        compose.setContent {
            view = LocalView.current
            KeepScreenOnWhileConnected(connected.value)
        }

        val host = checkNotNull(view)
        assertFalse("nothing is attached, so nothing holds the screen", host.keepScreenOn)

        connected.value = true
        compose.waitForIdle()
        assertTrue("a board is attached", host.keepScreenOn)

        connected.value = false
        compose.waitForIdle()
        assertFalse("the link went, so the phone may sleep again", host.keepScreenOn)
    }

    /**
     * And it is released when the composable leaves, not only when it is told the link went. A
     * hold that outlives its owner is a phone whose screen never sleeps again.
     */
    @Test
    fun theHoldIsReleasedWhenTheComposableLeaves() {
        val present = mutableStateOf(true)
        var view: View? = null
        compose.setContent {
            view = LocalView.current
            if (present.value) KeepScreenOnWhileConnected(connected = true)
        }

        val host = checkNotNull(view)
        assertTrue(host.keepScreenOn)

        present.value = false
        compose.waitForIdle()
        assertFalse(host.keepScreenOn)
    }
}
