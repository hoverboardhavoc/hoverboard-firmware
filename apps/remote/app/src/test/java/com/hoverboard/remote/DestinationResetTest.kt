package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.remote.ui.screens.AppDestination
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * A session that ends takes the app back to Ride.
 *
 * The destination survived a link drop, so a drop while the owner was on Setup or in the layout
 * editor reconnected straight back into an editor, with the machine live one screen away and
 * nothing saying the session had restarted. The view model already drops the arm level across a
 * reconnect for that reason, and the destination is the same kind of state.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class DestinationResetTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    @Test
    fun aLinkDropTakesTheAppBackToRideAndTheReconnectStartsThere() {
        val connected = mutableStateOf(true)
        var destination: MutableState<AppDestination>? = null
        compose.setContent { destination = rememberDestination(connected.value) }
        val where = checkNotNull(destination)

        where.value = AppDestination.SETUP
        compose.waitForIdle()
        assertEquals("a push while the link is up stands", AppDestination.SETUP, where.value)

        connected.value = false
        compose.waitForIdle()
        assertEquals("the session ended, so the app is back at the root", AppDestination.RIDE, where.value)

        connected.value = true
        compose.waitForIdle()
        assertEquals("and the new session starts where the machine is live", AppDestination.RIDE, where.value)
    }

    /** A disconnected start is the same case: the restored destination is not where a session begins. */
    @Test
    fun anAppThatStartsDisconnectedIsOnRide() {
        val connected = mutableStateOf(false)
        var destination: MutableState<AppDestination>? = null
        compose.setContent { destination = rememberDestination(connected.value) }

        compose.waitForIdle()
        assertEquals(AppDestination.RIDE, checkNotNull(destination).value)
    }
}
