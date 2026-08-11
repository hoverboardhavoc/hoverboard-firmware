package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the connect screen says while the L3 attach is running and after it fails.
 *
 * Both states are reachable on every connect (the attach runs before anything is drivable), and
 * both were rendered by no test at all: their strings existed and were never shown to anything.
 * A state that falls through to another state's label, or a failure that renders as the idle
 * screen, is a silent connect that leaves the rider with no diagnosis.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ConnectScreenTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()

    @Test
    fun attachingSaysSoAndOffersToStop() {
        show(ConnectionState.ATTACHING)

        compose.onNodeWithText(context.getString(R.string.connect_status_attaching)).assertIsDisplayed()
        // Attaching is a busy state, not an idle one: the action is to stop, not to scan again.
        compose.onNodeWithText(context.getString(R.string.connect_stop_scan)).assertIsDisplayed()
    }

    @Test
    fun attachFailedSaysWhyAndOffersToScanAgain() {
        show(ConnectionState.ATTACH_FAILED)

        compose.onNodeWithText(context.getString(R.string.connect_status_attach_failed)).assertIsDisplayed()
        // Terminal for this attempt: the app stopped retrying, so the user gets the button back.
        compose.onNodeWithText(context.getString(R.string.connect_scan, DEVICE_NAME)).assertIsDisplayed()
    }

    private fun show(state: ConnectionState) = compose.setContent {
        HoverboardRemoteTheme {
            ConnectScreen(
                connectionState = state,
                deviceName = DEVICE_NAME,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    private companion object {
        const val DEVICE_NAME = "Hoverboard"
    }
}
