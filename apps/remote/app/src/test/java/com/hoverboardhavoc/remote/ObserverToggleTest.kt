package com.hoverboardhavoc.remote

import android.app.Application
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertIsNotSelected
import androidx.compose.ui.test.assertIsSelected
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboardhavoc.remote.model.ConnectionState
import com.hoverboardhavoc.remote.ui.components.OBSERVER_TAG
import com.hoverboardhavoc.remote.ui.components.ObserverToggle
import com.hoverboardhavoc.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * The OBS button beside the arm control (`specs/remote-ui.md` 3.1, "The observer button"): when
 * [UiState.canObserve] enables it, and the button's three states (disabled, enabled and unlit,
 * lit).
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ObserverToggleTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()

    /** Enabled only while connected, armed and on a FOC machine. */
    @Test
    fun enabledOnlyArmedOnAFocMachine() {
        val armed = UiState(connectionState = ConnectionState.CONNECTED, armed = true, focMachine = true)
        assertTrue(armed.canObserve)
        assertFalse(armed.copy(armed = false).canObserve)
        assertFalse(armed.copy(focMachine = false).canObserve)
        assertFalse(armed.copy(connectionState = ConnectionState.DISCONNECTED).canObserve)
    }

    /** Disabled, it shows and does not tap; enabled and unlit, it taps; lit, it taps off. */
    @Test
    fun theThreeStates() {
        var taps = 0
        val on = mutableStateOf(false)
        val enabled = mutableStateOf(false)
        compose.setContent {
            HoverboardRemoteTheme {
                ObserverToggle(on = on.value, enabled = enabled.value, onToggle = { taps++ })
            }
        }
        compose.onNodeWithTag(OBSERVER_TAG).assertIsDisplayed().assertIsNotEnabled().assertIsNotSelected()
        compose.onNodeWithText(context.getString(R.string.observer_action)).assertIsDisplayed()
        compose.onNodeWithTag(OBSERVER_TAG).performClick()
        assertEquals("a disabled button does not tap", 0, taps)

        enabled.value = true
        compose.onNodeWithTag(OBSERVER_TAG).assertIsEnabled().assertIsNotSelected().performClick()
        assertEquals(1, taps)

        on.value = true
        compose.onNodeWithTag(OBSERVER_TAG).assertIsEnabled().assertIsSelected().performClick()
        assertEquals("lit, it taps off", 2, taps)
    }
}
