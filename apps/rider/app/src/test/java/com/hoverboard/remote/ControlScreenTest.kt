package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertHasClickAction
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.longClick
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performTouchInput
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.components.ARM_TAG
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.SIM_RIDER_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the ride screen says about the arm state, and how many places it says it.
 *
 * The screen used to carry an ARMED banner as well as the arm toggle, both reading the same flag
 * off the same state. Deleting the banner is only correct if the toggle still carries the whole
 * message, so these tests pin the two properties that made the banner deletable: there is exactly
 * ONE armed statement on screen, and the control that carries it is always tappable to stop.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ControlScreenTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()

    @Test
    fun armedStateIsStatedExactlyOnce() {
        show(armed = true)

        // One surface, not two. Counting nodes rather than asserting the banner's absence is the
        // check that survives: a future second indicator would fail this without anyone having to
        // remember the banner existed.
        val armedStatements = compose
            .onAllNodes(hasText(ARMED_WORD, substring = true))
            .fetchSemanticsNodes()
        assertEquals(1, armedStatements.size)

        compose.onNodeWithText(context.getString(R.string.arm_disarm_action)).assertIsDisplayed()
    }

    @Test
    fun disarmedScreenMakesNoArmedStatement() {
        show(armed = false)

        assertEquals(
            0,
            compose.onAllNodes(hasText(ARMED_WORD, substring = true)).fetchSemanticsNodes().size,
        )
        compose.onNodeWithText(context.getString(R.string.arm_action)).assertIsDisplayed()
    }

    /**
     * The safety property the banner deletion must not have taken with it: the toggle is the only
     * stop control left, so it is tappable while armed even in the state that refuses ARMING (a
     * deflected throttle). A stop control that can be unavailable is not a stop control.
     */
    @Test
    fun disarmIsTappableEvenWhileArmingIsRefused() {
        var toggles = 0
        val state = UiState(
            connectionState = ConnectionState.CONNECTED,
            telemetry = null,
            armed = true,
            throttleSpeed = 4_000,
            engaged = true,
        )
        // Precondition for the test to mean anything: this is a state where ARMING is refused.
        assertTrue(!state.canArm)

        compose.setContent {
            HoverboardRemoteTheme {
                ControlScreen(
                    state = state,
                    onArmToggle = { toggles++ },
                    onThrottleMove = { _, _ -> },
                    onThrottleRelease = {},
                    onDisconnect = {},
                    onSimulateRider = {},
                    showSimulateRider = false,
                )
            }
        }

        compose.onNodeWithTag(ARM_TAG).assertHasClickAction().performClick()
        assertEquals(1, toggles)
    }

    /**
     * The bench affordance is a real control, not a constructor argument: the app stopped asserting
     * the `INPUTS` rider bit as a copy of the arm level, and this row is the only remaining way to
     * put it on the wire. A screen that carried the flag but no way to change it would leave a
     * padless bench board unable to engage at all.
     *
     * It takes a LONG press. On a board that has pads this is not a display preference: the folded
     * rider level is a term in the FSM's engage conjunction and its inverse enables the wind-down,
     * so asserting it holds a machine engaged through the moment a rider steps off. A control with
     * that authority, inches above the throttle, must not answer to a passing thumb.
     */
    @Test
    fun theSimulateRiderRowTogglesOnALongPressAndNotOnATap() {
        var asked: Boolean? = null
        showWithSimulateRider(onSimulateRider = { asked = it })

        compose.onNodeWithText(context.getString(R.string.sim_rider_off)).assertIsDisplayed()

        compose.onNodeWithTag(SIM_RIDER_TAG).performClick()
        assertNull("a tap must not flip a control that can hold a board engaged", asked)

        compose.onNodeWithTag(SIM_RIDER_TAG).performTouchInput { longClick() }
        assertEquals(true, asked)
    }

    /**
     * And a rider's build does not carry it at all: the call site passes `BuildConfig.DEBUG`. The
     * flag is a [ControlScreen] parameter rather than a read of `BuildConfig` inside the composable
     * so that the absence is a property of the screen, testable in the build where the control
     * exists.
     */
    @Test
    fun theSimulateRiderRowIsAbsentUnlessTheBuildEnablesIt() {
        show(armed = false)

        compose.onNodeWithTag(SIM_RIDER_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.sim_rider_label)).assertDoesNotExist()
    }

    private fun showWithSimulateRider(onSimulateRider: (Boolean) -> Unit) = compose.setContent {
        HoverboardRemoteTheme {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = TelemetryUi(),
                    simulateRider = false,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = onSimulateRider,
                showSimulateRider = true,
            )
        }
    }

    private fun show(armed: Boolean) = compose.setContent {
        HoverboardRemoteTheme {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = TelemetryUi(),
                    armed = armed,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = {},
                showSimulateRider = false,
            )
        }
    }

    private companion object {
        /** The word every armed indicator this screen has ever had was built around. */
        const val ARMED_WORD = "ARMED"
    }
}
