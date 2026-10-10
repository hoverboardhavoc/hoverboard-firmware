package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.test.assertHasClickAction
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.down
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.longClick
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performTouchInput
import androidx.compose.ui.test.up
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.components.ARM_TAG
import com.hoverboard.remote.ui.components.JOYSTICK_TAG
import com.hoverboard.remote.ui.components.THROTTLE_TAG
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.SIM_RIDER_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.rules.RuleChain
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

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

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
                    onJoystickMove = { _, _, _, _ -> },
                    onThrottleRelease = {},
                    onDisconnect = {},
                    onSimulateRider = {},
                    onDriveMode = {},
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
    /** `specs/control.md` (i): the arm control says the rider requirement is waived when it is. */
    @Test
    fun theArmControlSaysTheRiderRequirementIsWaivedOnlyWhenItIs() {
        show(armed = false, riderWaiver = RiderWaiver.WAIVED)
        // The arm control merges its texts into its own node, so the line is a text OF that node.
        compose.onNode(hasTestTag(ARM_TAG) and hasText(context.getString(R.string.arm_rider_waived)))
            .assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.arm_rider_maybe_waived)).assertDoesNotExist()
    }

    /**
     * When the app cannot tell what the board runs (a write this session, then a link drop) and the
     * value it last ran or stores is 0, the arm control says the requirement MAY be waived: true
     * whichever way the board turns out, and never claiming the certainty of the plain line.
     */
    @Test
    fun theArmControlSaysTheRiderRequirementMayBeWaivedWhenItCannotTell() {
        show(armed = false, riderWaiver = RiderWaiver.POSSIBLY)
        compose.onNode(hasTestTag(ARM_TAG) and hasText(context.getString(R.string.arm_rider_maybe_waived)))
            .assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.arm_rider_waived)).assertDoesNotExist()
        assertTrue(context.getString(R.string.arm_rider_maybe_waived).contains("may be waived"))
    }

    @Test
    fun aBoardThatRequiresARiderGetsNoWaiverLine() {
        show(armed = false)
        compose.onNodeWithText(context.getString(R.string.arm_rider_waived)).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.arm_rider_maybe_waived)).assertDoesNotExist()
    }

    @Test
    fun theSimulateRiderRowIsAbsentUnlessTheBuildEnablesIt() {
        show(armed = false)

        compose.onNodeWithTag(SIM_RIDER_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.sim_rider_label)).assertDoesNotExist()
    }

    // --- the drive control, per mode (`specs/rider-ui.md` 3.2) ---------------------------------

    /**
     * DIFFERENTIAL is the mode with a turn axis, so it is the one with the two-axis stick. The other
     * two send one value per board and keep the throttle pad. Exactly one control is on screen, so
     * there is never a second surface a thumb could be on.
     */
    @Test
    fun theJoystickIsTheControlInDifferentialAndTheThrottlePadInTheOthers() {
        show(ride(armed = true, mode = DriveMode.SINGLE))
        for (mode in DriveMode.entries) {
            update(ride(armed = true, mode = mode))
            val differential = mode == DriveMode.DIFFERENTIAL
            compose.onNodeWithTag(if (differential) JOYSTICK_TAG else THROTTLE_TAG).assertIsDisplayed()
            compose.onNodeWithTag(if (differential) THROTTLE_TAG else JOYSTICK_TAG).assertDoesNotExist()
        }
    }

    /** The stick reports raw coordinates and its pad's size, and finger-up is a release. */
    @Test
    fun theJoystickReportsBothAxesAndReleasesOnFingerUp() {
        var reported: List<Float>? = null
        var releases = 0
        show(
            state = ride(armed = true, mode = DriveMode.DIFFERENTIAL),
            onJoystickMove = { x, y, width, height -> reported = listOf(x, y, width, height) },
            onThrottleRelease = { releases++ },
        )

        compose.onNodeWithTag(JOYSTICK_TAG).performTouchInput {
            down(Offset(30f, 40f))
            up()
        }

        val move = checkNotNull(reported) { "touch-down reports before any movement, as the pad does" }
        assertEquals(30f, move[0])
        assertEquals(40f, move[1])
        assertTrue("the pad reports its own size", move[2] > 0f && move[3] > 0f)
        assertEquals(1, releases)
    }

    /**
     * A mode that drives the slave is not offered until the session has one. The chip and
     * [MainViewModel.setDriveMode] apply the same gate, so a tap cannot reach a mode the row shows as
     * unavailable.
     */
    @Test
    fun aModeThatDrivesTheSlaveIsOfferedOnlyOnceOneIsFound() {
        show(ride(armed = false, mode = DriveMode.SINGLE))
        compose.onNodeWithText(context.getString(R.string.drive_mode_differential)).assertIsNotEnabled()

        update(ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02))
        compose.onNodeWithText(context.getString(R.string.drive_mode_differential)).assertIsEnabled()

        // Armed: the stick must not change meaning mid-drive, so no chip takes a tap.
        update(ride(armed = true, mode = DriveMode.SINGLE, slave = 0x02))
        compose.onNodeWithText(context.getString(R.string.drive_mode_differential)).assertIsNotEnabled()
        compose.onNodeWithText(context.getString(R.string.drive_mode_bound)).assertIsNotEnabled()
    }

    /** The slave's chip says it is being driven in DIFFERENTIAL too, where it has its own wheel. */
    @Test
    fun theSlaveChipSaysItIsCommandedInDifferential() {
        show(ride(armed = true, mode = DriveMode.DIFFERENTIAL, slave = 0x02))
        compose.onNodeWithText(context.getString(R.string.board_chip_slave_commanded_armed, "0x02"))
            .assertIsDisplayed()
    }

    /** A connected ride screen in [mode]. */
    private fun ride(armed: Boolean, mode: DriveMode, slave: Int? = null) = UiState(
        connectionState = ConnectionState.CONNECTED,
        telemetry = TelemetryUi(),
        armed = armed,
        driveMode = mode,
        masterBoard = 0x01,
        slaveBoard = slave,
    )

    /**
     * The state the ride screen below is composed from. A `setContent` is allowed once per test, so a
     * test that walks several states writes them here and recomposes rather than showing the screen
     * again.
     */
    private val rideState = mutableStateOf(UiState())

    private fun show(
        state: UiState,
        onJoystickMove: (Float, Float, Float, Float) -> Unit = { _, _, _, _ -> },
        onThrottleRelease: () -> Unit = {},
    ) {
        rideState.value = state
        compose.setContent {
            HoverboardRemoteTheme {
                ControlScreen(
                    state = rideState.value,
                    onArmToggle = {},
                    onThrottleMove = { _, _ -> },
                    onJoystickMove = onJoystickMove,
                    onThrottleRelease = onThrottleRelease,
                    onDisconnect = {},
                    onSimulateRider = {},
                    onDriveMode = {},
                    showSimulateRider = false,
                )
            }
        }
    }

    /** Put the shown screen in [state] and let it recompose. */
    private fun update(state: UiState) {
        rideState.value = state
        compose.waitForIdle()
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
                onJoystickMove = { _, _, _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = onSimulateRider,
                onDriveMode = {},
                showSimulateRider = true,
            )
        }
    }

    private fun show(armed: Boolean, riderWaiver: RiderWaiver = RiderWaiver.NONE) = compose.setContent {
        HoverboardRemoteTheme {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = TelemetryUi(),
                    armed = armed,
                    riderWaiver = riderWaiver,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onJoystickMove = { _, _, _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = {},
                onDriveMode = {},
                showSimulateRider = false,
            )
        }
    }

    private companion object {
        /** The word every armed indicator this screen has ever had was built around. */
        const val ARMED_WORD = "ARMED"
    }
}
