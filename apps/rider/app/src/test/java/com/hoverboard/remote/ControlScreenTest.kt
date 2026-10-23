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
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.components.ARM_TAG
import com.hoverboard.remote.ui.components.JOYSTICK_TAG
import com.hoverboard.remote.ui.components.THROTTLE_TAG
import com.hoverboard.remote.ui.screens.BOARD_CHIP_MACHINE_TAG
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.RIGID_NOT_LEADING_TAG
import com.hoverboard.remote.ui.screens.SIM_RIDER_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
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
     * The modes with a turn axis get the two-axis stick; the ones that send one value per board
     * keep the throttle pad. Exactly one control is on screen, so there is never a second surface a
     * thumb could be on.
     */
    @Test
    fun theJoystickIsTheControlInTheModesThatSteerAndTheThrottlePadInTheOthers() {
        show(ride(armed = true, mode = DriveMode.SINGLE))
        for (mode in DriveMode.entries) {
            update(ride(armed = true, mode = mode))
            val steers = mode.hasTurnAxis
            compose.onNodeWithTag(if (steers) JOYSTICK_TAG else THROTTLE_TAG).assertIsDisplayed()
            compose.onNodeWithTag(if (steers) THROTTLE_TAG else JOYSTICK_TAG).assertDoesNotExist()
        }
        assertTrue("DIFFERENTIAL and RIGID steer", DriveMode.entries.count { it.hasTurnAxis } == 2)
    }

    /**
     * The rigid mode names a board by its role, so it is offered only once a board reports itself
     * the leader (`specs/rover-one-imu.md` section 7). A pair with a slave but no leader is an
     * ordinary pivoting pair as far as this chip is concerned.
     */
    @Test
    fun theRigidModeIsOfferedOnlyOnceABoardReportsLeading() {
        val rigid = context.getString(R.string.drive_mode_rigid)
        show(ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02))
        compose.onNodeWithText(rigid).assertIsNotEnabled()

        update(ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02).copy(telemetry = leading()))
        compose.onNodeWithText(rigid).assertIsEnabled()

        // Both boards claiming the lead is the staging error, not a pair to drive.
        update(
            ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02)
                .copy(telemetry = leading(), slaveTelemetry = leading()),
        )
        compose.onNodeWithText(rigid).assertIsNotEnabled()
    }

    /**
     * The greyed chip gets a sentence exactly once: when the pair is STORED as one rigid machine
     * and neither board is running as its leader. A pivoting pair is told nothing.
     */
    @Test
    fun aRigidPairThatIsNotLeadingIsToldWhy() {
        show(ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02))
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertDoesNotExist()

        val configured = ride(armed = false, mode = DriveMode.SINGLE, slave = 0x02)
            .copy(machine = MachinePair(FramePair.Agree(rigid = true), SidePair.OneRight(Node.MASTER)))
        update(configured)
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertIsDisplayed()

        update(configured.copy(telemetry = leading()))
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertDoesNotExist()
    }

    /** Each mode's armed hint says what the stick does in it, and the rigid one says lean. */
    @Test
    fun theRigidModeSaysWhatTheStickDoes() {
        show(ride(armed = true, mode = DriveMode.RIGID, slave = 0x02).copy(telemetry = leading()))
        compose.onNodeWithText(context.getString(R.string.throttle_hint_armed_rigid)).assertIsDisplayed()
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

    /**
     * Each chip says what ITS board says. The screen is given a second board that is running while
     * the attached one sits at rest, which is a pair the old chip could not express at all: it read
     * the app's own drive mode, so it would have called the second board driven or not driven
     * without either board having said anything.
     */
    @Test
    fun eachBoardsChipReportsThatBoardsOwnState() {
        show(
            ride(armed = true, mode = DriveMode.DIFFERENTIAL, slave = 0x02)
                .copy(telemetry = reporting(MODE_OFF), slaveTelemetry = reporting(MODE_RUN)),
        )
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_off))
            .assertIsDisplayed()
        compose.onNodeWithText(chip(R.string.board_chip_slave, "0x02", R.string.board_state_live))
            .assertIsDisplayed()
    }

    /** A board the app has heard nothing from is silent, not idle: the chip claims no state for it. */
    @Test
    fun aBoardThatHasNotReportedIsSilent() {
        show(ride(armed = false, mode = DriveMode.BOUND, slave = 0x02).copy(telemetry = reporting(MODE_RUN)))
        compose.onNodeWithText(chip(R.string.board_chip_slave, "0x02", R.string.board_state_quiet))
            .assertIsDisplayed()
    }

    /** A fault on one board is on that board's chip and nowhere else. */
    @Test
    fun aFaultOnTheSlaveStaysOnTheSlavesChip() {
        show(
            ride(armed = true, mode = DriveMode.BOUND, slave = 0x02).copy(
                telemetry = reporting(MODE_RUN),
                slaveTelemetry = reporting(MODE_RUN, fault = 0x01),
            ),
        )
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_live))
            .assertIsDisplayed()
        compose.onNodeWithText(chip(R.string.board_chip_slave, "0x02", R.string.board_state_fault))
            .assertIsDisplayed()
    }

    /**
     * Every state a board can report has a word on the chip, including the ones only a moment of a
     * boot or a shutdown shows, and a mode byte from an image this build does not know.
     */
    @Test
    fun theChipNamesEveryStateTheBoardCanReport() {
        val ride = ride(armed = false, mode = DriveMode.SINGLE)
        show(ride.copy(telemetry = reporting(MODE_INIT)))
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_starting))
            .assertIsDisplayed()

        update(ride.copy(telemetry = reporting(MODE_SHUTDOWN)))
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_stopping))
            .assertIsDisplayed()

        update(ride.copy(telemetry = reporting(MODE_UNALLOCATED)))
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_unrecognised))
            .assertIsDisplayed()

        update(ride.copy(telemetry = reporting(MODE_RUN, flags = CyclicState.FLAG_LOCKDOWN)))
        compose.onNodeWithText(chip(R.string.board_chip_master, "0x01", R.string.board_state_lockdown))
            .assertIsDisplayed()
    }

    /** One board's chip text: the address, then what that board says. */
    private fun chip(chip: Int, address: String, state: Int) =
        context.getString(chip, address, context.getString(state))

    /** A board reporting [mode], with [fault] as its producer level mask and [flags] as its flags. */
    private fun reporting(mode: Int, fault: Int = 0, flags: Int = 0) = TelemetryUi().merge(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = 3_780,
            mode = mode,
            fault = fault,
            flags = flags,
            peerDemand = 0,
            obs = QUIET_OBS,
        ),
    )

    /** A board reporting itself the rigid frame's leader. */
    private fun leading() = reporting(MODE_OFF, flags = CyclicState.FLAG_LEADER)

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

    private fun show(
        armed: Boolean,
        riderWaiver: RiderWaiver = RiderWaiver.NONE,
        machine: MachinePair = MachinePair.UNKNOWN,
        boards: Pair<Int?, Int?> = null to null,
    ) = compose.setContent {
        HoverboardRemoteTheme {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = TelemetryUi(),
                    armed = armed,
                    riderWaiver = riderWaiver,
                    machine = machine,
                    masterBoard = boards.first,
                    slaveBoard = boards.second,
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

    // --- the rover pair (`specs/rider-ui.md` 3.7a) ---------------------------------------------

    /**
     * A pair holding two different frames: the arm control refuses, names both values, and the
     * badge says so beside the board chips, before the control is ever tapped.
     */
    @Test
    fun aPairWithTwoDifferentFramesRefusesTheArmAndBadgesTheChips() {
        show(
            armed = false,
            machine = MachinePair(FramePair.Differ(masterRigid = true), SidePair.NoneRight),
            boards = 0x01 to 0x02,
        )

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.board_chip_frame_differs)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.arm_blocked)).assertIsDisplayed()
        val statement = context.getString(
            R.string.arm_frames_differ,
            context.getString(R.string.setup_choice_frame_rigid),
            context.getString(R.string.setup_choice_frame_pivoting),
        )
        compose.onNodeWithText(statement).assertIsDisplayed()
    }

    /** The side warns and never refuses: the control still reads "tap to arm". */
    @Test
    fun aRigidPairWithNoRightWheelWarnsAndStillOffersTheArm() {
        show(
            armed = false,
            machine = MachinePair(FramePair.Agree(rigid = true), SidePair.NoneRight),
            boards = 0x01 to 0x02,
        )

        compose.onNodeWithText(context.getString(R.string.arm_action)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.arm_side_none_right)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.board_chip_no_right)).assertIsDisplayed()
    }

    /** A fresh pivoting pair is the factory state: no badge, nothing on the arm control. */
    @Test
    fun aFreshPivotingPairSaysNothingAboutTheMachine() {
        show(
            armed = false,
            machine = MachinePair(FramePair.Agree(rigid = false), SidePair.NoneRight),
            boards = 0x01 to 0x02,
        )

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.arm_action)).assertIsDisplayed()
    }

    /** No second board found: a warning that says to power it, and no badge, because nothing is known. */
    @Test
    fun aSessionWithOneBoardWarnsThatTheOtherHalfIsUnknown() {
        show(armed = false, machine = MachinePair.UNKNOWN, boards = 0x01 to null)

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.arm_frame_no_peer)).assertIsDisplayed()
    }

    /** A second board that was found but did not answer: the remedy is reading it again. */
    @Test
    fun aFrameThatDidNotReadWarnsToReadAgain() {
        show(
            armed = false,
            machine = MachinePair(FramePair.Unknown, SidePair.Unknown, peerFound = true),
            boards = 0x01 to 0x02,
        )

        compose.onNodeWithText(context.getString(R.string.arm_frame_unread)).assertIsDisplayed()
    }

    private companion object {
        /** The word every armed indicator this screen has ever had was built around. */
        const val ARMED_WORD = "ARMED"
    }
}

/** `CYCLIC_STATE.mode` as the firmware's mode machine emits it (`crates/state/src/mode.rs`). */
private const val MODE_OFF = 0
private const val MODE_INIT = 1
private const val MODE_RUN = 3
private const val MODE_SHUTDOWN = 4

/** A mode byte the firmware's machine never emits, which is what a newer image would look like. */
private const val MODE_UNALLOCATED = 9
