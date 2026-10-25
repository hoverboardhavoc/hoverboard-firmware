package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.test.assertHasClickAction
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertTextEquals
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
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.DriveFallback
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.components.ARM_TAG
import com.hoverboard.remote.ui.components.JOYSTICK_TAG
import com.hoverboard.remote.ui.components.THROTTLE_TAG
import com.hoverboard.remote.ui.screens.BOARD_CHIP_MACHINE_TAG
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.DRIVE_CHIP_TAG
import com.hoverboard.remote.ui.screens.DRIVE_FALLBACK_TAG
import com.hoverboard.remote.ui.screens.FRAME_STATS_TAG
import com.hoverboard.remote.ui.screens.boardChipTag
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

    /**
     * The one disarm the rider did not ask for says so, on the control that carries every other
     * thing this screen knows about the arm state. A machine that is not armed any more otherwise
     * looks the same whether the app did it, the board faulted or the link went, and the rider
     * goes looking at the machine for a cause that is not there (`specs/ble-reliability.md`, final
     * section).
     */
    @Test
    fun theArmControlSaysWhenTheAppDisarmedTheMachineItself() {
        show(ride(armed = false, mode = DriveMode.SINGLE).copy(selfDisarmed = true))

        compose.onNode(hasTestTag(ARM_TAG) and hasText(context.getString(R.string.arm_self_disarmed)))
            .assertIsDisplayed()
        // It is a report, not a refusal: the control still offers the arm it is explaining the
        // absence of.
        compose.onNodeWithText(context.getString(R.string.arm_action)).assertIsDisplayed()

        // And it is answered by the arm that makes it stale.
        update(ride(armed = true, mode = DriveMode.SINGLE))
        compose.onNodeWithText(context.getString(R.string.arm_self_disarmed)).assertDoesNotExist()
    }

    /** A machine the rider disarmed, or never armed, gets no such line. */
    @Test
    fun anOrdinaryDisarmedScreenClaimsNoSelfDisarm() {
        show(armed = false)

        compose.onNodeWithText(context.getString(R.string.arm_self_disarmed)).assertDoesNotExist()
    }

    /**
     * With no motor controller answered yet (or only a bridge found) there is nothing to arm, and
     * the control says that rather than blaming a throttle nobody is holding.
     */
    @Test
    fun withNoBoardToArmTheControlSaysSo() {
        show(armed = false, boards = emptyList())

        compose.onNodeWithText(context.getString(R.string.arm_no_board)).assertExists()
        compose.onNodeWithText(context.getString(R.string.arm_unavailable)).assertDoesNotExist()
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
     * The Ride screen offers no drive-mode selector any more: the mode is the machine's stored
     * setting, edited under Machine on Setup (`specs/rider-ui.md` 3.2). The strip names the mode in
     * force as a read-only chip, and while the stored mode is the one in force it says nothing more.
     */
    @Test
    fun theStripNamesTheModeInForceAndOffersNoSelector() {
        show(ride(armed = false, mode = DriveMode.BOUND, other = 0x02))

        compose.onNodeWithTag(DRIVE_CHIP_TAG).assertIsDisplayed()
        compose.onNodeWithText(driveChip(R.string.drive_mode_bound)).assertIsDisplayed()
        compose.onNodeWithTag(DRIVE_FALLBACK_TAG).assertDoesNotExist()
        // No chip offers another mode: none of the other labels is on the screen.
        for (label in listOf(R.string.drive_mode_single, R.string.drive_mode_differential, R.string.drive_mode_rigid)) {
            compose.onNodeWithText(context.getString(label), substring = true).assertDoesNotExist()
        }
    }

    /**
     * A stored mode that cannot run on this machine falls back to SINGLE, and the strip says which
     * mode the machine stores and why it is not in force (`specs/rider-ui.md` 3.2).
     */
    @Test
    fun aFallbackToSingleSaysWhy() {
        val rigid = context.getString(R.string.drive_mode_rigid)
        show(
            ride(armed = false, mode = DriveMode.SINGLE, other = 0x02)
                .copy(drive = DriveInForce(single(), stored = DriveMode.RIGID, fallback = DriveFallback.NO_LEADER)),
        )

        compose.onNodeWithText(driveChip(R.string.drive_mode_single)).assertIsDisplayed()
        compose.onNodeWithTag(DRIVE_FALLBACK_TAG).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.drive_fallback_no_leader, rigid)).assertIsDisplayed()

        update(
            ride(armed = false, mode = DriveMode.SINGLE, other = 0x02)
                .copy(drive = DriveInForce(single(), stored = null, fallback = DriveFallback.DISAGREE)),
        )
        compose.onNodeWithText(context.getString(R.string.drive_fallback_disagree)).assertIsDisplayed()
    }

    /**
     * The rigid sentence is said exactly once: when the machine is STORED as one rigid machine and
     * no board is running as its leader. A pivoting machine is told nothing.
     */
    @Test
    fun aRigidPairThatIsNotLeadingIsToldWhy() {
        show(ride(armed = false, mode = DriveMode.SINGLE, other = 0x02))
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertDoesNotExist()

        val configured = ride(armed = false, mode = DriveMode.SINGLE, other = 0x02)
            .copy(machine = Machine(FramePair.Agree(rigid = true), SidePair.OneRight(0x01), ModePair.Unknown))
        update(configured)
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertIsDisplayed()

        update(configured.reporting(0x01, leading()))
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertDoesNotExist()
    }

    /** Each mode's armed hint says what the stick does in it, and the rigid one says lean. */
    @Test
    fun theRigidModeSaysWhatTheStickDoes() {
        show(ride(armed = true, mode = DriveMode.RIGID, other = 0x02).reporting(0x01, leading()))
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
     * Each chip says what ITS board says, under that board's label and tag. The screen is given a
     * second board that is running while the first sits at rest.
     */
    @Test
    fun eachBoardsChipReportsThatBoardsOwnState() {
        show(
            ride(armed = true, mode = DriveMode.DIFFERENTIAL, other = 0x02)
                .reporting(0x01, reporting(MODE_OFF)).reporting(0x02, reporting(MODE_RUN)),
        )
        compose.onNodeWithText(chip(0x01, R.string.board_state_off)).assertIsDisplayed()
        compose.onNodeWithText(chip(0x02, R.string.board_state_live)).assertIsDisplayed()
        compose.onNode(hasTestTag(boardChipTag(0x02)) and hasText(chip(0x02, R.string.board_state_live)))
            .assertIsDisplayed()
    }

    /** A board the app has heard nothing from is silent, not idle: the chip claims no state for it. */
    @Test
    fun aBoardThatHasNotReportedIsSilent() {
        show(ride(armed = false, mode = DriveMode.BOUND, other = 0x02).reporting(0x01, reporting(MODE_RUN)))
        compose.onNodeWithText(chip(0x02, R.string.board_state_quiet)).assertIsDisplayed()
    }

    /** A fault on one board is on that board's chip and nowhere else. */
    @Test
    fun aFaultOnOneBoardStaysOnThatBoardsChip() {
        show(
            ride(armed = true, mode = DriveMode.BOUND, other = 0x02)
                .reporting(0x01, reporting(MODE_RUN))
                .reporting(0x02, reporting(MODE_RUN, fault = 0x01)),
        )
        compose.onNodeWithText(chip(0x01, R.string.board_state_live)).assertIsDisplayed()
        compose.onNodeWithText(chip(0x02, R.string.board_state_fault)).assertIsDisplayed()
    }

    /** A bridge drives nothing and reports nothing, so the strip has no chip for it. */
    @Test
    fun aBridgeGetsNoChip() {
        val nodes = listOf(
            NodeInfo(0x01, Fields.NodeClass.BRIDGE, "hb-esp-1a2b"),
            NodeInfo(0x02, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME),
        )
        show(
            ride(armed = false, mode = DriveMode.SINGLE)
                .copy(nodes = nodes, selected = 0x02, telemetryByBoard = mapOf(0x02 to reporting(MODE_OFF))),
        )
        compose.onNodeWithTag(boardChipTag(0x01)).assertDoesNotExist()
        compose.onNodeWithText(chip(0x02, R.string.board_state_off)).assertIsDisplayed()
    }

    /**
     * Every state a board can report has a word on the chip, including the ones only a moment of a
     * boot or a shutdown shows, and a mode byte from an image this build does not know.
     */
    @Test
    fun theChipNamesEveryStateTheBoardCanReport() {
        val ride = ride(armed = false, mode = DriveMode.SINGLE)
        show(ride.reporting(0x01, reporting(MODE_INIT)))
        compose.onNodeWithText(chip(0x01, R.string.board_state_starting))
            .assertIsDisplayed()

        update(ride.reporting(0x01, reporting(MODE_SHUTDOWN)))
        compose.onNodeWithText(chip(0x01, R.string.board_state_stopping))
            .assertIsDisplayed()

        update(ride.reporting(0x01, reporting(MODE_UNALLOCATED)))
        compose.onNodeWithText(chip(0x01, R.string.board_state_unrecognised))
            .assertIsDisplayed()

        update(ride.reporting(0x01, reporting(MODE_RUN, flags = CyclicState.FLAG_LOCKDOWN)))
        compose.onNodeWithText(chip(0x01, R.string.board_state_lockdown))
            .assertIsDisplayed()
    }

    /** One board's chip text: its label (name and address), then what that board says. */
    private fun chip(address: Int, state: Int) =
        context.getString(R.string.board_chip, "$BOARD_NAME 0x%02X".format(address), context.getString(state))

    /** The read-only drive chip's text for the mode labelled [label]. */
    private fun driveChip(label: Int) = context.getString(R.string.drive_chip, context.getString(label))

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

    /**
     * A connected ride screen attached to 0x01, with [other] found by the walk, every node a motor
     * controller and 0x01 selected, driving in [mode] as the mode in force. 0x01 has reported once
     * (an empty reading), so the panel draws its rows.
     */
    private fun ride(armed: Boolean, mode: DriveMode, other: Int? = null): UiState {
        val boards = listOfNotNull(0x01, other)
        val targets = when (mode) {
            DriveMode.SINGLE -> single()
            DriveMode.DIFFERENTIAL -> DriveTargets(mode, listOf(0x01, 0x02), right = 0x01)
            else -> DriveTargets(mode, listOf(0x01, 0x02))
        }
        return UiState(
            connectionState = ConnectionState.CONNECTED,
            armed = armed,
            nodes = boards.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME) },
            attachedNode = 0x01,
            selected = 0x01,
            drive = DriveInForce(targets, stored = mode, fallback = null),
            driving = targets,
            telemetryByBoard = mapOf(0x01 to TelemetryUi()),
        )
    }

    /** SINGLE on 0x01. */
    private fun single() = DriveTargets(DriveMode.SINGLE, listOf(0x01))

    /** This state with [board] having last reported [telemetry]. */
    private fun UiState.reporting(board: Int, telemetry: TelemetryUi) =
        copy(telemetryByBoard = telemetryByBoard + (board to telemetry))

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
                state = UiState(connectionState = ConnectionState.CONNECTED, simulateRider = false),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onJoystickMove = { _, _, _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = onSimulateRider,
                showSimulateRider = true,
            )
        }
    }

    private fun show(
        armed: Boolean,
        riderWaiver: RiderWaiver = RiderWaiver.NONE,
        machine: Machine = Machine.UNKNOWN,
        boards: List<Int> = listOf(0x01),
    ) = compose.setContent {
        HoverboardRemoteTheme {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    armed = armed,
                    riderWaiver = riderWaiver,
                    machine = machine,
                    nodes = boards.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME) },
                    attachedNode = boards.firstOrNull(),
                    selected = boards.firstOrNull(),
                    telemetryByBoard = boards.associateWith { TelemetryUi() },
                    // SINGLE on the first board: a machine with a motor controller to arm.
                    drive = DriveInForce(DriveTargets(DriveMode.SINGLE, boards.take(1)), DriveMode.SINGLE, null),
                    driving = DriveTargets(DriveMode.SINGLE, boards.take(1)),
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onJoystickMove = { _, _, _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = {},
                showSimulateRider = false,
            )
        }
    }

    // --- the rover pair (`specs/rider-ui.md` 3.7a) ---------------------------------------------

    /**
     * A machine holding two different frames: the arm control refuses, names each board's value,
     * and the badge says so beside the board chips, before the control is ever tapped.
     */
    @Test
    fun aPairWithTwoDifferentFramesRefusesTheArmAndBadgesTheChips() {
        show(
            armed = false,
            machine = Machine(
                FramePair.Differ(mapOf(0x01 to true, 0x02 to false)),
                SidePair.NoneRight,
                ModePair.Unknown,
            ),
            boards = listOf(0x01, 0x02),
        )

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.board_chip_frame_differs)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.arm_blocked)).assertIsDisplayed()
        val statement = context.getString(
            R.string.arm_frames_differ,
            "0x01 ${context.getString(R.string.setup_choice_frame_rigid)}, " +
                "0x02 ${context.getString(R.string.setup_choice_frame_pivoting)}",
        )
        compose.onNodeWithText(statement).assertIsDisplayed()
    }

    /** The side warns and never refuses: the control still reads "tap to arm". */
    @Test
    fun aRigidPairWithNoRightWheelWarnsAndStillOffersTheArm() {
        show(
            armed = false,
            machine = Machine(FramePair.Agree(rigid = true), SidePair.NoneRight, ModePair.Unknown),
            boards = listOf(0x01, 0x02),
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
            machine = Machine(FramePair.Agree(rigid = false), SidePair.NoneRight, ModePair.Unknown),
            boards = listOf(0x01, 0x02),
        )

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.arm_action)).assertIsDisplayed()
    }

    /** One motor controller found: a warning that says to power the other, and no badge, because nothing is known. */
    @Test
    fun aSessionWithOneBoardWarnsThatTheOtherHalfIsUnknown() {
        show(armed = false, machine = Machine.UNKNOWN, boards = listOf(0x01))

        compose.onNodeWithTag(BOARD_CHIP_MACHINE_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.arm_frame_no_peer)).assertIsDisplayed()
    }

    /** A second board that was found but did not answer: the remedy is reading it again. */
    @Test
    fun aFrameThatDidNotReadWarnsToReadAgain() {
        show(
            armed = false,
            machine = Machine(FramePair.Unknown, SidePair.Unknown, ModePair.Unknown, peerFound = true),
            boards = listOf(0x01, 0x02),
        )

        compose.onNodeWithText(context.getString(R.string.arm_frame_unread)).assertIsDisplayed()
    }

    /**
     * The debug-build stream line (`specs/rider-ui.md` 3.1): under the status strip when the build
     * asks for it, one entry per source in address order, and nowhere when it does not.
     */
    @Test
    fun theFrameStatsLineShowsOnlyWhenTheBuildAsksForIt() {
        val stats = mapOf(0x02 to FrameStats(206, 1_310), 0x01 to FrameStats(412, 410))
        val shown = mutableStateOf(true)
        compose.setContent {
            HoverboardRemoteTheme {
                ControlScreen(
                    state = UiState(connectionState = ConnectionState.CONNECTED, frameStats = stats),
                    onArmToggle = {},
                    onThrottleMove = { _, _ -> },
                    onJoystickMove = { _, _, _, _ -> },
                    onThrottleRelease = {},
                    onDisconnect = {},
                    onSimulateRider = {},
                    showSimulateRider = false,
                    showFrameStats = shown.value,
                )
            }
        }
        compose.onNodeWithTag(FRAME_STATS_TAG)
            .assertIsDisplayed()
            .assertTextEquals("0x01 412 · gap 0.41 s   0x02 206 · gap 1.31 s")

        shown.value = false
        compose.waitForIdle()
        compose.onNodeWithTag(FRAME_STATS_TAG).assertDoesNotExist()
    }

    private companion object {
        /** The `device.name` every board in these states carries: the registry default. */
        const val BOARD_NAME = "Hoverboard"

        /** The word every armed indicator this screen has ever had was built around. */
        const val ARMED_WORD = "ARMED"
    }
}

/** `CYCLIC_STATE.mode` as the firmware's mode machine emits it (`crates/state/src/mode.rs`,
 *  `ModeMachine`). */
private const val MODE_OFF = 0
private const val MODE_INIT = 1
private const val MODE_RUN = 3
private const val MODE_SHUTDOWN = 4

/** A mode byte the firmware's machine never emits, which is what a newer image would look like. */
private const val MODE_UNALLOCATED = 9
