package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertHasClickAction
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertTextEquals
import androidx.compose.ui.test.down
import androidx.compose.ui.test.hasScrollAction
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithTag
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.onRoot
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performTouchInput
import androidx.compose.ui.test.up
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.FaultDisarm
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SpeedFacts
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.TripBaseline
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.model.totalKmText
import com.hoverboard.remote.model.tripKmText
import com.hoverboard.remote.ui.components.ARM_TAG
import com.hoverboard.remote.ui.components.JOYSTICK_TAG
import com.hoverboard.remote.ui.components.THROTTLE_TAG
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.FAULT_GRID_TAG
import com.hoverboard.remote.ui.screens.FRAME_STATS_TAG
import com.hoverboard.remote.ui.screens.RIDE_BOARDS_OFF_TAG
import com.hoverboard.remote.ui.screens.RIDE_PACK_TAG
import com.hoverboard.remote.ui.screens.RIDE_PAD_SLOT_TAG
import com.hoverboard.remote.ui.screens.RIDE_REVERSE_TAG
import com.hoverboard.remote.ui.screens.RIDE_SPEED_TAG
import com.hoverboard.remote.ui.screens.RIDE_STATEMENT_TAG
import com.hoverboard.remote.ui.screens.RIDE_TRIP_TAG
import com.hoverboard.remote.ui.screens.lampTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * The Ride screen (`specs/remote-ui.md` 3.1 and 3.1b): where the arm state is said and how many
 * times, that the drive pad does not move whatever the screen is saying, the statements under the
 * ARM control, the fault grid and its sheet, and what a figure shows when it has no value.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ControlScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any): String =
        if (args.isEmpty()) context.getString(id) else context.getString(id, *args)

    // --- the arm control -----------------------------------------------------------------------

    @Test
    fun armedStateIsStatedExactlyOnce() {
        show(ride(armed = true, mode = DriveMode.SINGLE))

        // One surface, not two: a future second indicator would fail this.
        assertEquals(1, compose.onAllNodes(hasText(ARMED_WORD, substring = true)).fetchSemanticsNodes().size)
        compose.onNodeWithText(s(R.string.arm_disarm_action)).assertIsDisplayed()
    }

    @Test
    fun disarmedScreenMakesNoArmedStatement() {
        show(ride(armed = false, mode = DriveMode.SINGLE))

        assertEquals(0, compose.onAllNodes(hasText(ARMED_WORD, substring = true)).fetchSemanticsNodes().size)
        compose.onNodeWithText(s(R.string.arm_action)).assertIsDisplayed()
    }

    /**
     * The toggle is the only stop control, so it is tappable while armed even in a state that
     * refuses ARMING (a deflected throttle). A stop control that can be unavailable is not one.
     */
    @Test
    fun disarmIsTappableEvenWhileArmingIsRefused() {
        var toggles = 0
        val state = ride(armed = true, mode = DriveMode.SINGLE).copy(throttleSpeed = 4_000, engaged = true)
        assertTrue(!state.canArm)

        show(state, onArmToggle = { toggles++ })

        compose.onNodeWithTag(ARM_TAG).assertHasClickAction().performClick()
        assertEquals(1, toggles)
    }

    /** A board the arm would arm reporting an arm-blocking bit refuses the arm (`specs/remote-ui.md` 3.1). */
    @Test
    fun aBlockingFaultRefusesTheArm() {
        val state = ride(armed = false, mode = DriveMode.SINGLE)
            .reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL))
        assertTrue(!state.canArm)
        assertTrue(state.armBlocked)
        // A held lamp alone refuses nothing: the board has cleared its latch.
        val held = ride(armed = false, mode = DriveMode.SINGLE).copy(latchHeld = mapOf(0x01 to CyclicState.EV_LATCH_A))
        assertTrue(held.canArm)
        assertTrue("a held lamp still shows the grid", held.gridShows)
    }

    // --- the statements ------------------------------------------------------------------------

    /** `specs/control.md` (i): the rider requirement waived, said under the arm control. */
    @Test
    fun theStatementsSayTheRiderRequirementIsWaivedOnlyWhenItIs() {
        show(ride(armed = false, mode = DriveMode.SINGLE).copy(riderWaiver = RiderWaiver.WAIVED))
        compose.onNodeWithText(s(R.string.ride_rider_waived)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_rider_maybe_waived)).assertDoesNotExist()

        update(ride(armed = false, mode = DriveMode.SINGLE).copy(riderWaiver = RiderWaiver.POSSIBLY))
        compose.onNodeWithText(s(R.string.ride_rider_maybe_waived)).assertIsDisplayed()

        update(ride(armed = false, mode = DriveMode.SINGLE))
        compose.onNodeWithText(s(R.string.ride_rider_waived)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.ride_rider_maybe_waived)).assertDoesNotExist()
    }

    /** The one disarm the rider did not ask for is said, and the next arm answers it. */
    @Test
    fun theStatementsSayWhenTheAppDisarmedTheMachineItself() {
        show(ride(armed = false, mode = DriveMode.SINGLE).copy(selfDisarmed = true))
        compose.onNodeWithText(s(R.string.ride_self_disarmed)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.arm_action)).assertIsDisplayed()

        update(ride(armed = true, mode = DriveMode.SINGLE))
        compose.onNodeWithText(s(R.string.ride_self_disarmed)).assertDoesNotExist()
    }

    /** With no motor controller answered yet there is nothing to arm, and the line says so. */
    @Test
    fun withNoBoardToArmTheStatementSaysSo() {
        show(
            UiState(
                connectionState = ConnectionState.CONNECTED,
                nodes = listOf(NodeInfo(0x01, Fields.NodeClass.BRIDGE, "esp")),
                attachedNode = 0x01,
            ),
        )
        compose.onNodeWithText(s(R.string.ride_no_board)).assertIsDisplayed()
    }

    /** A drive refusal (`specs/remote-ui.md` 3.1c) is a statement with its short text. */
    @Test
    fun aDriveRefusalSaysWhy() {
        val pair = ride(armed = false, mode = DriveMode.SINGLE, other = 0x02)
        show(pair.driving(refused(DriveMode.RIGID, DriveRefusal.NO_LEADER)))
        compose.onNodeWithText(s(R.string.drive_refusal_no_leader)).assertIsDisplayed()
        // The pad is the stored mode's: RIGID steers.
        compose.onNodeWithTag(JOYSTICK_TAG).assertIsDisplayed()

        update(pair.driving(refused(null, DriveRefusal.DISAGREE)))
        compose.onNodeWithText(s(R.string.drive_refusal_disagree)).assertIsDisplayed()

        update(pair.driving(refused(DriveMode.BOUND, DriveRefusal.NOT_TWO)))
        compose.onNodeWithText(s(R.string.drive_refusal_not_two, "BOUND", 2)).assertIsDisplayed()
    }

    /** A pair holding two frames refuses the arm: the warning, and the ARM label faint. */
    @Test
    fun aPairWithTwoDifferentFramesSaysItCannotArm() {
        val frames = Machine(FramePair.Differ(mapOf(0x01 to true, 0x02 to false)), SidePair.NoneRight, ModePair.Unknown)
        val state = ride(armed = false, mode = DriveMode.SINGLE, other = 0x02).copy(machine = frames)
        assertTrue(!state.canArm)
        show(state)

        compose.onNodeWithText(s(R.string.ride_frames_differ)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.arm_action)).assertIsDisplayed()
    }

    /** The side warns and never refuses; one board and an unread frame each have their line. */
    @Test
    fun theMachineCautionsHaveTheirLines() {
        val pair = ride(armed = false, mode = DriveMode.SINGLE, other = 0x02)
        show(pair.copy(machine = Machine(FramePair.Agree(rigid = true), SidePair.NoneRight, ModePair.Unknown)))
        compose.onNodeWithText(s(R.string.ride_side_none_right)).assertIsDisplayed()

        update(pair.copy(machine = Machine(FramePair.Agree(rigid = true), SidePair.BothRight, ModePair.Unknown)))
        compose.onNodeWithText(s(R.string.ride_side_both_right)).assertIsDisplayed()

        update(pair.copy(machine = Machine(FramePair.Unknown, SidePair.Unknown, ModePair.Unknown, peerFound = true)))
        compose.onNodeWithText(s(R.string.ride_frame_unread)).assertIsDisplayed()

        update(ride(armed = false, mode = DriveMode.SINGLE).copy(machine = Machine.UNKNOWN))
        compose.onNodeWithText(s(R.string.ride_frame_no_peer)).assertIsDisplayed()
    }

    /** The first two statements that apply are shown, in the table's order, and no more. */
    @Test
    fun atMostTwoStatementsAreShown() {
        show(
            ride(armed = false, mode = DriveMode.SINGLE).copy(
                selfDisarmed = true,
                riderWaiver = RiderWaiver.WAIVED,
                machine = Machine.UNKNOWN,
            ),
        )
        compose.onAllNodesWithTag(RIDE_STATEMENT_TAG).assertCountEquals(2)
        compose.onNodeWithText(s(R.string.ride_self_disarmed)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_rider_waived)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_frame_no_peer)).assertDoesNotExist()
    }

    /**
     * The fault disarm's statement names the lamp and the board, shows while the grid does not, and
     * opens that lamp's sheet when tapped, releasing the drive first.
     */
    @Test
    fun theFaultDisarmStatementOpensItsLampsSheet() {
        var releases = 0
        val state = sided(armed = false).copy(faultDisarm = FaultDisarm(FaultLamp.HALL, 0x01))
        show(state, onThrottleRelease = { releases++ })

        val line = s(R.string.ride_disarmed_fault, s(R.string.lamp_hall_title), s(R.string.ride_board_left_inline))
        compose.onNodeWithText(line).assertIsDisplayed().performClick()
        assertEquals(1, releases)
        compose.onNodeWithText(s(R.string.lamp_hall_when)).assertExists()

        // While the grid shows, it says why, and the statement is not drawn.
        update(state.reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL)))
        compose.onNodeWithText(line).assertDoesNotExist()
    }

    // --- the fault grid ------------------------------------------------------------------------

    /** Dark when normal: no grid without a lit lamp or the lamp test. */
    @Test
    fun theGridShowsOnlyForALitLampOrTheLampTest() {
        show(sided(armed = false))
        compose.onNodeWithTag(FAULT_GRID_TAG).assertDoesNotExist()

        update(sided(armed = false).reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL)))
        compose.onNodeWithTag(FAULT_GRID_TAG).assertIsDisplayed()

        update(sided(armed = false).copy(lampTest = true))
        compose.onNodeWithTag(FAULT_GRID_TAG).assertIsDisplayed()
    }

    /** Tapping a lamp opens its sheet with its title, releases the drive first, and Close dismisses it. */
    @Test
    fun tappingALampOpensItsSheet() {
        var releases = 0
        show(
            sided(armed = false).reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL)),
            onThrottleRelease = { releases++ },
        )
        assertEquals("the grid took the pad's place, which releases the drive", 1, releases)
        releases = 0

        compose.onNodeWithTag(lampTag(FaultLamp.HALL)).performClick()
        assertEquals("the scrim covers the pad, so the drive is released first", 1, releases)
        compose.onNodeWithText(s(R.string.lamp_hall_title)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.lamp_where, s(R.string.lamp_board_left), "F8")).assertIsDisplayed()

        compose.onNodeWithText(s(R.string.lamp_close)).performClick()
        compose.waitForIdle()
        compose.onNodeWithText(s(R.string.lamp_hall_when)).assertDoesNotExist()
    }

    /** Every lamp's sheet has a title and three non-empty bodies. */
    @Test
    fun everyLampHasItsSheetText() {
        for (lamp in FaultLamp.entries) {
            for (id in listOf(lamp.title, lamp.whenShows, lamp.blocks, lamp.fix)) {
                assertTrue("$lamp has an empty sheet string", s(id).isNotBlank())
            }
        }
    }

    /** A dark lamp opens too, and says it is not lit. */

    @Test
    fun aDarkLampsSheetSaysNotLit() {
        show(sided(armed = false).reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL)))

        compose.onNodeWithTag(lampTag(FaultLamp.CAL)).performClick()
        compose.onNodeWithText(s(R.string.lamp_where, s(R.string.lamp_not_lit), "F9")).assertIsDisplayed()
    }

    // --- the pad does not move -----------------------------------------------------------------

    /**
     * The pad's bounds in root coordinates are equal across every state the screen can be in; the
     * fault grid and the lamp test are drawn in those bounds with no pad; the motor-controller-off
     * card ends at the pad's bottom; and the whole screen fits with nothing scrolling
     * (`specs/remote-ui.md` 3.1 and 3.1d).
     */
    @Test
    fun thePadDoesNotMove() = assertTheScreenFits()

    /** The same at the ASUS ROG's 1080 x 1920 px, where the instruments shrink and the pad does not. */
    @Test
    @Config(qualifiers = "w360dp-h640dp-xxhdpi")
    fun thePadDoesNotMoveAndNothingScrollsOnTheAsus() = assertTheScreenFits()

    private fun assertTheScreenFits() {
        val riding = sided(armed = true).copy(throttleSpeed = 3_000, engaged = true)
        val states = listOf(
            riding,
            sided(armed = true).copy(packCells = mapOf(0x01 to 10, 0x02 to 10)).reporting(0x01, board(battery = 3_050)),
            sided(armed = false).copy(
                telemetryByBoard = mapOf(0x02 to board()),
                imuModel = mapOf(0x01 to 0, 0x02 to 0),
            ),
            sided(armed = false).copy(selfDisarmed = true, riderWaiver = RiderWaiver.WAIVED),
            sided(armed = false).copy(
                speedFacts = listOf(SpeedFacts(0x01, 0, polePairs = 0, diameterMm = 165, side = 0)),
            ),
            riding.copy(frameStats = mapOf(0x01 to FrameStats(412, 410), 0x02 to FrameStats(206, 1_310))),
        )
        show(states.first(), showFrameStats = true)
        val first = padBounds()
        for (state in states.drop(1)) {
            update(state)
            assertEquals("the pad moved in $state", first, padBounds())
        }
        val gridStates = listOf(
            sided(armed = false)
                .reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL))
                .reporting(0x02, board(fault = CyclicState.EV_IMU_LOSS)),
            sided(armed = false).copy(lampTest = true),
        )
        for (state in gridStates) {
            update(state)
            compose.onNodeWithTag(THROTTLE_TAG).assertDoesNotExist()
            compose.onNodeWithTag(FAULT_GRID_TAG).assertIsDisplayed()
            assertEquals("the grid is in the pad's bounds", first, bounds(RIDE_PAD_SLOT_TAG))
        }
        update(sided(armed = false).copy(boardsOff = true))
        compose.onNodeWithTag(THROTTLE_TAG).assertDoesNotExist()
        assertEquals("the card ends at the pad's bottom", first.bottom, bounds(RIDE_BOARDS_OFF_TAG).bottom)

        val root = compose.onRoot().fetchSemanticsNode().boundsInRoot
        assertTrue("the pad starts below the instruments", first.top > 0f)
        for (tag in listOf(ARM_TAG, RIDE_TRIP_TAG, FRAME_STATS_TAG)) {
            val b = bounds(tag)
            assertTrue("$tag is on screen: $b in $root", b.bottom <= root.bottom)
        }
        compose.onAllNodes(hasScrollAction()).assertCountEquals(0)
    }

    private fun padBounds(): Rect = bounds(THROTTLE_TAG)

    private fun bounds(tag: String): Rect = compose.onNodeWithTag(tag).fetchSemanticsNode().boundsInRoot

    // --- the drive control, per mode (`specs/remote-ui.md` 3.2) ---------------------------------

    @Test
    fun theJoystickIsTheControlInTheModesThatSteerAndTheThrottlePadInTheOthers() {
        show(ride(armed = true, mode = DriveMode.SINGLE))
        for (mode in DriveMode.entries) {
            update(ride(armed = true, mode = mode))
            val steers = mode.hasTurnAxis
            compose.onNodeWithTag(if (steers) JOYSTICK_TAG else THROTTLE_TAG).assertIsDisplayed()
            compose.onNodeWithTag(if (steers) THROTTLE_TAG else JOYSTICK_TAG).assertDoesNotExist()
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

    // --- what a figure shows with no value -----------------------------------------------------

    @Test
    fun aMissingSpeedIsDashesAndSaysWhyInPlainWords() {
        var opened: Pair<Int, Key>? = null
        show(
            sided(armed = false).copy(telemetryByBoard = emptyMap()),
            onOpenSetup = { board, row -> opened = board to row },
        )
        compose.onNodeWithTag(RIDE_SPEED_TAG).assertTextEquals(s(R.string.ride_figure_none))
        compose.onNodeWithText(s(R.string.ride_no_board_reporting)).assertIsDisplayed()

        // Reporting, and the second board holds no wheel diameter: no field name, a statement that
        // opens Setup on that board at the top of its motor rows.
        update(
            sided(armed = false).copy(
                speedFacts = listOf(
                    SpeedFacts(0x01, 0, polePairs = 15, diameterMm = 165, side = 0),
                    SpeedFacts(0x02, 0, polePairs = 15, diameterMm = 0, side = 1),
                ),
                packCells = mapOf(0x01 to 10, 0x02 to 10),
            ),
        )
        compose.onNodeWithTag(RIDE_SPEED_TAG).assertTextEquals(s(R.string.ride_figure_none))
        compose.onAllNodes(hasText("motors.0", substring = true)).assertCountEquals(0)
        compose.onNodeWithText(s(R.string.ride_need_wheel_size)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_set_it)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_need_wheel_size)).performClick()
        assertEquals(0x02 to SetupFields.MOTOR_METHOD.key, opened)

        // Not read yet: plain words and no link.
        update(
            sided(armed = false).copy(
                speedFacts = listOf(SpeedFacts(0x01, 0, polePairs = null, diameterMm = 165, side = 0)),
                packCells = mapOf(0x01 to 10, 0x02 to 10),
            ),
        )
        compose.onNodeWithText(s(R.string.ride_reading_wheel)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_set_it)).assertDoesNotExist()
    }

    /** No pack voltage is words, never 0.0 V; a voltage with no cell count is the voltage and a statement. */
    @Test
    fun theWordsAPackLineHasForNoReading() {
        var opened: Pair<Int, Key>? = null
        show(
            sided(armed = false).reporting(0x01, board(battery = 0)).reporting(0x02, board(battery = 0)),
            onOpenSetup = { board, row -> opened = board to row },
        )
        compose.onNodeWithTag(RIDE_PACK_TAG).assertTextEquals(s(R.string.ride_no_pack_voltage))

        update(sided(armed = false))
        compose.onNodeWithTag(RIDE_PACK_TAG).assertTextEquals(s(R.string.ride_pack_volts, 37.8f))
        compose.onNodeWithText(s(R.string.ride_need_cells)).assertIsDisplayed().performClick()
        assertEquals(0x01 to SetupFields.PACK_CELLS.key, opened)
    }

    /** The speed is a magnitude: "R" beside the unit while it is below zero, and only then. */
    @Test
    fun theSpeedIsNeverNegativeAndReverseIsR() {
        fun at(rate: Int) = sided(armed = false).copy(
            speedFacts = listOf(
                SpeedFacts(0x01, rate, polePairs = 15, diameterMm = 165, side = Fields.ControlSide.LEFT),
                SpeedFacts(0x02, -rate, polePairs = 15, diameterMm = 165, side = Fields.ControlSide.RIGHT),
            ),
        )
        show(at(-40_000))
        val shown = compose.onNodeWithTag(RIDE_SPEED_TAG).fetchSemanticsNode()
            .config[androidx.compose.ui.semantics.SemanticsProperties.Text].joinToString()
        assertTrue("no minus sign: $shown", !shown.contains("-"))
        compose.onNodeWithTag(RIDE_REVERSE_TAG).assertIsDisplayed()

        update(at(40_000))
        compose.onNodeWithTag(RIDE_REVERSE_TAG).assertDoesNotExist()
    }

    /**
     * No motor controller reporting with the link up: one card in place of the instruments and the
     * pad, naming what the phone is connected to; no statements; ARM refuses a tap.
     */
    @Test
    fun theMotorControllerOffCardTakesThePadsPlace() {
        var toggles = 0
        val off = UiState(
            connectionState = ConnectionState.CONNECTED,
            nodes = listOf(NodeInfo(0x01, Fields.NodeClass.BRIDGE, "offroad-master")),
            attachedNode = 0x01,
            deviceName = "offroad-master",
            boardsOff = true,
        )
        assertTrue(!off.canArm)
        show(off, onArmToggle = { toggles++ })
        compose.onNodeWithTag(RIDE_BOARDS_OFF_TAG).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_off_title)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.ride_off_waiting, "offroad-master")).assertIsDisplayed()
        compose.onNodeWithTag(THROTTLE_TAG).assertDoesNotExist()
        compose.onNodeWithTag(RIDE_SPEED_TAG).assertDoesNotExist()
        compose.onAllNodesWithTag(RIDE_STATEMENT_TAG).assertCountEquals(0)
        compose.onNodeWithTag(ARM_TAG).performClick()
        assertEquals(0, toggles)
    }

    /** While armed the grid does not take the pad; a lamp lit on a board the arm does not drive is a statement. */
    @Test
    fun armedThePadStaysAndALitLampIsAStatement() {
        val single = ride(armed = true, mode = DriveMode.SINGLE, other = 0x02)
            .copy(sides = mapOf(0x01 to Fields.ControlSide.LEFT, 0x02 to Fields.ControlSide.RIGHT))
            .reporting(0x02, board(fault = CyclicState.EV_IMU_LOSS))
        show(single)
        compose.onNodeWithTag(THROTTLE_TAG).assertIsDisplayed()
        compose.onNodeWithTag(FAULT_GRID_TAG).assertDoesNotExist()
        val line = s(R.string.ride_lamp_lit_armed, s(R.string.lamp_imu_title), s(R.string.ride_board_right_inline))
        compose.onNodeWithText(line).assertIsDisplayed()
    }

    /** A driven wheel that does not turn is a warning statement naming the board. */
    @Test
    fun aWheelThatDoesNotTurnIsSaid() {
        show(sided(armed = true).copy(notTurning = setOf(0x02)))
        compose.onNodeWithText(s(R.string.ride_not_turning, s(R.string.lamp_board_right))).assertIsDisplayed()
    }

    /** The trip line is blank while there is no machine distance, and reads both figures when there is. */
    @Test
    fun theTripLineReadsTripAndTotal() {
        // No wheel geometry: the boards' travel is edges, not a distance, so the line is blank.
        val edges = sided(armed = false).copy(trip = TripBaseline("AA:BB", 1_200_000))
        assertTrue(edges.machineTravel !is MachineTravel.Distance)
        show(edges)
        compose.onNodeWithTag(RIDE_TRIP_TAG).assertTextEquals("")

        val wheel = WheelGeometry(polePairs = 15, diameterMm = 165)
        val odometer = board().cyclic!!.copy(obs = QUIET_OBS.copy(odometer = 1_000_000L))
        val travelled = edges
            .reporting(0x01, TelemetryUi().merge(odometer))
            .reporting(0x02, TelemetryUi().merge(odometer))
            .copy(wheelGeometry = mapOf(0x01 to wheel, 0x02 to wheel))
        val total = (travelled.machineTravel as MachineTravel.Distance).millimetres
        update(travelled)
        compose.onNodeWithTag(RIDE_TRIP_TAG)
            .assertTextEquals(s(R.string.ride_trip, tripKmText(total - 1_200_000), totalKmText(total)))
    }


    /**
     * The debug-build stream line (`specs/remote-ui.md` 3.1): one entry per source in address order
     * when the build asks for it, and nowhere when it does not.
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

    // --- fixtures ------------------------------------------------------------------------------

    /** A board's frame: [fault] its word, [battery] centivolts, at rest in OFF. */
    private fun board(fault: Int = 0, battery: Int = 3_780, flags: Int = 0) = TelemetryUi().merge(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = battery,
            mode = 0,
            fault = fault,
            flags = flags,
            peerDemand = 0,
            wheelPos = 0,
            wheelRate = 0,
            obs = QUIET_OBS,
        ),
    )

    /**
     * A connected ride screen attached to 0x01, with [other] found by the walk, every node a motor
     * controller and 0x01 selected, driving in [mode] as the mode in force. 0x01 has reported.
     */
    private fun ride(armed: Boolean, mode: DriveMode, other: Int? = null): UiState {
        val boards = listOfNotNull(0x01, other)
        val targets = when (mode) {
            DriveMode.SINGLE -> DriveTargets(DriveMode.SINGLE, listOf(0x01))
            DriveMode.DIFFERENTIAL -> DriveTargets(mode, listOf(0x01, 0x02), right = 0x01)
            else -> DriveTargets(mode, listOf(0x01, 0x02))
        }
        return UiState(
            connectionState = ConnectionState.CONNECTED,
            armed = armed,
            nodes = boards.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME) },
            attachedNode = 0x01,
            selected = 0x01,
            drive = DriveInForce(targets, stored = mode, refusal = null),
            driving = targets,
            telemetryByBoard = boards.associateWith { board() },
            machine = Machine(FramePair.Agree(rigid = false), SidePair.OneRight(0x02), ModePair.Agree(mode.byte)),
        )
    }

    /** The pair BOUND, 0x01 the left wheel and 0x02 the right, both reporting. */
    private fun sided(armed: Boolean) = ride(armed = armed, mode = DriveMode.BOUND, other = 0x02)
        .copy(sides = mapOf(0x01 to Fields.ControlSide.LEFT, 0x02 to Fields.ControlSide.RIGHT))

    /** This disarmed state driving [drive], as the view model has it: the arm would arm its targets. */
    private fun UiState.driving(drive: DriveInForce) = copy(drive = drive, driving = drive.targets)

    private fun refused(stored: DriveMode?, why: DriveRefusal) =
        DriveInForce(DriveTargets(stored ?: DriveMode.SINGLE, emptyList()), stored, why)

    /** This state with [board] having last reported [telemetry]. */
    private fun UiState.reporting(board: Int, telemetry: TelemetryUi) =
        copy(telemetryByBoard = telemetryByBoard + (board to telemetry))

    /** The state the screen is composed from: one `setContent` per test, recomposed by [update]. */
    private val rideState = mutableStateOf(UiState())

    private fun show(
        state: UiState,
        onArmToggle: () -> Unit = {},
        onJoystickMove: (Float, Float, Float, Float) -> Unit = { _, _, _, _ -> },
        onThrottleRelease: () -> Unit = {},
        showFrameStats: Boolean = false,
        onOpenSetup: (Int, Key) -> Unit = { _, _ -> },
    ) {
        rideState.value = state
        compose.setContent {
            HoverboardRemoteTheme {
                ControlScreen(
                    state = rideState.value,
                    onArmToggle = onArmToggle,
                    onThrottleMove = { _, _ -> },
                    onJoystickMove = onJoystickMove,
                    onThrottleRelease = onThrottleRelease,
                    showFrameStats = showFrameStats,
                    onOpenSetup = onOpenSetup,
                )
            }
        }
    }

    /** Put the shown screen in [state] and let it recompose. */
    private fun update(state: UiState) {
        rideState.value = state
        compose.waitForIdle()
    }

    private companion object {
        /** The `device.name` every board in these states carries: the registry default. */
        const val BOARD_NAME = "Hoverboard"

        /** The word every armed indicator this screen has ever had was built around. */
        const val ARMED_WORD = "ARMED"
    }
}
