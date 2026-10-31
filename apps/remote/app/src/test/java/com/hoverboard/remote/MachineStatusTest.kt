package com.hoverboard.remote

import android.app.Application
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Modifier
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertTextEquals
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.longClick
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performTouchInput
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.batteryReading
import com.hoverboard.remote.ui.screens.MachineStatusSection
import com.hoverboard.remote.ui.screens.statusBoardTag
import com.hoverboard.remote.ui.screens.RIGID_NOT_LEADING_TAG
import com.hoverboard.remote.ui.screens.SIM_RIDER_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What Setup's machine status section (`specs/remote-ui.md` 3.1c) says about each board's own state,
 * which the Ride screen's telemetry panel used to show for the selected board alone, and, more
 * importantly, what it refuses to say.
 *
 * `CYCLIC_STATE.fault` is the reporting board's OWN producer mask, so a quiet green indicator would
 * tell a rider their machine is healthy on the strength of one board's word: each board has its own
 * row, and a quiet row covers only that board. `battery` is 0 for UNKNOWN, which is the firmware
 * declining to report rather than a measurement, and is pinned in the model here; the Ride screen's
 * pack line has its own tests ([ControlScreenTest]).
 *
 * So these tests pin absences as hard as they pin presences.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class MachineStatusTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()

    @Test
    fun riderIsStatedWhenTheBoardSeesARider() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        compose.onNodeWithText(context.getString(R.string.status_rider)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.status_no_rider)).assertDoesNotExist()
    }

    @Test
    fun noRiderIsStatedTooBecauseItIsAReading() {
        // A board reporting no rider IS reporting something: its pads read low.
        show(TelemetryUi().merge(cyclic(flags = 0)))

        compose.onNodeWithText(context.getString(R.string.status_no_rider)).assertIsDisplayed()
    }

    @Test
    fun noLockdownLineWhileTheBoardIsNotInLockdown() {
        show(TelemetryUi().merge(cyclic(flags = 0)))

        compose.onNodeWithText(context.getString(R.string.status_lockdown)).assertDoesNotExist()
    }

    @Test
    fun lockdownLineAppearsWhenTheBoardAssertsIt() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_LOCKDOWN)))

        compose.onNodeWithText(context.getString(R.string.status_lockdown)).assertIsDisplayed()
        compose.onNodeWithText(boardRow(R.string.board_state_lockdown)).assertIsDisplayed()
    }

    /**
     * The app's reading of a driven wheel that does not turn: one line, under that board
     * (`specs/remote-ui.md` 3.1).
     */
    @Test
    fun aWheelThatDoesNotTurnHasItsLineUnderItsBoard() {
        showState(pair(DriveInForce.NONE).copy(notTurning = setOf(0x02)))

        compose.onAllNodesWithText(context.getString(R.string.status_not_turning)).assertCountEquals(1)
        val line = compose.onNodeWithText(context.getString(R.string.status_not_turning))
            .fetchSemanticsNode().boundsInRoot
        val left = compose.onNodeWithTag(statusBoardTag(0x01)).fetchSemanticsNode().boundsInRoot
        val right = compose.onNodeWithTag(statusBoardTag(0x02)).fetchSemanticsNode().boundsInRoot
        assertTrue("under the second board's row", line.top > right.top && right.top > left.top)
    }

    /** A board nothing has arrived from is SILENT, with no line under it that would be a reading. */
    @Test
    fun aSilentBoardIsSilentAndSaysNothingElse() {
        show(null)

        compose.onNodeWithText(boardRow(R.string.board_state_quiet)).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.status_no_rider)).assertDoesNotExist()
    }

    /** Bit 6 is a readback the board runs through, stated as caution and never as a fault. */
    @Test
    fun theModeReadbackIsALineAndNotAFault() {
        show(TelemetryUi().merge(cyclic(fault = CyclicState.EV_MODE_FAULT)))

        compose.onNodeWithText(context.getString(R.string.board_mode_demoted, BOARD)).assertIsDisplayed()
        compose.onNodeWithText(FAULT_WORD, substring = true).assertDoesNotExist()
    }

    /**
     * The one that is not a styling choice. The cyclic byte speaks for the attached board's own
     * producers only, so a fault line that renders "clear" renders a claim about the machine that
     * no code behind it can support. The panel therefore draws the assertion and never the
     * negation: silence says nothing, which is the only true thing available.
     */
    @Test
    fun aBoardReportingNoFaultGetsNoFaultChipAtAll() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        // The general pin first: NOTHING on a quiet panel says FAULT. Naming the two strings we
        // happen to render today would let a future green "FAULT CLEAR" chip through, which is
        // exactly the claim this test exists to forbid. The chips are the only uppercase text here,
        // so this does not collide with the lower-case note below.
        compose.onNodeWithText(FAULT_WORD, substring = true).assertDoesNotExist()
    }

    /** A board with a fault bit set says FAULT on its own row, and its peer's row does not. */
    @Test
    fun aFaultStaysOnItsOwnBoardsRow() {
        show(
            TelemetryUi().merge(cyclic(fault = CyclicState.EV_MOTOR_HALL)),
            peer = TelemetryUi().merge(cyclic(mode = 0)),
        )

        compose.onNodeWithText(boardRow(R.string.board_state_fault)).assertIsDisplayed()
        val peerRow = context.getString(R.string.board_chip, PEER, context.getString(R.string.board_state_off))
        compose.onNodeWithText(peerRow)
            .assertIsDisplayed()
    }

    /**
     * The cyclic fault word is the board's producer level mask, and a fault bit in it (here
     * bit 1, `EV_STOP_ALL`) is one line naming the board and the cause.
     */
    @Test
    fun aSetFaultBitIsALineNamingTheBoardAndTheCause() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = CyclicState.EV_STOP_ALL)))

        compose.onNodeWithText(faultLine(R.string.fault_cause_tilt_stop)).assertIsDisplayed()
    }

    /**
     * Every shown bit gets its own line: bits 0 to 2, 4, 5 and 8 to 11, the four motor causes
     * among them, so a motor fault says which of the four it is.
     */
    @Test
    fun eachSetCauseIsItsOwnLine() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 0x0F37)))

        for (cause in listOf(
            R.string.fault_cause_comms_loss,
            R.string.fault_cause_tilt_stop,
            R.string.fault_cause_imu_loss,
            R.string.fault_cause_latch_a,
            R.string.fault_cause_latch_b,
            R.string.fault_cause_motor_hall,
            R.string.fault_cause_motor_cal,
            R.string.fault_cause_motor_period,
            R.string.fault_cause_motor_arm,
        )) {
            compose.onNodeWithText(faultLine(cause)).assertIsDisplayed()
        }
    }

    /** Motor 1's latch is fault B, not fault A, and is still named. */
    @Test
    fun motorOnesLatchIsNamed() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = CyclicState.EV_LATCH_B)))

        compose.onNodeWithText(faultLine(R.string.fault_cause_latch_b)).assertIsDisplayed()
    }

    /** The retired bit 3 and the unallocated bits 12 to 15 name nothing. */
    @Test
    fun theRetiredAndUnallocatedBitsNameNothing() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 0xF008)))

        compose.onNodeWithText(FAULT_WORD, substring = true).assertDoesNotExist()
    }

    /**
     * The mask also carries two levels that are not faults, the mode fault (bit 6) and the power
     * request (bit 7), and an ARMED board holds the second one for the whole ride. Rendering the
     * byte unmasked would light FAULT on every arm, so the chip reads only the fault-A bits
     * (`CyclicState.FAULT_A_BITS`), and a mask with nothing else in it is a quiet panel.
     */
    @Test
    fun thePowerRequestAndModeFaultBitsDoNotLightTheFaultChip() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER, fault = 0xC0)))

        compose.onNodeWithText(FAULT_WORD, substring = true).assertDoesNotExist()
    }

    /** And a fault bit beside them is its one line, with no line for the non-fault bits. */
    @Test
    fun aFaultBesideTheNonFaultBitsIsTheOnlyLine() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 0xC0 or CyclicState.EV_COMMS_LOSS)))

        compose.onNodeWithText(faultLine(R.string.fault_cause_comms_loss)).assertIsDisplayed()
        compose.onAllNodesWithText("$FAULT_WORD $BOARD", substring = true).assertCountEquals(1)
    }

    private fun faultLine(cause: Int): String =
        context.getString(R.string.telemetry_fault_cause, BOARD, context.getString(cause))

    private fun boardRow(state: Int): String =
        context.getString(R.string.board_chip, BOARD, context.getString(state))

    /**
     * The absence is in the model, not only in the panel, so nothing downstream can score a pack
     * that was never measured: there is no voltage, and the reading that pairs it with the pack
     * says the board is reporting nothing.
     */
    @Test
    fun anUnknownReadingHasNoVoltageAndNoChargeInTheModel() {
        val unknown = TelemetryUi().merge(cyclic(battery = 0))
        assertNull(unknown.batteryVolts)
        assertEquals(BatteryReading.NoReading, batteryReading(unknown, PACK_CELLS, LI_ION))

        val sensed = TelemetryUi().merge(cyclic(battery = 2_900))
        assertEquals(29.0f, sensed.batteryVolts!!, 0.001f)
        val charge = batteryReading(sensed, PACK_CELLS, LI_ION) as BatteryReading.Charge
        assertEquals(BatteryCurve.fraction(29.0f, PACK_CELLS, LI_ION)!!, charge.fraction, 0.001f)
    }

    /** Nothing has arrived yet is the same absence, and must not read as zero volts either. */
    @Test
    fun aRecordWithNoStateYetHasNoVoltageEither() {
        assertNull(TelemetryUi().batteryVolts)
        assertEquals(BatteryReading.NoReading, batteryReading(TelemetryUi(), PACK_CELLS, LI_ION))
    }

    /**
     * The current display (`specs/remote-ui.md` 3.6): both figures, each worded for WHICH current
     * it is. The two differ by the duty and answer different questions, and a panel showing one
     * unlabelled number is how a supply reading got taken for a phase current on 2026-10-09.
     */
    @Test
    fun bothCurrentFiguresAreDrawnAndEachSaysWhichOneItIs() {
        // 910 counts at 455 counts per amp is 2.0 A at the motor, half-duty is 1.0 A at the supply.
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL / 2))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_current_value, 2.0f, 1.0f))
            .assertIsDisplayed()
    }

    /**
     * The qualifier, which is the condition of showing the figure at all. Every amp rests on a
     * per-board constant that has never been checked against a reference instrument, and the
     * display is how that gets confirmed, so the screen states the footing once rather than
     * waiting for it or passing a derived number off as a measurement.
     */
    @Test
    fun theCurrentFiguresCarryWhatTheyRestOn() {
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL))

        compose.onNodeWithText(context.getString(R.string.telemetry_current_note)).assertIsDisplayed()
    }

    /** Nothing has arrived from the board, which is not the same as a board carrying no current. */
    @Test
    fun aBoardNothingHasArrivedFromSaysSoAndShowsNoAmps() {
        show(null)

        compose.onNodeWithText(context.getString(R.string.telemetry_current_silent)).assertIsDisplayed()
        assertNoFigures()
    }

    /**
     * The stale-number case. The board stops refreshing its current window when its motor path
     * goes down and keeps sending the last one it closed, so a figure here would be a current the
     * machine stopped taking minutes ago.
     */
    @Test
    fun aBoardWithItsMotorsDownSaysSoAndShowsNoAmps() {
        // Mode 0 is OFF: the motor path is down.
        show(TelemetryUi().merge(cyclic(mode = 0)))

        compose.onNodeWithText(context.getString(R.string.telemetry_current_motors_off)).assertIsDisplayed()
        assertNoFigures()
    }

    /** A period that drove nothing: the magnitude it measured is the sense chain's own noise. */
    @Test
    fun aCoastingBoardSaysSoAndShowsNoAmps() {
        show(TelemetryUi().merge(cyclic()))

        compose.onNodeWithText(context.getString(R.string.telemetry_current_coasting)).assertIsDisplayed()
        assertNoFigures()
    }

    /** No scale yet: amps at the registry default would be some other board's constant. */
    @Test
    fun aBoardWhoseCalibrationIsUnreadSaysSoAndShowsNoAmps() {
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL), countsPerAmp = null)

        compose.onNodeWithText(context.getString(R.string.telemetry_current_unscaled)).assertIsDisplayed()
        assertNoFigures()
    }

    /** One board in the session, one row: the second is for a pair, and so is the total. */
    @Test
    fun aSessionWithOneBoardDrawsOneRowAndNoTotal() {
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL))

        compose.onNodeWithText(PEER, substring = true).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.telemetry_current_machine)).assertDoesNotExist()
    }

    /**
     * The pair's total, which is the figure a bench supply's own reading is compared against: the
     * supply feeds both boards, so one board's figure is only a share of what it shows.
     */
    @Test
    fun aPairAddsItsTwoSupplyFiguresIntoAMachineTotal() {
        show(
            driving(mean = 910, duty = CyclicObs.DUTY_FULL / 2),
            peer = driving(mean = 455, duty = CyclicObs.DUTY_FULL / 2),
        )

        compose
            .onNodeWithText(context.getString(R.string.telemetry_current_machine_value, 1.5f))
            .assertIsDisplayed()
    }

    /** And says so rather than falling back to the one figure it has. */
    @Test
    fun aPairWithOneBoardNotReportingShowsNoTotal() {
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL), peerFound = true)

        compose
            .onNodeWithText(context.getString(R.string.telemetry_current_machine_incomplete))
            .assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.telemetry_current_machine_value, 2.0f))
            .assertDoesNotExist()
    }

    /**
     * Both boards, because two wheels of a rigid machine can pull very differently and that is the
     * thing to see. A second board that has gone quiet still gets its column, saying so.
     */
    @Test
    fun aPairDrawsBothBoardsAndASilentPeerStillGetsItsColumn() {
        show(driving(mean = 910, duty = CyclicObs.DUTY_FULL), peerFound = true)

        compose.onAllNodesWithText(BOARD).assertCountEquals(1)
        compose.onAllNodesWithText(PEER).assertCountEquals(1)
        compose.onNodeWithText(context.getString(R.string.telemetry_current_silent)).assertIsDisplayed()
    }

    /** No amp figure of either kind is on the screen. */
    private fun assertNoFigures() {
        compose.onNodeWithText(MOTOR_FIGURE, substring = true).assertDoesNotExist()
        compose.onNodeWithText(SUPPLY_FIGURE, substring = true).assertDoesNotExist()
    }

    /** A board pulling [mean] counts at [duty], running (mode 3). */
    private fun driving(mean: Int, duty: Int): TelemetryUi =
        TelemetryUi().merge(cyclic(mode = 3).copy(obs = QUIET_OBS.copy(phaseMean = mean, dutyOn = duty)))

    private fun cyclic(
        flags: Int = 0,
        fault: Int = 0,
        // A sensed 29.0 V, so a case that is not about the battery still renders the full
        // reading rather than the no-reading branch.
        battery: Int = 2_900,
        mode: Int = 2,
    ) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = battery,
        mode = mode,
        fault = fault,
        flags = flags,
        peerDemand = 0,
        wheelPos = 0,
        wheelRate = 0,
        obs = QUIET_OBS,
    )

    /**
     * The section over a session whose walk found [BOARD] at 0x01, last reporting [telemetry] (null:
     * silent), and with [peer] or [peerFound] [PEER] at 0x02.
     */
    private fun show(
        telemetry: TelemetryUi?,
        peer: TelemetryUi? = null,
        peerFound: Boolean = peer != null,
        countsPerAmp: Int? = 455,
    ) {
        val addresses = if (peerFound) listOf(0x01, 0x02) else listOf(0x01)
        val reports = listOfNotNull(telemetry?.let { 0x01 to it }, peer?.let { 0x02 to it }).toMap()
        val state = UiState(
            connectionState = ConnectionState.CONNECTED,
            nodes = addresses.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, "Hoverboard") },
            attachedNode = 0x01,
            telemetryByBoard = reports,
            countsPerAmp = countsPerAmp?.let { c -> addresses.associateWith { c } }.orEmpty(),
        )
        compose.setContent {
            HoverboardRemoteTheme {
                Column(Modifier.verticalScroll(rememberScrollState())) {
                    MachineStatusSection(state, showSimulateRider = false, onSimulateRider = {})
                }
            }
        }
    }

    // --- the drive, the machine, and the bench control -----------------------------------------

    /** The drive in force and the boards it addresses, by node label. */
    @Test
    fun theDriveLineNamesTheModeAndItsBoards() {
        showState(pair(DriveInForce(DriveTargets(DriveMode.BOUND, listOf(0x01, 0x02)), DriveMode.BOUND, null)))

        val both = context.getString(R.string.status_and, BOARD, PEER)
        compose.onNodeWithText(context.getString(R.string.status_drive, "BOUND", both)).assertIsDisplayed()
    }

    /** A drive refusal is the drive line's long text (`specs/remote-ui.md` 3.1c). */
    @Test
    fun aDriveRefusalIsTheLongLine() {
        val refused = DriveInForce(DriveTargets(DriveMode.RIGID, emptyList()), DriveMode.RIGID, DriveRefusal.NO_LEADER)
        showState(pair(refused))

        compose.onNodeWithText(context.getString(R.string.drive_refusal_no_leader_long)).assertIsDisplayed()
    }

    /**
     * The rigid sentence is said exactly once: when the machine is STORED as one rigid machine and
     * no board is running as its leader. A pivoting machine is told nothing.
     */
    @Test
    fun aRigidPairThatIsNotLeadingIsToldWhy() {
        val configured = pair(DriveInForce.NONE)
            .copy(machine = Machine(FramePair.Agree(rigid = true), SidePair.OneRight(0x01), ModePair.Unknown))
        showState(configured)
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertIsDisplayed()
    }

    @Test
    fun aPivotingPairIsNotToldAboutTheRigidLead() {
        val pivoting = Machine(FramePair.Agree(rigid = false), SidePair.NoneRight, ModePair.Unknown)
        showState(pair(DriveInForce.NONE).copy(machine = pivoting))
        compose.onNodeWithTag(RIGID_NOT_LEADING_TAG).assertDoesNotExist()
    }

    /** Every state a board can report has a word on its row. */
    @Test
    fun theRowNamesEveryStateTheBoardCanReport() {
        for ((mode, word) in listOf(1 to R.string.board_state_starting, 3 to R.string.board_state_live,
            4 to R.string.board_state_stopping, 9 to R.string.board_state_unrecognised)) {
            assertEquals(word, wordFor(mode))
        }
    }

    /**
     * The bench affordance takes a LONG press: on a board that has pads it holds a machine engaged
     * through the moment a rider steps off, so it must not answer to a passing thumb.
     */
    @Test
    fun theSimulateRiderRowTogglesOnALongPressAndNotOnATap() {
        var asked: Boolean? = null
        showState(pair(DriveInForce.NONE), showSimulateRider = true, onSimulateRider = { asked = it })

        compose.onNodeWithTag(SIM_RIDER_TAG).performScrollTo().performClick()
        assertNull("a tap must not flip a control that can hold a board engaged", asked)
        compose.onNodeWithTag(SIM_RIDER_TAG).performTouchInput { longClick() }
        assertEquals(true, asked)
    }

    /** And a rider's build does not carry it at all: the call site passes `BuildConfig.DEBUG`. */
    @Test
    fun theSimulateRiderRowIsAbsentUnlessTheBuildEnablesIt() {
        showState(pair(DriveInForce.NONE))
        compose.onNodeWithTag(SIM_RIDER_TAG).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.sim_rider_label)).assertDoesNotExist()
    }

    /** What the row for a board reporting [mode] says, read off the screen. */
    private fun wordFor(mode: Int): Int {
        val words = listOf(
            R.string.board_state_off, R.string.board_state_starting, R.string.board_state_live,
            R.string.board_state_stopping, R.string.board_state_unrecognised,
        )
        val frame = TelemetryUi().merge(cyclic(mode = mode))
        rows.value = pair(DriveInForce.NONE).copy(telemetryByBoard = mapOf(0x01 to frame))
        if (!composed) {
            composed = true
            compose.setContent {
                HoverboardRemoteTheme {
                    Column(Modifier.verticalScroll(rememberScrollState())) {
                        MachineStatusSection(rows.value, showSimulateRider = false, onSimulateRider = {})
                    }
                }
            }
        }
        compose.waitForIdle()
        return words.single { w ->
            compose.onAllNodesWithText(context.getString(R.string.board_chip, BOARD, context.getString(w)))
                .fetchSemanticsNodes().isNotEmpty()
        }
    }

    private val rows = mutableStateOf(UiState())
    private var composed = false

    /** A connected pair at 0x01 and 0x02, both at rest, with [drive] in force. */
    private fun pair(drive: DriveInForce) = UiState(
        connectionState = ConnectionState.CONNECTED,
        nodes = listOf(0x01, 0x02).map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, "Hoverboard") },
        attachedNode = 0x01,
        drive = drive,
        driving = drive.targets,
        telemetryByBoard = mapOf(0x01 to TelemetryUi().merge(cyclic()), 0x02 to TelemetryUi().merge(cyclic())),
    )

    private fun showState(
        state: UiState,
        showSimulateRider: Boolean = false,
        onSimulateRider: (Boolean) -> Unit = {},
    ) = compose.setContent {
        HoverboardRemoteTheme {
            Column(Modifier.verticalScroll(rememberScrollState())) {
                MachineStatusSection(state, showSimulateRider = showSimulateRider, onSimulateRider = onSimulateRider)
            }
        }
    }

    private companion object {
        /** The two boards' labels ([com.hoverboard.remote.ui.screens.nodeLabel]). */
        const val BOARD = "Hoverboard 0x01"
        const val PEER = "Hoverboard 0x02"

        /** The rover's pack: eight cells in series, which is what the gauge is scored against. */
        const val PACK_CELLS = 8

        /** The chemistry the gauge is scored against: the default. */
        const val LI_ION = Fields.BatteryChemistry.LI_ION

        /** Every fault chip this panel has ever had, or could grow, is built around this word. */
        const val FAULT_WORD = "FAULT"

        /** What each amp figure is worded with, so a test can pin that neither was drawn. */
        const val MOTOR_FIGURE = "A motor"
        const val SUPPLY_FIGURE = "A supply"
    }
}
