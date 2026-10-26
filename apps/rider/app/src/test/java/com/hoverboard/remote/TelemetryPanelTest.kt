package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.batteryReading
import com.hoverboard.remote.model.currentReading
import com.hoverboard.remote.ui.components.TelemetryPanel
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the telemetry panel says about state it has always decoded and never shown, and, more
 * importantly, what it refuses to say.
 *
 * Two of the values it now renders are known-wrong in the firmware, and each is wrong in a way that
 * a naive rendering would launder into a reassurance:
 *
 *  - `CYCLIC_STATE.fault` is the reporting board's OWN producer mask and the other board's state
 *    is not in front of the rider, so a quiet green indicator would tell a rider their machine is
 *    healthy on the strength of one board's word.
 *  - `battery` is 0 for UNKNOWN, which is the firmware declining to report rather than a
 *    measurement, and which is broader than "cannot sense the rail": a master whose `board.vbatt`
 *    is staged reports it too until its motor is brought up. Rendered as a number it was 0.00 V,
 *    which on a rider's screen is a flat pack.
 *
 * So these tests pin absences as hard as they pin presences.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TelemetryPanelTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()

    @Test
    fun riderChipIsDrawnWhenTheBoardSeesARider() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_rider)).assertIsDisplayed()
    }

    @Test
    fun riderChipIsDrawnWhenItDoesNotToo() {
        // A board reporting no rider IS reporting something: its pads read low. Both values mean
        // something, so unlike the alarms this chip is drawn lit or dim rather than hidden.
        show(TelemetryUi().merge(cyclic(flags = 0)))

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_rider)).assertIsDisplayed()
    }

    @Test
    fun noLockdownChipWhileTheBoardIsNotInLockdown() {
        show(TelemetryUi().merge(cyclic(flags = 0)))

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_lockdown)).assertDoesNotExist()
    }

    @Test
    fun lockdownChipAppearsWhenTheBoardAssertsIt() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_LOCKDOWN)))

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_lockdown)).assertIsDisplayed()
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

        compose.onNodeWithText(FAULT_LEVEL_PREFIX, substring = true).assertDoesNotExist()
    }

    /**
     * ...but silence on its own says too little. Absence claims nothing only to a reader who
     * already knows what the byte covers; to anyone else a screen with no fault line looks exactly
     * like a working whole-machine fault display that happens to be clear. So a quiet panel says
     * what its silence does not cover, the same way the battery says its number is not measured.
     */
    @Test
    fun aQuietPanelSaysWhatItsSilenceDoesNotCover() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_fault_unreported))
            .assertIsDisplayed()
    }

    /** And it goes away once something IS reporting, because then it would be the false statement. */
    @Test
    fun theUnreportedNoteYieldsToAnActualFault() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 2)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_fault_unreported))
            .assertDoesNotExist()
    }

    /**
     * The cyclic fault byte is the board's producer level mask, and a fault-A bit in it (here
     * bit 1, `EV_STOP_ALL`) lights the chip with the masked level.
     */
    @Test
    fun aNonZeroCyclicFaultLevelIsRendered() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 2)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_chip_fault_level, 2))
            .assertIsDisplayed()
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

    /** And a fault-A bit beside them is rendered with the non-fault bits stripped. */
    @Test
    fun theFaultLevelIsRenderedWithTheNonFaultBitsStripped() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 0x81)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_chip_fault_level, 1))
            .assertIsDisplayed()
    }

    /**
     * A board that sends `battery = 0` is a board reporting UNKNOWN, and the panel says exactly
     * that and no more: no voltage, no percent, no bar. The number this replaced was 0.00 V, which
     * claimed a measurement the board never made and read as a flat pack while doing it.
     */
    @Test
    fun aBoardReportingUnknownGetsNoReadingAtAll() {
        // A throttle of 42% so the one percent this panel legitimately shows cannot be mistaken
        // for a state of charge: both render through "%1$d%%", so at 0% they are the same string.
        show(TelemetryUi().merge(cyclic(battery = 0)), throttlePercent = 42)

        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_unknown))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_unknown_note))
            .assertIsDisplayed()

        // The two renderings this replaces: a voltage, and a state of charge scored off it.
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_value, 0.0f))
            .assertDoesNotExist()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 0))
            .assertDoesNotExist()
        // And nothing claims the pack is low, which an absent reading must not do either.
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_low))
            .assertDoesNotExist()
    }

    /**
     * The absence is in the model, not only in the panel, so nothing downstream can score a pack
     * that was never measured: there is no voltage, and the reading that pairs it with the pack
     * says the board is reporting nothing.
     */
    @Test
    fun anUnknownReadingHasNoVoltageAndNoChargeInTheModel() {
        val unknown = TelemetryUi().merge(cyclic(battery = 0))
        assertNull(unknown.batteryVolts)
        assertEquals(BatteryReading.NoReading, batteryReading(unknown, PACK_CELLS))

        val sensed = TelemetryUi().merge(cyclic(battery = 2_900))
        assertEquals(29.0f, sensed.batteryVolts!!, 0.001f)
        val charge = batteryReading(sensed, PACK_CELLS) as BatteryReading.Charge
        assertEquals(BatteryCurve.fraction(29.0f, PACK_CELLS), charge.fraction, 0.001f)
    }

    /** Nothing has arrived yet is the same absence, and must not read as zero volts either. */
    @Test
    fun aRecordWithNoStateYetHasNoVoltageEither() {
        assertNull(TelemetryUi().batteryVolts)
        assertEquals(BatteryReading.NoReading, batteryReading(TelemetryUi(), PACK_CELLS))
    }

    @Test
    fun aSensedVoltageIsShownAndScoredAgainstTheStatedPack() {
        // 29.0 V on eight cells: 24.0 V empty, 33.6 V full, so 52%.
        show(TelemetryUi().merge(cyclic(battery = 2_900)), packCells = PACK_CELLS)

        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_value, 29.0f))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 52))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_unknown))
            .assertDoesNotExist()
    }

    /**
     * The same voltage with no pack stated: the number stands, the percentage does not.
     *
     * A percent needs the cell count, which arrives from the board's store and not on the wire, so
     * an unstated pack has no state of charge to show. The curve it used to be scored against was
     * a seven-cell one compiled into the app, which read 29.0 V as a full pack on both machines
     * here (`specs/todo.md`, the `BatteryCurve` entry).
     */
    @Test
    fun aSensedVoltageWithNoStatedPackIsShownWithNoPercentage() {
        show(TelemetryUi().merge(cyclic(battery = 2_900)), packCells = null)

        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_value, 29.0f))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_no_pack))
            .assertExists()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 52))
            .assertDoesNotExist()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 100))
            .assertDoesNotExist()
    }

    /**
     * The number beside the speed label is a hall EDGE COUNT per window, not a speed, so the label
     * has to say so: a road speed needs the motor's pole-pair count and the wheel's diameter,
     * registered fields that default to unset, so no board carries them
     * (`specs/link-control.md`, the payload shapes).
     */
    @Test
    fun theWheelSpeedWordIsLabelledAsTheCountItIs() {
        show(TelemetryUi().merge(cyclic(battery = 2_900)))

        val label = context.getString(R.string.telemetry_speed)
        compose.onNodeWithText(label).assertIsDisplayed()
        assertFalse("the label must not call the count a speed", label.contains("speed", true))
    }

    /**
     * The current display (`specs/rider-ui.md` 3.6): both figures, each worded for WHICH current
     * it is. The two differ by the duty and answer different questions, and a panel showing one
     * unlabelled number is how a supply reading got taken for a phase current on 2026-10-09.
     */
    @Test
    fun bothCurrentFiguresAreDrawnAndEachSaysWhichOneItIs() {
        // 910 counts at 455 counts per amp is 2.0 A at the motor, half-duty is 1.0 A at the supply.
        show(TelemetryUi().merge(cyclic()), current = amps(mean = 910, duty = CyclicObs.DUTY_FULL / 2))

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
        show(TelemetryUi().merge(cyclic()), current = amps(mean = 910, duty = CyclicObs.DUTY_FULL))

        compose.onNodeWithText(context.getString(R.string.telemetry_current_note)).assertIsDisplayed()
    }

    /** Nothing has arrived from the board, which is not the same as a board carrying no current. */
    @Test
    fun aBoardNothingHasArrivedFromSaysSoAndShowsNoAmps() {
        show(TelemetryUi().merge(cyclic()), current = CurrentReading.Silent)

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
        show(TelemetryUi().merge(cyclic()), current = CurrentReading.MotorsOff)

        compose.onNodeWithText(context.getString(R.string.telemetry_current_motors_off)).assertIsDisplayed()
        assertNoFigures()
    }

    /** A period that drove nothing: the magnitude it measured is the sense chain's own noise. */
    @Test
    fun aCoastingBoardSaysSoAndShowsNoAmps() {
        show(TelemetryUi().merge(cyclic()), current = CurrentReading.Coasting)

        compose.onNodeWithText(context.getString(R.string.telemetry_current_coasting)).assertIsDisplayed()
        assertNoFigures()
    }

    /** No scale yet: amps at the registry default would be some other board's constant. */
    @Test
    fun aBoardWhoseCalibrationIsUnreadSaysSoAndShowsNoAmps() {
        show(TelemetryUi().merge(cyclic()), current = CurrentReading.Unscaled)

        compose.onNodeWithText(context.getString(R.string.telemetry_current_unscaled)).assertIsDisplayed()
        assertNoFigures()
    }

    /** One board in the session, one row: the second is for a pair, and so is the total. */
    @Test
    fun aSessionWithOneBoardDrawsOneRowAndNoTotal() {
        show(TelemetryUi().merge(cyclic()), current = amps(mean = 910, duty = CyclicObs.DUTY_FULL))

        compose.onNodeWithText(PEER).assertDoesNotExist()
        compose.onNodeWithText(context.getString(R.string.telemetry_current_machine)).assertDoesNotExist()
    }

    /**
     * The pair's total, which is the figure a bench supply's own reading is compared against: the
     * supply feeds both boards, so one board's figure is only a share of what it shows.
     */
    @Test
    fun aPairAddsItsTwoSupplyFiguresIntoAMachineTotal() {
        show(
            TelemetryUi().merge(cyclic()),
            current = amps(mean = 910, duty = CyclicObs.DUTY_FULL / 2),
            peerCurrent = amps(mean = 455, duty = CyclicObs.DUTY_FULL / 2),
        )

        compose
            .onNodeWithText(context.getString(R.string.telemetry_current_machine_value, 1.5f))
            .assertIsDisplayed()
    }

    /** And says so rather than falling back to the one figure it has. */
    @Test
    fun aPairWithOneBoardNotReportingShowsNoTotal() {
        show(
            TelemetryUi().merge(cyclic()),
            current = amps(mean = 910, duty = CyclicObs.DUTY_FULL),
            peerCurrent = CurrentReading.Silent,
        )

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
        show(
            TelemetryUi().merge(cyclic()),
            current = amps(mean = 910, duty = CyclicObs.DUTY_FULL),
            peerCurrent = CurrentReading.Silent,
        )

        compose.onNodeWithText(BOARD).assertIsDisplayed()
        compose.onNodeWithText(PEER).assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.telemetry_current_silent)).assertIsDisplayed()
    }

    /** No amp figure of either kind is on the screen. */
    private fun assertNoFigures() {
        compose.onNodeWithText(MOTOR_FIGURE, substring = true).assertDoesNotExist()
        compose.onNodeWithText(SUPPLY_FIGURE, substring = true).assertDoesNotExist()
    }

    /** What [currentReading] makes of a board at [mean] counts and [duty], at 455 counts per amp. */
    private fun amps(mean: Int, duty: Int): CurrentReading = currentReading(
        TelemetryUi().merge(cyclic().copy(obs = QUIET_OBS.copy(phaseMean = mean, dutyOn = duty))),
        countsPerAmp = 455,
    )

    private fun cyclic(
        flags: Int = 0,
        fault: Int = 0,
        // A sensed 29.0 V, so a case that is not about the battery still renders the full
        // reading rather than the no-reading branch.
        battery: Int = 2_900,
    ) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = battery,
        mode = 2,
        fault = fault,
        flags = flags,
        peerDemand = 0,
        wheelPos = 0,
        obs = QUIET_OBS,
    )

    private fun show(
        telemetry: TelemetryUi,
        throttlePercent: Int = 0,
        packCells: Int? = PACK_CELLS,
        current: CurrentReading = CurrentReading.Silent,
        peerCurrent: CurrentReading? = null,
    ) = compose.setContent {
        HoverboardRemoteTheme {
            TelemetryPanel(
                telemetry = telemetry,
                throttlePercent = throttlePercent,
                armed = false,
                battery = batteryReading(telemetry, packCells),
                currents = listOfNotNull(BOARD to current, peerCurrent?.let { PEER to it }),
            )
        }
    }

    private companion object {
        /** The two boards' row labels, as the Ride screen names them ([com.hoverboard.remote.ui.screens.nodeLabel]). */
        const val BOARD = "Hoverboard 0x01"
        const val PEER = "Hoverboard 0x02"

        /** The rover's pack: eight cells in series, which is what the gauge is scored against. */
        const val PACK_CELLS = 8

        /** Every fault chip this panel has ever had, or could grow, is built around this word. */
        const val FAULT_WORD = "FAULT"

        /** Enough of the level chip's text to spot it; the full string takes a format argument. */
        const val FAULT_LEVEL_PREFIX = "FAULT LEVEL"

        /** What each amp figure is worded with, so a test can pin that neither was drawn. */
        const val MOTOR_FIGURE = "A motor"
        const val SUPPLY_FIGURE = "A supply"
    }
}
