package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.TelemetryUi
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
 *  - `CYCLIC_STATE.fault` is built as a literal 0 on every emission and `OP_FAULT` has no emitter in
 *    the firmware at all, so a fault indicator cannot fire today. A quiet green one would tell a
 *    rider their board is healthy on the strength of a lamp wired to nothing.
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
     * The one that is not a styling choice. Nothing in the firmware can set either fault source, so
     * a fault line that renders "clear" renders a claim no code behind it can support. The panel
     * therefore draws the assertion and never the negation: silence says nothing, which is the only
     * true thing available.
     */
    @Test
    fun aBoardReportingNoFaultGetsNoFaultChipAtAll() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        // The general pin first: NOTHING on a quiet panel says FAULT. Naming the two strings we
        // happen to render today would let a future green "FAULT CLEAR" chip through, which is
        // exactly the claim this test exists to forbid. The chips are the only uppercase text here,
        // so this does not collide with the lower-case note below.
        compose.onNodeWithText(FAULT_WORD, substring = true).assertDoesNotExist()

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_fault_stop)).assertDoesNotExist()
        compose.onNodeWithText(FAULT_LEVEL_PREFIX, substring = true).assertDoesNotExist()
    }

    /**
     * ...but silence on its own is not honest either. Absence claims nothing only to a reader who
     * already knows there is no producer; to anyone else a screen with no fault line looks exactly
     * like a working fault display that happens to be clear. So a quiet panel says the reporting
     * itself is missing, the same way the battery says its number is not measured.
     */
    @Test
    fun aQuietPanelSaysThatFaultReportingDoesNotExist() {
        show(TelemetryUi().merge(cyclic(flags = CyclicState.FLAG_RIDER)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_fault_unreported))
            .assertIsDisplayed()
    }

    /** And it goes away once something IS reporting, because then it would be the false statement. */
    @Test
    fun theUnreportedNoteYieldsToAnActualFault() {
        show(TelemetryUi().merge(cyclic(flags = 0)).copy(faultStop = true))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_fault_unreported))
            .assertDoesNotExist()
    }

    @Test
    fun aStopAllEdgeIsRendered() {
        show(TelemetryUi().merge(cyclic(flags = 0)).copy(faultStop = true, faultCode = 0x11))

        compose.onNodeWithText(context.getString(R.string.telemetry_chip_fault_stop)).assertIsDisplayed()
    }

    @Test
    fun aNotifyOnlyEdgeCodeIsRendered() {
        show(TelemetryUi().merge(cyclic(flags = 0)).copy(faultCode = 0x11))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_chip_fault_code, 0x11))
            .assertIsDisplayed()
    }

    /**
     * The cyclic LEVEL is read too, even though the emitter hardcodes it to 0. The consumer half of
     * a contract whose producer is missing is worth keeping wired: whichever producer is written
     * first lights this with no change in the app.
     */
    @Test
    fun aNonZeroCyclicFaultLevelIsRendered() {
        show(TelemetryUi().merge(cyclic(flags = 0, fault = 2)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_chip_fault_level, 2))
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
     * that was never measured: there is no voltage and no bar fraction to score.
     */
    @Test
    fun anUnknownReadingHasNoVoltageAndNoBarFractionInTheModel() {
        val unknown = TelemetryUi().merge(cyclic(battery = 0))
        assertNull(unknown.batteryVolts)
        assertNull(unknown.batteryFraction)
        assertFalse(unknown.batteryLow)

        val sensed = TelemetryUi().merge(cyclic(battery = 2_900))
        assertEquals(29.0f, sensed.batteryVolts!!, 0.001f)
        assertEquals(BatteryCurve.fraction(29.0f), sensed.batteryFraction!!, 0.001f)
    }

    /** Nothing has arrived yet is the same absence, and must not read as zero volts either. */
    @Test
    fun aRecordWithNoStateYetHasNoVoltageEither() {
        assertNull(TelemetryUi().batteryVolts)
        assertNull(TelemetryUi().batteryFraction)
    }

    @Test
    fun aSensedVoltageIsShownAndScored() {
        // 29.0 V, which the 7s curve scores at 95%.
        show(TelemetryUi().merge(cyclic(battery = 2_900)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_value, 29.0f))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 95))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_unknown))
            .assertDoesNotExist()
    }

    /**
     * The number beside the speed label is a hall EDGE COUNT per window, not a speed, so the label
     * has to say so: a road speed needs `motor.pole_pairs` and a wheel diameter, and neither is a
     * registered field (`specs/link-control.md`).
     */
    @Test
    fun theWheelSpeedWordIsLabelledAsTheCountItIs() {
        show(TelemetryUi().merge(cyclic(battery = 2_900)))

        val label = context.getString(R.string.telemetry_speed)
        compose.onNodeWithText(label).assertIsDisplayed()
        assertFalse("the label must not call the count a speed", label.contains("speed", true))
    }

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
        obs = QUIET_OBS,
    )

    private fun show(telemetry: TelemetryUi, throttlePercent: Int = 0) = compose.setContent {
        HoverboardRemoteTheme {
            TelemetryPanel(telemetry = telemetry, throttlePercent = throttlePercent, armed = false)
        }
    }

    private companion object {
        /** Every fault chip this panel has ever had, or could grow, is built around this word. */
        const val FAULT_WORD = "FAULT"

        /** Enough of the level chip's text to spot it; the full string takes a format argument. */
        const val FAULT_LEVEL_PREFIX = "FAULT LEVEL"
    }
}
