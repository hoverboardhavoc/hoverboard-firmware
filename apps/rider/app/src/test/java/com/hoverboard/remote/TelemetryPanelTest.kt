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
import org.junit.Rule
import org.junit.Test
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
 *  - `battery` is the constant `BATTERY_PLACEHOLDER_CENTIVOLT`, so every board on every rail reports
 *    36.00 V, which the pack curve then renders as a full green bar at 100%. That reading has
 *    already been believed once on the bench.
 *
 * So these tests pin absences as hard as they pin presences.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TelemetryPanelTest {

    @get:Rule
    val compose = createComposeRule()

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

    @Test
    fun theFirmwarePlaceholderVoltageIsLabelledAndNotScored() {
        show(TelemetryUi().merge(cyclic(battery = TelemetryUi.BATTERY_PLACEHOLDER_CENTIVOLT)))

        // The number is still shown: hiding the evidence is not the same as labelling it.
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_value, 36.0f))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_placeholder))
            .assertIsDisplayed()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_placeholder_note))
            .assertIsDisplayed()
        // And the state-of-charge judgement is withheld. A "100%" beside 36.0 V is precisely what
        // read as a healthy pack on the bench.
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, FULL_PERCENT))
            .assertDoesNotExist()
    }

    /**
     * The bar is drawn EMPTY, not merely grey.
     *
     * Neutralising the colour and leaving the fill was the half-measure: `BatteryCurve` tops out at
     * 29.4 V, so the 36.00 V placeholder clamps to 1.0 and the bar runs the full width. A full grey
     * bar still says full, and the fullness was half of what read as a healthy pack on the bench.
     */
    @Test
    fun thePlaceholderDrawsAnEmptyBarAndNotJustAGreyOne() {
        val placeholder = TelemetryUi()
            .merge(cyclic(battery = TelemetryUi.BATTERY_PLACEHOLDER_CENTIVOLT))
        // What the naive rendering would have drawn, and why grey alone was not enough.
        assertEquals(1f, BatteryCurve.fraction(placeholder.batteryVolts), 0.001f)
        assertEquals(0f, placeholder.batteryFraction, 0.001f)

        val measured = TelemetryUi().merge(cyclic(battery = 2_900))
        assertEquals(
            BatteryCurve.fraction(measured.batteryVolts),
            measured.batteryFraction,
            0.001f,
        )
    }

    @Test
    fun aVoltageThatIsNotThePlaceholderIsScoredNormally() {
        // 29.0 V on the 7s curve. Nothing sends this today; the branch exists so the tag retires
        // itself the day a sensing task puts a measured word on the wire.
        show(TelemetryUi().merge(cyclic(battery = 2_900)))

        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_placeholder))
            .assertDoesNotExist()
        compose
            .onNodeWithText(context.getString(R.string.telemetry_battery_percent, 95))
            .assertIsDisplayed()
    }

    private fun cyclic(
        flags: Int = 0,
        fault: Int = 0,
        battery: Int = TelemetryUi.BATTERY_PLACEHOLDER_CENTIVOLT,
    ) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = battery,
        mode = 2,
        fault = fault,
        flags = flags,
    )

    private fun show(telemetry: TelemetryUi) = compose.setContent {
        HoverboardRemoteTheme {
            TelemetryPanel(telemetry = telemetry, throttlePercent = 0, armed = false)
        }
    }

    private companion object {
        /** Enough of the level chip's text to spot it; the full string takes a format argument. */
        const val FAULT_LEVEL_PREFIX = "FAULT LEVEL"
        const val FULL_PERCENT = 100
    }
}
