package com.hoverboard.remote

import android.app.Application
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onRoot
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.github.takahirom.roborazzi.captureRoboImage
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.theme.DarkBackground
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode

/**
 * Roborazzi baseline screenshots of ConnectScreen and ControlScreen (house stack §Testing,
 * SPEC §12.2 layer 1). Composables are captured directly (no Activity), so no view hierarchy
 * or Koin graph is needed.
 *
 * Captures go through a compose test rule, not the standalone `captureRoboImage { }` overload.
 * The standalone overload syncs through Espresso, whose Robolectric idle loop never sees the
 * main looper idle while the busy spinner's infinite transition (ConnectScreen's
 * StatusIndicator) keeps scheduling frames, so the scanning and attaching cases spun until the
 * test JVM ran out of heap. Under the rule those cases pause the test clock and advance it a
 * fixed [SPINNER_PHASE_MS], so the spinner is captured at the same visible phase every run.
 */
@RunWith(AndroidJUnit4::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ScreenshotTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    @Test
    fun connectScreen_disconnected() {
        capture("connect_disconnected") {
            ConnectScreen(
                connectionState = ConnectionState.DISCONNECTED,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_scanning() {
        capture("connect_scanning", advanceMs = SPINNER_PHASE_MS) {
            ConnectScreen(
                connectionState = ConnectionState.SCANNING,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_attaching() {
        capture("connect_attaching", advanceMs = SPINNER_PHASE_MS) {
            ConnectScreen(
                connectionState = ConnectionState.ATTACHING,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_attachFailed() {
        capture("connect_attach_failed") {
            ConnectScreen(
                connectionState = ConnectionState.ATTACH_FAILED,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun controlScreen_connectedWithTelemetry() {
        // 37.80 V pack, speed 850 (raw), wheel A 3.20 A, wheel B 2.80 A.
        val telemetry = TelemetryUi()
            // battery is centivolts: 3780 cV = 37.8 V.
            .merge(
                CyclicState(
                    pitch = -180,
                    roll = 95,
                    wheelSpeed = 850,
                    battery = 3_780,
                    mode = 2,
                    fault = 0,
                    flags = CyclicState.FLAG_RIDER,
                    obs = QUIET_OBS,
                ),
            )
        capture("control_armed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    telemetry = telemetry,
                    armed = true,
                    throttleSpeed = 4_000,
                    engaged = true,
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

    /**
     * Two boards bound (`specs/rider-ui.md` 3.2), armed and driving: the master's chip confirmed by
     * its telemetry, the slave's outlined and saying only what it is commanded, and the waived
     * rider requirement stated on the arm control (`specs/control.md` (i)).
     */
    @Test
    fun controlScreen_boundArmed() {
        val telemetry = TelemetryUi().merge(CyclicState(-40, 10, 300, 3_780, 2, 0, 0, QUIET_OBS))
        capture("control_bound_armed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    slaveBoard = 0x02,
                    driveMode = DriveMode.BOUND,
                    riderWaiver = RiderWaiver.WAIVED,
                    telemetry = telemetry,
                    armed = true,
                    throttleSpeed = 6_000,
                    engaged = true,
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

    /**
     * DIFFERENTIAL (`specs/rider-ui.md` 3.2), armed and turning: the two-axis stick in place of the
     * throttle pad, its thumb off centre on both axes, and both boards driven. The demand drawn is
     * the mix's `v`; what each board is told is the pair the app sends, not a number on this screen.
     */
    @Test
    fun controlScreen_differentialArmed() {
        val telemetry = TelemetryUi().merge(CyclicState(-40, 10, 300, 3_780, 2, 0, 0, QUIET_OBS))
        capture("control_differential_armed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    slaveBoard = 0x02,
                    driveMode = DriveMode.DIFFERENTIAL,
                    telemetry = telemetry,
                    armed = true,
                    throttleSpeed = 9_000,
                    steer = 14_000,
                    engaged = true,
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

    /** The state a rider sees on arrival: connected, live telemetry, motors NOT live. */
    @Test
    fun controlScreen_connectedDisarmed() {
        val telemetry = TelemetryUi()
            .merge(CyclicState(-180, 95, 0, 3_780, 0, 0, 0, QUIET_OBS))
        capture("control_disarmed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    telemetry = telemetry,
                    armed = false,
                    throttleSpeed = 0,
                    engaged = false,
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

    /**
     * What a board reporting UNKNOWN for its pack looks like, which is every slave, any master
     * without `board.vbatt`, and any master whose motor has not been brought up: the battery row
     * says there is no reading and draws no number and no bar. The lockdown and fault chips are
     * forced on here because the firmware cannot raise either of them yet, and a rendering nothing
     * can exercise is a rendering nobody has looked at.
     */
    @Test
    fun controlScreen_unknownBatteryAndAlarms() {
        val telemetry = TelemetryUi()
            .merge(
                CyclicState(
                    pitch = 120,
                    roll = -40,
                    wheelSpeed = 0,
                    battery = 0,
                    mode = 2,
                    fault = 0,
                    flags = CyclicState.FLAG_RIDER or CyclicState.FLAG_LOCKDOWN,
                    obs = QUIET_OBS,
                ),
            )
            .copy(faultStop = true, faultCode = 0x11)
        capture("control_unknown_battery_and_alarms") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    telemetry = telemetry,
                    armed = false,
                    simulateRider = true,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onJoystickMove = { _, _, _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
                onSimulateRider = {},
                onDriveMode = {},
                // The one capture that carries the bench control, because it is the one showing
                // what a debug build on a bench board looks like. The rider-facing captures above
                // pass false: a release build does not have the row at all.
                showSimulateRider = true,
            )
        }
    }

    @Test
    fun controlScreen_lowBattery() {
        // 22.0 V pack, below the 23.1 V low threshold -> batteryLow.
        val telemetry = TelemetryUi()
            .merge(CyclicState(0, 0, 0, 2_200, 0, 0, 0, QUIET_OBS))
        capture("control_low_battery") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    masterBoard = 0x01,
                    telemetry = telemetry,
                    armed = false,
                    throttleSpeed = 0,
                    engaged = false,
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

    /**
     * With [advanceMs] set, the test clock is paused (auto-advance off, which also lets the
     * infinite spinner run) and stepped exactly that far before the capture.
     */
    private fun capture(name: String, advanceMs: Long? = null, content: @Composable () -> Unit) {
        if (advanceMs != null) compose.mainClock.autoAdvance = false
        compose.setContent {
            HoverboardRemoteTheme {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(DarkBackground),
                ) {
                    content()
                }
            }
        }
        if (advanceMs != null) compose.mainClock.advanceTimeBy(advanceMs)
        compose.onRoot().captureRoboImage("src/test/screenshots/$name.png")
    }

    private companion object {
        /** Spinner phase for the busy screenshots: far enough in to draw a visible arc. */
        const val SPINNER_PHASE_MS = 500L
    }
}
