package com.hoverboard.remote

import android.app.Application
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.github.takahirom.roborazzi.captureRoboImage
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.theme.DarkBackground
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode

/**
 * Roborazzi baseline screenshots of ConnectScreen and ControlScreen (house stack §Testing,
 * SPEC §12.2 layer 1). Composables are captured directly (no Activity), so no view hierarchy
 * or Koin graph is needed.
 */
@RunWith(AndroidJUnit4::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ScreenshotTest {

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
        capture("connect_scanning") {
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
        capture("connect_attaching") {
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
                ),
            )
        capture("control_armed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = telemetry,
                    armed = true,
                    throttleSpeed = 4_000,
                    engaged = true,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
            )
        }
    }

    /** The state a rider sees on arrival: connected, live telemetry, motors NOT live. */
    @Test
    fun controlScreen_connectedDisarmed() {
        val telemetry = TelemetryUi()
            .merge(CyclicState(-180, 95, 0, 3_780, 0, 0, 0))
        capture("control_disarmed") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = telemetry,
                    armed = false,
                    throttleSpeed = 0,
                    engaged = false,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun controlScreen_lowBattery() {
        // 22.0 V pack — below the 23.1 V low threshold -> batteryLow.
        val telemetry = TelemetryUi()
            .merge(CyclicState(0, 0, 0, 2_200, 0, 0, 0))
        capture("control_low_battery") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = telemetry,
                    armed = false,
                    throttleSpeed = 0,
                    engaged = false,
                ),
                onArmToggle = {},
                onThrottleMove = { _, _ -> },
                onThrottleRelease = {},
                onDisconnect = {},
            )
        }
    }

    private fun capture(name: String, content: @Composable () -> Unit) {
        captureRoboImage("src/test/screenshots/$name.png") {
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
    }
}
