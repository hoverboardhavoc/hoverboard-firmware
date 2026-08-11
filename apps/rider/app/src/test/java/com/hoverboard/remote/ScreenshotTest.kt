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
import com.hoverboard.remote.link.Telemetry
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
    fun controlScreen_connectedWithTelemetry() {
        // 37.80 V pack, speed 850 (raw), wheel A 3.20 A, wheel B 2.80 A.
        val telemetry = TelemetryUi()
            .merge(Telemetry(motorIndex = 0, batteryMv = 37_800, currentCa = 320, speed = 850, faultCode = 0, flags = 0))
            .merge(Telemetry(motorIndex = 1, batteryMv = 37_800, currentCa = 280, speed = 850, faultCode = 0, flags = 0))
        capture("control_connected") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = telemetry,
                    throttleSpeed = 200,
                    engaged = true,
                ),
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
            .merge(Telemetry(motorIndex = 0, batteryMv = 22_000, currentCa = 0, speed = 0, faultCode = 0, flags = 0))
        capture("control_low_battery") {
            ControlScreen(
                state = UiState(
                    connectionState = ConnectionState.CONNECTED,
                    telemetry = telemetry,
                    throttleSpeed = 0,
                    engaged = false,
                ),
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
