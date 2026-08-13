package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.ui.components.ArmToggle
import com.hoverboard.remote.ui.components.TelemetryPanel
import com.hoverboard.remote.ui.components.ThrottlePad
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextSecondary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Main control screen: the telemetry panel, the arm toggle and the throttle.
 *
 * Laid out for ONE thumb. The throttle is full width and takes all the remaining height, because it
 * is the control being modulated and the only one that is held; the arm toggle is a tap above it.
 * An earlier version put a held arm pad beside the throttle and it was wrong: the throttle is
 * already occupying the hand, so a second sustained touch just costs the rider their other hand.
 * See [com.hoverboard.remote.MainViewModel] for the safety argument.
 *
 * The throttle is disabled outright while disarmed, so the pad cannot show travel that nothing is
 * being asked to perform.
 *
 * ## One arm surface, not two
 *
 * This screen used to carry a full-width ARMED banner above the telemetry as well as the
 * [ArmToggle]. Both read `armed` off the same [UiState], so the banner held no state the toggle
 * lacked and could say nothing the toggle could not; it was one more thing to keep in agreement for
 * no information. It is gone, and [ArmToggle] is the single arm control and indicator.
 *
 * What the banner was actually buying was GLANCEABILITY: armed state legible from anywhere on the
 * screen, not just from the control at the bottom. That is kept without a second textual element by
 * tinting the two big surfaces instead. While armed, the telemetry panel and the throttle pad both
 * carry a red outline, so the armed state reads from the top of the screen, the middle, and the
 * control itself.
 */
@Composable
fun ControlScreen(
    state: UiState,
    onArmToggle: () -> Unit,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
    onDisconnect: () -> Unit,
    onSimulateRider: (Boolean) -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp),
    ) {
        Header(connected = state.isConnected, onDisconnect = onDisconnect)
        Spacer(modifier = Modifier.height(12.dp))

        TelemetryPanel(
            telemetry = state.telemetry,
            throttlePercent = state.throttlePercent,
            armed = state.armed,
        )

        Spacer(modifier = Modifier.height(8.dp))

        SimulateRiderRow(on = state.simulateRider, onChange = onSimulateRider)

        Spacer(modifier = Modifier.height(12.dp))

        Text(
            text = when {
                !state.isConnected -> stringResource(R.string.telemetry_disconnected)
                state.armed -> stringResource(R.string.throttle_hint_armed)
                else -> stringResource(R.string.throttle_hint_disarmed)
            },
            style = MaterialTheme.typography.bodyMedium,
            color = if (state.isConnected) TextSecondary else AccentRed,
        )

        Spacer(modifier = Modifier.height(8.dp))

        ArmToggle(
            armed = state.armed,
            enabled = state.canArm,
            onToggle = onArmToggle,
        )

        Spacer(modifier = Modifier.height(12.dp))

        ThrottlePad(
            speed = state.throttleSpeed,
            engaged = state.engaged,
            enabled = state.isConnected && state.armed,
            armed = state.armed,
            onMove = onThrottleMove,
            onRelease = onThrottleRelease,
            modifier = Modifier.fillMaxWidth().weight(1f),
        )
    }
}

/**
 * The bench affordance that stands in for foot pads.
 *
 * The app no longer asserts the `INPUTS` rider bit as a copy of the arm level, because the boards
 * have real pads and a remote that claims a rider pins the gain profile regardless of them
 * ([com.hoverboard.remote.model.RiderCommand.inputs]). A bench board with no pads wired reads no
 * rider at all, though, so this is the switch that puts the level back on the wire deliberately.
 *
 * It is a small, dim row rather than a prominent control: it is not part of riding, and it is off
 * every time the app starts.
 */
@Composable
private fun SimulateRiderRow(on: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clickable { onChange(!on) }
            .testTag(SIM_RIDER_TAG)
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.sim_rider_label),
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
        )
        Text(
            text = if (on) {
                stringResource(R.string.sim_rider_on)
            } else {
                stringResource(R.string.sim_rider_off)
            },
            style = MaterialTheme.typography.labelMedium,
            color = if (on) AccentGreen else ZeroLine,
        )
    }
}

@Composable
private fun Header(connected: Boolean, onDisconnect: () -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.control_title),
            style = MaterialTheme.typography.headlineSmall,
            color = MaterialTheme.colorScheme.onBackground,
        )
        if (connected) {
            OutlinedButton(onClick = onDisconnect) {
                Text(stringResource(R.string.connect_disconnect))
            }
        }
    }
}

/** Test tag for the bench rider-simulation row. */
const val SIM_RIDER_TAG = "simulate_rider"
