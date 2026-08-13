package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.ui.components.ArmToggle
import com.hoverboard.remote.ui.components.TelemetryPanel
import com.hoverboard.remote.ui.components.ThrottlePad
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Main control screen: the armed banner, the telemetry panel, the arm toggle and the throttle.
 *
 * Laid out for ONE thumb. The throttle is full width and takes all the remaining height, because it
 * is the control being modulated and the only one that is held; the arm toggle is a tap above it.
 * An earlier version put a held arm pad beside the throttle and it was wrong: the throttle is
 * already occupying the hand, so a second sustained touch just costs the rider their other hand.
 * See [com.hoverboard.remote.MainViewModel] for the safety argument.
 *
 * The throttle is disabled outright while disarmed, so the pad cannot show travel that nothing is
 * being asked to perform.
 */
@Composable
fun ControlScreen(
    state: UiState,
    onArmToggle: () -> Unit,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
    onDisconnect: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp),
    ) {
        Header(connected = state.isConnected, onDisconnect = onDisconnect)
        Spacer(modifier = Modifier.height(12.dp))

        ArmedBanner(armed = state.armed)
        Spacer(modifier = Modifier.height(12.dp))

        TelemetryPanel(
            telemetry = state.telemetry,
            throttlePercent = state.throttlePercent,
        )

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
            onMove = onThrottleMove,
            onRelease = onThrottleRelease,
            modifier = Modifier.fillMaxWidth().weight(1f),
        )
    }
}

/**
 * The unmistakable part: a full-width bar that says, in as few words as possible, whether this
 * machine's motors are live. Red and filled when armed, flat and grey when not.
 */
@Composable
private fun ArmedBanner(armed: Boolean) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(12.dp))
            .background(if (armed) AccentRed else PanelSurface)
            .padding(vertical = 10.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = if (armed) {
                stringResource(R.string.arm_banner_armed)
            } else {
                stringResource(R.string.arm_banner_disarmed)
            },
            style = MaterialTheme.typography.titleMedium,
            fontWeight = FontWeight.Bold,
            textAlign = TextAlign.Center,
            color = if (armed) TextPrimary else ZeroLine,
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
