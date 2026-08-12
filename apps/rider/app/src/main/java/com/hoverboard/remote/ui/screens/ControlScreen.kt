package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
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
import com.hoverboard.remote.ui.components.ArmPad
import com.hoverboard.remote.ui.components.TelemetryPanel
import com.hoverboard.remote.ui.components.ThrottlePad
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Main control screen: the armed banner, the telemetry panel, and the two held controls.
 *
 * The two controls sit at opposite edges so they are a two-thumb posture rather than something one
 * hand can cover: **arm** on the left, **throttle** on the right, both full height so neither needs
 * to be aimed at. Nothing moves unless both are held; see [com.hoverboard.remote.MainViewModel] for
 * why the scheme is shaped this way.
 *
 * The throttle is the wider of the two because it is the one being modulated. The arm control only
 * has to be held, so it needs to be easy to keep a thumb on, not precise.
 */
@Composable
fun ControlScreen(
    state: UiState,
    onArmPress: () -> Unit,
    onArmRelease: () -> Unit,
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

        Row(modifier = Modifier.fillMaxWidth().weight(1f)) {
            ArmPad(
                armed = state.armed,
                // Stays enabled while ARMED even though [UiState.canArm] goes false the moment the
                // throttle is touched. `enabled` is the pointerInput key, so flipping it mid-hold
                // would cancel the in-flight gesture, run ArmPad's finally, and disarm the board
                // the instant the rider reached for the throttle. canArm gates STARTING an arm;
                // it must never interrupt one.
                enabled = state.canArm || state.armed,
                onPress = onArmPress,
                onRelease = onArmRelease,
                modifier = Modifier.weight(ARM_WEIGHT).fillMaxHeight(),
            )
            Spacer(modifier = Modifier.width(12.dp))
            ThrottlePad(
                speed = state.throttleSpeed,
                engaged = state.engaged,
                enabled = state.isConnected,
                onMove = onThrottleMove,
                onRelease = onThrottleRelease,
                modifier = Modifier.weight(THROTTLE_WEIGHT).fillMaxHeight(),
            )
        }
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

/** The throttle gets the larger share: it is modulated, the arm control is only held. */
private const val ARM_WEIGHT = 1f
private const val THROTTLE_WEIGHT = 1.6f
