package com.hoverboard.remote.ui.screens

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
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.ui.components.TelemetryPanel
import com.hoverboard.remote.ui.components.ThrottlePad
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextSecondary

/**
 * Main control screen (SPEC §8.1, §9, §10): the deadman throttle plus the telemetry panel.
 *
 * The throttle is disabled (and the ViewModel forces 0) when the link is not connected, so
 * motion is impossible without a live connection (SPEC §11).
 */
@Composable
fun ControlScreen(
    state: UiState,
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

        TelemetryPanel(
            telemetry = state.telemetry,
            throttlePercent = state.throttlePercent,
        )

        Spacer(modifier = Modifier.height(16.dp))

        Text(
            text = if (state.isConnected) {
                stringResource(R.string.throttle_hint_release)
            } else {
                stringResource(R.string.telemetry_disconnected)
            },
            style = MaterialTheme.typography.bodyMedium,
            color = if (state.isConnected) TextSecondary else AccentRed,
        )

        Spacer(modifier = Modifier.height(8.dp))

        ThrottlePad(
            speed = state.throttleSpeed,
            engaged = state.engaged,
            enabled = state.isConnected,
            onMove = onThrottleMove,
            onRelease = onThrottleRelease,
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
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
