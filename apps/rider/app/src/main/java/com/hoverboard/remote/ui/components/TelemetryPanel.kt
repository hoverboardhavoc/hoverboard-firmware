package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextSecondary

/**
 * Telemetry display (SPEC §10), prioritised: link health, battery, speed + throttle, current.
 *
 * @param telemetry latest decoded frame, or null before the first arrives.
 * @param throttlePercent commanded throttle percent (signed), shown beside measured speed.
 */
@Composable
fun TelemetryPanel(
    telemetry: TelemetryUi?,
    throttlePercent: Int,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(20.dp))
            .background(PanelSurface)
            .padding(20.dp),
    ) {
        Text(
            text = stringResource(R.string.telemetry_title),
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onSurface,
        )
        Spacer(modifier = Modifier.height(16.dp))

        if (telemetry == null) {
            Text(
                text = stringResource(R.string.telemetry_no_data),
                style = MaterialTheme.typography.bodyMedium,
                color = TextSecondary,
            )
            return@Column
        }

        BatterySection(telemetry)
        Spacer(modifier = Modifier.height(16.dp))
        SpeedAndThrottleRow(telemetry, throttlePercent)
        Spacer(modifier = Modifier.height(16.dp))
        CurrentRow(telemetry)
    }
}

@Composable
private fun BatterySection(telemetry: TelemetryUi) {
    val percent = BatteryCurve.percent(telemetry.batteryVolts)
    val fraction = BatteryCurve.fraction(telemetry.batteryVolts)
    val low = telemetry.batteryLow || fraction <= BatteryCurve.LOW_FRACTION
    val barColor = when {
        low -> AccentRed
        fraction <= BATTERY_WARN_FRACTION -> AccentYellow
        else -> AccentGreen
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.telemetry_battery),
            style = MaterialTheme.typography.bodyMedium,
            color = TextSecondary,
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = stringResource(R.string.telemetry_battery_value, telemetry.batteryVolts),
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.onSurface,
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(
                text = stringResource(R.string.telemetry_battery_percent, percent),
                style = MaterialTheme.typography.bodyMedium,
                color = barColor,
            )
        }
    }
    Spacer(modifier = Modifier.height(8.dp))
    BatteryBar(fraction = fraction, color = barColor)
    if (low) {
        Spacer(modifier = Modifier.height(8.dp))
        Text(
            text = stringResource(R.string.telemetry_battery_low),
            style = MaterialTheme.typography.labelLarge,
            color = AccentRed,
        )
    }
}

@Composable
private fun BatteryBar(fraction: Float, color: Color) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(10.dp)
            .clip(RoundedCornerShape(5.dp))
            .background(color.copy(alpha = TRACK_ALPHA)),
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth(fraction)
                .height(10.dp)
                .clip(RoundedCornerShape(5.dp))
                .background(color),
        )
    }
}

@Composable
private fun SpeedAndThrottleRow(telemetry: TelemetryUi, throttlePercent: Int) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        // TODO: the link `speed` field's unit/scale is still open in the firmware telemetry
        // map. Surface the raw integer for now; label/scale once the unit is pinned.
        Metric(
            label = stringResource(R.string.telemetry_speed),
            value = stringResource(R.string.telemetry_speed_value, telemetry.speedRaw),
        )
        Metric(
            label = stringResource(R.string.telemetry_throttle),
            value = stringResource(R.string.telemetry_throttle_value, throttlePercent),
        )
    }
}

@Composable
private fun CurrentRow(telemetry: TelemetryUi) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Metric(
            label = "${stringResource(R.string.telemetry_current)} A",
            value = stringResource(R.string.telemetry_current_value, telemetry.currentAmpsA),
        )
        Metric(
            label = "${stringResource(R.string.telemetry_current)} B",
            value = stringResource(R.string.telemetry_current_value, telemetry.currentAmpsB),
        )
    }
}

@Composable
private fun Metric(label: String, value: String) {
    Column {
        Text(
            text = label,
            style = MaterialTheme.typography.bodySmall,
            color = TextSecondary,
        )
        Text(
            text = value,
            style = MaterialTheme.typography.titleLarge,
            color = MaterialTheme.colorScheme.onSurface,
        )
    }
}

private const val BATTERY_WARN_FRACTION = 0.30f
private const val TRACK_ALPHA = 0.25f
