package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
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
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.ui.theme.Caution
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary

/**
 * Pack voltage, state of charge and the charge bar: three rows that each depend on one more fact
 * than the last, so each is drawn only when that fact is in hand.
 *
 * `battery = 0` on the wire is UNKNOWN ([TelemetryUi.batteryVolts] has the firmware side). Drawn
 * as a number that was 0.00 V, and a 0.00 V on a rider's screen is a flat pack, which is a reading
 * and an alarming one. So the absent case draws no number, no percent and no bar, and says in
 * words that there is no reading, without claiming to know why there is not.
 *
 * A voltage is not a charge either. How full a pack is depends on how many cells are in it, which
 * no frame carries and nothing can work out from the voltage, so a machine whose pack has not been
 * stated gets its volts and a line saying why there is no percentage, rather than a gauge drawn
 * against an assumed pack.
 */
@Composable
internal fun BatterySection(battery: BatteryReading) {
    when (battery) {
        BatteryReading.NoReading -> UnknownBattery()
        is BatteryReading.Unjudged -> UnjudgedBattery(battery)
        is BatteryReading.Charge -> ChargedBattery(battery)
    }
}

/** The battery row of a board whose pack voltage is known and whose pack is stated. */
@Composable
private fun ChargedBattery(battery: BatteryReading.Charge) {
    val barColor = when {
        battery.low -> Caution
        else -> Primary
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.telemetry_battery),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = stringResource(R.string.telemetry_battery_value, battery.volts),
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.onSurface,
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(
                text = stringResource(R.string.telemetry_battery_percent, battery.percent),
                style = MaterialTheme.typography.bodyMedium,
                color = barColor,
            )
        }
    }
    Spacer(modifier = Modifier.height(8.dp))
    BatteryBar(fraction = battery.fraction, color = barColor)
    if (battery.low) {
        Spacer(modifier = Modifier.height(8.dp))
        CautionText(
            text = stringResource(R.string.telemetry_battery_low),
            style = MaterialTheme.typography.labelLarge,
        )
    }
}

/**
 * The battery row of a machine whose pack has not been stated, or stated as a chemistry this app
 * has no table for: the measured voltage, and a line saying what is missing and where it is set.
 *
 * No percent and no bar, for the reason the unknown row has none: both are judgements about how
 * full the pack is, and nothing here knows how full a pack this voltage makes. The voltage itself
 * is a measurement the board made and is shown.
 */
@Composable
private fun UnjudgedBattery(battery: BatteryReading.Unjudged) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.telemetry_battery),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
        Text(
            text = stringResource(R.string.telemetry_battery_value, battery.volts),
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onSurface,
        )
    }
    Spacer(modifier = Modifier.height(8.dp))
    Text(
        text = stringResource(
            when (battery.why) {
                BatteryReading.Unjudged.Why.NO_CELLS -> R.string.telemetry_battery_no_pack
                BatteryReading.Unjudged.Why.NO_CHEMISTRY -> R.string.telemetry_battery_no_chemistry
                BatteryReading.Unjudged.Why.UNKNOWN_CHEMISTRY -> R.string.telemetry_battery_unknown_chemistry
            },
        ),
        style = MaterialTheme.typography.bodySmall,
        color = FgMuted,
    )
}

/**
 * The battery row of a board reporting UNKNOWN: the label, the words where the reading would be,
 * and a line saying what the app does and does not know. No number, no percent, no bar, because
 * each of those would be a claim about a pack the board has not reported.
 *
 * The note names both causes as alternatives rather than picking one. The word on the wire does not
 * say which, and asserting "it does not sense the rail" at a part-configured master whose
 * `board.battery.pin` is already staged sends its operator to re-stage a field that was right.
 */
@Composable
private fun UnknownBattery() {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.telemetry_battery),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
        Text(
            text = stringResource(R.string.telemetry_battery_unknown),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
    }
    Spacer(modifier = Modifier.height(8.dp))
    Text(
        text = stringResource(R.string.telemetry_battery_unknown_note),
        style = MaterialTheme.typography.bodySmall,
        color = FgMuted,
    )
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

private const val TRACK_ALPHA = 0.25f
