package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
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
import com.hoverboard.remote.ui.theme.ARMED_OUTLINE
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextSecondary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Telemetry display (SPEC §10), prioritised: link health, battery, speed + throttle, attitude.
 *
 * @param telemetry latest decoded board state, or null before the first arrives.
 * @param throttlePercent commanded throttle percent (signed), shown beside measured speed.
 * @param armed whether the machine's motors are live. This panel is the screen's status strip, and
 *   its red outline is what makes the armed state legible from the top of the screen now that the
 *   armed banner is gone ([com.hoverboard.remote.ui.screens.ControlScreen]).
 */
@Composable
fun TelemetryPanel(
    telemetry: TelemetryUi?,
    throttlePercent: Int,
    armed: Boolean,
    modifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(20.dp)
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(shape)
            .background(PanelSurface)
            .border(ARMED_OUTLINE, if (armed) AccentRed else Color.Transparent, shape)
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

        StatusChips(telemetry)
        Spacer(modifier = Modifier.height(16.dp))
        BatterySection(telemetry)
        Spacer(modifier = Modifier.height(16.dp))
        SpeedAndThrottleRow(telemetry, throttlePercent)
        Spacer(modifier = Modifier.height(16.dp))
        AttitudeRow(telemetry)
    }
}

/**
 * The board's own state bits, decoded from `CYCLIC_STATE.flags` and the FAULT edge PDU and, until
 * now, decoded and thrown away.
 *
 * Two different kinds of thing are shown two different ways on purpose.
 *
 * RIDER is a STATE, and both of its values mean something: the board is reporting what its own foot
 * pads read this instant (the cyclic flag is built from LOCAL pads only,
 * `orchestrator::dispatch::cyclic_state`), so "no rider" is a reading, not a silence. It is drawn
 * either way, lit or dim.
 *
 * LOCKDOWN and FAULT are ALARMS, drawn only when they assert, and for FAULT that is not a
 * presentation preference. Neither of the two things that can set a fault here exists in the
 * firmware yet: `CYCLIC_STATE.fault` is built as a literal 0 on every emission, and `OP_FAULT` has
 * no emitter in `crates/` at all. A quiet FAULT chip would therefore be a lamp wired to nothing,
 * and a rider who learned to read it as "no faults" would be reading a claim the app cannot make
 * and the firmware cannot support. So this renders the assertion and never the negation, and the
 * code is live so that whichever producer is written first lights it with no change here.
 *
 * Silence alone is not enough, though, and this is the correction the battery half already had:
 * absence claims nothing only to a reader who already knows there is no producer, and to everyone
 * else a screen with no fault line is indistinguishable from a working fault display that happens
 * to be clear. So while nothing is asserting, a static note says the reporting itself is missing.
 * It is the same shape as the battery tag: show what is there, and say plainly what is not.
 *
 * With one difference that has to be stated rather than assumed. The battery half retires itself:
 * the board says whether it senses its rail, so the panel reads that and says nothing of its own
 * accord. This note cannot. There is no observable for "fault reporting exists" -- a board with a
 * working producer and nothing wrong reports exactly what a board with no producer reports -- so on
 * the day a producer ships, this renders against a board that DOES report faults and becomes a
 * false statement. Yielding to an asserting chip only covers a fault that is up right
 * now. Hence the wording, which is a claim about the firmware this app was BUILT FOR rather than
 * about the board in front of it, and hence the entry in `specs/todo.md` part 5: this string is
 * deleted by hand when a fault producer lands. Nothing here will notice.
 */
@Composable
private fun StatusChips(telemetry: TelemetryUi) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Chip(
            text = stringResource(R.string.telemetry_chip_rider),
            color = if (telemetry.riderPresent) AccentGreen else ZeroLine,
        )
        if (telemetry.lockdown) {
            Chip(text = stringResource(R.string.telemetry_chip_lockdown), color = AccentYellow)
        }
        if (telemetry.anyFault) {
            Chip(text = faultText(telemetry), color = AccentRed)
        }
    }
    if (!telemetry.anyFault) {
        Spacer(modifier = Modifier.height(8.dp))
        Text(
            text = stringResource(R.string.telemetry_fault_unreported),
            style = MaterialTheme.typography.bodySmall,
            color = TextSecondary,
        )
    }
}

/** What the fault chip says: the edge code when there is one, else the level, else just FAULT. */
@Composable
private fun faultText(telemetry: TelemetryUi): String = when {
    telemetry.faultStop -> stringResource(R.string.telemetry_chip_fault_stop)
    telemetry.faultCode != 0 -> stringResource(R.string.telemetry_chip_fault_code, telemetry.faultCode)
    else -> stringResource(R.string.telemetry_chip_fault_level, telemetry.faultLevel)
}

@Composable
private fun Chip(text: String, color: Color) {
    Text(
        text = text,
        style = MaterialTheme.typography.labelMedium,
        color = color,
        modifier = Modifier
            .clip(RoundedCornerShape(6.dp))
            .background(color.copy(alpha = CHIP_FILL_ALPHA))
            .padding(horizontal = 8.dp, vertical = 4.dp),
    )
}

/**
 * Pack voltage, state of charge and the charge bar, or, from a board that does not sense its rail,
 * the statement that there is no reading.
 *
 * `battery = 0` on the wire is the firmware saying this board cannot sense its battery rail: a
 * slave never can, and a master only with `board.vbatt` set ([TelemetryUi.batteryVolts] has the
 * firmware side). Drawn as a number that was 0.00 V, and a 0.00 V on a rider's screen is a flat
 * pack, which is a reading and an alarming one. So the absent case draws no number, no percent and
 * no bar, and says in words what it is.
 */
@Composable
private fun BatterySection(telemetry: TelemetryUi) {
    val volts = telemetry.batteryVolts
    if (volts == null) {
        UnsensedBattery()
        return
    }
    val percent = BatteryCurve.percent(volts)
    val fraction = BatteryCurve.fraction(volts)
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
                text = stringResource(R.string.telemetry_battery_value, volts),
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

/**
 * The battery row of a board that senses no rail: the label, the words "not sensed" where the
 * reading would be, and a line saying what that means. No number, no percent, no bar, because each
 * of those would be a claim about a pack this board cannot see.
 */
@Composable
private fun UnsensedBattery() {
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
        Text(
            text = stringResource(R.string.telemetry_battery_unsensed),
            style = MaterialTheme.typography.bodyMedium,
            color = TextSecondary,
        )
    }
    Spacer(modifier = Modifier.height(8.dp))
    Text(
        text = stringResource(R.string.telemetry_battery_unsensed_note),
        style = MaterialTheme.typography.bodySmall,
        color = TextSecondary,
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

@Composable
private fun SpeedAndThrottleRow(telemetry: TelemetryUi, throttlePercent: Int) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        // Labelled for what the number is, not for what a panel slot in that position usually
        // holds: `CYCLIC_STATE.wheel_speed` is the signed hall-edge count per 320-period window
        // (`specs/link-control.md`, the CYCLIC_STATE mirror section), and a road speed needs
        // `motor.pole_pairs` and a wheel diameter, neither of which is a registered field. So the
        // count is shown as a count. The word also has no writer on any board yet, so it reads 0.
        Metric(
            label = stringResource(R.string.telemetry_speed),
            value = stringResource(R.string.telemetry_speed_value, telemetry.hallEdgesPerWindow),
        )
        Metric(
            label = stringResource(R.string.telemetry_throttle),
            value = stringResource(R.string.telemetry_throttle_value, throttlePercent),
        )
    }
}

/**
 * Board attitude, in the slot the per-wheel current row used to occupy.
 *
 * CYCLIC_STATE is a board-level record and carries no per-motor current
 * (`crates/linkctl/src/lib.rs:88-106`), so there is nothing to put in the old row. Pitch and roll
 * are the two board-level values it does carry, so the layout is unchanged and the pane stays
 * two-up.
 */
@Composable
private fun AttitudeRow(telemetry: TelemetryUi) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Metric(
            label = stringResource(R.string.telemetry_pitch),
            value = stringResource(R.string.telemetry_angle_value, telemetry.pitchDegrees),
        )
        Metric(
            label = stringResource(R.string.telemetry_roll),
            value = stringResource(R.string.telemetry_angle_value, telemetry.rollDegrees),
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
private const val CHIP_FILL_ALPHA = 0.18f
