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
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.FaultCause
import com.hoverboard.remote.model.SpeedReading
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.WarningText

/**
 * Telemetry display (SPEC §10), prioritised: link health, battery, speed + throttle, attitude.
 *
 * @param telemetry latest decoded board state, or null before the first arrives.
 * @param throttlePercent commanded throttle percent (signed), shown beside measured speed.
 * @param armed whether the machine's motors are live. This panel is the screen's status strip, and
 *   its red outline is what makes the armed state legible from the top of the screen now that the
 *   armed banner is gone ([com.hoverboard.remote.ui.screens.ControlScreen]).
 * @param battery what the SELECTED board's battery display says ([BatterySection]): its reported
 *   pack voltage against the pack it stores, or the reason there is no state of charge.
 * @param currents each motor controller's current display ([CurrentSection]), labelled by its node
 *   label, in address order. One row is a machine with one board in it; a board that is there and
 *   silent draws a row saying so.
 * @param speed the machine's speed ([com.hoverboard.remote.model.machineSpeed]), or what it lacks.
 * @param board the label of the board [telemetry] came from ([com.hoverboard.remote.ui.screens.nodeLabel]),
 *   which each fault line names.
 */
@Composable
fun TelemetryPanel(
    telemetry: TelemetryUi?,
    throttlePercent: Int,
    battery: BatteryReading,
    currents: List<Pair<String, CurrentReading>>,
    speed: SpeedReading,
    board: String,
    modifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(20.dp)
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(shape)
            .background(BgElevated)
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
                color = FgMuted,
            )
            return@Column
        }

        StatusChips(telemetry, board)
        Spacer(modifier = Modifier.height(16.dp))
        BatterySection(battery)
        Spacer(modifier = Modifier.height(16.dp))
        CurrentSection(currents)
        Spacer(modifier = Modifier.height(16.dp))
        SpeedAndThrottleRow(speed, throttlePercent)
        Spacer(modifier = Modifier.height(16.dp))
        AttitudeRow(telemetry)
    }
}

/**
 * The board's own state bits, decoded from `CYCLIC_STATE`.
 *
 * Two different kinds of thing are shown two different ways on purpose.
 *
 * RIDER is a STATE, and both of its values mean something: the board is reporting what its own foot
 * pads read this instant (the cyclic flag is built from LOCAL pads only,
 * `orchestrator::dispatch::cyclic_state`), so "no rider" is a reading, not a silence. It is drawn
 * either way, lit or dim.
 *
 * LOCKDOWN and FAULT are ALARMS, drawn only when they assert, and for FAULT that is not a
 * presentation preference. `CYCLIC_STATE.fault` is the board's OWN producer level mask
 * (`orchestrator::events::Levels::own_mask`, which excludes the peer-derived level by
 * construction), so a quiet FAULT chip would say "no producer asserted on the board the phone is
 * attached to", which on a two-board machine is not "no faults". This renders the assertion and
 * never the negation.
 *
 * Silence alone is not enough, though, and this is the correction the battery half already had:
 * absence claims nothing only to a reader who already knows what the byte covers, and to everyone
 * else a screen with no fault line is indistinguishable from a whole-machine fault display that
 * happens to be clear. So while nothing is asserting, a static note says what the silence covers
 * and what it does not. It is the same shape as the battery tag: show what is there, and say
 * plainly what is not.
 */
@Composable
private fun StatusChips(telemetry: TelemetryUi, board: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Chip(
            text = stringResource(R.string.telemetry_chip_rider),
            color = if (telemetry.riderPresent) Primary else FgFaint,
        )
        if (telemetry.lockdown) {
            CautionText(text = stringResource(R.string.telemetry_chip_lockdown))
        }
    }
    // One line per set cause, naming the board, so a fault says where it is and what it is
    // (`specs/link-control.md`, `CYCLIC_STATE`).
    for (cause in telemetry.faultCauses) {
        Spacer(modifier = Modifier.height(8.dp))
        WarningText(
            text = stringResource(R.string.telemetry_fault_cause, board, stringResource(cause.label())),
            style = MaterialTheme.typography.labelMedium,
        )
    }
    if (!telemetry.anyFault) {
        Spacer(modifier = Modifier.height(8.dp))
        Text(
            text = stringResource(R.string.telemetry_fault_unreported),
            style = MaterialTheme.typography.bodySmall,
            color = FgMuted,
        )
    }
}

/** The string a [FaultCause] is named by. */
private fun FaultCause.label(): Int = when (this) {
    FaultCause.COMMS_LOSS -> R.string.fault_cause_comms_loss
    FaultCause.TILT_STOP -> R.string.fault_cause_tilt_stop
    FaultCause.IMU_LOSS -> R.string.fault_cause_imu_loss
    FaultCause.LATCH_A -> R.string.fault_cause_latch_a
    FaultCause.LATCH_B -> R.string.fault_cause_latch_b
    FaultCause.MOTOR_HALL -> R.string.fault_cause_motor_hall
    FaultCause.MOTOR_CAL -> R.string.fault_cause_motor_cal
    FaultCause.MOTOR_PERIOD -> R.string.fault_cause_motor_period
    FaultCause.MOTOR_ARM -> R.string.fault_cause_motor_arm
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

@Composable
private fun SpeedAndThrottleRow(speed: SpeedReading, throttlePercent: Int) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        // The machine's road speed from `CYCLIC_STATE.wheel_rate` (`specs/rider-ui.md` 3.1a):
        // signed forward, tenths below 10 km/h. "none" while a conversion fact is missing, and the
        // line under the row names it.
        Metric(
            label = stringResource(R.string.telemetry_speed),
            value = when (speed) {
                is SpeedReading.Kmh -> when (val shown = speed.shown) {
                    is SpeedReading.Shown.Tenths -> stringResource(R.string.telemetry_speed_tenths, shown.kmh)
                    is SpeedReading.Shown.Whole -> stringResource(R.string.telemetry_speed_whole, shown.kmh)
                }
                SpeedReading.Silent, is SpeedReading.NoSpeed -> stringResource(R.string.telemetry_speed_none)
            },
        )
        Metric(
            label = stringResource(R.string.telemetry_throttle),
            value = stringResource(R.string.telemetry_throttle_value, throttlePercent),
        )
    }
    if (speed is SpeedReading.NoSpeed) {
        Spacer(modifier = Modifier.height(8.dp))
        Text(
            text = if (speed.unread.isNotEmpty()) {
                stringResource(R.string.telemetry_speed_unread, speed.unread.joinToString(", "))
            } else {
                stringResource(R.string.telemetry_speed_unset, speed.unset.joinToString(", "))
            },
            style = MaterialTheme.typography.bodySmall,
            color = FgMuted,
        )
    }
}

/**
 * Board attitude, in the slot the per-wheel current row used to occupy.
 *
 * CYCLIC_STATE is a board-level record and carries no per-motor current
 * (`crates/linkctl/src/lib.rs`, `CyclicState`), so there is nothing to put in the old row. Pitch
 * and roll are the two board-level values it does carry, so the layout is unchanged and the pane
 * stays two-up.
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
            color = FgMuted,
        )
        Text(
            text = value,
            style = MaterialTheme.typography.titleLarge,
            color = MaterialTheme.colorScheme.onSurface,
        )
    }
}

private const val CHIP_FILL_ALPHA = 0.18f
