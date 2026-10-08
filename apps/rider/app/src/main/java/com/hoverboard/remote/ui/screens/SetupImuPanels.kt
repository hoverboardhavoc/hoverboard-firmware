package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.remote.CheckResult
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.display
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

@Composable
internal fun AttitudeNow(telemetry: TelemetryUi?) {
    Text(
        text = if (telemetry?.hasState == true) {
            stringResource(R.string.setup_attitude_now, deg(telemetry.pitchDegrees), deg(telemetry.rollDegrees))
        } else {
            stringResource(R.string.setup_attitude_none)
        },
        color = TextPrimary,
    )
}

@Composable
internal fun Level(state: SetupState, editable: Boolean, telemetry: TelemetryUi?, actions: SetupActions) {
    Panel {
        PanelTitle(R.string.setup_level_title)
        Caption(R.string.setup_level_body)
        AttitudeNow(telemetry)
        for (f in SetupFields.LEVEL_TRIM) {
            state.values[f.key]?.let {
                Text(
                    stringResource(R.string.setup_change_line, stringResource(f.label), it.display()),
                    color = TextSecondary,
                )
            }
        }
        Button(onClick = actions::setLevel, enabled = editable) { Text(stringResource(R.string.setup_level_action)) }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun OrientationPanel(state: SetupState, editable: Boolean, actions: SetupActions) {
    val stored = state.storedSigns
    val intended = state.intendedSigns
    Panel {
        PanelTitle(R.string.setup_orientation_title)
        Caption(R.string.setup_orientation_body)
        if (stored == null) {
            Text(stringResource(R.string.setup_orientation_stored_unread), color = TextSecondary)
        } else {
            Text(stringResource(R.string.setup_orientation_stored, stored.toString()), color = TextPrimary)
            if (Orientation.check(stored) != null) {
                Text(stringResource(R.string.setup_orientation_mirrored), color = AccentRed)
            } else {
                val rotation = Orientation.Rotation.of(Orientation.effective(stored))
                val name = stringResource(rotation?.let(::rotationLabel) ?: R.string.setup_rotation_mixed)
                // The board runs the map it read at boot. The stored map is that one only while no
                // sign index is pending or staged; otherwise it runs from the next power-up.
                val line = if (state.orientationSettled) {
                    R.string.setup_orientation_runs
                } else {
                    R.string.setup_orientation_stored_as
                }
                Text(stringResource(line, name), color = TextPrimary)
            }
            if (0 in stored) Text(stringResource(R.string.setup_orientation_unset), color = TextSecondary)
        }
        val chosen = intended?.let { Orientation.Rotation.of(it) }
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (r in Orientation.ROTATIONS) {
                FilterChip(
                    selected = chosen == r,
                    onClick = { actions.stageRotation(r) },
                    label = { Text(stringResource(rotationLabel(r))) },
                    enabled = editable,
                )
            }
        }
        Caption(R.string.setup_orientation_rover)
    }
}

@Composable
internal fun RotationCheckPanel(state: SetupState, telemetry: TelemetryUi?, actions: SetupActions) {
    Panel {
        PanelTitle(R.string.setup_check_title)
        Caption(R.string.setup_check_body)
        if (!state.orientationSettled) {
            Text(stringResource(R.string.setup_check_blocked), color = AccentYellow)
            return@Panel
        }
        AttitudeNow(telemetry)
        val check = state.rotationCheck
        CheckStep(R.string.setup_check_level, R.string.setup_check_level_hint, check.level, actions::checkLevel)
        CheckStep(
            R.string.setup_check_lean,
            R.string.setup_check_lean_hint,
            check.forwardLean,
            actions::checkForwardLean,
        )
    }
}

@Composable
internal fun CheckStep(action: Int, hint: Int, result: CheckResult?, onRun: () -> Unit) {
    Text(stringResource(hint), style = MaterialTheme.typography.bodySmall, color = TextSecondary)
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        OutlinedButton(onClick = onRun) { Text(stringResource(action)) }
        val (text, color) = when (result) {
            CheckResult.PASS -> R.string.setup_check_pass to AccentGreen
            CheckResult.FAIL -> R.string.setup_check_fail to AccentRed
            CheckResult.INCONCLUSIVE -> R.string.setup_check_inconclusive to AccentYellow
            null -> R.string.setup_check_not_run to TextSecondary
        }
        Text(stringResource(text), color = color)
    }
}

internal fun rotationLabel(r: Orientation.Rotation): Int = when (r) {
    Orientation.Rotation.IDENTITY -> R.string.setup_rotation_identity
    Orientation.Rotation.HALF_TURN_X -> R.string.setup_rotation_x
    Orientation.Rotation.HALF_TURN_Y -> R.string.setup_rotation_y
    Orientation.Rotation.HALF_TURN_Z -> R.string.setup_rotation_z
}

private fun deg(d: Float): String = "%.1f".format(d)
