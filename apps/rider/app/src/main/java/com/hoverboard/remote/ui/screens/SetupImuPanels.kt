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
import com.hoverboard.remote.model.OrientationPresets
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
    val roles = state.storedRoles
    Panel {
        PanelTitle(R.string.setup_orientation_title)
        Caption(R.string.setup_orientation_body)
        StoredFrame(state, stored, roles)
        val intendedRoles = state.intendedRoles?.let(Orientation::effectiveRoles)
        val flat = intendedRoles == Orientation.DEFAULT_ROLES
        val intended = state.intendedSigns?.let(Orientation::effective)
        PanelTitle(R.string.setup_preset_title)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (p in OrientationPresets.ALL) {
                FilterChip(
                    selected = intendedRoles == Orientation.effectiveRoles(p.roles) && intended == p.signs,
                    onClick = { actions.stageFrame(p.roles, p.signs) },
                    label = { Text(stringResource(p.label)) },
                    enabled = editable,
                )
            }
        }
        for (p in OrientationPresets.ALL) {
            Caption(p.source)
        }
        Caption(R.string.setup_orientation_rover)
        PanelTitle(R.string.setup_rotation_title)
        val chosen = if (flat) intended?.let { Orientation.Rotation.of(it) } else null
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (r in Orientation.ROTATIONS) {
                FilterChip(
                    selected = chosen == r,
                    onClick = { actions.stageFrame(Orientation.DEFAULT_ROLES, r.signs) },
                    label = { Text(stringResource(rotationLabel(r))) },
                    enabled = editable,
                )
            }
        }
    }
}

/** What the board stores: the sign map, the axis roles, and what the two make together. */
@Composable
private fun StoredFrame(state: SetupState, stored: List<Int>?, roles: List<Int>?) {
    Text(
        if (stored == null) {
            stringResource(R.string.setup_orientation_stored_unread)
        } else {
            stringResource(R.string.setup_orientation_stored, stored.toString())
        },
        color = if (stored == null) TextSecondary else TextPrimary,
    )
    Text(
        if (roles == null) stringResource(R.string.setup_orientation_roles_unread) else rolesLine(roles),
        color = if (roles == null) TextSecondary else TextPrimary,
    )
    if (stored == null || roles == null) return
    if (Orientation.check(stored, roles) != null) {
        Text(stringResource(R.string.setup_orientation_mirrored), color = AccentRed)
    } else {
        val name = stringResource(frameName(stored, roles))
        // The board runs the frame it read at boot. The stored frame is that one only while no
        // sign or role index is pending or staged; otherwise it runs from the next power-up.
        val line = if (state.orientationSettled) {
            R.string.setup_orientation_runs
        } else {
            R.string.setup_orientation_stored_as
        }
        Text(stringResource(line, name), color = TextPrimary)
    }
    if (0 in stored || 0 in roles) Text(stringResource(R.string.setup_orientation_unset), color = TextSecondary)
}

/** The name of a legal stored frame: a flat rotation's, or what kind of frame it is otherwise. */
private fun frameName(signs: List<Int>, roles: List<Int>): Int {
    if (Orientation.effectiveRoles(roles) != Orientation.DEFAULT_ROLES) return R.string.setup_rotation_permuted
    return Orientation.Rotation.of(Orientation.effective(signs))?.let(::rotationLabel) ?: R.string.setup_rotation_mixed
}

/** `UP = X, PITCH_RATE = Z`, with an unset role shown as the compiled one it falls back to. */
@Composable
private fun rolesLine(roles: List<Int>): String {
    // `1 = X`, `2 = Y`, `3 = Z`; anything else is shown as stored.
    val names = Orientation.effectiveRoles(roles).map { CHIP_AXES.getOrNull(it - 1)?.toString() ?: "?$it" }
    return stringResource(R.string.setup_orientation_roles, names[0], names[1])
}

private const val CHIP_AXES = "XYZ"

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
