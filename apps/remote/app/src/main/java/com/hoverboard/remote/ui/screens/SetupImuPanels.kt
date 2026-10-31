package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
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
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Tone
import com.hoverboard.remote.ui.theme.ToneText
import com.hoverboard.remote.ui.theme.WarningText

@Composable
internal fun AttitudeNow(telemetry: TelemetryUi?) {
    Text(
        text = if (telemetry?.hasState == true) {
            stringResource(R.string.setup_attitude_now, deg(telemetry.pitchDegrees), deg(telemetry.rollDegrees))
        } else {
            stringResource(R.string.setup_attitude_none)
        },
        color = Fg,
    )
}

@Composable
internal fun Level(state: SetupState, editable: Boolean, telemetry: TelemetryUi?, actions: SetupActions) {
    Panel {
        PanelTitle(R.string.setup_level_title)
        Caption(R.string.setup_level_body)
        // The generated line, as every writable row carries it: this flow writes the trim rows, so
        // when the board starts using what it writes is their answer, not a sentence of its own.
        TakeEffect(SetupFields.LEVEL_TRIM[0])
        AttitudeNow(telemetry)
        for (f in SetupFields.LEVEL_TRIM) {
            state.values[f.key]?.let {
                Text(
                    stringResource(R.string.setup_change_line, stringResource(f.label), it.display()),
                    color = FgMuted,
                )
            }
        }
        Button(onClick = actions::setLevel, enabled = editable) { Text(stringResource(R.string.setup_level_action)) }
    }
}

/**
 * The orientation flow (`specs/remote-ui.md` 3.4): what the board stores, a drawing of the board as it
 * runs now, and the pose picker that writes a whole frame.
 *
 * The frame is the one group here with an invariant across its fields, and write-on-change makes its
 * transient visible (3.6a): the eight indices go one write at a time, so between the two role writes
 * the board can hold a duplicate pair. [R.string.setup_frame_writing] says that while it happens
 * rather than leaving the operator to meet it as a refused arm.
 */
@Composable
internal fun OrientationPanel(state: SetupState, editable: Boolean, telemetry: TelemetryUi?, actions: SetupActions) {
    Panel {
        PanelTitle(R.string.setup_orientation_title)
        Caption(R.string.setup_orientation_body)
        StoredFrame(state, state.storedSigns, state.storedRoles)
        if (state.frameWriting) {
            Text(
                stringResource(R.string.setup_frame_writing),
                color = Primary,
                modifier = Modifier.testTag(SETUP_FRAME_WRITING_TAG),
            )
        }
        BoardModel(telemetry)
        PosePicker(state.intendedPose, editable) { actions.editFrame(it.roles, it.signs) }
        Caption(R.string.setup_orientation_family)
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
        color = if (stored == null) FgMuted else Fg,
    )
    Text(
        if (roles == null) stringResource(R.string.setup_orientation_roles_unread) else rolesLine(roles),
        color = if (roles == null) FgMuted else Fg,
    )
    if (stored == null || roles == null) return
    if (Orientation.check(stored, roles) != null) {
        WarningText(stringResource(R.string.setup_orientation_mirrored))
    } else {
        val pose = state.storedPose
        val name = when {
            Orientation.runsFallback(roles, stored) ->
                stringResource(R.string.setup_pose_unset, stringResource(faceLabel(Orientation.fallbackPose.face)))
            pose != null -> poseName(pose)
            else -> stringResource(R.string.setup_pose_custom, roles.toString(), stored.toString())
        }
        // The board runs the frame it read when it last armed or booted. The stored frame is that
        // one only while no sign or role index was written this session.
        val line = if (state.orientationSettled) {
            R.string.setup_orientation_runs
        } else {
            R.string.setup_orientation_stored_as
        }
        Text(stringResource(line, name), color = Fg)
    }
    if (0 in stored || 0 in roles) Text(stringResource(R.string.setup_orientation_unset), color = FgMuted)
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
            CautionText(stringResource(R.string.setup_check_blocked))
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
    Text(stringResource(hint), style = MaterialTheme.typography.bodySmall, color = FgMuted)
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        OutlinedButton(onClick = onRun) { Text(stringResource(action)) }
        val (text, tone) = when (result) {
            CheckResult.PASS -> R.string.setup_check_pass to Tone.PRIMARY
            CheckResult.FAIL -> R.string.setup_check_fail to Tone.WARNING
            CheckResult.INCONCLUSIVE -> R.string.setup_check_inconclusive to Tone.CAUTION
            null -> R.string.setup_check_not_run to Tone.MUTED
        }
        ToneText(stringResource(text), tone)

    }
}

private fun deg(d: Float): String = "%.1f".format(d)
