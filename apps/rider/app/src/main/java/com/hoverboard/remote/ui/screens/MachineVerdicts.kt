package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.ModePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.machine
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.driveModeLabel
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.FgMuted

/** Test tag on the disagreement badge, wherever it is shown. */
const val MACHINE_BADGE_TAG = "setup_machine_badge"

/**
 * What the machine reads as, in the owner's words: the drive mode, frame and side verdicts of
 * `specs/rider-ui.md` 3.2 and 3.7a, the disagreement badge of 3.7, and the board labels they use.
 *
 * Apart from [MachineSection] because the section is the controls and this is what they say; the
 * raw ADVANCED rows of This board use these without the section.
 */
/** The disagreement badge of 3.7: the boards do not hold what the machine needs them to. */
@Composable
internal fun Badge() {
    CautionText(
        stringResource(R.string.setup_machine_disagree),
        modifier = Modifier.testTag(MACHINE_BADGE_TAG),
    )

}

/** The frame as the machine holds it, naming each board's value when they differ. */
@Composable
internal fun frameVerdict(state: SetupState, pair: FramePair): String = when (pair) {
    FramePair.Unknown -> stringResource(R.string.setup_machine_unknown)
    is FramePair.Agree -> stringResource(R.string.setup_machine_frame_agree, stringResource(frameLabel(pair.rigid)))
    is FramePair.Differ -> eachBoard(state, pair.rigid.mapValues { (_, rigid) -> stringResource(frameLabel(rigid)) })
}

/** The drive mode as the machine stores it, naming each board's value when they differ. */
@Composable
internal fun modeVerdict(state: SetupState, pair: ModePair): String = when (pair) {
    ModePair.Unknown -> stringResource(R.string.setup_machine_mode_unknown)
    is ModePair.Agree -> stringResource(R.string.setup_machine_frame_agree, modeName(pair.byte))
    is ModePair.Differ -> eachBoard(state, pair.bytes.mapValues { (_, byte) -> modeName(byte) })
}

/** A stored drive-mode byte as the screen names it, or the byte itself when it is no known mode. */
@Composable
internal fun modeName(byte: Int): String =
    DriveMode.of(byte)?.let { stringResource(driveModeLabel(it)) }
        ?: stringResource(R.string.setup_machine_mode_raw, byte)

/** "<board>: <value>." for every board, in address order. */
@Composable
private fun eachBoard(state: SetupState, values: Map<Int, String>): String =
    values.toSortedMap()
        .map { (address, v) -> stringResource(R.string.setup_machine_value, boardLabel(state, address), v) }
        .joinToString(" ")

/** Which board is the right wheel, or why none is. */
@Composable
internal fun sideVerdict(state: SetupState, pair: SidePair): String = when (pair) {
    SidePair.Unknown -> stringResource(R.string.setup_machine_unknown)
    SidePair.NoneRight -> stringResource(R.string.setup_machine_side_none)
    SidePair.BothRight -> stringResource(R.string.setup_machine_side_both)
    is SidePair.OneRight -> stringResource(R.string.setup_machine_side_right, boardLabel(state, pair.right))
}

/** A board's label: its node label ([nodeLabel]), or its address while it is not among the nodes. */
internal fun boardLabel(state: SetupState, address: Int): String =
    state.nodeAt(address)?.let { nodeLabel(it) } ?: boardHex(address)

/**
 * The machine verdict under one of the three raw ADVANCED rows (`specs/rider-ui.md` 3.7a): a raw
 * edit that breaks the machine is said so where it was made, not only up in the Machine section.
 * Nothing for any other row.
 */
@Composable
internal fun MachineRowVerdict(field: SetupField, state: SetupState) {
    val pair = state.machine
    val small = MaterialTheme.typography.bodySmall
    when (field) {
        SetupFields.CONTROL_FRAME -> {
            Text(frameVerdict(state, pair.frame), style = small, color = FgMuted)
            if (pair.frame is FramePair.Differ) Badge()
        }
        SetupFields.DRIVE_MODE -> {
            Text(modeVerdict(state, pair.mode), style = small, color = FgMuted)
            if (pair.mode is ModePair.Differ) Badge()
        }
        SetupFields.CONTROL_SIDE -> {
            Text(sideVerdict(state, pair.side), style = small, color = FgMuted)
            if (pair.side is SidePair.BothRight) Badge()
        }
        else -> Unit
    }
}

@StringRes
internal fun frameLabel(rigid: Boolean): Int =
    if (rigid) R.string.setup_choice_frame_rigid else R.string.setup_choice_frame_pivoting
