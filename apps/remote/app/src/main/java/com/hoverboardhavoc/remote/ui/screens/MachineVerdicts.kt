package com.hoverboardhavoc.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboardhavoc.remote.FramePair
import com.hoverboardhavoc.remote.LeaderPair
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.SetupState
import com.hoverboardhavoc.remote.SidePair
import com.hoverboardhavoc.remote.machine
import com.hoverboardhavoc.remote.model.SetupField
import com.hoverboardhavoc.remote.model.SetupFields
import com.hoverboardhavoc.remote.ui.theme.CautionText
import com.hoverboardhavoc.remote.ui.theme.FgMuted

/** Test tag on the disagreement badge, wherever it is shown. */
const val MACHINE_BADGE_TAG = "setup_machine_badge"

/**
 * What the machine reads as, in the owner's words: the frame and side verdicts of
 * `specs/remote-ui.md` 3.7a, the disagreement badge of 3.7, and the board labels they use.
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

/** Which board is set as the rigid frame's leader, or why none is. */
@Composable
internal fun leaderVerdict(state: SetupState, pair: LeaderPair): String = when (pair) {
    LeaderPair.Unknown -> stringResource(R.string.setup_machine_unknown)
    LeaderPair.None -> stringResource(R.string.setup_machine_leader_none)
    LeaderPair.Both -> stringResource(R.string.setup_machine_leader_both)
    is LeaderPair.One -> stringResource(R.string.setup_machine_leader_one, boardLabel(state, pair.leader))
}

/** A board's label: its node label ([nodeLabel]), or its address while it is not among the nodes. */
internal fun boardLabel(state: SetupState, address: Int): String =
    state.nodeAt(address)?.let { nodeLabel(it) } ?: boardHex(address)

/**
 * The machine verdict under one of the two raw ADVANCED rows (`specs/remote-ui.md` 3.7a): a raw
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
