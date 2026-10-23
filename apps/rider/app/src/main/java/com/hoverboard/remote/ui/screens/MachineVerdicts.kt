package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.machine
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the disagreement badge, wherever it is shown. */
const val MACHINE_BADGE_TAG = "setup_machine_badge"

/**
 * What the pair reads as, in the owner's words: the frame and side verdicts of
 * `specs/rider-ui.md` 3.7a, the disagreement badge of 3.7, and the board labels both use.
 *
 * Apart from [MachineSection] because the section is the controls and this is what they say; the
 * raw ADVANCED rows of This board use these without the section.
 */
/** The disagreement badge of 3.7: the two boards do not hold the pair they should. */
@Composable
internal fun Badge() {
    Text(
        stringResource(R.string.setup_machine_disagree),
        color = AccentRed,
        modifier = Modifier.testTag(MACHINE_BADGE_TAG),
    )
}

/** The frame as the pair holds it, naming each board's value when they differ. */
@Composable
internal fun frameVerdict(state: SetupState, pair: FramePair): String = when (pair) {
    FramePair.Unknown -> stringResource(R.string.setup_machine_unknown)
    is FramePair.Agree -> stringResource(R.string.setup_machine_frame_agree, stringResource(frameLabel(pair.rigid)))
    is FramePair.Differ -> stringResource(
        R.string.setup_machine_frame_differ,
        boardLabel(state, Node.MASTER),
        stringResource(frameLabel(pair.masterRigid)),
        boardLabel(state, Node.SLAVE),
        stringResource(frameLabel(!pair.masterRigid)),
    )
}

/** Which board is the right wheel, or why none is. */
@Composable
internal fun sideVerdict(state: SetupState, pair: SidePair): String = when (pair) {
    SidePair.Unknown -> stringResource(R.string.setup_machine_unknown)
    SidePair.NoneRight -> stringResource(R.string.setup_machine_side_none)
    SidePair.BothRight -> stringResource(R.string.setup_machine_side_both)
    is SidePair.OneRight -> stringResource(R.string.setup_machine_side_right, boardLabel(state, pair.right))
}

/** A board's chip label: the role and the address this session learned for it. */
@Composable
internal fun boardLabel(state: SetupState, node: Node): String {
    val address = if (node == Node.MASTER) state.master else state.slave
    return when {
        address != null && node == Node.MASTER -> stringResource(R.string.board_target_master, boardHex(address))
        address != null -> stringResource(R.string.board_target_slave, boardHex(address))
        node == Node.MASTER -> stringResource(R.string.board_target_master_none)
        else -> stringResource(R.string.board_target_slave_none)
    }
}

/**
 * The pair verdict under one of the two raw ADVANCED rows (`specs/rider-ui.md` 3.7a): a raw edit
 * that breaks the pair is said so where it was made, not only up in the Machine section. Nothing
 * for any other row.
 */
@Composable
internal fun MachineRowVerdict(field: SetupField, state: SetupState) {
    val pair = state.machine
    val small = MaterialTheme.typography.bodySmall
    when (field) {
        SetupFields.CONTROL_FRAME -> {
            Text(frameVerdict(state, pair.frame), style = small, color = TextSecondary)
            if (pair.frame is FramePair.Differ) Badge()
        }
        SetupFields.CONTROL_SIDE -> {
            Text(sideVerdict(state, pair.side), style = small, color = TextSecondary)
            if (pair.side is SidePair.BothRight) Badge()
        }
        else -> Unit
    }
}

@StringRes
internal fun frameLabel(rigid: Boolean): Int =
    if (rigid) R.string.setup_choice_frame_rigid else R.string.setup_choice_frame_pivoting
