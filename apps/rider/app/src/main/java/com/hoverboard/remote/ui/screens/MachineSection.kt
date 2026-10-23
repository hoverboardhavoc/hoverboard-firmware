package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
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
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.machine
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the Machine section. */
const val MACHINE_SECTION_TAG = "setup_machine"

/** Test tags on the two Machine rows and the pair write. */
const val MACHINE_FRAME_TAG = "setup_machine_frame"
const val MACHINE_SIDE_TAG = "setup_machine_side"
const val MACHINE_WRITE_TAG = "setup_machine_write"

/** Test tag on the slave chip of the Right wheel picker. */
const val MACHINE_SIDE_SLAVE_TAG = "setup_machine_side_slave"

/** Test tag on the disagreement badge, wherever it is shown. */
const val MACHINE_BADGE_TAG = "setup_machine_badge"

/**
 * The Machine section of Setup (`specs/rider-ui.md` 3.7 and 3.7a): the settings that belong to the
 * MACHINE rather than to one board, so they are read from both boards, written to both, and badged
 * when the two disagree. Its first two rows are the rover's frame and its right wheel.
 *
 * The frame goes to both boards with the same value. The right wheel goes to both with DIFFERENT
 * values, and the picker offers only the two addresses, so it cannot express the pair with two
 * right wheels that a raw edit can.
 *
 * It does not follow the board selector above it: both rows are the pair's, whichever board the
 * rest of the screen is editing.
 */
@Composable
internal fun MachineSection(state: SetupState, editable: Boolean, actions: SetupActions) {
    val pair = state.machine
    Section(stringResource(R.string.setup_machine_title)) {
        Panel(modifier = Modifier.testTag(MACHINE_SECTION_TAG)) {
            Caption(R.string.setup_machine_body)
            Caption(R.string.setup_machine_order)
            Caption(R.string.setup_machine_effect)
            if (state.master != null && state.slave == null) Caption(R.string.setup_machine_no_slave)
        }
        FrameRow(state, pair.frame, editable, actions)
        SideRow(state, pair.side, editable, actions)
        WriteRow(state, editable, actions)
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun FrameRow(state: SetupState, pair: FramePair, editable: Boolean, actions: SetupActions) {
    val agreed = (pair as? FramePair.Agree)?.let {
        if (it.rigid) Fields.ControlFrame.RIGID else Fields.ControlFrame.PIVOTING
    }
    val chosen = state.machinePending.frame ?: agreed
    Panel(modifier = Modifier.testTag(MACHINE_FRAME_TAG)) {
        PanelTitle(R.string.setup_machine_frame)
        Caption(R.string.setup_machine_frame_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (value in listOf(Fields.ControlFrame.PIVOTING, Fields.ControlFrame.RIGID)) {
                FilterChip(
                    selected = chosen == value,
                    onClick = { actions.chooseFrame(value) },
                    label = { Text(stringResource(frameLabel(value == Fields.ControlFrame.RIGID))) },
                    enabled = editable,
                )
            }
        }
        Text(frameVerdict(state, pair), color = TextSecondary)
        if (pair is FramePair.Differ) Badge()
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun SideRow(state: SetupState, pair: SidePair, editable: Boolean, actions: SetupActions) {
    val chosen = state.machinePending.rightWheel ?: (pair as? SidePair.OneRight)?.right
    Panel(modifier = Modifier.testTag(MACHINE_SIDE_TAG)) {
        PanelTitle(R.string.setup_machine_side)
        Caption(R.string.setup_machine_side_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            FilterChip(
                selected = chosen == Node.MASTER,
                onClick = { actions.chooseRightWheel(Node.MASTER) },
                label = { Text(boardLabel(state, Node.MASTER)) },
                enabled = editable,
            )
            FilterChip(
                selected = chosen == Node.SLAVE,
                onClick = { actions.chooseRightWheel(Node.SLAVE) },
                label = { Text(boardLabel(state, Node.SLAVE)) },
                enabled = editable && state.slave != null,
                modifier = Modifier.testTag(MACHINE_SIDE_SLAVE_TAG),
            )
        }
        Text(sideVerdict(state, pair), color = TextSecondary)
        if (pair is SidePair.BothRight) Badge()
    }
}

@Composable
private fun WriteRow(state: SetupState, editable: Boolean, actions: SetupActions) {
    if (state.machineWriting) {
        Text(stringResource(R.string.setup_machine_writing), color = TextSecondary)
        return
    }
    if (!state.machinePending.any) return
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        Button(
            onClick = actions::writeMachine,
            enabled = editable,
            modifier = Modifier.testTag(MACHINE_WRITE_TAG),
        ) { Text(stringResource(R.string.setup_machine_write)) }
        OutlinedButton(onClick = actions::discardMachine) {
            Text(stringResource(R.string.setup_machine_discard))
        }
    }
}

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
