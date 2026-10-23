package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.frameSending
import com.hoverboard.remote.machine
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.rightWheelSending
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the Machine section. */
const val MACHINE_SECTION_TAG = "setup_machine"

/** Test tags on the two Machine rows. */
const val MACHINE_FRAME_TAG = "setup_machine_frame"
const val MACHINE_SIDE_TAG = "setup_machine_side"

/** Test tag on one board's line under a Machine row, reporting that board's own write. */
fun machineWriteTag(key: Key, node: Node): String =
    "setup_machine_write_${key.fieldId}_${node.name.lowercase()}"

/** Test tag on the slave chip of the Right wheel picker. */
const val MACHINE_SIDE_SLAVE_TAG = "setup_machine_side_slave"

/**
 * The Machine section of Setup (`specs/rider-ui.md` 3.7 and 3.7a): the settings that belong to the
 * MACHINE rather than to one board, so they are read from both boards, written to both, and badged
 * when the two disagree. Its first two rows are the rover's frame and its right wheel.
 *
 * The frame goes to both boards with the same value. The right wheel goes to both with DIFFERENT
 * values, and the picker offers only the two addresses, so it cannot express the pair with two
 * right wheels that a raw edit can.
 *
 * Like the rest of Setup (`specs/rider-ui.md` 3.6a), a chip writes when it is tapped: there is no
 * button to press afterwards and nothing to discard. The two boards are written in the order the
 * pair needs, and each board reports its own write under the row.
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
            if (state.master != null && state.slave == null) Caption(R.string.setup_machine_no_slave)
        }
        FrameRow(state, pair.frame, editable, actions)
        SideRow(state, pair.side, editable, actions)
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun FrameRow(state: SetupState, pair: FramePair, editable: Boolean, actions: SetupActions) {
    val agreed = (pair as? FramePair.Agree)?.let {
        if (it.rigid) Fields.ControlFrame.RIGID else Fields.ControlFrame.PIVOTING
    }
    val chosen = state.frameSending ?: agreed
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
        PairWrite(state, SetupFields.CONTROL_FRAME)
        TakeEffect(SetupFields.CONTROL_FRAME)
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun SideRow(state: SetupState, pair: SidePair, editable: Boolean, actions: SetupActions) {
    val chosen = state.rightWheelSending ?: (pair as? SidePair.OneRight)?.right
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
        PairWrite(state, SetupFields.CONTROL_SIDE)
        TakeEffect(SetupFields.CONTROL_SIDE)
    }
}

/**
 * What each board said about this row's write, named by the board that said it
 * (`specs/rider-ui.md` 3.7a: a refusal from one of the two is shown with that board's own status).
 * Nothing for a board this session has not written.
 */
@Composable
private fun PairWrite(state: SetupState, field: SetupField) {
    for (node in Node.entries) {
        val write = state.setupOf(node).writes[field.key] ?: continue
        val (line, colour) = writeStatus(field, write)
        Text(
            stringResource(R.string.setup_machine_board_line, boardLabel(state, node), line),
            style = MaterialTheme.typography.bodySmall,
            color = colour,
            modifier = Modifier.testTag(machineWriteTag(field.key, node)),
        )
    }
}
