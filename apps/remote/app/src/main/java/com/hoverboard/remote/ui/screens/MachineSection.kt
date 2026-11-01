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
import com.hoverboard.remote.machine
import com.hoverboard.remote.machineSending
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.rightWheelSending
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.ToneText

/** Test tag on the Machine section. */
const val MACHINE_SECTION_TAG = "setup_machine"

/** Test tags on the two Machine rows. */
const val MACHINE_FRAME_TAG = "setup_machine_frame"
const val MACHINE_SIDE_TAG = "setup_machine_side"

/** Test tag on one board's line under a Machine row, reporting that board's own write. */
fun machineWriteTag(key: Key, address: Int): String = "setup_machine_write_${key.fieldId}_${boardHex(address)}"

/** Test tag on the chip of the Right wheel picker for the board at [address]. */
fun machineSideTag(address: Int): String = "setup_machine_side_${boardHex(address)}"

/**
 * The Machine section of Setup (`specs/remote-ui.md` 3.2, 3.7 and 3.7a): the settings that belong to
 * the MACHINE rather than to one board, so they are read from every motor controller, written to
 * every one, and badged when the copies disagree. Its rows are the rover's frame and its right
 * wheel; the drive mode is not stored but derived from the frame, the control mode and the number
 * of motor controllers (`specs/machine-settings.md` section 7).
 *
 * The frame goes to every motor controller with the same value. The right wheel
 * goes with DIFFERENT values, and the picker offers one chip per motor controller, so it cannot
 * express a machine with two right wheels that a raw edit can.
 *
 * Like the rest of Setup (`specs/remote-ui.md` 3.6a), a chip writes when it is tapped: there is no
 * button to press afterwards and nothing to discard. The boards are written in the order the
 * machine needs, and each board reports its own write under the row.
 *
 * It does not follow the node selector above it: every row is the machine's, whichever node the
 * rest of the screen is editing.
 */
@Composable
internal fun MachineSection(state: SetupState, editable: Boolean, actions: SetupActions) {
    val machine = state.machine
    Section(stringResource(R.string.setup_machine_title)) {
        Panel(modifier = Modifier.testTag(MACHINE_SECTION_TAG)) {
            Caption(R.string.setup_machine_body)
            Caption(R.string.setup_machine_order)
            if (state.attached != null && state.motorControllers.size < 2) Caption(R.string.setup_machine_no_peer)
        }
        FrameRow(state, machine.frame, editable, actions)
        SideRow(state, machine.side, editable, actions)
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun FrameRow(state: SetupState, pair: FramePair, editable: Boolean, actions: SetupActions) {
    val agreed = (pair as? FramePair.Agree)?.let {
        if (it.rigid) Fields.ControlFrame.RIGID else Fields.ControlFrame.PIVOTING
    }
    val chosen = state.machineSending(SetupFields.CONTROL_FRAME.key) ?: agreed
    Panel(modifier = Modifier.testTag(MACHINE_FRAME_TAG)) {
        PanelTitle(R.string.setup_machine_frame)
        Caption(R.string.setup_machine_frame_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (value in listOf(Fields.ControlFrame.PIVOTING, Fields.ControlFrame.RIGID)) {
                FilterChip(
                    selected = chosen == value,
                    onClick = { actions.chooseFrame(value) },
                    label = { Text(stringResource(frameLabel(value == Fields.ControlFrame.RIGID))) },
                    enabled = editable && state.motorControllers.isNotEmpty(),
                )
            }
        }
        Text(frameVerdict(state, pair), color = FgMuted)
        if (pair is FramePair.Differ) Badge()
        PairWrite(state, SetupFields.CONTROL_FRAME)
        TakeEffect(SetupFields.CONTROL_FRAME)
    }
}

/**
 * The Right wheel picker: one chip per motor controller, the labels the selector shows, disabled
 * while the walk has found fewer than two (`specs/remote-ui.md` 3.7a). It preselects nothing.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun SideRow(state: SetupState, pair: SidePair, editable: Boolean, actions: SetupActions) {
    val chosen = state.rightWheelSending ?: (pair as? SidePair.OneRight)?.right
    val pickable = state.motorControllers.size >= 2
    Panel(modifier = Modifier.testTag(MACHINE_SIDE_TAG)) {
        PanelTitle(R.string.setup_machine_side)
        Caption(R.string.setup_machine_side_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (node in state.nodes.filter { it.isMotorController }) {
                FilterChip(
                    selected = chosen == node.address,
                    onClick = { actions.chooseRightWheel(node.address) },
                    label = { Text(nodeLabel(node)) },
                    enabled = editable && pickable,
                    modifier = Modifier.testTag(machineSideTag(node.address)),
                )
            }
        }
        Text(sideVerdict(state, pair), color = FgMuted)
        if (pair is SidePair.BothRight) Badge()
        PairWrite(state, SetupFields.CONTROL_SIDE)
        TakeEffect(SetupFields.CONTROL_SIDE)
    }
}

/**
 * What each board said about this row's write, named by the board that said it
 * (`specs/remote-ui.md` 3.7a: a refusal from one of them is shown with that board's own status).
 * Nothing for a board this session has not written.
 */
@Composable
private fun PairWrite(state: SetupState, field: SetupField) {
    for (address in state.boards.keys.sorted()) {
        val write = state.setupOf(address).writes[field.key] ?: continue
        val (line, tone) = writeStatus(field, write)
        ToneText(
            stringResource(R.string.setup_machine_board_line, boardLabel(state, address), line),
            tone,
            style = MaterialTheme.typography.bodySmall,
            modifier = Modifier.testTag(machineWriteTag(field.key, address)),
        )

    }
}
