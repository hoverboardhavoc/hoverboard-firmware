package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.ModePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.machine
import com.hoverboard.remote.machineSending
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.driveModeLabel
import com.hoverboard.remote.rightWheelSending
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the Machine section. */
const val MACHINE_SECTION_TAG = "setup_machine"

/** Test tags on the three Machine rows. */
const val MACHINE_MODE_TAG = "setup_machine_mode"
const val MACHINE_FRAME_TAG = "setup_machine_frame"
const val MACHINE_SIDE_TAG = "setup_machine_side"

/** Test tag on one board's line under a Machine row, reporting that board's own write. */
fun machineWriteTag(key: Key, address: Int): String = "setup_machine_write_${key.fieldId}_${boardHex(address)}"

/** Test tag on the chip of the Right wheel picker for the board at [address]. */
fun machineSideTag(address: Int): String = "setup_machine_side_${boardHex(address)}"

/** Test tag on a drive-mode chip of the Machine section. */
fun machineModeTag(mode: DriveMode): String = "setup_machine_mode_${mode.name.lowercase()}"

/** Test tag on the one-tap "write this value to every board" under a drive-mode disagreement. */
fun machineModeUseTag(byte: Int): String = "setup_machine_mode_use_$byte"

/**
 * The Machine section of Setup (`specs/rider-ui.md` 3.2, 3.7 and 3.7a): the settings that belong to
 * the MACHINE rather than to one board, so they are read from every motor controller, written to
 * every one, and badged when the copies disagree. Its rows are the drive mode the app drives by, the
 * rover's frame and its right wheel.
 *
 * The drive mode and the frame go to every motor controller with the same value. The right wheel
 * goes with DIFFERENT values, and the picker offers one chip per motor controller, so it cannot
 * express a machine with two right wheels that a raw edit can.
 *
 * Like the rest of Setup (`specs/rider-ui.md` 3.6a), a chip writes when it is tapped: there is no
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
        ModeRow(state, machine.mode, editable, actions)
        FrameRow(state, machine.frame, editable, actions)
        SideRow(state, machine.side, editable, actions)
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun ModeRow(state: SetupState, pair: ModePair, editable: Boolean, actions: SetupActions) {
    val chosen = state.machineSending(SetupFields.DRIVE_MODE.key) ?: (pair as? ModePair.Agree)?.byte
    Panel(modifier = Modifier.testTag(MACHINE_MODE_TAG)) {
        PanelTitle(R.string.setup_machine_mode)
        Caption(R.string.setup_machine_mode_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (mode in DriveMode.entries) {
                FilterChip(
                    selected = chosen == mode.byte,
                    onClick = { actions.chooseDriveMode(mode.byte) },
                    label = { Text(stringResource(driveModeLabel(mode))) },
                    enabled = editable && state.motorControllers.isNotEmpty(),
                    modifier = Modifier.testTag(machineModeTag(mode)),
                )
            }
        }
        Text(modeVerdict(state, pair), color = TextSecondary)
        if (pair is ModePair.Differ) {
            Badge()
            // The one-tap answer to a disagreement: either value the boards hold, written to all.
            for (byte in pair.bytes.values.distinct().sorted()) {
                TextButton(
                    onClick = { actions.chooseDriveMode(byte) },
                    enabled = editable && DriveMode.of(byte) != null,
                    modifier = Modifier.testTag(machineModeUseTag(byte)),
                ) {
                    Text(stringResource(R.string.setup_machine_mode_use, modeName(byte)))
                }
            }
        }
        PairWrite(state, SetupFields.DRIVE_MODE)
        TakeEffect(SetupFields.DRIVE_MODE)
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
        Text(frameVerdict(state, pair), color = TextSecondary)
        if (pair is FramePair.Differ) Badge()
        PairWrite(state, SetupFields.CONTROL_FRAME)
        TakeEffect(SetupFields.CONTROL_FRAME)
    }
}

/**
 * The Right wheel picker: one chip per motor controller, the labels the selector shows, disabled
 * while the walk has found fewer than two (`specs/rider-ui.md` 3.7a). It preselects nothing.
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
        Text(sideVerdict(state, pair), color = TextSecondary)
        if (pair is SidePair.BothRight) Badge()
        PairWrite(state, SetupFields.CONTROL_SIDE)
        TakeEffect(SetupFields.CONTROL_SIDE)
    }
}

/**
 * What each board said about this row's write, named by the board that said it
 * (`specs/rider-ui.md` 3.7a: a refusal from one of them is shown with that board's own status).
 * Nothing for a board this session has not written.
 */
@Composable
private fun PairWrite(state: SetupState, field: SetupField) {
    for (address in state.boards.keys.sorted()) {
        val write = state.setupOf(address).writes[field.key] ?: continue
        val (line, colour) = writeStatus(field, write)
        Text(
            stringResource(R.string.setup_machine_board_line, boardLabel(state, address), line),
            style = MaterialTheme.typography.bodySmall,
            color = colour,
            modifier = Modifier.testTag(machineWriteTag(field.key, address)),
        )
    }
}
