package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.ModePair
import com.hoverboard.remote.R
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.BoardMode
import com.hoverboard.remote.model.DriveFallback
import com.hoverboard.remote.model.driveModeLabel
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.DarkBackground
import com.hoverboard.remote.ui.theme.ZeroLine

/** Test tag on the chip for the motor controller at [address], read by the Ride screen's tests. */
fun boardChipTag(address: Int): String = "board_chip_${boardHex(address)}"

/** Test tag on the machine disagreement chip. */
const val BOARD_CHIP_MACHINE_TAG = "board_chip_machine"

/** Test tag on the read-only chip naming the drive mode in force. */
const val DRIVE_CHIP_TAG = "drive_chip"

/** Test tag on the line saying why the drive fell back to SINGLE. */
const val DRIVE_FALLBACK_TAG = "drive_fallback"

/**
 * The Ride screen's status strip (`specs/rider-ui.md` 3.1 and 3.2): one chip per motor controller,
 * each saying what THAT board says about itself, and one read-only chip naming the drive mode in
 * force. A bridge has no chip: it has no state to report.
 *
 * Each board reports under its own address and the app keeps them apart by it
 * ([UiState.telemetryOf]), so no chip speaks for another board. The drive mode is the machine's
 * stored setting, edited on Setup; the strip offers no selector, and when the app drives in SINGLE
 * because the stored mode cannot be trusted or cannot run here, the line under it says why.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun BoardChips(state: UiState) {
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
        BoardChip(
            text = stringResource(R.string.drive_chip, stringResource(driveModeLabel(state.driveMode))),
            color = if (state.drive.fallback == null) ZeroLine else AccentYellow,
            tag = DRIVE_CHIP_TAG,
        )
        for (node in state.motorControllers) {
            val telemetry = state.telemetryOf(node.address)
            BoardChip(
                text = stringResource(R.string.board_chip, nodeLabel(node), stringResource(boardSays(telemetry))),
                color = boardColor(telemetry),
                tag = boardChipTag(node.address),
            )
        }
        if (state.attachedNode != null && state.motorControllers.isEmpty()) {
            BoardChip(text = stringResource(R.string.board_chip_none), color = ZeroLine, tag = BOARD_CHIP_NONE_TAG)
        }
        machineBadge(state)?.let {
            BoardChip(text = stringResource(it), color = AccentRed, tag = BOARD_CHIP_MACHINE_TAG)
        }
    }
    driveFallbackText(state)?.let {
        Text(
            text = it,
            style = MaterialTheme.typography.labelMedium,
            color = AccentYellow,
            modifier = Modifier.testTag(DRIVE_FALLBACK_TAG),
        )
    }
}

/** Test tag on the chip shown while no motor controller is known yet. */
const val BOARD_CHIP_NONE_TAG = "board_chip_none"

/**
 * Why the drive is SINGLE although the machine stores something else, or cannot say what it stores
 * (`specs/rider-ui.md` 3.2: "the Ride status strip says which mode is in force and why"). Null while
 * the stored mode is the one in force, and while no node is attached.
 */
@Composable
private fun driveFallbackText(state: UiState): String? {
    if (state.attachedNode == null) return null
    val stored = state.drive.stored?.let { stringResource(driveModeLabel(it)) } ?: ""
    return when (state.drive.fallback ?: return null) {
        DriveFallback.UNREAD -> stringResource(R.string.drive_fallback_unread)
        DriveFallback.DISAGREE -> stringResource(R.string.drive_fallback_disagree)
        DriveFallback.UNKNOWN_MODE -> stringResource(R.string.drive_fallback_unknown_mode)
        DriveFallback.NOT_TWO -> stringResource(R.string.drive_fallback_not_two, stored, state.motorControllers.size)
        DriveFallback.NO_LEADER -> stringResource(R.string.drive_fallback_no_leader, stored)
        DriveFallback.NO_RIGHT_WHEEL -> stringResource(R.string.drive_fallback_no_right, stored)
    }
}

/**
 * The disagreement badge of `specs/rider-ui.md` 3.7, beside the board chips: the boards hold frames
 * or drive modes that differ, more than one claims the right wheel, or the machine runs rigid with
 * none on the right. Null when there is nothing to say, which includes a fresh pivoting machine with
 * every board on the left: that is the factory state and it is inert.
 */
@StringRes
private fun machineBadge(state: UiState): Int? {
    if (state.attachedNode == null) return null
    val machine = state.machine
    return when {
        machine.frame is FramePair.Differ -> R.string.board_chip_frame_differs
        machine.mode is ModePair.Differ -> R.string.board_chip_mode_differs
        machine.side is SidePair.BothRight -> R.string.board_chip_both_right
        machine.disagrees -> R.string.board_chip_no_right
        else -> null
    }
}

/**
 * What a board's chip says about it, from that board's own frames: nothing at all until it has
 * been heard from and again once it has gone quiet, then a fault or a lockdown ahead of the mode,
 * because either one is what an owner needs off a glance and the mode alone would hide it.
 *
 * Never the word ARMED: the arm control is the one armed statement on this screen. A board in
 * [BoardMode.RUN] is a board whose motors are live, which is the fact, and the control above says
 * who asked for it.
 */
private fun boardSays(telemetry: TelemetryUi?): Int = when {
    telemetry == null || !telemetry.hasState -> R.string.board_state_quiet
    telemetry.anyFault -> R.string.board_state_fault
    telemetry.lockdown -> R.string.board_state_lockdown
    else -> when (telemetry.mode) {
        BoardMode.OFF -> R.string.board_state_off
        BoardMode.INIT, BoardMode.READY -> R.string.board_state_starting
        BoardMode.RUN -> R.string.board_state_live
        BoardMode.SHUTDOWN -> R.string.board_state_stopping
        else -> R.string.board_state_unrecognised
    }
}

/** The same three-way reading in colour: trouble, live, and everything quiet or idle. */
private fun boardColor(telemetry: TelemetryUi?): Color = when {
    telemetry == null || !telemetry.hasState -> ZeroLine
    telemetry.anyFault -> AccentRed
    telemetry.lockdown -> AccentYellow
    telemetry.mode == BoardMode.RUN -> AccentGreen
    else -> ZeroLine
}

@Composable
private fun BoardChip(text: String, color: Color, tag: String) {
    val shape = RoundedCornerShape(12.dp)
    Text(
        text = text,
        style = MaterialTheme.typography.labelMedium,
        color = DarkBackground,
        modifier = Modifier
            .testTag(tag)
            .background(color, shape)
            .padding(horizontal = 10.dp, vertical = 4.dp),
    )
}
