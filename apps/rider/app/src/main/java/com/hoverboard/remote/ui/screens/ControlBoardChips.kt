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
import com.hoverboard.remote.R
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.BoardMode
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.DarkBackground
import com.hoverboard.remote.ui.theme.ZeroLine

/** Test tags for the per-board chips, read by the Ride screen's tests. */
const val BOARD_CHIP_MASTER_TAG = "board_chip_master"
const val BOARD_CHIP_SLAVE_TAG = "board_chip_slave"
const val BOARD_CHIP_MACHINE_TAG = "board_chip_machine"

/**
 * One chip per board (`specs/rider-ui.md` 3.2: partial states are shown, not hidden), each saying
 * what THAT board says about itself.
 *
 * The two chips are built the same way, because both boards report now. The board the phone is
 * attached to sends its own state, and it re-emits its partner's under the partner's address, so
 * the app keeps the two apart by the address each arrived under ([UiState.telemetryOf]) and
 * neither chip speaks for the other board. The second chip used to say what the app was SENDING
 * the second board, which was a guess about a board that had never answered.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun BoardChips(state: UiState) {
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
        BoardChip(
            text = stringResource(
                R.string.board_chip_master,
                state.masterBoard?.let { boardHex(it) } ?: "?",
                stringResource(boardSays(state.telemetry)),
            ),
            color = boardColor(state.telemetry),
            tag = BOARD_CHIP_MASTER_TAG,
        )
        machineBadge(state)?.let {
            BoardChip(text = stringResource(it), color = AccentRed, tag = BOARD_CHIP_MACHINE_TAG)
        }
        val slave = state.slaveBoard
        BoardChip(
            text = if (slave == null) {
                stringResource(R.string.board_chip_no_slave)
            } else {
                stringResource(
                    R.string.board_chip_slave,
                    boardHex(slave),
                    stringResource(boardSays(state.slaveTelemetry)),
                )
            },
            color = if (slave == null) ZeroLine else boardColor(state.slaveTelemetry),
            tag = BOARD_CHIP_SLAVE_TAG,
        )
    }
}

/**
 * The disagreement badge of `specs/rider-ui.md` 3.7, beside the board chips: the pair holds frames
 * that differ, both boards claim the right wheel, or the machine runs rigid with neither on the
 * right. Null when there is nothing to say, which includes a fresh pivoting pair with both boards
 * on the left: that is the factory state and it is inert.
 */
@StringRes
private fun machineBadge(state: UiState): Int? {
    if (state.masterBoard == null) return null
    val machine = state.machine
    return when {
        machine.frame is FramePair.Differ -> R.string.board_chip_frame_differs
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
