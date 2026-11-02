package com.hoverboard.remote.ui.screens

import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.BoardStatus
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Names
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.WarningText

/** Test tag on the [index]th boot status record line of the motor controller at [address]. */
fun statusEntryTag(address: Int, index: Int): String = "status_entry_${boardHex(address)}_$index"

/**
 * What the board at [address]'s last boot configured and could not bring up: one warning line per
 * boot status record entry, read at connect with `STATUS_READ` (`specs/configured-links.md`
 * section 5). Nothing while the list is empty. While the board's live word carries CONFIG
 * ([CyclicState.EV_CONFIG]) and the list says nothing (unread, or read before a restart), one line
 * says so. A boot fact, so it is shown whether or not the board is reporting.
 */
@Composable
internal fun ConfigLines(state: UiState, address: Int, telemetry: TelemetryUi?) {
    val small = MaterialTheme.typography.bodySmall
    val entries = state.boardStatus[address].orEmpty()
    for ((i, entry) in entries.withIndex()) {
        WarningText(boardStatusLine(entry), style = small, modifier = Modifier.testTag(statusEntryTag(address, i)))
    }
    val configBit = (telemetry?.cyclic?.fault ?: 0) and CyclicState.EV_CONFIG != 0
    if (entries.isEmpty() && configBit) CautionText(stringResource(R.string.status_config_unread), style = small)
}

/**
 * One boot status record entry in words (`specs/configured-links.md` section 5): the function it names
 * and why it did not come up, the way `hoverboard-cli board status` prints it, e.g. "IMU: did not
 * answer its probe". The configuration is checked before it is written, so the one reason a board
 * records is a peripheral that did not come up.
 */
@Composable
internal fun boardStatusLine(entry: BoardStatus.Entry): String =
    stringResource(R.string.status_config_line, statusFunction(entry), statusReason(entry))

/**
 * The function an entry names, by its field (byte 0): a group by what it is, and anything else by
 * its store path.
 */
@Composable
private fun statusFunction(entry: BoardStatus.Entry): String = when (entry.field) {
    Fields.MOTOR_GATE_HI_A.id -> stringResource(R.string.status_fn_gate, entry.index)
    Fields.MOTOR_HALL_A.id -> stringResource(R.string.status_fn_hall, entry.index)
    Fields.MOTOR_PHASE_A.id -> stringResource(R.string.status_fn_current, entry.index)
    Fields.LINK_TX.id -> stringResource(R.string.status_fn_link, entry.index)
    Fields.BOARD_BATTERY_PIN.id -> stringResource(R.string.status_fn_battery)
    Fields.IMU_SCL_PIN.id -> stringResource(R.string.status_fn_imu)
    else -> Names.byId(entry.field)?.let { Names.pathOf(Key(entry.field, entry.index)) }
        ?: stringResource(R.string.status_fn_unknown, hex(entry.field), entry.index)
}

@Composable
private fun statusReason(entry: BoardStatus.Entry): String = when (entry.reason) {
    BoardStatus.Reason.NOT_UP ->
        if (entry.detail == 0) {
            stringResource(R.string.status_reason_no_answer)
        } else {
            stringResource(R.string.status_reason_not_up, entry.detail)
        }
    null -> stringResource(R.string.status_reason_unknown, entry.reasonCode, entry.detail)
}

internal fun hex(b: Int): String = "0x%02X".format(b)
