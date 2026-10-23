package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.Blocked
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupNotice
import com.hoverboard.remote.WriteState
import com.hoverboard.remote.model.SetupFields

@Composable
internal fun fieldLabel(key: Key): String =
    SetupFields.forKey(key)?.let { stringResource(it.label) } ?: key.toString()

@Composable
internal fun noticeText(n: SetupNotice): String = when (n) {
    is SetupNotice.FrameRefused -> stringResource(frameText(n.refusal))
    is SetupNotice.OutOfRange -> stringResource(R.string.setup_notice_out_of_range, fieldLabel(n.key))
    is SetupNotice.LevelUnavailable -> stringResource(blockedText(n.reason))
    is SetupNotice.CheckUnavailable -> stringResource(blockedText(n.reason))
    is SetupNotice.BoardChanged -> stringResource(R.string.setup_notice_board_changed, boardHex(n.previous))
}

/**
 * What one row says about a write of its own field that did not simply succeed
 * (`specs/rider-ui.md` 3.6a). Each names this field and nothing else, because each edit is its own
 * transaction; [WriteState.Mismatch] takes the value written and the value echoed.
 */
internal fun writeFailureText(state: WriteState): Int = when (state) {
    is WriteState.Refused -> refusalText(state.refusal)
    is WriteState.Mismatch -> R.string.setup_write_mismatch
    WriteState.Unanswered -> R.string.setup_write_unanswered
    WriteState.Garbled -> R.string.setup_write_garbled
    WriteState.SlotBusy -> R.string.setup_write_slot_busy
    WriteState.NotSent -> R.string.setup_write_not_sent
    WriteState.Skipped -> R.string.setup_write_skipped
    WriteState.Sending, WriteState.Written -> error("not a failure: $state")
}

private fun frameText(r: Orientation.Refusal): Int = when (r) {
    Orientation.Refusal.NOT_A_SIGN -> R.string.setup_notice_not_a_sign
    Orientation.Refusal.ROLES -> R.string.setup_notice_roles
    Orientation.Refusal.ACCEL_MIRRORED -> R.string.setup_notice_accel_mirrored
    Orientation.Refusal.GYRO_MIRRORED -> R.string.setup_notice_gyro_mirrored
}

private fun refusalText(r: CfgRefusal): Int = when (r) {
    CfgRefusal.ARMED -> R.string.setup_write_refused_armed
    CfgRefusal.BAD -> R.string.setup_write_refused_bad
    CfgRefusal.UNKNOWN_KEY -> R.string.setup_write_refused_unknown
    CfgRefusal.TYPE_MISMATCH -> R.string.setup_write_refused_type
    CfgRefusal.STORE_ERR -> R.string.setup_write_refused_store
    CfgRefusal.NO_SPACE -> R.string.setup_write_refused_no_space
    CfgRefusal.COMMIT_FAILED -> R.string.setup_write_refused_commit
}

private fun blockedText(b: Blocked): Int = when (b) {
    Blocked.NO_TELEMETRY -> R.string.setup_notice_no_telemetry
    Blocked.NOT_READ -> R.string.setup_notice_not_read
    Blocked.AWAITING_ARM -> R.string.setup_notice_awaiting_arm
    Blocked.AWAITING_RESTART -> R.string.setup_notice_awaiting_restart
}

internal fun boardHex(addr: Int): String = "0x%02X".format(addr)
