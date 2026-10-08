package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.Blocked
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupNotice
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.display

@Composable
internal fun fieldLabel(key: Key): String =
    SetupFields.forKey(key)?.let { stringResource(it.label) } ?: key.toString()

@Composable
internal fun changeLine(key: Key, value: Value): String =
    stringResource(R.string.setup_change_line, fieldLabel(key), value.display())

@Suppress("CyclomaticComplexMethod") // one flat branch per SetupNotice, exhaustive over the sealed type
@Composable
internal fun noticeText(n: SetupNotice): String = when (n) {
    SetupNotice.ReadOnlyWhileArmed -> stringResource(R.string.setup_notice_armed)
    is SetupNotice.BoardRefused -> stringResource(refusalText(n.refusal), fieldLabel(n.key))
    is SetupNotice.Mismatch -> stringResource(
        R.string.setup_notice_mismatch,
        fieldLabel(n.key),
        n.wrote.display(),
        n.stored?.display() ?: "?",
    )
    is SetupNotice.Unanswered -> stringResource(R.string.setup_notice_unanswered, fieldLabel(n.key))
    is SetupNotice.Garbled -> stringResource(R.string.setup_notice_garbled, fieldLabel(n.key))
    SetupNotice.SlotBusy -> stringResource(R.string.setup_notice_slot_busy)
    SetupNotice.NotAttached -> stringResource(R.string.setup_notice_not_attached)
    is SetupNotice.FrameRefused -> stringResource(frameText(n.refusal))
    SetupNotice.FrameUnknown -> stringResource(R.string.setup_notice_frame_unknown)
    is SetupNotice.OutOfRange -> stringResource(R.string.setup_notice_out_of_range, fieldLabel(n.key))
    is SetupNotice.LevelUnavailable -> stringResource(blockedText(n.reason))
    is SetupNotice.CheckUnavailable -> stringResource(blockedText(n.reason))
    is SetupNotice.BoardChanged -> stringResource(R.string.setup_notice_board_changed, boardHex(n.previous))
}

private fun frameText(r: Orientation.Refusal): Int = when (r) {
    Orientation.Refusal.NOT_A_SIGN -> R.string.setup_notice_not_a_sign
    Orientation.Refusal.ROLES -> R.string.setup_notice_roles
    Orientation.Refusal.ACCEL_MIRRORED -> R.string.setup_notice_accel_mirrored
    Orientation.Refusal.GYRO_MIRRORED -> R.string.setup_notice_gyro_mirrored
}

private fun refusalText(r: CfgRefusal): Int = when (r) {
    CfgRefusal.ARMED -> R.string.setup_notice_refused_armed
    CfgRefusal.BAD -> R.string.setup_notice_refused_bad
    CfgRefusal.UNKNOWN_KEY -> R.string.setup_notice_refused_unknown
    CfgRefusal.TYPE_MISMATCH -> R.string.setup_notice_refused_type
    CfgRefusal.STORE_ERR -> R.string.setup_notice_refused_store
}

private fun blockedText(b: Blocked): Int = when (b) {
    Blocked.NO_TELEMETRY -> R.string.setup_notice_no_telemetry
    Blocked.NOT_READ -> R.string.setup_notice_not_read
    Blocked.NOT_APPLIED -> R.string.setup_notice_not_applied
}

internal fun boardHex(addr: Int): String = "0x%02X".format(addr)
