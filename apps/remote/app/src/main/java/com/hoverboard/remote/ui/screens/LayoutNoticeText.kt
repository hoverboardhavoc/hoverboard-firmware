package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.LayoutNotice
import com.hoverboard.remote.R

/**
 * What the layout editor says about the last operation: one flat branch per [LayoutNotice], which is
 * more than detekt's complexity threshold and is exhaustive over the sealed type, so a notice added
 * to the model fails to compile until it has words here.
 */
@Suppress("CyclomaticComplexMethod")
@Composable
internal fun layoutNoticeText(n: LayoutNotice, chip: Chip?): String = when (n) {
    LayoutNotice.ReadOnlyWhileArmed -> stringResource(R.string.layout_notice_armed)
    LayoutNotice.NotAttached -> stringResource(R.string.layout_notice_not_attached)
    LayoutNotice.NotRead -> stringResource(R.string.layout_not_read)
    LayoutNotice.SlotBusy -> stringResource(R.string.layout_notice_slot_busy)
    is LayoutNotice.BoardRefused -> stringResource(R.string.layout_notice_refused, keyLabel(n.key))
    is LayoutNotice.Mismatch -> stringResource(
        R.string.layout_notice_mismatch,
        keyLabel(n.key),
        n.wrote?.let { valueOfKey(n.key, it, chip) } ?: stringResource(R.string.layout_absent),
        n.stored?.let { valueOfKey(n.key, it, chip) } ?: "?",
    )
    is LayoutNotice.Unanswered -> stringResource(R.string.layout_notice_unanswered, keyLabel(n.key))
    is LayoutNotice.Garbled -> stringResource(R.string.layout_notice_garbled, keyLabel(n.key))
    is LayoutNotice.BoardChanged -> stringResource(R.string.layout_notice_board_changed, boardHex(n.previous))
    is LayoutNotice.CheckRefused -> stringResource(R.string.layout_notice_check_refused, n.refusal.msg)
    LayoutNotice.ChipUnknown -> stringResource(R.string.layout_notice_chip_unknown)
}

@Composable
private fun keyLabel(key: Key): String = Layout.forKey(key)?.let { layoutFieldLabel(it) } ?: key.toString()

@Composable
private fun valueOfKey(key: Key, value: Value, chip: Chip?): String =
    Layout.forKey(key)?.let { layoutValueText(value, chip) } ?: value.toString()
