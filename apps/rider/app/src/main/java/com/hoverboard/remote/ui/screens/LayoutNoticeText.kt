package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.PIN_ABSENT
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
internal fun layoutNoticeText(n: LayoutNotice): String = when (n) {
    LayoutNotice.ReadOnlyWhileArmed -> stringResource(R.string.layout_notice_armed)
    LayoutNotice.NotAttached -> stringResource(R.string.layout_notice_not_attached)
    LayoutNotice.NotRead -> stringResource(R.string.layout_not_read)
    LayoutNotice.NoPartSelected -> stringResource(R.string.layout_notice_no_part)
    LayoutNotice.VerdictNotClean -> stringResource(R.string.layout_notice_not_clean)
    LayoutNotice.SlotBusy -> stringResource(R.string.layout_notice_slot_busy)
    is LayoutNotice.BoardRefused -> stringResource(R.string.layout_notice_refused, keyLabel(n.key))
    is LayoutNotice.Mismatch -> stringResource(
        R.string.layout_notice_mismatch,
        keyLabel(n.key),
        valueOfKey(n.key, n.wrote.let { v -> rawOf(n.key, v) }),
        n.stored?.let { valueOfKey(n.key, rawOf(n.key, it)) } ?: "?",
    )
    is LayoutNotice.Unanswered -> stringResource(R.string.layout_notice_unanswered, keyLabel(n.key))
    is LayoutNotice.Garbled -> stringResource(R.string.layout_notice_garbled, keyLabel(n.key))
    is LayoutNotice.BoardChanged -> stringResource(R.string.layout_notice_board_changed, boardHex(n.previous))
}

@Composable
private fun keyLabel(key: com.hoverboard.protocol.store.Key): String =
    Layout.forKey(key)?.let { layoutFieldLabel(it) } ?: key.toString()

private fun rawOf(key: com.hoverboard.protocol.store.Key, value: com.hoverboard.protocol.store.Value): Int =
    Layout.forKey(key)?.raw(value) ?: PIN_ABSENT

@Composable
private fun valueOfKey(key: com.hoverboard.protocol.store.Key, raw: Int): String =
    Layout.forKey(key)?.let { layoutValueText(it, raw) } ?: raw.toString()
