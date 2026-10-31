package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.Parsed
import com.hoverboard.protocol.board.Pin
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutRows

/** How one layout field's raw value reads: a pin name, "not set", or the byte that is neither. */
@Composable
internal fun layoutValueText(slot: LayoutSlot, raw: Int): String {
    if (!slot.isPin) return raw.toString()
    return when (val p = Pin.parse(raw)) {
        Parsed.Absent -> stringResource(R.string.layout_absent)
        is Parsed.Valid -> p.pin.name
        Parsed.Invalid -> "0x%02X".format(raw)
    }
}

/** A field's label, by the slot it edits. */
@Composable
internal fun layoutFieldLabel(slot: LayoutSlot): String =
    LayoutRows.forSlot(slot)?.let { stringResource(it.label) } ?: slot.key.toString()

/** One line of the delta: the field, what the board holds, and what would be written. */
@Composable
internal fun layoutDeltaLine(slot: LayoutSlot, from: Int, to: Int): String = stringResource(
    R.string.layout_delta_line,
    layoutFieldLabel(slot),
    layoutValueText(slot, from),
    layoutValueText(slot, to),
)
