package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.BoardError
import com.hoverboard.protocol.board.BoardErrorKind
import com.hoverboard.protocol.board.FieldRef
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.Parsed
import com.hoverboard.protocol.board.Pin
import com.hoverboard.remote.LayoutNotice
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutRows

/** How a packed pin byte reads on its own: a pin name, "not set", or the byte that is neither. */
@Composable
internal fun pinText(raw: Int): String = layoutValueText(Layout.LATCH, raw)

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

/** The field a refusal names, as the screen labels it. */
@Composable
internal fun layoutFieldLabel(ref: FieldRef): String =
    Layout.forField(ref)?.let { layoutFieldLabel(it) } ?: ref.field.name

/** One line of the delta: the field, what the board holds, and what would be written. */
@Composable
internal fun layoutDeltaLine(slot: LayoutSlot, from: Int, to: Int): String = stringResource(
    R.string.layout_delta_line,
    layoutFieldLabel(slot),
    layoutValueText(slot, from),
    layoutValueText(slot, to),
)

/** Why the board refuses this layout, in the terms the field's own row uses. */
@Composable
internal fun layoutRefusalText(error: BoardError): String = stringResource(
    R.string.layout_verdict_invalid,
    layoutFieldLabel(error.field),
    layoutReasonText(error.kind),
)

@Composable
private fun layoutReasonText(kind: BoardErrorKind): String = when (kind) {
    is BoardErrorKind.BadEncoding -> stringResource(R.string.layout_reason_bad_encoding)
    BoardErrorKind.IncompleteGroup -> stringResource(R.string.layout_reason_incomplete)
    BoardErrorKind.MissingDeadTime -> stringResource(R.string.layout_reason_dead_time)
    is BoardErrorKind.DuplicatePin -> stringResource(R.string.layout_reason_duplicate, kind.pin.name)
    is BoardErrorKind.ReservedPin -> stringResource(R.string.layout_reason_reserved, kind.pin.name)
    is BoardErrorKind.UnknownPin -> stringResource(R.string.layout_reason_unknown_pin, kind.pin.name)
    is BoardErrorKind.GateCapableMisused -> stringResource(R.string.layout_reason_gate_capable, kind.pin.name)
    BoardErrorKind.InvalidGateSet -> stringResource(R.string.layout_reason_invalid_gate_set)
    is BoardErrorKind.NotAdcCapable -> stringResource(R.string.layout_reason_not_adc, kind.pin.name)
    BoardErrorKind.NotI2cPair -> stringResource(R.string.layout_reason_not_i2c)
    is BoardErrorKind.ImuFrame -> stringResource(R.string.layout_reason_imu_frame)
}
