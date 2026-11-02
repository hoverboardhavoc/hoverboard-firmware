package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.store.Value
import com.hoverboard.protocol.store.fleetPin
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutRows

/** A pin as a line names it: `PA8`. */
internal fun pinName(port: Int, pin: Int): String = "P${'A' + port}$pin"

/**
 * A layout value as it reads: a pin by name with its alternate function (left out on a [chip] that
 * remaps, where the stored `af` is not used) or its ADC channel, a number, or null for one this
 * reads no other way.
 */
internal fun valueText(value: Value, chip: Chip?): String? = when (value) {
    is Value.Pin -> pinName(value.port, value.pin)
    is Value.PinAf -> pinName(value.port, value.pin) + if (chip?.remaps == true) "" else " AF${value.af}"
    is Value.PinAdc -> pinName(value.port, value.pin) + " ADC${value.adc}"
    is Value.U8 -> value.v.toString()
    is Value.U16 -> value.v.toString()
    else -> null
}

/**
 * How one layout key's stored value reads: the value, or for a key the board stores nothing for, the
 * fleet pin it falls back to or "not set".
 */
@Composable
internal fun layoutValueText(slot: LayoutSlot, value: Value?, chip: Chip?): String {
    if (value == null) {
        return slot.def.fleetPin?.let { (port, pin) -> pinName(port, pin) } ?: stringResource(R.string.layout_absent)
    }
    return valueText(value, chip) ?: value.toString()
}

/** A field's label, by the slot it edits. */
@Composable
internal fun layoutFieldLabel(slot: LayoutSlot): String =
    LayoutRows.forSlot(slot)?.let { stringResource(it.label) } ?: slot.key.toString()

/** One line of the delta: the field, what the board holds, and what would be written. */
@Composable
internal fun layoutDeltaLine(slot: LayoutSlot, from: Value?, to: Value?, chip: Chip?): String = stringResource(
    R.string.layout_delta_line,
    layoutFieldLabel(slot),
    layoutValueText(slot, from, chip),
    layoutValueText(slot, to, chip),
)
