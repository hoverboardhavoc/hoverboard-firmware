package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.asLong
import com.hoverboard.remote.model.display
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import kotlin.math.roundToInt

/** Test tag on drive bound [key]'s row. */
fun tuneBoundTag(key: Key): String = "tune_bound_${key.fieldId}_${key.index}"

/** Test tag on drive bound [key]'s slider. */
fun tuneBoundSliderTag(key: Key): String = "tune_bound_slider_${key.fieldId}_${key.index}"

/**
 * One drive bound (`specs/remote-ui.md` 3.3b): the value the board stores, a slider over the range
 * the firmware keeps, the typed entry for an exact number, and the generated line saying when the
 * board will start using it.
 *
 * Two controls for one value, deliberately. The slider is the instrument the bounds are FOUND
 * with, the gains' argument of 3.3a applied to a value that is swept until the machine feels
 * right; the entry is how a number already known is set exactly, which a slider over 28500 counts
 * cannot do and which is what a session reads back from its notes. Both end in the same write.
 *
 * Unlike a gain slider, a drag sends nothing. Each write appends a record to the board's flash, so
 * the value goes once, when the finger lifts.
 */
@Composable
internal fun BoundRow(state: TuneState, field: SetupField, armed: Boolean, actions: TuneActions) {
    val range = (field.editor as Editor.Range).range
    val stored = state.bound(field.key)
    val editable = state.writable && !armed && stored != null
    Panel(modifier = Modifier.testTag(tuneBoundTag(field.key))) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                stringResource(field.label),
                style = MaterialTheme.typography.titleSmall,
                color = Fg,
                modifier = Modifier.weight(1f),
            )
            Text(
                stored?.toString() ?: stringResource(R.string.tune_bound_unread),
                style = MaterialTheme.typography.titleLarge,
                color = Fg,
            )
        }
        BoundSlider(field, range, stored, editable, actions)
        RangeNote(field)
        if (field.showDefault) {
            Text(
                stringResource(R.string.setup_default, field.def.default.display()),
                style = MaterialTheme.typography.bodySmall,
                color = FgMuted,
            )
        }
        field.note?.let { Caption(it, field.noteArgs) }
        TakeEffect(field)
        Entry(field, stored?.let { Value.I16(it) }, editable) { v ->
            v.asLong()?.let { actions.setBound(field.key, it.toInt()) }
        }
    }
}

/**
 * The bound's slider, over the whole range the firmware keeps rather than over what the board
 * holds: the range is a clamp, so every position on this track is a value the board will store as
 * written.
 *
 * The thumb follows the finger while the drag lasts and the stored value otherwise, which is the
 * gain slider's rule. What differs is the release: that is the only moment a value is sent.
 */
@Composable
private fun BoundSlider(
    field: SetupField,
    range: LongRange,
    stored: Int?,
    editable: Boolean,
    actions: TuneActions,
) {
    var dragging by remember { mutableStateOf(false) }
    var position by remember { mutableFloatStateOf(0f) }
    val shown = if (dragging) position else (stored ?: range.first.toInt()).toFloat()
    Slider(
        value = shown.coerceIn(range.first.toFloat(), range.last.toFloat()),
        valueRange = range.first.toFloat()..range.last.toFloat(),
        enabled = editable,
        onValueChange = {
            dragging = true
            position = it
        },
        onValueChangeFinished = {
            actions.setBound(field.key, position.roundToInt())
            dragging = false
        },
        modifier = Modifier
            .fillMaxWidth()
            .testTag(tuneBoundSliderTag(field.key)),
    )
}
