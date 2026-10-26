package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
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
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.BoardGains
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import kotlin.math.roundToInt

/** Test tag on the reference section's heading. */
const val TUNE_REFERENCE_TAG = "tune_reference"

/** Test tag on the one line saying whether the stick does anything. */
const val TUNE_REFERENCE_STATE_TAG = "tune_reference_state"

/** Test tag on reference row [index]. */
fun tuneReferenceRowTag(index: Int): String = "tune_reference_row_$index"

/** Test tag on reference row [index]'s slider. */
fun tuneReferenceSliderTag(index: Int): String = "tune_reference_slider_$index"

/** Test tag on reference row [index]'s typed entry. */
fun tuneReferenceEntryTag(index: Int): String = "tune_reference_entry_$index"

/** Test tag on reference row [index]'s Set button. */
fun tuneReferenceSetTag(index: Int): String = "tune_reference_set_$index"

/** What the screen calls each reference word, in key-index order. */
private val REFERENCE_LABELS = listOf(R.string.tune_reference_speed_max, R.string.tune_reference_speed_slew)

/** What each word does, in the units of `specs/state-feedback.md` section 4. */
private val REFERENCE_NOTES =
    listOf(R.string.tune_reference_speed_max_note, R.string.tune_reference_speed_slew_note)

/**
 * The reference row (`specs/state-feedback.md` sections 4 and 10): `speed_max` and `speed_slew` of
 * `CONTROL_REFERENCE`, which turn the stick into a target speed and slew it, the target speed then
 * integrating into the position reference the law holds.
 *
 * A section of its own under the gains, on the same RAM lane: the heading says Save is what makes
 * either survive a power cycle, and each row carries the flash value and the unsaved mark. No
 * converging mark: the firmware does not ramp this row, so the staged value is what the law uses on
 * its next tick, which is also why the state line may say the stick is dead rather than hedging.
 */
@Composable
internal fun Reference(state: TuneState, armed: Boolean, actions: TuneActions) {
    Text(
        stringResource(R.string.tune_reference_title),
        style = MaterialTheme.typography.titleMedium,
        color = TextPrimary,
        modifier = Modifier.testTag(TUNE_REFERENCE_TAG),
    )
    Caption(R.string.tune_reference_body)
    val dead = state.stickDead
    Text(
        stringResource(
            when (dead) {
                null -> R.string.tune_reference_unread
                true -> R.string.tune_reference_dead
                false -> R.string.tune_reference_live
            },
        ),
        style = MaterialTheme.typography.titleSmall,
        color = if (dead == false) TextPrimary else AccentYellow,
        modifier = Modifier.testTag(TUNE_REFERENCE_STATE_TAG),
    )
    if (armed) Text(stringResource(R.string.tune_reference_armed), color = AccentRed)
    for (i in state.referenceKeys.indices) ReferenceRow(state, i, actions)
}

/**
 * One reference word: what it is in its units, the staged value as last read, the slider, the
 * compiled range, the typed entry for an exact number, and the flash value with its unsaved mark.
 *
 * Slider AND entry for 3.3b's reason: the useful values (80 for `speed_max`, 41 for `speed_slew`)
 * are small against their tracks, which no thumb places exactly.
 */
@Composable
private fun ReferenceRow(state: TuneState, index: Int, actions: TuneActions) {
    val key = state.referenceKeys[index]
    val gains = state.gains
    val staged = gains.staged[key]
    val range = gains.min(key)..gains.max(key)
    Panel(modifier = Modifier.testTag(tuneReferenceRowTag(index))) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                stringResource(REFERENCE_LABELS[index]),
                style = MaterialTheme.typography.titleSmall,
                color = TextPrimary,
                modifier = Modifier.weight(1f),
            )
            Text(
                staged?.toString() ?: stringResource(R.string.tune_unread),
                style = MaterialTheme.typography.titleLarge,
                color = TextPrimary,
            )
        }
        ReferenceSlider(state, key, index, staged, range, actions)
        Caption(R.string.tune_reference_range, listOf(range.first.toString(), range.last.toString()))
        Caption(REFERENCE_NOTES[index])
        FlashLine(gains, key)
        LiveEntry(
            staged = staged,
            range = range,
            enabled = state.tunable && staged != null,
            entryTag = tuneReferenceEntryTag(index),
            setTag = tuneReferenceSetTag(index),
        ) { actions.slideEnd(key, it) }
    }
}

/**
 * The word's slider, over the COMPILED range (`control::config::sf::REFERENCE_MAX`, mirrored as
 * [Gains.REFERENCE_MAXIMA] and [Gains.REFERENCE_MINIMA]): there is no per-board maximum to read.
 *
 * The thumb follows the finger during a drag and the staged value otherwise, which is the gain
 * slider's rule and the reason is the same: the staged value comes back a cadence later.
 */
@Composable
private fun ReferenceSlider(
    state: TuneState,
    key: Key,
    index: Int,
    staged: Int?,
    range: IntRange,
    actions: TuneActions,
) {
    var dragging by remember { mutableStateOf(false) }
    var position by remember { mutableFloatStateOf(0f) }
    val shown = if (dragging) position else (staged ?: range.first).toFloat()
    Slider(
        value = shown.coerceIn(range.first.toFloat(), range.last.toFloat()),
        valueRange = range.first.toFloat()..range.last.toFloat(),
        enabled = state.tunable && staged != null,
        onValueChange = {
            dragging = true
            position = it
            actions.slide(key, it.roundToInt())
        },
        onValueChangeFinished = {
            actions.slideEnd(key, position.roundToInt())
            dragging = false
        },
        modifier = Modifier
            .fillMaxWidth()
            .testTag(tuneReferenceSliderTag(index)),
    )
}

/**
 * The exact number, typed, for a row of the live tune lane (a gain or a reference word): [onSet]
 * sends it as one `TUNE_WRITE` and its re-read, the same thing a released slider sends.
 *
 * An out-of-range number is impossible rather than refused: Set is off until the text is a whole
 * number inside [range], so the board is never asked for a value its seam would refuse, and the
 * reason is on screen while it is off. The text starts at, and follows, the [staged] value read
 * back.
 */
@Composable
internal fun LiveEntry(
    staged: Int?,
    range: IntRange,
    enabled: Boolean,
    entryTag: String,
    setTag: String,
    onSet: (Int) -> Unit,
) {
    var text by remember(staged) { mutableStateOf(staged?.toString().orEmpty()) }
    val parsed = text.toIntOrNull()?.takeIf { it in range }
    Row(verticalAlignment = Alignment.CenterVertically) {
        OutlinedTextField(
            value = text,
            onValueChange = { text = it },
            enabled = enabled,
            singleLine = true,
            isError = text.isNotEmpty() && parsed == null,
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            modifier = Modifier
                .weight(1f)
                .testTag(entryTag),
        )
        Spacer(modifier = Modifier.width(8.dp))
        Button(
            onClick = { parsed?.let(onSet) },
            enabled = enabled && parsed != null,
            modifier = Modifier.testTag(setTag),
        ) { Text(stringResource(R.string.setup_set)) }
    }
    if (text.isNotEmpty() && parsed == null) {
        Text(
            stringResource(R.string.tune_reference_entry_invalid, range.first.toString(), range.last.toString()),
            style = MaterialTheme.typography.bodySmall,
            color = AccentRed,
        )
    }
}

/**
 * The flash value of [key] and, when staged differs from it, the unsaved mark. A gain row and a
 * reference row both end with it.
 */
@Composable
internal fun FlashLine(gains: BoardGains, key: Key) {
    Row {
        Text(
            stringResource(R.string.tune_flash, gains.flash[key]?.toString() ?: stringResource(R.string.tune_unread)),
            color = TextSecondary,
        )
        if (gains.unsaved(key)) Text(" " + stringResource(R.string.tune_unsaved_mark), color = AccentYellow)
    }
}
