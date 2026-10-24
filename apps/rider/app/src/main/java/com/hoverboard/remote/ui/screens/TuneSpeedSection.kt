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
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import kotlin.math.roundToInt

/** Test tag on the speed loop's heading. */
const val TUNE_SPEED_TAG = "tune_speed"

/** Test tag on the one line saying whether the loop is running. */
const val TUNE_SPEED_STATE_TAG = "tune_speed_state"

/** Test tag on speed-loop row [index]. */
fun tuneSpeedRowTag(index: Int): String = "tune_speed_row_$index"

/** Test tag on speed-loop row [index]'s slider. */
fun tuneSpeedSliderTag(index: Int): String = "tune_speed_slider_$index"

/** Test tag on speed-loop row [index]'s typed entry. */
fun tuneSpeedEntryTag(index: Int): String = "tune_speed_entry_$index"

/** Test tag on speed-loop row [index]'s Set button. */
fun tuneSpeedSetTag(index: Int): String = "tune_speed_set_$index"

/** What the screen calls each speed-loop word, in key-index order. */
private val SPEED_LABELS = listOf(R.string.tune_speed_kp, R.string.tune_speed_ki, R.string.tune_speed_max)

/** What each word does, in the units the loop takes them in (`crates/control/src/outer.rs`, `OuterSpeed`). */
private val SPEED_NOTES =
    listOf(R.string.tune_speed_kp_note, R.string.tune_speed_ki_note, R.string.tune_speed_max_note)

/**
 * The outer speed loop's three words (`specs/rider-ui.md` 3.3c): `speed_kp`, `speed_ki` and
 * `speed_max` of `CONTROL_SPEED_LOOP`, which decide whether the stick commands SPEED or lean and
 * how fast a full stick asks to go.
 *
 * A section of its own rather than a third chip beside the two gain sets. Those two are picked by
 * the BOARD, by whether anybody is on the pads, and the line under the chips says so; this row is
 * picked by nothing and sits over whichever set is running. A chip would also hide it whenever a
 * gain set was selected, and `speed_kp` at 0 is the one value on the screen that changes what the
 * stick means, so it is the last thing to hide.
 *
 * Written over the gains' lane, which is RAM: the heading says that Save is what makes any of it
 * survive a power cycle, and each row carries the flash value and the unsaved mark that say
 * whether it has. No converging mark anywhere here: the firmware does not ramp this row
 * (`crates/control/src/outer.rs`, `OuterSpeed`), so the staged value is what the loop uses on its
 * next tick, which is also why the state line may say the loop is off rather than hedging.
 */
@Composable
internal fun SpeedLoop(state: TuneState, armed: Boolean, actions: TuneActions) {
    Text(
        stringResource(R.string.tune_speed_title),
        style = MaterialTheme.typography.titleMedium,
        color = TextPrimary,
        modifier = Modifier.testTag(TUNE_SPEED_TAG),
    )
    Caption(R.string.tune_speed_body)
    val off = state.speedLoopOff
    Text(
        stringResource(
            when (off) {
                null -> R.string.tune_speed_unread
                true -> R.string.tune_speed_off
                false -> R.string.tune_speed_on
            },
        ),
        style = MaterialTheme.typography.titleSmall,
        color = if (off == false) TextPrimary else AccentYellow,
        modifier = Modifier.testTag(TUNE_SPEED_STATE_TAG),
    )
    if (armed) Text(stringResource(R.string.tune_speed_armed), color = AccentRed)
    for (i in 0 until Gains.PER_PROFILE) SpeedRow(state, i, actions)
}

/**
 * One speed-loop word: what it is in the units it is taken in, the staged value, the slider, the
 * compiled range, the typed entry for an exact number, and the flash value with its unsaved mark.
 *
 * Slider AND entry for 3.3b's reason, which holds harder here: the useful values are about 100 for
 * `speed_kp` and 80 for `speed_max` against a 2,000-count track, which no thumb places.
 */
@Composable
private fun SpeedRow(state: TuneState, index: Int, actions: TuneActions) {
    val key = state.speedKeys[index]
    val gains = state.gains
    val staged = gains.staged[key]
    val max = gains.max(key)
    Panel(modifier = Modifier.testTag(tuneSpeedRowTag(index))) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                stringResource(SPEED_LABELS[index]),
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
        SpeedSlider(state, key, index, staged, max, actions)
        Caption(R.string.tune_speed_range, listOf(max.toString()))
        Caption(SPEED_NOTES[index])
        Row {
            Text(
                stringResource(
                    R.string.tune_flash,
                    gains.flash[key]?.toString() ?: stringResource(R.string.tune_unread),
                ),
                color = TextSecondary,
            )
            if (gains.unsaved(key)) Text(" " + stringResource(R.string.tune_unsaved_mark), color = AccentYellow)
        }
        SpeedEntry(key, index, staged, max, state.tunable && staged != null, actions)
    }
}

/**
 * The word's slider, over the COMPILED range (`control::config::outer::SPEED_GAIN_MAX`, mirrored as
 * [Gains.SPEED_MAXIMA]) rather than over a maximum read from the board: there is none to read, and
 * `CONTROL_GAIN_MAX`, which the gain sliders use, bounds the two PID profiles only.
 *
 * The thumb follows the finger during a drag and the staged value otherwise, which is the gain
 * slider's rule and the reason is the same: the staged value comes back a cadence later.
 */
@Composable
private fun SpeedSlider(
    state: TuneState,
    key: Key,
    index: Int,
    staged: Int?,
    max: Int,
    actions: TuneActions,
) {
    var dragging by remember { mutableStateOf(false) }
    var position by remember { mutableFloatStateOf(0f) }
    val shown = if (dragging) position else (staged ?: Gains.MIN).toFloat()
    Slider(
        value = shown.coerceIn(Gains.MIN.toFloat(), max.toFloat()),
        valueRange = Gains.MIN.toFloat()..max.toFloat(),
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
            .testTag(tuneSpeedSliderTag(index)),
    )
}

/**
 * The exact number, typed: one `TUNE_WRITE` and its re-read, the same thing a released slider
 * sends.
 *
 * An out-of-range number is impossible rather than refused: Set is off until the text is a whole
 * number inside the compiled range, so the board is never asked for a value its seam would refuse,
 * and the reason is on screen while it is off.
 */
@Composable
private fun SpeedEntry(key: Key, index: Int, staged: Int?, max: Int, enabled: Boolean, actions: TuneActions) {
    var text by remember(staged) { mutableStateOf(staged?.toString().orEmpty()) }
    val parsed = text.toIntOrNull()?.takeIf { it in Gains.MIN..max }
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
                .testTag(tuneSpeedEntryTag(index)),
        )
        Spacer(modifier = Modifier.width(8.dp))
        Button(
            onClick = { parsed?.let { actions.slideEnd(key, it) } },
            enabled = enabled && parsed != null,
            modifier = Modifier.testTag(tuneSpeedSetTag(index)),
        ) { Text(stringResource(R.string.setup_set)) }
    }
    if (text.isNotEmpty() && parsed == null) {
        Text(
            stringResource(R.string.tune_speed_entry_invalid, max.toString()),
            style = MaterialTheme.typography.bodySmall,
            color = AccentRed,
        )
    }
}
