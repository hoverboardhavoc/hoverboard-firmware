package com.hoverboardhavoc.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.TuneActions
import com.hoverboardhavoc.remote.TuneState
import com.hoverboardhavoc.remote.ui.theme.Fg
import com.hoverboardhavoc.remote.ui.theme.Primary

/** Test tag on the turn section's heading. */
const val TUNE_TURN_TAG = "tune_turn"

/** Test tag on turn row [index], in [TuneState.turnKeys] order. */
fun tuneTurnRowTag(index: Int): String = "tune_turn_row_$index"

/** Test tag on turn row [index]'s slider. */
fun tuneTurnSliderTag(index: Int): String = "tune_turn_slider_$index"

/** Test tag on turn row [index]'s typed entry. */
fun tuneTurnEntryTag(index: Int): String = "tune_turn_entry_$index"

/** Test tag on turn row [index]'s Set button. */
fun tuneTurnSetTag(index: Int): String = "tune_turn_set_$index"

/** Test tag on turn row [index]'s converging line. */
fun tuneTurnConvergingTag(index: Int): String = "tune_turn_converging_$index"

/** What the screen calls each turn word, in [TuneState.turnKeys] order. */
private val TURN_LABELS = listOf(
    R.string.tune_turn_gh,
    R.string.tune_turn_ghv,
    R.string.tune_turn_gff,
    R.string.tune_turn_rate_max,
    R.string.tune_turn_rate_slew,
)

/** What each turn word does, in the wording of `specs/heading-hold.md` section 10. */
private val TURN_NOTES = listOf(
    R.string.tune_turn_gh_note,
    R.string.tune_turn_ghv_note,
    R.string.tune_turn_gff_note,
    R.string.tune_turn_rate_max_note,
    R.string.tune_turn_rate_slew_note,
)

/**
 * The heading hold's five live words (`specs/heading-hold.md` section 10): `gh`, `ghv` and `gff` of
 * `CONTROL_TURN_GAINS`, then `rate_max` and `rate_slew` of `CONTROL_TURN_REFERENCE`, under the
 * reference section and on the same RAM lane, inside the same Save.
 *
 * Every range is compiled into the firmware (`control::config::turn`), so each row is the reference
 * row's kind: slider, typed entry, the compiled range, the flash value and the unsaved mark. The
 * three gains add the converging mark, because the firmware ramps them (`Gains.Ramp.TURN_CAPS`);
 * the two rate words are not ramped and carry none.
 */
@Composable
internal fun Turn(state: TuneState, armed: Boolean, actions: TuneActions) {
    Text(
        stringResource(R.string.tune_turn_title),
        style = MaterialTheme.typography.titleMedium,
        color = Fg,
        modifier = Modifier.testTag(TUNE_TURN_TAG),
    )
    Caption(R.string.tune_turn_body)
    if (armed) Text(stringResource(R.string.tune_turn_armed), color = Primary)
    for (i in state.turnKeys.indices) TurnRow(state, i, actions)
}

/**
 * One turn word: its name and unit, the staged value as last read with the converging mark while
 * the ramp's bound runs, the slider over the compiled range, the typed entry, and the flash value
 * with its unsaved mark.
 */
@Composable
private fun TurnRow(state: TuneState, index: Int, actions: TuneActions) {
    val key = state.turnKeys[index]
    val gains = state.gains
    val staged = gains.staged[key]
    val converging = gains.converging[key]
    val range = gains.min(key)..gains.max(key)
    Panel(modifier = Modifier.testTag(tuneTurnRowTag(index))) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                stringResource(TURN_LABELS[index]),
                style = MaterialTheme.typography.titleSmall,
                color = Fg,
                modifier = Modifier.weight(1f),
            )
            Text(
                staged?.toString() ?: stringResource(R.string.tune_unread),
                style = MaterialTheme.typography.titleLarge,
                color = Fg,
            )
            if (converging != null) {
                Text(
                    stringResource(R.string.tune_converging_mark),
                    style = MaterialTheme.typography.titleLarge,
                    color = Primary,
                )
            }
        }
        LiveSlider(state, key, tuneTurnSliderTag(index), staged, range, actions)
        Caption(R.string.tune_reference_range, listOf(range.first.toString(), range.last.toString()))
        Caption(TURN_NOTES[index])
        FlashLine(gains, key)
        if (converging != null) {
            Text(
                stringResource(R.string.tune_converging_row, seconds(converging)),
                style = MaterialTheme.typography.bodySmall,
                color = Primary,
                modifier = Modifier.testTag(tuneTurnConvergingTag(index)),
            )
        }
        LiveEntry(
            staged = staged,
            range = range,
            enabled = state.tunable && staged != null,
            entryTag = tuneTurnEntryTag(index),
            setTag = tuneTurnSetTag(index),
        ) { actions.slideEnd(key, it) }
    }
}
