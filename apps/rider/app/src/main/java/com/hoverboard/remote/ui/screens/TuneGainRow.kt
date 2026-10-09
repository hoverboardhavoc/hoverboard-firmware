package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
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
import com.hoverboard.protocol.store.Gains
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import kotlin.math.roundToInt

/** One gain: staged (with the converging mark), the slider, and flash (with the unsaved mark). */
@Composable
internal fun GainRow(state: TuneState, index: Int, actions: TuneActions) {
    val key = state.keys[index]
    val gains = state.gains
    val staged = gains.staged[key]
    val converging = gains.converging[key]
    Panel(modifier = Modifier.testTag(tuneRowTag(index))) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(GAIN_NAMES[index], style = MaterialTheme.typography.titleMedium, color = TextPrimary)
            Column(modifier = Modifier.weight(1f)) {
                Text(
                    stringResource(R.string.tune_staged),
                    style = MaterialTheme.typography.labelSmall,
                    color = TextSecondary,
                )
                Row {
                    Text(
                        staged?.toString() ?: stringResource(R.string.tune_unread),
                        style = MaterialTheme.typography.titleLarge,
                        color = TextPrimary,
                    )
                    if (converging != null) {
                        Text(
                            " " + stringResource(R.string.tune_converging_mark),
                            style = MaterialTheme.typography.titleLarge,
                            color = AccentYellow,
                        )
                    }
                }
            }
        }
        GainSlider(state, index, staged, actions)
        Text(
            stringResource(R.string.tune_range, gains.max(index).toString()),
            style = MaterialTheme.typography.bodySmall,
            color = TextSecondary,
        )
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
        if (converging != null) {
            Text(
                stringResource(R.string.tune_converging_row, seconds(converging)),
                style = MaterialTheme.typography.bodySmall,
                color = AccentYellow,
            )
        }
    }
}

/**
 * Gain [index]'s slider, over the range the BOARD accepts (`specs/rider-ui.md` section 3.3a: the
 * extent is `CONTROL_GAIN_MAX` as read from this board, per-board data rather than a constant).
 *
 * The thumb follows the finger, not the board. Each drag sample goes to [TuneActions.slide], which
 * sends on a cadence, so the staged value the board reports back arrives up to a cadence later and
 * following it would make the thumb stutter behind the drag. On release the exact position is sent
 * once through [TuneActions.slideEnd] and the thumb goes back to tracking staged, which is the
 * value the screen is accountable for.
 */
@Composable
private fun GainSlider(state: TuneState, index: Int, staged: Int?, actions: TuneActions) {
    val max = state.gains.max(index)
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
            actions.slide(index, it.roundToInt())
        },
        onValueChangeFinished = {
            actions.slideEnd(index, position.roundToInt())
            dragging = false
        },
        modifier = Modifier
            .fillMaxWidth()
            .testTag(tuneSliderTag(index)),
    )
}
