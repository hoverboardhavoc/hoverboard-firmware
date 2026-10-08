package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneModel
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** One gain: staged (with the converging mark), the steppers, and flash (with the unsaved mark). */
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
            Stepper(index, up = false, enabled = state.writable && staged != null, actions)
            Stepper(index, up = true, enabled = state.writable && staged != null, actions)
        }
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

/** One tap of gain [index]'s per-tap step, [up] or down. */
@Composable
private fun Stepper(index: Int, up: Boolean, enabled: Boolean, actions: TuneActions) {
    val step = TuneModel.TAP_STEP[index]
    OutlinedButton(
        onClick = { actions.step(index, up) },
        enabled = enabled,
        modifier = Modifier.testTag(tuneStepTag(index, up)),
    ) { Text(if (up) "+$step" else "-$step") }
}
