package com.hoverboard.remote.ui.components

import androidx.annotation.StringRes
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.machineSupplyAmps
import com.hoverboard.remote.ui.theme.TextSecondary

/**
 * The machine's current, beside the battery it is drawn from (`specs/rider-ui.md` 3.6).
 *
 * **Two figures per board, never one.** The board measures the PHASE current, which is what the
 * motor feels and what the soft limit chops on; a bench supply reads the DC LINK, which is what
 * the pack pays. In six-step they differ by the duty, so one is not the other, and a supply
 * reading taken for a phase current cost a morning on 2026-10-09. Both are worded for which one
 * they are.
 *
 * **One row per board**, because the two wheels of a rigid rover can pull very differently and
 * that is itself worth seeing, and because the scale that makes a count into an amp belongs to
 * each board's own sense chain. One row per motor controller the session found, labelled by its
 * node label (`specs/rider-ui.md` section 2), and the machine's sum under them when there are two.
 *
 * **No heading above it**, which is a decision about this screen rather than about this block:
 * the Ride screen is dense and the throttle pad's height is its resolution, so every row here is
 * paid for below. Each row names its board and its two currents in words, which leaves a heading
 * nothing to add.
 *
 * **The note is not decoration.** Every amp here rests on `motor.current_cal`, which has never
 * been checked against a reference instrument (`specs/todo.md` part 1), so the figure is shown
 * and its footing is stated once, quietly. That is also what the display is FOR: a derived link
 * current beside the supply's own display is what confirms the constant, so it cannot wait for
 * the constant to be confirmed.
 */
@Composable
internal fun CurrentSection(rows: List<Pair<String, CurrentReading>>) {
    rows.forEachIndexed { i, (label, current) ->
        if (i > 0) Spacer(modifier = Modifier.height(4.dp))
        BoardCurrent(label, current)
    }
    if (rows.size > 1) {
        Spacer(modifier = Modifier.height(4.dp))
        MachineCurrent(machineSupplyAmps(rows.map { it.second }))
    }
    Spacer(modifier = Modifier.height(8.dp))
    Text(
        text = stringResource(R.string.telemetry_current_note),
        style = MaterialTheme.typography.bodySmall,
        color = TextSecondary,
    )
}

/**
 * One board's row: the two amp figures, or the one reason there is no figure.
 *
 * The four reasons are kept apart and worded as what they are, because an owner does something
 * different about each: a board that has gone quiet, a board whose motors are down, a board
 * coasting, and a board whose scale this session has not read. None of them is a number, which is
 * the point: the board stops refreshing its current window the moment its motor path goes down,
 * so a figure drawn there would be whatever the last live window left behind.
 */
@Composable
private fun BoardCurrent(tag: String, reading: CurrentReading) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            text = tag,
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
            modifier = Modifier.width(BOARD_TAG_WIDTH),
        )
        // Exhaustive on the sealed type rather than an else: a fifth way to have no figure is a
        // fifth thing to word, and it should not reach the screen as whatever this branch said
        // last.
        when (reading) {
            is CurrentReading.Amps -> Text(
                text = stringResource(R.string.telemetry_current_value, reading.motor, reading.supply),
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface,
            )
            CurrentReading.Silent -> NoFigure(R.string.telemetry_current_silent)
            CurrentReading.MotorsOff -> NoFigure(R.string.telemetry_current_motors_off)
            CurrentReading.Coasting -> NoFigure(R.string.telemetry_current_coasting)
            CurrentReading.Unscaled -> NoFigure(R.string.telemetry_current_unscaled)
        }
    }
}

/**
 * The pair's supply current: the two boards' link figures added.
 *
 * It is the row a bench supply's own display is compared against, because the supply feeds both
 * boards and one board's figure is only a share of what it shows. Drawn for a pair alone: on a
 * one-board machine that board's row IS the machine, and a second row repeating it would be a
 * total that is not a sum of anything.
 *
 * Null is a board that cannot contribute ([machineSupplyAmps]), and it says so rather than
 * falling back to the one figure it does have.
 */
@Composable
private fun MachineCurrent(amps: Float?) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            text = stringResource(R.string.telemetry_current_machine),
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
            modifier = Modifier.width(BOARD_TAG_WIDTH),
        )
        if (amps == null) {
            NoFigure(R.string.telemetry_current_machine_incomplete)
        } else {
            Text(
                text = stringResource(R.string.telemetry_current_machine_value, amps),
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface,
            )
        }
    }
}

/** The words that stand where the figures would be. */
@Composable
private fun NoFigure(@StringRes text: Int) {
    Text(
        text = stringResource(text),
        style = MaterialTheme.typography.bodyMedium,
        color = TextSecondary,
    )
}

/** Room for the longest of the three row tags, so every row's figure starts in the same place. */
private val BOARD_TAG_WIDTH = 76.dp
