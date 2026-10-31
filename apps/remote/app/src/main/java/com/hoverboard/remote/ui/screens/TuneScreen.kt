package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.pluralStringResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Gains
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneNotice
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.Caution
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary

/** Test tag on the stale statement (the target is not reachable, or its last request went unanswered). */
const val TUNE_STALE_TAG = "tune_stale"

/** Test tag on the Save button. */
const val TUNE_SAVE_TAG = "tune_save"

/** Test tag on the Revert button. */
const val TUNE_REVERT_TAG = "tune_revert"

/** Test tag on the retry offered when the request slot stayed taken. */
const val TUNE_RETRY_TAG = "tune_retry"

/** Test-tag prefix of the Tune selector's chips ([nodeChipTag]). */
const val TUNE_NODE_TAG = "tune_target"

/** Test tag on the drive bounds' heading. */
const val TUNE_BOUNDS_TAG = "tune_bounds"

/** Test tag on the ride controls' heading. */
const val TUNE_RIDE_TAG = "tune_ride"

/** Test tag on gain [index]'s row. */
fun tuneRowTag(index: Int): String = "tune_row_$index"

/** Test tag on gain [index]'s slider. */
fun tuneSliderTag(index: Int): String = "tune_slider_$index"

/**
 * The Tune screen (`specs/remote-ui.md` section 3.3, `specs/state-feedback.md` section 10): the four
 * state-feedback gains of one named board, as three values kept apart. STAGED is what the sliders
 * write (RAM) and what each row shows is the value the board last REPORTED for it, read back after
 * every write and polled once a second while the screen is shown; FLASH is what a reboot restores;
 * and ENGAGED, what the law is using now, is not on the wire, so the screen never shows a number for
 * it: it says the law ramps to staged and marks a gain converging for the ramp's bound after a
 * change.
 *
 * One gain set: there are no profile chips and no rider-set line.
 *
 * Under the gains, on the same lane and inside the same Save, is the reference row ([Reference]):
 * `speed_max` and `speed_slew`, which shape what the stick asks for.
 *
 * Below them are the DRIVE BOUNDS (section 3.3b: the turn limit, its rate, and the position clamp)
 * and then the ride controls. The bounds are stored values rather than staged ones, so they are
 * written when a slider is released and the board takes them up at its next arm, and they are
 * disabled while armed; the ride controls are the Ride screen's own arm toggle and pad, here so that
 * a machine can be ridden, stopped and adjusted without leaving the screen.
 *
 * Live-lane sliders work armed or disarmed; Save is disabled armed with the label saying why. A
 * board that is not reachable shows its last-read values greyed, marked stale, with every write
 * off.
 */
@Composable
fun TuneScreen(
    state: TuneState,
    armed: Boolean,
    actions: TuneActions,
    modifier: Modifier = Modifier,
    ride: @Composable () -> Unit = {},
) {
    DisposableEffect(actions) {
        actions.onShown()
        onDispose { actions.onHidden() }
    }
    val stale = state.address == null || state.gains.stale
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(stringResource(R.string.tune_title), style = MaterialTheme.typography.headlineSmall, color = Fg)
        NodeChips(state.motors, state.target, actions::selectTarget, TUNE_NODE_TAG)
        state.notice?.let { Notice(it, state, actions) }
        if (stale) Stale()
        Column(
            modifier = Modifier.alpha(if (stale) STALE_ALPHA else 1f),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            for (i in 0 until Gains.COUNT) GainRow(state, i, actions)
            // The reference row is on the same RAM lane as the gains, so it sits inside the same
            // greying and above the Save button that persists both of them.
            Reference(state, armed, actions)
        }
        Buttons(state, armed, actions)
        Legend()
        // Below the gains and their legend, in the order a tuning session uses them: the bounds,
        // then the stick that says whether the new ones are right.
        Bounds(state, armed, actions)
        Ride(ride)
    }
}

@Composable
private fun Buttons(state: TuneState, armed: Boolean, actions: TuneActions) {
    val unsaved = state.unsavedKeys
    if (unsaved.isNotEmpty()) {
        Text(
            pluralStringResource(R.plurals.tune_unsaved_count, unsaved.size, unsaved.size),
            style = MaterialTheme.typography.titleMedium,
            color = Primary,
        )
    }
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
        OutlinedButton(
            onClick = actions::revert,
            enabled = state.writable && unsaved.isNotEmpty(),
            modifier = Modifier.testTag(TUNE_REVERT_TAG),
        ) { Text(stringResource(R.string.tune_revert)) }
        Button(
            onClick = actions::save,
            enabled = state.writable && unsaved.isNotEmpty() && !armed,
            modifier = Modifier.testTag(TUNE_SAVE_TAG),
        ) { Text(stringResource(if (armed) R.string.tune_save_armed else R.string.tune_save)) }
    }
    if (armed) Caption(R.string.tune_save_armed_note)
    when {
        state.reading -> Text(stringResource(R.string.tune_reading), color = FgMuted)
        state.writing -> Text(stringResource(R.string.tune_writing), color = FgMuted)
        state.address != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.tune_refresh))
        }
    }
}

/**
 * The drive bounds (`specs/remote-ui.md` 3.3b), under their own heading because they are not what
 * the gains above are: a gain slider writes RAM and the law ramps to it while the machine is
 * running, and these write flash and reach the law at the next arm.
 *
 * The heading's two lines say exactly that, and while the board is armed a third says why the
 * controls are off and what to do about it. The per-row line that places each value in time is
 * generated from the firmware's arm row and not written here ([TakeEffect]).
 */
@Composable
private fun Bounds(state: TuneState, armed: Boolean, actions: TuneActions) {
    Text(
        stringResource(R.string.tune_bounds_title),
        style = MaterialTheme.typography.titleMedium,
        color = Fg,
        modifier = Modifier.testTag(TUNE_BOUNDS_TAG),
    )
    Caption(R.string.tune_bounds_body)
    if (armed) Text(stringResource(R.string.tune_bounds_armed), color = Primary)
    for (field in SetupFields.DRIVE_BOUNDS) BoundRow(state, field, armed, actions)
}

/**
 * The ride controls, given by the caller ([TuneScreen]'s `ride` slot): the arm toggle and the pad,
 * exactly as the Ride screen has them. Empty in a test that does not pass them.
 */
@Composable
private fun Ride(ride: @Composable () -> Unit) {
    Text(
        stringResource(R.string.tune_ride_title),
        style = MaterialTheme.typography.titleMedium,
        color = Fg,
        modifier = Modifier.testTag(TUNE_RIDE_TAG),
    )
    Caption(R.string.tune_ride_body)
    ride()
}

@Composable
private fun Legend() {
    Panel {
        Caption(R.string.tune_legend_staged)
        Caption(R.string.tune_legend_engaged)
        Caption(R.string.tune_legend_flash)
        Caption(R.string.tune_legend_converging)
        Caption(R.string.tune_legend_unsaved)
    }
}

@Composable
private fun Stale() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, Caution, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(TUNE_STALE_TAG),
    ) {
        CautionText(stringResource(R.string.tune_stale_title), style = MaterialTheme.typography.titleMedium)
        Text(stringResource(R.string.tune_stale_body), style = MaterialTheme.typography.bodyMedium, color = Fg)
    }
}

@Composable
private fun Notice(notice: TuneNotice, state: TuneState, actions: TuneActions) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(BgElevated, RoundedCornerShape(8.dp))
            .border(1.dp, Border, RoundedCornerShape(8.dp))

            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(tuneNoticeText(notice), modifier = Modifier.weight(1f), color = Fg)
        // The slot was taken, the board was not asked: a read pass is the retry (a tap is re-tapped).
        if (notice == TuneNotice.SlotBusy && state.address != null) {
            TextButton(onClick = actions::refresh, modifier = Modifier.testTag(TUNE_RETRY_TAG)) {
                Text(stringResource(R.string.tune_retry))
            }
        }
        TextButton(onClick = actions::dismissNotice) { Text(stringResource(R.string.tune_dismiss)) }
    }
}

private const val STALE_ALPHA = 0.5f
