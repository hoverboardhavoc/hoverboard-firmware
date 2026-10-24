package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.border
import androidx.compose.foundation.background
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
import androidx.compose.material3.FilterChip
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
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the stale statement (the target is not reachable, or its last request went unanswered). */
const val TUNE_STALE_TAG = "tune_stale"

/** Test tag on the Save button. */
const val TUNE_SAVE_TAG = "tune_save"

/** Test tag on the Revert button. */
const val TUNE_REVERT_TAG = "tune_revert"

/** Test tag on the retry offered when the request slot stayed taken. */
const val TUNE_RETRY_TAG = "tune_retry"

/** Test tag on the chip for the rider's gain set. */
const val TUNE_PROFILE_A_TAG = "tune_profile_a"

/** Test tag on the chip for the no-rider gain set. */
const val TUNE_PROFILE_B_TAG = "tune_profile_b"

/** Test tag on the slave target chip. */
const val TUNE_SLAVE_TAG = "tune_target_slave"

/** Test tag on the drive bounds' heading. */
const val TUNE_BOUNDS_TAG = "tune_bounds"

/** Test tag on the ride controls' heading. */
const val TUNE_RIDE_TAG = "tune_ride"

/** Test tag on gain [index]'s row. */
fun tuneRowTag(index: Int): String = "tune_row_$index"

/** Test tag on gain [index]'s slider. */
fun tuneSliderTag(index: Int): String = "tune_slider_$index"

/**
 * The Tune screen (`specs/rider-ui.md` section 3.3): the balance gains of one named board, as three
 * values kept apart. STAGED is what the sliders write (RAM); FLASH is what a reboot restores; and
 * ENGAGED, what the loop is using now, is not on the wire, so the screen never shows a number for it:
 * it says the loop ramps to staged and marks a gain converging for the ramp's bound after a change.
 *
 * The instrument is a slider per gain rather than a pair of steppers (section 3.3a): a balance loop
 * is tuned by sweeping and feeling the response, not by discrete taps. Each gain already applied in
 * real time before the change and still does; what the drag changes is how a value is chosen, not
 * when it is applied.
 *
 * Gain sliders work armed or disarmed; Save is disabled armed with the label saying why. A board
 * that is not reachable shows its last-read values greyed, marked stale, with every write off.
 *
 * The gains come in two sets, and the board picks between them by itself, so the chips above them
 * choose what is EDITED and nothing else ([Profiles]).
 *
 * Under the gains, on the same lane and inside the same Save, is the outer speed loop's row
 * ([SpeedLoop], `specs/rider-ui.md` 3.3c): three words that are not a third gain set, which is why
 * they are a section rather than a chip.
 *
 * Below the gains are the four DRIVE BOUNDS (section 3.3b) and then the ride controls. The bounds
 * are stored values rather than staged ones, so they are written when a slider is released and the
 * board takes them up at its next arm, and they are disabled while armed; the ride controls are
 * the Ride screen's own arm toggle and pad, here so that a machine can be ridden, stopped and
 * adjusted without leaving the screen.
 */
@Composable
fun TuneScreen(
    state: TuneState,
    armed: Boolean,
    telemetry: TelemetryUi?,
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
        Text(stringResource(R.string.tune_title), style = MaterialTheme.typography.headlineSmall, color = TextPrimary)
        BoardTargets(state.target, state.master, state.slave, actions::selectTarget, TUNE_SLAVE_TAG)
        state.notice?.let { Notice(it, state, actions) }
        if (stale) Stale()
        Profiles(state, telemetry, actions)
        Column(
            modifier = Modifier.alpha(if (stale) STALE_ALPHA else 1f),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            for (i in 0 until Gains.PER_PROFILE) GainRow(state, i, actions)
            // The speed loop is on the same RAM lane as the gains, so it sits inside the same
            // greying and above the Save button that persists both of them.
            SpeedLoop(state, armed, actions)
        }
        Buttons(state, armed, actions)
        Legend(state)
        // Below the gains and their legend, in the order a tuning session uses them: the bounds,
        // then the stick that says whether the new ones are right.
        Bounds(state, armed, actions)
        Ride(ride)
    }
}

/**
 * The two gain sets, named by what selects them (`specs/rider-ui.md` section 4, "What the screen
 * calls the two sets"): the board is on the rider's set while somebody is on the pads and on the
 * other one the rest of the time, and it switches by itself.
 *
 * So the line under the chips says that, and says which set that leaves the board on right now,
 * as far as the app can tell: a chip is an editing choice and nothing more. A board that does not
 * require a rider can reach only the rider's set, and then there is no choice to offer: the chips
 * go and the line says the one set below is what always runs.
 *
 * The no-rider set carries a line of its own, because what it does is not what a reader would
 * assume from its name.
 */
@Composable
private fun Profiles(state: TuneState, telemetry: TelemetryUi?, actions: TuneActions) {
    if (!state.profileBHidden) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            FilterChip(
                selected = state.shownProfile == Gains.CONTROL_GAIN_A,
                onClick = { actions.selectProfile(Gains.CONTROL_GAIN_A) },
                label = { Text(stringResource(R.string.tune_profile_a)) },
                modifier = Modifier.testTag(TUNE_PROFILE_A_TAG),
            )
            FilterChip(
                selected = state.shownProfile == Gains.CONTROL_GAIN_B,
                onClick = { actions.selectProfile(Gains.CONTROL_GAIN_B) },
                label = { Text(stringResource(R.string.tune_profile_b)) },
                modifier = Modifier.testTag(TUNE_PROFILE_B_TAG),
            )
        }
    }
    Text(profileLine(state, telemetry), style = MaterialTheme.typography.bodyMedium, color = TextSecondary)
    if (state.shownProfile == Gains.CONTROL_GAIN_B) Caption(R.string.tune_profile_b_note)
    if (state.target == Node.SLAVE) Caption(R.string.tune_profile_slave_note)
}

@Composable
private fun Buttons(state: TuneState, armed: Boolean, actions: TuneActions) {
    val unsaved = state.unsavedKeys
    if (unsaved.isNotEmpty()) {
        Text(
            pluralStringResource(R.plurals.tune_unsaved_count, unsaved.size, unsaved.size),
            style = MaterialTheme.typography.titleMedium,
            color = AccentYellow,
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
        state.reading -> Text(stringResource(R.string.tune_reading), color = TextSecondary)
        state.writing -> Text(stringResource(R.string.tune_writing), color = TextSecondary)
        state.address != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.tune_refresh))
        }
    }
}

/**
 * The four drive bounds (`specs/rider-ui.md` 3.3b), under their own heading because they are not
 * what the gains above are: a gain slider writes RAM and the loop ramps to it while the machine is
 * running, and these write flash and reach the loop at the next arm.
 *
 * The heading's two lines say exactly that, and while the board is armed a third says why the four
 * controls are off and what to do about it. The per-row line that places each value in time is
 * generated from the firmware's arm row and not written here ([TakeEffect]).
 */
@Composable
private fun Bounds(state: TuneState, armed: Boolean, actions: TuneActions) {
    Text(
        stringResource(R.string.tune_bounds_title),
        style = MaterialTheme.typography.titleMedium,
        color = TextPrimary,
        modifier = Modifier.testTag(TUNE_BOUNDS_TAG),
    )
    Caption(R.string.tune_bounds_body)
    if (armed) Text(stringResource(R.string.tune_bounds_armed), color = AccentRed)
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
        color = TextPrimary,
        modifier = Modifier.testTag(TUNE_RIDE_TAG),
    )
    Caption(R.string.tune_ride_body)
    ride()
}

@Composable
private fun Legend(state: TuneState) {
    Panel {
        Caption(R.string.tune_legend_staged)
        Caption(R.string.tune_legend_engaged)
        Caption(R.string.tune_legend_flash)
        Caption(R.string.tune_legend_converging)
        Caption(R.string.tune_legend_unsaved)
        state.gains.staged[state.keys[Gains.PR]]?.let { pr ->
            Text(
                stringResource(R.string.tune_legend_engage, seconds(Gains.Ramp.boundMs(Gains.PR, pr, null))),
                style = MaterialTheme.typography.bodySmall,
                color = TextSecondary,
            )
        }
    }
}

@Composable
private fun Stale() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(TUNE_STALE_TAG),
    ) {
        Text(stringResource(R.string.tune_stale_title), style = MaterialTheme.typography.titleMedium, color = AccentRed)
        Text(stringResource(R.string.tune_stale_body), style = MaterialTheme.typography.bodyMedium, color = TextPrimary)
    }
}

@Composable
private fun Notice(notice: TuneNotice, state: TuneState, actions: TuneActions) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(AccentYellow.copy(alpha = 0.15f), RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(tuneNoticeText(notice, state), modifier = Modifier.weight(1f), color = TextPrimary)
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
