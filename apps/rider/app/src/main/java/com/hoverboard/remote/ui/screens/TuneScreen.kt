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

/** Test tag on the Profile B chip. */
const val TUNE_PROFILE_B_TAG = "tune_profile_b"

/** Test tag on the slave target chip. */
const val TUNE_SLAVE_TAG = "tune_target_slave"

/** Test tag on gain [index]'s row. */
fun tuneRowTag(index: Int): String = "tune_row_$index"

/** Test tag on gain [index]'s up (or down) stepper. */
fun tuneStepTag(index: Int, up: Boolean): String = "tune_step_${index}_${if (up) "up" else "down"}"

/**
 * The Tune screen (`specs/rider-ui.md` section 3.3): the balance gains of one named board, as three
 * values kept apart. STAGED is what the steppers write (RAM); FLASH is what a reboot restores; and
 * ENGAGED, what the loop is using now, is not on the wire, so the screen never shows a number for it:
 * it says the loop ramps to staged and marks a gain converging for the ramp's bound after a change.
 *
 * Steppers work armed or disarmed; Save is disabled armed with the label saying why. A board that is
 * not reachable shows its last-read values greyed, marked stale, with every write off.
 */
@Composable
fun TuneScreen(
    state: TuneState,
    armed: Boolean,
    telemetry: TelemetryUi?,
    actions: TuneActions,
    modifier: Modifier = Modifier,
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
        Targets(state, actions)
        Text(riderLine(state, telemetry), style = MaterialTheme.typography.bodyMedium, color = TextSecondary)
        state.notice?.let { Notice(it, state, actions::dismissNotice) }
        if (stale) Stale()
        Profiles(state, actions)
        Column(
            modifier = Modifier.alpha(if (stale) STALE_ALPHA else 1f),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            for (i in 0 until Gains.PER_PROFILE) GainRow(state, i, actions)
        }
        Buttons(state, armed, actions)
        Legend(state)
    }
}

@Composable
private fun Targets(state: TuneState, actions: TuneActions) {
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        FilterChip(
            selected = state.target == Node.MASTER,
            onClick = { actions.selectTarget(Node.MASTER) },
            label = {
                Text(
                    state.master?.let { stringResource(R.string.tune_target_master, boardHex(it)) }
                        ?: stringResource(R.string.tune_target_master_none),
                )
            },
        )
        FilterChip(
            selected = state.target == Node.SLAVE,
            onClick = { actions.selectTarget(Node.SLAVE) },
            enabled = state.slave != null,
            label = {
                Text(
                    state.slave?.let { stringResource(R.string.tune_target_slave, boardHex(it)) }
                        ?: stringResource(R.string.tune_target_slave_none),
                )
            },
            modifier = Modifier.testTag(TUNE_SLAVE_TAG),
        )
    }
}

@Composable
private fun Profiles(state: TuneState, actions: TuneActions) {
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        FilterChip(
            selected = state.shownProfile == Gains.CONTROL_GAIN_A,
            onClick = { actions.selectProfile(Gains.CONTROL_GAIN_A) },
            label = { Text(stringResource(R.string.tune_profile_a)) },
        )
        if (!state.profileBHidden) {
            FilterChip(
                selected = state.shownProfile == Gains.CONTROL_GAIN_B,
                onClick = { actions.selectProfile(Gains.CONTROL_GAIN_B) },
                label = { Text(stringResource(R.string.tune_profile_b)) },
                modifier = Modifier.testTag(TUNE_PROFILE_B_TAG),
            )
        }
    }
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
private fun Notice(notice: TuneNotice, state: TuneState, onDismiss: () -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(AccentYellow.copy(alpha = 0.15f), RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(tuneNoticeText(notice, state), modifier = Modifier.weight(1f), color = TextPrimary)
        TextButton(onClick = onDismiss) { Text(stringResource(R.string.tune_dismiss)) }
    }
}

private const val STALE_ALPHA = 0.5f
