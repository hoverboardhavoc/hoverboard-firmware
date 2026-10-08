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
import androidx.compose.runtime.setValue
import androidx.compose.runtime.getValue
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.pluralStringResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupNotice
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SetupGroup
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the armed lock statement. */
const val SETUP_LOCK_TAG = "setup_lock"

/** Test tag on the power-cycle instruction. */
const val SETUP_POWER_CYCLE_TAG = "setup_power_cycle"

/** Test tag on the Apply button. */
const val SETUP_APPLY_TAG = "setup_apply"

/** Test tag prefix on each field row: `setup_row_<field_id>_<index>`. */
fun setupRowTag(key: Key): String = "setup_row_${key.fieldId}_${key.index}"

/**
 * The Setup screen (`specs/rider-ui.md` section 3.4): a grouped store editor for the attached board,
 * minus the pin block (the layout editor, section 3.5).
 *
 * Edits go into a basket and Apply writes them; a verified write says "stored" and carries a
 * "staged, not applied" mark until the operator power-cycles, because no field applies live. While
 * [armed] the screen is visible and read-only and says why at the top.
 */
@Composable
fun SetupScreen(
    state: SetupState,
    armed: Boolean,
    telemetry: TelemetryUi?,
    actions: SetupActions,
    modifier: Modifier = Modifier,
) {
    DisposableEffect(actions) {
        actions.onShown()
        onDispose { actions.onHidden() }
    }
    val editable = !armed && !state.busy && state.board != null

    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(stringResource(R.string.setup_title), style = MaterialTheme.typography.headlineSmall, color = TextPrimary)
        Text(
            text = state.board?.let { stringResource(R.string.setup_target, boardHex(it)) }
                ?: stringResource(R.string.setup_no_board),
            style = MaterialTheme.typography.bodyMedium,
            color = TextSecondary,
        )
        if (armed) Lock()
        state.notice?.let { Notice(it, actions::dismissNotice) }
        Status(state, actions)
        Basket(state, editable, actions)
        PowerCycle(state, actions)

        for (group in SetupGroup.entries) Group(group, state, editable, telemetry, actions)
    }
}

/**
 * One group's rows. The IMU group leads with its flows (set level, orientation, the rotation check)
 * and keeps its raw editors behind ADVANCED (`specs/rider-ui.md` 3.4: "the primary UI is flows").
 */
@Composable
private fun Group(
    group: SetupGroup,
    state: SetupState,
    editable: Boolean,
    telemetry: TelemetryUi?,
    actions: SetupActions,
) {
    var advanced by rememberSaveable { mutableStateOf(false) }
    val rows = SetupFields.ALL.filter { it.group == group }
    Section(stringResource(group.title)) {
        rows.filterNot { it.advanced }.forEach { FieldRow(it, state, editable, actions) }
        if (group != SetupGroup.IMU) return@Section
        Level(state, editable, telemetry, actions)
        OrientationPanel(state, editable, actions)
        RotationCheckPanel(state, telemetry, actions)
        TextButton(onClick = { advanced = !advanced }) {
            Text(stringResource(if (advanced) R.string.setup_advanced_hide else R.string.setup_advanced_show))
        }
        if (advanced) rows.filter { it.advanced }.forEach { FieldRow(it, state, editable, actions) }
    }
}

@Composable
private fun Lock() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(SETUP_LOCK_TAG),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.setup_locked_title), style = type.titleMedium, color = AccentRed)
        Text(stringResource(R.string.setup_locked_body), style = type.bodyMedium, color = TextPrimary)
    }
}

@Composable
private fun Notice(notice: SetupNotice, onDismiss: () -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(AccentYellow.copy(alpha = 0.15f), RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(noticeText(notice), modifier = Modifier.weight(1f), color = TextPrimary)
        TextButton(onClick = onDismiss) { Text(stringResource(R.string.setup_dismiss)) }
    }
}

@Composable
private fun Status(state: SetupState, actions: SetupActions) {
    when {
        state.reading -> Text(stringResource(R.string.setup_reading), color = TextSecondary)
        state.applying -> Text(stringResource(R.string.setup_writing), color = TextSecondary)
        state.board != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.setup_refresh))
        }
    }
}

@Composable
private fun Basket(state: SetupState, editable: Boolean, actions: SetupActions) {
    if (state.pending.isEmpty()) return
    Panel {
        Text(
            pluralStringResource(R.plurals.setup_pending_count, state.pending.size, state.pending.size),
            style = MaterialTheme.typography.titleMedium,
            color = TextPrimary,
        )
        for ((key, value) in state.pending) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(changeLine(key, value), modifier = Modifier.weight(1f), color = TextPrimary)
                TextButton(onClick = { actions.discard(key) }) { Text(stringResource(R.string.setup_discard)) }
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                onClick = actions::apply,
                enabled = editable,
                modifier = Modifier.testTag(SETUP_APPLY_TAG),
            ) { Text(stringResource(R.string.setup_apply)) }
            OutlinedButton(onClick = actions::discardAll) { Text(stringResource(R.string.setup_discard_all)) }
        }
    }
}

@Composable
private fun PowerCycle(state: SetupState, actions: SetupActions) {
    if (!state.awaitingPowerCycle) return
    Panel(modifier = Modifier.testTag(SETUP_POWER_CYCLE_TAG)) {
        Text(
            pluralStringResource(R.plurals.setup_power_cycle, state.staged.size, state.staged.size),
            style = MaterialTheme.typography.titleMedium,
            color = TextPrimary,
        )
        for ((key, value) in state.staged) Text(changeLine(key, value), color = TextPrimary)
        Button(onClick = actions::confirmPowerCycled, enabled = state.linkDroppedSinceApply && !state.busy) {
            Text(stringResource(R.string.setup_power_cycled))
        }
        if (!state.linkDroppedSinceApply) {
            Caption(R.string.setup_power_cycle_wait)
        }
    }
}
