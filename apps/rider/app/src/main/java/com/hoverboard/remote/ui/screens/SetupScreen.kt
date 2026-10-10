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

/** Test tag on the warning that the stored sign map is one the board refuses. */
const val SETUP_FRAME_BAD_TAG = "setup_frame_bad"

/** Test tag on the line shown while the IMU frame's fields are being written. */
const val SETUP_FRAME_WRITING_TAG = "setup_frame_writing"

/** Test tag on a group's ADVANCED toggle. */
fun setupAdvancedTag(group: SetupGroup): String = "setup_advanced_${group.name}"

/** Test tag on one row's retry, offered after a write of that field failed. */
fun setupRetryTag(key: Key): String = "setup_retry_${key.fieldId}_${key.index}"

/** Test tag prefix on each field row: `setup_row_<field_id>_<index>`. */
fun setupRowTag(key: Key): String = "setup_row_${key.fieldId}_${key.index}"

/**
 * The Setup screen (`specs/rider-ui.md` sections 3.4 and 3.6a): a grouped store editor for the
 * attached board, minus the pin block (the layout editor, section 3.5).
 *
 * There is no Apply. Each field is written when it is edited and each row reports what the board
 * said about that field, including when the board will start using it. While [armed] the screen is
 * visible and read-only, and every row says so.
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
        state.notice?.let { Notice(it, actions) }
        Status(state, actions)
        if (state.storedFrameUnsafe) FrameBad()

        for (group in SetupGroup.entries) Group(group, state, armed, editable, telemetry, actions)
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
    armed: Boolean,
    editable: Boolean,
    telemetry: TelemetryUi?,
    actions: SetupActions,
) {
    var advanced by rememberSaveable { mutableStateOf(false) }
    val rows = SetupFields.ALL.filter { it.group == group }
    Section(stringResource(group.title)) {
        rows.filterNot { it.advanced }.forEach { FieldRow(it, state, armed, editable, actions) }
        if (group == SetupGroup.IMU) {
            Level(state, editable, telemetry, actions)
            OrientationPanel(state, editable, telemetry, actions)
            RotationCheckPanel(state, telemetry, actions)
        }
        val hidden = rows.filter { it.advanced }
        if (hidden.isEmpty()) return@Section
        TextButton(onClick = { advanced = !advanced }, modifier = Modifier.testTag(setupAdvancedTag(group))) {
            Text(stringResource(if (advanced) R.string.setup_advanced_hide else R.string.setup_advanced_show))
        }
        if (advanced) hidden.forEach { FieldRow(it, state, armed, editable, actions) }
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

/**
 * The screen's own notice: what no single row owns (a whole-frame refusal, a flow that cannot run,
 * a board swap). What came of a write belongs to its row, never here.
 */
@Composable
private fun Notice(notice: SetupNotice, actions: SetupActions) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(AccentYellow.copy(alpha = 0.15f), RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(noticeText(notice), modifier = Modifier.weight(1f), color = TextPrimary)
        TextButton(onClick = actions::dismissNotice) { Text(stringResource(R.string.setup_dismiss)) }
    }
}

@Composable
private fun Status(state: SetupState, actions: SetupActions) {
    when {
        state.reading -> Text(stringResource(R.string.setup_reading), color = TextSecondary)
        state.writing -> Text(stringResource(R.string.setup_writing), color = TextSecondary)
        state.board != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.setup_refresh))
        }
    }
}

/** The stored sign map is one the board refuses, so the IMU will not come up until it is fixed. */
@Composable
private fun FrameBad() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(SETUP_FRAME_BAD_TAG),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.setup_frame_incomplete_title), style = type.titleMedium, color = AccentRed)
        Text(stringResource(R.string.setup_frame_incomplete_body), style = type.bodyMedium, color = TextPrimary)
    }
}
