package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
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
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInRoot
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupNotice
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SetupGroup
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Warning
import com.hoverboard.remote.ui.theme.WarningText
import kotlin.math.roundToInt

/** Test tag on the armed lock statement. */
const val SETUP_LOCK_TAG = "setup_lock"

/** Test tag on the warning that the stored sign map is one the board refuses. */
const val SETUP_FRAME_BAD_TAG = "setup_frame_bad"

/** Test tag on the line shown while the IMU frame's fields are being written. */
const val SETUP_FRAME_WRITING_TAG = "setup_frame_writing"

/** Test-tag prefix of the Setup selector's chips ([nodeChipTag]). */
const val SETUP_NODE_TAG = "setup_target"

/** Test tag on a group's ADVANCED toggle. */
fun setupAdvancedTag(group: SetupGroup): String = "setup_advanced_${group.name}"

/** Test tag on one row's retry, offered after a write of that field failed. */
fun setupRetryTag(key: Key): String = "setup_retry_${key.fieldId}_${key.index}"

/** Test tag prefix on each field row: `setup_row_<field_id>_<index>`. */
fun setupRowTag(key: Key): String = "setup_row_${key.fieldId}_${key.index}"

/**
 * The Setup screen (`specs/remote-ui.md` sections 3.4, 3.6a and 3.7a): a grouped store editor for
 * one node the walk found (the selector at the top), minus the pin block (the layout editor,
 * section 3.5). A motor controller shows every row; a bridge, and a node whose class is not read
 * yet, shows the keys every node has ([SetupFields.rowsFor]).
 *
 * There is no Apply. Each field is written when it is edited and each row reports what the board
 * said about that field, including when the board will start using it. While [armed] the screen is
 * visible and read-only, and every row says so.
 *
 * @param telemetry what the SELECTED board last reported, which the attitude flows read. The
 *   caller resolves it from the selector, so the lean this screen levels is the lean of the board
 *   it is editing.
 * @param odometers how far each motor controller's wheel has travelled, labelled by its node
 *   label ([TravelSection]). Every board rather than the selected one, because the machine's figure
 *   is the mean of the wheels.
 * @param status the machine status section ([MachineStatusSection], `specs/remote-ui.md` 3.1c):
 *   the first thing after the title, with the armed lock, above the node selector, because it covers
 *   every board and does not follow the selector.
 * @param scrollTo the row to scroll to the top once it is placed: Ride's missing-setting statements
 *   open Setup this way (`specs/remote-ui.md` 3.1, "Missing settings, in plain words"). The screen
 *   scrolls to it once per key.
 */
@Composable
fun SetupScreen(
    state: SetupState,
    armed: Boolean,
    telemetry: TelemetryUi?,
    odometers: List<Pair<String, OdometerReading>>,
    actions: SetupActions,
    modifier: Modifier = Modifier,
    status: @Composable () -> Unit = {},
    scrollTo: Key? = null,
) {
    DisposableEffect(actions) {
        actions.onShown()
        onDispose { actions.onHidden() }
    }
    val editable = !armed && !state.busy && state.board != null
    val scroll = rememberScrollState()
    var columnTop by remember { mutableFloatStateOf(0f) }
    var scrolled by remember(scrollTo) { mutableStateOf(false) }
    var rowTop by remember(scrollTo) { mutableStateOf<Float?>(null) }
    val placed = { top: Float -> if (rowTop == null) rowTop = top }
    LaunchedEffect(rowTop) {
        val top = rowTop ?: return@LaunchedEffect
        scroll.scrollTo(scroll.value + (top - columnTop).roundToInt())
        scrolled = true
    }

    Column(
        modifier = modifier
            .fillMaxSize()
            .onGloballyPositioned { columnTop = it.positionInRoot().y }
            .verticalScroll(scroll)
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(stringResource(R.string.setup_title), style = MaterialTheme.typography.headlineSmall, color = Fg)
        if (armed) Lock()
        status()
        NodeChips(state.nodes, state.board, actions::selectTarget, SETUP_NODE_TAG)

        state.notice?.let { Notice(it, actions) }
        Status(state, actions)
        if (state.storedFrameUnsafe) FrameBad()
        MachineSection(state, editable, actions)
        TravelSection(odometers)

        for (group in SetupGroup.entries) {
            Group(group, state, armed, editable, telemetry, actions, scrollTo.takeUnless { scrolled }, placed)
        }
    }
}

/**
 * One group's rows. The IMU group leads with its flows (set level, orientation, the rotation check)
 * and keeps its raw editors behind ADVANCED (`specs/remote-ui.md` 3.4: "the primary UI is flows").
 *
 * Every motor controller gets the same flows, against its own readings: [telemetry] is whichever
 * board the selector names, and each board's lean reaches the app under its own address. A bridge
 * has no IMU, so its rows are the identity group's alone and no flow is drawn for it.
 */
@Composable
private fun Group(
    group: SetupGroup,
    state: SetupState,
    armed: Boolean,
    editable: Boolean,
    telemetry: TelemetryUi?,
    actions: SetupActions,
    scrollTo: Key?,
    onPlace: (rowTop: Float) -> Unit,
) {
    var advanced by rememberSaveable { mutableStateOf(false) }
    val rows = state.rows.filter { it.group == group }
    if (rows.isEmpty()) return
    Section(stringResource(group.title)) {
        rows.filterNot { it.advanced }.forEach {
            if (it.key == scrollTo) {
                Box(Modifier.onGloballyPositioned { c -> onPlace(c.positionInRoot().y) }) {
                    FieldRow(it, state, armed, editable, actions)
                }
            } else {
                FieldRow(it, state, armed, editable, actions)
            }
        }
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
        if (advanced) {
            hidden.forEach {
                FieldRow(it, state, armed, editable, actions)
                MachineRowVerdict(it, state)
            }
        }
    }
}

@Composable
private fun Lock() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, Primary, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(SETUP_LOCK_TAG),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.setup_locked_title), style = type.titleMedium, color = Primary)
        Text(stringResource(R.string.setup_locked_body), style = type.bodyMedium, color = Fg)
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
            .background(BgElevated, RoundedCornerShape(8.dp))
            .border(1.dp, Border, RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(noticeText(notice), modifier = Modifier.weight(1f), color = Fg)
        TextButton(onClick = actions::dismissNotice) { Text(stringResource(R.string.setup_dismiss)) }
    }
}

@Composable
private fun Status(state: SetupState, actions: SetupActions) {
    when {
        state.reading -> Text(stringResource(R.string.setup_reading), color = FgMuted)
        state.writing -> Text(stringResource(R.string.setup_writing), color = FgMuted)
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
            .border(2.dp, Warning, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(SETUP_FRAME_BAD_TAG),
    ) {
        val type = MaterialTheme.typography
        WarningText(stringResource(R.string.setup_frame_incomplete_title), style = type.titleMedium)

        Text(stringResource(R.string.setup_frame_incomplete_body), style = type.bodyMedium, color = Fg)
    }
}
