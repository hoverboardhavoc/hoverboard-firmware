package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
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
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.LayoutActions
import com.hoverboard.remote.LayoutNotice
import com.hoverboard.remote.LayoutState
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the verdict panel. */
const val LAYOUT_VERDICT_TAG = "layout_verdict"

/** Test tag on the statement that the board stores a layout it would refuse. */
const val LAYOUT_STORED_INVALID_TAG = "layout_stored_invalid"

/** Test tag on the Apply button. */
const val LAYOUT_APPLY_TAG = "layout_apply"

/** Test tag on the armed lock statement. */
const val LAYOUT_LOCK_TAG = "layout_lock"

/** Test tag on the power-cycle instruction. */
const val LAYOUT_POWER_CYCLE_TAG = "layout_power_cycle"

/** Test tag on the part picker's chip for [part]. */
fun layoutPartTag(part: ChipFamily): String = "layout_part_${part.name}"

/**
 * The board layout editor (`specs/rider-ui.md` section 3.5): the pins and the few board facts the
 * boot validator judges, for the attached board.
 *
 * Its own door rather than a section of Setup, because the editing model is not Setup's. Setup is a
 * list of independent values, each meaningful alone; a layout is ONE object with ownership rules
 * across its fields, where a half-applied change is not partial progress but an invalid board. So
 * the screen shows one verdict over the whole staged layout, Apply is refused outright unless that
 * verdict is clean, and the changes are listed as a delta from what the board holds.
 *
 * The verdict is exact, not advisory: the board's boot validator is a pure function of these fields,
 * the part's capabilities and the reserved pins, and the app runs that same function
 * ([com.hoverboard.protocol.board.validate]). What it cannot know without a reboot is whether the
 * hardware then behaves, and the panel says which is which.
 */
@Composable
fun LayoutScreen(
    state: LayoutState,
    armed: Boolean,
    actions: LayoutActions,
    modifier: Modifier = Modifier,
) {
    DisposableEffect(actions) {
        actions.onShown()
        onDispose { actions.onHidden() }
    }
    val editable = !armed && !state.busy && state.board != null && state.staged != null

    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(stringResource(R.string.layout_title), style = MaterialTheme.typography.headlineSmall, color = TextPrimary)
        Text(
            text = state.board?.let { stringResource(R.string.layout_target, boardHex(it)) }
                ?: stringResource(R.string.layout_no_board),
            style = MaterialTheme.typography.bodyMedium,
            color = TextSecondary,
        )
        if (armed) Lock()
        state.notice?.let { Notice(it, state, actions) }
        Status(state, actions)
        Presets(state, actions)
        LatchConfirmation(state, actions)
        PartPicker(state, actions)
        Verdict(state)
        Delta(state, armed, actions)
        PowerCycle(state, actions)
        if (state.staged == null && state.unread.isEmpty()) Caption(R.string.layout_not_read)
        GroupTabs(state, editable, actions)
    }
}

@Composable
private fun Lock() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(LAYOUT_LOCK_TAG),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.layout_locked_title), style = type.titleMedium, color = AccentRed)
        Text(stringResource(R.string.layout_locked_body), style = type.bodyMedium, color = TextPrimary)
    }
}

@Composable
private fun Notice(notice: LayoutNotice, state: LayoutState, actions: LayoutActions) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(AccentYellow.copy(alpha = 0.15f), RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(layoutNoticeText(notice), modifier = Modifier.weight(1f), color = TextPrimary)
        // The slot was taken, the board was not asked: applying again is the retry, and what was
        // not written is still in the delta.
        if (notice == LayoutNotice.SlotBusy && state.board != null) {
            TextButton(onClick = actions::apply) { Text(stringResource(R.string.layout_retry)) }
        }
        TextButton(onClick = actions::dismissNotice) { Text(stringResource(R.string.layout_dismiss)) }
    }
}

@Composable
private fun Status(state: LayoutState, actions: LayoutActions) {
    when {
        state.reading -> Text(stringResource(R.string.layout_reading), color = TextSecondary)
        state.applying -> Text(stringResource(R.string.layout_writing), color = TextSecondary)
        state.board != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.layout_refresh))
        }
    }
    if (state.unread.isNotEmpty()) {
        Text(stringResource(R.string.layout_unread, state.unread.size), color = AccentYellow)
    }
}

/**
 * Which part this board is. There is no verdict without it: the chip is not readable over the link,
 * and the same layout is valid on one part and refused on the next.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun PartPicker(state: LayoutState, actions: LayoutActions) {
    Panel {
        PanelTitle(R.string.layout_part_title)
        Caption(R.string.layout_part_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (part in ChipFamily.entries) {
                FilterChip(
                    selected = state.part == part,
                    onClick = { actions.selectPart(part) },
                    label = { Text(part.label) },
                    modifier = Modifier.testTag(layoutPartTag(part)),
                )
            }
        }
    }
}

/** The verdict over the staged layout, and what a verdict can and cannot say. */
@Composable
private fun Verdict(state: LayoutState) {
    val verdict = state.verdict
    val error = verdict?.error
    val colour = when {
        verdict == null -> TextSecondary
        error == null -> AccentGreen
        else -> AccentRed
    }
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, colour, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(LAYOUT_VERDICT_TAG),
        verticalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        PanelTitle(R.string.layout_verdict_title)
        Text(
            text = when {
                verdict == null -> stringResource(R.string.layout_verdict_unknown)
                error == null -> stringResource(R.string.layout_verdict_valid)
                else -> layoutRefusalText(error)
            },
            style = MaterialTheme.typography.bodyMedium,
            color = colour,
        )
        if (verdict != null) {
            Caption(R.string.layout_verdict_exact)
            Caption(R.string.layout_verdict_after_boot)
        }
        Caption(R.string.layout_verdict_reachable)
    }
    val stored = state.storedVerdict?.error ?: return
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(LAYOUT_STORED_INVALID_TAG),
    ) {
        Text(
            stringResource(R.string.layout_stored_invalid, layoutFieldLabel(stored.field)),
            style = MaterialTheme.typography.bodyMedium,
            color = AccentRed,
        )
    }
}

/** The changes Apply would write, from what the board holds to what is staged. */
@Composable
private fun Delta(state: LayoutState, armed: Boolean, actions: LayoutActions) {
    val stored = state.stored
    val staged = state.staged
    if (stored == null || staged == null) return
    if (state.delta.isEmpty()) {
        Caption(R.string.layout_no_changes)
        return
    }
    Panel {
        PanelTitle(R.string.layout_delta_title)
        for (slot in state.delta) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    layoutDeltaLine(slot, slot.of(stored), slot.of(staged)),
                    modifier = Modifier.weight(1f),
                    color = TextPrimary,
                )
                TextButton(onClick = { actions.revert(slot) }) { Text(stringResource(R.string.layout_revert)) }
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                onClick = actions::apply,
                enabled = state.canApply(armed),
                modifier = Modifier.testTag(LAYOUT_APPLY_TAG),
            ) { Text(stringResource(R.string.layout_apply)) }
            OutlinedButton(onClick = actions::revertAll) { Text(stringResource(R.string.layout_revert_all)) }
        }
        if (!state.clean) Caption(R.string.layout_apply_blocked)
    }
}

@Composable
private fun PowerCycle(state: LayoutState, actions: LayoutActions) {
    if (!state.awaitingPowerCycle) return
    Panel(modifier = Modifier.testTag(LAYOUT_POWER_CYCLE_TAG)) {
        Text(
            stringResource(R.string.layout_power_cycle, state.written.size),
            style = MaterialTheme.typography.titleMedium,
            color = TextPrimary,
        )
        Button(onClick = actions::confirmPowerCycled, enabled = state.linkDroppedSinceApply && !state.busy) {
            Text(stringResource(R.string.layout_power_cycled))
        }
        if (!state.linkDroppedSinceApply) Caption(R.string.layout_power_cycle_wait)
    }
}
