package com.hoverboardhavoc.remote.ui.screens

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
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Key
import com.hoverboardhavoc.remote.LayoutActions
import com.hoverboardhavoc.remote.LayoutNotice
import com.hoverboardhavoc.remote.LayoutState
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.ui.theme.BgElevated
import com.hoverboardhavoc.remote.ui.theme.Border
import com.hoverboardhavoc.remote.ui.theme.CautionText
import com.hoverboardhavoc.remote.ui.theme.Fg
import com.hoverboardhavoc.remote.ui.theme.FgMuted
import com.hoverboardhavoc.remote.ui.theme.Primary

/** Test tag on the Apply button. */
const val LAYOUT_APPLY_TAG = "layout_apply"

/** Test tag on the armed lock statement. */
const val LAYOUT_LOCK_TAG = "layout_lock"

/** Test tag on the power-cycle instruction. */
const val LAYOUT_POWER_CYCLE_TAG = "layout_power_cycle"

/** Test tag on the statement that the board's part is not known. */
const val LAYOUT_PART_UNKNOWN_TAG = "layout_part_unknown"

/**
 * The board layout editor (`specs/remote-ui.md` section 3.5): the pins and the few board facts
 * that configure them, for the attached board.
 *
 * Its own door rather than a section of Setup, because the editing model is not Setup's. Setup is a
 * list of independent values, each meaningful alone; a layout is ONE object, so the changes are
 * staged together and listed as a delta from what the board holds, and Apply writes them.
 *
 * It adjusts a board that is already onboarded. Taking a board from blank to a bootable layout is
 * done from `hoverboard-cli` at the bench (`specs/decision-cli-onboarding.md` section 1), so this
 * screen offers no whole-layout presets.
 *
 * The firmware applies what is stored and checks nothing, so Apply runs the configuration checks
 * (`specs/config-read-once.md` section 4) on the board as the changes would leave it and writes
 * nothing when one refuses, naming the field and the reason. What still fails at boot is a runtime
 * fact, a peripheral that did not come up, which the board lists in its boot status record and
 * Setup's machine status shows (`specs/configured-links.md` section 5).
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
    // The part is not in this conjunction: only a pin row needs it, to know which pins exist, and
    // the row says so itself (LayoutFieldRow).
    val editable = !armed && !state.busy && state.board != null && state.staged != null

    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(stringResource(R.string.layout_title), style = MaterialTheme.typography.headlineSmall, color = Fg)
        Text(
            text = state.board?.let { stringResource(R.string.layout_target, boardHex(it)) }
                ?: stringResource(R.string.layout_no_board),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
        if (state.board != null) Part(state)
        if (armed) Lock()
        state.notice?.let { Notice(it, state, actions) }
        Status(state, actions)
        Delta(state, armed, actions)
        PowerCycle(state)
        if (state.staged == null && state.unread.isEmpty()) Caption(R.string.layout_not_read)
        GroupTabs(state, editable, actions)
    }
}

@Composable
private fun Lock() {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, Primary, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(LAYOUT_LOCK_TAG),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.layout_locked_title), style = type.titleMedium, color = Primary)
        Text(stringResource(R.string.layout_locked_body), style = type.bodyMedium, color = Fg)
    }
}

@Composable
private fun Notice(notice: LayoutNotice, state: LayoutState, actions: LayoutActions) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(BgElevated, RoundedCornerShape(8.dp))
            .border(1.dp, Border, RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(layoutNoticeText(notice, state.part), modifier = Modifier.weight(1f), color = Fg)
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
        state.reading -> Text(stringResource(R.string.layout_reading), color = FgMuted)
        state.applying -> Text(stringResource(R.string.layout_writing), color = FgMuted)
        state.board != null -> OutlinedButton(onClick = actions::refresh) {
            Text(stringResource(R.string.layout_refresh))
        }
    }
    if (state.unread.isNotEmpty()) {
        CautionText(stringResource(R.string.layout_unread, state.unread.size))
    }
}

/**
 * Which part the board says it is: the `chip` tag of its own `CYCLIC_STATE`
 * (`shared/linkctl/src/lib.rs`, `CyclicObs`), which is what the pin picker offers pins of and what
 * the checks judge the layout against.
 *
 * Two boards cannot be named: one that has sent no `CYCLIC_STATE` yet, so no part has arrived, and
 * one whose boot probe measured a combination the firmware allocates no fleet part for, which sends
 * the unknown tag. On neither can a pin be picked, because which pins exist is a fact about the
 * part, so the screen says the part is not known. There is no way to state it by hand.
 */
@Composable
private fun Part(state: LayoutState) {
    val part = state.part
    if (part != null) {
        Text(
            text = stringResource(R.string.layout_part, part.name),
            style = MaterialTheme.typography.bodyMedium,
            color = FgMuted,
        )
        return
    }
    Panel(modifier = Modifier.testTag(LAYOUT_PART_UNKNOWN_TAG)) {
        PanelTitle(R.string.layout_part_unknown_title)
        Text(
            text = stringResource(
                if (state.chip == null) R.string.layout_part_unsent else R.string.layout_part_unnamed,
            ),
            style = MaterialTheme.typography.bodyMedium,
            color = Fg,
        )
        Caption(R.string.layout_part_unknown_body)
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
                    layoutDeltaLine(slot, slot.of(stored), slot.of(staged), state.part),
                    modifier = Modifier.weight(1f),
                    color = Fg,
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
    }
}

/**
 * The one power-cycle instruction, and nothing to confirm.
 *
 * The board's boot counter rides every `CYCLIC_STATE`, so the app sees the restart happen and
 * clears this panel itself (`LayoutModel.observeBoot`). It used to end in a button saying "I
 * power-cycled it", which took the operator's word for the one thing the wire already states.
 */
@Composable
private fun PowerCycle(state: LayoutState) {
    if (!state.awaitingPowerCycle) return
    Panel(modifier = Modifier.testTag(LAYOUT_POWER_CYCLE_TAG)) {
        Text(
            stringResource(R.string.layout_power_cycle, state.written.size),
            style = MaterialTheme.typography.titleMedium,
            color = Fg,
        )
        Caption(R.string.layout_power_cycle_watch)
    }
}
