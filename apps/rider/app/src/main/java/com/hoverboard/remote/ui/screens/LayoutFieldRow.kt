package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.SecondaryTabRow
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.LayoutActions
import com.hoverboard.remote.LayoutState
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutEditor
import com.hoverboard.remote.model.LayoutGroup
import com.hoverboard.remote.model.LayoutRow
import com.hoverboard.remote.model.LayoutRows
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on one field's row. */
fun layoutRowTag(key: Key): String = "layout_row_${key.fieldId}_${key.index}"

/** Test tag on the control that opens one row's dialog. */
fun layoutChangeTag(key: Key): String = "layout_change_${key.fieldId}_${key.index}"

/** Test tag on the tab for [group], and for [motor] where the group is per-motor. */
fun layoutTabTag(group: LayoutGroup, motor: Int? = null): String =
    "layout_tab_${group.name.lowercase()}" + (motor?.let { "_$it" } ?: "")

/** Test tag on the action that adds a second motor to a board configured from scratch. */
const val LAYOUT_ADD_MOTOR_TAG = "layout_add_motor"

/** One tab of the editor: a group of fields, and the motor it belongs to where there is one. */
private data class LayoutTab(val group: LayoutGroup, val motor: Int? = null)

/**
 * The editor's groups as tabs (`specs/rider-ui.md` section 3.5a): the board's own pins, the IMU
 * bus, and one per motor. Material Design 3 secondary tabs, which is the sub-sections-of-one-screen
 * case; one page of every field buries the motor behind the LEDs.
 *
 * A motor past the first gets a tab exactly when it holds a pin, which is the all-or-none rule the
 * validator enforces asked as a question ([LayoutState.motorHoldsAPin]). A bare 12-FET board being
 * configured from scratch holds none and never would, so [LAYOUT_ADD_MOTOR_TAG] is the only way
 * that tab can first appear.
 */
@OptIn(ExperimentalMaterial3Api::class) // SecondaryTabRow, experimental in Material 3 1.3.2
@Composable
internal fun GroupTabs(state: LayoutState, editable: Boolean, actions: LayoutActions) {
    var added by rememberSaveable { mutableStateOf(false) }
    var picked by rememberSaveable { mutableIntStateOf(0) }
    // Motor 0 always has a tab, every board drives one. A later motor earns one by holding a pin,
    // or by the add action, which is the only way a board configured from scratch can get there.
    val motors = (0 until BoardFields.MOTORS).filter { it == 0 || added || state.motorHoldsAPin(it) }
    val tabs = listOf(LayoutTab(LayoutGroup.BOARD), LayoutTab(LayoutGroup.IMU)) +
        motors.map { LayoutTab(LayoutGroup.MOTOR, it) }
    val selected = picked.coerceIn(0, tabs.lastIndex)
    SecondaryTabRow(selectedTabIndex = selected) {
        tabs.forEachIndexed { i, tab ->
            Tab(
                selected = i == selected,
                onClick = { picked = i },
                text = { Text(tabTitle(tab)) },
                modifier = Modifier.testTag(layoutTabTag(tab.group, tab.motor)),
            )
        }
    }
    if (motors.size < BoardFields.MOTORS) {
        OutlinedButton(
            onClick = {
                added = true
                picked = tabs.size
            },
            enabled = editable,
            modifier = Modifier.testTag(LAYOUT_ADD_MOTOR_TAG),
        ) { Text(stringResource(R.string.layout_add_motor)) }
    }
    val tab = tabs[selected]
    Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
        if (tab.group == LayoutGroup.MOTOR) Caption(R.string.layout_motor_caption)
        for (row in LayoutRows.of(tab.group, tab.motor)) FieldRow(row, state, editable, actions)
    }
}

@Composable
private fun tabTitle(tab: LayoutTab): String =
    tab.motor?.let { stringResource(tab.group.title, it) } ?: stringResource(tab.group.title)

/** One field: what the board holds, what is staged for it, its standing rule, and its editor. */
@Composable
internal fun FieldRow(row: LayoutRow, state: LayoutState, editable: Boolean, actions: LayoutActions) {
    val slot = row.slot
    val stored = state.stored?.let { slot.of(it) }
    val staged = state.staged?.let { slot.of(it) }
    Panel(modifier = Modifier.testTag(layoutRowTag(slot.key))) {
        Text(stringResource(row.label), style = MaterialTheme.typography.titleSmall, color = TextPrimary)
        Text(
            text = when {
                staged != null -> layoutValueText(slot, staged)
                slot.key in state.unread -> stringResource(R.string.layout_field_unread)
                else -> stringResource(R.string.layout_not_read)
            },
            color = if (staged != null && staged != stored) AccentYellow else TextSecondary,
        )
        if (slot.key in state.written) {
            Text(stringResource(R.string.layout_written_mark), color = AccentYellow)
        }
        row.note?.let { Caption(it, row.noteArgs) }
        Editor(row, state, staged, editable, actions)
    }
}

/**
 * The row's one control, and the dialog behind it.
 *
 * Nothing can be edited without the part: which pins are free and eligible is a fact about silicon,
 * and a value staged against no part could never be judged. The row SAYS so, in place of its
 * control: a row that renders no affordance and no reason is how the first build looked broken
 * (`specs/rider-ui.md` section 3.5a). The power latch is the exception, because it is never
 * editable and carries its own standing note.
 */
@Composable
private fun Editor(
    row: LayoutRow,
    state: LayoutState,
    staged: Int?,
    editable: Boolean,
    actions: LayoutActions,
) {
    var open by rememberSaveable(key = "open_${row.slot.key}") { mutableStateOf(false) }
    if (row.editor != LayoutEditor.ReadOnly && state.part == null) {
        Caption(R.string.layout_row_part_unknown)
        return
    }
    when (val editor = row.editor) {
        LayoutEditor.ReadOnly -> Unit
        LayoutEditor.Pin -> {
            Change(row, editable && state.reserved != null) { open = true }
            if (open) PinDialog(row, state, { open = false }, actions)
        }
        is LayoutEditor.Choices -> {
            Change(row, editable) { open = true }
            if (open) ChoiceDialog(row, editor, { open = false }, actions)
        }
        is LayoutEditor.Number -> {
            Change(row, editable) { open = true }
            if (open) NumberDialog(row, editor, staged, { open = false }, actions)
        }
    }
}

/** The row's one control: it opens the dialog, and the dialog does the editing. */
@Composable
private fun Change(row: LayoutRow, enabled: Boolean, onOpen: () -> Unit) {
    TextButton(
        onClick = onOpen,
        enabled = enabled,
        modifier = Modifier.testTag(layoutChangeTag(row.slot.key)),
    ) { Text(stringResource(R.string.layout_change)) }
}
