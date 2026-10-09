package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.candidatePins
import com.hoverboard.remote.LayoutActions
import com.hoverboard.remote.LayoutState
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutEditor
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.model.LayoutGroup
import com.hoverboard.remote.model.LayoutRow
import com.hoverboard.remote.model.LayoutRows
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on one field's row. */
fun layoutRowTag(key: Key): String = "layout_row_${key.fieldId}_${key.index}"

/** Test tag on the control that opens one row's pin picker. */
fun layoutChangeTag(key: Key): String = "layout_change_${key.fieldId}_${key.index}"

/** Test tag on the pin picker's offer of [packed] (or of "not set", as [PIN_ABSENT]). */
fun layoutPinTag(key: Key, packed: Int): String = "layout_pin_${key.fieldId}_${key.index}_$packed"

/** The editor's sections: the board's own pins, the IMU bus, and one per motor. */
@Composable
internal fun Groups(state: LayoutState, editable: Boolean, actions: LayoutActions) {
    Section(stringResource(LayoutGroup.BOARD.title)) {
        for (row in LayoutRows.of(LayoutGroup.BOARD)) FieldRow(row, state, editable, actions)
    }
    Section(stringResource(LayoutGroup.IMU.title)) {
        for (row in LayoutRows.of(LayoutGroup.IMU)) FieldRow(row, state, editable, actions)
    }
    for (m in 0 until BoardFields.MOTORS) {
        Section(stringResource(LayoutGroup.MOTOR.title, m)) {
            Caption(R.string.layout_motor_caption)
            for (row in LayoutRows.of(LayoutGroup.MOTOR, m)) FieldRow(row, state, editable, actions)
        }
    }
}

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
        row.note?.let { Caption(it) }
        when (val editor = row.editor) {
            LayoutEditor.ReadOnly -> Unit
            LayoutEditor.Pin -> PinEditor(row, state, editable, actions)
            is LayoutEditor.Choices -> ChoiceEditor(row, editor, staged, editable, actions)
            is LayoutEditor.Number -> NumberEditor(row, editor, staged, editable, actions)
        }
    }
}

/**
 * The pin picker: only the pins that are free and capability-eligible for this function, so an
 * invalid layout is hard to express rather than merely refused. It needs the part, because
 * eligibility is a fact about silicon.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun PinEditor(row: LayoutRow, state: LayoutState, editable: Boolean, actions: LayoutActions) {
    var open by rememberSaveable(row.slot.key.toString()) { mutableStateOf(false) }
    val part = state.part
    val staged = state.staged
    val reserved = state.reserved
    if (part == null || staged == null || reserved == null) return
    TextButton(
        onClick = { open = !open },
        enabled = editable,
        modifier = Modifier.testTag(layoutChangeTag(row.slot.key)),
    ) {
        Text(stringResource(if (open) R.string.layout_close else R.string.layout_change))
    }
    if (!open) return
    val candidates = candidatePins(row.slot, staged, part, reserved)
    fun pick(raw: Int) {
        actions.stage(row.slot, raw)
        open = false
    }
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        FilterChip(
            selected = row.slot.of(staged) == PIN_ABSENT,
            onClick = { pick(PIN_ABSENT) },
            label = { Text(stringResource(R.string.layout_unset)) },
            enabled = editable,
            modifier = Modifier.testTag(layoutPinTag(row.slot.key, PIN_ABSENT)),
        )
        for (pin in candidates) {
            FilterChip(
                selected = row.slot.of(staged) == pin.packed,
                onClick = { pick(pin.packed) },
                label = { Text(pin.name) },
                enabled = editable,
                modifier = Modifier.testTag(layoutPinTag(row.slot.key, pin.packed)),
            )
        }
    }
    if (candidates.isEmpty()) Caption(R.string.layout_no_candidates)
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun ChoiceEditor(
    row: LayoutRow,
    editor: LayoutEditor.Choices,
    staged: Int?,
    editable: Boolean,
    actions: LayoutActions,
) {
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        for (choice in editor.choices) {
            FilterChip(
                selected = staged == choice.value,
                onClick = { actions.stage(row.slot, choice.value) },
                label = { Text(stringResource(choice.label)) },
                enabled = editable,
                modifier = Modifier.testTag(layoutPinTag(row.slot.key, choice.value)),
            )
        }
    }
}

@Composable
private fun NumberEditor(
    row: LayoutRow,
    editor: LayoutEditor.Number,
    staged: Int?,
    editable: Boolean,
    actions: LayoutActions,
) {
    var text by remember(staged) { mutableStateOf(staged?.toString().orEmpty()) }
    val parsed = text.trim().toLongOrNull()?.takeIf { it in editor.range }
    Row(verticalAlignment = Alignment.CenterVertically) {
        OutlinedTextField(
            value = text,
            onValueChange = { text = it },
            enabled = editable,
            singleLine = true,
            isError = text.isNotEmpty() && parsed == null,
            modifier = Modifier.weight(1f),
        )
        Spacer(modifier = Modifier.width(8.dp))
        Button(
            onClick = { parsed?.let { actions.stage(row.slot, it.toInt()) } },
            enabled = editable && parsed != null,
            modifier = Modifier.testTag(layoutChangeTag(row.slot.key)),
        ) { Text(stringResource(R.string.layout_change)) }
    }
}
