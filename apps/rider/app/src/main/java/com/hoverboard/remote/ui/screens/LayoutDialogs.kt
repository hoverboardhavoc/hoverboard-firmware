package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.DialogProperties
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.candidatePins
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.LayoutActions
import com.hoverboard.remote.LayoutState
import com.hoverboard.remote.R
import com.hoverboard.remote.model.LayoutEditor
import com.hoverboard.remote.model.LayoutRow

// Editing is by dialog rather than in the row (`specs/rider-ui.md` section 3.5a). For "choose one
// value from a short list" Material Design 3 sanctions a simple dialog that commits on tap and
// closes, and for free text an edit dialog; every row here is one or the other, and a pin list is
// far too long to sit in a row on a phone. Two actions per dialog at most.

/**
 * The width every dialog here takes: Material's own constraints rather than the platform's.
 *
 * Not cosmetic. A platform-width dialog holding a text field never settles its measure pass, and
 * under Robolectric that is an outright hang, so the edit dialog cannot be tested with the platform
 * width at all. The two choosers take the same width so the three dialogs are one shape.
 */
private val DIALOG_WIDTH = DialogProperties(usePlatformDefaultWidth = false)

/** Test tag on the dialog that stages one field. */
fun layoutDialogTag(key: Key): String = "layout_dialog_${key.fieldId}_${key.index}"

/** Test tag on the dialog's offer of [value] (a packed pin, [PIN_ABSENT], or a choice's byte). */
fun layoutOptionTag(key: Key, value: Int): String = "layout_option_${key.fieldId}_${key.index}_$value"

/**
 * The pin picker: only the pins that are free and capability-eligible for this function, so an
 * invalid layout is hard to express rather than merely refused. It needs the part, because
 * eligibility is a fact about silicon.
 */
@Composable
internal fun PinDialog(row: LayoutRow, state: LayoutState, onClose: () -> Unit, actions: LayoutActions) {
    val part = state.part ?: return
    val staged = state.staged ?: return
    val reserved = state.reserved ?: return
    val candidates = candidatePins(row.slot, staged, part, reserved)
    OptionDialog(row, onClose) {
        Option(layoutOptionTag(row.slot.key, PIN_ABSENT), stringResource(R.string.layout_unset)) {
            actions.stage(row.slot, PIN_ABSENT)
            onClose()
        }
        for (pin in candidates) {
            Option(layoutOptionTag(row.slot.key, pin.packed), pin.name) {
                actions.stage(row.slot, pin.packed)
                onClose()
            }
        }
        if (candidates.isEmpty()) Caption(R.string.layout_no_candidates)
    }
}

/** One of a fixed set of byte values: the IMU model, the phase-current declaration. */
@Composable
internal fun ChoiceDialog(
    row: LayoutRow,
    editor: LayoutEditor.Choices,
    onClose: () -> Unit,
    actions: LayoutActions,
) {
    OptionDialog(row, onClose) {
        for (choice in editor.choices) {
            Option(layoutOptionTag(row.slot.key, choice.value), stringResource(choice.label)) {
                actions.stage(row.slot, choice.value)
                onClose()
            }
        }
    }
}

/** The edit dialog: the dead time, which is a number rather than a choice from a list. */
@Composable
internal fun NumberDialog(
    row: LayoutRow,
    editor: LayoutEditor.Number,
    staged: Int?,
    onClose: () -> Unit,
    actions: LayoutActions,
) {
    var text by rememberSaveable(key = "number_${row.slot.key}") { mutableStateOf(staged?.toString().orEmpty()) }
    val parsed = text.trim().toLongOrNull()?.takeIf { it in editor.range }
    AlertDialog(
        onDismissRequest = onClose,
        title = { Text(stringResource(row.label)) },
        text = {
            OutlinedTextField(
                value = text,
                onValueChange = { text = it },
                singleLine = true,
                isError = text.isNotEmpty() && parsed == null,
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            )
        },
        confirmButton = {
            TextButton(
                onClick = {
                    parsed?.let { actions.stage(row.slot, it.toInt()) }
                    onClose()
                },
                enabled = parsed != null,
            ) { Text(stringResource(R.string.layout_set)) }
        },
        dismissButton = { TextButton(onClick = onClose) { Text(stringResource(R.string.layout_cancel)) } },
        properties = DIALOG_WIDTH,
        modifier = Modifier.testTag(layoutDialogTag(row.slot.key)),
    )
}

/**
 * The simple dialog both choosers are: the field's name, its options, and one action.
 *
 * The options are a plain scrolling column rather than a lazy list because a pin list is short
 * enough to compose whole, and tapping one commits it and closes, so there is no confirm step to
 * leave an option half-chosen.
 */
@Composable
private fun OptionDialog(row: LayoutRow, onClose: () -> Unit, options: @Composable ColumnScope.() -> Unit) {
    AlertDialog(
        onDismissRequest = onClose,
        title = { Text(stringResource(row.label)) },
        text = {
            Column(
                modifier = Modifier
                    .heightIn(max = 360.dp)
                    .verticalScroll(rememberScrollState()),
                content = options,
            )
        },
        confirmButton = { TextButton(onClick = onClose) { Text(stringResource(R.string.layout_cancel)) } },
        properties = DIALOG_WIDTH,
        modifier = Modifier.testTag(layoutDialogTag(row.slot.key)),
    )
}

/** One offered value: a full-width row that stages it and closes the dialog. */
@Composable
private fun Option(tag: String, label: String, onPick: () -> Unit) {
    TextButton(
        onClick = onPick,
        modifier = Modifier
            .fillMaxWidth()
            .testTag(tag),
    ) {
        Text(label, modifier = Modifier.weight(1f), textAlign = TextAlign.Start)
    }
}
