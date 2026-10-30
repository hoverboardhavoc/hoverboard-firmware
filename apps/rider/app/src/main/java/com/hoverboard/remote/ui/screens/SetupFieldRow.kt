package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
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
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.FieldWrite
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.WriteState
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.FloorOutside
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import com.hoverboard.remote.model.asLong
import com.hoverboard.remote.model.display
import com.hoverboard.remote.model.fitsOneWrite
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Tone
import com.hoverboard.remote.ui.theme.ToneText
import com.hoverboard.remote.ui.theme.WarningText

/**
 * One field: its stored value, what came of the app's own write of it, when the board will use a
 * value written here, its caveat and its editor.
 *
 * The write result is the row's own (`specs/rider-ui.md` 3.6a): there is no Apply and no batch
 * verdict, so a refusal names this field and nothing else, and the retry re-sends this field alone.
 */
@Composable
internal fun FieldRow(
    field: SetupField,
    state: SetupState,
    armed: Boolean,
    editable: Boolean,
    actions: SetupActions,
) {
    val key = field.key
    val stored = state.values[key]
    val write = state.writes[key]
    Panel(modifier = Modifier.testTag(setupRowTag(key))) {
        Text(stringResource(field.label), style = MaterialTheme.typography.titleSmall, color = Fg)
        Text(
            text = when {
                stored != null -> stringResource(R.string.setup_stored, stored.display())
                // Before the plain unread case: a board that answered with something this app
                // cannot read has not failed to answer, and saying "Not read" about it is false.
                key in state.garbled -> stringResource(R.string.setup_garbled)
                key in state.unread -> stringResource(R.string.setup_unread)
                else -> stringResource(R.string.setup_not_read_yet)
            },
            color = FgMuted,
        )
        FloorWarning(field, state)
        if (field.showDefault) {
            Text(stringResource(R.string.setup_default, field.def.default.display()), color = FgMuted)
        }
        // Every row says so while the board is armed, rather than one banner saying it once: the
        // board refuses every config write then, and the refusal belongs to each field.
        if (armed && field.takesEffect != null) {
            Text(stringResource(R.string.setup_row_armed), color = Primary)
        }
        write?.let { WriteResult(field, it, editable, actions) }
        TakeEffect(field)
        field.note?.let { Caption(it, field.noteArgs) }
        RangeNote(field)
        val shown = write?.takeIf { it.sending }?.want ?: stored
        when (val editor = field.editor) {
            Editor.ReadOnly, is Editor.Flow -> Unit
            is Editor.Chips -> Chips(field, editor, shown, editable, actions)
            is Editor.Range, Editor.Generic -> Entry(field, shown, editable) { actions.edit(field.key, it) }
        }
    }
}

/** Test tag on the floor row's warning that the floor sits outside the stated pack. */
const val SETUP_FLOOR_OUTSIDE_TAG = "setup_floor_outside"

/**
 * The low-battery floor row's warning (`specs/rider-ui.md` 3.6d): the floor stays absolute and
 * operator-set, so a floor outside the stated pack is shown rather than corrected. Nothing on any
 * other row.
 */
@Composable
private fun FloorWarning(field: SetupField, state: SetupState) {
    if (field != SetupFields.BATTERY_FLOOR) return
    val text = when (state.floorOutside ?: return) {
        FloorOutside.AT_OR_ABOVE_FULL -> R.string.setup_floor_above_full
        FloorOutside.BELOW_EMPTY -> R.string.setup_floor_below_empty
    }
    CautionText(stringResource(text), modifier = Modifier.testTag(SETUP_FLOOR_OUTSIDE_TAG))
}

/** What came of this row's write ([writeStatus]), and a retry for the ones that can be sent again. */
@Composable
private fun WriteResult(field: SetupField, write: FieldWrite, editable: Boolean, actions: SetupActions) {
    val (body, tone) = writeStatus(field, write)
    Row(verticalAlignment = Alignment.CenterVertically) {
        ToneText(body, tone, modifier = Modifier.weight(1f))
        if (write.failed) {
            TextButton(
                onClick = { actions.retry(field.key) },
                enabled = editable,
                modifier = Modifier.testTag(setupRetryTag(field.key)),
            ) { Text(stringResource(R.string.setup_retry)) }
        }
    }
}

/**
 * What this row's write said, and the tone to say it in, for a row that renders it inline and for
 * the Machine section's two boards.
 *
 * A [TakesEffect.RESTART] field still marked written has not been taken up: the model drops that
 * record when the boot tag on the wire shows the board restarted, so its presence IS the statement
 * that no restart has happened. Nothing asks the operator to assert one.
 */
@Composable
internal fun writeStatus(field: SetupField, write: FieldWrite): Pair<String, Tone> {
    val restartDue = field.takesEffect == TakesEffect.RESTART
    val (text, colour) = when (write.state) {
        // The same word the screen-level indicator uses: one string for one fact.
        WriteState.Sending -> R.string.setup_writing to Tone.MUTED
        WriteState.Written ->
            (if (restartDue) R.string.setup_write_awaiting_restart else R.string.setup_write_written) to Tone.PRIMARY
        else -> writeFailureText(write.state) to Tone.WARNING
    }
    val body = when (val st = write.state) {
        is WriteState.Mismatch -> stringResource(text, write.want.display(), st.stored?.display() ?: "?")
        else -> stringResource(text)
    }
    return body to colour
}

/**
 * When this row's value reaches the board, in one line.
 *
 * Both sentences come off [SetupField.takesEffect], which reads the firmware's own arm-time row
 * through the drift-pinned mirror (`specs/integration.md`, "When a stored value takes effect: the
 * arm-time re-read"), so a row cannot state one class while the board runs the other. It is shown
 * before the edit as well as after it: the operator deciding whether to change a value is the one
 * who needs to know what it will cost. A read-only row says nothing, having no edit to place in
 * time.
 */
@Composable
internal fun TakeEffect(field: SetupField) {
    val line = when (field.takesEffect) {
        TakesEffect.NEXT_ARM -> R.string.setup_effect_arm
        TakesEffect.RESTART -> R.string.setup_effect_boot
        TakesEffect.APP_ONLY -> R.string.setup_effect_app
        null -> return
    }
    Caption(line)
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun Chips(field: SetupField, editor: Editor.Chips, shown: Value?, editable: Boolean, actions: SetupActions) {
    val current = shown?.asLong()
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        for (choice in editor.choices) {
            FilterChip(
                selected = current == choice.value.toLong(),
                onClick = { field.parse(choice.value.toString())?.let { actions.edit(field.key, it) } },
                label = { Text(stringResource(choice.label)) },
                enabled = editable,
            )
        }
    }
    if (current != null && editor.choices.none { it.value.toLong() == current }) {
        CautionText(stringResource(R.string.setup_choice_unknown, current.toString()))
    }
}

/**
 * The range a numeric row takes, which is the clamp the firmware applies as it reads the value: a
 * word outside it is stored as written and narrowed on the way into the loop, so the bound is
 * stated before the edit rather than discovered when the machine does not do what the number said.
 *
 * Read off the editor, so a row cannot state a range it does not enforce, and shown for no other
 * kind of editor: a chip row's choices are its range and a generic row has the type's own.
 */
@Composable
internal fun RangeNote(field: SetupField) {
    val editor = field.editor as? Editor.Range ?: return
    val (first, last) = editor.range.first.toString() to editor.range.last.toString()
    val unset = editor.unset
    if (unset == null) {
        Caption(R.string.setup_range, listOf(first, last))
    } else {
        Caption(R.string.setup_range_unset, listOf(first, last, unset.toString()))
    }
}

/**
 * A number or string typed into the row, and the button that sends it. [onSet] takes the parsed
 * value: a This-board row writes the selected board, a Machine row writes both
 * (`specs/rider-ui.md` 3.7b).
 */
@Composable
internal fun Entry(field: SetupField, shown: Value?, editable: Boolean, onSet: (Value) -> Unit) {
    var text by remember(shown) { mutableStateOf(shown?.display().orEmpty()) }
    val parsed = field.parse(text)
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
        Button(onClick = { parsed?.let(onSet) }, enabled = editable && parsed != null) {
            Text(stringResource(R.string.setup_set))
        }
    }
    if (text.isNotEmpty() && parsed == null) {
        // A string too long for one write gets its own reason, so the limit is not found by timeout.
        val reason = if (field.def.type == Type.Str && !Value.Str(text).fitsOneWrite()) {
            stringResource(R.string.setup_too_long, CONFIG_VALUE_MAX)
        } else {
            stringResource(R.string.setup_invalid)
        }
        WarningText(reason, style = MaterialTheme.typography.bodySmall)

    }
}
