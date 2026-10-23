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
import androidx.compose.runtime.setValue
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.R
import com.hoverboard.remote.FieldWrite
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.WriteState
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.TakesEffect
import com.hoverboard.remote.model.asLong
import com.hoverboard.remote.model.display
import com.hoverboard.remote.model.fitsOneWrite
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

@Composable
internal fun Section(title: String, content: @Composable ColumnScope.() -> Unit) {
    Spacer(modifier = Modifier.height(4.dp))
    Text(title, style = MaterialTheme.typography.titleLarge, color = TextPrimary)
    Column(verticalArrangement = Arrangement.spacedBy(12.dp), content = content)
}

@Composable
internal fun Panel(modifier: Modifier = Modifier, content: @Composable ColumnScope.() -> Unit) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .background(PanelSurface, RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp),
        content = content,
    )
}

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
        Text(stringResource(field.label), style = MaterialTheme.typography.titleSmall, color = TextPrimary)
        Text(
            text = when {
                stored != null -> stringResource(R.string.setup_stored, stored.display())
                key in state.unread -> stringResource(R.string.setup_unread)
                else -> stringResource(R.string.setup_not_read_yet)
            },
            color = TextSecondary,
        )
        if (field.showDefault) {
            Text(stringResource(R.string.setup_default, field.def.default.display()), color = TextSecondary)
        }
        // Every row says so while the board is armed, rather than one banner saying it once: the
        // board refuses every config write then, and the refusal belongs to each field.
        if (armed && field.takesEffect != null) {
            Text(stringResource(R.string.setup_row_armed), color = AccentRed)
        }
        write?.let { WriteResult(field, it, editable, actions) }
        TakeEffect(field)
        field.note?.let { Caption(it, field.noteArgs) }
        val shown = write?.takeIf { it.sending }?.want ?: stored
        when (val editor = field.editor) {
            Editor.ReadOnly, is Editor.Flow -> Unit
            is Editor.Chips -> Chips(field, editor, shown, editable, actions)
            is Editor.Range, Editor.Generic -> Entry(field, shown, editable, actions)
        }
    }
}

/** What came of this row's write ([writeStatus]), and a retry for the ones that can be sent again. */
@Composable
private fun WriteResult(field: SetupField, write: FieldWrite, editable: Boolean, actions: SetupActions) {
    val (body, colour) = writeStatus(field, write)
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text(body, modifier = Modifier.weight(1f), color = colour)
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
 * What this row's write said, and the colour to say it in, for a row that renders it inline and for
 * the Machine section's two boards.
 *
 * A [TakesEffect.RESTART] field still marked written has not been taken up: the model drops that
 * record when the boot tag on the wire shows the board restarted, so its presence IS the statement
 * that no restart has happened. Nothing asks the operator to assert one.
 */
@Composable
internal fun writeStatus(field: SetupField, write: FieldWrite): Pair<String, Color> {
    val restartDue = field.takesEffect == TakesEffect.RESTART
    val (text, colour) = when (write.state) {
        // The same word the screen-level indicator uses: one string for one fact.
        WriteState.Sending -> R.string.setup_writing to TextSecondary
        WriteState.Written ->
            (if (restartDue) R.string.setup_write_awaiting_restart else R.string.setup_write_written) to AccentGreen
        else -> writeFailureText(write.state) to AccentYellow
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
        Text(stringResource(R.string.setup_choice_unknown, current.toString()), color = AccentYellow)
    }
}

@Composable
internal fun Entry(field: SetupField, shown: Value?, editable: Boolean, actions: SetupActions) {
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
        Button(onClick = { parsed?.let { actions.edit(field.key, it) } }, enabled = editable && parsed != null) {
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
        Text(reason, style = MaterialTheme.typography.bodySmall, color = AccentRed)
    }
}

/** A panel's heading. */
@Composable
internal fun PanelTitle(@StringRes text: Int) {
    Text(stringResource(text), style = MaterialTheme.typography.titleSmall, color = TextPrimary)
}

/** A standing explanation under a heading or a control. */
@Composable
internal fun Caption(@StringRes text: Int, args: List<Any> = emptyList()) {
    val body = if (args.isEmpty()) stringResource(text) else stringResource(text, *args.toTypedArray())
    Text(body, style = MaterialTheme.typography.bodySmall, color = TextSecondary)
}
