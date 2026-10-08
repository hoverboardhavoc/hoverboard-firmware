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
import androidx.compose.runtime.Composable
import androidx.compose.runtime.setValue
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.R
import com.hoverboard.remote.SetupActions
import com.hoverboard.remote.SetupState
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.asLong
import com.hoverboard.remote.model.display
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

/** One field: its stored value, its marks, its caveat and its editor. */
@Composable
internal fun FieldRow(field: SetupField, state: SetupState, editable: Boolean, actions: SetupActions) {
    val key = field.key
    val stored = state.values[key]
    val pending = state.pending[key]
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
        pending?.let { Text(stringResource(R.string.setup_pending_mark, it.display()), color = AccentYellow) }
        if (key in state.staged) Text(stringResource(R.string.setup_staged_mark), color = AccentYellow)
        field.note?.let { Text(stringResource(it), style = MaterialTheme.typography.bodySmall, color = TextSecondary) }
        when (val editor = field.editor) {
            Editor.ReadOnly -> Unit
            is Editor.Chips -> Chips(field, editor, pending ?: stored, editable, actions)
            is Editor.Range, Editor.Generic -> Entry(field, pending ?: stored, editable, actions)
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun Chips(field: SetupField, editor: Editor.Chips, shown: Value?, editable: Boolean, actions: SetupActions) {
    val current = shown?.asLong()
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        for (choice in editor.choices) {
            FilterChip(
                selected = current == choice.value.toLong(),
                onClick = { field.parse(choice.value.toString())?.let { actions.stage(field.key, it) } },
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
        Button(onClick = { parsed?.let { actions.stage(field.key, it) } }, enabled = editable && parsed != null) {
            Text(stringResource(R.string.setup_stage))
        }
    }
    if (text.isNotEmpty() && parsed == null) {
        Text(stringResource(R.string.setup_invalid), style = MaterialTheme.typography.bodySmall, color = AccentRed)
    }
}

/** A panel's heading. */
@Composable
internal fun PanelTitle(@StringRes text: Int) {
    Text(stringResource(text), style = MaterialTheme.typography.titleSmall, color = TextPrimary)
}

/** A standing explanation under a heading or a control. */
@Composable
internal fun Caption(@StringRes text: Int) {
    Text(stringResource(text), style = MaterialTheme.typography.bodySmall, color = TextSecondary)
}
