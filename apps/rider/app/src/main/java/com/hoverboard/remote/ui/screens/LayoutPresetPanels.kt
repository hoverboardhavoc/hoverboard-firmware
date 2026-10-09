package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPreset
import com.hoverboard.protocol.board.LayoutPresets
import com.hoverboard.remote.LayoutActions
import com.hoverboard.remote.LayoutState
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the power-latch confirmation a preset can ask for. */
const val LAYOUT_LATCH_TAG = "layout_latch"

/** Test tag on the button that stages the preset's power-latch pin. */
const val LAYOUT_LATCH_CONFIRM_TAG = "layout_latch_confirm"

/** Test tag on [preset]'s chip. */
fun layoutPresetTag(preset: LayoutPreset): String = "layout_preset_${preset.name.hashCode()}"

/**
 * The known-good layouts: one tap each, which is what keeps the pin rows below a backstop rather
 * than the way a board is configured.
 *
 * A board whose part is not known is not offered one: the preset would stage a layout that could
 * never be judged, and so never applied.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun Presets(state: LayoutState, actions: LayoutActions) {
    Panel {
        PanelTitle(R.string.layout_preset_title)
        Caption(R.string.layout_preset_body)
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (preset in LayoutPresets.ALL) {
                FilterChip(
                    selected = false,
                    onClick = { actions.stagePreset(preset) },
                    label = { Text(preset.name) },
                    enabled = state.staged != null && !state.busy && state.part != null,
                    modifier = Modifier.testTag(layoutPresetTag(preset)),
                )
            }
        }
        for (preset in LayoutPresets.ALL) {
            preset.note?.let {
                Text(
                    stringResource(R.string.layout_preset_note, preset.name, it),
                    style = MaterialTheme.typography.bodySmall,
                    color = TextSecondary,
                )
            }
        }
    }
}

/**
 * The one confirmation this screen asks for: a preset that would move the power latch.
 *
 * It states the consequence in words rather than warning in the abstract, because the consequence is
 * specific: on battery the board powers itself off at boot and only SWD recovers it.
 */
@Composable
internal fun LatchConfirmation(state: LayoutState, actions: LayoutActions) {
    val wanted = state.pendingLatch ?: return
    val held = state.staged?.let { Layout.LATCH.of(it) } ?: return
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(2.dp, AccentRed, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .testTag(LAYOUT_LATCH_TAG),
        verticalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        val type = MaterialTheme.typography
        Text(stringResource(R.string.layout_latch_title), style = type.titleMedium, color = AccentRed)
        Text(
            stringResource(R.string.layout_latch_body, pinText(held), pinText(wanted)),
            style = type.bodyMedium,
            color = TextPrimary,
        )
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = actions::confirmLatchChange, modifier = Modifier.testTag(LAYOUT_LATCH_CONFIRM_TAG)) {
                Text(stringResource(R.string.layout_latch_confirm, pinText(wanted)))
            }
            OutlinedButton(onClick = actions::cancelLatchChange) {
                Text(stringResource(R.string.layout_latch_cancel, pinText(held)))
            }
        }
    }
}

