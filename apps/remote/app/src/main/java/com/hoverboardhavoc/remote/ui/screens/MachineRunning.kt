package com.hoverboardhavoc.remote.ui.screens

import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.UiState
import com.hoverboardhavoc.remote.model.SetupFields
import com.hoverboardhavoc.remote.model.TelemetryUi
import com.hoverboardhavoc.remote.model.display
import com.hoverboardhavoc.remote.ui.theme.CautionText
import com.hoverboardhavoc.remote.ui.theme.Tone
import com.hoverboardhavoc.remote.ui.theme.WarningText

/** Test tag on the list of machine settings that differ, shown while a board reports `EV_MACHINE`. */
const val MACHINE_DIFFER_TAG = "machine_differ"

/** Test tag on the "takes effect at the next arm" line of the motor controller at [address]. */
fun statusRearmTag(address: Int): String = "status_rearm_${boardHex(address)}"

/** Test tag on the "restart the board" line of the motor controller at [address]. */
fun statusRebootTag(address: Int): String = "status_reboot_${boardHex(address)}"

/**
 * Whether the board at [address] runs what it stores (`specs/machine-settings.md` section 5), from
 * its own flags: `FLAG_REARM` takes effect at the next arm, `FLAG_REBOOT` when the board restarts.
 * Nothing while it runs what it stores.
 */
@Composable
internal fun RunLines(address: Int, telemetry: TelemetryUi) {
    val small = MaterialTheme.typography.bodySmall
    if (telemetry.rearm) {
        CautionText(
            stringResource(R.string.status_rearm),
            style = small,
            modifier = Modifier.testTag(statusRearmTag(address)),
        )
    }
    if (telemetry.reboot) {
        CautionText(
            stringResource(R.string.status_reboot),
            style = small,
            modifier = Modifier.testTag(statusRebootTag(address)),
        )
    }
}

/**
 * What a board's `EV_MACHINE` comes to (`specs/machine-settings.md` section 7): the machine settings
 * whose copies differ, one line each naming every board's value, as `machine diff` lists them; or,
 * while none read differs, that the other board may be off or unheard.
 */
@Composable
internal fun MachineDifferences(state: UiState) {
    val small = MaterialTheme.typography.bodySmall
    val tag = Modifier.testTag(MACHINE_DIFFER_TAG)
    if (state.machineDifferences.isEmpty()) {
        WarningText(stringResource(R.string.setup_machine_differ_unread), modifier = tag)
        return
    }
    WarningText(stringResource(R.string.setup_machine_differ_title), modifier = tag)
    for ((key, values) in state.machineDifferences) {
        val name = SetupFields.forKey(key)?.let { stringResource(it.label) }
            ?: stringResource(R.string.setup_machine_differ_raw, hex(key.fieldId), key.index)
        val each = values.toSortedMap().map { (address, v) ->
            "${state.nodeAt(address)?.let { nodeLabel(it) } ?: boardHex(address)} ${v.display()}"
        }.joinToString(", ")
        WarningText(stringResource(R.string.setup_machine_differ_line, name, each), style = small)
    }
}

/**
 * A board that does not yet run what it stores, from its own flags (`FLAG_REBOOT`, `FLAG_REARM`,
 * `specs/machine-settings.md` section 5): restart it, or the setting takes effect at the next arm.
 */
@Composable
internal fun runStatements(state: UiState): List<Statement> = buildList {
    for (board in state.reporting) {
        val t = state.telemetryOf(board) ?: continue
        val name = boardName(state, board, inline = false)
        if (t.reboot) add(Statement(stringResource(R.string.ride_reboot, name), Tone.CAUTION))
        if (t.rearm) add(Statement(stringResource(R.string.ride_rearm, name), Tone.CAUTION))
    }
}
