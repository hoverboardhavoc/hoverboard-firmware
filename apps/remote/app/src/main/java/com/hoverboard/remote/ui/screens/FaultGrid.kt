package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.BottomSheetDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ModalBottomSheet
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.rememberModalBottomSheetState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.SlotSide
import com.hoverboard.remote.model.lampColumns
import com.hoverboard.remote.ui.theme.BgCode
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.OnWarning
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Warning
import com.hoverboard.remote.ui.theme.WarningGlyph
import kotlinx.coroutines.launch

/** Test tag on the fault grid. */
const val FAULT_GRID_TAG = "fault_grid"

/** Test tag on [lamp]'s cell in the grid. */
fun lampTag(lamp: FaultLamp): String = "lamp_${lamp.name.lowercase()}"

/** Test tag on the open lamp sheet. */
const val LAMP_SHEET_TAG = "lamp_sheet"

/**
 * The fault lamp grid (`specs/remote-ui.md` 3.1b): one lamp per cause that stops a board arming, for
 * the whole machine, with a mark per board slot inside each, so both boards' faults show at once.
 * Which lamps are in it is [UiState.lamps], and nothing here decides that.
 *
 * Four columns, or five when nine lamps are shown, so it is never more than two rows. A lamp is lit
 * while its bit is set on any reporting board (or the app holds a latch lit), and every shown lamp
 * is lit with every mark during the lamp test. Tapping a lamp, lit or dark, opens its sheet.
 */
@Composable
internal fun FaultGrid(state: UiState, onOpen: (FaultLamp) -> Unit, modifier: Modifier = Modifier) {
    val lamps = state.lamps
    val columns = lampColumns(lamps.size)
    Column(modifier = modifier.testTag(FAULT_GRID_TAG), verticalArrangement = Arrangement.spacedBy(LAMP_GAP)) {
        for (row in lamps.chunked(columns)) {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(LAMP_GAP)) {
                for (lamp in row) Lamp(state, lamp, onClick = { onOpen(lamp) }, modifier = Modifier.weight(1f))
                repeat(columns - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}

/** Whether [lamp]'s bit is set on the board at [address] as the grid draws it. */
private fun UiState.litOn(lamp: FaultLamp, address: Int): Boolean = (lampWords[address] ?: 0) and lamp.bit != 0

/** Whether [lamp] is lit by any reporting board, the lamp test aside. */
internal fun UiState.lit(lamp: FaultLamp): Boolean = lampWords.values.any { it and lamp.bit != 0 }

@Composable
private fun Lamp(state: UiState, lamp: FaultLamp, onClick: () -> Unit, modifier: Modifier = Modifier) {
    val on = state.lampTest || state.lit(lamp)
    val shape = RoundedCornerShape(LAMP_CORNER)
    Column(
        modifier = modifier
            .height(LAMP_HEIGHT)
            .clip(shape)
            .background(if (on) Warning else BgCode)
            .border(1.dp, if (on) Warning else Border, shape)
            .clickable(onClick = onClick)
            .testTag(lampTag(lamp)),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            if (on) {
                WarningGlyph(11.dp, color = OnWarning)
                Spacer(Modifier.width(4.dp))
            }
            Text(
                lamp.label,
                style = ride(12.sp, FontWeight.Bold, letterSpacing = 0.8.sp),
                color = if (on) OnWarning else FgFaint,
            )
        }
        Spacer(Modifier.height(5.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            for (slot in state.slots.all) {
                val mark = state.lampTest || state.litOn(lamp, slot.address)
                Text(
                    slotLabel(slot),
                    style = ride(9.sp, if (mark && on) FontWeight.Bold else FontWeight.Normal),
                    color = when {
                        !on -> FgFaint.copy(alpha = DARK_MARK_ALPHA)
                        mark -> OnWarning
                        else -> OnWarning.copy(alpha = UNLIT_MARK_ALPHA)
                    },
                )
            }
        }
    }
}

/**
 * The boards [lamp] is lit on, named for its sheet: "Left board", "Left and right boards", or by
 * [nodeLabel] for a board not placed by its side; "Not lit" when it is lit nowhere. It follows the
 * live words while the sheet is open.
 */
@Composable
internal fun litWhere(state: UiState, lamp: FaultLamp): String {
    val lit = state.lampWords.filterValues { it and lamp.bit != 0 }.keys.sorted()
    if (lit.isEmpty()) return stringResource(R.string.lamp_not_lit)
    val slots = lit.map { state.slots.of(it) }
    if (lit.size == 2 && slots.all { it?.bySide == true }) return stringResource(R.string.lamp_boards_both)
    return lit.map { boardName(state, it, inline = false) }
        .reduce { a, b -> stringResource(R.string.lamp_boards_and, a, b) }
}

/**
 * The board at [address] named in a sentence: by its side when its slot is placed by side ("Left
 * board", or "left board" [inline] in a statement), else by [nodeLabel].
 */
@Composable
internal fun boardName(state: UiState, address: Int, inline: Boolean): String {
    val slot = state.slots.of(address)
    return when {
        slot == null || !slot.bySide -> state.nodeAt(address)?.let { nodeLabel(it) } ?: boardHex(address)
        slot.side == SlotSide.LEFT ->
            stringResource(if (inline) R.string.ride_board_left_inline else R.string.lamp_board_left)
        else -> stringResource(if (inline) R.string.ride_board_right_inline else R.string.lamp_board_right)
    }
}

/**
 * A lamp's sheet (`specs/remote-ui.md` 3.1b): a Material 3 bottom sheet, opened full height, with
 * the title, where it is lit and its code, and the three headed paragraphs of what the firmware
 * does. It closes on Close, a scrim tap, a drag down and system back, all of which reach
 * [onDismiss]. The caller releases the drive before opening it: the scrim covers the pad, and a pad
 * that loses its pointer never sees its finger-up.
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
internal fun LampSheet(state: UiState, lamp: FaultLamp, onDismiss: () -> Unit, modifier: Modifier = Modifier) {
    val sheet = rememberModalBottomSheetState(skipPartiallyExpanded = true)
    val scope = rememberCoroutineScope()
    ModalBottomSheet(
        onDismissRequest = onDismiss,
        sheetState = sheet,
        containerColor = BgElevated,
        scrimColor = Color.Black.copy(alpha = SCRIM_ALPHA),
        dragHandle = { BottomSheetDefaults.DragHandle(color = FgMuted) },
        modifier = modifier.testTag(LAMP_SHEET_TAG),
    ) {
        Column(Modifier.padding(horizontal = 24.dp).padding(bottom = 16.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                if (state.lit(lamp)) {
                    WarningGlyph(20.dp)
                    Spacer(Modifier.width(12.dp))
                }
                Text(stringResource(lamp.title), style = ride(22.sp, FontWeight.Bold), color = Fg)
            }
            Text(
                stringResource(R.string.lamp_where, litWhere(state, lamp), lamp.code),
                style = ride(14.sp),
                color = FgMuted,
            )
            Spacer(Modifier.height(20.dp))
            for ((head, body) in listOf(
                R.string.lamp_head_when to lamp.whenShows,
                R.string.lamp_head_blocks to lamp.blocks,
                R.string.lamp_head_fix to lamp.fix,
            )) {
                Text(
                    stringResource(head).uppercase(),
                    style = ride(11.sp, FontWeight.Bold, letterSpacing = 1.2.sp),
                    color = Primary,
                )
                Spacer(Modifier.height(4.dp))
                Text(stringResource(body), style = ride(15.sp).copy(lineHeight = 21.sp), color = Fg)
                Spacer(Modifier.height(16.dp))
            }
            TextButton(
                onClick = { scope.launch { sheet.hide() }.invokeOnCompletion { onDismiss() } },
                modifier = Modifier.align(Alignment.End),
            ) {
                Text(stringResource(R.string.lamp_close))
            }
        }
    }
}

private val LAMP_HEIGHT = 52.dp
private val LAMP_GAP = 8.dp
private val LAMP_CORNER = 10.dp
private const val DARK_MARK_ALPHA = 0.5f
private const val UNLIT_MARK_ALPHA = 0.35f
private const val SCRIM_ALPHA = 0.6f
