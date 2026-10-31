package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.store.Gains
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneActions
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import kotlin.math.roundToInt
import kotlinx.coroutines.delay

/** Test tag on gain [index]'s read-back staged value. */
fun tuneStagedTag(index: Int): String = "tune_staged_$index"

/** Test tag on gain [index]'s pending value: the one being sent, shown until the board reports it. */
fun tunePendingTag(index: Int): String = "tune_pending_$index"

/** Test tag on gain [index]'s minus button. */
fun tuneStepDownTag(index: Int): String = "tune_step_down_$index"

/** Test tag on gain [index]'s plus button. */
fun tuneStepUpTag(index: Int): String = "tune_step_up_$index"

/** Test tag on gain [index]'s typed entry. */
fun tuneEntryTag(index: Int): String = "tune_entry_$index"

/** Test tag on gain [index]'s Set button. */
fun tuneSetTag(index: Int): String = "tune_set_$index"

/**
 * How long a pending value stays on screen once nothing is in flight and the board has reported
 * something else: a value it refused or echoed differently, or a send that never started. The
 * notice says which; the pending value then gives way to what the board holds.
 */
private const val PENDING_HOLD_MS = 1500L

/**
 * One gain: its name and unit, staged as last read from the board (with the converging mark) and,
 * beside it, the PENDING value while one is on its way, the slider between the minus and plus
 * buttons, the range this board accepts, the typed entry, and flash (with the unsaved mark).
 *
 * The staged number is the board's and only ever the board's. The pending one ("→ 240", in another
 * colour) is what the finger, a step button or Set has asked for: the poll pauses during a drag
 * and a write's re-read comes back a round trip later, so without it nothing would show the value
 * being sent. It goes as soon as the read-back reports that value ([PENDING_HOLD_MS] otherwise).
 *
 * Three instruments, one path: the slider over a useful span ([GAIN_SLIDER_SPANS]), the buttons
 * for a step of [GAIN_STEPS] from the pending value or else the staged one, and the entry for an
 * exact number. The buttons and the entry reach the board's whole maximum and send what a slider
 * release sends ([TuneActions.slideEnd]), so all three work armed exactly as the slider does. The
 * gp and gr rows also carry a back-off button ([BackOff]) on the same path.
 */
@Composable
internal fun GainRow(state: TuneState, index: Int, actions: TuneActions) {
    val key = state.keys[index]
    val gains = state.gains
    val staged = gains.staged[key]
    val converging = gains.converging[key]
    val max = gains.max(key)
    val enabled = state.tunable && staged != null
    var pending by remember(state.address, key) { mutableStateOf<Int?>(null) }
    var dragging by remember(state.address, key) { mutableStateOf(false) }
    LaunchedEffect(pending, dragging, staged, state.writing) {
        val p = pending ?: return@LaunchedEffect
        if (dragging || state.writing) return@LaunchedEffect
        if (p != staged) delay(PENDING_HOLD_MS)
        pending = null
    }
    val send = { v: Int ->
        pending = v
        actions.slideEnd(key, v)
    }
    Panel(modifier = Modifier.testTag(tuneRowTag(index))) {
        Header(index, staged, pending, converging != null)
        Caption(GAIN_UNITS[index])
        StepsAndSlider(
            index = index,
            max = max,
            base = pending ?: staged ?: Gains.MIN,
            enabled = enabled,
            onStep = send,
            onDrag = {
                dragging = true
                pending = it
                actions.slide(key, it)
            },
            onRelease = {
                dragging = false
                pending?.let { actions.slideEnd(key, it) }
            },
        )
        BackOff(index, staged, max, enabled, send)
        LiveEntry(
            staged = staged,
            range = Gains.MIN..max,
            enabled = enabled,
            entryTag = tuneEntryTag(index),
            setTag = tuneSetTag(index),
            onSet = send,
        )
        FlashLine(gains, key)
        if (converging != null) {
            Text(
                stringResource(R.string.tune_converging_row, seconds(converging)),
                style = MaterialTheme.typography.bodySmall,
                color = Primary,
            )
        }
    }
}

/**
 * The minus button, the slider and the plus button, and under them the board's range and, when
 * the slider stops short of it, where the slider ends. A step goes from [base] (the pending value,
 * else the staged one) and is held inside the board's [max]; each button is off at its end.
 */
@Composable
private fun StepsAndSlider(
    index: Int,
    max: Int,
    base: Int,
    enabled: Boolean,
    onStep: (Int) -> Unit,
    onDrag: (Int) -> Unit,
    onRelease: () -> Unit,
) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        OutlinedButton(
            onClick = { onStep(gainStepped(base, -GAIN_STEPS[index], max)) },
            enabled = enabled && base > Gains.MIN,
            modifier = Modifier.testTag(tuneStepDownTag(index)),
        ) { Text("-") }
        GainSlider(
            index = index,
            top = gainSliderTop(index, max),
            shown = base,
            enabled = enabled,
            onDrag = onDrag,
            onRelease = onRelease,
            modifier = Modifier.weight(1f),
        )
        OutlinedButton(
            onClick = { onStep(gainStepped(base, GAIN_STEPS[index], max)) },
            enabled = enabled && base < max,
            modifier = Modifier.testTag(tuneStepUpTag(index)),
        ) { Text("+") }
    }
    Text(
        stringResource(R.string.tune_range, max.toString()),
        style = MaterialTheme.typography.bodySmall,
        color = FgMuted,
    )
    val top = gainSliderTop(index, max)
    if (top < max) Caption(R.string.tune_slider_span, listOf(top.toString()))
}

/**
 * The gain's name, the staged value the board reported with its converging mark, and the pending
 * value beside them while it differs.
 */
@Composable
private fun Header(index: Int, staged: Int?, pending: Int?, converging: Boolean) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        Text(GAIN_NAMES[index], style = MaterialTheme.typography.titleMedium, color = Fg)
        Column(modifier = Modifier.weight(1f)) {
            Text(
                stringResource(R.string.tune_staged),
                style = MaterialTheme.typography.labelSmall,
                color = FgMuted,
            )
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    staged?.toString() ?: stringResource(R.string.tune_unread),
                    style = MaterialTheme.typography.titleLarge,
                    color = Fg,
                    modifier = Modifier.testTag(tuneStagedTag(index)),
                )
                if (converging) {
                    Text(
                        " " + stringResource(R.string.tune_converging_mark),
                        style = MaterialTheme.typography.titleLarge,
                        color = Primary,
                    )
                }
                if (pending != null && pending != staged) {
                    Text(
                        stringResource(R.string.tune_pending, pending.toString()),
                        style = MaterialTheme.typography.titleLarge,
                        color = Primary,
                        modifier = Modifier
                            .padding(start = 12.dp)
                            .testTag(tunePendingTag(index)),
                    )
                }
            }
        }
    }
}

/**
 * A gain's slider, from [Gains.MIN] to [top]: the gain's useful span ([GAIN_SLIDER_SPANS]), never
 * past the maximum this board accepts (`specs/remote-ui.md` section 3.3a: `CONTROL_GAIN_MAX` as read
 * from the board, [Gains.DEFAULT_MAX] while unread). A staged value above [top] parks the thumb at
 * the end; the readout still says what it is.
 *
 * The thumb sits at [shown], the pending value while there is one and the staged value otherwise,
 * so it follows the finger during a drag and does not jump back before the read-back arrives.
 * Each drag sample goes to [onDrag], which sends on a cadence; the release goes to [onRelease],
 * which sends the exact position once.
 */
@Composable
private fun GainSlider(
    index: Int,
    top: Int,
    shown: Int,
    enabled: Boolean,
    onDrag: (Int) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Slider(
        value = shown.coerceIn(Gains.MIN, top).toFloat(),
        valueRange = Gains.MIN.toFloat()..top.toFloat(),
        enabled = enabled,
        onValueChange = { onDrag(it.roundToInt()) },
        onValueChangeFinished = onRelease,
        modifier = modifier
            .fillMaxWidth()
            .testTag(tuneSliderTag(index)),
    )
}
