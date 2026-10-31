package com.hoverboard.remote.ui.components

import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.ui.theme.Atkinson
import com.hoverboard.remote.ui.theme.BgCode
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary

/**
 * Deadman throttle pad (SPEC §9), drawn as the 8a pad (`specs/remote-ui.md` 3.1). A vertical pad
 * active across its WHOLE height: touch anywhere to drive, release to stop. The centre line is
 * zero; the top half is forward (top edge = MAX forward) and the bottom half reverse, symmetric.
 *
 * The forward half is tinted [Primary] while the pad is live (connected and armed), and the thumb,
 * a full-width bar at the demand's height, is [Primary] while live and [FgMuted] otherwise. It is
 * drawn while a finger is down or a demand is out, so a pad at rest shows its zero line.
 *
 * Reports raw (y, height) on press/move so the §9 math + safety live in the ViewModel:
 *  - [onMove] is called on press-down and every move while held.
 *  - [onRelease] is called on finger-up (or cancel) -> the ViewModel commands 0.
 *
 * @param speed current commanded wire speed, used only to draw the thumb position.
 * @param engaged whether a touch is currently held in the engage zone.
 * @param enabled whether the pad accepts touches at all, which is also whether it is live.
 * @param blocked whether a board the arm would arm reports an arm-blocking fault: the zero line and
 *   the direction words dim to [BLOCKED_ALPHA].
 */
@Composable
fun ThrottlePad(
    speed: Int,
    engaged: Boolean,
    enabled: Boolean,
    blocked: Boolean,
    onMove: (y: Float, height: Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
    maxSpeed: Int = Throttle.MAX_SPEED,
) {
    PadFrame(
        blocked = blocked,
        modifier = modifier.testTag(THROTTLE_TAG),
        inputModifier = Modifier
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                awaitEachGesture {
                    // Engage on touch-down (the deadman fires before any movement).
                    val down = awaitFirstDown(requireUnconsumed = false)
                    down.consume()
                    onMove(down.position.y, size.height.toFloat())
                    do {
                        val event = awaitPointerEvent()
                        val change = event.changes.firstOrNull { it.id == down.id }
                            ?: event.changes.firstOrNull()
                        if (change != null && change.pressed) {
                            change.consume() // claim the drag so the system/parents don't steal it
                            onMove(change.position.y, size.height.toFloat())
                        }
                    } while (event.changes.any { it.pressed })
                    onRelease()
                }
            },
        draw = {
            drawPadSurface(live = enabled, blocked = blocked, crossLine = false)
            if (engaged || speed != 0) drawThrottleThumb(speed, maxSpeed, live = enabled)
        },
    )
}

/**
 * The pad's outline and its direction words over a surface [draw] paints, clipped to the pad's
 * corners: the frame both pads share, so the throttle and the joystick are one drawing with a
 * different thumb. [inputModifier] is the pad's gesture handling, inside the clip.
 */
@Composable
internal fun PadFrame(
    blocked: Boolean,
    draw: DrawScope.() -> Unit,
    modifier: Modifier = Modifier,
    inputModifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(PAD_CORNER)
    val words = FgFaint.copy(alpha = if (blocked) BLOCKED_ALPHA else 1f)
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(shape)
            .then(inputModifier)
            .drawBehind(draw)
            .border(1.dp, Border, shape),
    ) {
        Text(
            stringResource(R.string.pad_forward),
            style = PAD_WORD,
            color = words,
            modifier = Modifier.align(Alignment.TopCenter).padding(top = 10.dp),
        )
        Text(
            stringResource(R.string.pad_reverse),
            style = PAD_WORD,
            color = words,
            modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 10.dp),
        )
    }
}


/**
 * The pad's surface: [BgCode], the forward half tinted [Primary] at 12 percent while [live] and
 * white at 3 percent otherwise, and the zero line; with [crossLine], the joystick's vertical
 * no-turn line too. While [blocked] the lines draw at [BLOCKED_ALPHA].
 */
internal fun DrawScope.drawPadSurface(live: Boolean, blocked: Boolean, crossLine: Boolean) {
    drawRect(BgCode)
    val zeroY = Throttle.ZERO_FRACTION * size.height
    drawRect(
        color = if (live) Primary.copy(alpha = LIVE_TINT_ALPHA) else Color.White.copy(alpha = IDLE_TINT_ALPHA),
        size = Size(size.width, zeroY),
    )
    val line = FgMuted.copy(alpha = ZERO_LINE_ALPHA * if (blocked) BLOCKED_ALPHA else 1f)
    val stroke = ZERO_LINE_WIDTH.toPx()
    drawLine(line, Offset(0f, zeroY), Offset(size.width, zeroY), stroke)
    if (crossLine) {
        val zeroX = Throttle.ZERO_FRACTION * size.width
        drawLine(line, Offset(zeroX, 0f), Offset(zeroX, size.height), stroke)
    }
}

private fun DrawScope.drawThrottleThumb(speed: Int, maxSpeed: Int, live: Boolean) {
    // A degenerate canvas draws nothing rather than throwing: a Column can starve this of height
    // (measured on a OnePlus 8, where the thumb's span came out negative and crashed the process
    // from inside draw). Losing a frame is recoverable; an exception in a draw lambda is not.
    val thumb = THUMB_HEIGHT.toPx()
    if (size.width <= 0f || size.height <= thumb || maxSpeed == 0) return
    val zeroY = Throttle.ZERO_FRACTION * size.height
    val t = (speed.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    val thumbY = zeroY - t * (THROTTLE_HALF_SPAN * size.height)
    drawRoundRect(
        color = if (live) Primary else FgMuted,
        topLeft = Offset(0f, (thumbY - thumb / 2).coerceIn(0f, size.height - thumb)),
        size = Size(size.width, thumb),
        cornerRadius = CornerRadius(thumb / 2),
    )
}

/** The dimming of the pad's lines and words while a board the arm would arm reports a fault. */
internal const val BLOCKED_ALPHA = 0.45f

private const val LIVE_TINT_ALPHA = 0.12f
private const val IDLE_TINT_ALPHA = 0.03f
private const val ZERO_LINE_ALPHA = 0.6f
private const val THROTTLE_HALF_SPAN = 0.5f // centre -> edge (mirrors Throttle.ACTIVE_HALF_SPAN)
private val PAD_CORNER = 24.dp
private val ZERO_LINE_WIDTH = 2.dp
private val THUMB_HEIGHT = 24.dp
private val PAD_WORD = TextStyle(fontFamily = Atkinson, fontSize = 11.sp)

const val THROTTLE_TAG = "throttle_pad"
