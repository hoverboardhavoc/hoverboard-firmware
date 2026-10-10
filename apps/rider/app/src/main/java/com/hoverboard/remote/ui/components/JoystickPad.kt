package com.hoverboard.remote.ui.components

import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.ui.theme.ARMED_OUTLINE
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.ThrottleForward
import com.hoverboard.remote.ui.theme.ThrottleReverse
import com.hoverboard.remote.ui.theme.ThrottleTrack
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * The DIFFERENTIAL mode's two-axis stick (`specs/rider-ui.md` 3.2): one pad producing `(v, s)`,
 * forward/back and turn. Vertically it is the [ThrottlePad]'s mapping, centre-zero with the top edge
 * full forward; horizontally the centre line is no turn and the right edge is a full turn to the
 * right. Both axes are read by [Throttle], so there is one definition of where centre is.
 *
 * Reports raw `(x, y, width, height)` on press and move, so the mapping and the mix stay in the
 * ViewModel and the model where they are testable off-device:
 *  - [onMove] on press-down and every move while held.
 *  - [onRelease] on finger-up or cancel, which zeroes both axes.
 *
 * The gesture handling is the throttle pad's, deliberately to the letter: engage on touch-down
 * before any movement (the deadman fires first), claim the drag so no parent can steal it mid-drive,
 * and end on the release of the last pointer.
 *
 * @param speed the commanded forward demand, used to draw the thumb.
 * @param steer the commanded turn, used to draw the thumb.
 * @param engaged whether a touch is currently held. A held stick draws a filled thumb and a released
 *   one draws a ring, which is the pad's own statement of whether it has a finger on it.
 * @param enabled whether the pad accepts touches at all.
 * @param armed whether the machine's motors are live, drawn as the red outline that the ride screen's
 *   big surfaces carry instead of an armed banner ([com.hoverboard.remote.ui.screens.ControlScreen]).
 */
@Composable
fun JoystickPad(
    speed: Int,
    steer: Int,
    engaged: Boolean,
    enabled: Boolean,
    armed: Boolean,
    onMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
    maxSpeed: Int = Throttle.MAX_SPEED,
) {
    val shape = RoundedCornerShape(24.dp)
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(shape)
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                awaitEachGesture {
                    val down = awaitFirstDown(requireUnconsumed = false)
                    down.consume()
                    onMove(down.position.x, down.position.y, size.width.toFloat(), size.height.toFloat())
                    do {
                        val event = awaitPointerEvent()
                        val change = event.changes.firstOrNull { it.id == down.id }
                            ?: event.changes.firstOrNull()
                        if (change != null && change.pressed) {
                            change.consume() // claim the drag so the system/parents don't steal it
                            onMove(
                                change.position.x,
                                change.position.y,
                                size.width.toFloat(),
                                size.height.toFloat(),
                            )
                        }
                    } while (event.changes.any { it.pressed })
                    onRelease()
                }
            }
            .drawBehind { drawJoystick(speed = speed, steer = steer, maxSpeed = maxSpeed, held = engaged) }
            .border(ARMED_OUTLINE, if (armed) AccentRed else Color.Transparent, shape)
            .testTag(JOYSTICK_TAG),
    )
}

private fun DrawScope.drawJoystick(speed: Int, steer: Int, maxSpeed: Int, held: Boolean) {
    // A starved canvas draws nothing rather than throwing, for the reason ThrottlePad's draw says:
    // a lost frame is recoverable and an exception inside a draw lambda is not.
    if (size.width <= THUMB_DIAMETER || size.height <= THUMB_DIAMETER) return
    val w = size.width
    val h = size.height
    val zeroY = Throttle.ZERO_FRACTION * h
    val zeroX = Throttle.ZERO_FRACTION * w

    drawRect(color = ThrottleTrack)
    drawRect(
        color = ThrottleForward.copy(alpha = ZONE_ALPHA),
        topLeft = Offset(0f, 0f),
        size = Size(w, zeroY),
    )
    drawRect(
        color = ThrottleReverse.copy(alpha = ZONE_ALPHA),
        topLeft = Offset(0f, zeroY),
        size = Size(w, h - zeroY),
    )

    // The two rest lines: no travel across, no turn down.
    drawLine(color = ZeroLine, start = Offset(0f, zeroY), end = Offset(w, zeroY), strokeWidth = LINE_WIDTH * 2)
    drawLine(color = ZeroLine, start = Offset(zeroX, 0f), end = Offset(zeroX, h), strokeWidth = LINE_WIDTH)

    if (maxSpeed == 0) return
    val v = (speed.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    val s = (steer.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    val radius = THUMB_DIAMETER / 2f
    val centre = Offset(
        x = (zeroX + s * HALF_SPAN * w).coerceIn(radius, w - radius),
        y = (zeroY - v * HALF_SPAN * h).coerceIn(radius, h - radius),
    )
    val colour = if (speed >= 0) ThrottleForward else ThrottleReverse
    if (held) {
        drawCircle(color = colour, radius = radius, center = centre)
    } else {
        drawCircle(color = colour, radius = radius, center = centre, style = Stroke(width = LINE_WIDTH))
    }
}

private const val LINE_WIDTH = 3f
private const val THUMB_DIAMETER = 56f
private const val HALF_SPAN = 0.5f // centre -> edge, on both axes (mirrors Throttle's half span)
private const val ZONE_ALPHA = 0.12f

const val JOYSTICK_TAG = "joystick_pad"
