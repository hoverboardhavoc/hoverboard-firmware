package com.hoverboard.remote.ui.components

import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary

/**
 * The DIFFERENTIAL and RIGID modes' two-axis stick (`specs/rider-ui.md` 3.2), drawn as the 8a pad
 * (3.1): one pad producing `(v, s)`, forward/back and turn. Vertically it is the [ThrottlePad]'s
 * mapping, centre-zero with the top edge full forward; horizontally the centre line is no turn and
 * the right edge is a full turn to the right. Both axes are read by [Throttle], so there is one
 * definition of where centre is.
 *
 * The surface is the throttle pad's ([PadFrame], [drawPadSurface]) with the no-turn line drawn as
 * the zero line is, and the thumb a disc, [Primary] while live and [FgMuted] otherwise, drawn
 * while a finger is down or a demand is out.
 *
 * Reports raw `(x, y, width, height)` on press and move, so the mapping and the mix stay in the
 * ViewModel and the model where they are testable off-device:
 *  - [onMove] on press-down and every move while held.
 *  - [onRelease] on finger-up or cancel, which zeroes both axes.
 *
 * The gesture handling is the throttle pad's, to the letter: engage on touch-down before any
 * movement (the deadman fires first), claim the drag so no parent can steal it mid-drive, and end
 * on the release of the last pointer.
 *
 * @param speed the commanded forward demand, used to draw the thumb.
 * @param steer the commanded turn, used to draw the thumb.
 * @param engaged whether a touch is currently held.
 * @param enabled whether the pad accepts touches at all, which is also whether it is live.
 * @param blocked whether a board the arm would arm reports an arm-blocking fault: the lines dim.
 */
@Composable
fun JoystickPad(
    speed: Int,
    steer: Int,
    engaged: Boolean,
    enabled: Boolean,
    blocked: Boolean,
    onMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
    maxSpeed: Int = Throttle.MAX_SPEED,
) {
    PadFrame(
        blocked = blocked,
        modifier = modifier.testTag(JOYSTICK_TAG),
        inputModifier = Modifier.pointerInput(enabled) {
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
        },
        draw = {
            drawPadSurface(live = enabled, blocked = blocked, crossLine = true)
            if (engaged || speed != 0 || steer != 0) drawJoystickThumb(speed, steer, maxSpeed, live = enabled)
        },
    )
}

private fun DrawScope.drawJoystickThumb(speed: Int, steer: Int, maxSpeed: Int, live: Boolean) {
    // A starved canvas draws nothing rather than throwing, for the reason ThrottlePad's draw says:
    // a lost frame is recoverable and an exception inside a draw lambda is not.
    val diameter = THUMB_DIAMETER.toPx()
    if (size.width <= diameter || size.height <= diameter || maxSpeed == 0) return
    val zeroY = Throttle.ZERO_FRACTION * size.height
    val zeroX = Throttle.ZERO_FRACTION * size.width
    val v = (speed.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    val s = (steer.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    val radius = diameter / 2f
    val centre = Offset(
        x = (zeroX + s * HALF_SPAN * size.width).coerceIn(radius, size.width - radius),
        y = (zeroY - v * HALF_SPAN * size.height).coerceIn(radius, size.height - radius),
    )
    drawCircle(color = if (live) Primary else FgMuted, radius = radius, center = centre)
}

private val THUMB_DIAMETER = 48.dp
private const val HALF_SPAN = 0.5f // centre -> edge, on both axes (mirrors Throttle's half span)

const val JOYSTICK_TAG = "joystick_pad"
