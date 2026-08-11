package com.hoverboard.remote.ui.components

import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.BiasAlignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.ui.theme.ThrottleForward
import com.hoverboard.remote.ui.theme.ThrottleReverse
import com.hoverboard.remote.ui.theme.ThrottleTrack
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Deadman throttle pad (SPEC §9). A vertical pad active across its WHOLE height: touch
 * anywhere to drive, release to stop. The centre line (0.5 H) is zero/rest; the top half is
 * forward (top edge = MAX forward) and the bottom half is reverse (bottom edge = MAX reverse),
 * symmetric. The forward zone is tinted green and the reverse zone red, with the status label
 * on the centre/rest line, so it's clear where "stop" is and which way is forward/reverse.
 *
 * Reports raw (y, height) on press/move so the §9 math + safety live in the ViewModel:
 *  - [onMove] is called on press-down and every move while held.
 *  - [onRelease] is called on finger-up (or cancel) -> the ViewModel commands 0.
 *
 * @param speed current commanded wire speed, used only to draw the thumb position.
 * @param engaged whether a touch is currently held in the engage zone.
 */
@Composable
fun ThrottlePad(
    speed: Int,
    engaged: Boolean,
    enabled: Boolean,
    onMove: (y: Float, height: Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
    maxSpeed: Int = Throttle.MAX_SPEED,
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(24.dp))
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
            }
            .drawBehind { drawThrottle(speed = speed, maxSpeed = maxSpeed) }
            .testTag(THROTTLE_TAG),
    ) {
        Text(
            text = if (engaged) {
                stringResource(R.string.throttle_engaged)
            } else {
                stringResource(R.string.throttle_idle)
            },
            style = MaterialTheme.typography.titleMedium,
            color = if (engaged) ThrottleForward else ZeroLine,
            // Sit on the zero/rest line (0.75 H), not the pad centre.
            modifier = Modifier
                .align(BiasAlignment(0f, fractionToBias(Throttle.ZERO_FRACTION)))
                .padding(16.dp),
        )
    }
}

private fun DrawScope.drawThrottle(speed: Int, maxSpeed: Int) {
    val h = size.height
    val w = size.width

    // Track background.
    drawRect(color = ThrottleTrack)

    val zeroY = Throttle.ZERO_FRACTION * h // 0.5 H, the zero / rest line (pad centre)

    // The whole pad is active: tint forward (green) above the centre, reverse (red) below.
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

    // The zero / rest line at the centre (0.5 H).
    drawLine(
        color = ZeroLine,
        start = Offset(0f, zeroY),
        end = Offset(w, zeroY),
        strokeWidth = LINE_WIDTH * 2,
    )

    // Thumb: a band centred on the current commanded position, coloured by direction.
    if (maxSpeed == 0) return
    val t = (speed.toFloat() / maxSpeed).coerceIn(-1f, 1f)
    // Invert §9 mapping: y = zeroY - t * (0.25 H).
    val thumbY = zeroY - t * (THROTTLE_HALF_SPAN * h)
    val thumbColor = if (speed >= 0) ThrottleForward else ThrottleReverse
    val bandHalf = THUMB_HEIGHT / 2f
    drawRect(
        color = thumbColor,
        topLeft = Offset(0f, (thumbY - bandHalf).coerceIn(0f, h - THUMB_HEIGHT)),
        size = Size(w, THUMB_HEIGHT),
    )
}

/** Convert a top-down fraction [0,1] of the height to a Compose vertical bias [-1,+1]. */
private fun fractionToBias(fraction: Float): Float = fraction * 2f - 1f

private const val LINE_WIDTH = 3f
private const val THUMB_HEIGHT = 28f
private const val THROTTLE_HALF_SPAN = 0.5f // centre -> edge (mirrors Throttle.ACTIVE_HALF_SPAN)
private const val ZONE_ALPHA = 0.12f

const val THROTTLE_TAG = "throttle_pad"
