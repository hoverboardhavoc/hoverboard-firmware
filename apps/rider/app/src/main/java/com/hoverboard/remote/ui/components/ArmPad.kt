package com.hoverboard.remote.ui.components

import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * Hold-to-arm control: the app's only way to assert `power_request`, and the reason the motors are
 * ever live.
 *
 * Press and hold to arm; lift, or let the gesture be cancelled, to disarm. It is a level, not a
 * toggle, because that is what the firmware samples: the finger is the state.
 *
 * The pad refuses to arm while the throttle is held ([enabled] is false then), so the machine can
 * never come alive already being asked for travel. It is drawn in the same disabled grey as an
 * unconnected link, because from the rider's point of view both mean the same thing: this control
 * will not do anything right now.
 *
 * Armed is drawn as loudly as a control this small can be: the whole pad fills solid red, the label
 * changes, and [com.hoverboard.remote.ui.screens.ControlScreen] puts a full-width banner above it.
 * A rider glancing at the screen should never have to work out whether the board is live.
 */
@Composable
fun ArmPad(
    armed: Boolean,
    enabled: Boolean,
    onPress: () -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(24.dp))
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                awaitEachGesture {
                    val down = awaitFirstDown(requireUnconsumed = false)
                    down.consume()
                    onPress()
                    try {
                        // Hold until this pointer goes up. Every exit from the wait disarms,
                        // including the cancellation path: if the gesture is torn out from under
                        // us (the window goes away, a parent claims the pointer), the finally
                        // still runs and the level drops.
                        do {
                            val event = awaitPointerEvent()
                            event.changes.firstOrNull { it.id == down.id }?.consume()
                        } while (event.changes.any { it.pressed })
                    } finally {
                        onRelease()
                    }
                }
            }
            .drawBehind { drawRect(color = if (armed) AccentRed else PanelSurface) }
            .testTag(ARM_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = when {
                armed -> stringResource(R.string.arm_armed)
                !enabled -> stringResource(R.string.arm_unavailable)
                else -> stringResource(R.string.arm_hold)
            },
            style = MaterialTheme.typography.titleMedium,
            textAlign = TextAlign.Center,
            color = armColor(armed = armed, enabled = enabled),
            modifier = Modifier.padding(12.dp),
        )
    }
}

private fun armColor(armed: Boolean, enabled: Boolean): Color = when {
    armed -> TextPrimary
    !enabled -> ZeroLine
    else -> AccentRed
}

const val ARM_TAG = "arm_pad"
