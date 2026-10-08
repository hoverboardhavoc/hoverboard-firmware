package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * The arm toggle: the app's only way to assert `power_request`, and the reason the motors are ever
 * live. One tap arms, one tap disarms.
 *
 * It is both the control and the indicator, deliberately. A separate button and lamp can disagree;
 * one surface that is either loud red and reads ARMED or flat grey and reads TAP TO ARM cannot. It
 * is also the ONLY arm surface: the screen used to carry a full-width ARMED banner above it, reading
 * the same `armed` flag off the same state, and a second indicator that can say nothing this one
 * cannot is duplication rather than redundancy. Glanceability is kept by tinting the telemetry panel
 * and the throttle pad red while armed, not by a second piece of text
 * ([com.hoverboard.remote.ui.screens.ControlScreen]).
 *
 * [enabled] is false only when arming is refused (the throttle is held, or the link is down). It
 * never gates disarming: when [armed] is true this is always tappable, because a stop control that
 * can be unavailable is not a stop control.
 */
@Composable
fun ArmToggle(
    armed: Boolean,
    enabled: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
    riderWaived: Boolean = false,
) {
    val live = armed || enabled
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(20.dp))
            .background(if (armed) AccentRed else PanelSurface)
            .clickable(enabled = live) { onToggle() }
            .padding(vertical = 22.dp)
            .testTag(ARM_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = when {
                    armed -> stringResource(R.string.arm_disarm_action)
                    !enabled -> stringResource(R.string.arm_unavailable)
                    else -> stringResource(R.string.arm_action)
                },
                style = MaterialTheme.typography.titleLarge,
                fontWeight = FontWeight.Bold,
                textAlign = TextAlign.Center,
                color = armColor(armed = armed, enabled = enabled),
            )
            // `specs/control.md` (i): with the rider requirement waived, arming a balancing board is
            // the engage act, and the operator holds the frame upright when arming. The control that
            // does the arming is where that is said.
            if (riderWaived) {
                Text(
                    text = stringResource(R.string.arm_rider_waived),
                    style = MaterialTheme.typography.bodySmall,
                    textAlign = TextAlign.Center,
                    color = if (armed) TextPrimary else AccentYellow,
                    modifier = Modifier.padding(top = 4.dp, start = 12.dp, end = 12.dp),
                )
            }
        }
    }
}

private fun armColor(armed: Boolean, enabled: Boolean): Color = when {
    armed -> TextPrimary
    !enabled -> ZeroLine
    else -> AccentRed
}

const val ARM_TAG = "arm_toggle"
