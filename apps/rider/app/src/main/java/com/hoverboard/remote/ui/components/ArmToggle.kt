package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
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
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * The arm toggle: the app's only way to assert `power_request`, and the reason the motors are ever
 * live. One tap arms, one tap disarms.
 *
 * It is both the control and the indicator, deliberately. A separate button and lamp can disagree;
 * one surface that is either loud red and reads ARMED or flat grey and reads TAP TO ARM cannot. The
 * screen also carries a full-width banner above it
 * ([com.hoverboard.remote.ui.screens.ControlScreen]), so the armed state is legible from a glance
 * at any part of the screen.
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
    }
}

private fun armColor(armed: Boolean, enabled: Boolean): Color = when {
    armed -> TextPrimary
    !enabled -> ZeroLine
    else -> AccentRed
}

const val ARM_TAG = "arm_toggle"
