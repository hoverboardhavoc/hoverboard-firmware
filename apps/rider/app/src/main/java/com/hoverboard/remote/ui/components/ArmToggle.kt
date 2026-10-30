package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.Atkinson
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.OnPrimary
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.TABULAR_FIGURES

/**
 * The arm toggle (`specs/rider-ui.md` 3.1): the app's only way to assert `power_request`, and the
 * reason the motors are ever live. One tap arms, one tap disarms.
 *
 * It is the control and the indicator in one surface: filled [Primary] and reading "ARMED · tap to
 * disarm" while armed, outlined and reading "ARM" otherwise, in [FgMuted] while it can arm and in
 * [FgFaint] while arming is refused. Armed state reads from four places in [Primary], the top bar's
 * outline, this fill, the pad's tinted half and its thumb, so no banner repeats it. What the control
 * knows about the machine is said in the statements under it, not inside it.
 *
 * [enabled] is false only when arming is refused. It never gates disarming: while [armed] the
 * control is always tappable, because a stop control that can be unavailable is not a stop control.
 */
@Composable
fun ArmToggle(
    armed: Boolean,
    enabled: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(16.dp)
    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(ARM_HEIGHT)
            .clip(shape)
            .background(if (armed) Primary else Color.Transparent)
            .border(1.dp, if (armed) Primary else Border, shape)
            .clickable(enabled = armed || enabled) { onToggle() }
            .testTag(ARM_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = stringResource(if (armed) R.string.arm_disarm_action else R.string.arm_action),
            style = ARM_LABEL,
            color = when {
                armed -> OnPrimary
                enabled -> FgMuted
                else -> FgFaint
            },
        )
    }
}

/** The control's height, a fixed slot of the Ride screen's column. */
val ARM_HEIGHT = 52.dp

private val ARM_LABEL = TextStyle(
    fontFamily = Atkinson,
    fontWeight = FontWeight.Bold,
    fontSize = 16.sp,
    fontFeatureSettings = TABULAR_FIGURES,
)

const val ARM_TAG = "arm_toggle"
