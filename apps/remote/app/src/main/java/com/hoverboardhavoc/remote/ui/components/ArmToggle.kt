package com.hoverboardhavoc.remote.ui.components

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
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.model.ArmPhase
import com.hoverboardhavoc.remote.ui.theme.Atkinson
import com.hoverboardhavoc.remote.ui.theme.Border
import com.hoverboardhavoc.remote.ui.theme.FgFaint
import com.hoverboardhavoc.remote.ui.theme.OnPrimary
import com.hoverboardhavoc.remote.ui.theme.Primary
import com.hoverboardhavoc.remote.ui.theme.TABULAR_FIGURES

/**
 * The arm toggle (`specs/remote-ui.md` 3.1): the app's only way to assert `power_request`, and the
 * reason the motors are ever live. One tap arms, one tap disarms.
 *
 * It is the control and the indicator in one surface ([armLook]), and it shows only what the boards
 * confirm (`specs/arm-confirmed.md` section 3): filled [Primary] and reading "ARMED · tap to
 * disarm" once every latched board reports `FLAG_MOE`; outlined 2 dp in [Primary] with "ARM" in
 * [Primary] while it can arm, and with "ARMING..." while the boards have not confirmed yet;
 * outlined 1 dp in [Border] with "ARM" in [FgFaint] while arming is refused, and with
 * "DISARMING..." while a disarm is not confirmed yet. Armed state
 * reads from four places in [Primary], the top bar's outline, this fill, the pad's tinted half and
 * its thumb, so no banner repeats it. What the control knows about the machine is said in the
 * statements under it, not inside it.
 *
 * [enabled] is false only when arming is refused. It never gates disarming: while the app holds the
 * arm level ([ArmPhase.holdsLevel]) the control is always tappable, because a stop control that can
 * be unavailable is not a stop control. While DISARMING it is not tappable: the stop is already
 * on the wire, and only the label waits.
 */
@Composable
fun ArmToggle(
    phase: ArmPhase,
    enabled: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(16.dp)
    val look = armLook(phase, enabled)
    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(ARM_HEIGHT)
            .clip(shape)
            .background(look.fill)
            .border(look.outlineWidth, look.outline, shape)
            .clickable(enabled = phase.holdsLevel || (phase == ArmPhase.IDLE && enabled)) { onToggle() }
            .testTag(ARM_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = stringResource(
                when (phase) {
                    ArmPhase.IDLE -> R.string.arm_action
                    ArmPhase.ARMING -> R.string.arm_arming
                    ArmPhase.ARMED -> R.string.arm_disarm_action
                    ArmPhase.DISARMING -> R.string.arm_disarming
                },
            ),
            style = ARM_LABEL,
            color = look.label,
        )
    }
}

/** How the control is drawn in one of its three states (`specs/remote-ui.md` 3.1). */
internal data class ArmLook(val fill: Color, val outline: Color, val outlineWidth: Dp, val label: Color)

/**
 * The armed, can-arm and cannot-arm drawing: filled [Primary]; outlined in [Primary]; outlined in
 * [Border]. ARMING draws as can-arm (the control is live, a tap disarms) and DISARMING as cannot-arm.
 */
internal fun armLook(phase: ArmPhase, enabled: Boolean): ArmLook = when {
    phase == ArmPhase.ARMED ->
        ArmLook(fill = Primary, outline = Primary, outlineWidth = 2.dp, label = OnPrimary)
    phase == ArmPhase.ARMING || (phase == ArmPhase.IDLE && enabled) ->
        ArmLook(fill = Color.Transparent, outline = Primary, outlineWidth = 2.dp, label = Primary)
    else -> ArmLook(fill = Color.Transparent, outline = Border, outlineWidth = 1.dp, label = FgFaint)
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
