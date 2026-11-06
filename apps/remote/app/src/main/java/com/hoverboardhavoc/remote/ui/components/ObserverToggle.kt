package com.hoverboardhavoc.remote.ui.components

import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.selected
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.ui.theme.Atkinson
import com.hoverboardhavoc.remote.ui.theme.Border
import com.hoverboardhavoc.remote.ui.theme.FgFaint
import com.hoverboardhavoc.remote.ui.theme.FgMuted
import com.hoverboardhavoc.remote.ui.theme.Primary

/**
 * The observer button (`specs/remote-ui.md` 3.1, "The observer button"; `specs/flux-observer.md`,
 * "S4, first slice"): a square toggle to the right of the arm control, labelled "OBS". While [on],
 * the app sends `INPUTS.buttons` bit 1 and a FOC board commutates from its flux observer's angle.
 *
 * It shows the app's request, not the board's state. Three looks ([observerLook]): lit in [Primary]
 * while on; [FgMuted] while it can be turned on; [FgFaint] in a [Border] outline while it is
 * disabled ([enabled] false: not armed, or not a FOC machine), when a tap does nothing. A lit
 * button stays tappable, so the rider can always turn the request off.
 */
@Composable
fun ObserverToggle(
    on: Boolean,
    enabled: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val shape = RoundedCornerShape(16.dp)
    val look = observerLook(on, enabled)
    Box(
        modifier = modifier
            .size(ARM_HEIGHT)
            .clip(shape)
            .border(look.outlineWidth, look.outline, shape)
            .clickable(enabled = on || enabled) { onToggle() }
            .semantics { selected = on }
            .testTag(OBSERVER_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Text(text = stringResource(R.string.observer_action), style = OBSERVER_LABEL, color = look.label)
    }
}

/** How the button is drawn in one of its three states. */
internal data class ObserverLook(val outline: Color, val outlineWidth: Dp, val label: Color)

/** The on, off and disabled drawing: [Primary] outline and label; [FgMuted] label; [FgFaint] label. */
internal fun observerLook(on: Boolean, enabled: Boolean): ObserverLook = when {
    on -> ObserverLook(outline = Primary, outlineWidth = 2.dp, label = Primary)
    enabled -> ObserverLook(outline = Border, outlineWidth = 1.dp, label = FgMuted)
    else -> ObserverLook(outline = Border, outlineWidth = 1.dp, label = FgFaint)
}

private val OBSERVER_LABEL = TextStyle(
    fontFamily = Atkinson,
    fontWeight = FontWeight.Bold,
    fontSize = 16.sp,
)

const val OBSERVER_TAG = "observer_toggle"
