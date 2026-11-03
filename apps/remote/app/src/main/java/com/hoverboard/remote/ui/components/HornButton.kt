package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.Atkinson
import com.hoverboard.remote.ui.theme.OnPrimary
import com.hoverboard.remote.ui.theme.Primary

/**
 * The horn (`specs/tones.md` section 5): held, the `INPUTS` the app sends carry
 * `buttons` bit1 and the board plays its horn tune; released, they do not. [onHorn] is called with
 * `true` on the press and `false` on the release, and also when the gesture is cancelled or the
 * button leaves composition under a finger, so a held horn cannot outlive the control.
 *
 * Beside the arm toggle, at its height, so the Ride screen's fixed slots do not move.
 */
@Composable
fun HornButton(onHorn: (Boolean) -> Unit, modifier: Modifier = Modifier) {
    val shape = RoundedCornerShape(16.dp)
    val horn by rememberUpdatedState(onHorn)
    var held by remember { mutableStateOf(false) }
    Box(
        modifier = modifier
            .width(HORN_WIDTH)
            .height(ARM_HEIGHT)
            .clip(shape)
            .background(if (held) Primary else Color.Transparent)
            .border(2.dp, Primary, shape)
            .pointerInput(Unit) {
                detectTapGestures(
                    onPress = {
                        held = true
                        horn(true)
                        try {
                            awaitRelease()
                        } finally {
                            held = false
                            horn(false)
                        }
                    },
                )
            }
            .testTag(HORN_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = stringResource(R.string.horn_action),
            style = HORN_LABEL,
            color = if (held) OnPrimary else Primary,
        )
    }
}

private val HORN_WIDTH = 96.dp

private val HORN_LABEL = TextStyle(fontFamily = Atkinson, fontWeight = FontWeight.Bold, fontSize = 16.sp)

const val HORN_TAG = "horn_button"
