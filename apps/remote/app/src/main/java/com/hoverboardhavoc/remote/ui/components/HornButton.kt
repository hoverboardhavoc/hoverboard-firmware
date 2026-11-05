package com.hoverboardhavoc.remote.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.requiredSize
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.dp
import com.hoverboardhavoc.remote.R
import com.hoverboardhavoc.remote.ui.theme.OnPrimary
import com.hoverboardhavoc.remote.ui.theme.Primary

/**
 * The horn (`specs/tones.md` section 5): held, the `INPUTS` the app sends carry
 * `buttons` bit1 and the board plays its horn tune; released, they do not. [onHorn] is called with
 * `true` on the press and `false` on the release, and also when the gesture is cancelled or the
 * button leaves composition under a finger, so a held horn cannot outlive the control.
 *
 * A round button with a drawn horn glyph and no text, outlined in [Primary] and filled while held.
 * The circle is [HORN_DIAMETER] across, so it fits the gap between the Ride screen's phase
 * readouts; the touch target around it is [HORN_TOUCH] square and may reach over their ends, which
 * take no touches.
 */
@Composable
fun HornButton(onHorn: (Boolean) -> Unit, modifier: Modifier = Modifier) {
    val horn by rememberUpdatedState(onHorn)
    var held by remember { mutableStateOf(false) }
    val description = stringResource(R.string.horn_action)
    Box(
        modifier = modifier
            .requiredSize(HORN_TOUCH)
            .semantics { contentDescription = description }
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
        Canvas(Modifier.size(HORN_DIAMETER)) {
            val ring = HORN_OUTLINE.toPx()
            val r = size.minDimension / 2f
            if (held) drawCircle(Primary, r)
            drawCircle(Primary, r - ring / 2f, style = Stroke(ring))
            hornGlyph(if (held) OnPrimary else Primary)
        }
    }
}

/**
 * A horn pointing right, in unit coordinates of the circle: a short mouthpiece, a bell flaring to
 * the right, and one sound arc beyond the bell.
 */
private fun DrawScope.hornGlyph(color: Color) {
    val s = size.minDimension
    fun p(x: Float, y: Float) = Offset(x * s, y * s)
    val stroke = Stroke(HORN_GLYPH_STROKE.toPx(), cap = StrokeCap.Round, join = StrokeJoin.Round)
    val body = Path().apply {
        HORN_OUTLINE_POINTS.forEachIndexed { i, (x, y) ->
            val o = p(x, y)
            if (i == 0) moveTo(o.x, o.y) else lineTo(o.x, o.y)
        }
        close()
    }
    drawPath(body, color, style = stroke)
    val r = WAVE_RADIUS * s
    val c = p(HORN_BELL_X, HORN_AXIS_Y)
    drawArc(
        color = color,
        startAngle = -WAVE_HALF_SWEEP,
        sweepAngle = 2 * WAVE_HALF_SWEEP,
        useCenter = false,
        topLeft = Offset(c.x - r, c.y - r),
        size = Size(2 * r, 2 * r),
        style = stroke,
    )
}

/** The drawn circle: it sits in the 64 dp gap between the phase readouts with room on each side. */
private val HORN_DIAMETER = 30.dp

/** The touch target, at least 48 dp square. */
private val HORN_TOUCH = 48.dp

private val HORN_OUTLINE = 2.dp
private val HORN_GLYPH_STROKE = 1.6.dp

/** The mouthpiece and the bell, as one outline in unit coordinates of the circle. */
private val HORN_OUTLINE_POINTS = listOf(
    0.24f to 0.44f,
    0.36f to 0.44f,
    HORN_BELL_X to 0.30f,
    HORN_BELL_X to 0.70f,
    0.36f to 0.56f,
    0.24f to 0.56f,
)

/** The bell's mouth and the horn's axis, where the sound arc is centred. */
private const val HORN_BELL_X = 0.62f
private const val HORN_AXIS_Y = 0.5f
private const val WAVE_RADIUS = 0.14f
private const val WAVE_HALF_SWEEP = 40f

const val HORN_TAG = "horn_button"
