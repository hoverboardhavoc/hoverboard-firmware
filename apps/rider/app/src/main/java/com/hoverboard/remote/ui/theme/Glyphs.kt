package com.hoverboard.remote.ui.theme

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material3.LocalTextStyle
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.BlendMode
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.CompositingStrategy
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp


/*
 * The two alert glyphs and the one way to draw an alert line (`specs/rider-ui.md` 3.10, rule 2).
 *
 * Caution and warning differ in SHAPE as well as hue, so they stay apart under colour blindness:
 * caution is an outlined triangle with a bar and a dot, warning the same triangle filled with the
 * bar and dot knocked out. An alert is never a hue alone, so a caution or warning line is drawn
 * with [CautionText] or [WarningText] and nothing else.
 */

/** The triangle both glyphs are built on, its height [h] about [centre]. */
private fun triangle(centre: Offset, h: Float): Path = Path().apply {
    moveTo(centre.x, centre.y - h / 2)
    lineTo(centre.x + h * TRIANGLE_HALF_BASE, centre.y + h / 2)
    lineTo(centre.x - h * TRIANGLE_HALF_BASE, centre.y + h / 2)
    close()
}

/** The bar and the dot inside the triangle, drawn in [color] with [blendMode]. */
private fun DrawScope.barAndDot(color: Color, centre: Offset, h: Float, blendMode: BlendMode) {
    val bar = h * BAR_WIDTH
    val barTop = Offset(centre.x - bar / 2, centre.y - h * BAR_TOP)
    drawRect(color, barTop, Size(bar, h * BAR_LENGTH), blendMode = blendMode)
    drawRect(color, Offset(centre.x - bar / 2, centre.y + h * DOT_TOP), Size(bar, bar), blendMode = blendMode)
}

/**
 * The warning glyph: a filled triangle with the bar and dot knocked out to whatever is behind it.
 * The knock-out clears pixels, so the caller draws it in an offscreen layer ([WarningGlyph]).
 */
private fun DrawScope.drawWarningGlyph(color: Color, centre: Offset, h: Float) {
    drawPath(triangle(centre, h), color)
    barAndDot(Color.Black, centre, h, BlendMode.Clear)
}

/** The caution glyph: the same triangle outlined, with the bar and dot. */
private fun DrawScope.drawCautionGlyph(color: Color, centre: Offset, h: Float) {
    drawPath(triangle(centre, h), color, style = Stroke(h * OUTLINE_WIDTH))
    barAndDot(color, centre, h, BlendMode.SrcOver)
}

/** The caution glyph, [size] square, in [color]. */
@Composable
fun CautionGlyph(size: Dp, modifier: Modifier = Modifier, color: Color = Caution) {
    Canvas(modifier.size(size)) {
        drawCautionGlyph(color, Offset(this.size.width / 2, this.size.height / 2), this.size.height * CAUTION_SCALE)
    }
}

/** The warning glyph, [size] square, in [color], its bar and dot showing what is behind it. */
@Composable
fun WarningGlyph(size: Dp, modifier: Modifier = Modifier, color: Color = Warning) {
    Canvas(modifier.size(size).graphicsLayer(compositingStrategy = CompositingStrategy.Offscreen)) {
        drawWarningGlyph(color, Offset(this.size.width / 2, this.size.height / 2), this.size.height)
    }
}

/** A caution line: the caution glyph, then [text] in [Caution]. */
@Composable
fun CautionText(
    text: String,
    modifier: Modifier = Modifier,
    style: TextStyle = LocalTextStyle.current,
    maxLines: Int = Int.MAX_VALUE,
) {
    AlertLine(text, Caution, style, maxLines, modifier) { CautionGlyph(it) }
}

/** A warning line: the warning glyph, then [text] in [Warning]. */
@Composable
fun WarningText(
    text: String,
    modifier: Modifier = Modifier,
    style: TextStyle = LocalTextStyle.current,
    maxLines: Int = Int.MAX_VALUE,
) {
    AlertLine(text, Warning, style, maxLines, modifier) { WarningGlyph(it) }
}

/**
 * What a status line is: plain secondary text, normal state in [Primary], or one of the two alerts.
 * A screen that picks a line's look from its state picks a [Tone], and [ToneText] draws it, so an
 * alert tone always arrives with its glyph.
 */
enum class Tone { MUTED, PRIMARY, CAUTION, WARNING }

/** [text] in [tone]: [CautionText] or [WarningText] for an alert, else plain text in its colour. */
@Composable
fun ToneText(
    text: String,
    tone: Tone,
    modifier: Modifier = Modifier,
    style: TextStyle = LocalTextStyle.current,
    maxLines: Int = Int.MAX_VALUE,
) {
    when (tone) {
        Tone.CAUTION -> CautionText(text, modifier, style, maxLines)
        Tone.WARNING -> WarningText(text, modifier, style, maxLines)
        Tone.MUTED, Tone.PRIMARY -> Text(
            text = text,
            modifier = modifier,
            style = style,
            color = if (tone == Tone.PRIMARY) Primary else FgMuted,
            maxLines = maxLines,
            overflow = TextOverflow.Ellipsis,
        )
    }
}

/**
 * The glyph sized to the text and centred on its first line, then the text. Multi-line text wraps
 * beside the glyph, so the glyph marks the whole statement once.
 */
@Composable
private fun AlertLine(
    text: String,
    color: Color,
    style: TextStyle,
    maxLines: Int,
    modifier: Modifier = Modifier,
    glyph: @Composable (Dp) -> Unit,
) {
    val density = LocalDensity.current
    val glyphSize = style.fontSize.takeIf { it.isSp }?.let { with(density) { it.toDp() } } ?: DEFAULT_GLYPH
    val line = style.lineHeight.takeIf { it.isSp }?.let { with(density) { it.toDp() } } ?: glyphSize
    Row(modifier = modifier, verticalAlignment = Alignment.Top) {
        Box(modifier = Modifier.height(line), contentAlignment = Alignment.Center) { glyph(glyphSize) }
        Spacer(modifier = Modifier.width(GLYPH_GAP))
        Text(text = text, style = style, color = color, maxLines = maxLines, overflow = TextOverflow.Ellipsis)
    }
}

/** The triangle's half base over its height. */
private const val TRIANGLE_HALF_BASE = 0.58f

/** The bar's width, its top above the centre, and its length, over the height. */
private const val BAR_WIDTH = 0.1f
private const val BAR_TOP = 0.12f
private const val BAR_LENGTH = 0.36f

/** The dot's top below the centre, over the height. */
private const val DOT_TOP = 0.3f

/** The caution outline's stroke over the height. */
private const val OUTLINE_WIDTH = 0.12f

/** The outlined triangle is drawn a little inside its box, so its stroke is not clipped. */
private const val CAUTION_SCALE = 0.9f

/** The glyph's size beside a style that names no font size. */
private val DEFAULT_GLYPH = 14.dp

/** Between a glyph and its text. */
private val GLYPH_GAP = 6.dp
