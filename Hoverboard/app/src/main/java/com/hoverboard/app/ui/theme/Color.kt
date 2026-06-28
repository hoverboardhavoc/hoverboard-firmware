package com.hoverboard.app.ui.theme

import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color

/**
 * The "Hoverboard Havoc" brand palette, lifted from the blog theme
 * (`hoverboardhavoc.com/src/styles/global.css`): a pure-black OLED chassis with orange-flame
 * accents and a neon cyan-blue LED glow, sampled from the brand logo.
 */

// --- Surfaces (pure black / OLED) ---
/** `--bg`: the app background. */
val Bg = Color(0xFF000000)

/** `--bg-elevated`: cards / panels lifted off the black. */
val BgElevated = Color(0xFF101012)

/** `--bg-code`: a slightly lighter inset (code blocks on the blog). */
val BgInset = Color(0xFF141417)

/** `--border`: hairline dividers, `rgba(255,255,255,0.12)`. */
val Border = Color(0x1FFFFFFF)

// --- Text ---
/** `--text`: primary near-white. */
val TextPrimary = Color(0xFFF5F5F5)

/** `--text-muted`: secondary grey. */
val TextMuted = Color(0xFFB2B2B8)

// --- Flame accent (the brand orange gradient) ---
/** `--flame-1`: deep orange (the primary accent). */
val Flame1 = Color(0xFFFF5A00)

/** `--flame-2`: amber. */
val Flame2 = Color(0xFFFF8C1A)

/** `--flame-3`: hot yellow. */
val Flame3 = Color(0xFFFFC777)

// --- Neon LED glow (cyan-blue) ---
/** `--neon`: LED cyan-blue (the "connected / live" accent). */
val Neon = Color(0xFF2FD3F0)

// --- Status colors (derived from the palette) ---
/** Error / fault uses a hot red distinct from the flame orange. */
val StatusError = Color(0xFFFF4D4D)

/**
 * `--flame-gradient`: `linear-gradient(100deg, flame-1, flame-2 55%, flame-3)`. Compose has no
 * angle-degrees brush, so a 100deg (slightly past horizontal, downward-right) gradient is built as
 * a linear gradient with the flame stops at 0 / 0.55 / 1.0; the start/end offsets are supplied at
 * the call site so it can span the element being painted.
 */
fun flameBrush(): Brush =
    Brush.linearGradient(
        colorStops = arrayOf(
            0.0f to Flame1,
            0.55f to Flame2,
            1.0f to Flame3,
        ),
    )
