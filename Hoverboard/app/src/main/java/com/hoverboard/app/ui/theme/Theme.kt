package com.hoverboard.app.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable

/**
 * The Hoverboard app theme: a dark, high-contrast Material 3 scheme on the blog's pure-black
 * chassis. The flame orange ([Flame1]) is the primary accent; the neon cyan ([Neon]) is the
 * "live / connected" secondary; surfaces are the OLED blacks.
 */
private val HoverboardColorScheme = darkColorScheme(
    background = Bg,
    surface = BgElevated,
    surfaceVariant = BgInset,
    primary = Flame1,
    onPrimary = Bg,
    secondary = Neon,
    onSecondary = Bg,
    error = StatusError,
    onError = Bg,
    onBackground = TextPrimary,
    onSurface = TextPrimary,
    onSurfaceVariant = TextMuted,
    outline = Border,
)

@Composable
fun HoverboardTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = HoverboardColorScheme,
        content = content,
    )
}
