package com.hoverboard.remote.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable

private val ColorScheme = darkColorScheme(
    background = DarkBackground,
    surface = DarkSurface,
    surfaceVariant = PanelSurface,
    primary = AccentGreen,
    secondary = ThrottleReverse,
    error = AccentRed,
    onBackground = TextPrimary,
    onSurface = TextPrimary,
    onPrimary = DarkBackground,
)

@Composable
fun HoverboardRemoteTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = ColorScheme,
        content = content,
    )
}
