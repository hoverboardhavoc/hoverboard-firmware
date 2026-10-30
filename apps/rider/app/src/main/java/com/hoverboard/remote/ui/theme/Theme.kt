package com.hoverboard.remote.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

/**
 * Every role of the Material 3 scheme set from the tokens (`specs/rider-ui.md` 3.10), so no
 * Material default (a purple, a tonal tint) reaches a component.
 */
private val ColorScheme = darkColorScheme(
    primary = Primary,
    onPrimary = OnPrimary,
    primaryContainer = Primary,
    onPrimaryContainer = OnPrimary,
    inversePrimary = Primary,
    secondary = Primary,
    onSecondary = OnPrimary,
    secondaryContainer = Primary,
    onSecondaryContainer = OnPrimary,
    tertiary = Primary,
    onTertiary = OnPrimary,
    tertiaryContainer = Primary,
    onTertiaryContainer = OnPrimary,
    background = Bg,
    onBackground = Fg,
    surface = Bg,
    onSurface = Fg,
    surfaceVariant = BgElevated,
    onSurfaceVariant = FgMuted,
    // Transparent, so tonal elevation adds no orange to a raised surface.
    surfaceTint = Color.Transparent,
    inverseSurface = Fg,
    inverseOnSurface = Bg,
    error = Warning,
    onError = OnWarning,
    errorContainer = Warning,
    onErrorContainer = OnWarning,
    // An input's boundary needs 3:1, and Border is 1.2:1.
    outline = FgFaint,
    outlineVariant = Border,
    scrim = Bg,
    surfaceBright = BgCode,
    surfaceContainer = BgElevated,
    surfaceContainerHigh = BgCode,
    surfaceContainerHighest = BgCode,
    surfaceContainerLow = BgElevated,
    surfaceContainerLowest = Bg,
    surfaceDim = Bg,
)

/** The app's one `MaterialTheme`: the tokens and [AppTypography]. */
@Composable
fun HoverboardRemoteTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = ColorScheme,
        typography = AppTypography,
        content = content,
    )
}
