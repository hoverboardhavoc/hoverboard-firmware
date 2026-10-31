package com.hoverboard.remote.ui.theme

import androidx.compose.material3.Typography
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.Font
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import com.hoverboard.remote.R

/**
 * Atkinson Hyperlegible, regular and bold (`specs/remote-ui.md` 3.10), under the SIL Open Font
 * License 1.1; the licence ships beside them as `assets/fonts/OFL.txt`.
 */
val Atkinson = FontFamily(
    Font(R.font.atkinson_hyperlegible_regular, FontWeight.Normal),
    Font(R.font.atkinson_hyperlegible_bold, FontWeight.Bold),
)

/**
 * The font's figures are proportional by default and it carries a `tnum` feature, so every style
 * asks for tabular figures: a number that updates in place keeps its width.
 */
const val TABULAR_FIGURES = "tnum"

/** [style] on [Atkinson] at [weight], with tabular figures. */
private fun atkinson(style: TextStyle, weight: FontWeight): TextStyle =
    style.copy(fontFamily = Atkinson, fontWeight = weight, fontFeatureSettings = TABULAR_FIGURES)

/**
 * Material 3's default type scale with every style on [Atkinson]: sizes, line heights and
 * tracking as Material sets them. The family has no 500 weight, which Material's titles and labels
 * ask for and the font matcher would answer with regular, so they take bold.
 */
val AppTypography: Typography = Typography().let { t ->
    Typography(
        displayLarge = atkinson(t.displayLarge, FontWeight.Normal),
        displayMedium = atkinson(t.displayMedium, FontWeight.Normal),
        displaySmall = atkinson(t.displaySmall, FontWeight.Normal),
        headlineLarge = atkinson(t.headlineLarge, FontWeight.Normal),
        headlineMedium = atkinson(t.headlineMedium, FontWeight.Normal),
        headlineSmall = atkinson(t.headlineSmall, FontWeight.Normal),
        titleLarge = atkinson(t.titleLarge, FontWeight.Bold),
        titleMedium = atkinson(t.titleMedium, FontWeight.Bold),
        titleSmall = atkinson(t.titleSmall, FontWeight.Bold),
        bodyLarge = atkinson(t.bodyLarge, FontWeight.Normal),
        bodyMedium = atkinson(t.bodyMedium, FontWeight.Normal),
        bodySmall = atkinson(t.bodySmall, FontWeight.Normal),
        labelLarge = atkinson(t.labelLarge, FontWeight.Bold),
        labelMedium = atkinson(t.labelMedium, FontWeight.Bold),
        labelSmall = atkinson(t.labelSmall, FontWeight.Bold),
    )
}
