package com.hoverboard.remote.ui.theme

import androidx.compose.ui.unit.dp

/**
 * Stroke width of the armed outline.
 *
 * The ride screen has no armed banner; while armed, the telemetry panel and the throttle pad both
 * carry this outline instead, so the state reads from anywhere on the screen
 * ([com.hoverboard.remote.ui.screens.ControlScreen]). One value, owned here beside [AccentRed]
 * which draws it, because two surfaces have to agree for it to read as one signal and neither of
 * them is the constant's owner.
 */
val ARMED_OUTLINE = 3.dp
