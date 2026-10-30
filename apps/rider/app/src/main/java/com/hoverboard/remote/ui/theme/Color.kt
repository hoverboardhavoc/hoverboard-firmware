package com.hoverboard.remote.ui.theme

import androidx.compose.ui.graphics.Color

/*
 * The app's colour tokens (`specs/rider-ui.md` 3.10): the hoverboardhavoc.com palette, with
 * caution and warning from the 8a mockup. Every screen takes its colours from here, through
 * [HoverboardRemoteTheme] or by name; no screen defines a colour of its own.
 *
 * The rules the tokens are held to:
 *  1. Only [Primary] carries normal state (armed, selected, on, in progress, written, the live
 *     thumb, the bars and the ring).
 *  2. [Caution] and [Warning] are for alerts only, and never as a hue alone: an alert line is drawn
 *     with [CautionText] or [WarningText], which put the glyph in front of it.
 *  3. A screen with nothing wrong shows no caution or warning and no lit lamp.
 *  4. Solid fills only: no gradient anywhere.
 */

/** Screen background. */
val Bg = Color(0xFF000000)

/** Panels, menus, the bottom sheet. */
val BgElevated = Color(0xFF101012)

/** Tracks, the pad, dark lamps, unlit ring segments, dialogs. */
val BgCode = Color(0xFF141417)

/** 1 dp outlines on panels, the pad and lamps; dividers. White at 12 percent. */
val Border = Color(0x1FFFFFFF)

/** Text and figures: 19.3:1 on [Bg]. */
val Fg = Color(0xFFF5F5F5)

/** Labels, units, secondary text: 10.0:1 on [Bg]. */
val FgMuted = Color(0xFFB2B2B8)

/**
 * Inactive things only (dark lamp names, a refused ARM label, the pad's direction words, input
 * outlines). 3.1:1 on [Bg], under the 4.5:1 text minimum, which is why it never carries live text.
 */
val FgFaint = Color(0xFF5A5A62)

/** The one accent, the site's flame: solid only. 6.7:1 on [Bg]. */
val Primary = Color(0xFFFF5A00)

/** Text and glyphs on [Primary]. White on it is 2.9:1 and is never used. */
val OnPrimary = Color(0xFF000000)

/** The mini horizon's sky and the attitude corners' board marks, nothing else. */
val Neon = Color(0xFF2FD3F0)

/** Caution, always with the outlined glyph. */
val Caution = Color(0xFFFFD23F)

/** Warning, always with the filled glyph and text. */
val Warning = Color(0xFFFF1F7A)

/** Text and glyphs on a lit lamp. */
val OnWarning = Color(0xFF000000)
