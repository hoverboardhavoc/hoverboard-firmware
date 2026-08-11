package com.hoverboard.remote.model

import kotlin.math.roundToInt

/**
 * Deadman throttle coordinate math (SPEC §9).
 *
 * The throttle is a vertical pad of height H, active across its WHOLE bounds. Coordinates are
 * in pixels with y growing downward from the top (the Android convention).
 *
 * ```
 *  y = 0       top ──────────────  = MAX forward
 *              │    forward    │
 *  y = 0.5 H   │······ 0 ······│  = zero / rest (centre)
 *              │    reverse    │
 *  y = 1.0 H   bottom ────────────  = MAX reverse
 * ```
 *
 * - Engage: a touch anywhere on the pad drives; lifting -> immediate stop (0).
 * - Mapping: t = (0.5 H - y) / (0.5 H), clamped to [-1, +1]; speed = round(t * MAX_SPEED).
 *   - y = 0.5 H -> 0; y = 0 -> +MAX (forward); y = H -> -MAX (reverse).
 * - Forward (above centre) and reverse (below centre) are symmetric, each getting half the pad.
 */
object Throttle {

    /**
     * Maximum commanded magnitude (per wheel, on the wire scale of -1000..1000).
     * Full scale — the original 40% cap from SPEC §9 left top-end at ~0.3 km/h.
     * Bench-cap this in app-side code (or a settings screen) if more headroom hurts.
     */
    const val MAX_SPEED: Int = 1000

    /** Fraction of the pad height (from the top) of the zero / rest line — the pad centre. */
    const val ZERO_FRACTION: Float = 0.5f

    /** Half-span (as a fraction of height) from the zero line to a full-scale edge. */
    private const val ACTIVE_HALF_SPAN: Float = 0.5f

    /**
     * The whole pad is live: any touch on a non-empty pad engages (lift = stop). [y] is not
     * part of the gate any more — every touch drives — but [height] guards the degenerate pad.
     */
    fun isEngaged(height: Float): Boolean = height > 0f

    /**
     * Normalised throttle in [-1, +1] for a touch at [y] within a pad of [height].
     * Positive = forward (above centre), negative = reverse (below centre).
     */
    fun normalized(y: Float, height: Float): Float {
        if (height <= 0f) return 0f
        val t = (ZERO_FRACTION * height - y) / (ACTIVE_HALF_SPAN * height)
        return t.coerceIn(-1f, 1f)
    }

    /**
     * Commanded wire speed for a touch at [y] within a pad of [height], using [maxSpeed].
     * Positive = forward, negative = reverse, 0 at the centre line.
     */
    fun speedFor(y: Float, height: Float, maxSpeed: Int = MAX_SPEED): Int {
        return (normalized(y, height) * maxSpeed).roundToInt()
    }

    /**
     * Commanded speed gated by engagement: 0 for a degenerate (empty) pad, else the full
     * [speedFor] mapping. Lift-to-stop is handled by the caller.
     */
    fun engagedSpeedFor(y: Float, height: Float, maxSpeed: Int = MAX_SPEED): Int {
        return if (isEngaged(height)) speedFor(y, height, maxSpeed) else 0
    }
}
