package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.DriveCmd
import kotlin.math.abs
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
     * Maximum commanded magnitude: full pad travel is full `DRIVE_CMD` scale.
     *
     * This shipped once at a quarter of full scale, as a deliberate cap for the first build that
     * could move a wheel at all. The bench verdict on that was "slow", which it was: a quarter of
     * scale is a command of 249 in the firmware's 1000-domain, about 22% duty at the very top of
     * the pad. A capped pad also wastes the resolution the rider actually steers with, since the
     * mapping is proportional across the whole travel either way.
     *
     * The previous value before that was 1000, with a comment reasoning about top speed and a 40%
     * cap. Both the number and the reasoning were about `INPUTS.throttle`, a word no consumer reads
     * (`crates/swd-bridge/src/bin/drive.rs:14-16`), so nothing that comment described was ever
     * measured: the app had never moved a wheel.
     *
     * Two floors of the firmware's arithmetic sit inside this travel and are worth knowing when
     * reading the pad (both derived in [DriveCmd.FULL_SCALE]'s doc and pinned by the drift gate): a
     * demand under +-33 truncates to a zero command, and a stopped machine will not engage below
     * +-590, which is under 2% of travel from centre.
     */
    const val MAX_SPEED: Int = DriveCmd.FULL_SCALE

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
     *
     * Rounds away from zero rather than using [roundToInt] directly, so forward and reverse are
     * exactly symmetric. `roundToInt` breaks ties toward positive infinity, so a half-count landed
     * on +n forward and -(n-1) in reverse: with an odd [MAX_SPEED] that is every half-travel touch.
     * The difference is one count on a 32767 scale and could never reach a motor (it is under the
     * firmware's 33-count truncation floor), but a throttle that is not symmetric about its own
     * centre line is worth a line of arithmetic rather than a caveat.
     */
    fun speedFor(y: Float, height: Float, maxSpeed: Int = MAX_SPEED): Int {
        val scaled = normalized(y, height) * maxSpeed
        val magnitude = abs(scaled).roundToInt()
        return if (scaled < 0f) -magnitude else magnitude
    }

    /**
     * Commanded speed gated by engagement: 0 for a degenerate (empty) pad, else the full
     * [speedFor] mapping. Lift-to-stop is handled by the caller.
     */
    fun engagedSpeedFor(y: Float, height: Float, maxSpeed: Int = MAX_SPEED): Int {
        return if (isEngaged(height)) speedFor(y, height, maxSpeed) else 0
    }
}
