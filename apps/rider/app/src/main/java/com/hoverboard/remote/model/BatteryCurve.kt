package com.hoverboard.remote.model

import kotlin.math.roundToInt

/**
 * 7s pack battery state-of-charge mapping (SPEC §10).
 *
 * Linear voltage->percent map: 29.4 V (100%) -> 21.7 V (0% / cutoff).
 *
 * A linear voltage-to-percent map is intentionally simple: the real Li-ion discharge curve
 * is non-linear (flat in the middle, steep at the ends), so this over/under-reads mid-pack.
 * Per SPEC §14 the curve is to be tuned against the real pack later; for v1 the linear map
 * matches the spec's stated endpoints exactly and is honest about being a placeholder.
 */
object BatteryCurve {

    /** Full-charge voltage (100%). */
    const val FULL_VOLTS: Float = 29.4f

    /** Cutoff voltage (0%, 3.1 V/cell * 7 cells). */
    const val EMPTY_VOLTS: Float = 21.7f

    /** Battery is considered low at or below this fraction of remaining charge. */
    const val LOW_FRACTION: Float = 0.15f

    private const val PERCENT_SCALE = 100

    /** Remaining charge in [0f, 1f] for a given pack [volts], linearly clamped to the endpoints. */
    fun fraction(volts: Float): Float {
        return ((volts - EMPTY_VOLTS) / (FULL_VOLTS - EMPTY_VOLTS)).coerceIn(0f, 1f)
    }

    /** Remaining charge as an integer percent in [0, 100]. */
    fun percent(volts: Float): Int {
        return (fraction(volts) * PERCENT_SCALE).roundToInt()
    }
}
