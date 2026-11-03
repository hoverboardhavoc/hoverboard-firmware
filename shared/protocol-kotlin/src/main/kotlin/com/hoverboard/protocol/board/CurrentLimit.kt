package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields

/**
 * The current limit's arithmetic, mirrored from `motorcontroller/board/src/limit.rs` (`limit_counts`,
 * `full_scale_counts`, `request_counts`), for the configuration checks' `limit_floor`
 * ([com.hoverboard.protocol.checks.Check.LIMIT_FLOOR], `specs/config-read-once.md` section 4). The
 * bounds are pinned to the Rust by `RustSourceDriftTest`, and the shared check cases hold the two
 * conversions together.
 */
object CurrentLimit {
    /**
     * The calibration's seam band, counts per amp (`motorcontroller/board/src/limit.rs`,
     * `CURRENT_CAL_MIN`, `CURRENT_CAL_MAX`): a stored `current.cal` is clamped into it before the
     * conversion.
     */
    val CURRENT_CAL = 100L..819L

    /**
     * The converted limit's ceiling, counts (`motorcontroller/board/src/limit.rs`,
     * `MAX_LIMIT_COUNTS`): half the sensor's full scale.
     */
    const val MAX_LIMIT_COUNTS = Short.MAX_VALUE / 2L

    /**
     * The largest request the conversion multiplies, milliamps (`motorcontroller/board/src/limit.rs`,
     * `LIMIT_REQUEST_CAP_MA`): at the lowest calibration it already converts above the ceiling.
     */
    val LIMIT_REQUEST_CAP_MA = (MAX_LIMIT_COUNTS + 1) * 1_000 / CURRENT_CAL.first

    /**
     * A stored noise floor as the limit sees it (`motorcontroller/board/src/limit.rs`,
     * `noise_floor_counts`): clamped into [Fields.NOISE_FLOOR].
     */
    fun noiseFloorCounts(stored: Long): Long = stored.coerceIn(Fields.NOISE_FLOOR)

    /**
     * The request in counts before the window (`motorcontroller/board/src/limit.rs`,
     * `request_counts`): `ma * clamp(cal) / 1000`, the request capped at [LIMIT_REQUEST_CAP_MA].
     */
    fun requestCounts(ma: Long, cal: Long): Long = minOf(ma, LIMIT_REQUEST_CAP_MA) * cal.coerceIn(CURRENT_CAL) / 1_000

    /**
     * The soft limit in counts (`motorcontroller/board/src/limit.rs`, `limit_counts`): the request
     * clamped into the floor up to [MAX_LIMIT_COUNTS].
     */
    fun limitCounts(ma: Long, cal: Long, noiseFloor: Long): Long =
        requestCounts(ma, cal).coerceIn(noiseFloorCounts(noiseFloor), MAX_LIMIT_COUNTS)

    /**
     * FOC's current-reference full scale, counts (`motorcontroller/board/src/limit.rs`,
     * `full_scale_counts`): the soft limit less the floor, floored at zero.
     */
    fun fullScaleCounts(limitCounts: Long, noiseFloor: Long): Long =
        (limitCounts - noiseFloorCounts(noiseFloor)).coerceIn(0L, Short.MAX_VALUE.toLong())
}
