package com.hoverboard.protocol.checks

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The `limit_floor` refusal's text (`specs/config-read-once.md` section 4), as
 * `tools/hoverboard-cli/src/checks/tests.rs, the_limit_floor_refusal_names_the_counts_and_the_lowest_passing_limit`
 * pins the CLI's: the limit's counts, the floor's, and the lowest limit that passes.
 */
class LimitFloorTest {
    @Test
    fun theRefusalNamesTheCountsAndTheLowestPassingLimit() {
        assertEquals(
            "3000 mA is 1365 counts at a current.cal of 455, at or below the noise floor of 2100 counts, " +
                "so FOC's full scale is 0; the lowest limit that passes is 4700 mA",
            Checks.limitFloor(3_000, 455, 2_100),
        )
        assertNull(Checks.limitFloor(4_700, 455, 2_100))
        // The calibration is clamped into the seam's band first: a stored 50 converts at 100.
        assertEquals(
            "10000 mA is 1000 counts at a current.cal of 100, at or below the noise floor of 2100 counts, " +
                "so FOC's full scale is 0; the lowest limit that passes is 21100 mA",
            Checks.limitFloor(10_000, 50, 2_100),
        )
        // 10,100 mA, the formula's figure at 101 counts per amp and a floor of 1,020, converts to the
        // floor itself, so the next 100 mA is the lowest that passes.
        assertTrue(Checks.limitFloor(10_100, 101, 1_020) != null)
        assertNull(Checks.limitFloor(10_200, 101, 1_020))
        assertTrue(checkNotNull(Checks.limitFloor(1_000, 101, 1_020)).endsWith("the lowest limit that passes is 10200 mA"))
        assertTrue(
            checkNotNull(Checks.limitFloor(163_840, 455, 16_383))
                .endsWith("no limit passes a floor at the top of the limit window"),
        )
    }
}
