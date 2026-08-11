package com.hoverboard.remote

import com.hoverboard.remote.model.BatteryCurve
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/** 7s battery state-of-charge map: 29.4 V (100%) -> 21.7 V (0%) (SPEC §10). */
class BatteryCurveTest {

    @Test
    fun `full voltage is 100 percent`() {
        assertEquals(100, BatteryCurve.percent(29.4f))
        assertEquals(1f, BatteryCurve.fraction(29.4f), 0.0001f)
    }

    @Test
    fun `cutoff voltage is 0 percent`() {
        assertEquals(0, BatteryCurve.percent(21.7f))
        assertEquals(0f, BatteryCurve.fraction(21.7f), 0.0001f)
    }

    @Test
    fun `midpoint voltage is about 50 percent`() {
        // (29.4 + 21.7) / 2 = 25.55 V -> ~50%
        assertEquals(50, BatteryCurve.percent(25.55f))
        assertEquals(0.5f, BatteryCurve.fraction(25.55f), 0.01f)
    }

    @Test
    fun `above full clamps to 100`() {
        assertEquals(100, BatteryCurve.percent(30.5f))
    }

    @Test
    fun `below cutoff clamps to 0`() {
        assertEquals(0, BatteryCurve.percent(20.0f))
    }

    @Test
    fun `quarter span maps linearly`() {
        // 21.7 + 0.25 * (29.4 - 21.7) = 23.625 V -> 25%
        assertEquals(25, BatteryCurve.percent(23.625f))
    }
}
