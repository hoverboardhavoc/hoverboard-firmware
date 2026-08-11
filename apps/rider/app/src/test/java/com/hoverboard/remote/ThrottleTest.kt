package com.hoverboard.remote

import com.hoverboard.remote.model.Throttle
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Deadman throttle coordinate math (SPEC §9), full-pad center-zero mapping.
 *
 * Reference points for a pad of height H:
 *  - y = 0.5 H -> 0 (centre / rest)
 *  - y = 0    -> +MAX (forward, top edge)
 *  - y = H    -> -MAX (reverse, bottom edge)
 *  - any touch on a non-empty pad engages.
 */
class ThrottleTest {

    private val h = 1000f
    private val max = Throttle.MAX_SPEED

    @Test
    fun `max speed is full scale 1000`() {
        // The deadman maps to INPUTS.throttle as an i16 in -1000..1000, so full scale is
        // 1000 (the firmware CLAMP(-1000, 1000) range).
        assertEquals(1000, Throttle.MAX_SPEED)
    }

    @Test
    fun `any touch on a non-empty pad engages`() {
        assertTrue(Throttle.isEngaged(height = h))
        assertTrue(Throttle.isEngaged(height = 1f))
    }

    @Test
    fun `engage false for zero height`() {
        assertFalse(Throttle.isEngaged(height = 0f))
    }

    @Test
    fun `centre gives zero speed`() {
        assertEquals(0, Throttle.speedFor(y = 0.5f * h, height = h))
    }

    @Test
    fun `top gives max forward`() {
        assertEquals(max, Throttle.speedFor(y = 0f, height = h))
    }

    @Test
    fun `bottom gives max reverse`() {
        assertEquals(-max, Throttle.speedFor(y = h, height = h))
    }

    @Test
    fun `quarter above centre is half forward`() {
        // y = 0.25 H -> t = (0.5 - 0.25) / 0.5 = 0.5 -> +max/2
        assertEquals(max / 2, Throttle.speedFor(y = 0.25f * h, height = h))
    }

    @Test
    fun `quarter below centre is half reverse`() {
        // y = 0.75 H -> t = (0.5 - 0.75) / 0.5 = -0.5 -> -max/2
        assertEquals(-max / 2, Throttle.speedFor(y = 0.75f * h, height = h))
    }

    @Test
    fun `forward and reverse are symmetric`() {
        val fwd = Throttle.speedFor(y = 0.25f * h, height = h)
        val rev = Throttle.speedFor(y = 0.75f * h, height = h)
        assertEquals(fwd, -rev)
    }

    @Test
    fun `speed clamps beyond the edges`() {
        // Dragging above the top or below the bottom clamps to +-max, not beyond.
        assertEquals(max, Throttle.speedFor(y = -0.2f * h, height = h))
        assertEquals(-max, Throttle.speedFor(y = 1.2f * h, height = h))
        assertEquals(1f, Throttle.normalized(y = -0.2f * h, height = h), 0.0001f)
        assertEquals(-1f, Throttle.normalized(y = 1.2f * h, height = h), 0.0001f)
    }

    @Test
    fun `engagedSpeedFor is zero only for a degenerate pad`() {
        // No height -> not engaged -> 0, regardless of y.
        assertEquals(0, Throttle.engagedSpeedFor(y = 0f, height = 0f))
        // Valid pad: full mapping applies across the whole height.
        assertEquals(max, Throttle.engagedSpeedFor(y = 0f, height = h))
        assertEquals(0, Throttle.engagedSpeedFor(y = 0.5f * h, height = h))
        assertEquals(-max, Throttle.engagedSpeedFor(y = h, height = h))
    }

    @Test
    fun `custom max speed scales output`() {
        assertEquals(1000, Throttle.speedFor(y = 0f, height = h, maxSpeed = 1000))
        assertEquals(-1000, Throttle.speedFor(y = h, height = h, maxSpeed = 1000))
    }
}
