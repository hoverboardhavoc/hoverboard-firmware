package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.DriveCmd
import com.hoverboard.remote.model.DriveMix
import com.hoverboard.remote.model.WheelPair
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The DIFFERENTIAL mix (`specs/rider-ui.md` 3.2): `left = v + s/2`, `right = v - s/2`, clamped.
 *
 * The property these tests exist for is the clamp's, not the arithmetic's. A mix that saturates must
 * not quietly lose the turn, so what is pinned at every edge is the DIFFERENCE between the two
 * wheels: it is the yaw, and it survives saturation while the travel the wheels share gives way.
 */
class DriveMixTest {

    private val max = DriveCmd.FULL_SCALE

    /** What the stick asked for, which the pair must still differ by. */
    private fun askedDifference(steer: Int) = 2 * (steer / 2)

    @Test
    fun `no turn tells both wheels the same thing`() {
        for (v in listOf(-max, -1000, 0, 1000, max)) {
            val pair = DriveMix.of(forward = v, steer = 0)
            assertEquals(WheelPair(v, v), pair, "v = $v")
            assertEquals(0, pair.difference)
        }
    }

    @Test
    fun `a turn on the spot runs the wheels opposite ways`() {
        val pair = DriveMix.of(forward = 0, steer = 1000)
        assertEquals(WheelPair(left = 500, right = -500), pair)
        assertEquals(1000, pair.difference)
    }

    @Test
    fun `positive steer runs the left wheel faster, which is a turn to the right`() {
        val pair = DriveMix.of(forward = 4000, steer = 2000)
        assertEquals(WheelPair(left = 5000, right = 3000), pair)
        assertTrue(pair.left > pair.right)
    }

    @Test
    fun `a mix that fits is the mix, untouched`() {
        for (v in listOf(-10_000, 0, 500, 10_000)) {
            for (s in listOf(-4000, -1, 0, 1, 4000)) {
                val pair = DriveMix.of(forward = v, steer = s)
                assertEquals(v + s / 2, pair.left, "v = $v, s = $s")
                assertEquals(v - s / 2, pair.right, "v = $v, s = $s")
            }
        }
    }

    /**
     * The case the clamp exists for: full forward and a full turn asks for 1.5 of full scale on the
     * left wheel. Both wheels come down by the same amount, so the pair still differs by the whole
     * turn and the machine turns exactly as hard as it was asked to.
     */
    @Test
    fun `one wheel clipping moves both and keeps the whole turn`() {
        val pair = DriveMix.of(forward = max, steer = max)

        assertEquals(max, pair.left)
        assertEquals(max - askedDifference(max), pair.right)
        assertEquals(askedDifference(max), pair.difference)
        assertTrue(abs(pair.right) <= max)
        // And the turn is not what a per-wheel clamp would have left: that one puts both wheels on
        // the same rail and straightens the machine out at full deflection.
        assertNotEquals(0, pair.difference)
    }

    @Test
    fun `clipping in reverse keeps the turn as well, and its direction`() {
        val pair = DriveMix.of(forward = -max, steer = max)

        assertEquals(-max, pair.right)
        assertEquals(-max + askedDifference(max), pair.left)
        assertEquals(askedDifference(max), pair.difference)
        assertTrue(pair.left > pair.right, "a right turn still runs the left wheel faster")
    }

    @Test
    fun `a forward demand past full scale is held at it`() {
        val pair = DriveMix.of(forward = 2 * max, steer = 0)
        assertEquals(WheelPair(max, max), pair)
    }

    /**
     * Over a grid of deflections, including every corner: the pair differs by exactly what the stick
     * asked for, and neither wheel is outside the wire's range. These two together are the mix's
     * contract.
     */
    @Test
    fun `every deflection keeps the asked difference inside the range`() {
        val axis = listOf(-max, -max / 2, -33, 0, 33, max / 3, max / 2, max)
        for (v in axis) {
            for (s in axis) {
                val pair = DriveMix.of(forward = v, steer = s)
                assertEquals(askedDifference(s), pair.difference, "v = $v, s = $s")
                assertTrue(abs(pair.left) <= max, "left out of range at v = $v, s = $s")
                assertTrue(abs(pair.right) <= max, "right out of range at v = $v, s = $s")
            }
        }
    }

    /** Reversing both axes mirrors the pair, with no asymmetry introduced by the offset. */
    @Test
    fun `the mix is symmetric under negation`() {
        for (v in listOf(0, 1000, max / 2, max)) {
            for (s in listOf(0, 1, 1000, max)) {
                val pair = DriveMix.of(forward = v, steer = s)
                val mirrored = DriveMix.of(forward = -v, steer = -s)
                assertEquals(WheelPair(left = -pair.left, right = -pair.right), mirrored, "v = $v, s = $s")
            }
        }
    }

    /**
     * The one case where the difference cannot be kept: a turn wider than the whole range. Two
     * demands are at most 2 x full scale apart, so there is no pair that differs by more; the wheels
     * go to opposite rails in the direction asked, which is the hardest turn the wire can state, and
     * the forward demand is what is lost.
     */
    @Test
    fun `a turn wider than the range goes to the rails and loses the forward demand`() {
        val right = DriveMix.of(forward = 10, steer = 1000, limit = 100)
        assertEquals(WheelPair(left = 100, right = -100), right)
        assertEquals(200, right.difference, "the widest difference two demands can carry")
        assertTrue(right.difference < askedDifference(1000), "less than asked, which no pair of demands could carry")

        val left = DriveMix.of(forward = 10, steer = -1000, limit = 100)
        assertEquals(WheelPair(left = -100, right = 100), left)
    }

    /** The stick itself cannot ask for that: both axes span one full scale, so `s/2` is half of it. */
    @Test
    fun `a full-scale stick never reaches the case the difference cannot hold`() {
        for (s in listOf(-max, -max + 1, max - 1, max)) {
            val pair = DriveMix.of(forward = 0, steer = s)
            assertEquals(askedDifference(s), pair.difference, "s = $s")
        }
    }
}
