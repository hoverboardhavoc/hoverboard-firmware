package com.hoverboard.protocol.imu

import com.hoverboard.protocol.imu.Orientation.Refusal
import com.hoverboard.protocol.imu.Orientation.Rotation
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [Orientation]'s rule, checked against its definition rather than against examples: the
 * `RustSourceDriftTest` pins the rule's TEXT to the firmware's, and this pins the Kotlin's
 * BEHAVIOUR to what that text says.
 */
class OrientationTest {

    private val signs = listOf(-1, 0, 1)
    private val allTriples = signs.flatMap { a -> signs.flatMap { b -> signs.map { c -> listOf(a, b, c) } } }

    @Test
    fun aTripleIsARotationExactlyWhenEverySignIsUnitAndTheirProductIsPlusOne() {
        for (t in allTriples) {
            val expected = t.all { it != 0 } && t[0] * t[1] * t[2] == 1
            assertEquals(expected, Orientation.tripleIsRotation(t), "triple $t")
        }
        assertFalse(Orientation.tripleIsRotation(listOf(2, 1, 1)), "only +-1 are signs")
    }

    @Test
    fun theOfferedRotationsAreExactlyTheProperDiagonalOnes() {
        val proper = allTriples.filter { Orientation.tripleIsRotation(it) }.toSet()
        assertEquals(4, proper.size)
        assertEquals(proper, Orientation.ROTATIONS.map { it.triple }.toSet())
        for (r in Orientation.ROTATIONS) {
            assertEquals(r.triple + r.triple, r.signs, "$r stages the same rotation on accel and gyro")
            assertNull(Orientation.check(r.signs), "$r is legal")
            assertEquals(r, Rotation.of(r.signs))
        }
    }

    @Test
    fun theReferenceMapIsTheHalfTurnAboutY() {
        assertEquals(Rotation.HALF_TURN_Y, Rotation.of(Orientation.REFERENCE))
    }

    @Test
    fun anUnsetIndexRunsTheReferenceAndASetOneRunsAsStaged() {
        assertEquals(Orientation.REFERENCE, Orientation.effective(List(6) { 0 }))
        assertEquals(
            listOf(1, 1, -1, -1, -1, -1),
            Orientation.effective(listOf(1, 0, 0, 0, -1, 0)),
        )
    }

    @Test
    fun aMirroredFrameIsRefusedAfterTheUnsetRuleNotBefore() {
        // Negating only the up axis: the classic wrong fix for a board that reads upside down.
        assertEquals(Refusal.ACCEL_MIRRORED, Orientation.check(listOf(-1, 1, 1, -1, 1, -1)))
        assertEquals(Refusal.GYRO_MIRRORED, Orientation.check(listOf(-1, 1, -1, -1, 1, 1)))
        // One staged index against an otherwise-unset map: the reference fills the rest, and
        // flipping one sign of a rotation always mirrors it.
        assertEquals(Refusal.ACCEL_MIRRORED, Orientation.check(listOf(1, 0, 0, 0, 0, 0)))
        // An all-unset map is the reference, which is legal.
        assertNull(Orientation.check(List(6) { 0 }))
        assertEquals(Refusal.NOT_A_SIGN, Orientation.check(listOf(2, 1, 1, 1, 1, 1)))
    }

    @Test
    fun twoDifferentRotationsOnTheTwoVectorsAreLegalButAreNoNamedRotation() {
        val mixed = Rotation.IDENTITY.triple + Rotation.HALF_TURN_Z.triple
        assertNull(Orientation.check(mixed))
        assertNull(Rotation.of(mixed))
        assertTrue(Orientation.ROTATIONS.none { it.signs == mixed })
    }
}
