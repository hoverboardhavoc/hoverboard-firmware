package com.hoverboard.protocol.imu

import com.hoverboard.protocol.imu.Orientation.Face
import com.hoverboard.protocol.imu.Orientation.FrameError
import com.hoverboard.protocol.imu.Orientation.Heading
import com.hoverboard.protocol.imu.Orientation.Pose
import com.hoverboard.protocol.imu.Orientation.Refusal
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

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
        assertEquals(Refusal.ACCEL_MIRRORED, Orientation.check(listOf(-1, 1, 1, -1, 1, -1), UNSET))
        assertEquals(Refusal.GYRO_MIRRORED, Orientation.check(listOf(-1, 1, -1, -1, 1, 1), UNSET))
        // One staged index against an otherwise-unset map: the reference fills the rest, and
        // flipping one sign of a rotation always mirrors it.
        assertEquals(Refusal.ACCEL_MIRRORED, Orientation.check(listOf(1, 0, 0, 0, 0, 0), UNSET))
        // An all-unset map is the reference, which is legal.
        assertNull(Orientation.check(List(6) { 0 }, UNSET))
        assertEquals(Refusal.NOT_A_SIGN, Orientation.check(listOf(2, 1, 1, 1, 1, 1), UNSET))
    }

    @Test
    fun twoDifferentRotationsOnTheTwoVectorsAreLegalButAreNoPose() {
        // The accel as the reference, the gyro as a half turn about Z: each triple is a rotation.
        val mixed = listOf(-1, 1, -1, -1, -1, 1)
        assertNull(Orientation.check(mixed, UNSET))
        assertNull(Orientation.poseOf(UNSET, mixed))
    }

    @Test
    fun thereAreTwentyFourPosesFourPerFaceAndEachStagesALegalFrame() {
        assertEquals(24, Orientation.POSES.size)
        assertEquals(24, Orientation.POSES.toSet().size)
        for (face in Face.entries) assertEquals(4, Orientation.headingsFor(face).size, "$face")
        val frames = Orientation.POSES.map { Orientation.frameOf(it) }
        // 24 distinct frames: the picker reaches every rotation the two fields express.
        assertEquals(24, frames.toSet().size)
        for ((pose, frame) in Orientation.POSES.zip(frames)) {
            assertNull(Orientation.frameError(frame.signs, frame.roles), "$pose stages $frame")
            assertNull(Orientation.check(frame.signs, frame.roles), "$pose stages $frame")
            assertEquals(frame.signs.subList(0, 3), frame.signs.subList(3, 6), "$pose: accel and gyro alike")
            assertEquals(pose, Orientation.poseOf(frame.roles, frame.signs), "$pose reads back")
        }
    }

    @Test
    fun aHeadingAlongTheFaceNormalIsNoPose() {
        assertThrows<IllegalArgumentException> { Pose(Face.COMPONENT_UP, Heading.COMPONENT_SIDE) }
        assertThrows<IllegalArgumentException> { Pose(Face.STOCK_FORWARD_EDGE_DOWN, Heading.STOCK_REAR) }
    }

    /** The stock pose stages the measured family yaw explicitly, under the compiled roles. */
    @Test
    fun theStockPoseIsTheFamilyYaw() {
        val stock = Pose(Face.COMPONENT_UP, Heading.STOCK_FORWARD)
        assertEquals(stock, Orientation.STOCK_POSE)
        val yaw = listOf(-1, -1, 1, -1, -1, 1)
        assertEquals(Orientation.Frame(listOf(3, 2), yaw), Orientation.frameOf(stock))
        assertEquals(stock, Orientation.poseOf(listOf(3, 2), yaw))
        // Unset roles resolve to the compiled ones, so a flat pose staged over them reads back.
        assertEquals(stock, Orientation.poseOf(UNSET, yaw))
    }

    /**
     * An unset sign is no pose: the fallback it runs is the stock image's map, which on this family
     * is the board turned component side down.
     */
    @Test
    fun unsetSignsAreNoPoseAndTheFallbackIsComponentSideDown() {
        assertNull(Orientation.poseOf(UNSET, List(6) { 0 }))
        assertNull(Orientation.poseOf(UNSET, listOf(-1, -1, 1, -1, -1, 0)))
        assertEquals(Pose(Face.COMPONENT_DOWN, Heading.STOCK_FORWARD), Orientation.fallbackPose)
        assertTrue(Orientation.runsFallback(UNSET, List(6) { 0 }))
        assertTrue(Orientation.runsFallback(listOf(3, 2), List(6) { 0 }))
        assertFalse(Orientation.runsFallback(listOf(1, 3), List(6) { 0 }))
        assertFalse(Orientation.runsFallback(UNSET, listOf(-1, -1, 1, -1, -1, 0)))
    }

    /**
     * A flat face keeps chip Z up; a board on an edge puts a long board axis vertical, so UP is chip X
     * or chip Y and never Z. The rover's mount as `specs/imu.md` reads its model today (long axis
     * vertical, the component normal along the axle) is UP = X, PITCH_RATE = Z.
     */
    @Test
    fun anEdgeDownPoseNeverHasChipZUp() {
        for (pose in Orientation.POSES) {
            val up = Orientation.frameOf(pose).roles[0]
            val expected = when (pose.face) {
                Face.COMPONENT_UP, Face.COMPONENT_DOWN -> 3
                Face.STOCK_FORWARD_EDGE_DOWN, Face.STOCK_REAR_EDGE_DOWN -> 1
                Face.STOCK_LEFT_EDGE_DOWN, Face.STOCK_RIGHT_EDGE_DOWN -> 2
            }
            assertEquals(expected, up, "$pose")
        }
        for (face in listOf(Face.STOCK_FORWARD_EDGE_DOWN, Face.STOCK_REAR_EDGE_DOWN)) {
            for (heading in listOf(Heading.STOCK_LEFT, Heading.STOCK_RIGHT)) {
                assertEquals(listOf(1, 3), Orientation.frameOf(Pose(face, heading)).roles, "$face, $heading")
            }
        }
    }

    /** Each pose's board-to-body matrix is a proper rotation taking the face up and the heading forward. */
    @Test
    fun boardToBodyTakesTheFaceUpAndTheHeadingForward() {
        for (pose in Orientation.POSES) {
            val r = Orientation.boardToBody(pose)
            assertEquals(1, det(r), "$pose")
            fun apply(v: List<Int>) = r.map { row -> row.zip(v).sumOf { (a, b) -> a * b } }
            assertEquals(listOf(0, 0, 1), apply(pose.face.up), "$pose")
            assertEquals(listOf(1, 0, 0), apply(pose.heading.forward), "$pose")
        }
        val identity = listOf(listOf(1, 0, 0), listOf(0, 1, 0), listOf(0, 0, 1))
        assertEquals(identity, Orientation.boardToBody(Orientation.STOCK_POSE))
    }

    /** The composed frame, rebuilt as `body = P * S * chip`, equals board-to-body after the reference. */
    @Test
    fun theFactoredFrameIsTheComposition() {
        val ref = Orientation.STANDARD_CHIP_TO_BOARD
        for (pose in Orientation.POSES) {
            val f = Orientation.frameOf(pose)
            val r = Orientation.boardToBody(pose)
            val composed = r.map { row -> row.mapIndexed { j, v -> v * ref[j] } }
            assertEquals(composed, frameMatrix(f.roles, f.signs.subList(0, 3)), "$pose")
        }
    }

    /** The 3x3 matrix `body = M * chip` the roles and signs make, built row by row from the roles. */
    private fun frameMatrix(roles: List<Int>, triple: List<Int>): List<List<Int>> {
        val up = roles[0] - 1
        val pitch = roles[1] - 1
        val forward = (0..2).single { it != up && it != pitch }
        return listOf(forward, pitch, up).map { chip -> List(3) { c -> if (c == chip) triple[chip] else 0 } }
    }

    private fun det(m: List<List<Int>>): Int =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

    @Test
    fun aFrameIsARotationExactlyWhenItsMatrixHasDeterminantPlusOne() {
        val unit = listOf(-1, 1)
        val unitTriples = unit.flatMap { a -> unit.flatMap { b -> unit.map { c -> listOf(a, b, c) } } }
        var legal = 0
        for (up in 1..3) for (pitch in 1..3) {
            if (up == pitch) continue
            for (t in unitTriples) {
                val expected = det(frameMatrix(listOf(up, pitch), t)) == 1
                assertEquals(expected, Orientation.frameIsRotation(listOf(up, pitch), t), "roles [$up, $pitch], $t")
                if (expected) legal++
            }
            assertFalse(Orientation.frameIsRotation(listOf(up, pitch), listOf(1, 0, 1)), "only +-1 are signs")
        }
        // Half of the 8 sign triples under each of the 6 role pairs: the 24 rotations of the cube.
        assertEquals(24, legal)
        for (bad in listOf(listOf(0, 2), listOf(4, 2), listOf(2, 2))) {
            assertTrue(unitTriples.none { Orientation.frameIsRotation(bad, it) }, "roles $bad are not a frame")
        }
        for (t in allTriples) {
            assertEquals(Orientation.tripleIsRotation(t), Orientation.frameIsRotation(Orientation.DEFAULT_ROLES, t))
        }
    }

    @Test
    fun theBoardRefusesTheRolesThenTheAccelThenTheGyro() {
        val unset = listOf(0, 0)
        assertNull(Orientation.frameError(List(6) { 0 }, unset), "nothing staged is the stock frame")
        assertEquals(Orientation.DEFAULT_ROLES, Orientation.effectiveRoles(unset))
        assertEquals(listOf(1, 2), Orientation.effectiveRoles(listOf(1, 0)))
        // UP = X with PITCH_RATE unset (Y) is a transposition, so the reference half-turn about Y
        // (product +1) is now a reflection on both triples, and the accel is named first.
        assertEquals(FrameError.ACCEL, Orientation.frameError(List(6) { 0 }, listOf(1, 0)))
        assertNull(Orientation.frameError(listOf(1, 1, -1, -1, 1, 1), listOf(1, 0)))
        assertEquals(FrameError.GYRO, Orientation.frameError(listOf(1, 1, -1, -1, 1, -1), listOf(1, 0)))
        // A staged role equal to the other one's fallback names one chip axis twice, which is
        // refused before either triple is looked at.
        assertEquals(FrameError.ROLES, Orientation.frameError(listOf(1, 1, 1, 1, 1, 1), listOf(2, 0)))
        assertEquals(FrameError.ROLES, Orientation.frameError(List(6) { 0 }, listOf(4, 0)))
    }

    /**
     * The client judges a sign map with the roles the board holds, never under assumed defaults: the
     * same signs are legal under one pair of roles and refused under another, and a bad role pair is
     * named as such rather than as a mirrored triple.
     */
    @Test
    fun theClientCheckReadsTheSignsThroughTheStoredRoles() {
        // The reference half-turn about Y is legal flat and a reflection under the transposition
        // UP = X, PITCH_RATE = Y, on both triples, accel named first.
        assertNull(Orientation.check(Orientation.REFERENCE, UNSET))
        assertEquals(Refusal.ACCEL_MIRRORED, Orientation.check(Orientation.REFERENCE, listOf(1, 2)))
        // UP = X, PITCH_RATE = Z is a 3-cycle (FORWARD = Y): parity +1, so a product +1 triple holds.
        assertNull(Orientation.check(listOf(-1, 1, -1, -1, 1, -1), listOf(1, 3)))
        assertEquals(Refusal.GYRO_MIRRORED, Orientation.check(listOf(-1, 1, -1, -1, 1, 1), listOf(1, 3)))
        assertEquals(Refusal.ROLES, Orientation.check(List(6) { 0 }, listOf(3, 3)))
        assertEquals(Refusal.ROLES, Orientation.check(List(6) { 0 }, listOf(0, 3)))
        assertEquals(Refusal.ROLES, Orientation.check(List(6) { 0 }, listOf(4, 2)))
        // A bad sign is named before the roles are looked at.
        assertEquals(Refusal.NOT_A_SIGN, Orientation.check(listOf(2, 1, 1, 1, 1, 1), listOf(3, 3)))
    }

    private companion object {
        /** The roles field with nothing staged. */
        val UNSET = listOf(0, 0)
    }
}
