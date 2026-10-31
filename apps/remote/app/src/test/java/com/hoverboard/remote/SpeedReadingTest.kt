package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.SpeedFacts
import com.hoverboard.remote.model.SpeedReading
import com.hoverboard.remote.model.machineSpeed
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertInstanceOf
import org.junit.jupiter.api.Test

/**
 * The Ride panel's speed from `CYCLIC_STATE.wheel_rate` (`specs/remote-ui.md` 3.1a), on the rover's
 * wheel: 15 pole pairs and 165 mm. One edge per window (4096 in Q12) is 50 edges a second, 0.556
 * turns of the wheel, 288 mm/s, 1.037 km/h.
 */
class SpeedReadingTest {

    private fun board(rate: Int, side: Int? = LEFT, polePairs: Int? = 15, diameterMm: Int? = 165) =
        SpeedFacts(0x01, rate, polePairs, diameterMm, side)

    private fun kmh(reading: SpeedReading): Double = assertInstanceOf(SpeedReading.Kmh::class.java, reading).kmh

    @Test
    fun `one edge per window on the rover wheel is about one km per hour`() {
        assertEquals(ONE_EDGE_KMH, kmh(machineSpeed(listOf(board(Q12)))), TOLERANCE)
        assertEquals(-ONE_EDGE_KMH, kmh(machineSpeed(listOf(board(-Q12)))), TOLERANCE)
    }

    /** The right wheel's counter reads negative going forward, so its rate is mirrored. */
    @Test
    fun `a right wheel is folded into the forward sense`() {
        assertEquals(ONE_EDGE_KMH, kmh(machineSpeed(listOf(board(-Q12, side = RIGHT)))), TOLERANCE)
    }

    @Test
    fun `the machine speed is the mean of the boards in the forward sense`() {
        val straight = machineSpeed(listOf(board(Q12), board(-3 * Q12, side = RIGHT)))
        assertEquals(2 * ONE_EDGE_KMH, kmh(straight), TOLERANCE)
        val spinInPlace = machineSpeed(listOf(board(Q12), board(Q12, side = RIGHT)))
        assertEquals(0.0, kmh(spinInPlace), TOLERANCE)
    }

    /** An unset field is named, and a pair where one board lacks it shows no one-wheel figure. */
    @Test
    fun `an unset geometry field gives no speed and names the field`() {
        assertEquals(
            SpeedReading.NoSpeed(unread = emptyList(), unset = listOf(SpeedReading.POLE_PAIRS)),
            machineSpeed(listOf(board(Q12, polePairs = 0))),
        )
        assertEquals(
            SpeedReading.NoSpeed(unread = emptyList(), unset = listOf(SpeedReading.WHEEL_DIAMETER)),
            machineSpeed(listOf(board(Q12), board(Q12, side = RIGHT, diameterMm = 0))),
        )
    }

    @Test
    fun `an unread field gives no speed and is named as unread`() {
        assertEquals(
            SpeedReading.NoSpeed(unread = listOf(SpeedReading.WHEEL_DIAMETER, SpeedReading.SIDE), unset = emptyList()),
            machineSpeed(listOf(board(Q12, side = null, diameterMm = null))),
        )
    }

    @Test
    fun `no reporting board is silence`() {
        assertEquals(SpeedReading.Silent, machineSpeed(emptyList()))
    }

    @Test
    fun `tenths below ten km per hour and whole from ten`() {
        assertEquals(SpeedReading.Shown.Tenths(1.0), SpeedReading.Kmh(ONE_EDGE_KMH).shown)
        assertEquals(SpeedReading.Shown.Tenths(-3.3), SpeedReading.Kmh(-3.26).shown)
        assertEquals(SpeedReading.Shown.Tenths(9.9), SpeedReading.Kmh(9.94).shown)
        assertEquals(SpeedReading.Shown.Whole(10), SpeedReading.Kmh(9.96).shown)
        assertEquals(SpeedReading.Shown.Whole(-12), SpeedReading.Kmh(-12.4).shown)
        assertEquals(SpeedReading.Shown.Tenths(0.0), SpeedReading.Kmh(-0.02).shown, "never -0.0")
    }

    private companion object {
        const val Q12 = 4096
        const val LEFT = Fields.ControlSide.LEFT
        const val RIGHT = Fields.ControlSide.RIGHT

        /** 50 / 90 turns a second of a 165 mm wheel, in km/h. */
        const val ONE_EDGE_KMH = 50.0 / 90.0 * Math.PI * 165.0 * 0.0036
        const val TOLERANCE = 1e-9
    }
}
