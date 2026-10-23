package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.machineOdometerEdges
import com.hoverboard.remote.model.odometerReading
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertInstanceOf
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The odometer figures the panel shows (`specs/rider-ui.md` 3.6c).
 *
 * Two properties carry this type. The machine's figure is the MEAN of the two wheels and not
 * their sum, because both wheels cover the same ground and a sum reports twice the travel. And
 * zero is a READING: a board whose wheel has never turned has reported, and folding that into the
 * absent case would hide a board that is working.
 */
class OdometerReadingTest {

    @Test
    fun `a reporting board's figure is the edge count it sent`() {
        val reading = odometerReading(telemetry(edges = 1_284_310))

        assertEquals(OdometerReading.Edges(1_284_310), reading)
    }

    /** Nothing has arrived, which is the one way to have no figure. */
    @Test
    fun `a board that has sent nothing is silent`() {
        assertEquals(OdometerReading.Silent, odometerReading(null))
        assertEquals(OdometerReading.Silent, odometerReading(TelemetryUi()))
    }

    /**
     * Zero edges is not silence. A board that has reported a wheel which has never turned is
     * saying something true about the machine, and a display that hid it would be hiding a
     * working board behind the wording for a missing one.
     */
    @Test
    fun `zero edges is a reading and not an absence`() {
        val reading = odometerReading(telemetry(edges = 0))

        val edges = assertInstanceOf(OdometerReading.Edges::class.java, reading)
        assertEquals(0L, edges.edges)
    }

    /** A count past the signed 32-bit range survives, which is why the word is held as a Long. */
    @Test
    fun `a count past two billion is carried whole`() {
        val reading = odometerReading(telemetry(edges = 4_000_000_000L))

        assertEquals(OdometerReading.Edges(4_000_000_000L), reading)
    }

    /**
     * The machine's figure is the MEAN. Both wheels cover the same ground, so the sum would say
     * the machine had gone twice as far as it has. That is the opposite of the current section,
     * where the two boards draw from one pack and their draws do add.
     */
    @Test
    fun `the machine figure is the mean of the two wheels and not their sum`() {
        val master = OdometerReading.Edges(1_000_000)
        val slave = OdometerReading.Edges(1_000_000)

        assertEquals(1_000_000L, machineOdometerEdges(listOf(master, slave)))
    }

    /** Two wheels that disagree average, rather than one of them being taken for the machine. */
    @Test
    fun `two unequal wheels average`() {
        val master = OdometerReading.Edges(1_000_000)
        val slave = OdometerReading.Edges(1_400_000)

        assertEquals(1_200_000L, machineOdometerEdges(listOf(master, slave)))
    }

    /**
     * One wheel is not a machine. A board that has not reported is indistinguishable from one
     * that is not fitted, so the figure is withheld rather than taken from the wheel that did
     * report.
     */
    @Test
    fun `a board that has not reported leaves no machine figure`() {
        val master = OdometerReading.Edges(1_000_000)

        assertNull(machineOdometerEdges(listOf(master, OdometerReading.Silent)))
        assertNull(machineOdometerEdges(listOf(OdometerReading.Silent, master)))
    }

    /** A pair of resting wheels still averages: zero is a reading on both sides too. */
    @Test
    fun `a pair that has never moved averages to zero rather than to nothing`() {
        val resting = OdometerReading.Edges(0)

        assertEquals(0L, machineOdometerEdges(listOf(resting, resting)))
    }

    private fun telemetry(edges: Long) = TelemetryUi().merge(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = 2_900,
            mode = 3,
            fault = 0,
            flags = 0,
            peerDemand = 0,
            obs = CyclicObs(
                phasePeak = 0,
                phaseMean = 0,
                dutyOn = 0,
                bootTag = 0,
                chip = ChipTag.F130C8,
                odometer = edges,
            ),
        ),
    )
}
