package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.model.machineOdometer
import com.hoverboard.remote.model.odometerReading
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertInstanceOf
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The odometer figures the panel shows (`specs/rider-ui.md` 3.6c).
 *
 * Three properties carry this type. The machine's figure is the MEAN of the two wheels and not
 * their sum, because both wheels cover the same ground and a sum reports twice the travel. Zero
 * is a READING: a board whose wheel has never turned has reported, and folding that into the
 * absent case would hide a board that is working. And a distance appears only where the board has
 * been told BOTH halves of its wheel's geometry, because a count scaled by a guessed hub is a
 * confidently wrong number.
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

        assertEquals(MachineTravel.Edges(1_000_000), machineOdometer(listOf(master, slave)))
    }

    /** Two wheels that disagree average, rather than one of them being taken for the machine. */
    @Test
    fun `two unequal wheels average`() {
        val master = OdometerReading.Edges(1_000_000)
        val slave = OdometerReading.Edges(1_400_000)

        assertEquals(MachineTravel.Edges(1_200_000), machineOdometer(listOf(master, slave)))
    }

    /**
     * One wheel is not a machine. A board that has not reported is indistinguishable from one
     * that is not fitted, so the figure is withheld rather than taken from the wheel that did
     * report.
     */
    @Test
    fun `a board that has not reported leaves no machine figure`() {
        val master = OdometerReading.Edges(1_000_000)

        assertNull(machineOdometer(listOf(master, OdometerReading.Silent)))
        assertNull(machineOdometer(listOf(OdometerReading.Silent, master)))
    }

    /** A pair of resting wheels still averages: zero is a reading on both sides too. */
    @Test
    fun `a pair that has never moved averages to zero rather than to nothing`() {
        val resting = OdometerReading.Edges(0)

        assertEquals(MachineTravel.Edges(0), machineOdometer(listOf(resting, resting)))
    }

    /**
     * The worked conversion. 15 pole pairs is 90 edges per wheel revolution, and a 165 mm wheel
     * covers `PI * 165` = 518.4 mm in one, so a single edge is 5.76 mm.
     */
    @Test
    fun `a board told its wheel reports the distance those edges cover`() {
        val geometry = WheelGeometry(polePairs = 15, diameterMm = 165)

        val reading = odometerReading(telemetry(edges = 90), geometry)

        assertEquals(OdometerReading.Travelled(edges = 90, millimetres = 518), reading)
        assertEquals(6L, geometry.millimetres(1), "5.76 mm, to the nearest millimetre")
        assertEquals(51_836L, geometry.millimetres(9_000), "a hundred revolutions")
    }

    /**
     * A lifetime total at the wire word's ceiling still converts: a saturated `u32` of edges on
     * that hub is about 2.5e10 mm, ten times what an Int holds, which is why the arithmetic is in
     * Long millimetres.
     */
    @Test
    fun `a saturated edge count converts without overflowing`() {
        val geometry = WheelGeometry(polePairs = 15, diameterMm = 165)

        val reading = odometerReading(telemetry(edges = 4_294_967_295L), geometry)

        val travelled = assertInstanceOf(OdometerReading.Travelled::class.java, reading)
        assertEquals(24_737_235_786L, travelled.millimetres)
    }

    /**
     * Half a geometry is no geometry. Either fact alone fixes nothing about the scale, and a
     * missing pole-pair count filled in from what the fleet is believed to carry would scale every
     * figure by whatever that hub really is.
     */
    @Test
    fun `a board told only half its wheel cannot convert`() {
        assertNull(WheelGeometry.of(polePairs = 15, diameterMm = null))
        assertNull(WheelGeometry.of(polePairs = null, diameterMm = 165))
        assertNull(WheelGeometry.of(polePairs = null, diameterMm = null))
    }

    /**
     * Zero is the unset value of both fields, and it is also the divisor that would fault. 0 is
     * not a motor and 0 mm is not a wheel, so neither makes a geometry.
     */
    @Test
    fun `a zero in either half is unset and never divides`() {
        assertNull(WheelGeometry.of(polePairs = 0, diameterMm = 165))
        assertNull(WheelGeometry.of(polePairs = 15, diameterMm = 0))
    }

    /** A board with no geometry reports the edges it counted, and says they are edges. */
    @Test
    fun `a board that has not been told its wheel reports edges`() {
        val reading = odometerReading(telemetry(edges = 1_284_310), geometry = null)

        assertEquals(OdometerReading.Edges(1_284_310), reading)
    }

    /**
     * The machine figure is a distance only when EVERY reporting board converted. Two rows in
     * different units cannot be averaged into one number.
     */
    @Test
    fun `the machine figure is a distance only when both wheels converted`() {
        val converted = OdometerReading.Travelled(edges = 90, millimetres = 518)
        val counted = OdometerReading.Edges(90)

        assertEquals(MachineTravel.Distance(518), machineOdometer(listOf(converted, converted)))
        assertEquals(MachineTravel.Edges(90), machineOdometer(listOf(converted, counted)))
        assertEquals(MachineTravel.Edges(90), machineOdometer(listOf(counted, converted)))
    }

    /** And the distance mean is a mean: both wheels covered the same ground. */
    @Test
    fun `two converted wheels average their distances rather than adding them`() {
        val master = OdometerReading.Travelled(edges = 90, millimetres = 1_000)
        val slave = OdometerReading.Travelled(edges = 94, millimetres = 1_400)

        assertEquals(MachineTravel.Distance(1_200), machineOdometer(listOf(master, slave)))
    }

    /** A silent board withholds the machine figure whatever the other board could convert. */
    @Test
    fun `a converted wheel beside a silent board still leaves no machine figure`() {
        val master = OdometerReading.Travelled(edges = 90, millimetres = 518)

        assertNull(machineOdometer(listOf(master, OdometerReading.Silent)))
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
            wheelPos = 0,
            wheelRate = 0,
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
