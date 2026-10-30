package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.BatteryReading.Unjudged
import com.hoverboard.remote.model.BatteryReading.Unjudged.Why
import com.hoverboard.remote.model.FloorOutside
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.batteryReading
import com.hoverboard.remote.model.floorOutsidePack
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * State of charge against the pack the machine states (`specs/rider-ui.md` 3.6d): a resting cell
 * voltage table per chemistry, read linearly between its eleven points, clamped at its ends and
 * multiplied by `board.battery.cells`. A machine whose cell count is unstated, or whose chemistry
 * is a byte the app has no table for, gets no figure rather than one against an assumed pack.
 */
class BatteryCurveTest {

    /** The rover's pack: eight cells. */
    private val rover = 8

    /** A stock 36 V hoverboard pack: ten cells. */
    private val stock = 10

    private val liIon = Fields.BatteryChemistry.LI_ION
    private val lev50 = Fields.BatteryChemistry.LEV50
    private val lto = Fields.BatteryChemistry.LTO

    @Test
    fun `each table's endpoints are its cell's times the cell count`() {
        assertEquals(3_000..4_200, BatteryCurve.packCentivolts(stock, liIon))
        assertEquals(2_200..3_280, BatteryCurve.packCentivolts(rover, lev50))
        assertEquals(1_360..2_160, BatteryCurve.packCentivolts(rover, lto))
        assertEquals(1f, BatteryCurve.fraction(42.0f, stock, liIon)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(30.0f, stock, liIon)!!, 0.0001f)
        assertEquals(1f, BatteryCurve.fraction(32.8f, rover, lev50)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(22.0f, rover, lev50)!!, 0.0001f)
        assertEquals(1f, BatteryCurve.fraction(21.6f, rover, lto)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(13.6f, rover, lto)!!, 0.0001f)
    }

    @Test
    fun `a point of the table reads as its own percent`() {
        // LEV50's 50% point is 3.73 V a cell, 29.84 V on eight.
        assertEquals(0.5f, BatteryCurve.fraction(29.84f, rover, lev50)!!, 0.0001f)
    }

    @Test
    fun `between two points the read is linear`() {
        // Li-ion 3.71 V a cell is halfway from 40% (3.68 V) to 50% (3.74 V): 45%.
        assertEquals(0.45f, BatteryCurve.fraction(37.1f, stock, liIon)!!, 0.0001f)
        // LTO 2.305 V a cell is halfway from 40% (2.29 V) to 50% (2.32 V).
        assertEquals(0.45f, BatteryCurve.fraction(18.44f, rover, lto)!!, 0.0001f)
    }

    /** The defect the cell count exists to end: one voltage, two packs, two states of charge. */
    @Test
    fun `the same voltage reads differently on two packs`() {
        // 30.0 V is 3.75 V a cell on eight (50% to 60%) and 3.0 V a cell on ten (empty).
        assertEquals(0.51666f, BatteryCurve.fraction(30.0f, rover, liIon)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(30.0f, stock, liIon)!!, 0.0001f)
    }

    @Test
    fun `voltages outside the table clamp`() {
        assertEquals(1f, BatteryCurve.fraction(35.0f, rover, liIon)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(20.0f, rover, liIon)!!, 0.0001f)
        assertEquals(1f, BatteryCurve.fraction(30.0f, rover, lto)!!, 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(10.0f, rover, lto)!!, 0.0001f)
    }

    @Test
    fun `a chemistry the app has no table for has no fraction and no endpoints`() {
        assertNull(BatteryCurve.fraction(30.0f, rover, 3))
        assertNull(BatteryCurve.packCentivolts(rover, 3))
    }

    @Test
    fun `a board reporting no pack voltage has no reading`() {
        assertEquals(BatteryReading.NoReading, batteryReading(telemetry(battery = 0), rover, liIon))
        assertEquals(BatteryReading.NoReading, batteryReading(null, rover, liIon))
    }

    /** An unstated pack is a voltage and no judgement, never a judgement against a guessed pack. */
    @Test
    fun `an unstated pack yields the volts and no charge`() {
        val telemetry = telemetry(battery = 2_900)
        assertEquals(Unjudged(29.0f, Why.NO_CELLS), batteryReading(telemetry, null, liIon))
        assertEquals(Unjudged(29.0f, Why.NO_CELLS), batteryReading(telemetry, 0, liIon))
    }

    @Test
    fun `an unknown chemistry yields the volts and no charge`() {
        val telemetry = telemetry(battery = 2_900)
        assertEquals(Unjudged(29.0f, Why.UNKNOWN_CHEMISTRY), batteryReading(telemetry, rover, 3))
    }

    /** An unread chemistry is its own reason, never reported as a byte the app does not know. */
    @Test
    fun `an unread chemistry yields the volts and no charge, as unread`() {
        val telemetry = telemetry(battery = 2_900)
        assertEquals(Unjudged(29.0f, Why.NO_CHEMISTRY), batteryReading(telemetry, rover, null))
    }

    @Test
    fun `a stated pack yields the volts, the fraction and the percent`() {
        val charge = batteryReading(telemetry(battery = 2_984), rover, lev50) as BatteryReading.Charge
        assertEquals(29.84f, charge.volts, 0.001f)
        assertEquals(0.5f, charge.fraction, 0.0001f)
        assertEquals(50, charge.percent)
    }

    @Test
    fun `low is the bottom fraction of the charge, whichever pack it is`() {
        // 15% is halfway from 10% to 20%: Li-ion 3.50 V a cell (28.00 V on eight), LEV50 3.54 V
        // (28.32 V on eight), each side of it by a margin the float read cannot cross.
        assertTrue(low(batteryReading(telemetry(battery = 2_800), rover, liIon)))
        assertFalse(low(batteryReading(telemetry(battery = 2_802), rover, liIon)))
        assertTrue(low(batteryReading(telemetry(battery = 2_830), rover, lev50)))
        assertFalse(low(batteryReading(telemetry(battery = 2_834), rover, lev50)))
    }

    @Test
    fun `a floor at or above a full pack is flagged`() {
        // Eight LTO cells are 21.60 V full, under the 24.00 V default floor.
        assertEquals(FloorOutside.AT_OR_ABOVE_FULL, floorOutsidePack(2_400, rover, lto))
        assertEquals(FloorOutside.AT_OR_ABOVE_FULL, floorOutsidePack(2_160, rover, lto))
        assertNull(floorOutsidePack(2_159, rover, lto))
    }

    @Test
    fun `a floor below an empty pack is flagged`() {
        // Ten Li-ion cells are 30.00 V empty.
        assertEquals(FloorOutside.BELOW_EMPTY, floorOutsidePack(2_999, stock, liIon))
        assertNull(floorOutsidePack(3_000, stock, liIon))
        assertNull(floorOutsidePack(2_400, rover, liIon))
    }

    @Test
    fun `a floor with nothing to judge it against is not flagged`() {
        assertNull(floorOutsidePack(0, rover, lto), "no floor")
        assertNull(floorOutsidePack(-1, rover, lto), "no floor")
        assertNull(floorOutsidePack(null, rover, lto), "unread floor")
        assertNull(floorOutsidePack(2_400, 0, lto), "no cell count")
        assertNull(floorOutsidePack(2_400, null, lto), "unread cell count")
        assertNull(floorOutsidePack(2_400, rover, 3), "unknown chemistry")
        assertNull(floorOutsidePack(2_400, rover, null), "unread chemistry")
    }

    private fun low(reading: BatteryReading) = (reading as BatteryReading.Charge).low

    /** A board sitting still at [battery] centivolts. */
    private fun telemetry(battery: Int) = TelemetryUi().merge(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = battery,
            mode = 0,
            fault = 0,
            flags = 0,
            peerDemand = 0,
            wheelPos = 0,
            wheelRate = 0,
            obs = CyclicObs(
                phasePeak = 0,
                phaseMean = 0,
                dutyOn = 0,
                bootTag = 1,
                chip = ChipTag.F103C8,
                odometer = 0L,
            ),
        ),
    )
}
