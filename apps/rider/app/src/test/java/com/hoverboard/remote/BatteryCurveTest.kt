package com.hoverboard.remote

import com.hoverboard.remote.model.BatteryCurve
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.batteryReading
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * State of charge against the pack the machine states, per cell: 4.2 V full, 3.0 V empty.
 *
 * The curve used to be a single 29.4 V / 21.7 V pair compiled into the app, which is a seven-cell
 * pack and neither machine here: the rover's eight-cell LEV50 runs to 33.6 V and a stock pack is
 * higher again, so every real reading clamped to 100%. The endpoints now come from
 * `board.pack_cells`, and a machine that has not stated its pack gets no figure at all rather than
 * one against an assumed one.
 */
class BatteryCurveTest {

    /** The rover's pack: eight cells, 24.0 V to 33.6 V. */
    private val rover = 8

    /** A stock 36 V hoverboard pack: ten cells, 30.0 V to 42.0 V. */
    private val stock = 10

    @Test
    fun `a pack is full at its cell count times the full cell`() {
        assertEquals(33.6f, BatteryCurve.fullVolts(rover), 0.001f)
        assertEquals(42.0f, BatteryCurve.fullVolts(stock), 0.001f)
        assertEquals(1f, BatteryCurve.fraction(33.6f, rover), 0.0001f)
        assertEquals(1f, BatteryCurve.fraction(42.0f, stock), 0.0001f)
    }

    @Test
    fun `a pack is empty at its cell count times the empty cell`() {
        assertEquals(24.0f, BatteryCurve.emptyVolts(rover), 0.001f)
        assertEquals(30.0f, BatteryCurve.emptyVolts(stock), 0.001f)
        assertEquals(0f, BatteryCurve.fraction(24.0f, rover), 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(30.0f, stock), 0.0001f)
    }

    /** The defect the field exists to end: one voltage, two packs, two different states of charge. */
    @Test
    fun `the same voltage reads differently on two packs`() {
        assertEquals(0.5f, BatteryCurve.fraction(28.8f, rover), 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(28.8f, stock), 0.0001f)
    }

    @Test
    fun `the map between the endpoints is linear`() {
        // 24.0 + 0.25 * (33.6 - 24.0) = 26.4 V.
        assertEquals(0.25f, BatteryCurve.fraction(26.4f, rover), 0.0001f)
    }

    @Test
    fun `voltages outside the pack's own band clamp`() {
        assertEquals(1f, BatteryCurve.fraction(35.0f, rover), 0.0001f)
        assertEquals(0f, BatteryCurve.fraction(20.0f, rover), 0.0001f)
    }

    @Test
    fun `a board reporting no pack voltage has no reading`() {
        assertEquals(BatteryReading.NoReading, batteryReading(telemetry(battery = 0), rover))
        assertEquals(BatteryReading.NoReading, batteryReading(null, rover))
    }

    /** An unstated pack is a voltage and no judgement, never a judgement against a guessed pack. */
    @Test
    fun `an unstated pack yields the volts and no charge`() {
        val telemetry = telemetry(battery = 2_900)
        assertEquals(BatteryReading.Unjudged(29.0f), batteryReading(telemetry, null))
        assertEquals(BatteryReading.Unjudged(29.0f), batteryReading(telemetry, 0))
    }

    @Test
    fun `a stated pack yields the volts, the fraction and the percent`() {
        val charge = batteryReading(telemetry(battery = 2_640), rover) as BatteryReading.Charge
        assertEquals(26.4f, charge.volts, 0.001f)
        assertEquals(0.25f, charge.fraction, 0.0001f)
        assertEquals(25, charge.percent)
    }

    @Test
    fun `low is the bottom fraction of the pack, whichever pack it is`() {
        // 15% of the rover's 9.6 V band is 25.44 V, and of the stock pack's 12.0 V band 31.8 V.
        assertTrue((batteryReading(telemetry(battery = 2_540), rover) as BatteryReading.Charge).low)
        assertTrue(!(batteryReading(telemetry(battery = 2_550), rover) as BatteryReading.Charge).low)
        assertTrue((batteryReading(telemetry(battery = 3_170), stock) as BatteryReading.Charge).low)
        assertTrue(!(batteryReading(telemetry(battery = 3_190), stock) as BatteryReading.Charge).low)
    }

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
