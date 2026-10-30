package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.RideSlot
import com.hoverboard.remote.model.RideSlots
import com.hoverboard.remote.model.SlotSide
import com.hoverboard.remote.model.TripBaseline
import com.hoverboard.remote.model.angleText
import com.hoverboard.remote.model.batteryLow
import com.hoverboard.remote.model.litSegments
import com.hoverboard.remote.model.phaseFill
import com.hoverboard.remote.model.ringReading
import com.hoverboard.remote.model.rideSlots
import com.hoverboard.remote.model.totalKmText
import com.hoverboard.remote.model.tripKmText
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** The Ride screen's readings (`specs/rider-ui.md` 3.1 and 3.1d), on the pure functions alone. */
class RideReadingsTest {

    @Test
    fun `lit segments are the percent over ten, rounded up`() {
        assertEquals(0, litSegments(0))
        assertEquals(1, litSegments(1))
        assertEquals(1, litSegments(10))
        assertEquals(2, litSegments(11))
        assertEquals(10, litSegments(100))
    }

    private fun charge(percent: Int, volts: Float = 37f) =
        BatteryReading.Charge(volts = volts, fraction = percent / 100f, low = percent <= 15)

    /** The lowest Charge, the lower address on a tie, else the lowest-addressed reading. */
    @Test
    fun `the ring takes the lowest charge, else the lowest-addressed reading`() {
        val readings = mapOf(0x01 to charge(72), 0x02 to charge(68))
        assertEquals(0x02 to charge(68), ringReading(listOf(0x01, 0x02), readings::getValue))

        val tie = mapOf(0x02 to charge(50), 0x01 to charge(50))
        assertEquals(0x01, ringReading(listOf(0x02, 0x01), tie::getValue).first)

        val one = mapOf(0x01 to BatteryReading.NoReading, 0x02 to charge(90))
        assertEquals(0x02 to charge(90), ringReading(listOf(0x01, 0x02), one::getValue), "a Charge beats no reading")

        val unjudged = BatteryReading.Unjudged(30f, BatteryReading.Unjudged.Why.NO_CELLS)
        val none = mapOf(0x02 to BatteryReading.NoReading, 0x01 to unjudged)
        assertEquals(0x01, ringReading(listOf(0x02, 0x01), none::getValue).first)

        assertEquals(null to BatteryReading.NoReading, ringReading(emptyList()) { BatteryReading.NoReading })
    }

    /** Low at the 15 percent edge and at the floor's edge; a floor of 0 or unread adds nothing. */
    @Test
    fun `low is the bottom 15 percent or at or under the floor`() {
        assertTrue(batteryLow(charge(15), floorCentivolts = null))
        assertFalse(batteryLow(charge(16), floorCentivolts = null))

        assertTrue(batteryLow(charge(60, volts = 33.00f), floorCentivolts = 3_300), "at the floor")
        assertFalse(batteryLow(charge(60, volts = 33.01f), floorCentivolts = 3_300), "a centivolt above it")
        assertFalse(batteryLow(charge(60, volts = 1f), floorCentivolts = 0), "a floor of 0 adds nothing")
        assertFalse(batteryLow(charge(60, volts = 1f), floorCentivolts = null), "an unread floor adds nothing")
        assertFalse(batteryLow(BatteryReading.NoReading, floorCentivolts = 3_300))
    }

    /** The fill against the clamped limit: a stored 0 scales to 1 A, 50,000 to 40 A; no fill unread. */
    @Test
    fun `the phase fill is the figure over the clamped limit`() {
        val amps = CurrentReading.Amps(motor = 6f, supply = 1f)
        assertEquals(0.4f, phaseFill(amps, 15_000)!!, 1e-6f)
        // 0 mA clamps to 1 A: half an amp is half a bar.
        assertEquals(0.5f, phaseFill(CurrentReading.Amps(0.5f, 0f), 0)!!, 1e-6f)

        assertEquals(0.5f, phaseFill(CurrentReading.Amps(20f, 0f), 50_000)!!, 1e-6f, "50 A clamps to 40 A")
        assertEquals(1f, phaseFill(CurrentReading.Amps(60f, 0f), 15_000)!!, "clamped to full")
        assertNull(phaseFill(amps, null), "no fill with the limit unread")
        assertNull(phaseFill(CurrentReading.Coasting, 15_000))
    }


    /** Rounded half away from zero, and a value that rounds to zero prints 0.0. */
    @Test
    fun `angle text is to a tenth and never minus zero`() {
        assertEquals("0.0", angleText(-4))
        assertEquals("-0.1", angleText(-5))
        assertEquals("2.3", angleText(225))
        assertEquals("0.0", angleText(0))
        assertEquals("-12.3", angleText(-1_234))
    }

    /** The three slot cases. */
    @Test
    fun `boards are slotted by side, alone, or by address`() {
        val left = Fields.ControlSide.LEFT
        val right = Fields.ControlSide.RIGHT
        assertEquals(
            RideSlots(RideSlot(0x02, SlotSide.LEFT, true), RideSlot(0x01, SlotSide.RIGHT, true)),
            rideSlots(listOf(0x01, 0x02), mapOf(0x01 to right, 0x02 to left)::get),
        )
        // One board: on its side, or the left slot labelled by address while the side is unread.
        val alone = listOf(0x01)
        assertEquals(RideSlots(null, RideSlot(0x01, SlotSide.RIGHT, true)), rideSlots(alone, mapOf(0x01 to right)::get))
        assertEquals(RideSlots(RideSlot(0x01, SlotSide.LEFT, true), null), rideSlots(alone, mapOf(0x01 to left)::get))
        assertEquals(RideSlots(RideSlot(0x01, SlotSide.LEFT, false), null), rideSlots(listOf(0x01)) { null })
        // Anything else: the two lowest addresses, left then right, by address.
        val byAddress = RideSlots(RideSlot(0x01, SlotSide.LEFT, false), RideSlot(0x02, SlotSide.RIGHT, false))
        assertEquals(byAddress, rideSlots(listOf(0x02, 0x01), mapOf(0x01 to left, 0x02 to left)::get), "both the same")
        assertEquals(byAddress, rideSlots(listOf(0x01, 0x02), mapOf(0x01 to right)::get), "a side unread")
        val three = listOf(0x03, 0x02, 0x01)
        assertEquals(byAddress, rideSlots(three, mapOf(0x01 to left, 0x02 to right)::get), "three boards")
        assertEquals(RideSlots.NONE, rideSlots(emptyList()) { null })
    }

    /**
     * Set by the first distance, kept over a reconnect to the same pick, dropped on a new pick,
     * moved down by a lower total.
     */
    @Test
    fun `the trip baseline`() {
        val pick = "AA:BB"
        var b = TripBaseline.NONE.next(pick, MachineTravel.Edges(100))
        assertNull(b.millimetres, "edges are no distance")
        b = b.next(pick, MachineTravel.Distance(5_000_000))
        assertEquals(5_000_000, b.millimetres, "the first distance sets it")
        b = b.next(pick, MachineTravel.Distance(8_100_000))
        assertEquals(3_100_000, b.trip(MachineTravel.Distance(8_100_000)))
        // A reconnect to the same pick: the link drops (no total), then comes back.
        b = b.next(pick, null).next(pick, MachineTravel.Distance(8_200_000))
        assertEquals(5_000_000, b.millimetres, "kept over a reconnect to the same pick")
        // A board rebooted and lost travel: the baseline follows the total down.
        b = b.next(pick, MachineTravel.Distance(4_000_000))
        assertEquals(4_000_000, b.millimetres)
        assertEquals(0, b.trip(MachineTravel.Distance(4_000_000)))
        // A new pick drops it, and its first distance sets it again.
        b = b.next("CC:DD", MachineTravel.Distance(9_000_000))
        assertEquals(9_000_000, b.millimetres)
        assertNull(TripBaseline.NONE.trip(MachineTravel.Distance(1)))
    }

    @Test
    fun `distances print to a tenth, the total whole from 10 km`() {
        assertEquals("3.1", tripKmText(3_100_000))
        assertEquals("9.9", totalKmText(9_940_000))
        assertEquals("10", totalKmText(9_960_000))
        assertEquals("413", totalKmText(412_700_000))
        assertEquals("412.7", tripKmText(412_700_000))
    }
}
