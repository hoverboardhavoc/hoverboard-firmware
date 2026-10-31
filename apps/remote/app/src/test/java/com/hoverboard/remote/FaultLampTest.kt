package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.LampFacts
import com.hoverboard.remote.model.lampColumns
import com.hoverboard.remote.model.shownLamps
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The fault lamp grid's rules (`specs/remote-ui.md` 3.1b), on the pure model alone: which lamps
 * apply on which machine, that a set bit is never hidden, the column count, and the fault-to-sheet
 * mapping against `CyclicState.ARM_BLOCKING_BITS`.
 */
class FaultLampTest {

    private fun one(imu: Int?) = listOf(LampFacts(0x01, imu))
    private fun two(imu: Int?) = listOf(LampFacts(0x01, imu), LampFacts(0x02, imu))

    /** Every lamp against one and two motor controllers and `imu.model` 0, not 0 and unread. */
    @Test
    fun `each lamp applies where the firmware can light it`() {
        for (count in listOf(1, 2)) {
            for (imu in listOf(0, 1, null)) {
                val facts = if (count == 1) one(imu) else two(imu)
                val shown = shownLamps(facts, facts.associate { it.address to 0 })
                assertEquals(count == 2, FaultLamp.LINK in shown, "LINK on $count boards")
                // Unread counts as fitted, so an unread fact never hides a lamp.
                assertEquals(imu != 0, FaultLamp.TILT in shown, "TILT with imu.model $imu")
                assertEquals(imu != 0, FaultLamp.IMU in shown, "IMU with imu.model $imu")
                for (lamp in listOf(FaultLamp.LATCH, FaultLamp.HALL, FaultLamp.CAL, FaultLamp.PERIOD, FaultLamp.ARM)) {
                    assertTrue(lamp in shown, "$lamp on every motor controller")
                }
                assertFalse(FaultLamp.LATCH_2 in shown, "LATCH 2 never applies")
            }
        }
    }

    /** TILT and IMU apply when either board has an IMU: a rigid follower has none, its leader does. */
    @Test
    fun `an imu on one board shows the imu lamps for the machine`() {
        val shown = shownLamps(listOf(LampFacts(0x01, 1), LampFacts(0x02, 0)), emptyMap())
        assertTrue(FaultLamp.TILT in shown && FaultLamp.IMU in shown)
    }

    @Test
    fun `latch 2 is absent until its bit is set`() {
        assertFalse(FaultLamp.LATCH_2 in shownLamps(two(1), mapOf(0x01 to 0, 0x02 to 0)))
        assertTrue(FaultLamp.LATCH_2 in shownLamps(two(1), mapOf(0x01 to 0, 0x02 to CyclicState.EV_LATCH_B)))
    }

    /** Every bit of ARM_BLOCKING_BITS, set on a board for which its lamp does not apply, shows the lamp. */
    @Test
    fun `a set bit is never hidden`() {
        // One board with no IMU: LINK, TILT, IMU and LATCH 2 do not apply.
        for (lamp in FaultLamp.entries) {
            assertTrue(lamp in shownLamps(one(0), mapOf(0x01 to lamp.bit)), "$lamp set on a board it does not apply on")
        }
        // A bit set on a reporting board the walk has not classed is shown too.
        assertTrue(FaultLamp.HALL in shownLamps(emptyList(), mapOf(0x05 to CyclicState.EV_MOTOR_HALL)))
    }

    @Test
    fun `the lamps keep the grid's order`() {
        assertEquals(FaultLamp.entries.filter { it != FaultLamp.LATCH_2 }, shownLamps(two(1), emptyMap()))
    }

    /** Four columns for up to eight lamps and five for nine, so the grid is never more than two rows. */
    @Test
    fun `four columns, or five when nine lamps are shown`() {
        for (n in 1..8) assertEquals(4, lampColumns(n), "$n lamps")
        assertEquals(5, lampColumns(9))
        val all = shownLamps(two(1), mapOf(0x01 to CyclicState.EV_LATCH_B))
        assertEquals(9, all.size)
        assertTrue((all.size + lampColumns(all.size) - 1) / lampColumns(all.size) <= 2)
    }

    /**
     * The fault-to-sheet mapping: every bit of ARM_BLOCKING_BITS maps to exactly one lamp and every
     * lamp to one bit, each with a title, three bodies and the code "F" plus the bit; the bits that
     * are not lamps map to none.
     */
    @Test
    fun `every arm-blocking bit is exactly one lamp with its sheet`() {
        assertEquals(0x0F37, CyclicState.ARM_BLOCKING_BITS)
        val blocking = (0 until 16).filter { CyclicState.ARM_BLOCKING_BITS and (1 shl it) != 0 }
        assertEquals(blocking.size, FaultLamp.entries.size)
        for (n in blocking) {
            val lamp = FaultLamp.of(1 shl n)
            checkNotNull(lamp) { "bit $n has no lamp" }
            assertEquals("F$n", lamp.code)
            assertEquals(1, FaultLamp.entries.count { it.bit == 1 shl n })
            val strings = listOf(lamp.title, lamp.whenShows, lamp.blocks, lamp.fix)
            assertEquals(4, strings.toSet().size, "$lamp has four distinct strings")
            strings.forEach { assertNotEquals(0, it) }
        }
        assertEquals(CyclicState.ARM_BLOCKING_BITS, FaultLamp.entries.fold(0) { acc, l -> acc or l.bit })
        for (n in listOf(3, 6, 7, 12, 13, 14, 15)) assertNull(FaultLamp.of(1 shl n), "bit $n is not a lamp")
    }

    /** Only the two latches are held after their bit clears. */
    @Test
    fun `only the latches are held`() {
        assertEquals(setOf(FaultLamp.LATCH, FaultLamp.LATCH_2), FaultLamp.entries.filter { it.held }.toSet())
    }
}
