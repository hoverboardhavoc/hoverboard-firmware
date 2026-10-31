package com.hoverboard.remote.ble

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertSame
import org.junit.jupiter.api.Test

/**
 * Folding the stream of advertisements a scan delivers into the list the picker renders.
 *
 * Every case here is a thing a real scan does to a naive list: the same board over and over, a
 * report that drops the name the last one carried, and signal strength that moves while the user
 * is reaching for the row.
 */
class ScanListTest {

    private fun sighting(
        address: String,
        advertised: String? = null,
        cached: String? = null,
        rssi: Int? = null,
    ) = ScanCandidate(
        address = address,
        advertisedName = advertised,
        cachedName = cached,
        rssi = rssi,
    )

    private fun fold(vararg sightings: ScanCandidate): List<ScanCandidate> =
        sightings.fold(emptyList()) { seen, one -> ScanList.merge(seen, one) }

    @Test
    fun `a board re-advertising takes one row, not one per advertisement`() {
        val seen = fold(
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -60),
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -62),
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -58),
        )
        assertEquals(1, seen.size)
        assertEquals(-58, seen.single().rssi)
    }

    @Test
    fun `two boards take two rows`() {
        val seen = fold(
            sighting("11:11:11:11:11:11", advertised = "Hoverboard"),
            sighting("22:22:22:22:22:22", advertised = "hb-offroad-m"),
        )
        assertEquals(listOf("11:11:11:11:11:11", "22:22:22:22:22:22"), seen.map { it.address })
    }

    @Test
    fun `an advertisement carrying no name does not erase the name already learned`() {
        // Devices alternate between a name-bearing advertisement and one that is not, so taking
        // the newest report wholesale makes names blink in and out under the user's finger.
        val seen = fold(
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -60),
            sighting("AA:BB:CC:DD:EE:FF", advertised = null, rssi = -61),
        )
        assertEquals("Hoverboard", seen.single().displayName)
        assertEquals(-61, seen.single().rssi)
    }

    @Test
    fun `a sighting with no signal strength keeps the last one that had it`() {
        val seen = fold(
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -60),
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = null),
        )
        assertEquals(-60, seen.single().rssi)
    }

    @Test
    fun `rows keep the order they were first heard in`() {
        // The user is aiming at a moving list. A board that goes quiet and comes back must not
        // jump to the end, and a strong late arrival must not push the others down.
        val seen = fold(
            sighting("11:11:11:11:11:11", advertised = "first", rssi = -80),
            sighting("22:22:22:22:22:22", advertised = "second", rssi = -40),
            sighting("11:11:11:11:11:11", advertised = "first", rssi = -79),
        )
        assertEquals(listOf("first", "second"), seen.map { it.displayName })
    }

    @Test
    fun `the same address in the other case is the same board`() {
        val seen = fold(
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard"),
            sighting("aa:bb:cc:dd:ee:ff", advertised = "Hoverboard"),
        )
        assertEquals(1, seen.size)
    }

    @Test
    fun `an identical re-sighting returns the very same list`() {
        // So a board re-advertising an unchanged record costs no recomposition.
        val first = fold(sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -60))
        val again = ScanList.merge(
            first,
            sighting("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard", rssi = -60),
        )
        assertSame(first, again)
    }
}
