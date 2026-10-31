package com.hoverboard.remote.ble

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Which advertisement a connect accepts: the pinned address and nothing else, or, with no board
 * picked, the first advertisement answering to the configured name.
 */
class DeviceSelectionTest {

    private val name = LinkConfig.DEFAULT_DEVICE_NAME
    private val byName = BoardTarget.ByName(name)

    private fun candidate(
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

    /** The one line `BleHoverboardTransport.findDevice` applies to the scan, over a list. */
    private fun firstMatch(seen: List<ScanCandidate>, target: BoardTarget): ScanCandidate? =
        seen.firstOrNull { DeviceSelection.matches(it, target) }

    // ---- name matching ---------------------------------------------------------------------

    @Test
    fun `the advertised name matches after the module's trailing padding is trimmed`() {
        // The CC2541 pads the AT+NAME slot, so the local name arrives with trailing spaces.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard        ")
        assertTrue(DeviceSelection.matchesName(seen, name))
    }

    @Test
    fun `the cached device name matches too`() {
        // Some scan records carry no local name; Android's cached name is then the only one there is.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = null, cached = "Hoverboard")
        assertTrue(DeviceSelection.matchesName(seen, name))
    }

    @Test
    fun `matching is case-sensitive so an unstaged board does not masquerade as a staged one`() {
        // The firmware's DEVICE_NAME default is the lowercase "hoverboard". A board still carrying
        // that default has NOT been staged, and must not silently satisfy a scan for "Hoverboard".
        val unstaged = candidate("AA:BB:CC:DD:EE:FF", advertised = "hoverboard")
        assertFalse(DeviceSelection.matchesName(unstaged, name))
    }

    @Test
    fun `a blank target matches nothing`() {
        // Otherwise a cleared name box would connect to the first nameless advertisement in range.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard")
        assertFalse(DeviceSelection.matchesName(seen, "   "))
    }

    // ---- the name path ---------------------------------------------------------------------

    @Test
    fun `with no board picked the first name match wins`() {
        val seen = listOf(
            candidate("11:11:11:11:11:11", advertised = "SomeoneElse"),
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
            candidate("33:33:33:33:33:33", advertised = "Hoverboard"),
        )
        assertEquals("22:22:22:22:22:22", firstMatch(seen, byName)?.address)
    }

    @Test
    fun `nothing answering to the name yields nothing`() {
        val seen = listOf(candidate("11:11:11:11:11:11", advertised = "SomeoneElse"))
        assertNull(firstMatch(seen, byName))
        assertNull(firstMatch(emptyList(), byName))
    }

    // ---- the pinned path -------------------------------------------------------------------

    @Test
    fun `the pinned address wins over an earlier board with the same name`() {
        // The case the picker exists for: two masters staged with the SAME name. By name alone the
        // first one advertising wins, which is a coin toss over which board you are about to drive.
        val seen = listOf(
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
            candidate("33:33:33:33:33:33", advertised = "Hoverboard"),
        )
        val pinned = BoardTarget.Pinned("33:33:33:33:33:33")
        assertEquals("33:33:33:33:33:33", firstMatch(seen, pinned)?.address)
    }

    @Test
    fun `a renamed board is still the pinned board`() {
        // The requirement the address keying is for: the stored name is not a precondition. A board
        // whose AT+NAME changed, or whose cached GAP name the phone holds wrongly, is the same
        // hardware at the same MAC, and the pin does not consult either name.
        val seen = listOf(candidate("33:33:33:33:33:33", advertised = "OFFROAD", cached = "old-name"))
        val pinned = BoardTarget.Pinned("33:33:33:33:33:33")
        assertEquals("33:33:33:33:33:33", firstMatch(seen, pinned)?.address)
    }

    @Test
    fun `an absent pinned board is never traded for one that matches the name`() {
        // The owner's requirement, stated as a test: out of range is a connect that waits, not a
        // reason to drive a different machine. The old policy fell back to the name here.
        val seen = listOf(
            candidate("11:11:11:11:11:11", advertised = "Hoverboard"),
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
        )
        assertNull(firstMatch(seen, BoardTarget.Pinned("99:99:99:99:99:99")))
    }

    @Test
    fun `address comparison ignores case`() {
        val seen = listOf(candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard"))
        assertEquals(
            "AA:BB:CC:DD:EE:FF",
            firstMatch(seen, BoardTarget.Pinned("aa:bb:cc:dd:ee:ff"))?.address,
        )
    }

    // ---- what a row is called ----------------------------------------------------------------

    @Test
    fun `the advertised name is what a row shows, over the one Android cached`() {
        val renamed = candidate("AA:BB:CC:DD:EE:FF", advertised = "hb-offroad-m", cached = "Hoverboard")
        assertEquals("hb-offroad-m", renamed.displayName)
    }

    @Test
    fun `a board advertising no usable name has none to show`() {
        // Padding only, and an empty cached name: both are nothing, and the row falls back to the
        // placeholder rather than rendering a blank line where a name should be.
        assertNull(candidate("AA:BB:CC:DD:EE:FF", advertised = "   ", cached = "").displayName)
        assertNull(candidate("AA:BB:CC:DD:EE:FF").displayName)
    }
}
