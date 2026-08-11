package com.hoverboard.remote.ble

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Which advertisement the scan connects to: remembered address first, configured name as the
 * fallback.
 */
class DeviceSelectionTest {

    private val target = LinkConfig.DEFAULT_DEVICE_NAME

    private fun candidate(
        address: String,
        advertised: String? = null,
        cached: String? = null,
    ) = ScanCandidate(address = address, advertisedName = advertised, cachedName = cached)

    // ---- name matching ---------------------------------------------------------------------

    @Test
    fun `the advertised name matches after the module's trailing padding is trimmed`() {
        // The CC2541 pads the AT+NAME slot, so the local name arrives with trailing spaces.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard        ")
        assertTrue(DeviceSelection.matchesName(seen, target))
    }

    @Test
    fun `the cached device name matches too`() {
        // Some scan records carry no local name; Android's cached name is then the only one there is.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = null, cached = "Hoverboard")
        assertTrue(DeviceSelection.matchesName(seen, target))
    }

    @Test
    fun `matching is case-sensitive so an unstaged board does not masquerade as a staged one`() {
        // The firmware's DEVICE_NAME default is the lowercase "hoverboard". A board still carrying
        // that default has NOT been staged, and must not silently satisfy a scan for "Hoverboard".
        val unstaged = candidate("AA:BB:CC:DD:EE:FF", advertised = "hoverboard")
        assertFalse(DeviceSelection.matchesName(unstaged, target))
    }

    @Test
    fun `a blank target matches nothing`() {
        // Otherwise a cleared name box would connect to the first nameless advertisement in range.
        val seen = candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard")
        assertFalse(DeviceSelection.matchesName(seen, "   "))
    }

    // ---- selection -------------------------------------------------------------------------

    @Test
    fun `with nothing remembered the first name match wins`() {
        val seen = listOf(
            candidate("11:11:11:11:11:11", advertised = "SomeoneElse"),
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
            candidate("33:33:33:33:33:33", advertised = "Hoverboard"),
        )
        assertEquals(
            "22:22:22:22:22:22",
            DeviceSelection.choose(seen, target, preferredAddress = null)?.address,
        )
    }

    @Test
    fun `the remembered address wins over an earlier board with the same name`() {
        // The case this exists for: two masters staged with the SAME name. By name alone the first
        // one advertising wins, which is a coin toss over which board you are about to drive.
        val seen = listOf(
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
            candidate("33:33:33:33:33:33", advertised = "Hoverboard"),
        )
        assertEquals(
            "33:33:33:33:33:33",
            DeviceSelection.choose(seen, target, preferredAddress = "33:33:33:33:33:33")?.address,
        )
    }

    @Test
    fun `the remembered address wins even when its advertised name looks wrong`() {
        // A phone can hold a stale cached GAP name against a MAC long after the board was renamed.
        // The address is the identity that does not drift, so it does not have to pass the name
        // filter as well.
        val seen = listOf(candidate("33:33:33:33:33:33", advertised = "OFFROAD"))
        assertEquals(
            "33:33:33:33:33:33",
            DeviceSelection.choose(seen, target, preferredAddress = "33:33:33:33:33:33")?.address,
        )
    }

    @Test
    fun `address comparison ignores case`() {
        val seen = listOf(candidate("AA:BB:CC:DD:EE:FF", advertised = "Hoverboard"))
        assertEquals(
            "AA:BB:CC:DD:EE:FF",
            DeviceSelection.choose(seen, target, preferredAddress = "aa:bb:cc:dd:ee:ff")?.address,
        )
    }

    @Test
    fun `an absent remembered address falls back to the name filter`() {
        // The board was swapped, or is simply not powered. The app must still find a board by name
        // rather than holding out for a MAC that is not there.
        val seen = listOf(
            candidate("11:11:11:11:11:11", advertised = "SomeoneElse"),
            candidate("22:22:22:22:22:22", advertised = "Hoverboard"),
        )
        assertEquals(
            "22:22:22:22:22:22",
            DeviceSelection.choose(seen, target, preferredAddress = "99:99:99:99:99:99")?.address,
        )
    }

    @Test
    fun `nothing matching yields nothing`() {
        val seen = listOf(candidate("11:11:11:11:11:11", advertised = "SomeoneElse"))
        assertNull(DeviceSelection.choose(seen, target, preferredAddress = "99:99:99:99:99:99"))
        assertNull(DeviceSelection.choose(emptyList(), target, preferredAddress = null))
    }
}
