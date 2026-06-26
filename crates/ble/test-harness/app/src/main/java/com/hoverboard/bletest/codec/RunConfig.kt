package com.hoverboard.bletest.codec

/**
 * One run fixes a config point (`specs/ble.md`, "What it sweeps"). A sweep ramps one axis (rate, payload,
 * write type, MTU, connection priority) until loss appears. The module-side `AT+CON_INTERVAL` is fixed at
 * bring-up (recorded, not swept). This is the intent the host passes via `am start --es/--ei` extras.
 */
data class RunConfig(
    /** Which bench phone (resolved by `model:`, not address, see [Devices]). */
    val device: String,
    /** The advertised name to scan for (the firmware's `AT+NAME`). Per-run unique, so a stale/cached
     *  advert can't be matched by accident (the module persists its name; phones cache it). */
    val name: String,
    val mode: String, // "loopback" (Tier 3 board) or "fake" (Tier 2 local echo)
    val payload: Int, // payload bytes per packet (4, 16, 64, 128, MTU-cap, 255)
    val rate: Int, // offered packets/sec (the inter-frame gap is 1/rate); ramped to find the knee
    val durSec: Int, // run duration
    val write: WriteType, // WRITE_WITHOUT_RESPONSE vs WRITE
    val mtu: Int, // requested ATT MTU (23..247)
    val priority: ConnPriority,
    val out: String, // result JSON filename in the app's external files dir
)

enum class WriteType { NO_RESPONSE, RESPONSE;
    companion object {
        fun parse(s: String?): WriteType = when (s?.lowercase()) {
            "res", "response", "write" -> RESPONSE
            else -> NO_RESPONSE // default: WRITE_WITHOUT_RESPONSE (preferred, per spec)
        }
    }
}

enum class ConnPriority { HIGH, BALANCED, LOW_POWER;
    companion object {
        fun parse(s: String?): ConnPriority = when (s?.lowercase()) {
            "balanced" -> BALANCED
            "low", "low_power", "lowpower" -> LOW_POWER
            else -> HIGH
        }
    }
}
