package com.hoverboard.app.ble

/**
 * Transport-level BLE link configuration: the advertised device name to scan for. Kept out of any
 * codec so a different deployment can retarget the advertised name without touching the transport.
 *
 * The L2/L3 node ids (app node id, board dst) belong to the protocol mirror built in slice 2; this
 * connect-only slice needs nothing but the name.
 *
 * Default for the bench: the master firmware's `ble::bring_up` advertises **"hb-s5a"** (its `BLE_NAME`,
 * `crates/firmware/src/main.rs`). If the module is in a stale data-mode name from a prior run, scan for
 * whatever it actually advertises by overriding [deviceName].
 */
data class LinkConfig(
    val deviceName: String = DEFAULT_DEVICE_NAME,
) {
    companion object {
        /** Advertised name the master firmware sets (matches the firmware's `BLE_NAME`). */
        const val DEFAULT_DEVICE_NAME: String = "hb-s5a"
    }
}
