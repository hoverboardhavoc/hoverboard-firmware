package com.hoverboard.remote.model

import java.util.UUID

/**
 * The BLE link to the master hoverboard board's onboard CC2541 module.
 *
 * The CC2541 runs vendor "AT-firmware" that exposes a single transparent-UART
 * characteristic with **Write Without Response + Notify**. Whatever the phone writes
 * lands on the master board's USART; whatever the master sends comes back as notifications.
 *
 * The on-the-wire protocol carried by those bytes is OUR link frame
 * ([com.hoverboard.remote.link]). This contract is just the BLE-link metadata; the advertised
 * name and node ids live in [com.hoverboard.remote.ble.LinkConfig].
 *
 * Service + characteristic UUIDs are **not hardcoded**: per spec we discover them at
 * runtime by walking the GATT for a characteristic with both WRITE_WITHOUT_RESPONSE
 * and NOTIFY. The constants below are documented common defaults for this module class
 * (FFE0/FFE1) — useful as a fallback prefer-this-one hint, but not load-bearing.
 */
object GattContract {

    /**
     * Common-default service UUID for CC2541-class transparent-UART modules
     * (HM-10 / FFE0). Discovery is preferred; this is just the "look here first" hint.
     */
    val PREFERRED_SERVICE_UUID: UUID = uuidFrom16Bit(0xFFE0)

    /**
     * Common-default write+notify characteristic UUID for the same modules.
     * Used as a tiebreaker if multiple characteristics on a service qualify.
     */
    val PREFERRED_CHARACTERISTIC_UUID: UUID = uuidFrom16Bit(0xFFE1)

    /**
     * Bluetooth SIG base UUID. Standard 16-bit IDs expand into the
     * 0000xxxx-0000-1000-8000-00805F9B34FB form.
     */
    private fun uuidFrom16Bit(short16: Int): UUID =
        UUID.fromString("0000%04X-0000-1000-8000-00805F9B34FB".format(short16))
}
