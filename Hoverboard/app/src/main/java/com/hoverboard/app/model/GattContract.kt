package com.hoverboard.app.model

import java.util.UUID

/**
 * The BLE link to the master board's onboard CC2541 module (specs/ble_link.md).
 *
 * The CC2541 runs vendor "AT-firmware" exposing a transparent-UART pipe with **Write Without
 * Response + Notify**: whatever the phone writes lands on the master board's USART; whatever the
 * master sends comes back as notifications. The on-the-wire protocol carried by those bytes is the
 * project's L2/L3 frame (built in slice 2); this contract is only the BLE-link metadata.
 *
 * Service + characteristic UUIDs are **not hardcoded**: per spec we discover them at runtime by
 * walking the GATT for a characteristic with both WRITE_WITHOUT_RESPONSE and NOTIFY (the bench
 * module exposes a 0x1001 write / 0x1002 notify pair, runtime-discovered). The constants below are
 * documented common defaults (FFE0/FFE1) — a "look here first" hint, not load-bearing.
 */
object GattContract {

    /**
     * Common-default service UUID for CC2541-class transparent-UART modules (HM-10 / FFE0).
     * Discovery is preferred; this is just the hint.
     */
    val PREFERRED_SERVICE_UUID: UUID = uuidFrom16Bit(0xFFE0)

    /** Common-default write+notify characteristic UUID for the same modules. */
    val PREFERRED_CHARACTERISTIC_UUID: UUID = uuidFrom16Bit(0xFFE1)

    /** Bluetooth SIG base UUID: 16-bit IDs expand into 0000xxxx-0000-1000-8000-00805F9B34FB. */
    private fun uuidFrom16Bit(short16: Int): UUID =
        UUID.fromString("0000%04X-0000-1000-8000-00805F9B34FB".format(short16))
}
