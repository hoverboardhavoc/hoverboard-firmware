package com.hoverboard.stress.ble

/**
 * Transport-level BLE link configuration: the advertised device name to scan for. The stress firmware
 * (`stress-test/firmware/main.c`) brings the CC2541 up with `AT+NAME=hb-stress`, so the bench default
 * is "hb-stress". Override [deviceName] from the launch intent if the module advertises something else.
 */
data class LinkConfig(
    val deviceName: String = DEFAULT_DEVICE_NAME,
) {
    companion object {
        /** Advertised name the stress firmware sets (`AT_NAME` in `stress-test/firmware/main.c`). */
        const val DEFAULT_DEVICE_NAME: String = "hb-stress"
    }
}
