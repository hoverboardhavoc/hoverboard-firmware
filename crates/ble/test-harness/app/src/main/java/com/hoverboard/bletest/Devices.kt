package com.hoverboard.bletest

/**
 * The bench phones, the harness central MUST support BOTH (`specs/ble.md` / the bench notes). They are
 * identified by `model:` (Build.MODEL), NOT by address: DHCP drifts the IPs/ports, but the model is
 * stable, so the host `run-sweep.sh` resolves the right `adb` endpoint at runtime and the `--es device`
 * intent extra selects by this logical name.
 *
 * The critical per-device BLE quirk is [autoConnect]: the ASUS ROG (Android 8) takes ~133 s on a DIRECT
 * connect, and the only thing that has been made to work is `autoConnect=true`. The central reads this
 * flag from the resolved device and passes it into `connectGatt`. (Confirmed: bench-overview.md + the
 * `ble-app-loop-working` memory note.)
 */
data class Devices(
    /** The logical name the `--es device` intent extra selects ("OnePlus" / "Asus"). */
    val name: String,
    /** Build.MODEL the central matches against, so a wrong-phone launch is caught. */
    val model: String,
    /**
     * BLE GATT connect mode. true = autoConnect (slow background connect, but the ONLY mode the ASUS ROG
     * succeeds with); false = direct connect (fast, fine on modern stacks like the OnePlus 8).
     */
    val autoConnect: Boolean,
) {
    companion object {
        /** OnePlus 8, model IN2013 (adb at 192.168.0.201, random wireless-debug port). Direct connect. */
        val ONEPLUS_8 = Devices(name = "OnePlus", model = "IN2013", autoConnect = false)

        /**
         * ASUS ROG Phone, model ASUS_Z01RD (adb at 192.168.0.141:5555, Android 8). MUST use
         * autoConnect=true: direct connect takes ~133 s on its old BLE stack and is unreliable.
         */
        val ASUS_ROG = Devices(name = "Asus", model = "ASUS_Z01RD", autoConnect = true)

        val ALL = listOf(ONEPLUS_8, ASUS_ROG)

        /** Resolve a device by the logical `--es device` name (case-insensitive). */
        fun byName(name: String): Devices? =
            ALL.firstOrNull { it.name.equals(name, ignoreCase = true) }

        /** Resolve a device by Build.MODEL, so the central can confirm it is running on a known phone. */
        fun byModel(model: String): Devices? =
            ALL.firstOrNull { it.model.equals(model, ignoreCase = true) }
    }
}
