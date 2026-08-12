package com.hoverboard.remote.ble

/**
 * Compiled-in link defaults for the rider remote.
 *
 * There are no node ids here. The app's own address and the board's address are **session facts**,
 * not configuration: the app is a transient controller (`specs/l3.md`), so it takes a guest address
 * from `0x80..0xFE` in the board's `NODE_HELLO` reply at first contact and addresses that board by
 * the id the same exchange reported (assigning one if the board reports none). Both are held by
 * [BleHoverboardTransport] for the life of one connection and dropped on disconnect; a compiled-in
 * copy would be a second, wrong answer to a question the wire already answers.
 *
 * The advertised name to scan for is likewise not a field here. It is user-settable and persisted,
 * so it is owned by [LinkSettings] and there is exactly one place to read it from;
 * [DEFAULT_DEVICE_NAME] is the seed [LinkSettings] falls back to when nothing is stored.
 */
object LinkConfig {

    /**
     * The name a board advertises out of the box, and so the name the app looks for until the
     * user changes it.
     *
     * Case matters. The scan filter is an exact string comparison (after trimming the padding
     * the CC2541 adds), and the firmware's own store default for `DEVICE_NAME` is the LOWERCASE
     * `"hoverboard"`. A board left unstaged therefore does NOT match this, by design: the bench
     * procedure stages the name explicitly (specs/ble-session.md in the firmware repo), which is
     * also the only way to tell two boards apart on a scan list.
     */
    const val DEFAULT_DEVICE_NAME: String = "Hoverboard"

    /**
     * The rider command cadence: how often [CommandPump] re-sends the held
     * [com.hoverboard.remote.model.RiderCommand].
     *
     * It lives here rather than inside the transport because it is not only the transport's number
     * any more. The ViewModel has to know it too, to size the window it holds the link open for
     * after disarming (see [com.hoverboard.remote.MainViewModel.disconnect]); two copies of a
     * cadence, one of them wrong, is how a "disarm before you drop the link" guarantee quietly
     * becomes "usually".
     *
     * 10 Hz, chosen against two separate ceilings:
     *  - `DRIVE_CMD` decays after 200 ms (`crates/linkctl/src/lib.rs:57`), so the cadence has to be
     *    strictly inside that, with margin for a lost frame on a link that never retransmits.
     *  - The CC2541 meters its UART at 9600 baud. 30 Hz overran its BLE-to-UART buffer (drops and
     *    heat); 5 Hz had three-drops-in-a-row failures around RF dropouts.
     *
     * Note the cost of a tick doubled when the app started sending a `DRIVE_CMD` alongside the
     * `INPUTS` mirror: 11 and 12 bytes on the wire respectively, so 230 B/s of the module's ~960 B/s
     * budget, before telemetry coming the other way.
     */
    const val SEND_INTERVAL_MS: Long = 100L
}
