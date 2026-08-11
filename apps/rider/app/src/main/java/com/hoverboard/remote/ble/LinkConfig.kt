package com.hoverboard.remote.ble

import com.hoverboard.protocol.l3.BROADCAST

/**
 * Transport-level link configuration: the app's own node id and the destination node id. These are
 * kept out of the codec (the shared protocol module stays a pure byte codec) and out of
 * the GATT contract, so a different deployment can retarget node ids without touching frame encoding.
 *
 * The advertised name to scan for is deliberately NOT here. It is user-settable and persisted, so it
 * is owned by [LinkSettings] and there is exactly one place to read it from; a second copy on this
 * object would be a compiled-in default that silently disagrees with what the user configured.
 * [DEFAULT_DEVICE_NAME] stays here as the seed [LinkSettings] falls back to when nothing is stored.
 *
 * Defaults for v1:
 *  - [appNodeId]: a fixed, reserved high id for the rider remote (node assignment is an open
 *    question in the firmware spec; 0xA0 is reserved for the app).
 *  - [boardDst]: broadcast (0xFF) for the single-target v1 case.
 */
data class LinkConfig(
    val appNodeId: Int = DEFAULT_APP_NODE_ID,
    val boardDst: Int = BROADCAST,
) {
    companion object {
        /** Reserved app/rider-remote node id for v1. */
        const val DEFAULT_APP_NODE_ID: Int = 0xA0

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
    }
}
