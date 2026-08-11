package com.hoverboard.remote.model

/** BLE connection lifecycle for the [com.hoverboard.remote.ble.HoverboardTransport]. */
enum class ConnectionState {
    /** Idle, not scanning, not connected. */
    DISCONNECTED,

    /** Actively scanning for the Pal peripheral. */
    SCANNING,

    /** Found the peripheral; connection in progress. */
    CONNECTING,

    /**
     * GATT is up and the characteristics are picked; the L3 attach (`NODE_HELLO`, then `ASSIGN` if
     * the board reports no address) is in flight. Not drivable yet: the app has no address of its
     * own and does not know the board's, and the board emits no telemetry until it holds one.
     */
    ATTACHING,

    /** Connected, services discovered, command/telemetry characteristics ready. */
    CONNECTED,

    /**
     * The BLE link is fine but the board would not attach: it never answered `NODE_HELLO`, or it
     * refused the `ASSIGN` (the firmware refuses one while armed). Terminal for this connect
     * attempt, deliberately: the retransmits already covered a dropped frame, so what is left is a
     * board-state problem that silent retrying would only hide.
     */
    ATTACH_FAILED,

    /** A scan/connect/link error occurred. */
    ERROR,
}
