package com.hoverboard.app.model

/** BLE connection lifecycle for the [com.hoverboard.app.ble.HoverboardTransport]. */
enum class ConnectionState {
    /** Idle, not scanning, not connected. */
    DISCONNECTED,

    /** Actively scanning for the board's advertised module. */
    SCANNING,

    /** Found the peripheral; connection + service discovery in progress. */
    CONNECTING,

    /** Connected, services discovered, the write/notify characteristics are ready. */
    CONNECTED,

    /** A scan/connect/link error occurred. */
    ERROR,
}
