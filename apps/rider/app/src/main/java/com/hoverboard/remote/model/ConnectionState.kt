package com.hoverboard.remote.model

/** BLE connection lifecycle for the [com.hoverboard.remote.ble.HoverboardTransport]. */
enum class ConnectionState {
    /** Idle, not scanning, not connected. */
    DISCONNECTED,

    /** Actively scanning for the Pal peripheral. */
    SCANNING,

    /** Found the peripheral; connection in progress. */
    CONNECTING,

    /** Connected, services discovered, command/telemetry characteristics ready. */
    CONNECTED,

    /** A scan/connect/link error occurred. */
    ERROR,
}
