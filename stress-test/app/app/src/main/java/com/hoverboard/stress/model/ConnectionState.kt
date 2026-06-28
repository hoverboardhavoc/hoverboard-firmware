package com.hoverboard.stress.model

/** BLE connection lifecycle for the [com.hoverboard.stress.ble.BleStressTransport]. */
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
