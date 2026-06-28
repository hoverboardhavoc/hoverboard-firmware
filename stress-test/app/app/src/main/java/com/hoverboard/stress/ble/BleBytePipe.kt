package com.hoverboard.stress.ble

import kotlinx.coroutines.flow.Flow

/**
 * A raw bidirectional BLE byte stream (a copy of `Hoverboard/.../net/l3/BleWalk.kt`'s `BleBytePipe`,
 * lifted out of L3): [write] bytes to the module's write char (0x1001) and collect its notifications
 * (0x1002) from [incoming]. Frame boundaries are NOT preserved (the CC2541 bridge coalesces and
 * re-chunks); [com.hoverboard.stress.net.l2.BleStreamTransport] supplies the framing.
 */
interface BleBytePipe {
    /** Write a byte chunk to the GATT write char (the implementation may split to the ATT MTU). */
    suspend fun write(bytes: ByteArray)

    /** Notification bytes from the GATT notify char, in arrival order. */
    val incoming: Flow<ByteArray>
}
