package com.hoverboard.remote.link

/**
 * Wire constants for the hoverboard-firmware inter-board / BLE link frame.
 *
 * Mirrors crates/link/src/frame.rs. Frame layout (all multi-byte fields little-endian):
 *
 *   offset 0  SOF     0x5A
 *   offset 1  ver     PROTO_VER (1)
 *   offset 2  opcode  see [Opcode]
 *   offset 3  src     source node id
 *   offset 4  dst     destination node id (0xFF = broadcast)
 *   offset 5  len     payload length (0..255)
 *   offset 6  payload len bytes
 *   offset 6+len  crc CRC-16/MODBUS over bytes 0..(6+len), little-endian
 */
object LinkConstants {
    const val SOF: Byte = 0x5A
    const val PROTO_VER: Byte = 0x01
    const val HEADER_LEN = 6
    const val CRC_LEN = 2
    const val MAX_PAYLOAD = 255
    const val MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN
    const val BROADCAST: Int = 0xFF
}
