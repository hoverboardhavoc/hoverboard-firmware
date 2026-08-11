package com.hoverboard.remote.link

/**
 * CRC-16/MODBUS, the single checksum the link frame uses.
 *
 * Config (matching crates/link/src/crc16.rs, which wraps crc::CRC_16_MODBUS): reflected polynomial
 * 0xA001, init 0xFFFF, no final xor. Written little-endian on the wire by [Frame.encode].
 *
 * Cross-language check anchors:
 *   compute("123456789".toByteArray())     == 0x4B37
 *   compute(byteArrayOf(1, 2, 3, 4, 5))    == 0xBB2A
 *   compute(ByteArray(0))                  == 0xFFFF
 */
object Crc16Modbus {

    /** CRC-16/MODBUS over `length` bytes of `data` starting at `offset`. Returns 0..0xFFFF. */
    fun compute(data: ByteArray, offset: Int = 0, length: Int = data.size - offset): Int {
        var crc = 0xFFFF
        val end = offset + length
        for (i in offset until end) {
            crc = crc xor (data[i].toInt() and 0xFF)
            repeat(8) {
                crc = if (crc and 0x0001 != 0) {
                    (crc ushr 1) xor 0xA001
                } else {
                    crc ushr 1
                }
            }
        }
        return crc and 0xFFFF
    }
}
