package com.hoverboard.stress.net.l2

/**
 * CRC-16/MODBUS, a byte-for-byte mirror of the firmware's `base::crc16::modbus`
 * (`crc::CRC_16_MODBUS`): reflected poly 0xA001, init 0xFFFF, refin/refout true, no final xor;
 * little-endian on the wire. The L2 stream framer and the config-store records both use it, so the
 * checksum the app computes is identical to the firmware's.
 */
object Crc16 {

    private const val INIT = 0xFFFF
    private const val REFLECTED_POLY = 0xA001

    /** CRC-16/MODBUS over `bytes[from until from+len]`, returned in the low 16 bits. */
    fun modbus(bytes: ByteArray, from: Int = 0, len: Int = bytes.size - from): Int {
        var crc = INIT
        for (i in from until from + len) {
            crc = crc xor (bytes[i].toInt() and 0xFF)
            repeat(BITS_PER_BYTE) {
                crc = if (crc and 1 != 0) (crc ushr 1) xor REFLECTED_POLY else crc ushr 1
            }
        }
        return crc and 0xFFFF
    }

    private const val BITS_PER_BYTE = 8
}
