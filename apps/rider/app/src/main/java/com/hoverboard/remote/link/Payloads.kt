package com.hoverboard.remote.link

/**
 * Link payload codecs. Mirrors crates/link/src/payload.rs.
 *
 * All multi-byte fields are little-endian, scaled integers (no floats). Kotlin has no ergonomic
 * small unsigned ints, so i16 / u8 / u16 fields are carried as [Int]; the codec masks and
 * sign-extends at the byte boundary. Decoders read the committed leading fields and IGNORE trailing
 * bytes, matching the Rust trailing-field extensibility.
 */

private fun rdI16(b: ByteArray, off: Int): Int {
    val lo = b[off].toInt() and 0xFF
    val hi = b[off + 1].toInt() and 0xFF
    return ((hi shl 8) or lo).toShort().toInt() // sign-extend
}

private fun rdU16(b: ByteArray, off: Int): Int {
    val lo = b[off].toInt() and 0xFF
    val hi = b[off + 1].toInt() and 0xFF
    return (hi shl 8) or lo
}

private fun wrI16(out: ByteArray, off: Int, v: Int) {
    out[off] = (v and 0xFF).toByte()
    out[off + 1] = ((v shr 8) and 0xFF).toByte()
}

private fun wrU16(out: ByteArray, off: Int, v: Int) {
    out[off] = (v and 0xFF).toByte()
    out[off + 1] = ((v ushr 8) and 0xFF).toByte()
}

/** Rider input snapshot (opcode 0x50, LEN 4). */
data class Inputs(val throttle: Int, val buttons: Int, val rider: Int) {
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrI16(out, 0, throttle)
        out[2] = (buttons and 0xFF).toByte()
        out[3] = (rider and 0xFF).toByte()
        return out
    }

    companion object {
        const val LEN = 4
        fun decode(b: ByteArray): Inputs? {
            if (b.size < LEN) return null
            return Inputs(throttle = rdI16(b, 0), buttons = b[2].toInt() and 0xFF, rider = b[3].toInt() and 0xFF)
        }
    }
}

/** Drive command (opcode 0x11, LEN 5). `kind`: 0 torque, 1 speed, 2 voltage. */
data class DriveCmd(val kind: Int, val value: Int, val steer: Int) {
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        out[0] = (kind and 0xFF).toByte()
        wrI16(out, 1, value)
        wrI16(out, 3, steer)
        return out
    }

    companion object {
        const val LEN = 5
        fun decode(b: ByteArray): DriveCmd? {
            if (b.size < LEN) return null
            return DriveCmd(kind = b[0].toInt() and 0xFF, value = rdI16(b, 1), steer = rdI16(b, 3))
        }
    }
}

/** Per-motor telemetry (opcode 0x20, LEN 9). Decode ignores trailing bytes. */
data class Telemetry(
    val motorIndex: Int,
    val batteryMv: Int,
    val currentCa: Int,
    val speed: Int,
    val faultCode: Int,
    val flags: Int,
) {
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        out[0] = (motorIndex and 0xFF).toByte()
        wrU16(out, 1, batteryMv)
        wrI16(out, 3, currentCa)
        wrI16(out, 5, speed)
        out[7] = (faultCode and 0xFF).toByte()
        out[8] = (flags and 0xFF).toByte()
        return out
    }

    companion object {
        const val LEN = 9
        fun decode(b: ByteArray): Telemetry? {
            if (b.size < LEN) return null
            return Telemetry(
                motorIndex = b[0].toInt() and 0xFF,
                batteryMv = rdU16(b, 1),
                currentCa = rdI16(b, 3),
                speed = rdI16(b, 5),
                faultCode = b[7].toInt() and 0xFF,
                flags = b[8].toInt() and 0xFF,
            )
        }
    }
}

/** Cyclic exchange payload (opcode 0x10, LEN 7). Decode-only in the app, for diagnostics. */
data class CyclicState(val pitch: Int, val roll: Int, val wheelSpeed: Int, val status: Int) {
    companion object {
        const val LEN = 7
        fun decode(b: ByteArray): CyclicState? {
            if (b.size < LEN) return null
            return CyclicState(
                pitch = rdI16(b, 0),
                roll = rdI16(b, 2),
                wheelSpeed = rdI16(b, 4),
                status = b[6].toInt() and 0xFF,
            )
        }
    }
}

/** Read a config register by key (opcode 0x30, LEN 2). */
data class ConfigRead(val key: Int) {
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrU16(out, 0, key)
        return out
    }

    companion object {
        const val LEN = 2
        fun decode(b: ByteArray): ConfigRead? {
            if (b.size < LEN) return null
            return ConfigRead(key = rdU16(b, 0))
        }
    }
}

/** Write a config register: key + variable-length value blob (opcode 0x31, HEAD 2). */
class ConfigWrite(val key: Int, val value: ByteArray) {
    fun encode(): ByteArray {
        val out = ByteArray(HEAD_LEN + value.size)
        wrU16(out, 0, key)
        value.copyInto(out, HEAD_LEN)
        return out
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ConfigWrite) return false
        return key == other.key && value.contentEquals(other.value)
    }

    override fun hashCode(): Int = 31 * key + value.contentHashCode()

    companion object {
        const val HEAD_LEN = 2
        fun decode(b: ByteArray): ConfigWrite? {
            if (b.size < HEAD_LEN) return null
            return ConfigWrite(key = rdU16(b, 0), value = b.copyOfRange(HEAD_LEN, b.size))
        }
    }
}

/** Config response: key, status, variable-length value blob (opcode 0x32, HEAD 3). */
class ConfigResp(val key: Int, val status: Int, val value: ByteArray) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is ConfigResp) return false
        return key == other.key && status == other.status && value.contentEquals(other.value)
    }

    override fun hashCode(): Int = 31 * (31 * key + status) + value.contentHashCode()

    companion object {
        const val HEAD_LEN = 3
        fun decode(b: ByteArray): ConfigResp? {
            if (b.size < HEAD_LEN) return null
            return ConfigResp(
                key = rdU16(b, 0),
                status = b[2].toInt() and 0xFF,
                value = b.copyOfRange(HEAD_LEN, b.size),
            )
        }
    }
}

/** A fault report / command (opcode 0x40, LEN 2). `action`: 0 notify, 1 stop-all. */
data class Fault(val faultCode: Int, val action: Int) {
    fun encode(): ByteArray = byteArrayOf((faultCode and 0xFF).toByte(), (action and 0xFF).toByte())

    companion object {
        const val LEN = 2
        fun decode(b: ByteArray): Fault? {
            if (b.size < LEN) return null
            return Fault(faultCode = b[0].toInt() and 0xFF, action = b[1].toInt() and 0xFF)
        }
    }
}
