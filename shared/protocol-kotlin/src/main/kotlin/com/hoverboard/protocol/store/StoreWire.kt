package com.hoverboard.protocol.store

/**
 * The config store's on-wire vocabulary, a byte-for-byte mirror of `shared/store/src/{key,value}.rs`.
 * `CONFIG_*` PDUs carry a [Key] (`field_id`/`index`), a [Type] tag, and a little-endian [Value]
 * payload.
 */

/**
 * The `type` byte of a tombstone (`shared/store/src/key.rs`, `ERASED_TAG`): a `CONFIG_WRITE` whose
 * value is this byte alone un-stores the key, and the key then reads its default. It is no [Type]'s
 * tag.
 */
const val ERASED_TAG = 0x00

/** A store key: `field_id` names the field, `index` selects the instance (a singleton uses 0). */
data class Key(val fieldId: Int, val index: Int)

/** The storage-layout type; its [tag] is the on-wire `type` byte (mirror of `shared/store/src/key.rs`'s `Type`). */
enum class Type(val tag: Int) {
    U8(0x01),
    U16(0x02),
    U32(0x03),
    U64(0x04),
    I16(0x05),
    I32(0x06),
    I64(0x07),
    Bool(0x08),
    Blob(0x09),
    Str(0x0A),

    /** A pin: port (`0` = A to `5` = F), pin number (`specs/config-read-once.md` section 3). */
    Pin(0x0B),

    /** A pin and its alternate function: port, pin, af. */
    PinAf(0x0C),

    /** A pin and its ADC channel: port, pin, channel. */
    PinAdc(0x0D),
    ;

    companion object {
        fun fromTag(tag: Int): Type? = entries.firstOrNull { it.tag == tag }
    }
}

/**
 * A dynamically typed config value (mirror of `shared/store/src/value.rs`'s `Value`): a [Type] tag plus the data.
 * Scalars encode little-endian; `Str` is UTF-8 bytes; `Bytes` is raw. Unsigned 32/64-bit values are
 * carried in `Long` (only the low N bits are written).
 */
sealed class Value {
    data class U8(val v: Int) : Value()
    data class U16(val v: Int) : Value()
    data class U32(val v: Long) : Value()
    data class U64(val v: Long) : Value()
    data class I16(val v: Int) : Value()
    data class I32(val v: Int) : Value()
    data class I64(val v: Long) : Value()
    data class Bool(val v: Boolean) : Value()
    data class Str(val v: String) : Value()
    data class Bytes(val v: ByteArray) : Value() {
        override fun equals(other: Any?): Boolean = other is Bytes && v.contentEquals(other.v)
        override fun hashCode(): Int = v.contentHashCode()
    }

    /** A pin ([Type.Pin]): its port (`0` = A) and pin number. */
    data class Pin(val port: Int, val pin: Int) : Value()

    /** A pin and its alternate function ([Type.PinAf]); `af` is 0 on a chip that remaps. */
    data class PinAf(val port: Int, val pin: Int, val af: Int) : Value()

    /** A pin and its ADC channel ([Type.PinAdc]). */
    data class PinAdc(val port: Int, val pin: Int, val adc: Int) : Value()

    /** The storage [Type] this value carries (the tag a schema-less consumer reads). */
    fun kind(): Type = when (this) {
        is U8 -> Type.U8
        is U16 -> Type.U16
        is U32 -> Type.U32
        is U64 -> Type.U64
        is I16 -> Type.I16
        is I32 -> Type.I32
        is I64 -> Type.I64
        is Bool -> Type.Bool
        is Str -> Type.Str
        is Bytes -> Type.Blob
        is Pin -> Type.Pin
        is PinAf -> Type.PinAf
        is PinAdc -> Type.PinAdc
    }

    /** Encode the value payload little-endian (the record `type` byte is [kind].tag, written separately). */
    fun encode(): ByteArray = when (this) {
        is U8 -> byteArrayOf(v.toByte())
        is Bool -> byteArrayOf(if (v) 1 else 0)
        is U16 -> le(v.toLong(), 2)
        is I16 -> le(v.toLong(), 2)
        is U32 -> le(v, 4)
        is I32 -> le(v.toLong(), 4)
        is U64 -> le(v, 8)
        is I64 -> le(v, 8)
        is Str -> v.toByteArray(Charsets.UTF_8)
        is Bytes -> v
        is Pin -> byteArrayOf(port.toByte(), pin.toByte())
        is PinAf -> byteArrayOf(port.toByte(), pin.toByte(), af.toByte())
        is PinAdc -> byteArrayOf(port.toByte(), pin.toByte(), adc.toByte())
    }

    companion object {
        /** Decode a value of storage `kind` from its little-endian payload, or null on a width/UTF-8 fault. */
        fun decode(kind: Type, bytes: ByteArray): Value? = when (kind) {
            Type.U8 -> bytes.getOrNull(0)?.let { U8(it.toInt() and 0xFF) }
            Type.Bool -> bytes.getOrNull(0)?.let { Bool(it.toInt() != 0) }
            Type.U16 -> fixed(bytes, 2)?.let { U16((leToLong(it) and 0xFFFF).toInt()) }
            Type.I16 -> fixed(bytes, 2)?.let { I16(leToLong(it).toShort().toInt()) }
            Type.U32 -> fixed(bytes, 4)?.let { U32(leToLong(it) and 0xFFFFFFFFL) }
            Type.I32 -> fixed(bytes, 4)?.let { I32(leToLong(it).toInt()) }
            Type.U64 -> fixed(bytes, 8)?.let { U64(leToLong(it)) }
            Type.I64 -> fixed(bytes, 8)?.let { I64(leToLong(it)) }
            Type.Str -> try {
                Str(bytes.toString(Charsets.UTF_8).also {
                    // Reject invalid UTF-8: decoding then re-encoding must round-trip the bytes.
                    if (!it.toByteArray(Charsets.UTF_8).contentEquals(bytes)) return null
                })
            } catch (_: Exception) {
                null
            }
            Type.Blob -> Bytes(bytes)
            Type.Pin -> fixed(bytes, 2)?.let { Pin(u(it, 0), u(it, 1)) }
            Type.PinAf -> fixed(bytes, 3)?.let { PinAf(u(it, 0), u(it, 1), u(it, 2)) }
            Type.PinAdc -> fixed(bytes, 3)?.let { PinAdc(u(it, 0), u(it, 1), u(it, 2)) }
        }

        private fun u(bytes: ByteArray, i: Int): Int = bytes[i].toInt() and 0xFF

        private fun le(value: Long, n: Int): ByteArray {
            val out = ByteArray(n)
            for (i in 0 until n) out[i] = ((value ushr (8 * i)) and 0xFF).toByte()
            return out
        }

        private fun leToLong(bytes: ByteArray): Long {
            var v = 0L
            for (i in bytes.indices) v = v or ((bytes[i].toLong() and 0xFF) shl (8 * i))
            return v
        }

        private fun fixed(bytes: ByteArray, n: Int): ByteArray? = if (bytes.size == n) bytes else null
    }
}
