package com.hoverboard.protocol.linkctl

/**
 * Kotlin mirror of `crates/linkctl/src/lib.rs`: the four L7 control payload families that ride
 * L3 PDUs in the reserved control opcode block `0x10..0x2F`.
 *
 * An L7 payload here is the payload of one L3 PDU (`[opcode][src][dst][payload...]`, one PDU per
 * L2 packet). This file owns only the payload bytes; [com.hoverboard.protocol.l3.Pdu] owns the
 * header and [com.hoverboard.protocol.l2] owns the frame.
 *
 * Conventions carried across from the Rust, all pinned by `WireDriftTest`:
 * - Every multi-byte field is **little-endian** (`crates/linkctl/src/lib.rs:96-97`).
 * - **Committed-prefix decode**: a decoder reads its committed prefix and ignores trailing bytes,
 *   so a firmware build that appends a field still decodes here. A payload shorter than the
 *   prefix is rejected (`crates/linkctl/src/lib.rs:98-102`).
 * - All four families are best-effort / latest-wins: no seq, no ack, no retransmit
 *   (`crates/linkctl/src/lib.rs:104-105`). A rejected payload is dropped, not raised, which is
 *   why every `decode` here returns null rather than throwing.
 *
 * These opcodes are NOT the L3 opcodes in [com.hoverboard.protocol.l3.Opcode] and must never be
 * merged with them: `0x10` is a valid, forwardable L3 opcode byte that L3 deliberately does not
 * interpret (`crates/net/src/pdu.rs:41`).
 */

/** `CYCLIC_STATE`: board state broadcast. `crates/linkctl/src/lib.rs:34`. */
const val OP_CYCLIC_STATE: Int = 0x10

/** `DRIVE_CMD`: controller -> board drive reference. `crates/linkctl/src/lib.rs:37`. */
const val OP_DRIVE_CMD: Int = 0x11

/** `INPUTS`: controller/peer -> board input mirror. `crates/linkctl/src/lib.rs:40`. */
const val OP_INPUTS: Int = 0x12

/** `FAULT`: board -> peer, on latch edge. `crates/linkctl/src/lib.rs:43`. */
const val OP_FAULT: Int = 0x13

/**
 * Peer-staleness trip in 250 Hz ticks (100 ms). `crates/linkctl/src/lib.rs:51`.
 *
 * Mirrored for the controller's own staleness display; the firmware owns the actual supervision.
 */
const val CYCLIC_TIMEOUT_TICKS: Int = 25

/** Drive-staleness decay in 250 Hz ticks (200 ms). `crates/linkctl/src/lib.rs:56`. */
const val DRIVE_TIMEOUT_TICKS: Int = 50

// --- little-endian helpers ----------------------------------------------------------------------

private const val BYTE_MASK = 0xFF
private const val BYTE_BITS = 8
private const val SIGN_BIT_16 = 0x8000
private const val WRAP_16 = 0x10000

private fun rdU16(b: ByteArray, at: Int): Int =
    (b[at].toInt() and BYTE_MASK) or ((b[at + 1].toInt() and BYTE_MASK) shl BYTE_BITS)

private fun rdI16(b: ByteArray, at: Int): Int {
    val raw = rdU16(b, at)
    return if (raw and SIGN_BIT_16 != 0) raw - WRAP_16 else raw
}

private fun wrU16(b: ByteArray, at: Int, v: Int) {
    b[at] = (v and BYTE_MASK).toByte()
    b[at + 1] = ((v ushr BYTE_BITS) and BYTE_MASK).toByte()
}

private fun rdU8(b: ByteArray, at: Int): Int = b[at].toInt() and BYTE_MASK

// --- CYCLIC_STATE (11 B) ------------------------------------------------------------------------

/**
 * Board state, emitted cyclically. Mirror of `crates/linkctl/src/lib.rs:88-106`.
 *
 * Wire layout, 11 bytes (`crates/linkctl/src/lib.rs:131-141`):
 * ```
 * off 0..2   i16 LE  pitch        centidegrees
 * off 2..4   i16 LE  roll         centidegrees
 * off 4..6   i16 LE  wheelSpeed   stock-native speed word
 * off 6..8   u16 LE  battery      CENTIVOLTS (crates/orchestrator/src/dispatch.rs:42,113)
 * off 8      u8      mode
 * off 9      u8      fault        latched code, 0 = healthy
 * off 10     u8      flags        bit0 rider, bit7 lockdown
 * ```
 *
 * Note [fault] is hardcoded to 0 by the current emitter
 * (`crates/orchestrator/src/dispatch.rs:435`); the field is carried but never yet non-zero.
 *
 * [battery] and [mode] are held as unsigned values in an Int, since Kotlin's Byte/Short are signed.
 */
data class CyclicState(
    val pitch: Int,
    val roll: Int,
    val wheelSpeed: Int,
    val battery: Int,
    val mode: Int,
    val fault: Int,
    val flags: Int,
) {
    /** Rider-present flag, bit0. `crates/linkctl/src/lib.rs:121-123`. */
    fun riderPresent(): Boolean = flags and FLAG_RIDER != 0

    /** Lockdown flag, bit7. `crates/linkctl/src/lib.rs:126-128`. */
    fun lockdown(): Boolean = flags and FLAG_LOCKDOWN != 0

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs:131-141`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrU16(out, 0, pitch)
        wrU16(out, 2, roll)
        wrU16(out, 4, wheelSpeed)
        wrU16(out, 6, battery)
        out[8] = mode.toByte()
        out[9] = fault.toByte()
        out[10] = flags.toByte()
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs:110`. */
        const val LEN = 11

        /** `flags` bit0: rider present. `crates/linkctl/src/lib.rs:113`. */
        const val FLAG_RIDER = 1 shl 0

        /** `flags` bit7: lockdown. `crates/linkctl/src/lib.rs:118`. */
        const val FLAG_LOCKDOWN = 1 shl 7

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs:144-157`.
         */
        fun decode(b: ByteArray): CyclicState? {
            if (b.size < LEN) return null
            return CyclicState(
                pitch = rdI16(b, 0),
                roll = rdI16(b, 2),
                wheelSpeed = rdI16(b, 4),
                battery = rdU16(b, 6),
                mode = rdU8(b, 8),
                fault = rdU8(b, 9),
                flags = rdU8(b, 10),
            )
        }
    }
}

// --- DRIVE_CMD (5 B) ----------------------------------------------------------------------------

/** The `DRIVE_CMD.kind` discriminant. `crates/linkctl/src/lib.rs:166-171`. */
enum class DriveKind(val value: Int) {
    /** Reference zero; `value`/`steer` are not live. */
    Neutral(0),

    /** `value`/`steer` live. */
    Throttle(1),
    ;

    companion object {
        /**
         * An unknown kind byte decodes as [Neutral], fail-safe.
         * `crates/linkctl/src/lib.rs:206-209`.
         */
        fun fromU8(b: Int): DriveKind = if (b == Throttle.value) Throttle else Neutral
    }
}

/**
 * A controller's drive reference. Mirror of `crates/linkctl/src/lib.rs:177-184`.
 *
 * Wire layout, 5 bytes (`crates/linkctl/src/lib.rs:191-197`):
 * ```
 * off 0      u8      kind
 * off 1..3   i16 LE  value
 * off 3..5   i16 LE  steer
 * ```
 */
data class DriveCmd(val kind: DriveKind, val value: Int, val steer: Int) {
    /** Encode the committed prefix. `crates/linkctl/src/lib.rs:191-197`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        out[0] = kind.value.toByte()
        wrU16(out, 1, value)
        wrU16(out, 3, steer)
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs:188`. */
        const val LEN = 5

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs:202-215`.
         */
        fun decode(b: ByteArray): DriveCmd? {
            if (b.size < LEN) return null
            return DriveCmd(
                kind = DriveKind.fromU8(rdU8(b, 0)),
                value = rdI16(b, 1),
                steer = rdI16(b, 3),
            )
        }
    }
}

// --- INPUTS (4 B) -------------------------------------------------------------------------------

/**
 * Remote input mirror. Mirror of `crates/linkctl/src/lib.rs:224-231`.
 *
 * Wire layout, 4 bytes (`crates/linkctl/src/lib.rs:254-260`):
 * ```
 * off 0..2   i16 LE  throttle
 * off 2      u8      buttons   bit0 power request
 * off 3      u8      rider     bit0 rider present
 * ```
 */
data class Inputs(val throttle: Int, val buttons: Int, val rider: Int) {
    /** Power-request level, `buttons` bit0. `crates/linkctl/src/lib.rs:244-246`. */
    fun powerRequest(): Boolean = buttons and BUTTON_POWER != 0

    /** Rider-present level, `rider` bit0. `crates/linkctl/src/lib.rs:249-251`. */
    fun riderPresent(): Boolean = rider and RIDER_PRESENT != 0

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs:254-260`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrU16(out, 0, throttle)
        out[2] = buttons.toByte()
        out[3] = rider.toByte()
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs:235`. */
        const val LEN = 4

        /** `buttons` bit0: power request. `crates/linkctl/src/lib.rs:238`. */
        const val BUTTON_POWER = 1 shl 0

        /** `rider` bit0: rider present. `crates/linkctl/src/lib.rs:241`. */
        const val RIDER_PRESENT = 1 shl 0

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs:263-272`.
         */
        fun decode(b: ByteArray): Inputs? {
            if (b.size < LEN) return null
            return Inputs(throttle = rdI16(b, 0), buttons = rdU8(b, 2), rider = rdU8(b, 3))
        }
    }
}

// --- FAULT (2 B) --------------------------------------------------------------------------------

/**
 * Latch-edge notification, emitted once per latch edge, not cyclically; the fault *level* lives
 * in [CyclicState.fault]. Mirror of `crates/linkctl/src/lib.rs:282-287`.
 *
 * Wire layout, 2 bytes (`crates/linkctl/src/lib.rs:305-310`):
 * ```
 * off 0      u8      code     state::fault codes, 0 = healthy
 * off 1      u8      action   0 notify, 1 STOP_ALL
 * ```
 */
data class Fault(val code: Int, val action: Int) {
    /**
     * True when the action byte is exactly [ACTION_STOP_ALL]; any other value is notify-only.
     * `crates/linkctl/src/lib.rs:300-302`.
     */
    fun stopAll(): Boolean = action == ACTION_STOP_ALL

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs:305-310`. */
    fun encode(): ByteArray = byteArrayOf(code.toByte(), action.toByte())

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs:291`. */
        const val LEN = 2

        /** `action` 0: notify only. `crates/linkctl/src/lib.rs:294`. */
        const val ACTION_NOTIFY = 0

        /** `action` 1: STOP_ALL. `crates/linkctl/src/lib.rs:297`. */
        const val ACTION_STOP_ALL = 1

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs:313-321`.
         */
        fun decode(b: ByteArray): Fault? {
            if (b.size < LEN) return null
            return Fault(code = rdU8(b, 0), action = rdU8(b, 1))
        }
    }
}

// --- Dispatch -----------------------------------------------------------------------------------

/** A decoded control-block payload, tagged by family. `crates/linkctl/src/lib.rs:328-337`. */
sealed class ControlPayload {
    data class Cyclic(val state: CyclicState) : ControlPayload()

    data class Drive(val cmd: DriveCmd) : ControlPayload()

    data class Input(val inputs: Inputs) : ControlPayload()

    data class Faulted(val fault: Fault) : ControlPayload()
}

/**
 * Decode a delivered control-block PDU payload by opcode. Mirror of
 * `crates/linkctl/src/lib.rs:343-351`.
 *
 * Returns null for an opcode this file does not allocate, or a payload shorter than the family's
 * committed prefix: the delivery class is best-effort, so the PDU is simply dropped.
 */
fun decodeControl(opcode: Int, payload: ByteArray): ControlPayload? = when (opcode) {
    OP_CYCLIC_STATE -> CyclicState.decode(payload)?.let { ControlPayload.Cyclic(it) }
    OP_DRIVE_CMD -> DriveCmd.decode(payload)?.let { ControlPayload.Drive(it) }
    OP_INPUTS -> Inputs.decode(payload)?.let { ControlPayload.Input(it) }
    OP_FAULT -> Fault.decode(payload)?.let { ControlPayload.Faulted(it) }
    else -> null
}
