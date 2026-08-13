package com.hoverboard.protocol.linkctl

/**
 * Kotlin mirror of `crates/linkctl/src/lib.rs`: the four L7 control payload families that ride
 * L3 PDUs in the reserved control opcode block `0x10..0x2F`.
 *
 * An L7 payload here is the payload of one L3 PDU (`[opcode][src][dst][payload...]`, one PDU per
 * L2 packet). This file owns only the payload bytes; [com.hoverboard.protocol.l3.Pdu] owns the
 * header and [com.hoverboard.protocol.l2] owns the frame.
 *
 * Conventions carried across from the Rust, all pinned by `WireDriftTest` and all stated in that
 * crate's module doc (`crates/linkctl/src/lib.rs`, the header):
 * - Every multi-byte field is **little-endian**.
 * - **Committed-prefix decode**: a decoder reads its committed prefix and ignores trailing bytes,
 *   so a firmware build that appends a field still decodes here. A payload shorter than the
 *   prefix is rejected.
 * - All four families are best-effort / latest-wins: no seq, no ack, no retransmit. A rejected
 *   payload is dropped, not raised, which is why every `decode` here returns null rather than
 *   throwing.
 *
 * These opcodes are NOT the L3 opcodes in [com.hoverboard.protocol.l3.Opcode] and must never be
 * merged with them: `0x10` is a valid, forwardable L3 opcode byte that L3 deliberately does not
 * interpret (`crates/net/src/pdu.rs`, `Opcode::from_u8`).
 *
 * **Citations here name a FILE and a SYMBOL, never a line number.** Every declaration below points
 * at the Rust it mirrors, and this file carried 43 `file:line` citations that had rotted where they
 * stood: an insertion anywhere above a constant moves it, nothing rebuilds the number, and a reader
 * following one lands on an unrelated doc line and cannot tell whether the citation or the claim is
 * the stale part. A symbol survives every edit that does not rename it, and a rename is exactly the
 * moment a human should be re-reading the claim anyway. Line numbers are kept ONLY for citations
 * into frozen external sources (the EFeru dump, the Declassyfied decompile of the stock firmware),
 * which nobody here edits and which therefore cannot rot; this file cites none.
 *
 * That rule is enforced rather than merely stated: `tools/check-citations.py` runs in CI, resolves
 * every citation below against the file it names, and fails on a symbol that file does not declare,
 * on a line number coming back, and on a citation written in a shape it cannot parse.
 */

/** `CYCLIC_STATE`: board state broadcast. `crates/linkctl/src/lib.rs`, `OP_CYCLIC_STATE`. */
const val OP_CYCLIC_STATE: Int = 0x10

/** `DRIVE_CMD`: controller -> board drive reference. `crates/linkctl/src/lib.rs`, `OP_DRIVE_CMD`. */
const val OP_DRIVE_CMD: Int = 0x11

/** `INPUTS`: controller/peer -> board input mirror. `crates/linkctl/src/lib.rs`, `OP_INPUTS`. */
const val OP_INPUTS: Int = 0x12

/** `FAULT`: board -> peer, on latch edge. `crates/linkctl/src/lib.rs`, `OP_FAULT`. */
const val OP_FAULT: Int = 0x13

/**
 * Peer-staleness trip in 250 Hz ticks (100 ms). `crates/linkctl/src/lib.rs`, `CYCLIC_TIMEOUT_TICKS`.
 *
 * Mirrored for the controller's own staleness display; the firmware owns the actual supervision.
 */
const val CYCLIC_TIMEOUT_TICKS: Int = 25

/** Drive-staleness decay in 250 Hz ticks (200 ms). `crates/linkctl/src/lib.rs`, `DRIVE_TIMEOUT_TICKS`. */
const val DRIVE_TIMEOUT_TICKS: Int = 50

/**
 * Remote-`INPUTS`-mirror staleness in 250 Hz ticks (1.5 s).
 * `crates/linkctl/src/lib.rs`, `INPUTS_TIMEOUT_TICKS`.
 *
 * This is the window a controller's arm level survives in the firmware once the controller stops
 * being heard from: past it the mirror stops being a source at all, every level it carries reads as
 * released, and an armed board disarms. So it is a CONTRACT between this protocol and whatever
 * cadence the controller sends on, not a firmware-internal number: a controller whose traffic gaps
 * exceed it drops its own bridge mid-ride.
 *
 * Mirrored here so both halves of that contract are pinned in one place. The firmware owns the
 * supervision; `RustSourceDriftTest` pins this value against the Rust AND checks that it still
 * outlasts the rider app's keepalive period by the stated margin, so moving either side fails.
 */
const val INPUTS_TIMEOUT_TICKS: Int = 375

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
 * Board state, emitted cyclically. Mirror of `crates/linkctl/src/lib.rs`, `CyclicState`.
 *
 * Wire layout, 11 bytes (`crates/linkctl/src/lib.rs`, `CyclicState::encode`):
 * ```
 * off 0..2   i16 LE  pitch        centidegrees
 * off 2..4   i16 LE  roll         centidegrees
 * off 4..6   i16 LE  wheelSpeed   stock-native speed word
 * off 6..8   u16 LE  battery      CENTIVOLTS (crates/orchestrator/src/dispatch.rs, BATTERY_PLACEHOLDER_CENTIVOLT)
 * off 8      u8      mode
 * off 9      u8      fault        latched code, 0 = healthy
 * off 10     u8      flags        bit0 rider, bit7 lockdown
 * ```
 *
 * Note [fault] is hardcoded to 0 by the current emitter
 * (`crates/orchestrator/src/dispatch.rs`, `cyclic_state`); the field is carried but never yet non-zero.
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
    /** Rider-present flag, bit0. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_RIDER`. */
    fun riderPresent(): Boolean = flags and FLAG_RIDER != 0

    /** Lockdown flag, bit7. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LOCKDOWN`. */
    fun lockdown(): Boolean = flags and FLAG_LOCKDOWN != 0

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs`, `CyclicState::encode`. */
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
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs`, `CyclicState::LEN`. */
        const val LEN = 11

        /** `flags` bit0: rider present. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_RIDER`. */
        const val FLAG_RIDER = 1 shl 0

        /** `flags` bit7: lockdown. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LOCKDOWN`. */
        const val FLAG_LOCKDOWN = 1 shl 7

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs`, `CyclicState::decode`.
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

/** The `DRIVE_CMD.kind` discriminant. `crates/linkctl/src/lib.rs`, `DriveKind`. */
enum class DriveKind(val value: Int) {
    /** Reference zero; `value`/`steer` are not live. */
    Neutral(0),

    /** `value`/`steer` live. */
    Throttle(1),
    ;

    companion object {
        /**
         * An unknown kind byte decodes as [Neutral], fail-safe.
         * `crates/linkctl/src/lib.rs`, `DriveCmd::decode`.
         */
        fun fromU8(b: Int): DriveKind = if (b == Throttle.value) Throttle else Neutral
    }
}

/**
 * A controller's drive reference. Mirror of `crates/linkctl/src/lib.rs`, `DriveCmd`.
 *
 * Wire layout, 5 bytes (`crates/linkctl/src/lib.rs`, `DriveCmd::encode`):
 * ```
 * off 0      u8      kind
 * off 1..3   i16 LE  value
 * off 3..5   i16 LE  steer
 * ```
 */
data class DriveCmd(val kind: DriveKind, val value: Int, val steer: Int) {
    /** Encode the committed prefix. `crates/linkctl/src/lib.rs`, `DriveCmd::encode`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        out[0] = kind.value.toByte()
        wrU16(out, 1, value)
        wrU16(out, 3, steer)
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs`, `DriveCmd::LEN`. */
        const val LEN = 5

        /**
         * Full-scale magnitude of [value] and [steer]: the demand word is a fraction of this,
         * not an engineering unit. `crates/control/src/config.rs`, `FRAME_IN_MAX`.
         *
         * `linkctl` itself carries no numbers (`DriveCmd.value` in `crates/linkctl/src/lib.rs` says only
         * "`ControlDispatch::throttle_reference` input scale"), because the scale is established
         * downstream at the frame-in adapter, `crates/control/src/throttle.rs`, `throttle_tick`:
         * ```rust
         * let speed_cmd = ((speed_in as i32) * (cfgc::CMD_LIMIT as i32) / cfgc::FRAME_IN_MAX) as i16;
         * ```
         * so `value` is mapped onto the EFeru command domain `+-CMD_LIMIT` (1000,
         * `crates/control/src/config.rs`, `CMD_LIMIT`), which is the only hard saturation on the path. A
         * sender that treats `value` as if it were already in that 1000-domain therefore commands
         * a thirty-third of what it meant to.
         *
         * Two consequences a sender has to know, both arithmetic on the line above:
         * - The division truncates, so `|value| < 33` is indistinguishable from zero.
         * - The engagement gate needs a reference above `GATING_THRESHOLD` (500,
         *   `crates/control/src/config.rs`, `GATING_THRESHOLD`, tested by the engage gate in
         *   `crates/control/src/fsm.rs`, `idle`) to pick
         *   up from idle, and the reference is `speed_cmd * 57 / 2`
         *   (`crates/control/src/throttle.rs`, `throttle_tick`), so the smallest `value` that will ever start a
         *   stopped machine is +-590.
         */
        const val FULL_SCALE = 32767

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs`, `DriveCmd::decode`.
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
 * Remote input mirror. Mirror of `crates/linkctl/src/lib.rs`, `Inputs`.
 *
 * Wire layout, 4 bytes (`crates/linkctl/src/lib.rs`, `Inputs::encode`):
 * ```
 * off 0..2   i16 LE  throttle
 * off 2      u8      buttons   bit0 power request
 * off 3      u8      rider     bit0 rider present
 * ```
 */
data class Inputs(val throttle: Int, val buttons: Int, val rider: Int) {
    /** Power-request level, `buttons` bit0. `crates/linkctl/src/lib.rs`, `Inputs::power_request`. */
    fun powerRequest(): Boolean = buttons and BUTTON_POWER != 0

    /** Rider-present level, `rider` bit0. `crates/linkctl/src/lib.rs`, `Inputs::rider_present`. */
    fun riderPresent(): Boolean = rider and RIDER_PRESENT != 0

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs`, `Inputs::encode`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrU16(out, 0, throttle)
        out[2] = buttons.toByte()
        out[3] = rider.toByte()
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs`, `Inputs::LEN`. */
        const val LEN = 4

        /** `buttons` bit0: power request. `crates/linkctl/src/lib.rs`, `Inputs::BUTTON_POWER`. */
        const val BUTTON_POWER = 1 shl 0

        /** `rider` bit0: rider present. `crates/linkctl/src/lib.rs`, `Inputs::RIDER_PRESENT`. */
        const val RIDER_PRESENT = 1 shl 0

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs`, `Inputs::decode`.
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
 * in [CyclicState.fault]. Mirror of `crates/linkctl/src/lib.rs`, `Fault`.
 *
 * Wire layout, 2 bytes (`crates/linkctl/src/lib.rs`, `Fault::encode`):
 * ```
 * off 0      u8      code     state::fault codes, 0 = healthy
 * off 1      u8      action   0 notify, 1 STOP_ALL
 * ```
 */
data class Fault(val code: Int, val action: Int) {
    /**
     * True when the action byte is exactly [ACTION_STOP_ALL]; any other value is notify-only.
     * `crates/linkctl/src/lib.rs`, `Fault::stop_all`.
     */
    fun stopAll(): Boolean = action == ACTION_STOP_ALL

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs`, `Fault::encode`. */
    fun encode(): ByteArray = byteArrayOf(code.toByte(), action.toByte())

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs`, `Fault::LEN`. */
        const val LEN = 2

        /** `action` 0: notify only. `crates/linkctl/src/lib.rs`, `Fault::ACTION_NOTIFY`. */
        const val ACTION_NOTIFY = 0

        /** `action` 1: STOP_ALL. `crates/linkctl/src/lib.rs`, `Fault::ACTION_STOP_ALL`. */
        const val ACTION_STOP_ALL = 1

        /**
         * Decode the committed prefix, ignoring trailing bytes; null when shorter than [LEN].
         * `crates/linkctl/src/lib.rs`, `Fault::decode`.
         */
        fun decode(b: ByteArray): Fault? {
            if (b.size < LEN) return null
            return Fault(code = rdU8(b, 0), action = rdU8(b, 1))
        }
    }
}

// --- Dispatch -----------------------------------------------------------------------------------

/** A decoded control-block payload, tagged by family. `crates/linkctl/src/lib.rs`, `Payload`. */
sealed class ControlPayload {
    data class Cyclic(val state: CyclicState) : ControlPayload()

    data class Drive(val cmd: DriveCmd) : ControlPayload()

    data class Input(val inputs: Inputs) : ControlPayload()

    data class Faulted(val fault: Fault) : ControlPayload()
}

/**
 * Decode a delivered control-block PDU payload by opcode. Mirror of
 * `crates/linkctl/src/lib.rs`, `decode`.
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
