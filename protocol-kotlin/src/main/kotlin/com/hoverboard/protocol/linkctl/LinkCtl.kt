package com.hoverboard.protocol.linkctl

/**
 * Kotlin mirror of `crates/linkctl/src/lib.rs`: the three L7 control payload families that ride
 * L3 PDUs in the reserved control opcode block `0x10..0x2F`.
 *
 * An L7 payload here is the payload of one L3 PDU (`[opcode][src][dst][payload...]`, one PDU per
 * L2 packet). This file owns only the payload bytes; [com.hoverboard.protocol.l3.Pdu] owns the
 * header and [com.hoverboard.protocol.l2] owns the frame.
 *
 * Conventions carried across from the Rust, all pinned by `WireDriftTest` and all stated in that
 * crate's module doc (`crates/linkctl/src/lib.rs`, the header):
 * - Every multi-byte field is **little-endian**.
 * - **Committed-prefix decode** for [DriveCmd] and [Inputs]: a decoder reads its
 *   committed prefix and ignores trailing bytes, so a firmware build that appends a field still
 *   decodes here. A payload shorter than the prefix is rejected.
 * - **[CyclicState] is decoded by EXACT LENGTH** ([CyclicState.ENCODED_LEN]): the committed words
 *   plus the appended [CyclicObs] block, and any other length is a protocol error. It is the one
 *   family that does not tolerate a short or a long payload, and the firmware's decoder is the
 *   same (`crates/linkctl/src/lib.rs`, `CyclicState::decode`).
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
 * moment a human should be re-reading the claim anyway. A line number is never permitted, with no
 * exception: this header used to allow one for citations into frozen external sources (the EFeru
 * dump, the Declassyfied decompile), which nothing implemented and this file never used.
 *
 * That rule is enforced rather than merely stated: `tools/check-citations.py` runs in CI, resolves
 * every citation below against the file it names, and fails on a symbol that file does not declare,
 * on a line number coming back, and on a citation written in a shape it cannot parse. It reads
 * citations into `.rs`, `.md`, `.c` and `.h` paths; a Kotlin filename is prose here, not a citation.
 *
 * Its one exemption is paths under `specs/`, which are skipped with a count printed rather than
 * resolved, because `specs/` is gitignored and exists in the primary checkout alone - not in a
 * worktree, not in a clean clone, not on CI. This file cites none today; WireDriftTest cites three.
 */

/** `CYCLIC_STATE`: board state broadcast. `crates/linkctl/src/lib.rs`, `OP_CYCLIC_STATE`. */
const val OP_CYCLIC_STATE: Int = 0x10

/** `DRIVE_CMD`: controller -> board drive reference. `crates/linkctl/src/lib.rs`, `OP_DRIVE_CMD`. */
const val OP_DRIVE_CMD: Int = 0x11

/** `INPUTS`: controller/peer -> board input mirror. `crates/linkctl/src/lib.rs`, `OP_INPUTS`. */
const val OP_INPUTS: Int = 0x12

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
private const val U16_BITS = 16
private const val U16_MASK = 0xFFFFL

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

/**
 * A wire `u32` as a [Long], because Kotlin's `Int` is signed and a count past 2^31 would read
 * negative. The caller holds the value in a `Long` for the same reason.
 */
private fun rdU32(b: ByteArray, at: Int): Long =
    (rdU16(b, at).toLong()) or (rdU16(b, at + 2).toLong() shl U16_BITS)

private fun wrU32(b: ByteArray, at: Int, v: Long) {
    wrU16(b, at, (v and U16_MASK).toInt())
    wrU16(b, at + 2, ((v ushr U16_BITS) and U16_MASK).toInt())
}

// --- CYCLIC_STATE (27 B: 15 B committed + a 12 B appended block) --------------------------------

/**
 * The part a board reports in [CyclicObs.chip]: what `detect_chip` identified at boot.
 * Mirror of `crates/linkctl/src/lib.rs`, `ChipTag`.
 *
 * The tags are the firmware's own allocation, not a silicon id: the GD32 parts carry no readable
 * part number, so the board derives the tag from what its detect probe MEASURED
 * (`crates/linkctl/src/lib.rs`, `ChipTag::from_detected`).
 *
 * [Unknown] is the fail-safe for a byte this build does not allocate, as [DriveKind.Neutral] is
 * for an unknown kind byte: a client that cannot name the part must say so rather than assume one.
 * A board that named no part at all is a different state again, and it is not one a payload can
 * carry: it means nothing has arrived from the board yet.
 */
enum class ChipTag(val value: Int) {
    /** The part is not named: a byte this build does not allocate. */
    Unknown(0),

    /** GD32F103C8, LQFP48: the bench F103 master and the 6-FET split boards. */
    F103C8(1),

    /** GD32F130C8, LQFP48: the bench F130 slave and the offroad pair. */
    F130C8(2),

    /** GD32F103RC, LQFP64: the 12-FET dual-motor mainboard, two advanced timers. */
    F103RC(3),
    ;

    companion object {
        /**
         * The tag a wire byte names; anything unallocated is [Unknown] (fail-safe).
         * `crates/linkctl/src/lib.rs`, `ChipTag::from_u8`.
         */
        fun fromU8(b: Int): ChipTag = entries.firstOrNull { it.value == b } ?: Unknown
    }
}

/**
 * The APPENDED observation block of a `CYCLIC_STATE`. Mirror of `crates/linkctl/src/lib.rs`,
 * `CyclicObs`.
 *
 * Wire layout, 12 bytes from offset 15 (`crates/linkctl/src/lib.rs`, `CyclicObs`):
 * ```
 * off 15..17  i16 LE  phasePeak   window peak phase-current magnitude, stock current counts
 * off 17..19  i16 LE  phaseMean   the SAME window's mean magnitude, same counts
 * off 19..21  u16 LE  dutyOn      the last applied on-duty, 0..ARR
 * off 21      u8      bootTag     boot_count's low byte
 * off 22      u8      chip        the part detect_chip identified (ChipTag)
 * off 23..27  u32 LE  odometer    this wheel's lifetime travel in absolute hall edges
 * ```
 *
 * The offsets above moved by two when `wheelPos` joined the committed prefix, as they had moved
 * by two when `peerDemand` did (they were stale from `peerDemand` until the odometer). Additions
 * go on the END of this block, so every offset in it survives one, but the block's own start moves
 * with the prefix and these numbers are against the PAYLOAD.
 *
 * The two current words describe ONE 64-period window (the firmware closes both in one call), so
 * they are comparable: [phasePeak] is a maximum over ADC samples and reads high near the noise
 * floor, which is why a calibration cross-check compares [phaseMean]
 * (`specs/rider-ui.md`, "Current in the panel, and the live calibration cross-check"). The
 * DC-link current a bench PSU displays is
 * `(phaseMean / cal) * (dutyOn / ARR)`, where `cal` is the board's own `MOTOR_CURRENT_CAL`.
 *
 * [dutyOn] is 0 for a period that coasted: no phase conducted in it, whatever the duty registers
 * still held.
 *
 * [odometer] is this wheel's LIVE lifetime travel in absolute hall edges: the total the board's
 * store last recorded plus every edge counted since it booted (`crates/store/src/field.rs`,
 * `MOTOR_ODOMETER`). Reversing adds, so it never winds back, and it is COUNTS rather than a
 * distance, because converting needs a pole-pair count and a wheel diameter
 * (`Fields.MOTOR_POLE_PAIRS` and `Fields.WHEEL_DIAMETER_MM`), both registered and both defaulting
 * to unset. A consumer converts for a board that carries both and otherwise shows counts and says
 * they are counts.
 *
 * [odometer] is held as an unsigned value in a Long, for the reason [CyclicState.battery] and
 * [CyclicState.mode] are held as unsigned values in an Int: Kotlin's Int is signed and a `u32`
 * runs to 4,294,967,295, which would read negative past half of it.
 */
data class CyclicObs(
    val phasePeak: Int,
    val phaseMean: Int,
    val dutyOn: Int,
    val bootTag: Int,
    val chip: ChipTag,
    val odometer: Long,
) {
    /** Encode the block alone, as it sits after the committed words. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        wrU16(out, 0, phasePeak)
        wrU16(out, 2, phaseMean)
        wrU16(out, 4, dutyOn)
        out[6] = bootTag.toByte()
        out[7] = chip.value.toByte()
        wrU32(out, 8, odometer)
        return out
    }

    companion object {
        /** On-wire length of the appended block. `crates/linkctl/src/lib.rs`, `CyclicObs::LEN`. */
        const val LEN = 12

        /**
         * The [dutyOn] value of a fully conducting period: the PWM period itself, `commutation::ARR`
         * (the stock 16 kHz centre-aligned timer at the fleet's 72 MHz clock).
         *
         * Mirrored here because [dutyOn] means nothing without it: the word is a duty against this
         * period, and the DC-link current a consumer derives is `(phaseMean / cal) * (dutyOn /
         * DUTY_FULL)`. `crates/commutation/src/lib.rs`, `ARR` owns the number and
         * `RustSourceDriftTest` pins this against it.
         */
        const val DUTY_FULL = 2_250

        /** Decode the block from [b] at [at], which must hold [LEN] bytes from there. */
        fun decode(b: ByteArray, at: Int): CyclicObs = CyclicObs(
            phasePeak = rdI16(b, at),
            phaseMean = rdI16(b, at + 2),
            dutyOn = rdU16(b, at + 4),
            bootTag = rdU8(b, at + 6),
            chip = ChipTag.fromU8(rdU8(b, at + 7)),
            odometer = rdU32(b, at + 8),
        )
    }
}

/**
 * Board state, emitted cyclically. Mirror of `crates/linkctl/src/lib.rs`, `CyclicState`.
 *
 * Wire layout, 27 bytes and only 27 (`crates/linkctl/src/lib.rs`, `CyclicState::encode`), of
 * which the first 15 are the committed words and the last 12 the appended [CyclicObs] block:
 * ```
 * off 0..2   i16 LE  pitch        centidegrees
 * off 2..4   i16 LE  roll         centidegrees
 * off 4..6   i16 LE  wheelSpeed   raw hall edge count per window, see below
 * off 6..8   u16 LE  battery      CENTIVOLTS, 0 = unknown (crates/orchestrator/src/battery.rs, battery_source)
 * off 8      u8      mode
 * off 9      u8      fault        the gating-producer level mask, 0 = every producer clear
 * off 10     u8      flags        bit0 rider, bit1 rigid leader, bit2 rigid follower, bit7 lockdown
 * off 11..13 i16 LE  peerDemand   the torque a rigid LEADER commands its peer to apply, else 0
 * off 13..15 i16 LE  wheelPos     the low 16 bits of the wheel's signed hall-edge position
 * off 15..27         the CyclicObs block
 * ```
 *
 * [wheelSpeed] is the RAW hall edge count per 320-period window, in the motor ISR's own unit with
 * no rescaling: the `SPEED` cell saturated into an i16 (`crates/firmware/src/motor.rs`,
 * `wheel_speed_word`, `SPEED`), so a stationary wheel reads exactly 0. It is not the stock speed
 * unit, and it is not a road speed: deriving one needs the motor's pole-pair count and a wheel
 * diameter, and neither is a registered store field, so this mirror reports the edge count as it is
 * rather than inventing a unit for it.
 *
 * [fault] is the board's gating-producer LEVEL MASK as of the pass that built the frame
 * (`crates/orchestrator/src/dispatch.rs`, `cyclic_state`, from `crates/orchestrator/src/events.rs`,
 * `Levels`): one bit per producer, so a reader sees which producers are asserted right now rather
 * than a code it could miss between emissions. Not every bit is a fault: the mask also carries the
 * mode fault and the power request, and [FAULT_A_BITS] is the group that is. [faultA] applies it.
 *
 * [peerDemand] is the +-28500 stock-native torque a rigid-frame LEADER commands its peer to apply
 * (`crates/linkctl/src/lib.rs`, `CyclicState::peer_demand`), and 0 from every other board; [leader]
 * says which reading applies.
 *
 * [wheelPos] is the low 16 bits of the board's signed wheel position in hall edges
 * (`crates/linkctl/src/lib.rs`, `CyclicState::wheel_pos`; `specs/state-feedback.md` section 3), in
 * the hall counter's own sense and not mirrored to forward, wrapping. A rigid leader unwraps it by
 * delta; the telemetry model shows nothing for it.
 *
 * [battery] and [mode] are held as unsigned values in an Int, since Kotlin's Byte/Short are signed.
 *
 * [obs] is not nullable and has no default: every payload this mirror decodes carries the block,
 * because a payload that does not carry it is refused, so a zeroed window has one reading only,
 * a board carrying no current. A caller building a [CyclicState] for a test says what the board
 * reported rather than leaving it out.
 */
data class CyclicState(
    val pitch: Int,
    val roll: Int,
    val wheelSpeed: Int,
    val battery: Int,
    val mode: Int,
    val fault: Int,
    val flags: Int,
    val peerDemand: Int,
    val wheelPos: Int,
    val obs: CyclicObs,
) {
    /** Rider-present flag, bit0. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_RIDER`. */
    fun riderPresent(): Boolean = flags and FLAG_RIDER != 0

    /** Rigid-frame leader flag, bit1. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LEADER`. */
    fun leader(): Boolean = flags and FLAG_LEADER != 0

    /** Rigid-frame follower flag, bit2. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_FOLLOWER`. */
    fun follower(): Boolean = flags and FLAG_FOLLOWER != 0

    /**
     * Whether any FAULT-A producer is asserted: [fault] masked by [FAULT_A_BITS], which is what
     * the firmware's own aggregate computes (`crates/orchestrator/src/events.rs`, `Levels::fault_a`).
     */
    fun faultA(): Boolean = fault and FAULT_A_BITS != 0

    /** Lockdown flag, bit7. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LOCKDOWN`. */
    fun lockdown(): Boolean = flags and FLAG_LOCKDOWN != 0

    /**
     * Encode: the committed words, then the appended block, [ENCODED_LEN] bytes.
     * `crates/linkctl/src/lib.rs`, `CyclicState::encode`.
     */
    fun encode(): ByteArray {
        val out = ByteArray(ENCODED_LEN)
        wrU16(out, 0, pitch)
        wrU16(out, 2, roll)
        wrU16(out, 4, wheelSpeed)
        wrU16(out, 6, battery)
        out[8] = mode.toByte()
        out[9] = fault.toByte()
        out[10] = flags.toByte()
        wrU16(out, 11, peerDemand)
        wrU16(out, 13, wheelPos)
        obs.encode().copyInto(out, LEN)
        return out
    }

    companion object {
        /**
         * Length of the committed words, which is the offset the appended block sits at.
         * `crates/linkctl/src/lib.rs`, `CyclicState::LEN`.
         */
        const val LEN = 15

        /**
         * The ONE on-wire length of a `CYCLIC_STATE` payload: what the firmware emits and the only
         * length [decode] accepts. `crates/linkctl/src/lib.rs`, `CyclicState::ENCODED_LEN`.
         */
        const val ENCODED_LEN = LEN + CyclicObs.LEN

        /** `flags` bit0: rider present. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_RIDER`. */
        const val FLAG_RIDER = 1 shl 0

        /** `flags` bit1: rigid-frame leader. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LEADER`. */
        const val FLAG_LEADER = 1 shl 1

        /** `flags` bit2: rigid-frame follower. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_FOLLOWER`. */
        const val FLAG_FOLLOWER = 1 shl 2

        /**
         * The FAULT-A group of the [fault] mask: comms loss, the tilt cutout, IMU loss, the
         * motor-side fault level and motor 0's latch. `crates/orchestrator/src/events.rs`, `FAULT_A_BITS`
         * (bits 0, 1, 2, 3 and 4 of that file's `EV_*` allocation).
         */
        const val FAULT_A_BITS = 0x1F

        /** `flags` bit7: lockdown. `crates/linkctl/src/lib.rs`, `CyclicState::FLAG_LOCKDOWN`. */
        const val FLAG_LOCKDOWN = 1 shl 7

        /**
         * Decode a payload of exactly [ENCODED_LEN]; null for ANY other length, short or long.
         * `crates/linkctl/src/lib.rs`, `CyclicState::decode`.
         *
         * The exception to this file's committed-prefix rule, and the firmware's decoder makes the
         * same one. The committed prefix alone decoded here until 2026-10-10, for a board on an image from
         * before the appended block: nothing is owed an older image before a release, and the
         * other argument for it (a truncated frame must not cost the lockdown flag) was wrong on
         * the framing, since L2 drops a frame whose CRC-16 fails and resyncs at the next SOF
         * (`specs/l2.md`, "The L2 frame"). A payload of any other length is a peer that is not speaking this
         * protocol, and the delivery class makes dropping it the whole response.
         */
        fun decode(b: ByteArray): CyclicState? {
            if (b.size != ENCODED_LEN) return null
            return CyclicState(
                pitch = rdI16(b, 0),
                roll = rdI16(b, 2),
                wheelSpeed = rdI16(b, 4),
                battery = rdU16(b, 6),
                mode = rdU8(b, 8),
                fault = rdU8(b, 9),
                flags = rdU8(b, 10),
                peerDemand = rdI16(b, 11),
                wheelPos = rdI16(b, 13),
                obs = CyclicObs.decode(b, LEN),
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

// --- INPUTS (2 B) -------------------------------------------------------------------------------

/**
 * Remote input mirror: the LEVELS a controller asserts about itself. Mirror of
 * `crates/linkctl/src/lib.rs`, `Inputs`.
 *
 * Wire layout, 2 bytes (`crates/linkctl/src/lib.rs`, `Inputs::encode`):
 * ```
 * off 0      u8      buttons   bit0 power request
 * off 1      u8      rider     bit0 rider present
 * ```
 *
 * A raw `throttle` i16 used to lead this payload, mirroring a board's own throttle HARDWARE into a
 * filter nothing read; it is deleted (`specs/todo.md`, "Robo power model" part), which shifted both
 * remaining fields.
 * Demand does not travel here at all: it is [DriveCmd].
 */
data class Inputs(val buttons: Int, val rider: Int) {
    /** Power-request level, `buttons` bit0. `crates/linkctl/src/lib.rs`, `Inputs::power_request`. */
    fun powerRequest(): Boolean = buttons and BUTTON_POWER != 0

    /** Rider-present level, `rider` bit0. `crates/linkctl/src/lib.rs`, `Inputs::rider_present`. */
    fun riderPresent(): Boolean = rider and RIDER_PRESENT != 0

    /** Encode the committed prefix. `crates/linkctl/src/lib.rs`, `Inputs::encode`. */
    fun encode(): ByteArray {
        val out = ByteArray(LEN)
        out[0] = buttons.toByte()
        out[1] = rider.toByte()
        return out
    }

    companion object {
        /** On-wire length of the committed prefix. `crates/linkctl/src/lib.rs`, `Inputs::LEN`. */
        const val LEN = 2

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
            return Inputs(buttons = rdU8(b, 0), rider = rdU8(b, 1))
        }
    }
}

// --- Dispatch -----------------------------------------------------------------------------------

/** A decoded control-block payload, tagged by family. `crates/linkctl/src/lib.rs`, `Payload`. */
sealed class ControlPayload {
    data class Cyclic(val state: CyclicState) : ControlPayload()

    data class Drive(val cmd: DriveCmd) : ControlPayload()

    data class Input(val inputs: Inputs) : ControlPayload()
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
    else -> null
}
