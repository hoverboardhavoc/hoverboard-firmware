package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields

/**
 * The board's boot status record, as a `STATUS` reply to `STATUS_READ` carries it
 * (`specs/machine-settings.md` section 6.1): what did not come up when the board brought its layout
 * up (`specs/configured-links.md` section 5; `motorcontroller/board/src/status.rs`, which
 * `RustSourceDriftTest` pins this against).
 *
 * The payload is the record's entries as bytes, [ENTRY_LEN] per entry and at most [MAX_ENTRIES] of
 * them. No entries means everything configured came up. A bridge answers it empty.
 */
object BoardStatus {
    /** The most entries the record holds (`status::MAX_ENTRIES`). */
    const val MAX_ENTRIES = 14

    /** The bytes of one entry: field, index, reason, detail (`status::ENTRY_LEN`). */
    const val ENTRY_LEN = 4

    /** Why a function is absent: byte 2 of an entry (`status::Reason`), with what byte 3 means. */
    enum class Reason(val code: Int) {
        /** A stored byte is not a pin encoding. Detail: the byte. */
        NOT_A_PIN(1),

        /** The group is incomplete. Detail: the id of the first unset field. */
        INCOMPLETE(2),

        /** The chip does not route this (no ADC channel, I2C instance, gate timer or USART). Detail: the first pin. */
        NOT_ROUTED(3),

        /** An SWD pin (PA13/PA14). Detail: the pin. */
        SWD_PIN(4),

        /** The gate group's dead time is below [DEAD_TIME_MIN_DTG]. Detail: the stored dead time. */
        DEAD_TIME(5),

        /** The peripheral did not come up. Detail: the HAL's error code, or 0 for a probe that did not answer. */
        NOT_UP(6),

        /** The IMU frame is not a proper rotation. Detail: the first `imu.axis_sign` index of the refused triple. */
        IMU_FRAME(7),
        ;

        companion object {
            /** The reason [code] names, or null for a byte no reason has. */
            fun of(code: Int): Reason? = entries.firstOrNull { it.code == code }
        }
    }

    /**
     * One entry: the function named by [field] (its group's first field id, or its own) at [index]
     * (the motor or the link; 0 for a single field) is absent for [reasonCode], with [detail].
     */
    data class Entry(val field: Int, val index: Int, val reasonCode: Int, val detail: Int) {
        /** The [Reason] this entry carries, or null for a byte no reason has. */
        val reason: Reason? get() = Reason.of(reasonCode)
    }

    /** Decode a `STATUS` payload into its entries. A trailing partial entry is ignored. */
    fun decode(bytes: ByteArray): List<Entry> =
        bytes.toList().chunked(ENTRY_LEN).filter { it.size == ENTRY_LEN }.map { e ->
            Entry(e[0].toInt() and 0xFF, e[1].toInt() and 0xFF, e[2].toInt() and 0xFF, e[3].toInt() and 0xFF)
        }

    /** How a named field is indexed (`store::Index`): one value, one per motor, an array, or one per link. */
    enum class IndexKind { SINGLE, MOTOR, ARRAY, LINK }

    /** A field's name in the store's name table (`store::NAMES`): its path and how it is indexed. */
    data class Name(val path: String, val index: IndexKind) {
        /** The path index [i] prints as, the way the CLI prints it: `motors[0].hall.a`, `links[1].tx`. */
        fun at(i: Int): String = when (index) {
            IndexKind.SINGLE -> path
            IndexKind.MOTOR -> "motors[$i].$path"
            IndexKind.ARRAY -> "$path[$i]"
            IndexKind.LINK -> "links[$i].$path"
        }
    }

    /**
     * The names of the fields an entry can carry, by id: the function fields of byte 0 and the
     * group members a [Reason.INCOMPLETE] detail names, which are the layout fields, the IMU frame
     * and the links. Each row is pinned against its `store::NAMES` row by `RustSourceDriftTest`.
     */
    val NAMES: Map<Int, Name> = listOf(
        Fields.BOARD_SELF_HOLD to "power.self_hold",
        Fields.BOARD_BATTERY_PIN to "battery_sense.pin",
        Fields.BOARD_BUZZER to "buzzer",
        Fields.LED_GREEN to "leds.green",
        Fields.LED_ORANGE to "leds.orange",
        Fields.LED_RED to "leds.red",
        Fields.PAD_A to "pads.a",
        Fields.PAD_B to "pads.b",
        Fields.BOARD_BUTTON to "power.button",
        Fields.IMU_SCL_PIN to "imu.scl",
        Fields.IMU_SDA_PIN to "imu.sda",
        Fields.IMU_MODEL to "imu.model",
    ).associate { (def, path) -> def.id to Name(path, IndexKind.SINGLE) } +
        listOf(
            Fields.MOTOR_HALL_A to "hall.a",
            Fields.MOTOR_HALL_B to "hall.b",
            Fields.MOTOR_HALL_C to "hall.c",
            Fields.MOTOR_GATE_HI_A to "gate.hi_a",
            Fields.MOTOR_GATE_HI_B to "gate.hi_b",
            Fields.MOTOR_GATE_HI_C to "gate.hi_c",
            Fields.MOTOR_GATE_LO_A to "gate.lo_a",
            Fields.MOTOR_GATE_LO_B to "gate.lo_b",
            Fields.MOTOR_GATE_LO_C to "gate.lo_c",
            Fields.MOTOR_DEAD_TIME to "gate.dead_time",
            Fields.MOTOR_PHASE_A to "current.phase_a",
            Fields.MOTOR_PHASE_B to "current.phase_b",
        ).associate { (def, path) -> def.id to Name(path, IndexKind.MOTOR) } +
        listOf(
            Fields.IMU_AXIS_SIGN.id to "imu.axis_sign",
            Fields.IMU_AXIS_ROLE.id to "imu.axis_role",
        ).associate { (id, path) -> id to Name(path, IndexKind.ARRAY) } +
        listOf(
            Fields.LINK_TX.id to "tx",
            Fields.LINK_RX.id to "rx",
        ).associate { (id, path) -> id to Name(path, IndexKind.LINK) }
}
