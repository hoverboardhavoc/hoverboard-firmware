package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value

/**
 * One store key a layout is made of: the registered field, at [index], and the motor it names
 * where the field is per motor.
 */
class LayoutSlot internal constructor(
    val def: FieldDef,
    val index: Int,
    /** The motor this slot names, or null for a field that is not per motor. */
    val motor: Int?,
) {
    /** The key naming this slot on its board. */
    val key: Key get() = def.key(index)

    /** Whether the value is a pin ([Type.Pin], [Type.PinAf] or [Type.PinAdc]). */
    val isPin: Boolean get() = def.type in PIN_TYPES

    /** This slot's stored value in [fields], or null when the board stores none. */
    fun of(fields: BoardFields): Value? = fields.values[key]

    override fun toString(): String = "LayoutSlot(${def.id}/$index)"

    private companion object {
        val PIN_TYPES = setOf(Type.Pin, Type.PinAf, Type.PinAdc)
    }
}

/**
 * A board's layout: the stored value of each layout key. A key the board does not store is not in
 * [values]: a pin that is not stored is an absent function, or its fleet pin where the field has one
 * (`specs/config-read-once.md` section 3).
 */
data class BoardFields(val values: Map<Key, Value> = emptyMap()) {
    /** These fields with [slot] set to [value]. */
    fun with(slot: LayoutSlot, value: Value): BoardFields = copy(values = values + (slot.key to value))
}

/**
 * The layout as ONE object: which store keys it is made of, read and checked together. Every key
 * the pin checks read is here (the links' pins, USARTs and remaps too), so the checks run on the
 * whole board as a write would leave it.
 */
object Layout {
    /** Every key of a layout: the board's own pins, the IMU bus, the links, then each motor's. */
    val SLOTS: List<LayoutSlot> = buildList {
        fun single(def: FieldDef) = add(LayoutSlot(def, 0, null))

        for (def in listOf(
            Fields.BOARD_SELF_HOLD, Fields.BOARD_BATTERY_PIN, Fields.BOARD_BUZZER, Fields.LED_GREEN,
            Fields.LED_ORANGE, Fields.LED_RED, Fields.PAD_A, Fields.PAD_B, Fields.BOARD_BUTTON,
            Fields.IMU_SCL_PIN, Fields.IMU_SDA_PIN, Fields.IMU_MODEL, Fields.IMU_I2C, Fields.IMU_REMAP,
        )) {
            single(def)
        }
        for (i in 0 until Fields.LINKS) {
            add(LayoutSlot(Fields.LINK_TX, i, null))
            add(LayoutSlot(Fields.LINK_RX, i, null))
            add(LayoutSlot(Fields.LINK_USART.at(i), i, null))
            add(LayoutSlot(Fields.LINK_REMAP.at(i), i, null))
        }
        for (m in 0 until Fields.MOTORS) {
            for (def in listOf(
                Fields.MOTOR_HALL_A, Fields.MOTOR_HALL_B, Fields.MOTOR_HALL_C,
                Fields.MOTOR_GATE_HI_A, Fields.MOTOR_GATE_HI_B, Fields.MOTOR_GATE_HI_C,
                Fields.MOTOR_GATE_LO_A, Fields.MOTOR_GATE_LO_B, Fields.MOTOR_GATE_LO_C,
                Fields.MOTOR_GATE_TIMER, Fields.MOTOR_GATE_REMAP, Fields.MOTOR_DEAD_TIME,
                Fields.MOTOR_DIRECTION, Fields.MOTOR_ALIGN_OFFSET, Fields.MOTOR_CURRENT_CAL,
                Fields.MOTOR_PHASE_A, Fields.MOTOR_PHASE_B,
            )) {
                add(LayoutSlot(def, m, m))
            }
        }
    }

    private val byKey: Map<Key, LayoutSlot> = SLOTS.associateBy { it.key }

    /**
     * The power-latch field, named here because every client has to treat it specially: it is the
     * one field whose staging can leave a board unreachable (on battery a wrong pin powers the board
     * off at boot, recoverable only over SWD), and the firmware asserts it before any other
     * bring-up.
     */
    val LATCH: LayoutSlot = checkNotNull(forField(Fields.BOARD_SELF_HOLD))

    /** The slot [key] names, or null when the key is not part of a layout. */
    fun forKey(key: Key): LayoutSlot? = byKey[key]

    /** The slot holding [def] at [index] (the motor or the link), or null for a field no slot holds. */
    fun forField(def: FieldDef, index: Int? = null): LayoutSlot? = forKey(def.key(index ?: 0))

    /**
     * The layout [read] describes, or null when any slot is unread or holds the wrong type. [read]
     * holds each read slot's stored value, or null for a slot the board answered with its default
     * (stores none). A layout is one object, so a partial read is not a layout with gaps.
     */
    fun fieldsFrom(read: Map<Key, Value?>): BoardFields? {
        if (missing(read).isNotEmpty()) return null
        return BoardFields(SLOTS.mapNotNull { s -> read[s.key]?.let { s.key to it } }.toMap())
    }

    /** The slots [read] has no usable answer for (what a read pass has not read yet). */
    fun missing(read: Map<Key, Value?>): List<LayoutSlot> =
        SLOTS.filter { s -> s.key !in read || read[s.key]?.let { it.kind() != s.def.type } == true }

    /** Every slot whose value differs between [from] and [to], in [SLOTS] order. */
    fun delta(from: BoardFields, to: BoardFields): List<LayoutSlot> =
        SLOTS.filter { it.of(from) != it.of(to) }

    /**
     * The writes that turn [from] into [to]: one `CONFIG_WRITE` per changed slot, in slot order. A
     * slot [to] stores nothing for is not written: a layout edit sets values and never un-stores one.
     */
    fun writes(from: BoardFields, to: BoardFields): List<Pair<Key, Value>> =
        delta(from, to).mapNotNull { s -> s.of(to)?.let { s.key to it } }
}
