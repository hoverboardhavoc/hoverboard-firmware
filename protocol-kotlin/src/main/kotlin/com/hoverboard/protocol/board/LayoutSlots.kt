package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value

/**
 * One store field a layout is made of: the registered field, the motor it names where the field is
 * per-motor, and how it sits in [BoardFields].
 *
 * [boardField] is the validator's name for the slot where it has one, which is how a refusal
 * ([BoardError]) is matched back to the slot that caused it. The three carried per-motor facts
 * (direction, align offset, counts per amp) have none, because the validator never names them: it
 * carries them into the plan without checking them.
 */
class LayoutSlot internal constructor(
    val def: FieldDef,
    /** The motor this slot names, or null for a singleton. */
    val motor: Int?,
    /** The validator's name for this slot, or null for a carried fact it never names. */
    val boardField: BoardField?,
    /** Whether the value is a packed pin (so [PIN_ABSENT] means absent and [Pin] can spell it). */
    val isPin: Boolean,
    private val read: (BoardFields) -> Int,
    private val write: (BoardFields, Int) -> BoardFields,
) {
    /** The key naming this slot on its board. */
    val key: Key get() = def.key(motor ?: 0)

    /** This slot's raw value in [fields]. */
    fun of(fields: BoardFields): Int = read(fields)

    /** [fields] with this slot set to [raw]. */
    fun on(fields: BoardFields, raw: Int): BoardFields = write(fields, raw)

    /** [raw] as the value one `CONFIG_WRITE` carries for this slot, in the field's own type. */
    fun value(raw: Int): Value = when (def.type) {
        Type.U8 -> Value.U8(raw)
        Type.U16 -> Value.U16(raw)
        else -> error("$key is a ${def.type} field, which no layout slot holds")
    }

    /** The raw value [v] holds for this slot, or null when it is not the field's own type. */
    fun raw(v: Value): Int? = when {
        def.type == Type.U8 && v is Value.U8 -> v.v
        def.type == Type.U16 && v is Value.U16 -> v.v
        else -> null
    }

    override fun toString(): String = "LayoutSlot(${def.id}/${motor ?: 0})"
}

/**
 * The layout as ONE object: which store fields it is made of, and the conversions between those
 * fields and [BoardFields].
 *
 * [SLOTS] is the Kotlin mirror of `board::plumbing`'s `read_fields`, the function that assembles the
 * same struct from the same registered fields at boot, and `RustSourceDriftTest` pins the two
 * against each other. That matters more here than it looks: a layout field this table forgets is a
 * field the board validates and a client's prediction does not, which turns an exact verdict back
 * into a guess.
 *
 * Both motors are carried, not just the one the firmware consumes today. The verdict is a function
 * of the WHOLE field set (a motor-1 pin can duplicate a motor-0 pin, and a half-staged motor-1
 * group is a refusal), so a client that read only motor 0 could predict a clean boot for a board
 * that refuses its layout.
 */
object Layout {
    /**
     * Every field of a layout, in the order `read_fields` assembles them: the singletons, then each
     * motor's pins and facts.
     */
    val SLOTS: List<LayoutSlot> = buildList {
        fun pin(def: FieldDef, f: BoardField, read: (BoardFields) -> Int, write: (BoardFields, Int) -> BoardFields) =
            add(LayoutSlot(def, null, f, isPin = true, read = read, write = write))

        pin(Fields.BOARD_SELF_HOLD, BoardField.SELF_HOLD, { it.selfHold }, { b, v -> b.copy(selfHold = v) })
        pin(Fields.BOARD_VBATT, BoardField.VBATT, { it.vbatt }, { b, v -> b.copy(vbatt = v) })
        pin(Fields.BOARD_BUZZER, BoardField.BUZZER, { it.buzzer }, { b, v -> b.copy(buzzer = v) })
        pin(Fields.LED_GREEN, BoardField.LED_GREEN, { it.ledGreen }, { b, v -> b.copy(ledGreen = v) })
        pin(Fields.LED_ORANGE, BoardField.LED_ORANGE, { it.ledOrange }, { b, v -> b.copy(ledOrange = v) })
        pin(Fields.LED_RED, BoardField.LED_RED, { it.ledRed }, { b, v -> b.copy(ledRed = v) })
        pin(Fields.PAD_A, BoardField.PAD_A, { it.padA }, { b, v -> b.copy(padA = v) })
        pin(Fields.PAD_B, BoardField.PAD_B, { it.padB }, { b, v -> b.copy(padB = v) })
        pin(Fields.BOARD_BUTTON, BoardField.BUTTON, { it.button }, { b, v -> b.copy(button = v) })
        pin(Fields.IMU_SCL_PIN, BoardField.IMU_SCL, { it.imuScl }, { b, v -> b.copy(imuScl = v) })
        pin(Fields.IMU_SDA_PIN, BoardField.IMU_SDA, { it.imuSda }, { b, v -> b.copy(imuSda = v) })
        add(
            LayoutSlot(
                Fields.IMU_MODEL, null, BoardField.IMU_MODEL, isPin = false,
                read = { it.imuModel }, write = { b, v -> b.copy(imuModel = v) },
            ),
        )

        for (m in 0 until BoardFields.MOTORS) {
            fun motorSlot(
                def: FieldDef,
                f: BoardField?,
                isPin: Boolean,
                read: (MotorFields) -> Int,
                write: (MotorFields, Int) -> MotorFields,
            ) = add(
                LayoutSlot(
                    def, m, f, isPin,
                    read = { read(it.motors[m]) },
                    write = { b, v -> b.copy(motors = b.motors.mapIndexed { i, mf -> if (i == m) write(mf, v) else mf }) },
                ),
            )

            motorSlot(Fields.MOTOR_HALL_A, BoardField.HALL_A, true, { it.hallA }, { mf, v -> mf.copy(hallA = v) })
            motorSlot(Fields.MOTOR_HALL_B, BoardField.HALL_B, true, { it.hallB }, { mf, v -> mf.copy(hallB = v) })
            motorSlot(Fields.MOTOR_HALL_C, BoardField.HALL_C, true, { it.hallC }, { mf, v -> mf.copy(hallC = v) })
            motorSlot(Fields.MOTOR_GATE_HI_A, BoardField.GATE_HI_A, true, { it.gateHiA }, { mf, v -> mf.copy(gateHiA = v) })
            motorSlot(Fields.MOTOR_GATE_HI_B, BoardField.GATE_HI_B, true, { it.gateHiB }, { mf, v -> mf.copy(gateHiB = v) })
            motorSlot(Fields.MOTOR_GATE_HI_C, BoardField.GATE_HI_C, true, { it.gateHiC }, { mf, v -> mf.copy(gateHiC = v) })
            motorSlot(Fields.MOTOR_GATE_LO_A, BoardField.GATE_LO_A, true, { it.gateLoA }, { mf, v -> mf.copy(gateLoA = v) })
            motorSlot(Fields.MOTOR_GATE_LO_B, BoardField.GATE_LO_B, true, { it.gateLoB }, { mf, v -> mf.copy(gateLoB = v) })
            motorSlot(Fields.MOTOR_GATE_LO_C, BoardField.GATE_LO_C, true, { it.gateLoC }, { mf, v -> mf.copy(gateLoC = v) })
            motorSlot(Fields.MOTOR_DEAD_TIME, BoardField.DEAD_TIME, false, { it.deadTime }, { mf, v -> mf.copy(deadTime = v) })
            motorSlot(Fields.MOTOR_DIRECTION, null, false, { it.direction }, { mf, v -> mf.copy(direction = v) })
            motorSlot(Fields.MOTOR_ALIGN_OFFSET, null, false, { it.alignOffset }, { mf, v -> mf.copy(alignOffset = v) })
            motorSlot(
                Fields.MOTOR_CURRENT_SENSE, BoardField.CURRENT_SENSE, false,
                { it.currentSense }, { mf, v -> mf.copy(currentSense = v) },
            )
            motorSlot(Fields.MOTOR_CURRENT_CAL, null, false, { it.currentCal }, { mf, v -> mf.copy(currentCal = v) })
            motorSlot(Fields.MOTOR_PHASE_A, BoardField.PHASE_A, true, { it.phaseA }, { mf, v -> mf.copy(phaseA = v) })
            motorSlot(Fields.MOTOR_PHASE_B, BoardField.PHASE_B, true, { it.phaseB }, { mf, v -> mf.copy(phaseB = v) })
        }
    }

    /**
     * The power-latch field, named here because every client has to treat it specially: it is the
     * one field whose staging can leave a board unreachable (on battery a wrong pin powers the board
     * off at boot, recoverable only over SWD), and the validator resolves it first and reports it
     * whatever becomes of the rest of the layout.
     */
    val LATCH: LayoutSlot = checkNotNull(forField(FieldRef(BoardField.SELF_HOLD)))

    /** The slot [key] names, or null when the key is not part of a layout. */
    fun forKey(key: Key): LayoutSlot? = byKey[key]

    private val byKey: Map<Key, LayoutSlot> = SLOTS.associateBy { it.key }

    /** The slot the validator means by [ref], or null for a field no slot holds. */
    fun forField(ref: FieldRef): LayoutSlot? =
        SLOTS.firstOrNull { it.boardField == ref.field && it.motor == (if (it.boardField.perMotor) ref.motor else null) }

    /**
     * The layout [stored] describes, or null when any slot is missing or holds the wrong type: a
     * layout is one object, so a partial read is not a layout with gaps, it is no layout yet.
     */
    fun fieldsFrom(stored: Map<Key, Value>): BoardFields? {
        var fields = BoardFields()
        for (slot in SLOTS) {
            val raw = stored[slot.key]?.let(slot::raw) ?: return null
            fields = slot.on(fields, raw)
        }
        return fields
    }

    /** The slots [stored] has no usable value for (what a read pass still owes). */
    fun missing(stored: Map<Key, Value>): List<LayoutSlot> =
        SLOTS.filter { slot -> stored[slot.key]?.let(slot::raw) == null }

    /** Every slot whose value differs between [from] and [to], in [SLOTS] order. */
    fun delta(from: BoardFields, to: BoardFields): List<LayoutSlot> =
        SLOTS.filter { it.of(from) != it.of(to) }

    /** The writes that turn [from] into [to]: one `CONFIG_WRITE` per changed slot, in slot order. */
    fun writes(from: BoardFields, to: BoardFields): List<Pair<Key, Value>> =
        delta(from, to).map { it.key to it.value(it.of(to)) }
}
