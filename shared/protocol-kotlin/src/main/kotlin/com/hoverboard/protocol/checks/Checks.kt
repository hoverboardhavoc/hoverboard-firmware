package com.hoverboard.protocol.checks

import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.pincap.Func
import com.hoverboard.protocol.pincap.FuncKind
import com.hoverboard.protocol.pincap.PinTable
import com.hoverboard.protocol.pincap.SWD
import com.hoverboard.protocol.store.Effect
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Names
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import com.hoverboard.protocol.store.fleetPin

/**
 * The checks a board's configuration passes before it is written (`specs/config-read-once.md`
 * section 4): a port of `tools/hoverboard-cli/src/checks.rs`, with the same check ids, the same
 * order, and the same key named by each refusal. The firmware applies what is stored and refuses
 * nothing, so the app runs these before every write, as the CLI does, and writes nothing when one
 * refuses.
 *
 * The two implementations are held together by the shared check cases (`shared/config-checks/`),
 * which both test suites run, and the tables they read ([com.hoverboard.protocol.pincap], the
 * range table [Fields.RANGES]) are mirrors pinned to the Rust.
 */

/** One check of section 4, by the id the shared cases name it with (`checks::Check`). */
enum class Check(val id: String) {
    /** The port and pin exist on the chip. */
    PIN_EXISTS("pin_exists"),

    /** PA13 and PA14 (SWD) are never used. */
    SWD("swd"),

    /** No pin is used twice. */
    PIN_TWICE("pin_twice"),

    /** A group is complete or absent. */
    GROUP("group"),

    /** The instance key names a peripheral the chip has. */
    INSTANCE("instance"),

    /** No peripheral instance serves two groups. */
    INSTANCE_TWICE("instance_twice"),

    /** On a chip with a per-pin mux, the stored `af` selects the function the field needs. */
    AF("af"),

    /** On the GD32F10x, the group's remap routes the pin to the function the field needs. */
    REMAP("remap"),

    /** A `PinAf` carries `af` on a mux chip and leaves it out on the GD32F10x. */
    AF_FORM("af_form"),

    /** A remap key is stored only on a chip with AFIO. */
    REMAP_CHIP("remap_chip"),

    /** A `PinAdc` stores the ADC channel of its pin. */
    ADC("adc"),

    /** A configured gate group's dead time is at or above the floor. */
    DEAD_TIME("dead_time"),

    /** An arm-row or calibration value is in its range. */
    RANGE("range"),

    /** The IMU axis signs and roles form a proper rotation. */
    FRAME("frame"),
}

/** A check that refused, the key it names, and why ([msg] starts with the key's path). */
data class Refusal(val check: Check, val key: Key, val msg: String)

/**
 * A `PinAf` value as a write gives it: its [key], and whether the `af` was given. A settings file
 * leaves `af` out on the GD32F10x and gives it on the other chips ([Check.AF_FORM]).
 */
data class PinForm(val key: Key, val afGiven: Boolean)

/** What a pin of a group carries (`checks::Role`). */
sealed interface Role {
    /** A plain input or output: no alternate function. */
    data object Plain : Role

    /** An ADC input. */
    data object Adc : Role

    /** A peripheral function [kind], on the timer channel [ch] for a gate. */
    data class Af(val kind: FuncKind, val ch: Int = 0) : Role {
        /** The function on peripheral instance [n]. */
        fun func(n: Int): Func = Func(kind, n, ch)
    }
}

/** What an instance key names. */
enum class Periph(val label: String) {
    TIMER("advanced timer"),
    USART("USART"),
    I2C("I2C"),
    ;

    fun has(t: PinTable, n: Int): Boolean = when (this) {
        TIMER -> t.hasTimer(n)
        USART -> t.hasUsart(n)
        I2C -> t.hasI2c(n)
    }
}

/**
 * A function's keys: its pins with their roles, and for a peripheral group its instance key and
 * remap key. Complete when every pin and the instance are stored, absent when none is. A group
 * with a [switch] (the IMU's `imu.model`) is all or none with the switch: off (0 or not stored),
 * its pins and instance may be stored or not.
 */
class Group(
    val pins: List<Pair<PinSlot, Role>>,
    val instance: Pair<Key, Periph>? = null,
    val remap: Key? = null,
    val switch: Key? = null,
)

/** One pin field at one index: its key and its registered field. */
data class PinSlot(val def: FieldDef, val index: Int) {
    val key: Key get() = def.key(index)
}

/**
 * The checks, run on a board's stored values as a write would leave them: the newest value of
 * each stored key, a key that is not stored absent from the map.
 */
object Checks {
    /** Every function a board can configure, by its groups (`checks::groups`). */
    val GROUPS: List<Group> = buildList {
        add(Group(listOf(PinSlot(Fields.BOARD_SELF_HOLD, 0) to Role.Plain)))
        add(Group(listOf(PinSlot(Fields.BOARD_BATTERY_PIN, 0) to Role.Adc)))
        for (f in listOf(
            Fields.BOARD_BUZZER, Fields.LED_GREEN, Fields.LED_ORANGE, Fields.LED_RED,
            Fields.PAD_A, Fields.PAD_B, Fields.BOARD_BUTTON,
        )) {
            add(Group(listOf(PinSlot(f, 0) to Role.Plain)))
        }
        add(
            Group(
                listOf(
                    PinSlot(Fields.IMU_SCL_PIN, 0) to Role.Af(FuncKind.SCL),
                    PinSlot(Fields.IMU_SDA_PIN, 0) to Role.Af(FuncKind.SDA),
                ),
                Fields.IMU_I2C.key() to Periph.I2C,
                Fields.IMU_REMAP.key(),
                Fields.IMU_MODEL.key(),
            ),
        )
        for (i in 0 until Fields.LINKS) {
            add(
                Group(
                    listOf(
                        PinSlot(Fields.LINK_TX, i) to Role.Af(FuncKind.TX),
                        PinSlot(Fields.LINK_RX, i) to Role.Af(FuncKind.RX),
                    ),
                    Key(Fields.LINK_USART.id, i) to Periph.USART,
                    Key(Fields.LINK_REMAP.id, i),
                ),
            )
        }
        for (m in 0 until Fields.MOTORS) {
            add(
                Group(
                    listOf(Fields.MOTOR_HALL_A, Fields.MOTOR_HALL_B, Fields.MOTOR_HALL_C)
                        .map { PinSlot(it, m) to Role.Plain },
                ),
            )
            add(
                Group(
                    listOf(
                        PinSlot(Fields.MOTOR_GATE_HI_A, m) to Role.Af(FuncKind.GATE, 0),
                        PinSlot(Fields.MOTOR_GATE_HI_B, m) to Role.Af(FuncKind.GATE, 1),
                        PinSlot(Fields.MOTOR_GATE_HI_C, m) to Role.Af(FuncKind.GATE, 2),
                        PinSlot(Fields.MOTOR_GATE_LO_A, m) to Role.Af(FuncKind.GATE_N, 0),
                        PinSlot(Fields.MOTOR_GATE_LO_B, m) to Role.Af(FuncKind.GATE_N, 1),
                        PinSlot(Fields.MOTOR_GATE_LO_C, m) to Role.Af(FuncKind.GATE_N, 2),
                    ),
                    Fields.MOTOR_GATE_TIMER.key(m) to Periph.TIMER,
                    Fields.MOTOR_GATE_REMAP.key(m),
                ),
            )
            add(
                Group(
                    listOf(Fields.MOTOR_PHASE_A, Fields.MOTOR_PHASE_B).map { PinSlot(it, m) to Role.Adc },
                ),
            )
        }
    }

    /** The role the pin at [key] plays in its group, or null for a key that is no group's pin. */
    fun roleOf(key: Key): Role? =
        GROUPS.firstNotNullOfOrNull { g -> g.pins.firstOrNull { it.first.key == key }?.second }

    /** The pin [key] names as (port, pin), stored or by its fleet default, or null when unset. */
    fun pinAt(state: Map<Key, Value>, key: Key): Pair<Int, Int>? {
        val slot = GROUPS.firstNotNullOfOrNull { g -> g.pins.firstOrNull { it.first.key == key }?.first }
            ?: return null
        return pinOf(state, slot)?.let { it[0] to it[1] }
    }

    /**
     * Every check, in the order the CLI runs them on a write (`config::check_plan`): the value
     * checks, the `af` rule of the written pins, then the pin checks on [chip].
     */
    fun run(chip: Chip, state: Map<Key, Value>, forms: List<PinForm>): List<Refusal> =
        checkValues(state) + checkForms(chip, forms) + checkPins(chip, state)

    /** The stored bytes of a pin field, or its fleet default (`checks::pin_of`). */
    private fun pinOf(state: Map<Key, Value>, slot: PinSlot): List<Int>? {
        val v = state[slot.key] ?: return slot.def.fleetPin?.let { (port, pin) -> listOf(port, pin, 0) }
        if (v.kind() != slot.def.type) return null
        return when (v) {
            is Value.Pin -> listOf(v.port, v.pin)
            is Value.PinAf -> listOf(v.port, v.pin, v.af)
            is Value.PinAdc -> listOf(v.port, v.pin, v.adc)
            else -> null
        }
    }

    private fun byteOf(state: Map<Key, Value>, key: Key): Int? = (state[key] as? Value.U8)?.v

    private fun intOf(v: Value): Long? = when (v) {
        is Value.U8 -> v.v.toLong()
        is Value.U16 -> v.v.toLong()
        is Value.U32 -> v.v
        is Value.I16 -> v.v.toLong()
        is Value.I32 -> v.v.toLong()
        is Value.Bool -> if (v.v) 1L else 0L
        else -> null
    }

    /** `PA8`. */
    private fun pinText(port: Int, pin: Int): String = "P${'A' + port.coerceAtMost(25)}$pin"

    private fun refusal(check: Check, key: Key, msg: String) = Refusal(check, key, "${Names.pathOf(key)}: $msg")

    /** Every check that needs the chip, on the board as [state] would leave it (`checks::check_pins`). */
    @Suppress("CyclomaticComplexMethod", "LongMethod", "NestedBlockDepth")
    fun checkPins(chip: Chip, state: Map<Key, Value>): List<Refusal> {
        val t = chip.table
        val out = mutableListOf<Refusal>()
        val name = chip.id
        // Every pin that is set, stored or by its fleet default.
        val used = mutableMapOf<Pair<Int, Int>, Key>()
        for (g in GROUPS) {
            for ((slot, _) in g.pins) {
                val b = pinOf(state, slot) ?: continue
                val (port, pin) = b[0] to b[1]
                val at = pinText(port, pin)
                if (!t.bonded(port, pin)) {
                    out += refusal(Check.PIN_EXISTS, slot.key, "$at does not exist on the $name")
                    continue
                }
                if ((port to pin) in SWD) {
                    out += refusal(Check.SWD, slot.key, "$at is an SWD pin, the configuration path")
                    continue
                }
                val first = used[port to pin]
                if (first != null) {
                    out += refusal(Check.PIN_TWICE, slot.key, "$at is already ${Names.pathOf(first)}")
                } else {
                    used[port to pin] = slot.key
                }
            }
        }
        // Each group: complete or absent, and a complete one routed on this chip.
        val instances = mutableMapOf<Pair<Periph, Int>, Key>()
        for (g in GROUPS) {
            val set = g.pins.map { (slot, _) -> slot.key in state || slot.def.fleetPin != null }
            val instance = g.instance?.let { (k, p) -> Triple(k, p, byteOf(state, k)) }
            val all = set.all { it } && (instance == null || instance.third != null)
            val none = set.none { it } && (instance == null || instance.third == null)
            // A switched group: on, it is all or none with its switch, so it needs every key; off,
            // its keys may be stored or not, and a whole bus is still routed below.
            val on = g.switch?.let { (byteOf(state, it) ?: 0) != 0 }
            if (on == false && !all) continue
            if (none && on != true) continue
            if (!all) {
                val missing = g.pins.zip(set).firstOrNull { !it.second }?.first?.first?.key
                    ?: checkNotNull(instance).first
                out += refusal(
                    Check.GROUP,
                    g.pins.first().first.key,
                    "the group is incomplete: ${Names.pathOf(missing)} is not stored",
                )
                continue
            }
            var n: Int? = null
            if (instance != null) {
                val (k, p, stored) = instance
                val inst = checkNotNull(stored)
                if (!p.has(t, inst)) {
                    out += refusal(Check.INSTANCE, k, "the $name has no ${p.label} $inst")
                    continue
                }
                val first = instances[p to inst]
                if (first != null) {
                    out += refusal(Check.INSTANCE_TWICE, k, "${p.label} $inst is already ${Names.pathOf(first)}")
                } else {
                    instances[p to inst] = k
                }
                n = inst
            }
            val remap = g.remap?.let { byteOf(state, it) } ?: 0
            for ((slot, role) in g.pins) {
                val b = pinOf(state, slot) ?: continue
                val (port, pin) = b[0] to b[1]
                val at = pinText(port, pin)
                if (role == Role.Adc) {
                    val want = t.adcChannel(port, pin)
                    if (want != b[2]) {
                        val msg = if (want != null) {
                            "$at is ADC channel $want, not ${b[2]}"
                        } else {
                            "$at is not an ADC input on the $name"
                        }
                        out += refusal(Check.ADC, slot.key, msg)
                    }
                    continue
                }
                val func = (role as? Role.Af)?.let { r -> n?.let { r.func(it) } } ?: continue
                val sel = t.sel(port, pin, func)
                if (chip.remaps) {
                    if (sel != remap) {
                        val msg = if (sel != null) {
                            "$at is $func at remap $sel, not $remap"
                        } else {
                            "$at cannot carry $func on the $name"
                        }
                        out += refusal(Check.REMAP, slot.key, msg)
                    }
                } else if (sel != b[2]) {
                    val msg = if (sel != null) {
                        "$at is $func at AF$sel, not AF${b[2]}"
                    } else {
                        "$at cannot carry $func on the $name"
                    }
                    out += refusal(Check.AF, slot.key, msg)
                }
            }
            if (g.instance?.second == Periph.TIMER) {
                val m = g.pins.first().first.index
                val key = Fields.MOTOR_DEAD_TIME.key(m)
                val dt = byteOf(state, key) ?: 0
                if (dt < DEAD_TIME_MIN_DTG) {
                    out += refusal(
                        Check.DEAD_TIME,
                        key,
                        "$dt is below the floor of $DEAD_TIME_MIN_DTG with the gates configured",
                    )
                }
            }
        }
        // A remap key is a GD32F10x key.
        if (!chip.remaps) {
            val remaps = listOf(Fields.IMU_REMAP.key()) +
                (0 until Fields.LINKS).map { Key(Fields.LINK_REMAP.id, it) } +
                (0 until Fields.MOTORS).map { Fields.MOTOR_GATE_REMAP.key(it) }
            for (k in remaps) {
                if (k in state) out += refusal(Check.REMAP_CHIP, k, "the $name has no AFIO remap")
            }
        }
        return out
    }

    /**
     * The `af` rule of the written `PinAf` values (`checks::check_forms`): given on a chip with a
     * per-pin mux, left out on the GD32F10x, whose `af` byte is not used. [forms] in write order.
     */
    fun checkForms(chip: Chip, forms: List<PinForm>): List<Refusal> = forms.mapNotNull { f ->
        if (Fields.typeOf(f.key.fieldId) != Type.PinAf) return@mapNotNull null
        val msg = when {
            chip.remaps && f.afGiven -> "the ${chip.id} routes by its remap and stores no af: leave af out"
            !chip.remaps && !f.afGiven -> "the ${chip.id} needs the pin's af"
            else -> return@mapNotNull null
        }
        refusal(Check.AF_FORM, f.key, msg)
    }

    /** Every check that needs no chip: the range table and the IMU frame (`checks::check_values`). */
    fun checkValues(state: Map<Key, Value>): List<Refusal> {
        val out = mutableListOf<Refusal>()
        for (key in state.keys.sortedWith(compareBy({ it.fieldId }, { it.index }))) {
            // The arm row and the calibration only: another field's range is the forceable one a
            // tool's encoding makes (`checks.rs`, `check_values`).
            val arm = Fields.ALL.values.any { it.id == key.fieldId && it.effect == Effect.ARM } ||
                Fields.INDEXED.values.any { it.id == key.fieldId && it.effect == Effect.ARM }
            val boardCheck = arm || key.fieldId in Fields.CALIBRATION
            if (!boardCheck) continue
            val range = Fields.rangeOf(key) ?: continue
            val v = intOf(state.getValue(key)) ?: continue
            if (v !in range) out += refusal(Check.RANGE, key, "$v is outside ${range.first}..${range.last}")
        }
        val signs = (0 until Fields.IMU_AXIS_SIGN.size).map { i ->
            (state[Key(Fields.IMU_AXIS_SIGN.id, i)] as? Value.I32)?.v ?: 0
        }
        val roles = (0 until Fields.IMU_AXIS_ROLE.size).map { i -> byteOf(state, Key(Fields.IMU_AXIS_ROLE.id, i)) ?: 0 }
        when (Orientation.frameError(signs, roles)) {
            Orientation.FrameError.ROLES -> out += refusal(
                Check.FRAME,
                Key(Fields.IMU_AXIS_ROLE.id, 0),
                "the two roles must name two different chip axes",
            )
            Orientation.FrameError.ACCEL -> out += refusal(
                Check.FRAME,
                Key(Fields.IMU_AXIS_SIGN.id, 0),
                "the signs read through the roles are a mirror image, not a rotation",
            )
            null -> Unit
        }
        return out
    }

    /**
     * Whether writing [key] changes what the pin checks see: a pin, instance, remap, dead-time or
     * `imu.model` key (`checks::needs_chip`). Such a write is refused while the board's chip is not
     * known.
     */
    fun needsChip(key: Key): Boolean {
        val pin = Fields.typeOf(key.fieldId) in setOf(Type.Pin, Type.PinAf, Type.PinAdc)
        val extra = setOf(
            Fields.IMU_I2C.id,
            Fields.IMU_MODEL.id,
            Fields.IMU_REMAP.id,
            Fields.LINK_USART.id,
            Fields.LINK_REMAP.id,
            Fields.MOTOR_GATE_TIMER.id,
            Fields.MOTOR_GATE_REMAP.id,
            Fields.MOTOR_DEAD_TIME.id,
        )
        return pin || key.fieldId in extra
    }
}
