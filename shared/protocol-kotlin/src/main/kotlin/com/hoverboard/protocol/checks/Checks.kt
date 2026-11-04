package com.hoverboard.protocol.checks

import com.hoverboard.protocol.board.CurrentLimit
import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.pincap.ADC2
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.pincap.Func
import com.hoverboard.protocol.pincap.FuncKind
import com.hoverboard.protocol.pincap.Phase
import com.hoverboard.protocol.pincap.PinTable
import com.hoverboard.protocol.pincap.SWD
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Names
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value

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
 *
 * One check looks at the machine rather than at one board: [Checks.checkLeader], the rigid frame's
 * leader (`specs/rigid-leader.md` section 6.1), run only by a write that changes `machine.mode`,
 * `machine.frame` or `control.leader` ([Checks.changesLeader]).
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

    /** A motor's sensed phase pair is one the chip's image reads. */
    PHASE_PAIR("phase_pair"),

    /**
     * Motor 1 is stored only beside motor 0, and on a two-motor chip each motor's gates are on its
     * own timer.
     */
    MOTOR_TIMER("motor_timer"),

    /** A value with a range-table row (but `wifi.tx_power`) is in its range. */
    RANGE("range"),

    /** A motor's current limit converts above its noise floor. */
    LIMIT_FLOOR("limit_floor"),

    /** The IMU axis signs and roles form a proper rotation. */
    FRAME("frame"),

    /** A balance machine in the rigid frame has exactly one board set as leader. */
    LEADER("leader"),

    /**
     * A protection is enabled only on a motor that stores its input
     * (`specs/optional-protections.md` section 7).
     */
    PROTECT_INPUT("protect_input"),

    /** A protection input is stored only on a chip whose image reads it. */
    PROTECT_CHIP("protect_chip"),
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

    /**
     * A DC-link current input: an ADC input on ADC2, whose inserted group converts it
     * (`specs/optional-protections.md` section 3.2).
     */
    data object DcLink : Role

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
 * its pins and instance may be stored or not. A group with a [pick] (a motor's phase-current sense)
 * is complete when exactly that many of its pins are stored. A group with a [via] (a motor's break
 * input) has no instance of its own: its pin is routed on the timer that key names, another
 * group's instance.
 */
class Group(
    val pins: List<Pair<PinSlot, Role>>,
    val instance: Pair<Key, Periph>? = null,
    val remap: Key? = null,
    val switch: Key? = null,
    val pick: Int? = null,
    val via: Key? = null,
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
    /**
     * A motor's phase-current sense pins with the phase each names, in phase order: a motor senses
     * the two that are stored (`checks::SENSE_PHASES`, `specs/twelve-fet.md` section 5.3).
     */
    private val SENSE_PHASES: List<Pair<FieldDef, Phase>> = listOf(
        Fields.MOTOR_PHASE_A to Phase.A,
        Fields.MOTOR_PHASE_B to Phase.B,
        Fields.MOTOR_PHASE_C to Phase.C,
    )

    /**
     * The advanced timer each motor's gates use where the chip drives two (`checks::MOTOR_TIMER`,
     * `specs/twelve-fet.md` section 5.2): TIMER0 triggers the injected ADC group and starts TIMER7.
     */
    private val MOTOR_TIMER: List<Int> = listOf(0, 7)

    /** The gate timer's break input, routed on the motor's gate timer. */
    private val BREAK: Role = Role.Af(FuncKind.BREAK)

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
            add(Group(SENSE_PHASES.map { (def, _) -> PinSlot(def, m) to Role.Adc }, pick = 2))
            add(Group(listOf(PinSlot(Fields.MOTOR_DC_LINK_PIN, m) to Role.DcLink)))
            add(
                Group(
                    listOf(PinSlot(Fields.MOTOR_BREAK_PIN, m) to BREAK),
                    remap = Fields.MOTOR_GATE_REMAP.key(m),
                    via = Fields.MOTOR_GATE_TIMER.key(m),
                ),
            )
        }
    }

    /**
     * Whether the chip's image reads a pin of [role] (`checks::image_reads`): a protection input
     * only where the chip's table says so ([com.hoverboard.protocol.pincap.Inputs]), every other
     * pin everywhere.
     */
    private fun imageReads(t: PinTable, role: Role): Boolean = when (role) {
        Role.DcLink -> t.inputs.dcLink
        BREAK -> t.inputs.brk
        else -> true
    }

    /**
     * Whether [chip]'s image reads the pin at [key]: a protection input only where the chip's table
     * says so, every other key on every chip. A layout editor offers a pin only where it is read.
     */
    fun imageReads(chip: Chip, key: Key): Boolean = roleOf(key)?.let { imageReads(chip.table, it) } ?: true

    /**
     * A protection [setting] and the input it needs on its motor (`checks::check_values`,
     * `protect_input`): [pick] of the pin fields [inputs] stored, named [input] in a refusal.
     */
    private class Protection(
        val setting: FieldDef,
        val inputs: List<FieldDef>,
        val pick: Int,
        val input: (Int) -> String,
    ) {
        fun stored(state: Map<Key, Value>, m: Int): Boolean = inputs.count { it.key(m) in state } == pick
    }

    /**
     * The protections with an input (`specs/optional-protections.md` section 7): the DC-link limit
     * beside its pin, the break input's switch beside its pin, and the rail skip beside a sensed
     * pair (two of the phase pins, `checks::senses`).
     */
    private val PROTECTIONS: List<Protection> = listOf(
        Protection(Fields.MOTOR_DC_LINK_LIMIT, listOf(Fields.MOTOR_DC_LINK_PIN), 1) {
            Names.pathOf(Fields.MOTOR_DC_LINK_PIN.key(it))
        },
        Protection(Fields.MOTOR_BREAK_ENABLE, listOf(Fields.MOTOR_BREAK_PIN), 1) {
            Names.pathOf(Fields.MOTOR_BREAK_PIN.key(it))
        },
        Protection(Fields.MOTOR_RAIL_SKIP, SENSE_PHASES.map { it.first }, 2) {
            "a sensed pair (two of motors[$it].current.phase_a to phase_c)"
        },
    )

    /**
     * Whether [state] stores the input the protection setting at [key] needs on motor [Key.index]
     * (the `protect_input` rule), or null for a key that is no protection setting with an input.
     */
    fun storesProtectionInput(key: Key, state: Map<Key, Value>): Boolean? =
        PROTECTIONS.firstOrNull { it.setting.id == key.fieldId }?.stored(state, key.index)

    /** The pin keys motor [m]'s protection inputs are stored in, each once, in field order. */
    fun protectionInputKeys(m: Int): List<Key> = PROTECTIONS.flatMap { p -> p.inputs.map { it.key(m) } }.distinct()

    /** The role the pin at [key] plays in its group, or null for a key that is no group's pin. */
    fun roleOf(key: Key): Role? =
        GROUPS.firstNotNullOfOrNull { g -> g.pins.firstOrNull { it.first.key == key }?.second }

    /** The pin [key] stores as (port, pin), or null when it stores none. */
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

    /** The stored bytes of a pin field, or null when it is not stored (`checks::pin_of`). */
    private fun pinOf(state: Map<Key, Value>, slot: PinSlot): List<Int>? {
        val v = state[slot.key] ?: return null
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

    /** Whether every pin of [g] is stored, or exactly [Group.pick] of them, and its instance stored. */
    private fun complete(g: Group, state: Map<Key, Value>): Boolean {
        val set = g.pins.count { (slot, _) -> slot.key in state }
        return set == (g.pick ?: g.pins.size) && (g.instance == null || byteOf(state, g.instance.first) != null)
    }

    /**
     * The group check for [g] (`checks::check_pins`, "complete or absent"): its refusal when it is
     * partly stored, else null. A switched group (the IMU's `imu.model`) is all or none with its
     * switch: on, it needs every key; off (0 or not stored), its keys may be stored or not.
     */
    private fun groupRefusal(g: Group, state: Map<Key, Value>): Refusal? {
        val set = g.pins.map { (slot, _) -> slot.key in state }
        val instance = g.instance?.let { (k, _) -> k to byteOf(state, k) }
        val none = set.none { it } && (instance == null || instance.second == null)
        val on = g.switch?.let { (byteOf(state, it) ?: 0) != 0 }
        if (complete(g, state) || on == false || (none && on != true)) return null
        val head = g.pins.first().first.key
        if (g.pick != null) {
            val paths = g.pins.joinToString(", ") { Names.pathOf(it.first.key) }
            return refusal(Check.GROUP, head, "${g.pick} of $paths are stored together, not ${set.count { it }}")
        }
        val missing = g.pins.zip(set).firstOrNull { !it.second }?.first?.first?.key
            ?: checkNotNull(instance).first
        return refusal(Check.GROUP, head, "the group is incomplete: ${Names.pathOf(missing)} is not stored")
    }

    /**
     * The group check alone, which needs no chip: each group of [state] complete, absent or
     * switched off. Setup runs it before an `imu.model` write, with the IMU bus as the board
     * stores it.
     */
    fun checkGroups(state: Map<Key, Value>): List<Refusal> = GROUPS.mapNotNull { groupRefusal(it, state) }

    /** Every check that needs the chip, on the board as [state] would leave it (`checks::check_pins`). */
    @Suppress("CyclomaticComplexMethod", "LongMethod", "NestedBlockDepth")
    fun checkPins(chip: Chip, state: Map<Key, Value>): List<Refusal> {
        val t = chip.table
        val out = mutableListOf<Refusal>()
        val name = chip.id
        // Every pin that is stored.
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
            // A protection input the image does not read is `protect_chip`'s alone (below).
            if (!g.pins.all { (_, role) -> imageReads(t, role) }) continue
            val incomplete = groupRefusal(g, state)
            if (incomplete != null) out += incomplete
            if (incomplete != null || !complete(g, state)) continue
            val instance = g.instance?.let { (k, p) -> Triple(k, p, byteOf(state, k)) }
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
            } else {
                n = g.via?.let { byteOf(state, it) }
            }
            val remap = g.remap?.let { byteOf(state, it) } ?: 0
            for ((slot, role) in g.pins) {
                val b = pinOf(state, slot) ?: continue
                val (port, pin) = b[0] to b[1]
                val at = pinText(port, pin)
                if (role == Role.Adc || role == Role.DcLink) {
                    val want = t.adcChannel(port, pin)
                    if (want != b[2]) {
                        val msg = if (want != null) {
                            "$at is ADC channel $want, not ${b[2]}"
                        } else {
                            "$at is not an ADC input on the $name"
                        }
                        out += refusal(Check.ADC, slot.key, msg)
                    } else if (role == Role.DcLink && !t.feeds(port, pin, ADC2)) {
                        out += refusal(
                            Check.ADC,
                            slot.key,
                            "$at does not feed ADC2, whose inserted group converts the DC-link inputs",
                        )
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
        // Each motor's sensed pair is one the chip's image reads.
        for (m in 0 until Fields.MOTORS) {
            val sensed = SENSE_PHASES.filter { (def, _) -> def.key(m) in state }
            if (sensed.size != 2) continue
            val (first, second) = sensed[0].second to sensed[1].second
            if (!t.senses(first, second)) {
                val pairs = t.sensePairs.joinToString(" or ") { (a, b) -> "$a and $b" }
                out += refusal(
                    Check.PHASE_PAIR,
                    sensed[1].first.key(m),
                    "the $name image senses phases $pairs, not $first and $second",
                )
            }
        }
        // Motor 1 runs only beside motor 0, and on a two-motor chip each motor is on its own timer.
        // A motor is stored when its gate group is complete.
        val present = (0 until Fields.MOTORS).map { m ->
            val g = GROUPS.first { it.pins.first().first.key == Fields.MOTOR_GATE_HI_A.key(m) }
            g.pins.all { (slot, _) -> slot.key in state } && g.instance?.let { byteOf(state, it.first) } != null
        }
        if (present[1] && !present[0]) {
            out += refusal(
                Check.MOTOR_TIMER,
                Fields.MOTOR_GATE_HI_A.key(1),
                "motor 1 is stored without motor 0, whose TIMER0 triggers the ADC and starts motor 1's timer",
            )
        }
        if (MOTOR_TIMER.all { t.hasTimer(it) }) {
            for ((m, want) in MOTOR_TIMER.withIndex()) {
                val key = Fields.MOTOR_GATE_TIMER.key(m)
                val n = byteOf(state, key) ?: continue
                if (present[m] && n != want) {
                    out += refusal(
                        Check.MOTOR_TIMER,
                        key,
                        "motor $m's gates are on advanced timer $want on the $name, not $n",
                    )
                }
            }
        }
        // A protection input is stored only where the chip's image reads it
        // (`specs/optional-protections.md` section 5.2).
        for (m in 0 until Fields.MOTORS) {
            for ((def, role) in listOf(Fields.MOTOR_DC_LINK_PIN to Role.DcLink, Fields.MOTOR_BREAK_PIN to BREAK)) {
                val key = def.key(m)
                if (key in state && !imageReads(t, role)) {
                    out += refusal(Check.PROTECT_CHIP, key, "the $name image does not read this input")
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

    /**
     * Every check that needs no chip (`checks::check_values`): the range table, each motor's limit
     * above its noise floor, each protection's input, and the IMU frame.
     */
    fun checkValues(state: Map<Key, Value>): List<Refusal> {
        val out = mutableListOf<Refusal>()
        for (key in state.keys.sortedWith(compareBy({ it.fieldId }, { it.index }))) {
            // Every row but `wifi.tx_power`'s, the forceable one a tool's encoding makes
            // (`checks.rs`, `check_values`).
            if (key.fieldId == Fields.WIFI_TX_POWER.id) continue
            val range = Fields.rangeOf(key) ?: continue
            val v = intOf(state.getValue(key)) ?: continue
            if (v !in range) out += refusal(Check.RANGE, key, "$v is outside ${range.first}..${range.last}")
        }
        // Each motor's limit above its floor, each of the three as the write leaves it or its
        // registry default where it is not stored.
        for (m in 0 until Fields.MOTORS) {
            fun value(def: FieldDef): Long? = intOf(state[def.key(m)] ?: def.default)
            val limit = Fields.MOTOR_CURRENT_LIMIT.key(m)
            val ma = value(Fields.MOTOR_CURRENT_LIMIT) ?: continue
            val cal = value(Fields.MOTOR_CURRENT_CAL) ?: continue
            val floor = value(Fields.MOTOR_NOISE_FLOOR) ?: continue
            limitFloor(ma, cal, floor)?.let { out += refusal(Check.LIMIT_FLOOR, limit, it) }
        }
        // Each protection enabled only beside its input (`specs/optional-protections.md` section 7):
        // a setting as the write leaves it, 0 where it is not stored (each one's default).
        for (m in 0 until Fields.MOTORS) {
            for (p in PROTECTIONS) {
                val key = p.setting.key(m)
                val on = state[key]?.let { intOf(it) }?.let { it != 0L } ?: false
                if (on && !p.stored(state, m)) {
                    out += refusal(Check.PROTECT_INPUT, key, "enabled without its input, ${p.input(m)}")
                }
            }
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
     * Why a motor's limit of [ma] milliamps fails the `limit_floor` check at [cal] counts per amp
     * and a noise floor of [floor] counts, or null when it passes (`checks::limit_floor`): by the
     * firmware's arithmetic ([CurrentLimit]), a limit passes when FOC's full scale is above zero. The
     * lowest limit that passes is `floor * 1000 / cal + 1` rounded up to the next 100 mA, stepped up
     * by 100 mA while the conversion's rounding still leaves it at the floor.
     */
    fun limitFloor(ma: Long, cal: Long, floor: Long): String? {
        fun passes(ma: Long) = CurrentLimit.fullScaleCounts(CurrentLimit.limitCounts(ma, cal, floor), floor) > 0
        if (passes(ma)) return null
        val scale = cal.coerceIn(CurrentLimit.CURRENT_CAL)
        val floorCounts = CurrentLimit.noiseFloorCounts(floor)
        val start = (floorCounts * 1_000 / scale + 1 + 99) / 100 * 100
        val lowest = (start..CurrentLimit.LIMIT_REQUEST_CAP_MA step 100).firstOrNull { passes(it) }
        val fix = lowest?.let { "the lowest limit that passes is $it mA" }
            ?: "no limit passes a floor at the top of the limit window"
        return "$ma mA is ${CurrentLimit.requestCounts(ma, cal)} counts at a current.cal of $scale, at or below the " +
            "noise floor of $floorCounts counts, so FOC's full scale is 0; $fix"
    }

    /**
     * The keys whose write can change [checkLeader]'s verdict: `machine.mode`, `machine.frame` and
     * `control.leader` (`checks::LEADER_KEYS`).
     */
    val LEADER_KEYS: List<Key> = listOf(Fields.CONTROL_MODE.key(), Fields.CONTROL_FRAME.key(), Fields.CONTROL_LEADER.key())

    /** Whether writing [key] runs the leader check (`checks::changes_leader`). */
    fun changesLeader(key: Key): Boolean = key in LEADER_KEYS

    /**
     * The leader check (`checks::check_leader`, `specs/rigid-leader.md` section 6.1) over every
     * motor controller of the machine, each by its address and its stored values as the write would
     * leave them: when any of them stores `machine.mode` balance and `machine.frame` rigid, exactly
     * one stores `control.leader` set. Any other mode or frame passes, whatever the flags hold.
     */
    fun checkLeader(boards: List<Pair<Int, Map<Key, Value>>>): Refusal? {
        fun Map<Key, Value>.byte(key: Key): Int = byteOf(this, key) ?: 0
        val rigidBalance = boards.any { (_, s) ->
            s.byte(Fields.CONTROL_MODE.key()) == 1 && s.byte(Fields.CONTROL_FRAME.key()) == 1
        }
        if (!rigidBalance) return null
        val set = boards.filter { (_, s) -> s.byte(Fields.CONTROL_LEADER.key()) != 0 }.map { it.first }
        val why = when (set.size) {
            1 -> return null
            0 -> "none is set"
            else -> {
                val names = set.map { "0x%02x".format(it) }
                val verb = if (set.size == 2) "both" else "all"
                "${names.dropLast(1).joinToString(", ")} and ${names.last()} are $verb set"
            }
        }
        return refusal(
            Check.LEADER,
            Fields.CONTROL_LEADER.key(),
            "a balance machine in the rigid frame needs exactly one board set as leader; $why",
        )
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
