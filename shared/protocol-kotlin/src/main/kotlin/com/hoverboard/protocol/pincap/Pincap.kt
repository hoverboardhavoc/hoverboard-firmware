package com.hoverboard.protocol.pincap

import com.hoverboard.protocol.linkctl.ChipTag

/**
 * What each pin of each chip the HAL drives can do: a mirror of `motorcontroller/hal/src/pincap.rs`
 * (`specs/config-read-once.md` section 4), the table the configuration checks run against. Every
 * table, route, ADC input, sense pair and protection input here is pinned to the Rust by
 * `RustSourceDriftTest`, both ways.
 *
 * On a chip with a per-pin mux (GD32F1x0, MM32SPIN05) a route's selector is the AF number a `PinAf`
 * stores; on the GD32F10x it is the AFIO remap value the group's `remap` key stores, and the stored
 * `af` is 0.
 */

/** A peripheral function's kind (`pincap::Func`'s variants). */
enum class FuncKind { GATE, GATE_N, TX, RX, SCL, SDA, BREAK }

/**
 * A peripheral function a pin can carry: [kind] on instance [n] (the HAL's label number), and for
 * a gate the timer channel [ch].
 */
data class Func(val kind: FuncKind, val n: Int, val ch: Int = 0) {
    override fun toString(): String = when (kind) {
        FuncKind.GATE -> "Gate($n, $ch)"
        FuncKind.GATE_N -> "GateN($n, $ch)"
        FuncKind.TX -> "Tx($n)"
        FuncKind.RX -> "Rx($n)"
        FuncKind.SCL -> "Scl($n)"
        FuncKind.SDA -> "Sda($n)"
        FuncKind.BREAK -> "Break($n)"
    }
}

/** One alternate-function route: pin [pin] of port [port] carries [func] when routed by [sel]. */
data class Route(val port: Int, val pin: Int, val func: Func, val sel: Int)

/**
 * One ADC input: pin [pin] of port [port] is channel [channel] of each converter in [adcs], a bit
 * each: [ADC0], [ADC1], [ADC2] (`specs/optional-protections.md` section 5.2).
 */
data class AdcInput(val port: Int, val pin: Int, val channel: Int, val adcs: Int)

/** [AdcInput.adcs]'s bit for ADC0 (the MM32SPIN05's one converter, the HAL's ADC0). */
const val ADC0 = 1 shl 0

/** [AdcInput.adcs]'s bit for ADC1. */
const val ADC1 = 1 shl 1

/** [AdcInput.adcs]'s bit for ADC2. */
const val ADC2 = 1 shl 2

/**
 * The inputs a chip's image reads beyond the motor and board groups every image reads
 * (`pincap::Inputs`, `specs/optional-protections.md` section 5.2): the protection inputs,
 * `current.dc_link.pin` ([dcLink]) and `gate.break.pin` ([brk]).
 */
data class Inputs(val dcLink: Boolean, val brk: Boolean)

/** A motor phase, as a phase-current sense pin names it (`pincap::Phase`). */
enum class Phase { A, B, C }

/** The SWD pins, PA13 and PA14, as (port, pin): never given to a function. */
val SWD: List<Pair<Int, Int>> = listOf(0 to 13, 0 to 14)

/**
 * One chip's pins (`pincap::Table`): the bonded pins per port `A` to `F` (bit `n` = pin `n`), every
 * route the HAL drives, every ADC input, and the phase pairs a motor's current sense may store
 * there, the pairs the chip's image reads (`specs/twelve-fet.md` section 5.3), each in phase order,
 * and the protection inputs the chip's image reads.
 */
class PinTable(
    val bonding: List<Int>,
    val routes: List<Route>,
    val adc: List<AdcInput>,
    val sensePairs: List<Pair<Phase, Phase>>,
    val inputs: Inputs,
) {
    /** Whether pin [pin] of port [port] is bonded out. */
    fun bonded(port: Int, pin: Int): Boolean =
        pin in 0..15 && port >= 0 && (bonding.getOrNull(port) ?: 0) and (1 shl pin) != 0

    /** The selector that routes [func] onto the pin, or null when the pin cannot carry it. */
    fun sel(port: Int, pin: Int, func: Func): Int? =
        routes.firstOrNull { it.port == port && it.pin == pin && it.func == func }?.sel

    /** The ADC channel on the pin, or null. */
    fun adcChannel(port: Int, pin: Int): Int? {
        if (!bonded(port, pin)) return null
        return adc.firstOrNull { it.port == port && it.pin == pin }?.channel
    }

    /** Whether the pin is an ADC input that feeds every converter in [adcs] ([ADC0], [ADC1], [ADC2]). */
    fun feeds(port: Int, pin: Int, adcs: Int): Boolean =
        bonded(port, pin) && adc.any { it.port == port && it.pin == pin && it.adcs and adcs == adcs }

    /** Whether the chip carries advanced timer [n]: a table routes its outputs exactly when it does. */
    fun hasTimer(n: Int): Boolean = routes.any { it.func == Func(FuncKind.GATE, n, 0) }

    /** Whether the chip carries USART [n]. */
    fun hasUsart(n: Int): Boolean = routes.any { it.func == Func(FuncKind.TX, n) }

    /** Whether the chip carries I2C [n]. */
    fun hasI2c(n: Int): Boolean = routes.any { it.func == Func(FuncKind.SCL, n) }

    /** Whether a motor's current sense may store the pair [first], [second] (in phase order). */
    fun senses(first: Phase, second: Phase): Boolean = (first to second) in sensePairs
}

private fun route(port: Int, pin: Int, func: Func, sel: Int) = Route(port, pin, func, sel)
private fun adc(port: Int, pin: Int, channel: Int, adcs: Int) = AdcInput(port, pin, channel, adcs)
private fun gate(n: Int, ch: Int) = Func(FuncKind.GATE, n, ch)
private fun gateN(n: Int, ch: Int) = Func(FuncKind.GATE_N, n, ch)
private fun tx(n: Int) = Func(FuncKind.TX, n)
private fun rx(n: Int) = Func(FuncKind.RX, n)
private fun scl(n: Int) = Func(FuncKind.SCL, n)
private fun sda(n: Int) = Func(FuncKind.SDA, n)
private fun brk(n: Int) = Func(FuncKind.BREAK, n)

/** `F10X_ROUTES`: the routes every F10x part carries (remap values). */
private val F10X_ROUTES = listOf(
    route(0, 8, gate(0, 0), 0),
    route(0, 9, gate(0, 1), 0),
    route(0, 10, gate(0, 2), 0),
    route(1, 13, gateN(0, 0), 0),
    route(1, 14, gateN(0, 1), 0),
    route(1, 15, gateN(0, 2), 0),
    route(1, 6, tx(0), 1),
    route(1, 7, rx(0), 1),
    route(0, 2, tx(1), 0),
    route(0, 3, rx(1), 0),
    route(1, 10, tx(2), 0),
    route(1, 11, rx(2), 0),
    route(1, 6, scl(0), 0),
    route(1, 7, sda(0), 0),
    route(1, 10, scl(1), 0),
    route(1, 11, sda(1), 0),
)

/**
 * `F10X_HD_ROUTES`: the F10x routes, the GD32F103RC's TIMER7, and the two break inputs, TIMER0's
 * on PB12 at remap 0 and TIMER7's on PA6.
 */
private val F10X_HD_ROUTES = F10X_ROUTES + listOf(
    route(2, 6, gate(7, 0), 0),
    route(2, 7, gate(7, 1), 0),
    route(2, 8, gate(7, 2), 0),
    route(0, 7, gateN(7, 0), 0),
    route(1, 0, gateN(7, 1), 0),
    route(1, 1, gateN(7, 2), 0),
    route(1, 12, brk(0), 0),
    route(0, 6, brk(7), 0),
)

/**
 * `gd32_adc`: PA0-7 are channels 0-7, PB0-1 are 8-9, PC0-5 are 10-15, PA0-3 and PC0-3 feeding the
 * converters in [withAdc2] and the others those in [rest].
 */
private fun gd32Adc(withAdc2: Int, rest: Int): List<AdcInput> =
    (0..7).map { adc(0, it, it, if (it < 4) withAdc2 else rest) } +
        listOf(adc(1, 0, 8, rest), adc(1, 1, 9, rest)) +
        (0..5).map { adc(2, it, 10 + it, if (it < 4) withAdc2 else rest) }

/** The protection inputs of a one-motor image: none. */
private val NO_INPUTS = Inputs(dcLink = false, brk = false)

/** The pair an image with one advanced timer reads: phases A and B, injected ranks 0 and 1. */
private val PAIR_AB = listOf(Phase.A to Phase.B)

private val GD32F103C8_TABLE = PinTable(
    listOf(0xFFFF, 0xFFFF, 0xE000, 0x0003, 0, 0),
    F10X_ROUTES,
    gd32Adc(ADC0 or ADC1, ADC0 or ADC1),
    PAIR_AB,
    NO_INPUTS,
)

private val GD32F103RC_TABLE = PinTable(
    listOf(0xFFFF, 0xFFFF, 0xFFFF, 0x0007, 0, 0),
    F10X_HD_ROUTES,
    gd32Adc(ADC0 or ADC1 or ADC2, ADC0 or ADC1),
    PAIR_AB + (Phase.B to Phase.C),
    Inputs(dcLink = true, brk = true),
)

private val GD32F130C8_TABLE = PinTable(
    listOf(0xFFFF, 0xFFFF, 0xE000, 0, 0, 0x00C3),
    listOf(
        route(0, 8, gate(0, 0), 2),
        route(0, 9, gate(0, 1), 2),
        route(0, 10, gate(0, 2), 2),
        route(1, 13, gateN(0, 0), 2),
        route(1, 14, gateN(0, 1), 2),
        route(1, 15, gateN(0, 2), 2),
        route(1, 6, tx(0), 0),
        route(1, 7, rx(0), 0),
        route(0, 2, tx(1), 1),
        route(0, 3, rx(1), 1),
        route(1, 6, scl(0), 1),
        route(1, 7, sda(0), 1),
        route(1, 10, scl(1), 1),
        route(1, 11, sda(1), 1),
    ),
    gd32Adc(ADC0, ADC0),
    PAIR_AB,
    NO_INPUTS,
)

private val MM32SPIN05_TABLE = PinTable(
    listOf(0xFFFF, 0xFFFF, 0xE000, 0x000F, 0, 0),
    listOf(
        route(0, 8, gate(0, 0), 2),
        route(0, 9, gate(0, 1), 2),
        route(0, 10, gate(0, 2), 2),
        route(1, 13, gateN(0, 0), 2),
        route(1, 14, gateN(0, 1), 2),
        route(1, 15, gateN(0, 2), 2),
        route(1, 6, tx(0), 0),
        route(1, 3, tx(0), 3),
        route(1, 7, rx(0), 0),
        route(1, 4, rx(0), 3),
        route(0, 2, tx(1), 1),
        route(1, 7, tx(1), 4),
        route(0, 3, rx(1), 1),
        route(0, 6, rx(1), 3),
        route(0, 15, rx(1), 1),
        route(1, 8, rx(1), 4),
        route(1, 6, scl(0), 1),
        route(1, 8, scl(0), 1),
        route(1, 10, scl(0), 1),
        route(3, 1, scl(0), 1),
        route(1, 7, sda(0), 1),
        route(1, 9, sda(0), 1),
        route(1, 11, sda(0), 1),
        route(3, 0, sda(0), 1),
    ),
    (0..7).map { adc(0, it, it, ADC0) } +
        listOf(adc(1, 0, 8, ADC0), adc(1, 1, 9, ADC0), adc(1, 3, 10, ADC0), adc(1, 4, 11, ADC0)),
    PAIR_AB,
    NO_INPUTS,
)

/**
 * A chip the HAL builds an image for (`pincap::Chip`), by the name its HAL feature and the check
 * cases spell it ([id]), and whether it routes a peripheral by the AFIO remap ([remaps], the
 * GD32F10x) rather than a per-pin mux.
 */
enum class Chip(val id: String, val remaps: Boolean) {
    /** LQFP48: the rover master. */
    GD32F103C8("gd32f103c8", true),

    /** LQFP64: the 12-FET. */
    GD32F103RC("gd32f103rc", true),

    /** LQFP48: the offroad pair and the rover slave. */
    GD32F130C8("gd32f130c8", false),

    /** LQFP48: the Gen2.4.2 split boards. */
    MM32SPIN05("mm32spin05", false),
    ;

    /** The chip's table. */
    val table: PinTable
        get() = when (this) {
            GD32F103C8 -> GD32F103C8_TABLE
            GD32F103RC -> GD32F103RC_TABLE
            GD32F130C8 -> GD32F130C8_TABLE
            MM32SPIN05 -> MM32SPIN05_TABLE
        }

    companion object {
        /** The chip [id] names, or null. */
        fun fromId(id: String): Chip? = entries.firstOrNull { it.id == id }

        /**
         * The chip a board names in its `CYCLIC_STATE` (`shared/linkctl/src/lib.rs`, `ChipTag`), or
         * null for [ChipTag.Unknown]: a part with no table, on which no pin can be checked.
         */
        fun forTag(tag: ChipTag): Chip? = when (tag) {
            ChipTag.F103C8 -> GD32F103C8
            ChipTag.F130C8 -> GD32F130C8
            ChipTag.F103RC -> GD32F103RC
            ChipTag.Unknown -> null
        }
    }
}
