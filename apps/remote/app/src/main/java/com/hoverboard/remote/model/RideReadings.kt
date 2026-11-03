package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields
import kotlin.math.abs
import kotlin.math.roundToInt

/*
 * What the Ride screen's instruments show (`specs/remote-ui.md` 3.1), as pure functions of what the
 * boards sent and what they store, so every rule the screen draws by is tested off the screen.
 */

/** Which side of the screen a wheel slot is on. */
enum class SlotSide { LEFT, RIGHT }

/**
 * One wheel slot on the Ride screen (`specs/remote-ui.md` 3.1, "Wheel slots"): the wheel [wheel],
 * labelled by its side ("L", "R") when [bySide], else by its board's address and, on a board with
 * two motors ([twoMotor]), the motor ("0x02:1").
 *
 * Its wheel figures (the phase bar, the speed's share) are [wheel]'s motor's; its board figures
 * (the link dot, the attitude corner, the lamp marks) are its board's, [address], so both slots of
 * a 12-FET show the same board's.
 */
data class RideSlot(val wheel: Wheel, val side: SlotSide, val bySide: Boolean, val twoMotor: Boolean = false) {
    /** The slot's board: whose board figures it shows. */
    val address: Int get() = wheel.node
}

/**
 * The Ride screen's two wheel slots (`specs/remote-ui.md` 3.1, "Wheel slots"): at most two wheels
 * are drawn, one left and one right. An empty slot is null and draws nothing.
 */
data class RideSlots(val left: RideSlot?, val right: RideSlot?) {
    /** The filled slots, left then right. */
    val all: List<RideSlot> get() = listOfNotNull(left, right)

    /** The first slot the board at [address] is drawn in, or null when it is in neither. */
    fun of(address: Int): RideSlot? = all.firstOrNull { it.address == address }

    /** Whether the board at [address] fills both slots, as one 12-FET does. */
    fun both(address: Int): Boolean = all.size == 2 && all.all { it.address == address }

    companion object {
        /** No motor controller: nothing drawn. */
        val NONE = RideSlots(null, null)
    }
}

/**
 * The slots for [wheels] (any order), each wheel's stored `side` at its motor index from [side]
 * (null while unread; [Fields.ControlSide.RIGHT] is the right wheel and every other byte the left,
 * as the firmware reads it):
 *
 * - two wheels whose sides read one left and one right: by side, labelled "L" and "R";
 * - one wheel: on the side it names and labelled by it, or while that is unread the left slot,
 *   labelled with its board's address;
 * - anything else (a side unread, both sides the same, more than two wheels): the first two wheels
 *   by address then motor index, left then right, labelled with the board's address and, on a board
 *   with two motors, the motor.
 *
 * A machine of two one-motor boards draws as it did when the slots were boards.
 */
fun rideSlots(wheels: List<Wheel>, side: (Wheel) -> Int?): RideSlots {
    fun sideOf(byte: Int): SlotSide = if (byte == Fields.ControlSide.RIGHT) SlotSide.RIGHT else SlotSide.LEFT
    val ws = wheels.sorted()
    val sides = ws.map { side(it)?.let(::sideOf) }
    fun twoMotor(w: Wheel): Boolean = ws.count { it.node == w.node } > 1
    fun slot(w: Wheel, at: SlotSide, bySide: Boolean) = RideSlot(w, at, bySide, twoMotor(w))
    return when {
        ws.isEmpty() -> RideSlots.NONE
        ws.size == 1 -> {
            val one = sides.single()?.let { slot(ws.single(), it, bySide = true) }
                ?: slot(ws.single(), SlotSide.LEFT, bySide = false)
            if (one.side == SlotSide.LEFT) RideSlots(one, null) else RideSlots(null, one)
        }
        ws.size == 2 && null !in sides && sides.toSet().size == 2 -> RideSlots(
            slot(ws[sides.indexOf(SlotSide.LEFT)], SlotSide.LEFT, bySide = true),
            slot(ws[sides.indexOf(SlotSide.RIGHT)], SlotSide.RIGHT, bySide = true),
        )
        else -> RideSlots(slot(ws[0], SlotSide.LEFT, false), slot(ws[1], SlotSide.RIGHT, false))
    }
}

/** The battery ring's segment count. */
const val RING_SEGMENTS = 10

/** How many of the ring's ten segments [percent] lights: `ceil(percent / 10)`, so 0 lights none. */
fun litSegments(percent: Int): Int {
    val p = percent.coerceIn(0, PERCENT_FULL)
    return (p + PERCENT_PER_SEGMENT - 1) / PERCENT_PER_SEGMENT
}

/**
 * The board whose reading the ring shows, and that reading (`specs/remote-ui.md` 3.1, "Whose
 * reading"). The machine has one pack and every motor controller reports it, so the ring takes the
 * reporting board whose reading is a [BatteryReading.Charge] with the lowest percent, the lower
 * address on a tie; with no Charge, the lowest-addressed reporting board's reading. Null address and
 * [BatteryReading.NoReading] while no board reports.
 *
 * @param reporting the motor controllers the phone hears, any order.
 */
fun ringReading(reporting: List<Int>, batteryOf: (Int) -> BatteryReading): Pair<Int?, BatteryReading> {
    val boards = reporting.sorted()
    val charged = boards.mapNotNull { a -> (batteryOf(a) as? BatteryReading.Charge)?.let { a to it } }
    charged.minWithOrNull(compareBy<Pair<Int, BatteryReading.Charge>> { it.second.percent }.thenBy { it.first })
        ?.let { return it }
    val first = boards.firstOrNull() ?: return null to BatteryReading.NoReading
    return first to batteryOf(first)
}

/**
 * Whether the ring and the pack line read caution: the reading's own [BatteryReading.Charge.low]
 * (the bottom 15 percent of state of charge), or its pack voltage at or below the same board's
 * `board.battery.floor` [floorCentivolts] while that floor is above 0. Below the floor the board
 * refuses a balance engage, so a floor set above 15 percent turns the gauge to caution where that
 * refusal starts. A floor that is 0 or unread adds nothing.
 */
fun batteryLow(reading: BatteryReading, floorCentivolts: Int?): Boolean {
    val volts = when (reading) {
        BatteryReading.NoReading -> return false
        is BatteryReading.Unjudged -> reading.volts
        is BatteryReading.Charge -> if (reading.low) return true else reading.volts
    }
    val floor = floorCentivolts?.takeIf { it > 0 } ?: return false
    return (volts * CENTIVOLTS_PER_VOLT).roundToInt() <= floor
}

/**
 * A phase bar's fill (`specs/remote-ui.md` 3.1): the figure's magnitude over the wheel's own
 * `motors.<motor>.current.limit` [limitMa], clamped to [Fields.CURRENT_LIMIT_MA] as the firmware clamps it
 * at boot, so a full bar is that board at its limit. Clamped to full. Null (no fill) without a
 * figure or while the limit is unread.
 */
fun phaseFill(reading: CurrentReading, limitMa: Int?): Float? {
    val amps = (reading as? CurrentReading.Amps)?.motor ?: return null
    val limit = limitMa?.toLong()?.coerceIn(Fields.CURRENT_LIMIT_MA) ?: return null
    return (abs(amps) * MA_PER_A / limit).coerceIn(0f, 1f)
}

/**
 * An attitude figure in [centidegrees] to 0.1 degree, rounded half away from zero, as "2.3"; a
 * value that rounds to zero prints "0.0", never "-0.0".
 */
fun angleText(centidegrees: Int): String {
    val tenths = (abs(centidegrees) + CENTI_PER_TENTH / 2) / CENTI_PER_TENTH
    val sign = if (centidegrees < 0 && tenths != 0) "-" else ""
    return "$sign${tenths / TENTHS}.${tenths % TENTHS}"
}

/** A distance in [millimetres] as the strip prints the total: 0.1 km below 10 km, whole km at and above. */
fun totalKmText(millimetres: Long): String {
    val tenths = tenthsOfKm(millimetres)
    if (tenths >= WHOLE_FROM_TENTHS) return "${(tenths + TENTHS / 2) / TENTHS}"
    return "${tenths / TENTHS}.${tenths % TENTHS}"
}

/** A distance in [millimetres] to 0.1 km, as the strip prints the trip. */
fun tripKmText(millimetres: Long): String {
    val tenths = tenthsOfKm(millimetres)
    return "${tenths / TENTHS}.${tenths % TENTHS}"
}

private fun tenthsOfKm(millimetres: Long): Long = (millimetres.coerceAtLeast(0) + MM_PER_TENTH_KM / 2) / MM_PER_TENTH_KM

/**
 * The trip's baseline (`specs/remote-ui.md` 3.1, "Trip"): the first machine distance computed after
 * a connect, held for the board picked off the Connect screen ([pick], its address, or null for a
 * board reached by name). It survives a reconnect to the same pick and is dropped when the pick
 * changes; when the total ever reads below it (a board rebooted and lost travel its store had not
 * recorded yet), it moves down to the total.
 */
data class TripBaseline(val pick: String?, val millimetres: Long?) {

    /** The baseline after a machine [total] is computed with [pick] in force. */
    fun next(pick: String?, total: MachineTravel?): TripBaseline {
        val kept = if (pick == this.pick) millimetres else null
        val mm = (total as? MachineTravel.Distance)?.millimetres ?: return TripBaseline(pick, kept)
        return TripBaseline(pick, if (kept == null) mm else minOf(kept, mm))
    }

    /** The trip at [total], or null when there is no machine distance or no baseline yet. */
    fun trip(total: MachineTravel?): Long? {
        val mm = (total as? MachineTravel.Distance)?.millimetres ?: return null
        return mm - (millimetres ?: return null)
    }

    companion object {
        /** Nothing picked and nothing computed: what the process starts with. */
        val NONE = TripBaseline(pick = null, millimetres = null)
    }
}

private const val PERCENT_FULL = 100
private const val PERCENT_PER_SEGMENT = 10
private const val CENTIVOLTS_PER_VOLT = 100f
private const val MA_PER_A = 1000f
private const val CENTI_PER_TENTH = 10
private const val TENTHS = 10
private const val WHOLE_FROM_TENTHS = 100L
private const val MM_PER_TENTH_KM = 100_000L

/**
 * The ring's and the speed's scale in an instruments slot [hDp] dp tall (`specs/remote-ui.md` 3.1):
 * (h - 64) / 212, at most 1, where 64 dp is what the slot needs beside the ring (the phase row, and
 * the pack line under it) and 212 dp the ring's share at full size.
 */
fun ringScale(hDp: Float): Float = ((hDp - SCALE_FIXED_DP) / SCALE_SPAN_DP).coerceAtMost(1f)

private const val SCALE_FIXED_DP = 64f
private const val SCALE_SPAN_DP = 212f
