package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields
import kotlin.math.abs
import kotlin.math.roundToInt

/*
 * What the Ride screen's instruments show (`specs/remote-ui.md` 3.1), as pure functions of what the
 * boards sent and what they store, so every rule the screen draws by is tested off the screen.
 */

/** Which side of the screen a board slot is on. */
enum class SlotSide { LEFT, RIGHT }

/**
 * One board slot on the Ride screen: the board at [address], labelled by its side ("L", "R") when
 * [bySide], else by its address.
 */
data class RideSlot(val address: Int, val side: SlotSide, val bySide: Boolean)

/**
 * The Ride screen's two board slots (`specs/remote-ui.md` 3.1, "Board slots"): wherever a per-board
 * figure appears, at most two boards are drawn, one left and one right. An empty slot is null and
 * draws nothing.
 */
data class RideSlots(val left: RideSlot?, val right: RideSlot?) {
    /** The filled slots, left then right. */
    val all: List<RideSlot> get() = listOfNotNull(left, right)

    /** The slot the board at [address] is drawn in, or null when it is in neither. */
    fun of(address: Int): RideSlot? = all.firstOrNull { it.address == address }

    companion object {
        /** No motor controller: nothing drawn. */
        val NONE = RideSlots(null, null)
    }
}

/**
 * The slots for [motorControllers] (ascending addresses), each board's stored `control.side` from
 * [side] (null while unread; [Fields.ControlSide.RIGHT] is the right wheel and every other byte the
 * left, as the firmware reads it):
 *
 * - two boards whose sides read one left and one right: by side, labelled "L" and "R";
 * - one board: on the side its `control.side` names and labelled by it, or while that is unread
 *   the left slot, labelled with its address;
 * - anything else (a side unread, both sides the same, more than two boards): the two lowest
 *   addresses, left then right, labelled with their addresses.
 */
fun rideSlots(motorControllers: List<Int>, side: (Int) -> Int?): RideSlots {
    fun sideOf(byte: Int): SlotSide = if (byte == Fields.ControlSide.RIGHT) SlotSide.RIGHT else SlotSide.LEFT
    val mcs = motorControllers.sorted()
    val sides = mcs.map { side(it)?.let(::sideOf) }
    return when {
        mcs.isEmpty() -> RideSlots.NONE
        mcs.size == 1 -> {
            val slot = sides.single()?.let { RideSlot(mcs.single(), it, bySide = true) }
                ?: RideSlot(mcs.single(), SlotSide.LEFT, bySide = false)
            if (slot.side == SlotSide.LEFT) RideSlots(slot, null) else RideSlots(null, slot)
        }
        mcs.size == 2 && null !in sides && sides.toSet().size == 2 -> RideSlots(
            RideSlot(mcs[sides.indexOf(SlotSide.LEFT)], SlotSide.LEFT, bySide = true),
            RideSlot(mcs[sides.indexOf(SlotSide.RIGHT)], SlotSide.RIGHT, bySide = true),
        )
        else -> RideSlots(RideSlot(mcs[0], SlotSide.LEFT, false), RideSlot(mcs[1], SlotSide.RIGHT, false))
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
 * A phase bar's fill (`specs/remote-ui.md` 3.1): the figure's magnitude over the board's own
 * `motors.0.current.limit` [limitMa], clamped to [Fields.CURRENT_LIMIT_MA] as the firmware clamps it
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
