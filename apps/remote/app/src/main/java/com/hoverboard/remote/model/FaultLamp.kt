package com.hoverboard.remote.model

import androidx.annotation.StringRes
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.R

/**
 * One lamp of the Ride screen's fault grid (`specs/remote-ui.md` 3.1b): one per cause that stops a
 * board arming, which is one bit of [CyclicState.ARM_BLOCKING_BITS], in the grid's order.
 *
 * A board's own `CYCLIC_STATE.fault` word blocks its arm when it carries any of these bits: the
 * mode machine holds a board in OFF while Fault A or Fault B is asserted, and Fault B is motor 1's
 * latch alone. Bit 6 (the mode fault, a readback) and bit 7 (the power request itself) are in the
 * word and are not lamps; bit 3 and bits 12 to 15 name nothing.
 *
 * @param bit the lamp's bit mask in the fault word.
 * @param label the name the lamp carries in the grid.
 * @param title the sheet's title, and the lamp's name in a statement.
 * @param whenShows the sheet's "When it shows" paragraph, verbatim from what the firmware does.
 * @param blocks the sheet's "What it blocks" paragraph.
 * @param fix the sheet's "Fix" paragraph.
 */
enum class FaultLamp(
    val bit: Int,
    val label: String,
    @StringRes val title: Int,
    @StringRes val whenShows: Int,
    @StringRes val blocks: Int,
    @StringRes val fix: Int,
) {
    LINK(
        CyclicState.EV_COMMS_LOSS,
        "LINK",
        R.string.lamp_link_title, R.string.lamp_link_when, R.string.lamp_link_blocks, R.string.lamp_link_fix,
    ),
    TILT(
        CyclicState.EV_STOP_ALL,
        "TILT",
        R.string.lamp_tilt_title, R.string.lamp_tilt_when, R.string.lamp_tilt_blocks, R.string.lamp_tilt_fix,
    ),
    IMU(
        CyclicState.EV_IMU_LOSS,
        "IMU",
        R.string.lamp_imu_title, R.string.lamp_imu_when, R.string.lamp_imu_blocks, R.string.lamp_imu_fix,
    ),
    LATCH(
        CyclicState.EV_LATCH_A,
        "LATCH",
        R.string.lamp_latch_title, R.string.lamp_latch_when, R.string.lamp_latch_blocks, R.string.lamp_latch_fix,
    ),
    LATCH_2(
        CyclicState.EV_LATCH_B,
        "LATCH 2",
        R.string.lamp_latch2_title, R.string.lamp_latch2_when, R.string.lamp_latch2_blocks, R.string.lamp_latch2_fix,
    ),
    HALL(
        CyclicState.EV_MOTOR_HALL,
        "HALL",
        R.string.lamp_hall_title, R.string.lamp_hall_when, R.string.lamp_hall_blocks, R.string.lamp_hall_fix,
    ),
    CAL(
        CyclicState.EV_MOTOR_CAL,
        "CAL",
        R.string.lamp_cal_title, R.string.lamp_cal_when, R.string.lamp_cal_blocks, R.string.lamp_cal_fix,
    ),
    PERIOD(
        CyclicState.EV_MOTOR_PERIOD,
        "PERIOD",
        R.string.lamp_period_title, R.string.lamp_period_when, R.string.lamp_period_blocks, R.string.lamp_period_fix,
    ),
    ARM(
        CyclicState.EV_MOTOR_ARM,
        "ARM",
        R.string.lamp_arm_title, R.string.lamp_arm_when, R.string.lamp_arm_blocks, R.string.lamp_arm_fix,
    ),
    ;

    /** The lamp's code on the sheet: "F" and the bit's number, "F8" for HALL. */
    val code: String get() = "F${Integer.numberOfTrailingZeros(bit)}"

    /** Whether the app holds this lamp lit after its bit clears, until the next ARM tap. */
    val held: Boolean get() = bit and LATCH_BITS != 0

    companion object {
        /** The two latch bits the app holds lit ([held]): the board clears a latch about 8 ms after it trips. */
        const val LATCH_BITS = CyclicState.EV_LATCH_A or CyclicState.EV_LATCH_B

        /** The lamp for one bit mask of the fault word, or null for a bit that is not a lamp. */
        fun of(bit: Int): FaultLamp? = entries.firstOrNull { it.bit == bit }

        /** The lamps whose bits are set in [word], in the grid's order. */
        fun lit(word: Int): List<FaultLamp> = entries.filter { word and it.bit != 0 }
    }
}

/**
 * What the app knows about one motor controller that decides which lamps apply on it: its
 * address and its stored `imu.model` (0x60), null while unread.
 */
data class LampFacts(val address: Int, val imuModel: Int?) {
    /** Whether the board has an IMU fitted. Unread counts as fitted, so an unread fact never hides a lamp. */
    val imuFitted: Boolean get() = imuModel != 0
}

/**
 * The lamps the grid shows (`specs/remote-ui.md` 3.1b), in the grid's order: each lamp that applies
 * on at least one motor controller in [facts], and each whose bit is set in any reporting board's
 * word in [words]. A set bit is never hidden, whatever the rule says. Nothing else decides which
 * lamps are drawn.
 *
 * - LINK applies when the walk found two motor controllers: comms loss asserts only once a peer
 *   has been heard, so a one-board machine cannot light it.
 * - TILT and IMU apply on a board whose IMU is fitted ([LampFacts.imuFitted]): the cutout and the
 *   loss level run only on a board whose IMU answered at power-up, in Throttle as in Balance.
 * - LATCH 2 never applies: no board drives a second motor, so it shows only while its bit is set.
 * - LATCH and the four motor causes apply on every motor controller: the motor bring-up they gate
 *   on is not visible over the link.
 *
 * @param facts every motor controller the walk found, one entry each; their count is the motor
 *   controller count LINK reads.
 * @param words each reporting board's fault word by address, with any bit the app holds lit.
 */
fun shownLamps(facts: List<LampFacts>, words: Map<Int, Int>): List<FaultLamp> {
    val set = words.values.fold(0) { acc, w -> acc or w }
    return FaultLamp.entries.filter { lamp ->
        set and lamp.bit != 0 || applies(lamp, facts)
    }
}

private fun applies(lamp: FaultLamp, facts: List<LampFacts>): Boolean = when (lamp) {
    FaultLamp.LINK -> facts.size == 2
    FaultLamp.TILT, FaultLamp.IMU -> facts.any { it.imuFitted }
    FaultLamp.LATCH_2 -> false
    FaultLamp.LATCH, FaultLamp.HALL, FaultLamp.CAL, FaultLamp.PERIOD, FaultLamp.ARM -> facts.isNotEmpty()
}

/**
 * The grid's column count for [shown] lamps: four, or five when nine are shown, so the grid is
 * never more than two rows (the fleet's fullest machine applies eight; the ninth is LATCH 2 lit).
 */
fun lampColumns(shown: Int): Int = if (shown > FOUR_COLUMN_MAX) FIVE_COLUMNS else FOUR_COLUMNS

private const val FOUR_COLUMNS = 4
private const val FIVE_COLUMNS = 5
private const val FOUR_COLUMN_MAX = 8

/**
 * Why the app dropped the arm level by itself for a board fault (`specs/remote-ui.md` 3.1): the
 * first [lamp] that lit on the board at [address], one the arm had armed. Kept until the rider's
 * next ARM tap, so a lamp that goes out a frame later still leaves a statement saying why the
 * machine stopped.
 */
data class FaultDisarm(val lamp: FaultLamp, val address: Int)
