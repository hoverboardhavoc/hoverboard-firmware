package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.DriveCmd
import kotlin.math.abs

/**
 * What one stick deflection asks of each wheel, in the `DRIVE_CMD` domain and already inside it.
 *
 * [difference] is the quantity that steers: two wheels told the same thing go straight whatever the
 * value, and the yaw comes from how far apart the two demands are. [DriveMix] is the thing that
 * keeps it.
 */
data class WheelPair(val left: Int, val right: Int) {
    /** How far apart the two wheels are told to run: the commanded turn. */
    val difference: Int get() = left - right
}

/**
 * The per-wheel mix for DIFFERENTIAL drive: `left = v + s/2`, `right = v - s/2`, clamped to the wire
 * range (`specs/rider-ui.md` 3.2). The mix happens here, in the app, and each board is then told its
 * own wheel's demand with `steer = 0`; the firmware's own steer mixer stays inert.
 *
 * ## What the clamp does when a wheel saturates
 *
 * Both wheels are moved by the SAME offset until the pair fits, so their difference survives
 * untouched and what gives is the travel they share. At `v` = full forward and `s` = full right the
 * stick asks for `(1.5, 0.5)` of full scale; this returns `(1.0, 0.0)`, still exactly `s/2 * 2`
 * apart. The machine turns as hard as it was asked to and runs slower than it was asked to.
 *
 * Two other readings of "clamp" were available and both lose the turn:
 *
 * - Clamping each wheel on its own is the one that bites hardest. The pair no longer differs by `s`
 *   once either wheel is on its rail, so the machine straightens out exactly when it is asked to
 *   turn hardest, which is the surprise a rider cannot plan around.
 * - Multiplying both demands by a common factor shrinks the difference along with everything else
 *   (a factor of 2/3 on the pair above leaves them 2/3 of `s` apart), so it has the same fault in a
 *   gentler form. An offset is the only adjustment that brings a pair inside a range and leaves the
 *   distance between the two values alone, which is why this is an offset.
 *
 * Giving up travel rather than turn is the right trade in that order: a machine that turns as asked
 * and travels slower is doing something a rider can see and ride, and a machine that stops turning
 * at full deflection is not.
 *
 * ## The one case where the difference cannot hold
 *
 * A turn wider than the whole range, `|s/2|` past [DriveCmd.FULL_SCALE], cannot be delivered by any
 * pair of demands: the two wheels at opposite rails are already as far apart as the wire can say.
 * There the wheels GO to those opposite rails, in the direction asked, and the forward demand is
 * what is lost: the turn is the hardest one the machine has. The stick cannot reach this on its own
 * (both axes span one full scale, so `|s/2|` is at most half of it), so it is the backstop on the
 * arithmetic rather than a case the rider rides into.
 */
object DriveMix {

    /**
     * The pair for a forward demand [forward] and a turn [steer], both on the [limit] scale.
     * Positive [steer] runs the left wheel faster, which turns the machine to the right.
     *
     * @param limit the magnitude each wheel is held inside. The wire's own full scale, the same
     *   backstop [RiderCommand.armed] applies to a demand, rather than the pad's travel.
     */
    fun of(forward: Int, steer: Int, limit: Int = DriveCmd.FULL_SCALE): WheelPair {
        val half = steer / 2
        // Wider than the range itself: the rails are as far apart as two demands get.
        if (abs(half) > limit) return WheelPair(left = sign(half) * limit, right = -sign(half) * limit)

        val left = forward + half
        val right = forward - half
        // One offset for both wheels, so the difference comes through untouched. Only one of the
        // two rails can be past: the pair is at most 2 * limit wide by the guard above.
        val offset = when {
            maxOf(left, right) > limit -> limit - maxOf(left, right)
            minOf(left, right) < -limit -> -limit - minOf(left, right)
            else -> 0
        }
        return WheelPair(left = left + offset, right = right + offset)
    }

    private fun sign(value: Int): Int = if (value < 0) -1 else 1
}
