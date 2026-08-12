package com.hoverboard.remote.model

import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.linkctl.DriveCmd
import com.hoverboard.protocol.linkctl.DriveKind
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_DRIVE_CMD
import com.hoverboard.protocol.linkctl.OP_INPUTS

/**
 * One tick of rider intent: whether the rider is holding the machine armed, and how much drive
 * they are asking for. This is the thing the app streams; [inputs] and [drive] are how it is
 * spelled on the wire, not two independent commands.
 *
 * ## Why both payloads, and why they are derived rather than stored
 *
 * The firmware needs two different words to move a wheel, and the app used to send neither of the
 * right ones:
 *
 * - `INPUTS.buttons` bit0 ([Inputs.BUTTON_POWER]) is `power_request`, a **level** the mode machine
 *   samples every 4 ms tick. Held, it walks `Off -> Init -> Ready -> Run` in three ticks and sets
 *   the motor output enables; dropped, `Run -> Shutdown -> Off` clears them
 *   (`crates/state/src/mode.rs:174-227`). Nothing else on these boards can assert it: the physical
 *   power button is bridged out.
 * - `DRIVE_CMD` ([DriveCmd], opcode 0x11) carries the demand the control task turns into the
 *   throttle reference. **This is the word that moves a wheel.**
 *
 * `INPUTS.throttle` is neither. It is the ADC mirror of a board's OWN throttle hardware, filtered
 * into `throttle_filtered`, which nothing reads (`crates/orchestrator/src/lib.rs:817`,
 * `crates/swd-bridge/src/bin/drive.rs:7-17`). The app streamed exactly that word and nothing else,
 * so it could not have moved anything even with the board armed by other means. It is left at zero
 * here deliberately: carrying a demand in a field no consumer reads is how that mistake happened,
 * and a second plausible-looking throttle word is worth more as an absence than as a duplicate.
 *
 * ## The invariant this type exists to hold
 *
 * A disarmed command **cannot** carry a demand: the constructor is private, [DISARMED] is the only
 * disarmed value, and it is a constant. So there is no representable state in which the app has let
 * go of the arm control while still asking for motion, and disarming can never be a two-frame
 * sequence with a live demand in between: one tick carries both halves or neither.
 */
data class RiderCommand private constructor(val armed: Boolean, val demand: Int) {

    /**
     * The `INPUTS` mirror: the arm level in `buttons` bit0, and the same level in `rider` bit0.
     *
     * Rider-present tracks the arm control rather than the throttle because the arm control IS the
     * rider deadman here: a hand is on it or the machine is disarmed. In throttle mode the level
     * only picks a control profile (`crates/orchestrator/src/dispatch.rs:352,367` note the pad gate
     * is balance-only), so this is a truthful report rather than a gate the app is steering.
     */
    val inputs: Inputs
        get() = Inputs(
            throttle = 0,
            buttons = if (armed) Inputs.BUTTON_POWER else 0,
            rider = if (armed) Inputs.RIDER_PRESENT else 0,
        )

    /**
     * The drive demand. Disarmed sends [DriveKind.Neutral], which the firmware reads as a literal
     * `(0, 0)` regardless of the words carried (`crates/orchestrator/src/dispatch.rs:170-175`), so
     * the neutral frame is inert twice over: neutral kind AND a zero value.
     */
    val drive: DriveCmd
        get() = DriveCmd(
            kind = if (armed) DriveKind.Throttle else DriveKind.Neutral,
            value = demand,
            steer = NO_STEER,
        )

    /**
     * This command spelled as L3 PDUs, in the order they go on the wire: the `INPUTS` mirror
     * carrying the arm level, then the `DRIVE_CMD` carrying the demand.
     *
     * `INPUTS` first because that is the order the levels take effect in on the board
     * (`power_request` walks the mode machine to `Run` over three ticks before any demand can be
     * enveloped), and because the reverse order would, on the disarming tick, put a drive frame on
     * the wire after the frame that dropped the power level.
     *
     * Building the frames here rather than inside the BLE transport is what makes the opcode
     * testable off-device. The defect this replaces was precisely an opcode choice: the app streamed
     * `INPUTS` alone, and no host test could see that the one word that moves a wheel was missing.
     */
    fun pdus(src: Int, dst: Int): List<ByteArray> = listOf(
        Pdu(OP_INPUTS, src, dst, inputs.encode()).encode(),
        Pdu(OP_DRIVE_CMD, src, dst, drive.encode()).encode(),
    )

    companion object {
        /**
         * The all-stop command: no power request, no rider, neutral drive, zero demand.
         *
         * This is what the pump holds before the first touch and what it falls back to on stop, so
         * every path that stops producing commands stops on this one rather than on whatever was
         * last asked for.
         */
        val DISARMED = RiderCommand(armed = false, demand = 0)

        /**
         * An armed command asking for [demand] on the [DriveCmd.FULL_SCALE] scale, clamped to it.
         *
         * The clamp is a wire-domain backstop, not the app's travel limit: [Throttle.MAX_SPEED]
         * decides how much of the scale the pad actually spans.
         */
        fun armed(demand: Int): RiderCommand =
            RiderCommand(armed = true, demand = demand.coerceIn(-DriveCmd.FULL_SCALE, DriveCmd.FULL_SCALE))

        /**
         * No steering from this app. A rider remote holds one throttle axis; differential steer is
         * a second axis nothing here produces, and sending a constant zero is the honest report of
         * that rather than a field left to whatever a previous frame carried.
         */
        private const val NO_STEER = 0
    }
}
