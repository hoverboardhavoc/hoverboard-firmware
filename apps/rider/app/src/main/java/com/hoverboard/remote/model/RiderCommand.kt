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
 *   (`crates/state/src/mode.rs`, `ModeMachine::tick`). Nothing else on these boards can assert
 *   it: the physical power button is bridged out.
 * - `DRIVE_CMD` ([DriveCmd], opcode 0x11) carries the demand the control task turns into the
 *   throttle reference. **This is the word that moves a wheel.**
 *
 * `INPUTS` carries neither. It once led with a raw ADC mirror of a board's OWN throttle hardware,
 * a word no consumer read; that word is deleted and `INPUTS` is two level bytes
 * (`crates/linkctl/src/lib.rs`, `Inputs`), with the two words called "throttle" told apart in
 * `crates/swd-bridge/src/bin/drive.rs`, the module doc. The app streamed exactly that deleted
 * word and nothing else, so it could not have moved anything even with the board armed by other
 * means. Carrying a demand in a field no consumer reads is how that mistake happened, and a
 * second plausible-looking throttle word is worth more as an absence than as a duplicate.
 *
 * ## The invariant this type exists to hold
 *
 * A disarmed command **cannot** carry a demand, a turn, or a rider claim: the constructor is
 * private, [DISARMED] is the only disarmed value, and it is a constant. So there is no representable
 * state in which the app has let go of the arm control while still asking for motion, and disarming
 * can never be a two-frame sequence with a live demand in between: one tick carries both halves or
 * neither.
 */
data class RiderCommand private constructor(
    val armed: Boolean,
    val demand: Int,
    val rider: Boolean,
    val steer: Int = 0,
) {

    /**
     * The `INPUTS` mirror: the arm level in `buttons` bit0, and the rider level in `rider` bit0.
     *
     * ## Why `rider` is not the arm level
     *
     * It used to be: the app asserted rider-present whenever it was armed, on the argument that the
     * arm control was itself the rider deadman. That was defensible while nothing else could assert
     * rider, and it stops being defensible now that the foot pads do.
     *
     * The board folds three sources with OR: its own pads, this mirror, and the peer's cyclic flag
     * (`orchestrator::dispatch::rider_level`). The folded level picks the PID profile,
     * rider-present selecting the stiff profile A (`control::config::select_profile`). So an app
     * that asserts rider whenever it is armed pins the board to profile A and the pads can never
     * select anything else: the phone would be overriding a rider-presence decision it has no
     * sensor for. A remote reporting a rider it cannot see is not a truthful report.
     *
     * So [rider] is false by default and the app says nothing about rider presence, leaving the
     * pads to answer it. [simulatingRider] is the deliberate exception, for a bench board with no
     * pad hardware wired, and it is a thing the operator turns on rather than a side effect of
     * arming.
     */
    val inputs: Inputs
        get() = Inputs(
            buttons = if (armed) Inputs.BUTTON_POWER else 0,
            rider = if (rider) Inputs.RIDER_PRESENT else 0,
        )

    /**
     * The drive demand, and the turn beside it. Disarmed sends [DriveKind.Neutral], which the
     * firmware reads as a literal `(0, 0)` regardless of the words carried
     * (`crates/orchestrator/src/dispatch.rs`, `effective_drive`), so the neutral frame is inert
     * twice over: neutral kind AND a zero value.
     *
     * [steer] is zero on every command but a rigid leader's ([withTurn]). Which board may carry
     * one, and what it costs to carry one anywhere else, is [DriveFrame]'s rule.
     */
    val drive: DriveCmd
        get() = DriveCmd(
            kind = if (armed) DriveKind.Throttle else DriveKind.Neutral,
            value = demand,
            steer = steer,
        )

    /**
     * The same arm level and rider level, asking for [demand] instead, and turning nothing.
     *
     * This is how one rider intent becomes two per-wheel commands ([DriveFrame], DIFFERENTIAL): the
     * mix decides each wheel's value and the levels come through unchanged. A DISARMED command stays
     * [DISARMED] whatever is asked of it, so the invariant above survives the detour: a mix cannot
     * put a demand on a command that is not armed.
     *
     * The turn is dropped rather than carried, and that is the rule of 3.2 spelled in the type: a
     * per-wheel value IS the turn once the mix has spent it, so a command that carries one must not
     * also state one for the firmware to apply a second time.
     */
    fun withDemand(demand: Int): RiderCommand =
        if (armed) armed(demand, simulatingRider = rider) else DISARMED

    /**
     * The same command asking for a turn of [steer] as well, on the [DriveCmd.FULL_SCALE] scale.
     *
     * The rigid frame's leader is the one board that is sent one (`specs/rover-one-imu.md`
     * section 5): it holds the whole machine's common torque and splits the turn out of it
     * itself, so the app states the turn once and mixes nothing. A DISARMED command stays
     * [DISARMED]: a machine nobody is holding armed turns no more than it drives.
     */
    fun withTurn(steer: Int): RiderCommand =
        if (armed) copy(steer = steer.coerceIn(-DriveCmd.FULL_SCALE, DriveCmd.FULL_SCALE)) else DISARMED

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
    fun pdus(src: Int, dst: Int): List<ByteArray> = listOf(inputsPdu(src, dst), drivePdu(src, dst))

    /**
     * The arm level alone. Sent on change and as a slow keepalive rather than every tick, because
     * the firmware's mirror is kept alive by the demand stream and not by this frame:
     * see [com.hoverboard.remote.ble.LinkConfig.INPUTS_KEEPALIVE_TICKS].
     */
    fun inputsPdu(src: Int, dst: Int): ByteArray =
        Pdu(OP_INPUTS, src, dst, inputs.encode()).encode()

    /** The demand alone. This is the one that has to keep arriving, because it decays. */
    fun drivePdu(src: Int, dst: Int): ByteArray =
        Pdu(OP_DRIVE_CMD, src, dst, drive.encode()).encode()

    companion object {
        /**
         * The all-stop command: no power request, no rider, neutral drive, zero demand.
         *
         * This is what the pump holds before the first touch and what it falls back to on stop, so
         * every path that stops producing commands stops on this one rather than on whatever was
         * last asked for. The rider bit is false here whatever the developer toggle says: the
         * all-stop frame states nothing about anything.
         */
        val DISARMED = RiderCommand(armed = false, demand = 0, rider = false)

        /**
         * An armed command asking for [demand] on the [DriveCmd.FULL_SCALE] scale, clamped to it.
         *
         * The clamp is a wire-domain backstop, not the app's travel limit: [Throttle.MAX_SPEED]
         * decides how much of the scale the pad actually spans.
         *
         * @param simulatingRider assert `rider` bit0 as well. A BENCH affordance, off by default:
         *   see [inputs] for why a remote does not otherwise claim a rider is aboard. A board with
         *   no pad hardware wired reads no rider at all, so nothing on it will engage in balance
         *   mode (`control::fsm` requires rider present) and the profile never leaves B; this is
         *   how a bench operator stands in for the pads, and it is the same level
         *   `swd-mailbox-inputs --rider` asserts over SWD.
         */
        fun armed(demand: Int, simulatingRider: Boolean = false): RiderCommand =
            RiderCommand(
                armed = true,
                demand = demand.coerceIn(-DriveCmd.FULL_SCALE, DriveCmd.FULL_SCALE),
                rider = simulatingRider,
            )
    }
}
