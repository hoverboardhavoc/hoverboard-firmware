package com.hoverboard.remote.ble

import com.hoverboard.remote.model.RiderCommand
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

/** Which of a [RiderCommand]'s two payloads one tick puts on the wire. */
enum class TickFrames {
    /** The demand only. The steady state: the arm level is already latched on the board. */
    DRIVE_ONLY,

    /** Demand and arm level. Sent when the level changed, and on the slow keepalive. */
    BOTH,
}

/**
 * Serialised, rate-limited sender for the rider's [RiderCommand] stream.
 *
 * Why this exists: BLE allows only one outstanding GATT operation at a time. Launching a coroutine
 * per touch-move event (a finger fires ~90/s) produced overlapping concurrent writes to the same
 * characteristic, which the BLE stack rejects. This pump instead holds the *latest* command and
 * writes it on a single coroutine at a fixed cadence, so there is exactly one write in flight.
 *
 * ## The two payloads are on different schedules, because they have different failure modes
 *
 * This is the fix for a motor that ran slow and jittery with drop-outs on the bench, and the two
 * halves of it pull in opposite directions:
 *
 * - **`DRIVE_CMD` decays.** The firmware zeroes the drive reference once no fresh command has
 *   arrived for `DRIVE_TIMEOUT_TICKS` (50 ticks at 250 Hz = 200 ms, `crates/linkctl/src/lib.rs:57`,
 *   `crates/orchestrator/src/lib.rs:255-260`). The link is best-effort with no retransmit, so the
 *   cadence alone decides how many consecutive losses it takes to zero the demand. At the 10 Hz
 *   this pump first shipped with, that number was ONE, and every single dropped frame was a visible
 *   stutter. It goes out every tick, now at 20 Hz ([LinkConfig.SEND_INTERVAL_MS]).
 * - **`INPUTS` does not decay.** The firmware stores the remote mirror latest-wins with no age at
 *   all (`crates/orchestrator/src/lib.rs:223`), so re-sending an unchanged arm level buys nothing;
 *   the board is already holding it. Streaming it every tick was pure cost on a metered 9600-baud
 *   module, and that cost came out of the demand's timing budget. It goes out on change and on a
 *   slow keepalive ([LinkConfig.INPUTS_KEEPALIVE_TICKS]).
 *
 * The decay is still the safety property, and a longer cadence does not weaken it: kill the app,
 * drop the link, lose the phone, and the demand is gone within 200 ms without anything having to
 * notice. Decaying is the safe direction, so a lost frame is a stutter and never a runaway.
 *
 * A failed individual write is swallowed and retried on the next tick, and a failed write is NOT
 * counted as having delivered the arm level: see [start]. [start]/[stop] bracket a connection.
 */
class CommandPump(
    private val scope: CoroutineScope,
    private val intervalMs: Long,
    private val write: suspend (RiderCommand, TickFrames) -> Unit,
) {
    private val pending = MutableStateFlow(RiderCommand.DISARMED)
    private var job: Job? = null

    /** Update the [RiderCommand] to be streamed. Cheap; safe to call at UI event rate. */
    fun set(command: RiderCommand) {
        pending.value = command
    }

    /**
     * Begin streaming at [intervalMs]. Idempotent per connection.
     *
     * The arm-level bookkeeping is deliberately only advanced after a write that did NOT throw. A
     * level that latches forever is the one frame that must not be quietly dropped: treating a
     * failed write as delivered could leave a board armed after a disarm the app believes it sent.
     */
    fun start() {
        if (job?.isActive == true) return
        job = scope.launch {
            // Null, not false: nothing has been delivered yet, so the first tick must send the
            // level rather than assume the board already agrees with us.
            var deliveredArmed: Boolean? = null
            var repeatsLeft = 0
            // Ticks since the last INPUTS send, INCLUDING the one about to be decided. Counted at
            // the top rather than after a drive-only send, so the keepalive falls on the Nth tick
            // after the last INPUTS rather than the N+1th: counting after the decision made
            // [LinkConfig.INPUTS_KEEPALIVE_TICKS] mean one tick more than it says.
            var ticksSinceInputs = 0

            while (isActive) {
                val command = pending.value
                val changed = deliveredArmed != command.armed
                if (changed) repeatsLeft = LinkConfig.INPUTS_CHANGE_REPEATS
                ticksSinceInputs++

                val withInputs = changed ||
                    repeatsLeft > 0 ||
                    ticksSinceInputs >= LinkConfig.INPUTS_KEEPALIVE_TICKS
                val frames = if (withInputs) TickFrames.BOTH else TickFrames.DRIVE_ONLY

                try {
                    write(command, frames)
                    if (withInputs) {
                        deliveredArmed = command.armed
                        // Only a write that did NOT throw restarts the interval; a failed keepalive
                        // is still owed and goes out on the next tick.
                        ticksSinceInputs = 0
                        if (repeatsLeft > 0) repeatsLeft--
                    }
                } catch (e: CancellationException) {
                    throw e
                } catch (_: Exception) {
                    // Transient write failure (link blip, permission revoked, op in flight). Drop
                    // this frame and retry; the bookkeeping above is untouched, so a level that did
                    // not make it is still owed.
                }
                delay(intervalMs)
            }
        }
    }

    /**
     * Stop streaming and reset the held value to [RiderCommand.DISARMED].
     *
     * The reset matters on reconnect, not on stop: a new session starts a new pump loop against
     * this held value, and it must not resume an arm level the rider is no longer being asked to
     * confirm. Stopping does NOT itself disarm the board, because the firmware holds the last
     * `INPUTS` level it was sent with no staleness; see [BleHoverboardTransport.disconnect].
     */
    fun stop() {
        job?.cancel()
        job = null
        pending.value = RiderCommand.DISARMED
    }
}
