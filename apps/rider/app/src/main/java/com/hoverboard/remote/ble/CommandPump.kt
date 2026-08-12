package com.hoverboard.remote.ble

import com.hoverboard.remote.model.RiderCommand
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

/**
 * Serialised, rate-limited sender for the rider's [RiderCommand] stream (the app streams at 10 Hz).
 *
 * Why this exists: BLE allows only one outstanding GATT operation at a time. Launching a
 * coroutine per touch-move event (a finger fires ~90/s) produced overlapping concurrent
 * writes to the same characteristic, which the BLE stack rejects. This pump instead holds the
 * *latest* [RiderCommand] and writes it on a single coroutine at a fixed cadence:
 *  - exactly one write in flight at a time (no concurrency), and
 *  - the held value is re-sent every tick, which is not an optimisation but the thing that keeps a
 *    demand alive at all.
 *
 * ## Why the re-send is load-bearing
 *
 * `DRIVE_CMD` decays. The firmware zeroes the drive reference once no fresh command has arrived for
 * `DRIVE_TIMEOUT_TICKS` (50 ticks at 250 Hz = 200 ms, `crates/linkctl/src/lib.rs:57`,
 * `crates/orchestrator/src/lib.rs:255-260`), so a single send is a 200 ms blip rather than a demand.
 * That decay is the safety property: it means an app that stops, a phone that dies, and a link that
 * drops all stop the wheels within 200 ms without anything having to notice.
 *
 * The cadence has to sit inside that window with room for a lost frame, because the link is
 * best-effort with no retransmit. At [LinkConfig.SEND_INTERVAL_MS] = 100 ms, one lost
 * frame still leaves the next one arriving at 200 ms; the decay fires strictly after 50 ticks
 * (`drive_age > DRIVE_TIMEOUT_TICKS`), so the margin is thin but real, and two consecutive losses
 * do decay the reference. Decaying is the safe direction: the reference ramps to zero through the
 * control conditioning and the machine stays armed, so a recovered frame picks the demand straight
 * back up. The failure of a lost frame is a stutter, never a runaway.
 *
 * A failed individual write is swallowed (the next tick retries). [start]/[stop] bracket a
 * connection's lifetime.
 */
class CommandPump(
    private val scope: CoroutineScope,
    private val intervalMs: Long,
    private val write: suspend (RiderCommand) -> Unit,
) {
    private val pending = MutableStateFlow(RiderCommand.DISARMED)
    private var job: Job? = null

    /** Update the [RiderCommand] to be streamed. Cheap; safe to call at UI event rate. */
    fun set(command: RiderCommand) {
        pending.value = command
    }

    /** Begin streaming the latest [RiderCommand] at [intervalMs]. Idempotent per connection. */
    fun start() {
        if (job?.isActive == true) return
        job = scope.launch {
            while (isActive) {
                try {
                    write(pending.value)
                } catch (e: CancellationException) {
                    throw e
                } catch (_: Exception) {
                    // Transient write failure (link blip, permission revoked, op in flight):
                    // drop this frame and retry on the next tick.
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
     * `INPUTS` level it was sent with no staleness at all; see [BleHoverboardTransport.disconnect].
     */
    fun stop() {
        job?.cancel()
        job = null
        pending.value = RiderCommand.DISARMED
    }
}
