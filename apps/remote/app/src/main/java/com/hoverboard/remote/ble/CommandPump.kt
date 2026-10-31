package com.hoverboard.remote.ble

import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.remote.model.DriveFrame
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

    /**
     * The arm level only, for a board the frame addresses but does not drive
     * ([DriveFrame.driven]): the rigid frame's follower, which takes its wheel's number from the
     * leader's cyclic state rather than from this link.
     *
     * Such a board gets no tick at all on the ticks in between, so its mirror is held alive by the
     * `INPUTS` keepalive by itself. That is the relation
     * `RustSourceDriftTest.theArmMirrorWindowOutlastsTheAppsKeepalive` already pins: the firmware's
     * 1.5 s mirror window against the app's 500 ms keepalive, with the gate written as though the
     * keepalive were the only traffic. Here it is.
     */
    INPUTS_ONLY,
}

/** One board's share of one tick: the board's address [node], what it is told, and which payloads go out. */
data class NodeTick(val node: Int, val command: RiderCommand, val frames: TickFrames)

/**
 * Serialised, rate-limited sender for the rider's [RiderCommand] stream, to one board or two.
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
 * - **`DRIVE_CMD` decays fast.** The firmware stops honouring the drive reference once no fresh
 *   command has arrived for more than `linkctl::DRIVE_TIMEOUT_TICKS` (50 ticks at 250 Hz, so
 *   strictly 204 ms; the predicate is `orchestrator::LinkInbox::drive_stale`), and then ramps it
 *   down rather than snapping it to zero ([LinkConfig.SEND_INTERVAL_MS] has the arithmetic). The
 *   link is best-effort with no retransmit, so the cadence alone decides how many consecutive
 *   losses it takes to start that ramp. At the 10 Hz this pump first shipped with, that number was
 *   ONE, and every single dropped frame was a visible stutter. It goes out every tick, now at
 *   20 Hz ([LinkConfig.SEND_INTERVAL_MS]).
 * - **`INPUTS` decays slowly, and not on its own frames.** The mirror's CONTENT is latest-wins and
 *   written by `INPUTS` alone; its LIVENESS is refreshed by any payload from the mirror's owner
 *   that counts as operating the board, `INPUTS` or `DRIVE_CMD`
 *   (`orchestrator::LinkInbox::refreshes_mirror`), and expires at `linkctl::INPUTS_TIMEOUT_TICKS`
 *   (375 ticks, 1.5 s). So while the demand stream is flowing it is holding the arm level alive by
 *   itself and re-sending an unchanged level buys nothing; the board is already holding it.
 *   Streaming it every tick was pure cost on a metered 9600-baud module, and that cost came out of
 *   the demand's timing budget. It goes out on change and on a slow keepalive
 *   ([LinkConfig.INPUTS_KEEPALIVE_TICKS]). "Change" here means the `INPUTS` payload changed, arm
 *   level or rider level; see [start].
 *
 * The decay is the safety property, and a longer `INPUTS` cadence does not weaken it: kill the app,
 * drop the link, lose the phone, and the demand starts falling after 204 ms of silence and is gone
 * a ramp later (~133 ms from full), and the arm level itself is released 1.5 s in, without anything
 * having to notice. That ORDER is compile-time asserted in the firmware
 * (`INPUTS_TIMEOUT_TICKS > DRIVE_TIMEOUT_TICKS`), so the wheels are always already stopped by the
 * time the machine disarms. Decaying is the safe direction, so a lost frame is a stutter and never
 * a runaway.
 *
 * ## Two boards (`specs/remote-ui.md` 3.2)
 *
 * The held value is a [DriveFrame]: a command per board address, and the subset of them being DRIVEN. Each
 * DRIVEN board gets its own `DRIVE_CMD` every tick, and every addressed board gets its own `INPUTS`
 * bookkeeping, so an arm or disarm reaches each node with the change burst plus the keepalive, per
 * node. Both boards' payloads are staged by ONE [write] per tick, at the same cadence: a second
 * DRIVEN board doubles the bytes per tick (about 520 B/s of the CC2541's ~960 B/s for two boards)
 * and never raises the rate.
 *
 * A board that is addressed but not driven costs only its keepalive, about 20 B/s, and on the ticks
 * in between it produces no tick at all ([TickFrames.INPUTS_ONLY]). That is what the rigid frame's
 * follower is (`specs/rover-one-imu.md` section 7): a board holding an arm level the app asserts
 * and taking its wheel's number from its leader rather than from this link.
 *
 * A board that leaves the frame (a disarm, or a drive mode that addresses fewer boards) is not
 * simply dropped. If the last level it
 * was told is not the disarmed one, or its disarm burst has not finished, it keeps getting
 * [RiderCommand.DISARMED] until it has: a board that stops hearing from the app is otherwise released
 * only by its own 1.5 s mirror timeout.
 *
 * A failed individual write is swallowed and retried on the next tick, and a failed write is NOT
 * counted as having delivered the arm level: see [start]. [start]/[stop] bracket a connection.
 */
class CommandPump(
    private val scope: CoroutineScope,
    private val intervalMs: Long,
    private val write: suspend (List<NodeTick>) -> Unit,
) {
    private val pending = MutableStateFlow(DriveFrame.DISARMED)
    private var job: Job? = null

    /** Update the [DriveFrame] to be streamed. Cheap; safe to call at UI event rate. */
    fun set(frame: DriveFrame) {
        pending.value = frame
    }

    /** One board's `INPUTS` bookkeeping, for the life of one pump loop. */
    private class Book {
        // Null, not a disarmed payload: nothing has been delivered yet, so the first tick must
        // send the levels rather than assume the board already agrees with us.
        var delivered: Inputs? = null
        var repeatsLeft = 0

        // Ticks since the last INPUTS send, INCLUDING the one about to be decided. Counted at the
        // top rather than after a drive-only send, so the keepalive falls on the Nth tick after the
        // last INPUTS rather than the N+1th: counting after the decision made
        // [LinkConfig.INPUTS_KEEPALIVE_TICKS] mean one tick more than it says.
        var ticksSinceInputs = 0

        /** A board outside the frame is owed nothing once it holds the disarmed level, burst done. */
        val settledDisarmed: Boolean
            get() = delivered == null || (delivered == RiderCommand.DISARMED.inputs && repeatsLeft == 0)

        /**
         * What this board is owed on the tick about to go out, or null when it is owed nothing and
         * should not appear in the write at all. ADVANCES the tick counter, so it is called exactly
         * once per board per tick.
         *
         * [driven] is whether the board gets a `DRIVE_CMD` this tick, which is the frame's answer
         * ([DriveFrame.driven]) and also true of a board being let go, whose disarm rides the
         * ordinary pair of payloads.
         */
        fun owing(command: RiderCommand, driven: Boolean): TickFrames? {
            if (delivered != command.inputs) repeatsLeft = LinkConfig.INPUTS_CHANGE_REPEATS
            ticksSinceInputs++
            val withInputs = delivered != command.inputs ||
                repeatsLeft > 0 ||
                ticksSinceInputs >= LinkConfig.INPUTS_KEEPALIVE_TICKS
            return when {
                driven && withInputs -> TickFrames.BOTH
                driven -> TickFrames.DRIVE_ONLY
                withInputs -> TickFrames.INPUTS_ONLY
                else -> null
            }
        }
    }

    /**
     * Begin streaming at [intervalMs]. Idempotent per connection.
     *
     * "Changed" is the whole `INPUTS` payload, not the arm level alone, and that is deliberate: this
     * loop's job is to re-send `INPUTS` when its CONTENT differs from what the board was last told,
     * so the thing it compares has to be the content it delivers. Comparing only `armed` left the
     * rider bit riding the 500 ms keepalive, so a bench toggle took up to half a second and missed
     * the repeat burst that protects every other level change on a link with no retransmit. The
     * throttle word is a constant 0, so widening the comparison adds no traffic on a demand change.
     *
     * The bookkeeping is deliberately only advanced after a write that did NOT throw. A level
     * nothing else will correct for a second and a half is the one frame that must not be quietly
     * dropped: treating a failed write as delivered could leave a board armed after a disarm the app
     * believes it sent.
     */
    fun start() {
        if (job?.isActive == true) return
        job = scope.launch {
            val books = mutableMapOf<Int, Book>()
            while (isActive) {
                val frame = pending.value
                // Every node in the frame, plus every node that left it still owed its disarm.
                val owed = books.filter { (node, book) -> node !in frame.commands && !book.settledDisarmed }
                    .mapValues { RiderCommand.DISARMED }
                val targets = (frame.commands + owed).toSortedMap()
                books.keys.retainAll(targets.keys)

                // A board the frame addresses but does not drive is owed no DRIVE_CMD, so on a
                // tick with no INPUTS to send it is owed nothing and appears in no write.
                val ticks = targets.mapNotNull { (node, command) ->
                    val driven = node in frame.driven || node in owed
                    books.getOrPut(node) { Book() }.owing(command, driven)
                        ?.let { NodeTick(node, command, it) }
                }

                try {
                    write(ticks)
                    for (t in ticks) {
                        if (t.frames == TickFrames.DRIVE_ONLY) continue
                        val book = books.getValue(t.node)
                        book.delivered = t.command.inputs
                        // Only a write that did NOT throw restarts the interval; a failed keepalive
                        // is still owed and goes out on the next tick.
                        book.ticksSinceInputs = 0
                        if (book.repeatsLeft > 0) book.repeatsLeft--
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
     * Stop streaming and reset the held value to [DriveFrame.DISARMED].
     *
     * The reset matters on reconnect, not on stop: a new session starts a new pump loop against
     * this held value, and it must not resume an arm level the rider is no longer being asked to
     * confirm. Stopping does NOT itself disarm the board; going quiet only gets there by timing
     * out, 1.5 s later. See [BleHoverboardTransport.disconnect].
     */
    fun stop() {
        job?.cancel()
        job = null
        pending.value = DriveFrame.DISARMED
    }
}
