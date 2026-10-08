package com.hoverboard.remote.ble

import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Retransmit
import kotlinx.coroutines.delay

/**
 * What first contact settled: the guest address the board granted this app (its `src`), and the
 * address the board holds (the `dst` for everything the app sends it).
 */
data class Attachment(val guestAddr: Int, val boardAddr: Int)

/** How the L3 first contact ended. */
sealed interface AttachOutcome {
    /** The board holds an address and granted the app one: the session is drivable. */
    data class Attached(val attachment: Attachment) : AttachOutcome

    /** The outstanding request went unanswered through its whole retransmit budget. */
    data object Unanswered : AttachOutcome

    /** The overall attach deadline elapsed with first contact still incomplete. */
    data object Deadline : AttachOutcome
}

/** How the walk behind the attached board ended ([L3Session.discover]). */
sealed interface DiscoverOutcome {
    /** Every board the walk addressed or adopted, the attached one included, sorted ascending. */
    val boards: List<Int>

    /**
     * The slave to offer BOUND for (`specs/rider-ui.md` 3.2): the one board other than [master],
     * and only from a [Complete] walk. An [Abandoned] walk names none, even when [boards] holds one:
     * a slave adopted from the master's `PORTS` reply that never answered its own probe has not
     * been shown to be reachable, and BOUND would drive it blind.
     */
    fun slave(master: Int): Int? = when (this) {
        is Complete -> boards.filter { it != master }.singleOrNull()
        is Abandoned -> null
    }

    /** The walk finished: [boards] is the whole tree behind the attached board. */
    data class Complete(override val boards: List<Int>) : DiscoverOutcome

    /**
     * The walk was abandoned (a request went unanswered through its whole budget, or the deadline
     * passed): [boards] is what it had reached by then, which may still name the slave.
     */
    data class Abandoned(override val boards: List<Int>) : DiscoverOutcome
}

/**
 * Turns one session's [BleWalkEngine]. The engine is synchronous and I/O-free, so something has to
 * drive it: this owns the poll cadence, hands every packet the walk did not consume to [onPacket],
 * and writes every outgoing byte through [write]. It is the link's SINGLE writer.
 *
 * All Android and GATT types stay outside it ([write] is a plain suspending lambda, [nowMs] a plain
 * clock), so the attach loop that used to live inline in [BleHoverboardTransport] is exercised by
 * unit tests against a fake board rather than only on the bench.
 */
class L3Session(
    private val engine: BleWalkEngine,
    private val lock: Any,
    private val nowMs: () -> Long,
    private val onPacket: (ByteArray) -> Unit,
    private val deadlineMs: Long = ATTACH_DEADLINE_MS,
    private val pollIdleMs: Long = POLL_IDLE_MS,
    private val write: suspend (ByteArray) -> Unit,
) {

    /** One engine turn: pump, dispatch what came back, flush what is going out. */
    suspend fun turn() {
        synchronized(lock) { engine.pump() }
        while (true) {
            val packet = synchronized(lock) { engine.takeInbound() } ?: break
            onPacket(packet)
        }
        flush()
    }

    /** Write the engine's pending outgoing bytes. */
    suspend fun flush() {
        while (true) {
            val out = synchronized(lock) { engine.takeOutgoing() } ?: return
            write(out)
        }
    }

    /**
     * Run the L3 first contact to quiescence: `NODE_HELLO` out, adopt the granted `your_addr`, and
     * either assign the board an address (it reported `node_id = 0x00`) or adopt the one it already
     * reports. All of that is [BleWalkEngine]/`Controller`; this only supplies the timing.
     *
     * A dead link throws out of [write] instead of returning, which is a different outcome: the
     * caller reconnects for that and gives up for a returned failure.
     */
    suspend fun attach(): AttachOutcome {
        val deadline = nowMs() + deadlineMs
        while (nowMs() < deadline) {
            turn()
            engine.boardAddr?.let { board ->
                return AttachOutcome.Attached(Attachment(guestAddr = engine.guestAddr, boardAddr = board))
            }
            // l3.md's acknowledged control plane retransmits against an idempotent responder, and
            // the engine decides when its own outstanding request is overdue. A spent budget means
            // the board is not answering at all, not that a frame was lost.
            when (synchronized(lock) { engine.serviceRetransmit() }) {
                Retransmit.EXHAUSTED -> return AttachOutcome.Unanswered
                Retransmit.SENT -> flush()
                Retransmit.IDLE -> Unit
            }
            delay(pollIdleMs)
        }
        return AttachOutcome.Deadline
    }

    /**
     * Run the rest of the walk behind the attached board, to learn the slave's address
     * (`specs/rider-ui.md` section 2, discovery on attach). The engine must have been built to walk
     * (not attach-only) and [attach] must already have succeeded. The walk is the one
     * `BleWalkDriver.discover` runs, driven here on the session's own engine so that it shares the
     * session's link, guest address and single writer.
     *
     * A walk that does not finish is ABANDONED on the engine ([BleWalkEngine.abandonWalk]): the
     * session goes on to carry drive and config traffic on this link, and a late walk reply must not
     * resume the walk in the middle of it. A dead link throws out of [write], as in [attach].
     */
    suspend fun discover(): DiscoverOutcome {
        val deadline = nowMs() + DISCOVER_DEADLINE_MS
        while (nowMs() < deadline) {
            turn()
            synchronized(lock) {
                if (engine.walkComplete) return DiscoverOutcome.Complete(engine.addressedBoards())
            }
            when (synchronized(lock) { engine.serviceRetransmit() }) {
                Retransmit.EXHAUSTED -> return abandon()
                Retransmit.SENT -> flush()
                Retransmit.IDLE -> Unit
            }
            delay(pollIdleMs)
        }
        return abandon()
    }

    private fun abandon(): DiscoverOutcome = synchronized(lock) {
        engine.abandonWalk()
        DiscoverOutcome.Abandoned(engine.addressedBoards())
    }

    companion object {
        /**
         * Bound on [discover]. The board answers a `PROBE_PORTS` only after its probe window
         * (~500 ms), once per board walked, and each reply crosses the 9600-baud port; for the
         * master/slave pair that is two probes and their replies, so this is the backstop and the
         * retransmit budget the normal way a silent board is called.
         */
        const val DISCOVER_DEADLINE_MS = 8_000L

        /**
         * Overall bound on the L3 attach. Generous next to the retransmit budget, so the deadline is
         * the backstop and the budget is the normal way an unresponsive board is called: a board
         * answering slowly still attaches.
         */
        const val ATTACH_DEADLINE_MS = 12_000L

        /** Idle backoff between engine turns while waiting for the link to produce something. */
        const val POLL_IDLE_MS = 20L

        /**
         * How long the engine lets an unanswered request stand before re-sending it. It must clear
         * a real reply's round trip: the board meters its BLE port a byte at a time at 9600 baud,
         * and the phone's connection interval adds tens of ms on top, so a tighter timeout would
         * re-send over replies that were merely in flight.
         */
        const val REPLY_TIMEOUT_MS = 1_000L
    }
}
