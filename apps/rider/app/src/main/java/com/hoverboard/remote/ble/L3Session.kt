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

    companion object {
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
