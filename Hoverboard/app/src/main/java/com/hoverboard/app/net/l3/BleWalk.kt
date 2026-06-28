package com.hoverboard.app.net.l3

import com.hoverboard.app.net.l2.BleStreamTransport
import com.hoverboard.app.net.l2.Link
import com.hoverboard.app.net.store.Key
import com.hoverboard.app.net.store.Value
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.withTimeoutOrNull

/** The fleet a controller walk discovered: the app's own (guest) address + every addressed board. */
data class WalkOutcome(
    /** The app's own transient guest address (`0x80..0xFE`), granted by the entry board on `NODE_HELLO`. */
    val controllerAddr: Int,
    /** The entry board the app's BLE link attaches to (the one it routes through), or null if none. */
    val entryAddr: Int?,
    /** Every board addressed (or adopted) this walk, sorted ascending (the entry board sorts first). */
    val boards: List<Int>,
    /** A human-readable two-hop CONFIG echo (e.g. the slave's `node_address` read back), or null. */
    val configEcho: String? = null,
)

/**
 * The controller-side walk engine over one BLE byte-stream link. Owns the L3 [Controller], an L2
 * [Link], and the [BleStreamTransport] adapter; drives the walk and CONFIG round-trips purely by
 * feeding received notification bytes in ([onReceive]) and draining stream bytes out ([takeOutgoing]).
 *
 * Synchronous and I/O-free, so the same engine is driven by the host integration test (through a mock
 * byte-stream loopback to the firmware-mirrored mock boards) and by [BleWalkDriver] (through the real
 * CC2541 GATT pipe). The BLE bridge re-chunks freely; the [BleStreamTransport]'s length-delimited
 * framing tolerates it, so the engine never assumes one notification per frame.
 */
class BleWalkEngine(frameCapacity: Int = BleStreamTransport.DEFAULT_FRAME_CAPACITY) {

    /** The byte-stream adapter: feed it notification bytes, drain its outgoing stream bytes. */
    val transport = BleStreamTransport(frameCapacity)
    private val link = Link(transport)
    private val controller = Controller()
    private val configInbox = ArrayDeque<ByteArray>()

    /**
     * The last request sent and still awaiting its reply (for retransmit), and how many re-sends remain
     * for it. l3.md's acknowledged control plane (`NODE_HELLO`/`PROBE_PORTS`/`ASSIGN`/`CONFIG_*`)
     * retransmits on timeout against an idempotent responder; the BLE byte stream drops frames and the
     * GATT link can blip, so a request whose reply never arrives must be re-sent or the walk stalls.
     */
    private var pending: ByteArray? = null
    private var retxBudget = 0

    /** Send a request out the link and arm it for retransmit (overwriting any prior pending request). */
    private fun sendRequest(bytes: ByteArray) {
        link.send(bytes)
        pending = bytes
        retxBudget = MAX_RETRANSMITS
    }

    /** Feed raw bytes received from the notify char (0x1002). */
    fun onReceive(bytes: ByteArray) = transport.onReceive(bytes)

    /** Drain pending outgoing stream bytes to write to the write char (0x1001), or null if none. */
    fun takeOutgoing(): ByteArray? = transport.drainOutgoing()

    /** The discovery walk is finished: nothing queued and nothing outstanding. */
    val walkComplete: Boolean get() = controller.isComplete()

    /** The controller's (guest) address, adopted from the gateway's grant. */
    val guestAddr: Int get() = controller.guestAddr

    /** The board addresses handed out (or adopted) this walk, sorted ascending. */
    fun addressedBoards(): List<Int> = controller.assignedAddrs().sorted()

    /**
     * One processing pass: drain every reassembled inbound packet (answer a probe of our own port,
     * capture a `CONFIG_RESP`, else advance the walk), then emit the next due request. Returns true
     * if it sent or received anything, so a caller can loop to quiescence.
     */
    fun pump(): Boolean {
        var moved = false
        while (true) {
            val frame = link.pollRecv() ?: break
            moved = true
            val reply = controller.replyToProbe(frame)
            when {
                // A probe of our own port (the master probing us): answer it; not a reply to `pending`.
                reply != null -> link.send(reply)
                // A reply to our outstanding request: it is satisfied, so disarm retransmit.
                Pdu.decodeOrNull(frame)?.known() == Opcode.ConfigResp -> {
                    configInbox.addLast(frame)
                    pending = null
                }
                else -> {
                    controller.onReply(frame)
                    pending = null
                }
            }
        }
        controller.nextRequest()?.let {
            sendRequest(it)
            moved = true
        }
        return moved
    }

    /**
     * Re-send the outstanding (unacked) request, against an idempotent responder. The caller invokes
     * this only on a stall (no reply within the reply timeout). Returns false once the retransmit
     * budget for the current request is spent (a genuinely lost peer), so the caller can give up.
     */
    fun retransmitPending(): Boolean {
        val p = pending ?: return false
        if (retxBudget <= 0) return false
        retxBudget--
        link.send(p)
        return true
    }

    /** Send a `CONFIG_WRITE` to [dst] (routed by the board mesh); the reply arrives via [takeConfigResp]. */
    fun sendConfigWrite(dst: Int, key: Key, value: Value) =
        sendRequest(controller.buildConfigWrite(dst, key, value))

    /** Send a `CONFIG_READ` to [dst]; the reply arrives via [takeConfigResp]. */
    fun sendConfigRead(dst: Int, key: Key) =
        sendRequest(controller.buildConfigRead(dst, key))

    /** The next captured `CONFIG_RESP` PDU bytes, or null if none has arrived. */
    fun takeConfigResp(): ByteArray? = configInbox.removeFirstOrNull()

    private companion object {
        /** Re-sends allowed per request before giving up (covers a drop in each direction + margin). */
        const val MAX_RETRANSMITS = 4
    }
}

/**
 * A raw bidirectional BLE byte stream: [write] bytes to the module's write char (0x1001) and collect
 * its notifications (0x1002) from [incoming]. Frame boundaries are NOT preserved (the CC2541 bridge
 * coalesces and re-chunks); [BleStreamTransport] supplies the framing.
 */
interface BleBytePipe {
    /** Write a byte chunk to the GATT write char (the implementation may split to the ATT MTU). */
    suspend fun write(bytes: ByteArray)

    /** Notification bytes from the GATT notify char, in arrival order. */
    val incoming: Flow<ByteArray>
}

/**
 * A source of live BLE byte pipes to the module. Each [connect] (re)establishes a connection and
 * returns a fresh pipe, or null if it cannot within its own budget. The walk driver calls it again to
 * **reconnect** after a mid-walk GATT drop (the bench shows the CC2541 link dropping on a ~5 s
 * supervision timeout, recurringly).
 */
fun interface BlePipeSource {
    suspend fun connect(): BleBytePipe?
}

/**
 * Runs the controller-side walk (and a two-hop CONFIG read) over the BLE link, **surviving a mid-walk
 * GATT drop** by reconnecting and restarting. The protocol stepping is [BleWalkEngine] (host-tested);
 * this plumbs async BLE I/O - collect notifications into the engine, drain the engine's bytes to the
 * write char - and adds the connect/drop/reconnect loop:
 *
 * - When the notify flow ends (the `rxJob` completes) or a write throws, the link dropped: abort the
 *   attempt and reconnect.
 * - A restarted walk is **idempotent / re-walkable** (the responder adopts already-assigned boards),
 *   so re-running it on a fresh connection is safe and converges.
 * - Bounded by [MAX_ATTEMPTS] reconnects and an overall [OVERALL_DEADLINE_MS]; on exhaustion it
 *   returns an empty [WalkOutcome] (a clear "could not complete").
 */
class BleWalkDriver(private val pipes: BlePipeSource) {

    /** Walk the fleet, reconnecting and restarting across drops, until it completes or the budget runs out. */
    suspend fun discover(): WalkOutcome {
        val outcome = withTimeoutOrNull(OVERALL_DEADLINE_MS) {
            repeat(MAX_ATTEMPTS) {
                val pipe = pipes.connect()
                if (pipe == null) {
                    delay(RECONNECT_BACKOFF_MS)
                    return@repeat
                }
                when (val attempt = attemptWalk(pipe)) {
                    is Attempt.Done -> return@withTimeoutOrNull attempt.outcome
                    Attempt.Dropped -> delay(RECONNECT_BACKOFF_MS) // reconnect + restart (re-walk is idempotent)
                }
            }
            null
        }
        return outcome ?: WalkOutcome(controllerAddr = 0, entryAddr = null, boards = emptyList())
    }

    private sealed interface Attempt {
        data class Done(val outcome: WalkOutcome) : Attempt
        data object Dropped : Attempt
    }

    /** One full walk + two-hop CONFIG over a single connection; [Attempt.Dropped] if the link dies first. */
    private suspend fun attemptWalk(pipe: BleBytePipe): Attempt = coroutineScope {
        val engine = BleWalkEngine()
        engine.transport.resetRx()
        // The notify flow completing (rxJob done) is the drop signal; cancel it on the way out so this
        // `coroutineScope` is not held open by the otherwise-endless collector.
        val rxJob = pipe.incoming.onEach { engine.onReceive(it) }.launchIn(this)
        try {
            if (!pumpUntil(pipe, engine, rxJob) { engine.walkComplete }) {
                return@coroutineScope Attempt.Dropped
            }
            val boards = engine.addressedBoards()
            if (boards.isEmpty()) return@coroutineScope Attempt.Dropped

            // Two-hop path without mutating flash: read node_address back from the farthest board (for
            // the master/slave pair this routes through the entry board to the slave).
            val configEcho = boards.lastOrNull()?.let { dst ->
                engine.sendConfigRead(dst, NODE_ADDRESS_KEY)
                if (!flush(pipe, engine)) return@coroutineScope Attempt.Dropped
                val v = pumpForConfig(pipe, engine, rxJob) ?: return@coroutineScope Attempt.Dropped
                "node_address(0x${Integer.toHexString(dst)}) = 0x${Integer.toHexString(v)}"
            }

            Attempt.Done(
                WalkOutcome(
                    controllerAddr = engine.guestAddr,
                    entryAddr = boards.firstOrNull(),
                    boards = boards,
                    configEcho = configEcho,
                ),
            )
        } finally {
            rxJob.cancel()
        }
    }

    /**
     * Pump the engine until [done], draining its outgoing bytes to the pipe and retransmitting on a
     * reply timeout. Returns false if the link dropped (notify flow ended or a write threw) or the
     * per-attempt [WALK_TIMEOUT_MS] elapsed - in which case the caller reconnects.
     */
    private suspend fun pumpUntil(
        pipe: BleBytePipe,
        engine: BleWalkEngine,
        rxJob: Job,
        done: () -> Boolean,
    ): Boolean {
        var idlePolls = 0
        val ok = withTimeoutOrNull(WALK_TIMEOUT_MS) {
            while (!done()) {
                if (rxJob.isCompleted) return@withTimeoutOrNull false // notify flow ended -> dropped
                val moved = engine.pump()
                if (!flush(pipe, engine)) return@withTimeoutOrNull false
                if (moved) {
                    idlePolls = 0
                } else {
                    if (++idlePolls >= RETX_IDLE_POLLS) {
                        if (engine.retransmitPending() && !flush(pipe, engine)) {
                            return@withTimeoutOrNull false
                        }
                        idlePolls = 0
                    }
                    delay(POLL_IDLE_MS)
                }
            }
            true
        }
        return ok ?: false
    }

    /** Pump until a `CONFIG_RESP` is captured; the decoded `node_address`, or null if dropped/timed out. */
    private suspend fun pumpForConfig(pipe: BleBytePipe, engine: BleWalkEngine, rxJob: Job): Int? {
        var resp: ConfigResp? = null
        val done = pumpUntil(pipe, engine, rxJob) {
            engine.takeConfigResp()?.let { bytes ->
                Pdu.decodeOrNull(bytes)?.let { resp = ConfigResp.parse(it) }
            }
            resp != null
        }
        if (!done) return null
        return (resp?.decodeValue() as? Value.U8)?.v
    }

    /** Write the engine's pending outgoing bytes; false if the write fails (the link is gone). */
    @Suppress("TooGenericExceptionCaught", "SwallowedException")
    private suspend fun flush(pipe: BleBytePipe, engine: BleWalkEngine): Boolean {
        val out = engine.takeOutgoing() ?: return true
        return try {
            pipe.write(out)
            true
        } catch (e: Exception) {
            // Any write failure means the GATT link is gone; the caller treats false as a drop and
            // reconnects. The specific cause is immaterial (the link is dead either way).
            false
        }
    }

    private companion object {
        /** Bound one attempt's walk/CONFIG loop so a silent or wedged link surfaces (and reconnects). */
        const val WALK_TIMEOUT_MS = 8_000L

        /** Overall deadline across all reconnect attempts. */
        const val OVERALL_DEADLINE_MS = 30_000L

        /** Reconnect attempts before giving up. */
        const val MAX_ATTEMPTS = 5

        /** Backoff between a drop and the reconnect (lets the module re-advertise). */
        const val RECONNECT_BACKOFF_MS = 200L

        /** Idle backoff between polls while waiting for the next reply to arrive over the link. */
        const val POLL_IDLE_MS = 15L

        /** Idle polls (~`x POLL_IDLE_MS` reply timeout) with no progress before retransmitting. */
        const val RETX_IDLE_POLLS = 20
    }
}

/** The `node_address` field (field 0x01, singleton index 0), mirror of `store::NODE_ADDRESS`. */
val NODE_ADDRESS_KEY = Key(0x01, 0)
