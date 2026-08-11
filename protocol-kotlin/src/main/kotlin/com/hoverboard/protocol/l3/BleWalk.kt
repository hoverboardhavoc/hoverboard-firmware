package com.hoverboard.protocol.l3

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
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
 * The default clock for request ages and attach deadlines: MONOTONIC, not wall-clock.
 *
 * Only differences are ever taken from it, so its arbitrary origin does not matter, but its
 * monotonicity does: `System.currentTimeMillis` can step (an NTP correction, the user setting the
 * clock) mid-attach, which would either fire a retransmit immediately or stall one past its
 * timeout. `System.nanoTime` is the JVM's monotonic source and this module is plain JVM.
 *
 * Android callers should inject `SystemClock::elapsedRealtime` instead, which is monotonic AND
 * keeps counting while the device is in deep sleep.
 */
fun monotonicNowMs(): Long = System.nanoTime() / 1_000_000L

/** What [BleWalkEngine.serviceRetransmit] did with the outstanding request. */
enum class Retransmit {
    /** Nothing is outstanding, or the outstanding request has not been waiting long enough yet. */
    IDLE,

    /** The outstanding request was re-sent; its bytes are waiting in [BleWalkEngine.takeOutgoing]. */
    SENT,

    /** The request went unanswered through its whole budget: the peer is not answering at all. */
    EXHAUSTED,
}

/**
 * The controller-side walk engine over one BLE byte-stream link. Owns the L3 [Controller], an L2
 * [Link], and the [BleStreamTransport] adapter; drives the walk and CONFIG round-trips purely by
 * feeding received notification bytes in ([onReceive]) and draining stream bytes out ([takeOutgoing]).
 *
 * Synchronous and I/O-free, so the same engine is driven by the host integration test (through a mock
 * byte-stream loopback to the firmware-mirrored mock boards) and by [BleWalkDriver] (through the real
 * CC2541 GATT pipe). The BLE bridge re-chunks freely; the [BleStreamTransport]'s length-delimited
 * framing tolerates it, so the engine never assumes one notification per frame.
 *
 * ## The retransmit timer lives here, not in the driver
 *
 * A driver polls; it does not decide WHEN a request is overdue. That decision needs the one fact
 * only this engine holds - which request is outstanding and since when - and every driver that
 * re-derived it from its own "did anything happen" signal got it wrong the same way: an addressed
 * board streams CYCLIC_STATE at 5 Hz, so "something arrived" is true several times a second while
 * the reply the walk is actually waiting for is long lost. So [serviceRetransmit] is the ONLY way
 * to re-send, it times off the outstanding request's own age, and it is reset by that request being
 * satisfied or replaced - never by unrelated traffic. A driver's whole duty is to call it each poll
 * and act on the answer.
 */
class BleWalkEngine(
    frameCapacity: Int = BleStreamTransport.DEFAULT_FRAME_CAPACITY,
    /**
     * Stop driving the walk once first contact has addressed the board on this link, instead of
     * descending into `PROBE_PORTS` and the rest of the tree.
     *
     * This is a property of the DRIVER, not of [Controller]: how far to walk is the caller's
     * policy, so the controller stays a straight mirror of `walk.rs`. A rider remote on a
     * point-to-point BLE link to one board needs the attach leg and nothing else; a fleet
     * controller leaves this false and walks the tree.
     */
    private val attachOnly: Boolean = false,
    /**
     * How long an unanswered request waits before [serviceRetransmit] re-sends it. It must clear a
     * real reply's round trip: the board meters its BLE port a byte at a time at 9600 baud and the
     * phone's connection interval adds tens of ms on top, so a tighter timeout re-sends over
     * replies that were merely in flight.
     */
    private val replyTimeoutMs: Long = DEFAULT_REPLY_TIMEOUT_MS,
    /** The clock the request's age is measured against (injected so tests can drive it). */
    private val nowMs: () -> Long = ::monotonicNowMs,
) {

    /** The byte-stream adapter: feed it notification bytes, drain its outgoing stream bytes. */
    val transport = BleStreamTransport(frameCapacity)
    private val link = Link(transport)
    private val controller = Controller()
    private val configInbox = ArrayDeque<ByteArray>()
    private val inbound = ArrayDeque<ByteArray>()

    /**
     * The last request sent and still awaiting its reply (for retransmit), and how many re-sends remain
     * for it. l3.md's acknowledged control plane (`NODE_HELLO`/`PROBE_PORTS`/`ASSIGN`/`CONFIG_*`)
     * retransmits on timeout against an idempotent responder; the BLE byte stream drops frames and the
     * GATT link can blip, so a request whose reply never arrives must be re-sent or the walk stalls.
     */
    private var pending: ByteArray? = null
    private var retxBudget = 0

    /** What [pending] is, so a reply can be matched to the CLASS of request that is outstanding. */
    private var pendingOp: Opcode? = null

    /** When the outstanding request was last put on the wire: the age [serviceRetransmit] times off. */
    private var pendingSentAtMs = 0L

    /** Send a request out the link and arm it for retransmit (overwriting any prior pending request). */
    private fun sendRequest(bytes: ByteArray) {
        link.send(bytes)
        pending = bytes
        pendingOp = Pdu.decodeOrNull(bytes)?.known()
        retxBudget = MAX_RETRANSMITS
        pendingSentAtMs = nowMs()
    }

    /** The outstanding request is answered (or abandoned): disarm its retransmit. */
    private fun clearPending() {
        pending = null
        pendingOp = null
        retxBudget = 0
    }

    /** Feed raw bytes received from the notify char (0x1002). */
    fun onReceive(bytes: ByteArray) = transport.onReceive(bytes)

    /** Drain pending outgoing stream bytes to write to the write char (0x1001), or null if none. */
    fun takeOutgoing(): ByteArray? = transport.drainOutgoing()

    /** The discovery walk is finished: nothing queued and nothing outstanding. */
    val walkComplete: Boolean get() = controller.isComplete()

    /**
     * First contact is done: the board on this link holds an address, and [guestAddr] is the one it
     * granted. This is the completion condition for an [attachOnly] engine ([walkComplete] never
     * becomes true there, because the tree walk it would need is deliberately not driven).
     */
    val attached: Boolean get() = controller.gatewayAddr() != null

    /** The address of the board on this link, or null before first contact addressed it. */
    val boardAddr: Int? get() = controller.gatewayAddr()

    /** The controller's (guest) address, adopted from the gateway's grant. */
    val guestAddr: Int get() = controller.guestAddr

    /** The board addresses handed out (or adopted) this walk, sorted ascending. */
    fun addressedBoards(): List<Int> = controller.assignedAddrs().sorted()

    /**
     * Send an already-encoded PDU best-effort on this engine's link: no retransmit is armed and no
     * outstanding request is disturbed. This is the L3 best-effort class (`l3.md`: drive, telemetry,
     * cyclic state are fire-and-forget, latest-wins), and it is how a caller that shares this engine's
     * L2 link for its own traffic sends it. Sharing the one link is the point: a second [Link] over
     * the same byte stream would split reassembly and restart the fragment id.
     */
    fun sendPacket(packet: ByteArray) = link.send(packet)

    /**
     * The next received packet the walk did not consume (not a probe of our own port, not a
     * `CONFIG_RESP`, and not the reply that satisfied an outstanding request), or null. Frames the
     * engine has no opinion about, such as a board's `CYCLIC_STATE` emissions, surface here.
     */
    fun takeInbound(): ByteArray? = inbound.removeFirstOrNull()

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
                // A `CONFIG_RESP`: capture it, and disarm the retransmit only if what is outstanding
                // is itself a CONFIG exchange. A late duplicate response (the retransmit budget
                // exists precisely because responses are re-sent) must not disarm an unrelated walk
                // request, which is the same mistake as the else-branch below guards against.
                Pdu.decodeOrNull(frame)?.known() == Opcode.ConfigResp -> {
                    configInbox.addLast(frame)
                    if (pendingOp == Opcode.ConfigRead || pendingOp == Opcode.ConfigWrite) clearPending()
                }
                else -> {
                    // Disarm the retransmit only if this frame actually SATISFIED the outstanding
                    // request. Clearing unconditionally stalls the walk forever on a link that also
                    // carries unrelated traffic: an already-addressed board emits CYCLIC_STATE at
                    // 5 Hz, so one of those arriving between a request and its reply would disarm
                    // the retransmit, and a subsequently-lost reply would never be re-sent.
                    val wasOutstanding = controller.hasOutstanding
                    controller.onReply(frame)
                    if (wasOutstanding && !controller.hasOutstanding) clearPending() else inbound.addLast(frame)
                }
            }
        }
        if (!attachOnly || !attached) {
            controller.nextRequest()?.let {
                sendRequest(it)
                moved = true
            }
        }
        return moved
    }

    /**
     * Re-send the outstanding (unacked) request if it has now gone unanswered for [replyTimeoutMs],
     * against an idempotent responder. Call it once per poll: it decides for itself whether the
     * request is overdue, so a driver never keeps a timer of its own (see the class doc).
     *
     * [Retransmit.EXHAUSTED] means the request survived its whole budget unanswered, i.e. a
     * genuinely absent peer rather than a lost frame, so the caller can give up.
     */
    fun serviceRetransmit(): Retransmit {
        val p = pending ?: return Retransmit.IDLE
        if (nowMs() - pendingSentAtMs < replyTimeoutMs) return Retransmit.IDLE
        if (retxBudget <= 0) return Retransmit.EXHAUSTED
        retxBudget--
        pendingSentAtMs = nowMs() // the re-sent request gets its own full reply window
        link.send(p)
        return Retransmit.SENT
    }

    /** Send a `CONFIG_WRITE` to [dst] (routed by the board mesh); the reply arrives via [takeConfigResp]. */
    fun sendConfigWrite(dst: Int, key: Key, value: Value) =
        sendRequest(controller.buildConfigWrite(dst, key, value))

    /** Send a `CONFIG_READ` to [dst]; the reply arrives via [takeConfigResp]. */
    fun sendConfigRead(dst: Int, key: Key) =
        sendRequest(controller.buildConfigRead(dst, key))

    /** The next captured `CONFIG_RESP` PDU bytes, or null if none has arrived. */
    fun takeConfigResp(): ByteArray? = configInbox.removeFirstOrNull()

    companion object {
        /** Re-sends allowed per request before giving up (covers a drop in each direction + margin). */
        const val MAX_RETRANSMITS = 4

        /** Reply timeout a driver gets unless it asks for another (~one 9600-baud round trip + slack). */
        const val DEFAULT_REPLY_TIMEOUT_MS = 1_000L
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
class BleWalkDriver(
    private val pipes: BlePipeSource,
    /** The clock the engine ages its outstanding request against (injected so tests can drive it). */
    private val nowMs: () -> Long = ::monotonicNowMs,
) {

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
        val engine = BleWalkEngine(replyTimeoutMs = REPLY_TIMEOUT_MS, nowMs = nowMs)
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
     * Pump the engine until [done], draining its outgoing bytes to the pipe and letting the engine
     * re-send an overdue request. Returns false if the link dropped (notify flow ended or a write
     * threw), the peer stopped answering entirely, or the per-attempt [WALK_TIMEOUT_MS] elapsed -
     * in which case the caller reconnects.
     */
    private suspend fun pumpUntil(
        pipe: BleBytePipe,
        engine: BleWalkEngine,
        rxJob: Job,
        done: () -> Boolean,
    ): Boolean {
        val ok = withTimeoutOrNull(WALK_TIMEOUT_MS) {
            while (!done()) {
                if (rxJob.isCompleted) return@withTimeoutOrNull false // notify flow ended -> dropped
                engine.pump()
                if (!flush(pipe, engine)) return@withTimeoutOrNull false
                when (engine.serviceRetransmit()) {
                    // Spent the whole budget on one request: this connection is not going to
                    // finish the walk, so reconnect and restart it (the walk is idempotent).
                    Retransmit.EXHAUSTED -> return@withTimeoutOrNull false
                    Retransmit.SENT -> if (!flush(pipe, engine)) return@withTimeoutOrNull false
                    Retransmit.IDLE -> Unit
                }
                delay(POLL_IDLE_MS)
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

        /** How long an unanswered request waits before the engine re-sends it. */
        const val REPLY_TIMEOUT_MS = 300L
    }
}

/** The `node_address` field (field 0x01, singleton index 0), mirror of `store::NODE_ADDRESS`. */
val NODE_ADDRESS_KEY = Key(0x01, 0)
