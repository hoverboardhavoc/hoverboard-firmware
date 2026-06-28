package com.hoverboard.app.net.l3

import com.hoverboard.app.net.l2.BleStreamTransport
import com.hoverboard.app.net.l2.Link
import com.hoverboard.app.net.store.Key
import com.hoverboard.app.net.store.Value
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.withTimeoutOrNull

/** The fleet a controller walk discovered: the gateway (master) it attached to + every addressed board. */
data class WalkOutcome(
    /** The gateway/master address (the board the app's BLE link attaches to), or null if none. */
    val gatewayAddr: Int?,
    /** Every board addressed (or adopted) this walk, sorted ascending. */
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
                reply != null -> link.send(reply)
                Pdu.decodeOrNull(frame)?.known() == Opcode.ConfigResp -> configInbox.addLast(frame)
                else -> controller.onReply(frame)
            }
        }
        controller.nextRequest()?.let {
            link.send(it)
            moved = true
        }
        return moved
    }

    /** Send a `CONFIG_WRITE` to [dst] (routed by the board mesh); the reply arrives via [takeConfigResp]. */
    fun sendConfigWrite(dst: Int, key: Key, value: Value) =
        link.send(controller.buildConfigWrite(dst, key, value))

    /** Send a `CONFIG_READ` to [dst]; the reply arrives via [takeConfigResp]. */
    fun sendConfigRead(dst: Int, key: Key) =
        link.send(controller.buildConfigRead(dst, key))

    /** The next captured `CONFIG_RESP` PDU bytes, or null if none has arrived. */
    fun takeConfigResp(): ByteArray? = configInbox.removeFirstOrNull()
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
 * Runs the controller-side walk (and an optional two-hop CONFIG read) over a live [BleBytePipe]. This
 * is the app-runtime glue: the protocol stepping is [BleWalkEngine] (host-tested through a loopback),
 * so this only plumbs async BLE I/O - collect notifications into the engine, drain the engine's bytes
 * to the write char, loop to quiescence. The on-phone run against a real advertising module is the
 * deferred slice-4 silicon verify.
 */
class BleWalkDriver(
    private val pipe: BleBytePipe,
    private val engine: BleWalkEngine = BleWalkEngine(),
) {

    /** Walk the fleet over the BLE link, then read each board's `node_address` back (two-hop to the slave). */
    suspend fun discover(): WalkOutcome = coroutineScope {
        engine.transport.resetRx()
        // Collect notifications into the engine for the duration of the walk; cancel it before
        // returning so this `coroutineScope` is not held open by the never-ending collector.
        val rxJob = pipe.incoming.onEach { engine.onReceive(it) }.launchIn(this)
        try {
            runUntil { engine.walkComplete }
            val boards = engine.addressedBoards()

            // Demonstrate the two-hop path without mutating flash: read node_address back from the
            // farthest board (for the master/slave pair this routes through the gateway to the slave).
            val configEcho = boards.lastOrNull()?.let { dst ->
                engine.sendConfigRead(dst, NODE_ADDRESS_KEY)
                engine.takeOutgoing()?.let { pipe.write(it) }
                val resp = runForConfigResp()
                resp?.let { "node_address(0x${Integer.toHexString(dst)}) = 0x${Integer.toHexString(it)}" }
            }

            WalkOutcome(gatewayAddr = boards.firstOrNull(), boards = boards, configEcho = configEcho)
        } finally {
            rxJob.cancel()
        }
    }

    private suspend fun runUntil(done: () -> Boolean) {
        withTimeoutOrNull(WALK_TIMEOUT_MS) {
            while (!done()) {
                val moved = engine.pump()
                engine.takeOutgoing()?.let { pipe.write(it) }
                if (!moved) delay(POLL_IDLE_MS)
            }
        }
    }

    /** Pump until a `CONFIG_RESP` is captured; return its decoded `node_address` value, or null. */
    private suspend fun runForConfigResp(): Int? {
        var resp: ConfigResp? = null
        runUntil {
            engine.takeConfigResp()?.let { bytes ->
                Pdu.decodeOrNull(bytes)?.let { resp = ConfigResp.parse(it) }
            }
            resp != null
        }
        val value = resp?.decodeValue()
        return (value as? Value.U8)?.v
    }

    private companion object {
        /** Bound the walk / CONFIG loop so a silent or wedged link surfaces instead of hanging. */
        const val WALK_TIMEOUT_MS = 10_000L

        /** Idle backoff between polls while waiting for the next reply to arrive over the link. */
        const val POLL_IDLE_MS = 15L
    }
}

/** The `node_address` field (field 0x01, singleton index 0), mirror of `store::NODE_ADDRESS`. */
val NODE_ADDRESS_KEY = Key(0x01, 0)
