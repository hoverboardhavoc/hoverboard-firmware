package com.hoverboard.protocol

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l2.Transport
import com.hoverboard.protocol.l3.BleBytePipe
import com.hoverboard.protocol.l3.BlePipeSource
import com.hoverboard.protocol.l3.BleWalkDriver
import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_CYCLIC_STATE
import com.hoverboard.protocol.linkctl.OP_INPUTS
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Slice-4 integration test: the controller walk driven through the REAL BLE byte-stream adapter
 * ([BleStreamTransport]) over a mock CC2541 bridge, against the firmware-mirrored [MockBoard]
 * responders (the master/slave pair). This is `WalkTest`'s property exercised through the new adapter
 * wiring (not the [com.hoverboard.protocol.l3.Controller] in isolation): the controller's L3 PDUs are
 * L2-framed, SOF/len/CRC-wrapped, RE-CHUNKED by the mock bridge (frames span chunks; chunks span
 * frame boundaries), reassembled by the firmware-side transport, answered, and ferried back. So it
 * tests the byte-stream path end to end, the "BLE bridge is a byte stream" reality and all.
 *
 * Two layers are covered: the synchronous [BleWalkEngine] (the controller-side composition the app
 * uses) directly, and the async [BleWalkDriver] (the app-runtime glue) over a loopback [BleBytePipe].
 */
class BleWalkTest {

    /** `store::MOTOR_CURRENT_LIMIT` (field 0x20, singleton index 0). */
    private val motorCurrentLimit = Key(0x20, 0)

    @Test
    fun masterSlavePairDiscoveredOverTheBleAdapter() {
        val f = SyncFleet()
        f.runWalk()

        // Both boards addressed positionally over the BLE byte stream: gateway 0x01, slave 0x02.
        assertEquals(0x01, f.boards.masterAddr())
        assertEquals(0x02, f.boards.slaveAddr())
        assertEquals(listOf(0x01, 0x02), f.engine.addressedBoards())
        // Both PERSIST (survive a reboot), not just live.
        assertEquals(0x01, f.boards.masterPersisted())
        assertEquals(0x02, f.boards.slavePersisted())
    }

    @Test
    fun twoHopConfigToTheSlaveRoundTripsOverTheBleAdapter() {
        val f = SyncFleet()
        f.runWalk()

        // A CONFIG to the slave (0x02) reaches it THROUGH the gateway (two hops), all over the BLE
        // byte stream: source-learning routes the request out and the reply back.
        val wResp = f.configWrite(0x02, motorCurrentLimit, Value.U32(21_000))
        assertEquals(Walk.CFG_OK, wResp.status)

        val rResp = f.configRead(0x02, motorCurrentLimit)
        assertEquals(Walk.CFG_OK, rResp.status)
        assertEquals(motorCurrentLimit.fieldId, rResp.fieldId)
        assertEquals(Value.U32(21_000), rResp.decodeValue())
    }

    @Test
    fun aConfigToTheGatewayRoundTripsOverTheBleAdapter() {
        val f = SyncFleet()
        f.runWalk()

        val wResp = f.configWrite(0x01, motorCurrentLimit, Value.U32(15_000))
        assertEquals(Walk.CFG_OK, wResp.status)
        assertEquals(Value.U32(15_000), wResp.decodeValue())
        val rResp = f.configRead(0x01, motorCurrentLimit)
        assertEquals(Value.U32(15_000), rResp.decodeValue())
    }

    @Test
    fun walkAndConfigSurviveADroppedFrameEachDirectionViaRetransmit() {
        // The BLE byte stream drops the first stream frame in EACH direction of the walk and of each
        // CONFIG exchange. With no retransmit the walk stalls; the engine re-sends the unacked request
        // (against the idempotent responder) and the whole walk + two-hop CONFIG still complete.
        val f = SyncFleet(dropFirstFrame = true)
        f.runWalk()

        assertEquals(0x01, f.boards.masterAddr())
        assertEquals(0x02, f.boards.slaveAddr())
        assertEquals(listOf(0x01, 0x02), f.engine.addressedBoards())

        val wResp = f.configWrite(0x02, motorCurrentLimit, Value.U32(21_000))
        assertEquals(Walk.CFG_OK, wResp.status)
        val rResp = f.configRead(0x02, motorCurrentLimit)
        assertEquals(Value.U32(21_000), rResp.decodeValue())
    }

    // --- attach-only: the rider remote's leg of the walk ---------------------------------------

    @Test
    fun anAttachOnlyEngineAddressesTheBoardAndStopsBeforeTheTreeWalk() {
        val f = SyncFleet(attachOnly = true)
        f.runAttach()

        // First contact did its whole job: the board is addressed and persisted, and the app took
        // the guest address the board granted it.
        assertEquals(0x01, f.boards.masterAddr())
        assertEquals(0x01, f.boards.masterPersisted())
        assertEquals(0x01, f.engine.boardAddr)
        assertTrue(f.engine.guestAddr in 0x80..0xFE, "guestAddr=${f.engine.guestAddr}")

        // ...and stopped there. The tree behind the board is deliberately not mapped, so the slave
        // is still unaddressed - assigning it is the fleet controller's job, not the rider's.
        assertEquals(0x00, f.boards.slaveAddr())
        assertEquals(null, f.boards.slavePersisted())
        // `walkComplete` is NOT the completion condition here: the queued PROBE_PORTS is simply
        // never issued, which is what [attached] exists to say instead.
        assertFalse(f.engine.walkComplete)
    }

    @Test
    fun anAttachOnlyEngineAdoptsABoardThatAlreadyHasAnIdentity() {
        val f = SyncFleet(attachOnly = true)
        f.boards.preassignMaster(0x02) // a past session's address, persisted
        f.runAttach()

        // Adopted, not reassigned: the address the board reports is the one the app addresses it by.
        assertEquals(0x02, f.boards.masterAddr())
        assertEquals(0x02, f.boards.masterPersisted())
        assertEquals(0x02, f.engine.boardAddr)
    }

    @Test
    fun anAttachedEngineCarriesOrdinaryTrafficOnTheSameLink() {
        val f = SyncFleet(attachOnly = true)
        f.runAttach()

        // A board emission the walk has no opinion about surfaces to the caller rather than being
        // swallowed: this is how the rider's telemetry reaches it over the attach link.
        val cyclic = Pdu(OP_CYCLIC_STATE, 0x01, 0x00, ByteArray(CyclicState.LEN)).encode()
        f.deliverFromBoard(cyclic)
        f.engine.pump()
        val got = f.engine.takeInbound()
        assertNotNull(got)
        assertEquals(OP_CYCLIC_STATE, Pdu.decode(got!!).opcode)

        // ...and the caller's own best-effort PDU goes out that one link, no retransmit armed.
        f.engine.sendPacket(Pdu(OP_INPUTS, f.engine.guestAddr, 0x01, ByteArray(Inputs.LEN)).encode())
        assertNotNull(f.engine.takeOutgoing())
        assertFalse(f.engine.retransmitPending())
    }

    @Test
    fun unrelatedTrafficDoesNotDisarmAnOutstandingRequestsRetransmit() {
        // An already-addressed board emits CYCLIC_STATE at 5 Hz, so one lands between a request and
        // its reply routinely. If that disarmed the retransmit, a subsequently-lost reply would
        // stall the exchange forever.
        val engine = BleWalkEngine(attachOnly = true)
        engine.pump() // sends NODE_HELLO
        assertNotNull(engine.takeOutgoing())

        val t = BleStreamTransport()
        Link(t).send(Pdu(OP_CYCLIC_STATE, 0x01, 0x00, ByteArray(CyclicState.LEN)).encode())
        engine.onReceive(t.drainOutgoing()!!)
        engine.pump()

        assertFalse(engine.attached)
        assertTrue(engine.retransmitPending(), "the unanswered NODE_HELLO must still be re-sendable")
        assertNotNull(engine.takeOutgoing())
    }

    @Test
    fun theAsyncDriverWalksAndReadsBackOverALoopbackPipe() = runTest {
        val boards = BoardFleet()
        // A pipe source that hands out a fresh (non-dropping) loopback pipe over the same boards.
        val outcome = BleWalkDriver { LoopbackPipe(boards) }.discover()

        // The app driver discovered both boards over the (async) BLE pipe...
        assertEquals(0x01, outcome.entryAddr)
        assertEquals(listOf(0x01, 0x02), outcome.boards)
        assertEquals(0x01, boards.masterAddr())
        assertEquals(0x02, boards.slaveAddr())
        // ...and adopted a transient guest address for itself (0x80..0xFE).
        assertTrue(outcome.controllerAddr in 0x80..0xFE, "controllerAddr=${outcome.controllerAddr}")
        // ...and its two-hop node_address read-back of the farthest board (the slave) round-tripped.
        assertNotNull(outcome.configEcho)
        assertTrue(outcome.configEcho!!.contains("0x2"), "config echo: ${outcome.configEcho}")
    }

    @Test
    fun theDriverReconnectsAndRestartsAfterAMidWalkDrop() = runTest {
        // The FIRST connection drops part-way through the walk (the bench's ~5 s supervision timeout);
        // the driver must reconnect to the same boards and RESTART the walk (idempotent - it adopts any
        // already-assigned boards) to still complete the fleet + the two-hop CONFIG.
        val boards = BoardFleet()
        val source = DroppingPipeSource(boards, dropAfterRxChunks = 3)
        val outcome = BleWalkDriver(source).discover()

        assertEquals(0x01, outcome.entryAddr)
        assertEquals(listOf(0x01, 0x02), outcome.boards)
        assertNotNull(outcome.configEcho)
        assertTrue(outcome.configEcho!!.contains("0x2"), "config echo: ${outcome.configEcho}")
        assertTrue(source.attempts >= 2, "expected at least one reconnect; attempts=${source.attempts}")
    }

    // -------------------------------------------------------------------------------------------
    // Harness. A master/slave MockBoard fleet whose BLE-facing port is a BleStreamTransport (the
    // firmware's BLE link is also byte-stream), so BOTH ends of the controller<->master link use the
    // real adapter. The CC2541 bridge is modelled by re-chunking every byte stream into small pieces.
    // -------------------------------------------------------------------------------------------

    /** A datagram transport for the inter-board (master<->slave) UART hop: one frame per transaction. */
    private class DatagramPort(
        private val tx: ArrayDeque<ByteArray>,
        private val rx: ArrayDeque<ByteArray>,
    ) : Transport {
        override fun frameCapacity(): Int = 255
        override fun sendL2Frame(l2: ByteArray) = tx.addLast(l2.copyOf())
        override fun recvL2Frame(): ByteArray? = rx.removeFirstOrNull()
    }

    /** The board side: master (BLE port 0 + UART port 1) + slave (UART port 0), driven to quiescence. */
    private class BoardFleet {
        /** The master's BLE-facing byte-stream port (the far end of the link from the controller). */
        val masterBle = BleStreamTransport()

        private val master = MockBoard(
            nPorts = 2,
            portKinds = intArrayOf(Walk.PORT_BLE, Walk.PORT_UART, Walk.PORT_UART, Walk.PORT_UART),
            mcu = 0x10,
            fwVer = 0x0001,
        )
        private val slave = MockBoard(
            nPorts = 1,
            portKinds = intArrayOf(Walk.PORT_UART, Walk.PORT_UART, Walk.PORT_UART, Walk.PORT_UART),
            mcu = 0x10,
            fwVer = 0x0001,
        )
        private val masterPorts: Array<Link?>
        private val slavePorts: Array<Link?>

        init {
            val mToS = ArrayDeque<ByteArray>()
            val sToM = ArrayDeque<ByteArray>()
            masterPorts = arrayOf(Link(masterBle), Link(DatagramPort(tx = mToS, rx = sToM)))
            slavePorts = arrayOf(Link(DatagramPort(tx = sToM, rx = mToS)))
        }

        fun masterAddr(): Int = master.addr()
        fun slaveAddr(): Int = slave.addr()

        /** The master kept an address from a past session (it boots reporting it). */
        fun preassignMaster(addr: Int) = master.preassign(addr)
        fun masterPersisted(): Int? = master.persistedAddr()
        fun slavePersisted(): Int? = slave.persistedAddr()

        /** One processing pass over both boards (ingest ready frames, route emissions). */
        fun step(): Boolean {
            var moved = false
            if (driveBoard(master, masterPorts)) moved = true
            if (driveBoard(slave, slavePorts)) moved = true
            return moved
        }

        /** Fire any pending probe tick (the firmware's "probe window elapsed"). */
        fun fireProbes(): Boolean {
            var ticked = false
            if (master.probing()) {
                val emits = ArrayList<Emission>()
                master.pollProbe(emits)
                route(masterPorts, emits)
                ticked = true
            }
            if (slave.probing()) {
                val emits = ArrayList<Emission>()
                slave.pollProbe(emits)
                route(slavePorts, emits)
                ticked = true
            }
            return ticked
        }

        /** Drive the boards to quiescence on their own (used after bytes are delivered to the master). */
        fun settle() {
            repeat(MAX_STEPS) {
                if (!step() && !fireProbes()) return
            }
            error("boards did not quiesce")
        }

        private fun driveBoard(board: MockBoard, ports: Array<Link?>): Boolean {
            var moved = false
            for (p in ports.indices) {
                while (true) {
                    val frame = ports[p]?.pollRecv() ?: break
                    moved = true
                    val emits = ArrayList<Emission>()
                    board.ingest(p, frame, emits)
                    route(ports, emits)
                }
            }
            return moved
        }

        private fun route(ports: Array<Link?>, emits: List<Emission>) {
            for (e in emits) ports.getOrNull(e.port)?.send(e.bytes)
        }
    }

    /** The synchronous controller engine + a [BoardFleet], pumped together over the BLE byte loopback. */
    private inner class SyncFleet(
        private val dropFirstFrame: Boolean = false,
        attachOnly: Boolean = false,
    ) {
        val engine = BleWalkEngine(attachOnly = attachOnly)
        val boards = BoardFleet()

        // Per-phase frame counters (reset each pump phase) so `dropFirstFrame` drops the first stream
        // frame in EACH direction of the walk AND of each CONFIG exchange - forcing retransmit.
        private var c2mFrame = 0
        private var m2cFrame = 0

        fun runWalk() {
            pumpToQuiescence()
            check(engine.walkComplete) { "walk did not complete" }
        }

        /** Run only first contact (the rider remote's completion condition). */
        fun runAttach() {
            pumpToQuiescence()
            check(engine.attached) { "attach did not complete" }
        }

        /** Deliver an unsolicited board emission (a CYCLIC_STATE) up the same BLE byte stream. */
        fun deliverFromBoard(pdu: ByteArray) {
            val t = BleStreamTransport()
            Link(t).send(pdu)
            rechunk(t.drainOutgoing()!!).forEach { engine.onReceive(it) }
        }

        fun configWrite(dst: Int, key: Key, value: Value): ConfigResp {
            engine.sendConfigWrite(dst, key, value)
            pumpToQuiescence()
            return ConfigResp.parse(Pdu.decode(engine.takeConfigResp() ?: error("no CONFIG_RESP")))!!
        }

        fun configRead(dst: Int, key: Key): ConfigResp {
            engine.sendConfigRead(dst, key)
            pumpToQuiescence()
            return ConfigResp.parse(Pdu.decode(engine.takeConfigResp() ?: error("no CONFIG_RESP")))!!
        }

        /** Drop the first frame this phase in the controller->master direction (models a lost BLE write). */
        private fun dropC2M(): Boolean = dropFirstFrame && c2mFrame++ == 0

        /** Drop the first frame this phase in the master->controller direction (a lost notification). */
        private fun dropM2C(): Boolean = dropFirstFrame && m2cFrame++ == 0

        private fun pumpToQuiescence() {
            c2mFrame = 0
            m2cFrame = 0
            repeat(MAX_STEPS) {
                var moved = false
                // The BLE byte stream, both directions, RE-CHUNKED (the bridge does not preserve frame
                // boundaries): drain each side's outgoing stream and feed it as small chunks to the other
                // - except a dropped frame, which is drained but never delivered (a lost frame).
                engine.takeOutgoing()?.let { bytes ->
                    moved = true
                    if (!dropC2M()) rechunk(bytes).forEach { boards.masterBle.onReceive(it) }
                }
                boards.masterBle.drainOutgoing()?.let { bytes ->
                    moved = true
                    if (!dropM2C()) rechunk(bytes).forEach { engine.onReceive(it) }
                }
                if (engine.pump()) moved = true
                if (boards.step()) moved = true
                if (moved) return@repeat
                // Stalled: a probe tick, else retransmit the lost request, else genuinely quiesced.
                if (boards.fireProbes()) return@repeat
                if (engine.retransmitPending()) return@repeat
                return
            }
            error("fleet did not quiesce")
        }
    }

    /**
     * A loopback [BleBytePipe] over a [BoardFleet]: a controller write is delivered (re-chunked) to the
     * master, the boards run to quiescence, and the master's reply stream is emitted (re-chunked) back
     * on [incoming]. Models the async CC2541 GATT pipe the real driver uses.
     *
     * If [dropAfterRxChunks] is set, after delivering that many reply chunks the pipe "drops": it closes
     * [incoming] (the notify flow ends) and makes further [write]s throw - exactly the signals the driver
     * reads as a GATT supervision-timeout drop.
     */
    private class LoopbackPipe(
        private val boards: BoardFleet,
        private val dropAfterRxChunks: Int = Int.MAX_VALUE,
    ) : BleBytePipe {
        // An unbounded channel buffers every chunk regardless of when the driver's collector subscribes
        // (a SharedFlow would drop emissions made before subscription, stalling the walk).
        private val channel = Channel<ByteArray>(Channel.UNLIMITED)
        override val incoming: Flow<ByteArray> = channel.receiveAsFlow()
        private var rxChunks = 0
        private var dropped = false

        override suspend fun write(bytes: ByteArray) {
            if (dropped) error("link dropped")
            rechunk(bytes).forEach { boards.masterBle.onReceive(it) }
            boards.settle()
            boards.masterBle.drainOutgoing()?.let { reply ->
                for (chunk in rechunk(reply)) {
                    if (rxChunks >= dropAfterRxChunks) {
                        dropped = true
                        channel.close() // ends `incoming` -> the driver's rxJob completes -> drop detected
                        return
                    }
                    channel.trySend(chunk)
                    rxChunks++
                }
            }
        }
    }

    /**
     * A [BlePipeSource] whose FIRST connection drops mid-walk (after [dropAfterRxChunks] reply chunks)
     * and whose later connections are clean - so the driver must reconnect once and restart the walk.
     */
    private class DroppingPipeSource(
        private val boards: BoardFleet,
        private val dropAfterRxChunks: Int,
    ) : BlePipeSource {
        var attempts = 0
            private set

        override suspend fun connect(): BleBytePipe {
            attempts++
            return LoopbackPipe(boards, if (attempts == 1) dropAfterRxChunks else Int.MAX_VALUE)
        }
    }

    private companion object {
        const val MAX_STEPS = 100_000
        const val RECHUNK = 7

        /**
         * Re-chunk a byte buffer into small fixed-size pieces to model the CC2541 transparent-UART
         * bridge: frames (~20 B) span several chunks and chunk boundaries fall mid-frame, so the
         * resyncing [com.hoverboard.protocol.l2.StreamFramer] must accumulate across chunks. Deterministic.
         */
        fun rechunk(bytes: ByteArray, size: Int = RECHUNK): List<ByteArray> {
            val out = ArrayList<ByteArray>()
            var off = 0
            while (off < bytes.size) {
                val end = minOf(off + size, bytes.size)
                out.add(bytes.copyOfRange(off, end))
                off = end
            }
            return out
        }
    }
}
