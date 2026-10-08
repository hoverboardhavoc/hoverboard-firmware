package com.hoverboard.remote.ble

import com.hoverboard.protocol.config.ConfigClient
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.OP_CYCLIC_STATE
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.isActive
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The app's own attach loop, against a fake board over the real L2 byte stream.
 *
 * The loop [L3Session.attach] runs is the rider's ENTIRE first contact: until it succeeds the app
 * has no address, the board emits no telemetry, and nothing is drivable. It used to live inline in
 * [BleHoverboardTransport] where nothing could exercise it, which is exactly where a defect that
 * disabled the retransmit on the common reconnect path survived a full audit.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class L3SessionTest {

    /**
     * THE regression test for that defect.
     *
     * A board addressed in an earlier session restores its address at boot and streams CYCLIC_STATE
     * at 5 Hz from power-on, so a reconnect attaches while telemetry is already flowing: that is the
     * NORMAL case, not an edge case. When the reply to the app's NODE_HELLO is lost on that stream,
     * the retransmit is the only thing that can finish the attach. A timer that resets on "the
     * engine moved" never fires here, because a telemetry frame every 200 ms moves the engine long
     * before the reply timeout elapses.
     */
    @Test
    fun `a lost hello reply is retransmitted while the board streams telemetry`() = runTest {
        val telemetry = ArrayList<ByteArray>()
        val h = Harness(this, nodeId = 0x01, dropReplies = 1, onPacket = { telemetry.add(it) })
        h.streamCyclicState()

        val outcome = h.session.attach()

        assertTrue(h.helloCount >= 2, "the lost NODE_HELLO was never re-sent (sent ${h.helloCount}x)")
        assertEquals(AttachOutcome.Attached(Attachment(guestAddr = GUEST, boardAddr = 0x01)), outcome)
        // The telemetry that hid the defect is still delivered to the app, not swallowed by the walk.
        assertTrue(telemetry.isNotEmpty(), "no CYCLIC_STATE reached the app during the attach")
        assertTrue(
            telemetry.all { Pdu.decode(it).opcode == OP_CYCLIC_STATE },
            "unexpected packet dispatched during the attach",
        )
    }

    @Test
    fun `attach adopts the address a board already holds`() = runTest {
        val h = Harness(this, nodeId = 0x02)

        val outcome = h.session.attach()

        assertEquals(AttachOutcome.Attached(Attachment(guestAddr = GUEST, boardAddr = 0x02)), outcome)
        assertEquals(1, h.helloCount, "a board that answers first time needs no retransmit")
    }

    @Test
    fun `attach gives up when the board never answers`() = runTest {
        val h = Harness(this, nodeId = 0x01, dropReplies = Int.MAX_VALUE)
        h.streamCyclicState()

        val outcome = h.session.attach()

        assertEquals(AttachOutcome.Unanswered, outcome)
        // One send plus the engine's whole retransmit budget, all of them under live telemetry.
        assertEquals(BleWalkEngine.MAX_RETRANSMITS + 1, h.helloCount)
    }

    @Test
    fun `attach stops at its deadline`() = runTest {
        // A deadline shorter than one reply timeout: the backstop ends it, not the retransmit budget.
        val h = Harness(this, nodeId = 0x01, dropReplies = Int.MAX_VALUE, deadlineMs = 200L)

        assertEquals(AttachOutcome.Deadline, h.session.attach())
        assertEquals(1, h.helloCount)
    }

    /**
     * The wiring [BleHoverboardTransport] uses for config: a [ConfigClient] over the session's own
     * engine and lock, with NOTHING turning the engine but the session's service loop. The client
     * stages requests and waits; the loop flushes them and pumps the replies in. If the client and
     * the loop did not share the engine and the lock, this read would never be answered.
     */
    @Test
    fun `a config client over the session engine is driven by the session loop`() = runTest {
        val h = Harness(this, nodeId = 0x01)
        h.session.attach()
        backgroundScope.launch {
            while (isActive) {
                h.session.turn()
                delay(L3Session.POLL_IDLE_MS)
            }
        }
        val client = ConfigClient(h.engine, h.lock)
        val trim = Fields.ATTITUDE_LEVEL_TRIM.key(0)

        assertEquals(WriteVerified(Value.I16(-266)), client.write(trim, Value.I16(-266), 0x01))
        assertEquals(ReadValue(Value.I16(-266)), client.read(trim, 0x01))
    }

    // -----------------------------------------------------------------------------------------

    /**
     * The app's engine and [L3Session] wired to a fake board over a loopback byte stream. The board
     * answers NODE_HELLO with the identity it already holds and can stream CYCLIC_STATE the way an
     * addressed board does; [dropReplies] swallows that many replies, which is a lost frame on a
     * stream the repo documents as lossy.
     *
     * Both the engine's reply timer and the loop's deadline read the test scheduler's clock, so the
     * whole 12 s attach runs in virtual time.
     */
    private class Harness(
        private val scope: TestScope,
        private val nodeId: Int,
        private val dropReplies: Int = 0,
        deadlineMs: Long = L3Session.ATTACH_DEADLINE_MS,
        onPacket: (ByteArray) -> Unit = {},
    ) {
        private val clock = { scope.testScheduler.currentTime }
        private val wire = BleStreamTransport()
        private val board = Link(wire)
        val engine = BleWalkEngine(
            attachOnly = true,
            replyTimeoutMs = L3Session.REPLY_TIMEOUT_MS,
            nowMs = clock,
        )
        val lock = Any()

        /** The board's store, as a map: written by `CONFIG_WRITE`, read back by both config opcodes. */
        private val store = HashMap<Pair<Int, Int>, Value>()

        val session = L3Session(
            engine = engine,
            lock = lock,
            nowMs = clock,
            onPacket = onPacket,
            deadlineMs = deadlineMs,
            write = { bytes -> onWrite(bytes) },
        )

        /** How many NODE_HELLO requests the board received (the first send plus every retransmit). */
        var helloCount = 0
            private set

        /** Start the board's own emissions, at the firmware's decimated 5 Hz BLE cadence. */
        fun streamCyclicState() {
            scope.backgroundScope.launch {
                while (true) {
                    delay(CYCLIC_PERIOD_MS)
                    board.send(Pdu(OP_CYCLIC_STATE, nodeId, 0x00, ByteArray(CyclicState.LEN)).encode())
                    drainToApp()
                }
            }
        }

        private fun onWrite(bytes: ByteArray) {
            wire.onReceive(bytes)
            while (true) {
                val frame = board.pollRecv() ?: break
                val pdu = Pdu.decodeOrNull(frame)
                when (pdu?.known()) {
                    Opcode.NodeHello -> {
                        helloCount++
                        if (helloCount > dropReplies) board.send(helloReply(pdu.src))
                    }
                    Opcode.ConfigWrite, Opcode.ConfigRead -> board.send(configReply(pdu))
                    else -> Unit
                }
            }
            drainToApp()
        }

        /** `[node_id, proto_ver, fw_lo, fw_hi, mcu, your_addr]` (specs/l3.md, the merged reply). */
        private fun helloReply(askedBy: Int): ByteArray = Pdu.of(
            Opcode.NodeHello,
            nodeId,
            askedBy,
            byteArrayOf(nodeId.toByte(), Walk.PROTO_VER.toByte(), 0, 0, MCU_TAG, GUEST.toByte()),
        ).encode()

        /**
         * `CONFIG_RESP` `[field_id, index, CFG_OK, type, value]` echoing the stored value; a write's
         * payload is `[field_id, index, type, value]`.
         */
        private fun configReply(req: Pdu): ByteArray {
            val p = req.payload
            val key = (p[0].toInt() and 0xFF) to (p[1].toInt() and 0xFF)
            if (req.known() == Opcode.ConfigWrite) {
                val type = Type.fromTag(p[2].toInt() and 0xFF)!!
                store[key] = Value.decode(type, p.copyOfRange(3, p.size))!!
            }
            val v = store.getValue(key)
            val body = byteArrayOf(p[0], p[1], Walk.CFG_OK.toByte(), v.kind().tag.toByte()) + v.encode()
            return Pdu.of(Opcode.ConfigResp, nodeId, req.src, body).encode()
        }

        private fun drainToApp() {
            wire.drainOutgoing()?.let { engine.onReceive(it) }
        }
    }

    private companion object {
        /** The guest address the fake board grants (`0x80..0xFE`, one past the provisional 0x80). */
        const val GUEST = 0x81
        const val MCU_TAG: Byte = 0x10

        /** The firmware's decimated BLE telemetry cadence: 5 Hz. */
        const val CYCLIC_PERIOD_MS = 200L
    }
}
