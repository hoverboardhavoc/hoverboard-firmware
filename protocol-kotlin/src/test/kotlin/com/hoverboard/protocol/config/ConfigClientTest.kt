package com.hoverboard.protocol.config

import com.hoverboard.protocol.l2.BleStreamTransport
import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.EnumSource
import java.lang.reflect.Modifier

/**
 * [ConfigClient] driven through a REAL [BleWalkEngine] (so its retransmit is the engine's own, not a
 * re-implementation) against [FakeConfigBoards], a scripted board on the far side of the BLE byte
 * stream. Virtual time throughout: the engine's clock is the test scheduler's.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class ConfigClientTest {

    private val limit = Key(0x20, 0) // MOTOR_CURRENT_LIMIT, u32
    private val master = 0x01
    private val slave = 0x02

    private companion object {
        /** Long enough for the fake's driver to carry a request across, short of any retransmit. */
        const val ONE_TURN_MS = 50L
    }

    @Test
    fun aReadReturnsTheStoredValue() = runTest {
        val f = FakeConfigBoards(this)
        f.store[slave to limit] = Value.U32(15_000)

        assertEquals(ReadValue(Value.U32(15_000)), f.client.read(limit, slave))
        assertEquals(listOf(Opcode.ConfigRead.value), f.requests.map { it.opcode })
    }

    @Test
    fun aWriteIsVerifiedAgainstTheStoredValueItsResponseEchoes() = runTest {
        val f = FakeConfigBoards(this)

        assertEquals(WriteVerified(Value.U32(21_000)), f.client.write(limit, Value.U32(21_000), slave))
        assertEquals(Value.U32(21_000), f.store[slave to limit])
    }

    @Test
    fun aWriteWhoseEchoDisagreesIsAMismatchNotASuccess() = runTest {
        val f = FakeConfigBoards(this)
        f.echoOverride = Value.U32(9_999)

        assertEquals(
            WriteMismatch(wrote = Value.U32(21_000), stored = Value.U32(9_999)),
            f.client.write(limit, Value.U32(21_000), slave),
        )
    }

    @Test
    fun aWriteAcceptedWithNoEchoIsAMismatchWithNoStoredValue() = runTest {
        // The firmware answers CFG_OK with no value when its post-write read fails (walk.rs).
        val f = FakeConfigBoards(this)
        f.omitEcho = true

        assertEquals(WriteMismatch(wrote = Value.U32(21_000), stored = null), f.client.write(limit, Value.U32(21_000), slave))
    }

    @ParameterizedTest
    @EnumSource(CfgRefusal::class)
    fun eachRefusalStatusDecodesToItsNamedRefusal(refusal: CfgRefusal) = runTest {
        val f = FakeConfigBoards(this)
        f.status = refusal.code

        assertEquals(Refused(refusal), f.client.write(limit, Value.U32(1), slave))
        assertEquals(Refused(refusal), f.client.read(limit, slave))
    }

    /**
     * Every `CFG_*` status in [Walk] other than `CFG_OK`, read by reflection (as
     * `RustSourceDriftTest` reads the same object), so a status pinned into [Walk] from the Rust
     * fails here until [CfgRefusal] names it.
     */
    @Test
    fun theRefusalSetIsEveryNonOkStatusTheWireDefines() {
        val wire = Walk::class.java.declaredFields
            .filter { Modifier.isStatic(it.modifiers) && it.type == Int::class.javaPrimitiveType }
            .filter { it.name.startsWith("CFG_") && it.name != "CFG_OK" }
            .associate { it.name.removePrefix("CFG_") to it.getInt(null) }
        assertTrue(wire.isNotEmpty(), "no CFG_ statuses found on Walk by reflection")
        assertEquals(wire, CfgRefusal.entries.associate { it.name to it.code })
    }

    @Test
    fun anOkReadWhoseValueDoesNotDecodeIsMalformed() = runTest {
        val f = FakeConfigBoards(this)
        f.store[slave to limit] = Value.U32(15_000)

        f.readBodyOverride = byteArrayOf(0) // type tag 0: no such type
        assertTrue(f.client.read(limit, slave) is Malformed)

        f.readBodyOverride = byteArrayOf(Type.U32.tag.toByte(), 1, 2) // a u32 cut to two bytes
        assertTrue(f.client.read(limit, slave) is Malformed)
    }

    @Test
    fun anUndefinedStatusIsMalformed() = runTest {
        val f = FakeConfigBoards(this)
        f.status = 0x42

        assertTrue(f.client.write(limit, Value.U32(1), slave) is Malformed)
        assertTrue(f.client.read(limit, slave) is Malformed)
    }

    @Test
    fun aSilentBoardTimesOutAfterTheEnginesWholeRetransmitBudget() = runTest {
        val f = FakeConfigBoards(this)
        f.silent = true

        assertEquals(TimedOut, f.client.write(limit, Value.U32(1), slave))
        // The original plus every re-send the engine allows, all of the same request.
        assertEquals(1 + BleWalkEngine.MAX_RETRANSMITS, f.requests.size)
        assertEquals(1, f.requests.distinct().size)
    }

    @Test
    fun aLostRequestIsRecoveredByTheEnginesRetransmit() = runTest {
        val f = FakeConfigBoards(this)
        f.dropNext = 1

        assertEquals(WriteVerified(Value.U32(7)), f.client.write(limit, Value.U32(7), slave))
        assertEquals(2, f.requests.size)
    }

    @Test
    fun aSecondOperationWhileOneIsInFlightIsRefusedAndSendsNothing() = runTest {
        val f = FakeConfigBoards(this)
        f.hold = true

        val first = async { f.client.write(limit, Value.U32(5), slave) }
        advanceTimeBy(ONE_TURN_MS) // the first request reaches the board, which holds it
        assertEquals(1, f.requests.size)
        assertEquals(Busy, f.client.read(limit, master))
        assertEquals(Busy, f.client.write(limit, Value.U32(6), master))
        advanceTimeBy(ONE_TURN_MS)
        assertEquals(1, f.requests.size, "a refused operation put something on the wire")

        f.hold = false
        assertEquals(WriteVerified(Value.U32(5)), first.await())
        // Once the first completes, the client takes the next operation.
        assertEquals(ReadValue(Value.U32(5)), f.client.read(limit, slave))
    }

    @Test
    fun eachOperationGoesToTheBoardItNames() = runTest {
        val f = FakeConfigBoards(this)
        f.store[master to limit] = Value.U32(100)
        f.store[slave to limit] = Value.U32(200)

        assertEquals(ReadValue(Value.U32(200)), f.client.read(limit, slave))
        assertEquals(ReadValue(Value.U32(100)), f.client.read(limit, master))
        f.client.write(limit, Value.U32(300), slave)

        assertEquals(listOf(slave, master, slave), f.requests.map { it.dst })
        assertEquals(Value.U32(100), f.store[master to limit], "a write to the slave touched the master")
        assertEquals(Value.U32(300), f.store[slave to limit])
    }

    @Test
    fun aResponseFromAnotherBoardOrForAnotherKeyIsNotTheAnswer() = runTest {
        val f = FakeConfigBoards(this)
        f.store[slave to limit] = Value.U32(200)
        // Ahead of the real answer, the link delivers a response from the wrong board and one for
        // the wrong key (late duplicates of earlier exchanges).
        f.strayAhead = listOf(
            Pdu.of(Opcode.ConfigResp, master, 0, byteArrayOf(0x20, 0, 0, 0x03, 1, 0, 0, 0)),
            Pdu.of(Opcode.ConfigResp, slave, 0, byteArrayOf(0x01, 0, 0, 0x01, 9)),
        )

        assertEquals(ReadValue(Value.U32(200)), f.client.read(limit, slave))
    }

    @Test
    fun aTargetThatIsNotABoardAddressIsRejected() = runTest {
        val f = FakeConfigBoards(this)
        assertThrows<IllegalArgumentException> { f.client.read(limit, 0xFF) } // broadcast
        assertThrows<IllegalArgumentException> { f.client.write(limit, Value.U32(1), 0x80) } // a guest
        assertTrue(f.requests.isEmpty())
    }
}

/**
 * The far side of the link for [ConfigClientTest]: every board the session can address, answering
 * `CONFIG_READ` / `CONFIG_WRITE` the way `crates/net/src/walk.rs` does (a write echoes the stored
 * value), with knobs to make it refuse, disagree, drop or stay silent. It also plays the session's
 * single driver (the app's `L3Session` service loop): a background coroutine pumps the engine and
 * carries bytes both ways, which the client relies on and never does itself.
 *
 * The board side is a real [Link] over a [BleStreamTransport], so requests cross the same
 * SOF/len/CRC framing the CC2541 bridge carries.
 */
@OptIn(ExperimentalCoroutinesApi::class)
internal class FakeConfigBoards(scope: TestScope) {
    val engine = BleWalkEngine(attachOnly = true, nowMs = { scope.testScheduler.currentTime })
    private val lock = Any()
    private val exchange = ConfigExchange(engine, lock)
    val client = ConfigClient(exchange)

    /** The tune lane's client, on the SAME exchange as [client] (one request in flight between them). */
    val tune = TuneClient(exchange)

    /** (board address, key) -> the RAM shadow the tune lane reads and writes; never [store]. */
    val shadow = HashMap<Pair<Int, Key>, Int>()

    private val wire = BleStreamTransport()
    private val link = Link(wire)

    /** (board address, key) -> stored value. */
    val store = HashMap<Pair<Int, Key>, Value>()

    /** Every CONFIG and TUNE request that reached the boards, re-sends included, in order. */
    val requests = mutableListOf<Pdu>()

    /** The status every CONFIG response carries (a non-OK one makes it a refusal). */
    var status = Walk.CFG_OK

    /** Echo this instead of the stored value on a write. */
    var echoOverride: Value? = null

    /** Answer a write's CFG_OK with no type tag or value. */
    var omitEcho = false

    /** Answer a read's CFG_OK with these bytes after the status instead of the tagged stored value. */
    var readBodyOverride: ByteArray? = null

    /** Never answer. */
    var silent = false

    /** Drop this many requests (lost on the link) before answering. */
    var dropNext = 0

    /** Hold requests unanswered while true; answer them once it is false. */
    var hold = false
    private val held = ArrayDeque<Pdu>()

    /** Responses delivered ahead of the next real answer. */
    var strayAhead: List<Pdu> = emptyList()

    init {
        attach()
        scope.backgroundScope.launch {
            while (isActive) {
                synchronized(lock) { turn() }
                delay(POLL_MS)
            }
        }
    }

    /** First contact, as the app does before any config: the board at 0x01 grants guest 0x80. */
    private fun attach() {
        engine.pump()
        engine.takeOutgoing()
        val hello = byteArrayOf(0x01, Walk.PROTO_VER.toByte(), 0, 0, 0, 0x80.toByte())
        link.send(Pdu.of(Opcode.NodeHello, 0x01, 0x80, hello).encode())
        engine.onReceive(wire.drainOutgoing()!!)
        engine.pump()
        check(engine.attached)
    }

    private fun turn() {
        engine.pump()
        engine.takeOutgoing()?.let { wire.onReceive(it) }
        while (true) {
            val pdu = link.pollRecv()?.let { Pdu.decodeOrNull(it) } ?: break
            val op = pdu.known()
            val tune = pdu.opcode == Walk.OP_TUNE_READ || pdu.opcode == Walk.OP_TUNE_WRITE
            if (op != Opcode.ConfigRead && op != Opcode.ConfigWrite && !tune) continue
            requests.add(pdu)
            if (silent) continue
            if (dropNext > 0) {
                dropNext--
                continue
            }
            held.addLast(pdu)
        }
        while (!hold && held.isNotEmpty()) {
            val req = held.removeFirst()
            for (stray in strayAhead) link.send(Pdu.of(Opcode.ConfigResp, stray.src, req.src, stray.payload).encode())
            strayAhead = emptyList()
            link.send(answer(req).encode())
        }
        wire.drainOutgoing()?.let { engine.onReceive(it) }
        engine.pump()
    }

    private fun answer(req: Pdu): Pdu {
        val p = req.payload
        val key = Key(p[0].toInt() and 0xFF, p[1].toInt() and 0xFF)
        val head = byteArrayOf(p[0], p[1], status.toByte())
        val body: ByteArray = when {
            status != Walk.CFG_OK -> byteArrayOf(0)
            req.opcode == Walk.OP_TUNE_WRITE -> {
                val v = Value.decode(Type.I16, p.copyOfRange(3, p.size)) as Value.I16
                if (!Gains.inRange(key.index, v.v)) {
                    return Pdu.of(Opcode.ConfigResp, req.dst, req.src, byteArrayOf(p[0], p[1], Walk.CFG_BAD.toByte(), 0))
                }
                shadow[req.dst to key] = v.v
                tagged(echoOverride ?: v)
            }
            req.opcode == Walk.OP_TUNE_READ -> readBodyOverride ?: shadow[req.dst to key]?.let { tagged(Value.I16(it)) }
                ?: return Pdu.of(Opcode.ConfigResp, req.dst, req.src, byteArrayOf(p[0], p[1], Walk.CFG_UNKNOWN_KEY.toByte(), 0))
            req.known() == Opcode.ConfigWrite -> {
                val type = Type.fromTag(p[2].toInt() and 0xFF)!!
                store[req.dst to key] = Value.decode(type, p.copyOfRange(3, p.size))!!
                if (omitEcho) ByteArray(0) else tagged(echoOverride ?: store[req.dst to key]!!)
            }
            else -> readBodyOverride ?: store[req.dst to key]?.let { tagged(it) } ?: byteArrayOf(0)
        }
        // The addressed board answers as itself, back to the requester.
        return Pdu.of(Opcode.ConfigResp, req.dst, req.src, head + body)
    }

    private fun tagged(v: Value) = byteArrayOf(v.kind().tag.toByte()) + v.encode()

    private companion object {
        const val POLL_MS = 20L
    }
}
