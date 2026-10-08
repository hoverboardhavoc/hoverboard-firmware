package com.hoverboard.protocol.config

import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * The [ConfigExchange]'s settle window: after a request was re-sent, the re-send's reply (a
 * byte-identical `CONFIG_RESP` whichever lane asked) can arrive after the exchange was answered, and
 * it must not be taken as the answer to the next request of the same key.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class ConfigExchangeSettleTest {

    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val bkA = Gains.key(Gains.CONTROL_GAIN_A, Gains.BK)
    private val master = 0x01

    /** A `TUNE_READ` of [kpA] whose first reply is slow enough to be re-sent; the original is then answered. */
    private suspend fun TestScope.retransmittedTuneRead(f: FakeConfigBoards) {
        f.hold = true
        val first = async { f.tune.read(kpA, master) }
        advanceTimeBy(BleWalkEngine.DEFAULT_REPLY_TIMEOUT_MS + 2 * ONE_TURN_MS)
        assertEquals(2, f.requests.size, "the slow request was not re-sent")
        f.release = 1 // the original's reply; the re-send's stays held
        assertEquals(StagedValue(7000), first.await())
    }

    @Test
    fun aLateDuplicateOfARetransmittedReplyDoesNotAnswerTheNextRequestOfTheSameKey() = runTest {
        val f = FakeConfigBoards(this)
        f.shadow[master to kpA] = 7000
        f.store[master to kpA] = Value.I16(6000)
        retransmittedTuneRead(f)

        // The Tune model's pass reads the flash value of the same key next.
        val second = async { f.client.read(kpA, master) }
        advanceTimeBy(ONE_TURN_MS)
        f.release = 1 // the re-send's reply, the staged 7000, now arrives
        advanceTimeBy(2 * ONE_TURN_MS)
        f.hold = false

        assertEquals(ReadValue(Value.I16(6000)), second.await(), "the late duplicate was taken as the flash value")
        assertEquals(Walk.OP_TUNE_READ, f.requests[1].opcode)
        assertEquals(Opcode.ConfigRead.value, f.requests.last().opcode)
    }

    @Test
    fun aRequestOfAnotherKeyIsNotHeldBack() = runTest {
        val f = FakeConfigBoards(this)
        f.shadow[master to kpA] = 7000
        f.store[master to bkA] = Value.I16(500)
        retransmittedTuneRead(f)

        f.hold = false
        val next = async { f.client.read(bkA, master) }
        advanceTimeBy(2 * ONE_TURN_MS)
        assertEquals(3, f.requests.size, "a request of another key waited out the settle window")
        assertEquals(ReadValue(Value.I16(500)), next.await())
    }

    private companion object {
        const val ONE_TURN_MS = 50L
    }
}
