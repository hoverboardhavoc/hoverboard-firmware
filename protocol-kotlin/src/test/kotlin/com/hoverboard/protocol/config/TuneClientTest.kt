package com.hoverboard.protocol.config

import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

/**
 * [TuneClient] through a real [BleWalkEngine] against [FakeConfigBoards]' RAM shadow: the tune
 * lane's contract (`specs/rider-ui.md` section 4) and the one-in-flight slot it shares with
 * [ConfigClient].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneClientTest {

    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val master = 0x01
    private val slave = 0x02

    @Test
    fun aWriteStagesTheShadowUnderTheTuneOpcodeAndNeverTheStore() = runTest {
        val f = FakeConfigBoards(this)

        assertEquals(TuneVerified(6100), f.tune.write(kpA, 6100, slave))
        assertEquals(6100, f.shadow[slave to kpA])
        assertTrue(f.store.isEmpty(), "a tune write reached the store")
        assertEquals(listOf(Walk.OP_TUNE_WRITE), f.requests.map { it.opcode })
    }

    @Test
    fun aReadReturnsTheStagedValueNotTheStoredOne() = runTest {
        val f = FakeConfigBoards(this)
        f.shadow[master to kpA] = 7000
        f.store[master to kpA] = Value.I16(6000)

        assertEquals(StagedValue(7000), f.tune.read(kpA, master))
        assertEquals(ReadValue(Value.I16(6000)), f.client.read(kpA, master))
        assertEquals(listOf(Walk.OP_TUNE_READ, 0x30), f.requests.map { it.opcode })
    }

    @Test
    fun aValueOutsideTheSeamsRangeIsRefusedBad() = runTest {
        val f = FakeConfigBoards(this)

        assertEquals(Refused(CfgRefusal.BAD), f.tune.write(kpA, Gains.DEFAULT_MAX[Gains.KP] + 1, master))
        assertTrue(f.shadow.isEmpty())
    }

    @Test
    fun aKeyOffTheAllowlistIsRefusedUnknown() = runTest {
        val f = FakeConfigBoards(this)
        assertEquals(Refused(CfgRefusal.UNKNOWN_KEY), f.tune.read(Key(0x20, 0), master))
    }

    @Test
    fun anEchoThatDisagreesIsAMismatchAndANonI16EchoIsMalformed() = runTest {
        val f = FakeConfigBoards(this)
        f.echoOverride = Value.I16(42)
        assertEquals(TuneMismatch(wrote = 6100, staged = 42), f.tune.write(kpA, 6100, master))

        f.shadow[master to kpA] = 1
        f.readBodyOverride = byteArrayOf(Type.U32.tag.toByte(), 1, 0, 0, 0)
        assertTrue(f.tune.read(kpA, master) is Malformed)
    }

    @Test
    fun aSilentBoardTimesOutAfterTheWholeRetransmitBudget() = runTest {
        // A board whose firmware predates the lane does not serve the opcodes at all.
        val f = FakeConfigBoards(this)
        f.silent = true

        assertEquals(TimedOut, f.tune.write(kpA, 6100, master))
        assertEquals(1 + BleWalkEngine.MAX_RETRANSMITS, f.requests.size)
    }

    @Test
    fun theTuneAndConfigLanesShareOneRequestInFlight() = runTest {
        // Both lanes answer with CONFIG_RESP, so a CONFIG_READ and a TUNE_READ of one key in flight
        // together could each take the other's answer. The shared exchange refuses the second.
        val f = FakeConfigBoards(this)
        f.hold = true
        f.store[master to kpA] = Value.I16(6000)

        val first = async { f.client.read(kpA, master) }
        advanceTimeBy(ONE_TURN_MS)
        assertEquals(Busy, f.tune.read(kpA, master))
        assertEquals(Busy, f.tune.write(kpA, 6100, master))
        advanceTimeBy(ONE_TURN_MS)
        assertEquals(1, f.requests.size, "a refused tune operation put something on the wire")

        f.hold = false
        assertEquals(ReadValue(Value.I16(6000)), first.await())
        assertEquals(TuneVerified(6100), f.tune.write(kpA, 6100, master))
    }

    @Test
    fun aTargetThatIsNotABoardAddressIsRejected() = runTest {
        val f = FakeConfigBoards(this)
        assertThrows<IllegalArgumentException> { f.tune.write(kpA, 1, 0xFF) }
        assertTrue(f.requests.isEmpty())
    }

    private companion object {
        const val ONE_TURN_MS = 50L
    }
}
