package com.hoverboard.remote

import com.hoverboard.remote.ble.CommandPump
import com.hoverboard.protocol.linkctl.Inputs
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.delay
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [CommandPump] is the serialised, rate-limited [Inputs] sender. These tests pin the properties
 * that fix the per-event-coroutine crash: conflation, continuous re-send (feeding the board's
 * no-command timeout), one write at a time, and resilience to a failing write.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class CommandPumpTest {

    private fun inputs(throttle: Int, rider: Int = 1) =
        Inputs(throttle = throttle, buttons = 0, rider = rider)

    @Test
    fun `streams only the latest inputs, conflating rapid updates`() = runTest {
        val writes = mutableListOf<Inputs>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        runCurrent() // first tick sends the initial STOP

        // Three updates inside one interval — only the last must reach the wire.
        pump.set(inputs(100))
        pump.set(inputs(200))
        pump.set(inputs(300))
        advanceTimeBy(INTERVAL + 1)
        runCurrent()

        assertEquals(inputs(300), writes.last())
        assertTrue(writes.none { it == inputs(100) || it == inputs(200) })
    }

    @Test
    fun `re-sends a held inputs every tick so the link stays fed`() = runTest {
        val writes = mutableListOf<Inputs>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        pump.set(inputs(150))
        advanceTimeBy(INTERVAL * 3 + 1)
        runCurrent()

        // A stationary finger must keep streaming, or the board's no-command timeout stops it.
        assertTrue(writes.count { it == inputs(150) } >= 3)
    }

    @Test
    fun `writes never overlap even when a write is slower than the interval`() = runTest {
        var inFlight = 0
        var maxConcurrent = 0
        val pump = CommandPump(backgroundScope, INTERVAL) {
            inFlight++
            maxConcurrent = maxOf(maxConcurrent, inFlight)
            delay(INTERVAL * 5) // a write that takes far longer than one tick
            inFlight--
        }
        pump.start()
        pump.set(inputs(100))
        advanceTimeBy(INTERVAL * 20)
        runCurrent()

        assertEquals(1, maxConcurrent)
    }

    @Test
    fun `a failing write does not stop the pump`() = runTest {
        var calls = 0
        val pump = CommandPump(backgroundScope, INTERVAL) {
            calls++
            if (calls == 1) throw TransientBleError() // mimics a Nordic BLE write failure
        }
        pump.start()
        advanceTimeBy(INTERVAL * 3 + 1)
        runCurrent()

        assertTrue(calls >= 3) // kept ticking after the exception
    }

    @Test
    fun `stop resets the held inputs to the deadman STOP`() = runTest {
        val writes = mutableListOf<Inputs>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        pump.set(inputs(300))
        advanceTimeBy(INTERVAL + 1)
        pump.stop()

        pump.start()
        runCurrent()
        assertEquals(CommandPump.STOP, writes.last())
    }

    private companion object {
        const val INTERVAL = 33L
    }

    private class TransientBleError : Exception()
}
