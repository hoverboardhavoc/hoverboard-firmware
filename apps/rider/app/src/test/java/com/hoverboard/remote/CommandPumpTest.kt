package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.DRIVE_TIMEOUT_TICKS
import com.hoverboard.remote.ble.CommandPump
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.model.RiderCommand
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.delay
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [CommandPump] is the serialised, rate-limited [RiderCommand] sender. These tests pin the
 * properties that fix the per-event-coroutine crash (conflation, one write at a time, resilience to
 * a failing write) and the one that keeps a demand alive at all: continuous re-send inside the
 * firmware's 200 ms `DRIVE_CMD` decay window.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class CommandPumpTest {

    @Test
    fun `streams only the latest command, conflating rapid updates`() = runTest {
        val writes = mutableListOf<RiderCommand>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        runCurrent() // first tick sends the initial DISARMED

        // Three updates inside one interval, only the last must reach the wire.
        pump.set(RiderCommand.armed(100))
        pump.set(RiderCommand.armed(200))
        pump.set(RiderCommand.armed(300))
        advanceTimeBy(INTERVAL + 1)
        runCurrent()

        assertEquals(RiderCommand.armed(300), writes.last())
        assertTrue(writes.none { it.demand == 100 || it.demand == 200 })
    }

    @Test
    fun `re-sends a held command every tick so the demand does not decay`() = runTest {
        val writes = mutableListOf<RiderCommand>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        pump.set(RiderCommand.armed(150))
        advanceTimeBy(INTERVAL * 3 + 1)
        runCurrent()

        // A stationary finger must keep streaming: DRIVE_CMD decays after 200 ms of silence
        // (crates/linkctl/src/lib.rs:57), so a single send is a blip rather than a demand.
        assertTrue(writes.count { it == RiderCommand.armed(150) } >= 3)
    }

    @Test
    fun `the shipped cadence fits inside the firmware decay window with room for a lost frame`() {
        // DRIVE_TIMEOUT_TICKS at the scheduler's 250 Hz (crates/scheduler/src/lib.rs:50).
        val decayMs = DRIVE_TIMEOUT_TICKS * TICK_MS
        assertEquals(200, decayMs)
        assertTrue(
            LinkConfig.SEND_INTERVAL_MS * 2 <= decayMs,
            "one lost frame must still leave the next one arriving inside the window",
        )
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
        pump.set(RiderCommand.armed(100))
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
    fun `a restarted pump starts disarmed, never resuming a held arm level`() = runTest {
        val writes = mutableListOf<RiderCommand>()
        val pump = CommandPump(backgroundScope, INTERVAL) { writes.add(it) }
        pump.start()
        pump.set(RiderCommand.armed(300))
        advanceTimeBy(INTERVAL + 1)
        pump.stop()

        // A reconnect starts a new session against this held value. It must not be an arm level
        // the rider is no longer being asked to confirm.
        pump.start()
        runCurrent()
        assertEquals(RiderCommand.DISARMED, writes.last())
    }

    private companion object {
        const val INTERVAL = 33L

        /** 250 Hz scheduler tick, `crates/scheduler/src/lib.rs:50,66`. */
        const val TICK_MS = 4
    }

    private class TransientBleError : Exception()
}
