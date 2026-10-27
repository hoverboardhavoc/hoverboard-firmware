package com.hoverboard.remote

import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.INPUTS_TIMEOUT_TICKS
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.CommandPump
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.ble.TickFrames
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveFallback
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.DriveMode
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/**
 * The bench report of 2026-10-27 (`specs/agent-runs/cloud-app-slave-flap-prompt.md`, "What was
 * seen on the bench"): on the classywalk offroad pair, both boards storing `machine.drive_mode`
 * BOUND, the slave (0x02) armed and shut down 1,136 times in about 40 minutes while the app was
 * up, and the ride screen sat in the UNREAD fallback because the slave's copy of 0x78 had not been
 * read.
 *
 * The suspected app path: a failed config read deletes the stored value (`SetupModel.record`), so
 * on a lossy slave link the drive mode in force flips between BOUND and the UNREAD fallback
 * (SINGLE), each flip re-sends the frame (`MainViewModel`'s `combine(setup.state, ...)` collector),
 * and a board that leaves the frame is owed DISARMED (`CommandPump`). This suite runs exactly that:
 * the real [MainViewModel] and [SetupModel] over the fake transport, with the read of 0x78 from 0x02
 * alternating between a timeout and BOUND, every frame handed to a real [CommandPump] the way
 * `BleHoverboardTransport.sendCommand` hands it, and every tick addressed to 0x02 recorded with the
 * virtual time it went out.
 *
 * What drives the re-reads is a change of the slave's boot tag in its `CYCLIC_STATE`, which is the
 * one path that re-reads the ride facts (0x78 among them) mid-session with Setup closed
 * (`SetupModel.onRestarted`). It is only the cheapest way to make the app read 0x78 again; the
 * question is what the flip does to the wire, not what caused the re-read.
 *
 * The answer the three cases pin: with the app armed the arm latches its targets, so the flip
 * never reaches the frame and 0x02's power level never toggles; disarmed, 0x02 enters and leaves
 * the frame but is only ever told DISARMED; armed from the UNREAD fallback, 0x02 is not addressed
 * at all. The app does not toggle 0x02's power bit in any of them.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class AppSlaveFlapTest {

    private val dispatcher = StandardTestDispatcher()
    private lateinit var fake: FakeHoverboardTransport
    private lateinit var transport: LossySlaveReads
    private lateinit var viewModel: MainViewModel

    /** One tick of the pump that addressed [SLAVE]: when, which payloads, and the power bit it carried. */
    private data class SlaveTick(val atMs: Long, val frames: TickFrames, val power: Boolean) {
        val inputs: Boolean get() = frames != TickFrames.DRIVE_ONLY
        val drive: Boolean get() = frames != TickFrames.INPUTS_ONLY
    }

    private val slaveTicks = mutableListOf<SlaveTick>()

    /** The drive mode in force and its fallback, once per cycle, as the ride screen would state it. */
    private val driveSeen = mutableListOf<Pair<DriveMode, DriveFallback?>>()

    /**
     * The fake transport with the slave's 0x78 read made lossy, and every frame also handed to a
     * real [CommandPump], which is what `BleHoverboardTransport.sendCommand` does with it.
     *
     * [fails] answers whether the [n]th read of 0x78 from [SLAVE] (from 0) times out.
     */
    private class LossySlaveReads(
        private val fake: FakeHoverboardTransport,
        private val fails: (n: Int) -> Boolean,
    ) : HoverboardTransport by fake {
        var pump: CommandPump? = null
        var slaveModeReads = 0
            private set

        override suspend fun readConfig(key: Key, target: Int): ConfigReadResult? {
            if (target == SLAVE && key == Fields.MACHINE_DRIVE_MODE.key()) {
                if (fails(slaveModeReads++)) return TimedOut
            }
            return fake.readConfig(key, target)
        }

        override fun sendCommand(frame: DriveFrame) {
            fake.sendCommand(frame)
            pump?.set(frame)
        }
    }

    @BeforeEach
    fun setUp() {
        Dispatchers.setMain(dispatcher)
    }

    @AfterEach
    fun tearDown() {
        Dispatchers.resetMain()
    }

    /** Both boards store BOUND; [fails] decides which of the slave's 0x78 reads time out. */
    private fun TestScope.connect(fails: (n: Int) -> Boolean) {
        fake = FakeHoverboardTransport(dispatcher.scheduler)
        for (b in listOf(MASTER, SLAVE)) {
            fake.store[b to Fields.MACHINE_DRIVE_MODE.key()] = Value.U8(DriveMode.BOUND.byte)
        }
        transport = LossySlaveReads(fake, fails)
        viewModel = MainViewModel(transport, LinkSettings(FakeSharedPreferences()), FakeBoardScanner())
        val pump = CommandPump(backgroundScope, LinkConfig.SEND_INTERVAL_MS) { ticks ->
            for (t in ticks) {
                if (t.node != SLAVE) continue
                slaveTicks += SlaveTick(testScheduler.currentTime, t.frames, t.command.inputs.powerRequest())
            }
        }
        transport.pump = pump
        fake.setConnectionState(ConnectionState.CONNECTED)
        fake.setAttachedBoard(MASTER, SLAVE)
        pump.start()
        // The first boot tag each board reports is an observation, not a restart.
        fake.emitCyclicState(cyclic(bootTag = 1), from = MASTER)
        fake.emitCyclicState(cyclic(bootTag = 1), from = SLAVE)
        settle()
    }

    /**
     * [n] cycles of the lossy link: the slave reports a new boot tag, the app re-reads the ride
     * facts (0x78 from 0x02 among them, failing or not by the connect's pattern), and a second passes.
     */
    private fun TestScope.flap(n: Int) {
        repeat(n) { i ->
            fake.emitCyclicState(cyclic(bootTag = 2 + i % 200), from = SLAVE)
            settle()
            driveSeen += drive().let { it.mode to it.fallback }
        }
    }

    private fun TestScope.settle() {
        advanceTimeBy(CYCLE_MS)
        runCurrent()
    }

    private fun drive() = viewModel.setup.state.value.driveInForce(fake.telemetry.value)

    /** Whether the frame the app last handed the pump arms [node]. */
    private fun armsLast(node: Int): Boolean = fake.frames.last().commands[node]?.armed == true

    /** The number of times the power bit changed across the `INPUTS` sent to [SLAVE] from [fromMs]. */
    private fun powerToggles(fromMs: Long = 0): Int =
        slaveTicks.filter { it.inputs && it.atMs >= fromMs }.map { it.power }.zipWithNext().count { (a, b) -> a != b }

    /** The longest stretch with no payload at all addressed to [SLAVE], between [fromMs] and [toMs]. */
    private fun longestSilenceMs(fromMs: Long, toMs: Long): Long {
        val times = listOf(fromMs) + slaveTicks.map { it.atMs }.filter { it in fromMs..toMs } + listOf(toMs)
        return times.zipWithNext().maxOf { (a, b) -> b - a }
    }

    /** The flip under test really happened: the slave's 0x78 was re-read and the mode in force moved both ways. */
    private fun assertTheModeFlapped() {
        val reads = transport.slaveModeReads
        assertTrue(reads >= FLAPS, "0x78 was re-read from 0x02 each cycle: $reads")
        assertTrue(DriveMode.BOUND to null in driveSeen, "BOUND was in force on some cycles: $driveSeen")
        assertTrue(DriveMode.SINGLE to DriveFallback.UNREAD in driveSeen, "and UNREAD on others: $driveSeen")
        val flips = driveSeen.zipWithNext().count { (a, b) -> a != b }
        assertTrue(flips >= FLAPS - 1, "the mode in force flipped on every cycle: $flips")
    }

    /** One line per case for the run log: what went to 0x02, counted from [fromMs] (the arm, where there is one). */
    private fun report(case: String, fromMs: Long = 0) {
        val ticks = slaveTicks.filter { it.atMs >= fromMs }
        val inputs = ticks.filter { it.inputs }
        println(
            "$case: 0x78 reads from 0x02=${transport.slaveModeReads}, " +
                "mode flips=${driveSeen.zipWithNext().count { (a, b) -> a != b }}, " +
                "from t=$fromMs ms: ticks to 0x02=${ticks.size}, INPUTS to 0x02=${inputs.size} " +
                "(power=1: ${inputs.count { it.power }}, power=0: ${inputs.count { !it.power }}), " +
                "DRIVE_CMD to 0x02=${ticks.count { it.drive }}, power toggles=${powerToggles(fromMs)}",
        )
    }

    @Test
    fun `armed in BOUND, a flapping 0x78 read never toggles the slave's power bit`() = runTest(dispatcher) {
        // The first read answers BOUND, so the arm is taken with both boards in force; after that
        // every other read times out.
        connect { n -> n % 2 == 1 }
        assertEquals(DriveMode.BOUND, drive().mode)
        viewModel.onArmToggle()
        runCurrent()
        assertTrue(armsLast(SLAVE) && armsLast(MASTER), "the arm addresses both boards")
        val armedAt = testScheduler.currentTime

        flap(FLAPS)
        val flappedUntil = testScheduler.currentTime

        report("armed in BOUND", fromMs = armedAt)
        assertTheModeFlapped()
        assertTrue(armsLast(SLAVE), "the latched arm still addresses 0x02 at the end")
        val armedInputs = slaveTicks.filter { it.inputs && it.atMs >= armedAt }
        assertTrue(armedInputs.size >= FLAPS, "the keepalive kept stating the level: ${armedInputs.size}")
        assertTrue(armedInputs.all { it.power }, "every INPUTS to 0x02 while armed carried power=1")
        assertEquals(0, powerToggles(fromMs = armedAt), "the power bit to 0x02 never toggled while armed")
        // The board holds its mirror on any INPUTS or DRIVE_CMD from its owner within 1.5 s
        // (`orchestrator::LinkInbox::refreshes_mirror`). The app leaves no gap anywhere near that:
        // a DRIVE_CMD goes to 0x02 on every tick, so only the hop can starve the mirror.
        assertTrue(slaveTicks.filter { it.atMs >= armedAt }.all { it.drive }, "a DRIVE_CMD to 0x02 on every armed tick")
        assertEquals(LinkConfig.SEND_INTERVAL_MS, longestSilenceMs(armedAt, flappedUntil))
        assertTrue(LinkConfig.SEND_INTERVAL_MS < INPUTS_TIMEOUT_TICKS * TICK_MS)

        // And the disarm still reaches it: a DISARMED burst, then nothing armed.
        viewModel.onArmToggle()
        val disarmedAt = testScheduler.currentTime
        settle()
        val after = slaveTicks.filter { it.inputs && it.atMs >= disarmedAt }
        assertTrue(after.size >= LinkConfig.INPUTS_CHANGE_REPEATS, "the disarm burst reached 0x02: ${after.size}")
        assertTrue(after.none { it.power }, "and carried power=0")
    }

    @Test
    fun `disarmed, a flapping 0x78 read moves 0x02 in and out of the frame but never arms it`() = runTest(dispatcher) {
        connect { n -> n % 2 == 1 }
        flap(FLAPS)

        report("disarmed")
        assertTheModeFlapped()
        assertTrue(slaveTicks.isNotEmpty(), "0x02 was addressed on the BOUND cycles")
        // It leaves the frame on each UNREAD cycle and is re-entered on each BOUND one, so it sees
        // stretches with no payload at all: the frame really is moving.
        val ticksEnd = testScheduler.currentTime
        assertTrue(longestSilenceMs(0, ticksEnd) > LinkConfig.SEND_INTERVAL_MS, "0x02 left the frame on some cycles")
        assertTrue(slaveTicks.filter { it.inputs }.none { it.power }, "no INPUTS to 0x02 ever carried power=1")
        assertEquals(0, powerToggles())
    }

    @Test
    fun `armed from the UNREAD fallback, 0x02 is never addressed while armed`() = runTest(dispatcher) {
        // The first read times out, so the arm is taken in the UNREAD fallback (SINGLE on the
        // attached board); after that the reads alternate as before.
        connect { n -> n % 2 == 0 }
        assertEquals(DriveMode.SINGLE to DriveFallback.UNREAD, drive().let { it.mode to it.fallback })
        viewModel.onArmToggle()
        runCurrent()
        assertTrue(armsLast(MASTER), "the arm addresses the attached board")
        assertEquals(setOf(MASTER), fake.frames.last().commands.keys, "and nothing else")
        val armedAt = testScheduler.currentTime

        flap(FLAPS)

        report("armed from UNREAD", fromMs = armedAt)
        assertTheModeFlapped()
        assertEquals(setOf(MASTER), fake.frames.last().commands.keys, "the latched arm never takes 0x02 in")
        assertTrue(slaveTicks.none { it.atMs >= armedAt }, "nothing at all went to 0x02 while armed")
        assertTrue(slaveTicks.none { it.power }, "and nothing ever carried power=1 to it")
    }

    private companion object {
        const val MASTER = 0x01
        const val SLAVE = 0x02

        /** Re-read cycles per case, each [CYCLE_MS] of virtual time. */
        const val FLAPS = 40

        /** One cycle: a re-read and a second of pump ticks, about the bench's flap period. */
        const val CYCLE_MS = 1_000L

        /** 250 Hz scheduler tick, `crates/scheduler/src/lib.rs` (`TICK_HZ`, `TICK_MS`). */
        const val TICK_MS = 4

        fun cyclic(bootTag: Int) = CyclicState(0, 0, 0, 3_600, 0, 0, 0, 0, 0, QUIET_OBS.copy(bootTag = bootTag))
    }
}
