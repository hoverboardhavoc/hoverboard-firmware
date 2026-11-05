package com.hoverboardhavoc.remote.bench

import android.os.Bundle
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.test.junit4.createAndroidComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performTouchInput
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.hoverboardhavoc.remote.MainActivity
import com.hoverboardhavoc.remote.MainViewModel
import com.hoverboardhavoc.remote.UiState
import com.hoverboardhavoc.remote.model.BoardMode
import com.hoverboardhavoc.remote.model.ConnectionState
import com.hoverboardhavoc.remote.model.FaultLamp
import com.hoverboardhavoc.remote.model.SpeedReading
import com.hoverboardhavoc.remote.model.Throttle
import com.hoverboardhavoc.remote.model.machineSpeed
import com.hoverboardhavoc.remote.ui.components.ARM_TAG
import com.hoverboardhavoc.remote.ui.components.THROTTLE_TAG
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.koin.androidx.viewmodel.ext.android.getViewModel
import kotlin.math.roundToInt
import kotlin.math.sign

/**
 * The bench smoke test's drive (`specs/bench-ci.md` section 5): the real app, with the real
 * `BleHoverboardTransport`, connects to the pair's master bridge by name, waits for every board,
 * arms, drives forward and then in reverse through the throttle pad within section 6's gentle
 * limits, and disarms.
 *
 * It fails on what the app shows: a board missing from the session, a wheel whose speed reads with
 * the wrong sign or not at all, a lamp that refuses the arm, and a disarm it did not ask for. What
 * the halls and the current did is judged on the bench host from the CLI's recording, not here.
 *
 * The schedule is reported as instrumentation status lines (`bench_phase=<name>`) as each phase
 * starts, so the bench host can stamp them on its own clock and line the recording up with them.
 *
 * Arguments (`am instrument -e <name> <value>`), each with the default the bench uses:
 * `device` the master bridge's advertised name (`offroad-master`), `boards` the motor controllers
 * the session must find (2), `throttle` the steady demand in the drive command's 32,767 scale
 * (2000), `ramp_ms` (2000), `steady_ms` (3000) and `coast_ms` (4000).
 */
@RunWith(AndroidJUnit4::class)
class BenchDriveTest {

    @get:Rule
    val compose = createAndroidComposeRule<MainActivity>()

    private val args = BenchArgs.from(InstrumentationRegistry.getArguments())

    private lateinit var vm: MainViewModel

    private val state: UiState get() = vm.uiState.value

    @Test
    fun drivesEachDirectionGently() {
        vm = compose.activity.getViewModel()
        // The throttle pad, not the joystick: BOUND drives both wheels with the one demand.
        vm.setTurnAxis(false)
        vm.setDeviceName(args.device)
        phase("connect")
        compose.runOnUiThread { vm.connect() }
        waitFor("connected to ${args.device}", CONNECT_MS) { state.isConnected }
        waitFor("${args.boards} motor controllers reporting", BOARDS_MS) {
            state.motorControllers.size >= args.boards && state.reporting.size >= args.boards
        }
        waitFor("the arm control to accept a press", CAN_ARM_MS) { state.canArm }
        try {
            phase("arm")
            compose.onNodeWithTag(ARM_TAG).performClick()
            waitFor("every board in RUN", ARM_MS) {
                state.armed && boards().all { state.telemetryOf(it)?.mode == BoardMode.RUN }
            }
            drive(+1)
            phase("coast")
            hold(args.coastMs)
            drive(-1)
            phase("disarm")
            compose.onNodeWithTag(ARM_TAG).performClick()
            waitFor("the app disarmed", ARM_MS) { !state.armed }
            waitFor("every board out of RUN", ARM_MS) {
                boards().all { state.telemetryOf(it)?.mode != BoardMode.RUN }
            }
            phase("done")
        } finally {
            compose.runOnUiThread {
                vm.onThrottleRelease()
                vm.disconnect()
            }
        }
    }

    /**
     * One direction: ramp the pad from zero to the throttle, hold it, ramp back to zero, then lift.
     * The wheels are checked on every tick throughout, and must read the commanded sign at the end
     * of the steady hold.
     */
    private fun drive(dir: Int) {
        val name = if (dir > 0) "forward" else "reverse"
        val pad = compose.onNodeWithTag(THROTTLE_TAG)
        val size = pad.fetchSemanticsNode().size
        val x = size.width / 2f
        val h = size.height.toFloat()
        fun y(demand: Int): Float = h * Throttle.ZERO_FRACTION * (1f - demand.toFloat() / Throttle.MAX_SPEED)
        // The ramps follow the clock rather than a step count: an injected move and its check take
        // longer than a tick, so a fixed number of steps would stretch the ramp.
        fun ramp(from: Float, to: Float) {
            val start = System.currentTimeMillis()
            do {
                val f = ((System.currentTimeMillis() - start).toFloat() / args.rampMs).coerceAtMost(1f)
                val demand = dir * (args.throttle * (from + (to - from) * f)).roundToInt()
                pad.performTouchInput { moveTo(Offset(x, y(demand))) }
                tick(dir, steady = false)
            } while (f < 1f)
        }

        phase("${name}_ramp_up")
        pad.performTouchInput { down(Offset(x, y(0))) }
        ramp(0f, 1f)
        phase("${name}_steady")
        hold(args.steadyMs) { tick(dir, steady = true) }
        boards().forEach { a ->
            val s = wheelSign(a) ?: failRun("$name: board $a shows no speed at the end of the hold")
            if (s != dir) failRun("$name: board $a shows speed of sign $s at the end of the hold")
        }
        phase("${name}_ramp_down")
        ramp(1f, 0f)
        pad.performTouchInput { up() }
        compose.waitForIdle()
    }

    /** Hold for [ms], running [each] once a tick (the safety checks by default). */
    private fun hold(ms: Long, each: () -> Unit = { tick(0, steady = false) }) {
        val end = System.currentTimeMillis() + ms
        while (System.currentTimeMillis() < end) each()
    }

    /** One tick of the drive: sleep, then check what the app shows. */
    private fun tick(dir: Int, steady: Boolean) {
        Thread.sleep(TICK_MS)
        val st = state
        if (!st.armed) failRun("the app disarmed without being asked (self-disarm ${st.selfDisarmed}, fault ${st.faultDisarm})")
        st.faultDisarm?.let { failRun("a fault disarmed board ${it.address}: ${it.lamp}") }
        val missing = boards().filter { it !in st.reporting }
        if (missing.isNotEmpty()) failRun("board(s) $missing stopped reporting")
        st.lampWords.forEach { (a, w) ->
            val lit = FaultLamp.entries.filter { it.bit and w and FaultLamp.REFUSING_BITS != 0 }
            if (lit.isNotEmpty()) failRun("board $a shows the lamp(s) $lit")
        }
        boards().forEach { a ->
            if (st.telemetryOf(a)?.mode != BoardMode.RUN) failRun("board $a left RUN while armed")
        }
        if (steady) {
            boards().forEach { a ->
                val s = wheelSign(a)
                if (s != null && s != 0 && s != dir) failRun("board $a shows speed of sign $s while driving $dir")
            }
        }
    }

    /**
     * The sign of board [address]'s wheel speed as the app shows it, or null when it shows none: the
     * figure the panel prints for that wheel alone, so a residual rate that prints as 0.0 is zero.
     */
    private fun wheelSign(address: Int): Int? {
        val facts = state.speedFacts.filter { it.address == address && it.motor == 0 }
        val reading = machineSpeed(facts) as? SpeedReading.Kmh ?: return null
        val shown = when (val s = reading.shown) {
            is SpeedReading.Shown.Tenths -> s.kmh
            is SpeedReading.Shown.Whole -> s.kmh.toDouble()
        }
        return shown.sign.toInt()
    }

    private fun failRun(message: String): Nothing = throw AssertionError(message)

    private fun boards(): List<Int> = state.motorControllers.map { it.address }

    private fun waitFor(what: String, ms: Long, ok: () -> Boolean) {
        val end = System.currentTimeMillis() + ms
        while (!ok()) {
            if (System.currentTimeMillis() > end) {
                failRun("timed out after $ms ms waiting for $what (connection ${state.connectionState}, " +
                    "boards ${boards()}, reporting ${state.reporting}, armed ${state.armed}, " +
                    "lamps ${state.lampWords}, arm statement ${state.machineArm})")
            }
            Thread.sleep(TICK_MS)
        }
    }

    /** Report the phase that starts now, for the bench host to stamp. */
    private fun phase(name: String) {
        InstrumentationRegistry.getInstrumentation().sendStatus(
            STATUS_PHASE,
            Bundle().apply { putString("bench_phase", name) },
        )
    }

    private companion object {
        const val TICK_MS = 50L
        const val CONNECT_MS = 60_000L
        const val BOARDS_MS = 30_000L
        const val CAN_ARM_MS = 20_000L
        const val ARM_MS = 10_000L

        /** A status code the test runner itself does not use. */
        const val STATUS_PHASE = 99
    }
}

/** The test's arguments, as `am instrument -e` gives them. */
internal data class BenchArgs(
    val device: String,
    val boards: Int,
    val throttle: Int,
    val rampMs: Long,
    val steadyMs: Long,
    val coastMs: Long,
) {
    companion object {
        fun from(b: Bundle) = BenchArgs(
            device = b.getString("device") ?: "offroad-master",
            boards = b.getString("boards")?.toInt() ?: 2,
            throttle = b.getString("throttle")?.toInt() ?: 2000,
            rampMs = b.getString("ramp_ms")?.toLong() ?: 2000L,
            steadyMs = b.getString("steady_ms")?.toLong() ?: 3000L,
            coastMs = b.getString("coast_ms")?.toLong() ?: 4000L,
        )
    }
}
