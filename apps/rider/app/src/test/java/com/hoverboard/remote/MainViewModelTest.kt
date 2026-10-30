package com.hoverboard.remote

import androidx.lifecycle.ViewModelStore
import app.cash.turbine.test
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** What the arm control, the throttle, the release paths and the link do to the ViewModel's state. */
@OptIn(ExperimentalCoroutinesApi::class)
class MainViewModelTest : RiderVmTest() {

    /** Connect, and arm by pressing and holding the arm control. */
    private fun connectAndArm() {
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        dispatcher.scheduler.runCurrent()
        viewModel.onArmToggle()
    }

    // --- arming ---------------------------------------------------------------------------------

    /** `specs/control.md` (i): the arm control learns the board's rider requirement on attach. */
    @Test
    fun `a board running with the rider requirement waived says so in the ui state`() = runTest(dispatcher) {
        transport.store[0x01 to Fields.CONTROL_RIDER_REQUIRED.key()] = Value.U8(0)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        assertEquals(RiderWaiver.WAIVED, currentState().riderWaiver)

        transport.setAttachedBoard(null)
        assertEquals(RiderWaiver.NONE, currentState().riderWaiver)
    }

    /**
     * `specs/rider-ui.md` 3.7a: the arm control refuses while the two boards are known to hold
     * different frames, because one wheel would balance and the other coast and no board can see
     * its peer's store.
     */
    @Test
    fun `a pair holding two different frames refuses the arm and says which board holds what`() =
        runTest(dispatcher) {
            transport.store[0x01 to Fields.CONTROL_FRAME.key()] = Value.U8(Fields.ControlFrame.RIGID)
            transport.store[0x02 to Fields.CONTROL_FRAME.key()] = Value.U8(Fields.ControlFrame.PIVOTING)
            transport.setConnectionState(ConnectionState.CONNECTED)
            transport.setAttachedBoard(0x01, 0x02)

            val state = currentState()
            assertEquals(MachineArm.FramesDiffer(mapOf(0x01 to true, 0x02 to false)), state.machineArm)
            assertFalse(state.canArm, "the arm control refuses while the frames differ")

            viewModel.onArmToggle()
            advanceUntilIdle()
            assertFalse(currentState().armed, "the tap was refused by the model, not only by the control")
            assertTrue(transport.sent.none { it.armed })
        }

    /** The side is never a refusal: it warns, and the arm goes through. */
    @Test
    fun `a rigid pair with no right wheel warns but still arms`() = runTest(dispatcher) {
        for (board in listOf(0x01, 0x02)) {
            transport.store[board to Fields.CONTROL_FRAME.key()] = Value.U8(Fields.ControlFrame.RIGID)
            transport.store[board to Fields.CONTROL_SIDE.key()] = Value.U8(Fields.ControlSide.LEFT)
        }
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01, 0x02)
        reporting(0x01, 0x02)

        val state = currentState()
        assertEquals(MachineArm.SideNotOneRight(SidePair.NoneRight), state.machineArm)
        assertTrue(state.canArm)

        viewModel.onArmToggle()
        advanceUntilIdle()
        assertTrue(currentState().armed)
    }

    @Test
    fun `boots disarmed, at zero, disconnected`() = runTest(dispatcher) {
        val state = viewModel.uiState.first()
        assertFalse(state.armed)
        assertEquals(0, state.throttleSpeed)
        assertFalse(state.engaged)
        assertEquals(ConnectionState.DISCONNECTED, state.connectionState)
    }

    @Test
    fun `connecting does not arm`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        advanceUntilIdle()
        // Arming has to be an act, not a consequence of the link coming up.
        assertFalse(currentState().armed)
        assertTrue(transport.sent.none { it.armed })
    }

    @Test
    fun `arming asserts power and toggling off clears it`() = runTest(dispatcher) {
        connectAndArm()
        assertTrue(checkNotNull(transport.last).armed)
        assertTrue(checkNotNull(transport.last).inputs.powerRequest())

        viewModel.onArmToggle()
        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(checkNotNull(transport.last).inputs.powerRequest())
    }

    /**
     * Arming says nothing about a rider any more.
     *
     * The board ORs this mirror with its own foot pads and picks a PID profile from the result, so
     * an app that asserts rider whenever it is armed pins every board to the rider-present profile
     * and the pads cannot select anything. Arming and rider-presence are now separate claims, and
     * the app only makes the first one.
     */
    @Test
    fun `arming does not claim a rider`() = runTest(dispatcher) {
        connectAndArm()

        val last = checkNotNull(transport.last)
        assertTrue(last.armed)
        assertFalse(last.rider)
        assertFalse(last.inputs.riderPresent())
    }

    @Test
    fun `the bench toggle is the one way the app asserts rider, and it outlives a disarm`() =
        runTest(dispatcher) {
            transport.setConnectionState(ConnectionState.CONNECTED)
            reporting(ONE_BOARD)
            viewModel.setSimulateRider(true)
            // Staged, not stated: a disarmed frame claims nothing whatever the toggle says.
            assertFalse(checkNotNull(transport.last).rider)
            assertEquals(RiderCommand.DISARMED, transport.last)

            viewModel.onArmToggle()
            assertTrue(checkNotNull(transport.last).rider)
            assertTrue(checkNotNull(transport.last).inputs.riderPresent())

            // Disarming is a rider-intent reset; it is not a reason to forget an operator setting.
            viewModel.onArmToggle()
            assertEquals(RiderCommand.DISARMED, transport.last)
            advanceUntilIdle()
            assertTrue(currentState().simulateRider)

            viewModel.onArmToggle()
            assertTrue(checkNotNull(transport.last).rider, "re-arming keeps the operator's setting")

            viewModel.setSimulateRider(false)
            assertFalse(checkNotNull(transport.last).rider)
            assertTrue(checkNotNull(transport.last).armed, "turning it off does not disarm")
        }

    /**
     * Losing the link clears it, and that is the point.
     *
     * The simulation is set for a board with no pads wired. Carried across a reconnect it would
     * follow the app onto whatever it attaches to next, including a board that HAS pads, where the
     * folded rider level is a term in the FSM's engage conjunction and the inverse of its wind-down
     * gate: the app would hold that machine engaged through a step-off, having been told to stand in
     * for pads that are physically there. A setting that outlives the board it was made for is a
     * latch, so the link dropping resets it with everything else.
     */
    @Test
    fun `losing the link clears the bench rider simulation`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.setSimulateRider(true)
        viewModel.onArmToggle()
        assertTrue(checkNotNull(transport.last).rider)

        transport.setConnectionState(ConnectionState.DISCONNECTED)
        assertFalse(currentState().simulateRider)
        assertFalse(currentState().armed)

        // And a reconnect comes back with it off, so re-arming claims no rider.
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onArmToggle()
        assertTrue(checkNotNull(transport.last).armed)
        assertFalse(checkNotNull(transport.last).rider)
    }

    @Test
    fun `the throttle alone never arms and never demands`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.onThrottleMove(y = 0f, height = h) // full forward, unarmed

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertTrue(transport.sent.none { it.armed || it.demand != 0 })
        // The pad must not display travel that is not being commanded.
        assertEquals(0, currentState().throttleSpeed)
    }

    @Test
    fun `arming is refused while the throttle is held`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        reporting(ONE_BOARD)
        viewModel.onThrottleMove(y = 0f, height = h) // finger down on a deflected throttle
        assertFalse(currentState().canArm)

        viewModel.onArmToggle()
        assertFalse(checkNotNull(transport.last).armed, "must not arm into a deflected throttle")

        // Let go, and the same press is accepted.
        viewModel.onThrottleRelease()
        viewModel.onArmToggle()
        assertTrue(checkNotNull(transport.last).armed)
    }

    @Test
    fun `disarming is never refused, even with the throttle held`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h) // engaged, so ARMING would be refused

        viewModel.onArmToggle()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `armed throttle travel is commanded as a demand`() = runTest(dispatcher) {
        connectAndArm()

        viewModel.onThrottleMove(y = 0f, height = h)
        assertEquals(Throttle.MAX_SPEED, checkNotNull(transport.last).demand)

        viewModel.onThrottleMove(y = h, height = h)
        assertEquals(-Throttle.MAX_SPEED, checkNotNull(transport.last).demand)

        viewModel.onThrottleMove(y = 0.5f * h, height = h)
        assertEquals(0, checkNotNull(transport.last).demand)
    }

    @Test
    fun `releasing the throttle zeroes the demand but stays armed`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        viewModel.onThrottleRelease()
        val last = checkNotNull(transport.last)
        assertEquals(0, last.demand)
        assertTrue(last.armed, "a rider resting at a stop has not stopped riding")
    }

    @Test
    fun `nothing is sent while not connected`() = runTest(dispatcher) {
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        assertTrue(transport.sent.isEmpty())
    }

    // --- the four release paths, each ending disarmed --------------------------------------------

    @Test
    fun `release path 1, tapping the arm control off disarms`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)
        assertTrue(checkNotNull(transport.last).armed)

        viewModel.onArmToggle()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `release path 2, backgrounding the app disarms`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        // No pointer-cancel: Android does not promise one when it takes the window away.
        viewModel.onAppBackgrounded()

        assertEquals(RiderCommand.DISARMED, transport.last)
        assertFalse(currentState().armed)
    }

    @Test
    fun `release path 3, disconnecting disarms BEFORE the link is dropped`() = runTest(dispatcher) {
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)

        viewModel.disconnect()

        // The disarming command is produced immediately...
        assertEquals(RiderCommand.DISARMED, transport.last)
        val sentBefore = transport.sent.size

        // ... and the link is held open for it. Dropping it first would leave the board armed until
        // the remote INPUTS mirror expired 1.5 s later (linkctl::INPUTS_TIMEOUT_TICKS).
        assertEquals(0, transport.disconnectCalls, "link dropped before the disarm could be sent")

        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS + 1)
        runCurrent()

        assertEquals(1, transport.disconnectCalls)
        assertEquals(sentBefore, transport.sentAtDisconnect)
        assertEquals(RiderCommand.DISARMED, transport.sent[transport.sentAtDisconnect!! - 1])
        assertTrue(
            MainViewModel.DISARM_SETTLE_MS >= 2 * LinkConfig.SEND_INTERVAL_MS,
            "the window must cover more than one pump tick on a link with no retransmit",
        )
    }

    @Test
    fun `release path 4, losing the link disarms the app and requires a fresh press`() =
        runTest(dispatcher) {
            connectAndArm()
            viewModel.onThrottleMove(y = 0f, height = h)
            assertTrue(currentState().armed)

            // The link drops on its own; nothing can be sent from here on.
            transport.setConnectionState(ConnectionState.DISCONNECTED)
            assertFalse(currentState().armed)

            // Reconnecting must not resume the arm level off a finger that never lifted.
            val sentBefore = transport.sent.size
            transport.setConnectionState(ConnectionState.CONNECTED)
            advanceUntilIdle()
            assertEquals(sentBefore, transport.sent.size, "reconnect must not re-assert power")
            assertFalse(currentState().armed)

            viewModel.onArmToggle()
            assertTrue(checkNotNull(transport.last).armed)
        }

    /**
     * The settle window is not a quiet period: the link is still up, the control screen is still on
     * screen, and a finger is still on the glass. If the arm control works inside it, one mis-tap
     * re-arms the board and the delayed drop then leaves it armed with nothing the app can do about
     * it: the firmware releases the level only when the mirror expires, a second and a half after
     * the app was last heard from (`linkctl::INPUTS_TIMEOUT_TICKS`). The app would meanwhile reset
     * itself to disarmed and render "disconnected", which is the worst possible pair: a live
     * machine that the app believes it has left, for as long as the timeout takes.
     */
    @Test
    fun `a tap inside the settle window cannot leave the board armed`() = runTest(dispatcher) {
        backgroundScope.launch { viewModel.uiState.collect {} }
        runCurrent()
        connectAndArm()
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()
        assertTrue(viewModel.uiState.value.armed)

        viewModel.disconnect()
        runCurrent()
        val disarmedAt = transport.sent.size
        assertEquals(RiderCommand.DISARMED, transport.last)

        // Half way through the window. Nothing about the UI has changed yet: still CONNECTED, still
        // the control screen, and the drop is still pending.
        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS / 2)
        runCurrent()
        assertEquals(ConnectionState.CONNECTED, viewModel.uiState.value.connectionState)
        assertEquals(0, transport.disconnectCalls)

        // The mis-tap, and a finger that follows it onto the throttle.
        viewModel.onArmToggle()
        viewModel.onThrottleMove(y = 0f, height = h)
        runCurrent()

        // The window runs out and the link goes.
        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS)
        runCurrent()

        assertEquals(1, transport.disconnectCalls)
        val atDrop = checkNotNull(transport.sentAtDisconnect)
        assertFalse(
            transport.sent[atDrop - 1].armed,
            "the link was dropped with the board armed; nothing can correct that until the timeout",
        )
        assertTrue(
            transport.sent.subList(disarmedAt, atDrop).none { it.armed },
            "nothing may re-assert power between the disarm and the drop",
        )
        assertFalse(viewModel.uiState.value.armed)
    }

    /**
     * The same refusal as the test above, rendered for the user rather than enforced on the wire.
     * A control that silently ignores a press is a control the rider will press again; the arm
     * button has to go unavailable for the whole window, and come back only with the link.
     */
    @Test
    fun `the arm control is unavailable for the whole settle window`() = runTest(dispatcher) {
        backgroundScope.launch { viewModel.uiState.collect {} }
        runCurrent()
        connectAndArm()
        runCurrent()
        assertTrue(viewModel.uiState.value.canArm)

        viewModel.disconnect()
        runCurrent()
        assertFalse(viewModel.uiState.value.canArm, "refused as soon as the disconnect is asked for")

        advanceTimeBy(MainViewModel.DISARM_SETTLE_MS - 1)
        runCurrent()
        assertEquals(ConnectionState.CONNECTED, viewModel.uiState.value.connectionState)
        assertFalse(viewModel.uiState.value.canArm, "still refused on the last tick of the window")

        // A fresh session arms normally: the refusal is scoped to the disconnect, not sticky.
        advanceTimeBy(2)
        runCurrent()
        transport.setConnectionState(ConnectionState.DISCONNECTED)
        runCurrent()
        assertFalse(viewModel.uiState.value.canArm, "nothing is armable while disconnected")
        transport.setConnectionState(ConnectionState.CONNECTED)
        runCurrent()
        assertTrue(viewModel.uiState.value.canArm, "a new connection must be armable again")
    }

    /**
     * The settle runs in `viewModelScope`, so a ViewModel cleared inside the window would cancel it
     * with the link still up and the transport still holding a session. The board is already
     * disarmed and pinned there by the pump, so this is a leak rather than a hazard, but the link
     * must not outlive the thing that owns it.
     */
    @Test
    fun `a ViewModel cleared inside the settle window still drops the link`() = runTest(dispatcher) {
        val store = ViewModelStore()
        store.put("main", viewModel)
        connectAndArm()

        viewModel.disconnect()
        runCurrent()
        assertEquals(0, transport.disconnectCalls, "the window has not run out yet")

        // The activity goes away mid-window: clear() cancels viewModelScope, then calls onCleared().
        store.clear()
        advanceUntilIdle()

        assertEquals(1, transport.disconnectCalls, "the link outlived the ViewModel that owned it")
    }

    // --- ui state -------------------------------------------------------------------------------

    @Test
    fun `armed and engaged are both reflected in ui state`() = runTest(dispatcher) {
        transport.setConnectionState(ConnectionState.CONNECTED)
        viewModel.uiState.test {
            assertFalse(awaitItem().armed)

            viewModel.onArmToggle()
            var state = awaitItem()
            while (!state.armed) state = awaitItem()
            assertFalse(state.engaged)

            viewModel.onThrottleMove(y = 0f, height = h)
            state = awaitItem()
            while (!state.engaged) state = awaitItem()
            assertEquals(Throttle.MAX_SPEED, state.throttleSpeed)
            assertEquals(100, state.throttlePercent)

            viewModel.onArmToggle()
            state = awaitItem()
            while (state.armed) state = awaitItem()
            assertEquals(0, state.throttleSpeed)
            assertFalse(state.engaged)
            cancelAndIgnoreRemainingEvents()
        }
    }

    /**
     * The gauge follows the pack the board stores (`specs/rider-ui.md` 3.6b): the cell count is
     * read with the ride facts, so a state of charge is shown without Setup ever being opened.
     *
     * 33.0 V on eight Li-ion cells (the default chemistry) is 4.125 V a cell, between the table's 90%
     * (4.07 V) and 100% (4.20 V) points, so 94%. The curve this replaced was
     * a seven-cell one compiled into the app, which read the same voltage as a full pack: every
     * reading at or above 29.4 V clamped to 100% on both machines here.
     */
    @Test
    fun `the battery gauge is scored against the pack the board stores`() = runTest(dispatcher) {
        transport.store[0x01 to Fields.BOARD_BATTERY_CELLS.key()] = Value.U8(8)
        transport.setConnectionState(ConnectionState.CONNECTED)
        transport.setAttachedBoard(0x01)
        transport.emitCyclicState(CyclicState(0, 0, 0, 3_300, 1, 0, 0, 0, 0, QUIET_OBS, 0))
        advanceUntilIdle()

        val charge = currentState().batteryOf(0x01) as BatteryReading.Charge
        assertEquals(33.0f, charge.volts, 0.01f)
        assertEquals(94, charge.percent)
        assertFalse(charge.low)
    }

    @Test
    fun `connect delegates to transport`() = runTest(dispatcher) {
        viewModel.connect()
        assertEquals(1, transport.connectCalls)
    }

    @Test
    fun `injected telemetry maps into ui state with battery percent`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial

            transport.setConnectionState(ConnectionState.CONNECTED)
            // Attached first: telemetry is resolved by the address a board's frames arrive under,
            // so a session with no board attached has no board's readings to show.
            transport.setAttachedBoard(0x01)
            // batt 3555 cV = 35.55 V -> mid pack, not low. CYCLIC_STATE.battery is CENTIvolts
            // (`crates/orchestrator/src/dispatch.rs`, `BlockWords`), not the millivolts this app
            // used to assume; reading it as mV would show a 36 V pack as 3.6 V and trip
            // battery-low.
            transport.emitCyclicState(
                CyclicState(
                    pitch = -250,
                    roll = 125,
                    wheelSpeed = 600,
                    battery = 3_555,
                    mode = 2,
                    fault = 0,
                    flags = CyclicState.FLAG_RIDER,
                    peerDemand = 0,
                    wheelPos = 0,
                    wheelRate = 8_192,
                    obs = CyclicObs(
                        phasePeak = 980,
                        phaseMean = 410,
                        dutyOn = 1_125,
                        bootTag = 7,
                        chip = ChipTag.F130C8,
                        odometer = 1_284_310L,
                    ),
                ),
            )

            var state = awaitItem()
            while (state.telemetry == null) {
                state = awaitItem()
            }
            assertTrue(state.isConnected)
            val telem = state.telemetry!!
            assertEquals(35.55f, telem.batteryVolts!!, 0.01f)
            assertEquals(8_192, telem.wheelRateQ12)
            assertEquals(-2.5f, telem.pitchDegrees, 0.001f)
            assertEquals(1.25f, telem.rollDegrees, 0.001f)
            assertTrue(telem.riderPresent)
            assertFalse(telem.lockdown)
            // The appended block reaches the model in the board's own units: counts for the two
            // current words (amps need the board's MOTOR_CURRENT_CAL), the on-duty against ARR.
            assertEquals(980, telem.phasePeakCounts)
            assertEquals(410, telem.phaseMeanCounts)
            assertEquals(1_125, telem.dutyOn)
            assertEquals(7, telem.bootTag)
            assertEquals(ChipTag.F130C8, telem.chip)
            // The odometer is a count of hall edges, carried as a Long because the wire word is
            // a u32: the app has no wheel size and converts it to nothing.
            assertEquals(1_284_310L, telem.odometerEdges)
            // 35.55 V with no pack read from this board: the voltage stands and no charge figure
            // is derived from it (`specs/rider-ui.md` 3.6b).
            assertEquals(
                BatteryReading.Unjudged(telem.batteryVolts!!, BatteryReading.Unjudged.Why.NO_CELLS),
                state.batteryOf(0x01),
            )
            cancelAndIgnoreRemainingEvents()
        }
    }

    /**
     * Replaces the old "telemetry merges latest per motor index".
     *
     * There is no per-motor telemetry frame any more: CYCLIC_STATE is one board-level record
     * (`crates/linkctl/src/lib.rs`, `CyclicState`) with no motor index and no per-wheel current,
     * and it is best-effort latest-wins (that crate's module doc). So the property worth pinning
     * flipped from "keep one entry per motor" to "the newest state wins outright", and it holds
     * for the appended block as much as for the committed words.
     */
    @Test
    fun `cyclic state is latest-wins`() = runTest(dispatcher) {
        viewModel.uiState.test {
            awaitItem() // initial
            transport.setConnectionState(ConnectionState.CONNECTED)
            transport.setAttachedBoard(0x01)

            transport.emitCyclicState(
                CyclicState(0, 0, 100, 3_600, 1, 0, 0, 0, 0, QUIET_OBS, 0),
            )
            transport.emitCyclicState(
                CyclicState(
                    0, 0, 250, 3_550, 2, 0, CyclicState.FLAG_RIDER, 0, 0,
                    CyclicObs(
                        phasePeak = 980,
                        phaseMean = 410,
                        dutyOn = 1_125,
                        bootTag = 7,
                        chip = ChipTag.F130C8,
                        odometer = 1_284_310L,
                    ),
                    -4_096,
                ),
            )

            var state = awaitItem()
            while (state.telemetry?.cyclic?.wheelSpeed != 250) {
                state = awaitItem()
            }
            val telem = state.telemetry!!
            assertEquals(-4_096, telem.wheelRateQ12)
            assertEquals(35.5f, telem.batteryVolts!!, 0.01f)
            assertTrue(telem.riderPresent)
            // The appended block is latest-wins with the rest of the payload: the newest state's
            // window and identity are what is read, not the first one's zeroed window.
            assertEquals(410, telem.phaseMeanCounts)
            assertEquals(1_125, telem.dutyOn)
            assertEquals(ChipTag.F130C8, telem.chip)
            assertEquals(1_284_310L, telem.odometerEdges)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `the ui carries the configured board name, defaulting to Hoverboard`() = runTest(dispatcher) {
        // The Scan button labels itself with this, so it has to reach the UI state.
        assertEquals(LinkConfig.DEFAULT_DEVICE_NAME, viewModel.uiState.first().deviceName)
    }

    @Test
    fun `setting the board name reaches both the ui and the store`() = runTest(dispatcher) {
        viewModel.setDeviceName("hb-offroad-m")

        assertEquals("hb-offroad-m", settings.deviceName.value)
        viewModel.uiState.test {
            var state = awaitItem()
            while (state.deviceName != "hb-offroad-m") {
                state = awaitItem()
            }
            assertEquals("hb-offroad-m", state.deviceName)
            cancelAndIgnoreRemainingEvents()
        }
    }

    @Test
    fun `every disarming path produces the same single command`() = runTest(dispatcher) {
        // One value, so no path can disarm halfway: there is no representable command that has let
        // go of the arm level while still carrying a demand.
        val paths = listOf<(MainViewModel) -> Unit>(
            { it.onArmToggle() },
            { it.onAppBackgrounded() },
            { it.disconnect() },
        )
        for (path in paths) {
            transport = FakeHoverboardTransport(dispatcher.scheduler)
            viewModel = MainViewModel(transport, settings, FakeBoardScanner())
            connectAndArm()
            viewModel.onThrottleMove(y = 0f, height = h)

            path(viewModel)

            val last = checkNotNull(transport.last)
            assertNotNull(last)
            assertEquals(RiderCommand.DISARMED, last)
            assertFalse(last.inputs.powerRequest())
            assertEquals(0, last.demand)
            advanceUntilIdle()
        }
    }
}
