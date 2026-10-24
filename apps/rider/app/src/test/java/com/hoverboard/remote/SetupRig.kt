package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent

/** A [SetupModel] over the fake transport, whose board answers every field's default until told otherwise. */
internal class SetupRig(scope: TestScope) {
    var armed = false
    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        for (f in SetupFields.ALL) defaults[f.key] = f.def.default
        // Not a Setup row, but a ride fact the model reads on attach: the Ride screen's current
        // display needs each board's counts per amp (`specs/rider-ui.md` 3.6).
        defaults[Fields.MOTOR_CURRENT_CAL.key()] = Fields.MOTOR_CURRENT_CAL.default
        // The same for the two the Travel section converts with. Their registry defaults are the
        // UNSET value, so a rig board answers them exactly as a fresh board does: it has them and
        // they say nothing, which is not the same as a board that refuses the key.
        defaults[Fields.MOTOR_POLE_PAIRS.key()] = Fields.MOTOR_POLE_PAIRS.default
        defaults[Fields.WHEEL_DIAMETER_MM.key()] = Fields.WHEEL_DIAMETER_MM.default
    }
    val model = SetupModel(transport, scope.backgroundScope) { armed }
    val state: SetupState get() = model.state.value
}

/** A rig attached to [board] (with [slave] discovered, if any) with the Setup screen shown and its read pass done. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownRig(board: Int, slave: Int? = null): SetupRig = SetupRig(this).also {
    it.transport.setAttachedBoard(board, slave)
    it.model.onShown()
    runCurrent()
}

/**
 * Let the settle window close ([SetupModel.EDIT_SETTLE_MS]) and the queued writes drain.
 *
 * Every edit is written on its own after that window, so a test that edits and does not come
 * through here has sent nothing, which is exactly what the coalescing tests assert.
 *
 * Three windows of virtual time, because an edit that arrives inside a window in progress makes the
 * model wait out a fresh one: one for the window that was running, one for the window the edit
 * bought, and room to spare. `advanceUntilIdle` is NOT the tool here: it leaves background work
 * alone, and the model's whole queue runs in `backgroundScope`.
 */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.flush() {
    advanceTimeBy(SetupModel.EDIT_SETTLE_MS * 3 + 1)
    runCurrent()
}

/** Advance [ms] of virtual time without closing a settle window, for the held-stepper case. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.tick(ms: Long) {
    advanceTimeBy(ms)
    runCurrent()
}

/**
 * A `CYCLIC_STATE` from a board on its [bootTag]-th boot, sitting still and level.
 *
 * [from] names the board it came from, defaulting to the attached one: each board's state reaches
 * the app under its own address (`specs/link-control.md`, the peer forward), and the boot tag the
 * model watches is per board.
 */
internal fun SetupRig.emitBoot(bootTag: Int, from: Int? = null) =
    transport.emitCyclicState(CyclicState(0, 0, 0, 0, 0, 0, 0, 0, QUIET_OBS.copy(bootTag = bootTag)), from)

internal fun SetupRig.emitAttitude(pitch: Int, roll: Int, obs: CyclicObs = QUIET_OBS, from: Int? = null) =
    transport.emitCyclicState(CyclicState(pitch, roll, 0, 0, 0, 0, 0, 0, obs), from)
