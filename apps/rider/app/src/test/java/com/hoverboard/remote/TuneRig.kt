package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent

/** A [TuneModel] over the fake transport, whose boards hold the default gains until told otherwise. */
internal class TuneRig(scope: TestScope) {
    var armed = false

    /** The rider requirement each board runs, as the Setup model would report it, by address. */
    val waivers = MutableStateFlow<Map<Int, RiderWaiver>>(emptyMap())
    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        for (p in listOf(Gains.CONTROL_GAIN_A, Gains.CONTROL_GAIN_B)) {
            for (i in 0 until Gains.PER_PROFILE) defaults[Gains.key(p, i)] = Value.I16(Gains.default(p, i)!!)
        }
        // The outer speed loop's row, which the read pass reads with the gains (`specs/rider-ui.md`
        // 3.3c). A board that never staged one answers 0 at every index, which is the loop off.
        for (i in 0 until Gains.PER_PROFILE) {
            defaults[Gains.key(Gains.CONTROL_SPEED_LOOP, i)] = Fields.CONTROL_SPEED_LOOP.defaults[i]
        }
        defaults[Fields.CONTROL_RIDER_REQUIRED.key(0)] = Fields.CONTROL_RIDER_REQUIRED.default
        for (i in 0 until Gains.PER_PROFILE) defaults[Fields.CONTROL_GAIN_MAX.at(i).key(i)] = Fields.CONTROL_GAIN_MAX.defaults[i]
        // The four drive bounds the read pass also reads (`specs/rider-ui.md` 3.3b), each at the
        // default a board that never staged one answers with.
        for (field in SetupFields.DRIVE_BOUNDS) defaults[field.key] = field.def.default
    }
    /** The nodes the walk found and the one picked, standing in for the Setup model's selection. */
    val selection = MutableStateFlow(TuneSelection())
    val model = TuneModel(
        transport,
        scope.backgroundScope,
        { armed },
        combine(selection, waivers) { sel, w -> sel.copy(riderWaivers = w) },
    ) { address -> selection.value = selection.value.copy(selected = address) }
    val state: TuneState get() = model.state.value

    /**
     * Attach to [board] with [others] found by the walk, every one a motor controller, and [board]
     * selected: the Setup model's selection on a fresh connect.
     */
    fun attach(board: Int?, vararg others: Int?) {
        val nodes = if (board == null) emptyList() else (listOf(board) + others.filterNotNull()).distinct().sorted()
        selection.value = TuneSelection(
            motors = nodes.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, "Hoverboard") },
            selected = board,
        )
        transport.setAttachedBoard(board, *others)
    }
}

/**
 * Drag gain [index]'s slider [delta] counts from its staged value and let go: one settled value,
 * which is what a single release costs the lane. The smallest useful gesture, and the unit most of
 * these tests want.
 */
internal fun TuneRig.nudge(index: Int, delta: Int) {
    val key = state.keys[index]
    val from = state.gains.staged[key] ?: return
    model.slideEnd(key, from + delta)
}

/**
 * Drag gain [index]'s slider from its staged value to [to] over [samples] frames and let go, the
 * way a finger does: one [TuneActions.slide] per frame and one [TuneActions.slideEnd] at the end.
 *
 * [frameMs] apart in virtual time, 16 ms by default (a 60 Hz display), so what the model sends is
 * the cadence's doing and not the test's.
 */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.dragGain(
    rig: TuneRig,
    index: Int,
    to: Int,
    samples: Int,
    frameMs: Long = 16,
) {
    val key = rig.state.keys[index]
    val from = rig.state.gains.staged[key] ?: return
    for (i in 1..samples) {
        rig.model.slide(key, from + (to - from) * i / samples)
        advanceTimeBy(frameMs)
    }
    rig.model.slideEnd(key, to)
    // The release goes out when the send loop next wakes, which is within one cadence interval of
    // the finger lifting, so the gesture is not over until virtual time has passed that.
    advanceTimeBy(TuneModel.DRAG_SEND_INTERVAL_MS + 1)
    runCurrent()
}

/** A rig attached to [board] (with [other] found by the walk) and the Tune screen shown with its read pass done. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownTune(board: Int = 0x01, other: Int? = 0x02): TuneRig = TuneRig(this).also {
    it.attach(board, other)
    it.model.onShown()
    runCurrent()
}
