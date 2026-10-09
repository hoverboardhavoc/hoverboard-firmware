package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent

/** A [TuneModel] over the fake transport, whose boards hold the default gains until told otherwise. */
internal class TuneRig(scope: TestScope) {
    var armed = false
    val waiver = MutableStateFlow(RiderWaiver.NONE)
    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        for (p in listOf(Gains.CONTROL_GAIN_A, Gains.CONTROL_GAIN_B)) {
            for (i in 0 until Gains.PER_PROFILE) defaults[Gains.key(p, i)] = Value.I16(Gains.default(p, i)!!)
        }
        defaults[Fields.CONTROL_RIDER_REQUIRED.key(0)] = Fields.CONTROL_RIDER_REQUIRED.default
        for (i in 0 until Gains.PER_PROFILE) defaults[Fields.CONTROL_GAIN_MAX.at(i).key(i)] = Fields.CONTROL_GAIN_MAX.defaults[i]
    }
    val model = TuneModel(transport, scope.backgroundScope, { armed }, waiver)
    val state: TuneState get() = model.state.value
}

/**
 * Drag gain [index]'s slider [delta] counts from its staged value and let go: one settled value,
 * which is what a single release costs the lane. The smallest useful gesture, and the unit most of
 * these tests want.
 */
internal fun TuneRig.nudge(index: Int, delta: Int) {
    val from = state.gains.staged[state.keys[index]] ?: return
    model.slideEnd(index, from + delta)
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
    val from = rig.state.gains.staged[rig.state.keys[index]] ?: return
    for (i in 1..samples) {
        rig.model.slide(index, from + (to - from) * i / samples)
        advanceTimeBy(frameMs)
    }
    rig.model.slideEnd(index, to)
    // The release goes out when the send loop next wakes, which is within one cadence interval of
    // the finger lifting, so the gesture is not over until virtual time has passed that.
    advanceTimeBy(TuneModel.DRAG_SEND_INTERVAL_MS + 1)
    runCurrent()
}

/** A rig attached to [master] (with [slave] discovered) and the Tune screen shown with its read pass done. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownTune(master: Int = 0x01, slave: Int? = 0x02): TuneRig = TuneRig(this).also {
    it.transport.setAttachedBoard(master, slave)
    it.model.onShown()
    runCurrent()
}
