package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent

/** A [TuneModel] over the fake transport, whose boards hold the default gains until told otherwise. */
internal class TuneRig(scope: TestScope) {
    var armed = false

    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        // The live lane's two rows and the gain maxima, as a board that never staged any answers.
        for (i in 0 until Gains.COUNT) {
            defaults[Gains.key(Gains.CONTROL_SF_GAINS, i)] = Fields.CONTROL_SF_GAINS.defaults[i]
            defaults[Fields.CONTROL_GAIN_MAX.at(i).key(i)] = Fields.CONTROL_GAIN_MAX.defaults[i]
        }
        for (i in Gains.REFERENCE_DEFAULT.indices) {
            defaults[Gains.key(Gains.CONTROL_REFERENCE, i)] = Fields.CONTROL_REFERENCE.defaults[i]
        }
        // The drive bounds the read pass also reads (`specs/remote-ui.md` 3.3b), each at the
        // default a board that never stored one answers with.
        for (field in SetupFields.DRIVE_BOUNDS) defaults[field.key] = field.def.default
    }

    /** The nodes the walk found and the one picked, standing in for the Setup model's selection. */
    val selection = MutableStateFlow(TuneSelection())
    val model = TuneModel(
        transport,
        scope.backgroundScope,
        { armed },
        selection,
    ) { address -> selection.value = selection.value.copy(selected = address) }
    val state: TuneState get() = model.state.value

    /**
     * Make [board] hold the start set in flash and in its shadow: the four gains of
     * `specs/state-feedback.md` section 7 ([START_GAINS]) and the reference row [START_REFERENCE], the
     * values a staged rover runs, so a test moves real numbers rather than the inert zeros.
     */
    fun holdStart(board: Int) {
        for ((i, v) in START_GAINS.withIndex()) hold(board, Gains.key(Gains.CONTROL_SF_GAINS, i), v)
        for ((i, v) in START_REFERENCE.withIndex()) hold(board, Gains.key(Gains.CONTROL_REFERENCE, i), v)
    }

    /** Make [board] hold [v] at live-lane [key] in flash and in its shadow. */
    fun hold(board: Int, key: Key, v: Int) {
        transport.store[board to key] = Value.I16(v)
        transport.shadow[board to key] = v
    }

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

/** The section-7 start gains, `[gp, gr, gx, gv]` (`specs/state-feedback.md` section 7). */
internal val START_GAINS = listOf(3491, 4002, 197, 926)

/** The section-7 reference row, `[speed_max, speed_slew]`. */
internal val START_REFERENCE = listOf(80, 41)

/**
 * A rig attached to [board] (with [other] found by the walk), both holding the start set
 * ([TuneRig.holdStart]) and then whatever [setup] adds, and the Tune screen shown with its read
 * pass done.
 */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownTune(
    board: Int = 0x01,
    other: Int? = 0x02,
    setup: TuneRig.() -> Unit = {},
): TuneRig = TuneRig(this).also {
    it.holdStart(board)
    other?.let(it::holdStart)
    it.setup()
    it.attach(board, other)
    it.model.onShown()
    runCurrent()
}
