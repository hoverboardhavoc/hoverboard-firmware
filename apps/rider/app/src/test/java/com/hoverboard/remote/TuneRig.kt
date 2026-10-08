package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.TestScope
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
    }
    val model = TuneModel(transport, scope.backgroundScope, { armed }, waiver)
    val state: TuneState get() = model.state.value
}

/** A rig attached to [master] (with [slave] discovered) and the Tune screen shown with its read pass done. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownTune(master: Int = 0x01, slave: Int? = 0x02): TuneRig = TuneRig(this).also {
    it.transport.setAttachedBoard(master, slave)
    it.model.onShown()
    runCurrent()
}
