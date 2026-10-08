package com.hoverboard.remote

import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent

/** A [SetupModel] over the fake transport, whose board answers every field's default until told otherwise. */
internal class SetupRig(scope: TestScope) {
    var armed = false
    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        for (f in SetupFields.ALL) defaults[f.key] = f.def.default
    }
    val model = SetupModel(transport, scope.backgroundScope) { armed }
    val state: SetupState get() = model.state.value
}

/** A rig attached to [board] with the Setup screen shown and its read pass done. */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.shownRig(board: Int): SetupRig = SetupRig(this).also {
    it.transport.setAttachedBoard(board)
    it.model.onShown()
    runCurrent()
}
