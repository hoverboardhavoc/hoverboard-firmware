package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.setupRetryTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/** The Setup screen's word for a request slot that stayed taken, and the row's own retry. */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupSlotBusyScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()

    /** Records the last action the screen asked for. */
    private class Recorder : SetupActions {
        var last: String? = null
        override fun onShown() = Unit
        override fun onHidden() = Unit
        override fun refresh() { last = "refresh" }
        override fun edit(key: Key, value: Value) { last = "edit" }
        override fun retry(key: Key) { last = "retry:$key" }
        override fun editFrame(roles: List<Int>, signs: List<Int>) = Unit
        override fun setLevel() = Unit
        override fun checkLevel() = Unit
        override fun checkForwardLean() = Unit
        override fun dismissNotice() { last = "dismiss" }
    }

    @Test
    fun aTakenSlotSaysNothingWasWrittenOnThatRowAndOffersItsOwnRetry() {
        val r = Recorder()
        val mode = SetupFields.CONTROL_MODE.key
        compose.setContent {
            HoverboardRemoteTheme {
                SetupScreen(
                    state = SetupState(
                        board = 0x01,
                        values = SetupFields.ALL.associate { it.key to it.def.default },
                        writes = mapOf(mode to FieldWrite(Value.U8(1), WriteState.SlotBusy)),
                    ),
                    armed = false,
                    telemetry = null,
                    actions = r,
                )
            }
        }
        compose.onNodeWithText(context.getString(R.string.setup_write_slot_busy)).assertExists()
        compose.onNodeWithTag(setupRetryTag(mode)).performScrollTo().performClick()
        assertEquals("retry:$mode", r.last)
    }
}
