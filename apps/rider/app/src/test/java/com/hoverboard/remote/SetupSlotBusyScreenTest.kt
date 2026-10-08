package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ui.screens.SETUP_RETRY_TAG
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/** The Setup screen's word for a request slot that stayed taken, and its retry (an Apply). */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupSlotBusyScreenTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()

    /** Records the last action the screen asked for. */
    private class Recorder : SetupActions {
        var last: String? = null
        override fun onShown() = Unit
        override fun onHidden() = Unit
        override fun refresh() { last = "refresh" }
        override fun stage(key: Key, value: Value) = Unit
        override fun discard(key: Key) = Unit
        override fun discardAll() = Unit
        override fun apply() { last = "apply" }
        override fun stageFrame(roles: List<Int>, signs: List<Int>) = Unit
        override fun setLevel() = Unit
        override fun checkLevel() = Unit
        override fun checkForwardLean() = Unit
        override fun confirmPowerCycled() = Unit
        override fun dismissNotice() { last = "dismiss" }
    }

    @Test
    fun aTakenSlotSaysNothingWasWrittenAndOffersAnApply() {
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme {
                SetupScreen(
                    state = SetupState(board = 0x01, notice = SetupNotice.SlotBusy),
                    armed = false,
                    telemetry = null,
                    actions = r,
                )
            }
        }
        compose.onNodeWithText(context.getString(R.string.setup_notice_slot_busy)).assertExists()
        compose.onNodeWithTag(SETUP_RETRY_TAG).performClick()
        assertEquals("apply", r.last)
    }
}
