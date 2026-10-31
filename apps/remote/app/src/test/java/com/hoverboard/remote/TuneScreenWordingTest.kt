package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.ui.screens.TUNE_RETRY_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen's notices say: a row named by the law's own name (`gp`, `speed_max`), and a
 * request slot that stayed taken (`specs/remote-ui.md` section 3.3).
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneScreenWordingTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }
    private val read = BoardGains(
        staged = keys.zip(listOf(3491, 4002, 197, 926)).toMap(),
        flash = keys.zip(listOf(3491, 4002, 197, 926)).toMap(),
        stale = false,
    )

    /** Records what the screen asked for. */
    private class Recorder : TuneActions {
        val calls = mutableListOf<String>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(address: Int) { calls += "target:$address" }
        override fun refresh() { calls += "refresh" }
        override fun slide(key: Key, value: Int) { calls += "slide:${key.fieldId}:${key.index}:$value" }
        override fun slideEnd(key: Key, value: Int) { calls += "end:${key.fieldId}:${key.index}:$value" }
        override fun setBound(key: Key, value: Int) { calls += "bound" }
        override fun save() { calls += "save" }
        override fun revert() { calls += "revert" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    private fun show(state: TuneState): Recorder {
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme { TuneScreen(state = state, armed = false, actions = r) }
        }
        return r
    }

    @Test
    fun aRefusalNamesTheGainByTheLawsName() {
        val notice = TuneNotice.BoardRefused(keys[Gains.GX], CfgRefusal.BAD)
        show(TuneState(target = 0x01, boards = mapOf(0x01 to read), notice = notice))
        compose.onNodeWithText(s(R.string.tune_notice_refused_bad, "gx")).assertExists()
    }

    @Test
    fun aMismatchNamesTheReferenceWord() {
        val key = Gains.key(Gains.CONTROL_REFERENCE, Gains.SPEED_SLEW)
        show(TuneState(target = 0x01, boards = mapOf(0x01 to read), notice = TuneNotice.Mismatch(key, 41, 40)))
        compose.onNodeWithText(s(R.string.tune_notice_mismatch, "speed_slew", "41", "40")).assertExists()
    }

    @Test
    fun aTakenSlotSaysSoAndOffersARetry() {
        val r = show(TuneState(target = 0x01, boards = mapOf(0x01 to read), notice = TuneNotice.SlotBusy))
        compose.onNodeWithText(s(R.string.tune_notice_slot_busy)).assertExists()
        compose.onNodeWithTag(TUNE_RETRY_TAG).performClick()
        assertEquals("refresh", r.calls.last())
    }
}
