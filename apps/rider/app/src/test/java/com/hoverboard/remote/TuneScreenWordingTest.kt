package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Gains
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.screens.TUNE_RETRY_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says where its knowledge is partial: a master that may run its rider
 * requirement waived, the slave's profiles judged from its stored value, and a request slot that
 * stayed taken (`specs/rider-ui.md` section 3.3).
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneScreenWordingTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keysA = (0 until Gains.PER_PROFILE).map { Gains.key(Gains.CONTROL_GAIN_A, it) }
    private val read = BoardGains(
        staged = keysA.zip(listOf(6000, 2000, 40)).toMap(),
        flash = keysA.zip(listOf(6000, 2000, 40)).toMap(),
        stale = false,
    )

    /** Records what the screen asked for. */
    private class Recorder : TuneActions {
        val calls = mutableListOf<String>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(node: Node) { calls += "target:$node" }
        override fun selectProfile(fieldId: Int) { calls += "profile:$fieldId" }
        override fun refresh() { calls += "refresh" }
        override fun slide(index: Int, value: Int) { calls += "slide:$index:$value" }
        override fun slideEnd(index: Int, value: Int) { calls += "end:$index:$value" }
        override fun save() { calls += "save" }
        override fun revert() { calls += "revert" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    private fun show(state: TuneState, telemetry: TelemetryUi? = null): Recorder {
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme { TuneScreen(state = state, armed = false, telemetry = telemetry, actions = r) }
        }
        return r
    }

    private val padsOff = TelemetryUi().merge(CyclicState(0, 0, 0, 3_300, 0, 0, 0, QUIET_OBS))

    @Test
    fun aMasterThatMayRunWaivedQualifiesThePadsLevel() {
        show(
            TuneState(master = 0x01, masterRiderWaiver = RiderWaiver.POSSIBLY, boards = mapOf(Node.MASTER to read)),
            padsOff,
        )
        val off = s(R.string.tune_rider_off)
        val line = s(R.string.tune_rider_maybe_waived, off)
        compose.onNodeWithText(line).assertExists()
        // Only possibly waived, so only possibly not the level it acts on.
        assertTrue(line, line.endsWith("may not be the rider level it acts on."))
        compose.onNodeWithText(off).assertDoesNotExist()
    }

    @Test
    fun aMasterNotWaivedShowsThePadsLevelPlain() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)), padsOff)
        compose.onNodeWithText(s(R.string.tune_rider_off)).assertExists()
    }

    @Test
    fun theSlavesProfilesSayTheyFollowItsStoredRequirement() {
        val slave = TuneState(master = 0x01, slave = 0x02, target = Node.SLAVE, boards = mapOf(Node.SLAVE to read))
        show(slave)
        val note = s(R.string.tune_profile_slave_note)
        compose.onNodeWithText(note).assertExists()
        // Re-read on every slave pass (Refresh included), not once a session; still assumed to be what runs.
        assertTrue(note, note.contains("read from its store"))
        assertFalse(note, note.contains("once this session"))
        assertTrue(note, note.contains("taken to be what it runs"))
    }

    @Test
    fun theMastersProfilesCarryNoSlaveNote() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)))
        compose.onNodeWithText(s(R.string.tune_profile_slave_note)).assertDoesNotExist()
    }

    @Test
    fun aTakenSlotSaysSoAndOffersARetry() {
        val r = show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read), notice = TuneNotice.SlotBusy))
        compose.onNodeWithText(s(R.string.tune_notice_slot_busy)).assertExists()
        compose.onNodeWithTag(TUNE_RETRY_TAG).performClick()
        assertEquals("refresh", r.calls.last())
    }
}
