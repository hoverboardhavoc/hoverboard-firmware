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
import com.hoverboard.protocol.store.Key
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
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says about the two gain sets, and what it says where its knowledge is
 * partial: how the sets are named and who switches between them (`specs/rider-ui.md` section 4,
 * "What the screen calls the two sets"), a master that may run its rider requirement waived, the
 * slave's sets judged from its stored value, and a request slot that stayed taken (section 3.3).
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneScreenWordingTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

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
        val bounds = mutableListOf<Pair<Key, Int>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(node: Node) { calls += "target:$node" }
        override fun selectProfile(fieldId: Int) { calls += "profile:$fieldId" }
        override fun refresh() { calls += "refresh" }
        override fun slide(index: Int, value: Int) { calls += "slide:$index:$value" }
        override fun slideEnd(index: Int, value: Int) { calls += "end:$index:$value" }
        override fun setBound(key: Key, value: Int) { bounds += key to value }
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

    private val padsOff = TelemetryUi().merge(CyclicState(0, 0, 0, 3_300, 0, 0, 0, 0, QUIET_OBS))
    private val padsOn = TelemetryUi().merge(CyclicState(0, 0, 0, 3_300, 0, 0, CyclicState.FLAG_RIDER, 0, QUIET_OBS))

    private fun withRider() = s(R.string.tune_profile_a)
    private fun noRider() = s(R.string.tune_profile_b)

    /** The chips name the two sets by what selects each one, never by the field that stores it. */
    @Test
    fun theTwoSetsAreNamedByWhatSelectsThem() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)), padsOff)
        compose.onNodeWithText(withRider()).assertExists()
        compose.onNodeWithText(noRider()).assertExists()
        for (name in listOf(withRider(), noRider())) {
            assertTrue(name, name.contains("rider"))
            assertFalse(name, name.contains("Profile"))
        }
    }

    /**
     * Nothing on the screen used to say the board switches sets on its own, so a reader could take
     * the chips for the switch. The line under them says both halves: who switches, and that a tap
     * only chooses what is edited.
     */
    @Test
    fun theLineSaysTheBoardSwitchesSetsByItselfAndTheChipsOnlyChooseWhatIsEdited() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)), padsOff)
        val line = s(R.string.tune_profile_auto, s(R.string.tune_profile_live_none, noRider()))
        compose.onNodeWithText(line).assertExists()
        assertTrue(line, line.contains("by itself"))
        assertTrue(line, line.contains("only pick which set you are editing"))
    }

    /** Which set is live now, from the pads level the master reports: both ways round. */
    @Test
    fun thePadsLevelSaysWhichSetTheBoardIsOnNow() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)), padsOn)
        compose.onNodeWithText(s(R.string.tune_profile_auto, s(R.string.tune_profile_live_rider, withRider())))
            .assertExists()
    }

    /** No cyclic yet: the app does not know the pads level, so it claims no set. */
    @Test
    fun withoutTelemetryTheScreenSaysItDoesNotKnowWhichSetIsLive() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)))
        val line = s(R.string.tune_profile_auto, s(R.string.tune_profile_live_unknown))
        compose.onNodeWithText(line).assertExists()
    }

    @Test
    fun aMasterThatMayRunWaivedQualifiesThePadsLevel() {
        show(
            TuneState(master = 0x01, masterRiderWaiver = RiderWaiver.POSSIBLY, boards = mapOf(Node.MASTER to read)),
            padsOff,
        )
        val clear = s(R.string.tune_profile_live_none, noRider())
        val line = s(R.string.tune_profile_auto, s(R.string.tune_profile_live_maybe_waived, clear, withRider()))
        compose.onNodeWithText(line).assertExists()
        // Only possibly waived, so only possibly on the other set than the pads say.
        assertTrue(line, line.contains("may be waived"))
        compose.onNodeWithText(s(R.string.tune_profile_auto, clear)).assertDoesNotExist()
    }

    /**
     * The no-rider set does not hold an empty machine up, which is what its name invites a reader
     * to assume: the board never engages on it. Said where it is read, under the set it is about.
     */
    @Test
    fun theNoRiderSetSaysWhatItActuallyShapes() {
        show(
            TuneState(master = 0x01, profile = Gains.CONTROL_GAIN_B, boards = mapOf(Node.MASTER to read)),
            padsOff,
        )
        val note = s(R.string.tune_profile_b_note)
        compose.onNodeWithText(note).assertExists()
        assertTrue(note, note.contains("does not engage on this set"))
        assertTrue(note, note.contains("step off"))
    }

    /** It is about the no-rider set, so it is not shown while the rider's set is the one edited. */
    @Test
    fun theRidersSetCarriesNoSuchNote() {
        show(TuneState(master = 0x01, boards = mapOf(Node.MASTER to read)), padsOff)
        compose.onNodeWithText(s(R.string.tune_profile_b_note)).assertDoesNotExist()
    }

    /** One set to reach, so no chips and no switching to explain: one line says what runs. */
    @Test
    fun aBoardThatDoesNotRequireARiderIsToldItHasOneSet() {
        show(
            TuneState(master = 0x01, masterRiderWaiver = RiderWaiver.WAIVED, boards = mapOf(Node.MASTER to read)),
            padsOff,
        )
        compose.onNodeWithText(s(R.string.tune_profile_waived)).assertExists()
        compose.onNodeWithText(s(R.string.tune_profile_auto, s(R.string.tune_profile_live_none, noRider())))
            .assertDoesNotExist()
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
