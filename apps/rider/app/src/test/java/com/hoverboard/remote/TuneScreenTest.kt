package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.semantics.SemanticsActions
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performSemanticsAction
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Gains
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.ui.screens.TUNE_PROFILE_B_TAG
import com.hoverboard.remote.ui.screens.TUNE_REVERT_TAG
import com.hoverboard.remote.ui.screens.TUNE_SAVE_TAG
import com.hoverboard.remote.ui.screens.TUNE_SLAVE_TAG
import com.hoverboard.remote.ui.screens.TUNE_STALE_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.tuneRowTag
import com.hoverboard.remote.ui.screens.tuneSliderTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says (`specs/rider-ui.md` section 3.3): staged and flash as numbers, engaged
 * never as one, the converging and unsaved marks, Save off while armed with the reason, the sliders
 * live while armed, and a stale board greyed with writes off.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keysA = (0 until Gains.PER_PROFILE).map { Gains.key(Gains.CONTROL_GAIN_A, it) }

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

    private fun gains(
        staged: List<Int>,
        flash: List<Int>,
        converging: Map<Int, Long> = emptyMap(),
        stale: Boolean = false,
    ) =
        BoardGains(
            staged = keysA.zip(staged).toMap(),
            flash = keysA.zip(flash).toMap(),
            stale = stale,
            converging = converging.mapKeys { keysA[it.key] },
        )

    private fun read(
        g: BoardGains = gains(listOf(6000, 2000, 40), listOf(6000, 2000, 40)),
        slave: Int? = 0x02,
        waiver: RiderWaiver = RiderWaiver.NONE,
    ) = TuneState(master = 0x01, slave = slave, masterRiderWaiver = waiver, boards = mapOf(Node.MASTER to g))

    private fun show(state: TuneState, armed: Boolean = false): Recorder {
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme { TuneScreen(state = state, armed = armed, telemetry = null, actions = r) }
        }
        return r
    }

    private fun inRow(index: Int, text: String) =
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneRowTag(index))))

    @Test
    fun stagedAndFlashAreShownAsNumbersAndEngagedIsSaidToBeUnreadable() {
        show(read(gains(listOf(6100, 2000, 40), listOf(6000, 2000, 40))))

        inRow(0, "6100").assertExists()
        inRow(0, s(R.string.tune_flash, "6000")).assertExists()
        inRow(0, " *").assertExists()
        inRow(1, " *").assertDoesNotExist()
        compose.onNodeWithText(s(R.string.tune_legend_engaged)).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(context.resources.getQuantityString(R.plurals.tune_unsaved_count, 1, 1))
            .performScrollTo().assertIsDisplayed()
    }

    @Test
    fun aConvergingGainCarriesTheMarkAndTheBoundNotAClaimThatItArrived() {
        show(read(gains(listOf(6100, 2000, 40), listOf(6000, 2000, 40), converging = mapOf(0 to 400L))))

        inRow(0, " ~").assertExists()
        inRow(0, s(R.string.tune_converging_row, "0.4")).assertExists()
        inRow(1, " ~").assertDoesNotExist()
    }

    @Test
    fun armedTheSlidersAndRevertWorkAndSaveIsOffSayingWhy() {
        val r = show(read(gains(listOf(6100, 2000, 40), listOf(6000, 2000, 40))), armed = true)

        compose.onNodeWithTag(tuneSliderTag(0)).assertIsEnabled()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(6500f) }
        compose.onNodeWithTag(TUNE_REVERT_TAG).performScrollTo().assertIsEnabled()
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.tune_save_armed)).assertExists()
        compose.onNodeWithText(s(R.string.tune_save_armed_note)).assertExists()

        // Both halves reach the model, at the value the control was taken to: the samples the model
        // sends on a cadence, and the one release that carries the committed value.
        assertEquals(listOf("shown", "slide:0:6500", "end:0:6500"), r.calls)
    }

    /** The slider's extent is the board's own maximum as read, not a constant (3.3a). */
    @Test
    fun theSliderSpansTheRangeReadFromThisBoard() {
        val g = gains(listOf(6000, 2000, 40), listOf(6000, 2000, 40)).copy(maxima = mapOf(0 to 6050))
        show(read(g))

        inRow(0, s(R.string.tune_range, "6050")).assertExists()
        inRow(1, s(R.string.tune_range, Gains.DEFAULT_MAX[1].toString())).assertExists()
    }

    @Test
    fun disarmedWithUnsavedGainsSaveIsOffered() {
        val r = show(read(gains(listOf(6100, 2000, 40), listOf(6000, 2000, 40))))
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsEnabled().performClick()
        assertEquals("save", r.calls.last())
    }

    @Test
    fun aStaleBoardShowsItsLastValuesWithEveryWriteOff() {
        show(read(gains(listOf(6100, 2000, 40), listOf(6000, 2000, 40), stale = true)))

        compose.onNodeWithTag(TUNE_STALE_TAG).assertIsDisplayed()
        inRow(0, "6100").assertExists()
        compose.onNodeWithTag(tuneSliderTag(0)).assertIsNotEnabled()
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(TUNE_REVERT_TAG).assertIsNotEnabled()
    }

    @Test
    fun profileBIsOfferedUnlessTheBoardRunsTheRiderRequirementWaived() {
        show(read(waiver = RiderWaiver.WAIVED))
        compose.onNodeWithTag(TUNE_PROFILE_B_TAG).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.tune_rider_waived)).assertExists()
    }

    @Test
    fun bothProfilesAreOfferedOtherwise() {
        val r = show(read())
        compose.onNodeWithTag(TUNE_PROFILE_B_TAG).performClick()
        assertEquals("profile:${Gains.CONTROL_GAIN_B}", r.calls.last())
    }

    @Test
    fun theSlaveIsOfferedOnlyOnceDiscovered() {
        show(read(slave = null))
        compose.onNodeWithTag(TUNE_SLAVE_TAG).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.tune_target_slave_none)).assertExists()
    }
}
