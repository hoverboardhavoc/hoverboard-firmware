package com.hoverboardhavoc.remote

import android.app.Application
import androidx.compose.ui.semantics.SemanticsActions
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performSemanticsAction
import androidx.compose.ui.test.performTextClearance
import androidx.compose.ui.test.performTextInput
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboardhavoc.remote.ui.screens.TUNE_REFERENCE_TAG
import com.hoverboardhavoc.remote.ui.screens.TUNE_TURN_TAG
import com.hoverboardhavoc.remote.ui.screens.TuneScreen
import com.hoverboardhavoc.remote.ui.screens.tuneTurnConvergingTag
import com.hoverboardhavoc.remote.ui.screens.tuneTurnEntryTag
import com.hoverboardhavoc.remote.ui.screens.tuneTurnRowTag
import com.hoverboardhavoc.remote.ui.screens.tuneTurnSetTag
import com.hoverboardhavoc.remote.ui.screens.tuneTurnSliderTag
import com.hoverboardhavoc.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says about the heading hold's five live words (`specs/heading-hold.md`
 * section 10): a section under the reference row, each word in its units, the compiled range
 * stated as every board's, the converging mark on the three gains and none on the rate pair, the
 * unsaved mark, and the slider and entry that cannot send a value outside the range.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneTurnScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val turnKeys = TuneState().turnKeys

    /** Records what the screen asked for. */
    private class Recorder : TuneActions {
        val calls = mutableListOf<String>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(address: Int) { calls += "target:$address" }
        override fun refresh() { calls += "refresh" }
        override fun slide(key: Key, value: Int) { calls += "slide:${key.fieldId}:${key.index}:$value" }
        override fun slideEnd(key: Key, value: Int) { calls += "end:${key.fieldId}:${key.index}:$value" }
        override fun setBound(key: Key, value: Int) { calls += "bound:${key.fieldId}:${key.index}:$value" }
        override fun save() { calls += "save" }
        override fun revert() { calls += "revert" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    /** A board read this session whose turn words are staged at [turn] and whose flash holds [flash]. */
    private fun show(
        turn: List<Int> = START_TURN,
        flash: List<Int> = turn,
        converging: Map<Key, Long> = emptyMap(),
        armed: Boolean = false,
    ): Recorder {
        val gains = BoardGains(
            staged = turnKeys.zip(turn).toMap(),
            flash = turnKeys.zip(flash).toMap(),
            stale = false,
            converging = converging,
        )
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme {
                TuneScreen(
                    state = TuneState(target = 0x01, boards = mapOf(0x01 to gains)),
                    armed = armed,
                    actions = r,
                )
            }
        }
        return r
    }

    private fun inRow(index: Int, text: String) =
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneTurnRowTag(index))))

    @Test
    fun theSectionSitsUnderTheReferenceRowAndNamesEachWordInItsUnits() {
        show()

        // The scroll column composes every section, so the two headings' positions compare directly.
        val reference = compose.onNodeWithTag(TUNE_REFERENCE_TAG).fetchSemanticsNode().positionInRoot.y
        val turn = compose.onNodeWithTag(TUNE_TURN_TAG).fetchSemanticsNode().positionInRoot.y
        assertTrue("the turn section is not under the reference row", turn > reference)

        val words = listOf(
            R.string.tune_turn_gh to R.string.tune_turn_gh_note,
            R.string.tune_turn_ghv to R.string.tune_turn_ghv_note,
            R.string.tune_turn_gff to R.string.tune_turn_gff_note,
            R.string.tune_turn_rate_max to R.string.tune_turn_rate_max_note,
            R.string.tune_turn_rate_slew to R.string.tune_turn_rate_slew_note,
        )
        for ((i, pair) in words.withIndex()) {
            compose.onNodeWithTag(tuneTurnRowTag(i)).performScrollTo()
            inRow(i, s(pair.first)).assertExists()
            inRow(i, s(pair.second)).assertExists()
        }
        assertTrue(s(R.string.tune_turn_rate_max_note).contains("holds its heading"))
        assertTrue(s(R.string.tune_turn_gff_note).contains("0 is loop only"))
    }

    @Test
    fun eachRowStatesItsCompiledRange() {
        show()
        val ranges = Gains.TURN_GAIN_MAXIMA.map { Gains.MIN to it } +
            Gains.TURN_REFERENCE_MINIMA.zip(Gains.TURN_REFERENCE_MAXIMA)
        for ((i, range) in ranges.withIndex()) {
            compose.onNodeWithTag(tuneTurnRowTag(i)).performScrollTo()
            inRow(i, s(R.string.tune_reference_range, range.first.toString(), range.second.toString())).assertExists()
        }
    }

    /** The three gains carry the ramp's mark; the rate pair is not ramped and never does. */
    @Test
    fun theConvergingMarkShowsOnAGainRowOnly() {
        show(converging = mapOf(turnKeys[Gains.GHV] to 400L))

        compose.onNodeWithTag(tuneTurnConvergingTag(Gains.GHV)).performScrollTo().assertExists()
        inRow(Gains.GHV, s(R.string.tune_converging_row, "0.4")).assertExists()
        for (i in listOf(Gains.GH, Gains.GFF, 3, 4)) {
            compose.onNodeWithTag(tuneTurnConvergingTag(i)).assertDoesNotExist()
        }
    }

    @Test
    fun anUnsavedTurnWordIsMarkedAgainstItsFlashValue() {
        show(turn = listOf(80, 60, 0, 20, 41), flash = START_TURN)

        compose.onNodeWithTag(tuneTurnRowTag(Gains.GH)).performScrollTo()
        inRow(Gains.GH, s(R.string.tune_flash, "50")).assertExists()
        inRow(Gains.GH, " *").assertExists()
        compose.onNodeWithTag(tuneTurnRowTag(Gains.GHV)).performScrollTo()
        inRow(Gains.GHV, " *").assertDoesNotExist()
        compose.onNodeWithText(
            context.resources.getQuantityString(R.plurals.tune_unsaved_count, 1, 1),
        ).performScrollTo().assertExists()
    }

    @Test
    fun theSliderStaysInsideTheCompiledMaximum() {
        val max = Gains.TURN_GAIN_MAXIMA[Gains.GH]
        val r = show()

        compose.onNodeWithTag(tuneTurnSliderTag(Gains.GH)).performScrollTo()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(9_000f) }

        val key = turnKeys[Gains.GH]
        assertTrue(r.calls.toString(), r.calls.contains("end:${key.fieldId}:${key.index}:$max"))
        assertTrue(
            "the slider offered a value past the compiled maximum: ${r.calls}",
            r.calls.all { !it.startsWith("slide:") || it.endsWith(":$max") },
        )
    }

    @Test
    fun aTypedValueOutsideTheCompiledRangeCannotBeSent() {
        val r = show()
        val slewIndex = Gains.TURN_COUNT + Gains.RATE_SLEW
        val slew = compose.onNodeWithTag(tuneTurnEntryTag(slewIndex))

        // rate_slew's floor is 1.
        slew.performScrollTo().performTextClearance()
        slew.performTextInput("0")
        compose.onNodeWithTag(tuneTurnSetTag(slewIndex)).assertIsNotEnabled()

        slew.performScrollTo().performTextClearance()
        slew.performTextInput("60")
        compose.onNodeWithTag(tuneTurnSetTag(slewIndex)).performScrollTo().assertIsEnabled().performClick()

        assertEquals("end:${Gains.CONTROL_TURN_REFERENCE}:${Gains.RATE_SLEW}:60", r.calls.last())
    }

    @Test
    fun armedTheWordsStayLive() {
        show(armed = true)

        compose.onNodeWithText(s(R.string.tune_turn_armed)).performScrollTo().assertExists()
        compose.onNodeWithTag(tuneTurnSliderTag(Gains.GH)).performScrollTo().assertIsEnabled()
    }
}
