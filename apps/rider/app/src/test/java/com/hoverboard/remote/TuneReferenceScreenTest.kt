package com.hoverboard.remote

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
import com.hoverboard.remote.ui.screens.TUNE_REFERENCE_STATE_TAG
import com.hoverboard.remote.ui.screens.TUNE_REFERENCE_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.tuneReferenceEntryTag
import com.hoverboard.remote.ui.screens.tuneReferenceRowTag
import com.hoverboard.remote.ui.screens.tuneReferenceSetTag
import com.hoverboard.remote.ui.screens.tuneReferenceSliderTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says about the reference row (`specs/state-feedback.md` sections 4 and 10):
 * what `speed_max` and `speed_slew` mean in their units, that `speed_max` 0 is a dead stick and a
 * held position, the compiled range stated as every board's, and the two instruments that cannot
 * send a value outside it.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneReferenceScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val gainKeys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }
    private val referenceKeys = Gains.REFERENCE_DEFAULT.indices.map { Gains.key(Gains.CONTROL_REFERENCE, it) }

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

    /** A board read this session whose reference row is staged at [reference] and whose flash holds [flash]. */
    private fun show(
        reference: List<Int>?,
        flash: List<Int> = reference.orEmpty(),
        armed: Boolean = false,
    ): Recorder {
        val gainValues = gainKeys.zip(listOf(3491, 4002, 197, 926)).toMap()
        val gains = BoardGains(
            staged = gainValues + referenceKeys.zip(reference.orEmpty()).toMap(),
            flash = gainValues + referenceKeys.zip(flash).toMap(),
            stale = false,
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
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneReferenceRowTag(index))))

    @Test
    fun speedMaxZeroIsSaidOutrightToBeADeadStickThatHoldsPosition() {
        show(listOf(0, 41))

        compose.onNodeWithTag(TUNE_REFERENCE_TAG).performScrollTo().assertExists()
        compose.onNodeWithTag(TUNE_REFERENCE_STATE_TAG).performScrollTo().assertExists()
        val dead = s(R.string.tune_reference_dead)
        compose.onNodeWithText(dead).assertExists()
        assertTrue(dead, dead.contains("the stick does nothing") && dead.contains("holds position"))
        compose.onNodeWithText(s(R.string.tune_reference_live)).assertDoesNotExist()
    }

    @Test
    fun aNonzeroSpeedMaxIsALiveStick() {
        show(listOf(80, 41))

        compose.onNodeWithText(s(R.string.tune_reference_live)).assertExists()
        compose.onNodeWithText(s(R.string.tune_reference_dead)).assertDoesNotExist()
    }

    @Test
    fun anUnreadBoardClaimsNothingAboutTheStick() {
        show(reference = null, flash = listOf(0, 41))

        compose.onNodeWithText(s(R.string.tune_reference_unread)).assertExists()
        compose.onNodeWithTag(tuneReferenceSliderTag(Gains.SPEED_MAX)).performScrollTo().assertIsNotEnabled()
    }

    @Test
    fun eachWordSaysWhatItIsInItsUnits() {
        show(listOf(80, 41))

        inRow(Gains.SPEED_MAX, s(R.string.tune_reference_speed_max)).assertExists()
        inRow(Gains.SPEED_MAX, s(R.string.tune_reference_speed_max_note)).assertExists()
        inRow(Gains.SPEED_SLEW, s(R.string.tune_reference_speed_slew)).assertExists()
        inRow(Gains.SPEED_SLEW, s(R.string.tune_reference_speed_slew_note)).assertExists()
    }

    /** The range is the compiled one, stated as every board's, and the slider cannot leave it. */
    @Test
    fun theRangeIsTheCompiledOneAndTheSliderCannotPassIt() {
        val min = Gains.REFERENCE_MINIMA[Gains.SPEED_SLEW]
        val max = Gains.REFERENCE_MAXIMA[Gains.SPEED_SLEW]
        val r = show(listOf(80, 41))

        inRow(Gains.SPEED_SLEW, s(R.string.tune_reference_range, min.toString(), max.toString())).assertExists()
        compose.onNodeWithTag(tuneReferenceSliderTag(Gains.SPEED_SLEW)).performScrollTo()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(9_000f) }

        assertTrue(
            "the slider offered a value past the compiled maximum: ${r.calls}",
            r.calls.all { !it.startsWith("slide:") || it.endsWith(":$max") },
        )
    }

    @Test
    fun aTypedValueOutsideTheCompiledRangeCannotBeSent() {
        val max = Gains.REFERENCE_MAXIMA[Gains.SPEED_MAX]
        val r = show(listOf(80, 41))
        val entry = compose.onNodeWithTag(tuneReferenceEntryTag(Gains.SPEED_MAX))

        entry.performScrollTo().performTextClearance()
        entry.performTextInput("9000")
        compose.onNodeWithTag(tuneReferenceSetTag(Gains.SPEED_MAX)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.tune_reference_entry_invalid, "0", max.toString())).assertExists()

        // speed_slew's floor is 1: a 0 is outside its range too.
        val slew = compose.onNodeWithTag(tuneReferenceEntryTag(Gains.SPEED_SLEW))
        slew.performScrollTo().performTextClearance()
        slew.performTextInput("0")
        compose.onNodeWithTag(tuneReferenceSetTag(Gains.SPEED_SLEW)).assertIsNotEnabled()

        entry.performScrollTo().performTextClearance()
        entry.performTextInput("120")
        compose.onNodeWithTag(tuneReferenceSetTag(Gains.SPEED_MAX)).assertIsEnabled().performClick()

        assertEquals("end:${Gains.CONTROL_REFERENCE}:${Gains.SPEED_MAX}:120", r.calls.last())
    }

    @Test
    fun anUnsavedReferenceWordIsMarkedAgainstItsFlashValue() {
        show(listOf(80, 41), flash = listOf(0, 41))

        inRow(Gains.SPEED_MAX, s(R.string.tune_flash, "0")).assertExists()
        inRow(Gains.SPEED_MAX, " *").assertExists()
        inRow(Gains.SPEED_SLEW, " *").assertDoesNotExist()
        compose.onNodeWithText(
            context.resources.getQuantityString(R.plurals.tune_unsaved_count, 1, 1),
        ).performScrollTo().assertExists()
    }

    @Test
    fun armedTheWordsStayLive() {
        show(listOf(80, 41), armed = true)

        compose.onNodeWithTag(tuneReferenceSliderTag(Gains.SPEED_MAX)).performScrollTo().assertIsEnabled()
        compose.onNodeWithText(s(R.string.tune_reference_armed)).assertExists()
    }
}
