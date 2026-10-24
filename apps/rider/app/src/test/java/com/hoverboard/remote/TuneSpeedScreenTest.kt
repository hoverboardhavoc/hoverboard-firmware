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
import com.hoverboard.remote.ui.screens.TUNE_SPEED_STATE_TAG
import com.hoverboard.remote.ui.screens.TUNE_SPEED_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.tuneSpeedEntryTag
import com.hoverboard.remote.ui.screens.tuneSpeedRowTag
import com.hoverboard.remote.ui.screens.tuneSpeedSetTag
import com.hoverboard.remote.ui.screens.tuneSpeedSliderTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says about the outer speed loop (`specs/rider-ui.md` 3.3c): what each of
 * the three words means in the units it is taken in, that `speed_kp` at 0 means the loop is off
 * and the stick commands lean, the compiled range stated as every board's rather than this one's,
 * and the two instruments that cannot send a value outside it.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneSpeedScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keysA = (0 until Gains.PER_PROFILE).map { Gains.key(Gains.CONTROL_GAIN_A, it) }
    private val speedKeys = (0 until Gains.PER_PROFILE).map { Gains.key(Gains.CONTROL_SPEED_LOOP, it) }

    /** Records what the screen asked for. */
    private class Recorder : TuneActions {
        val calls = mutableListOf<String>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(address: Int) { calls += "target:$address" }
        override fun selectProfile(fieldId: Int) { calls += "profile:$fieldId" }
        override fun refresh() { calls += "refresh" }
        override fun slide(key: Key, value: Int) { calls += "slide:${key.fieldId}:${key.index}:$value" }
        override fun slideEnd(key: Key, value: Int) { calls += "end:${key.fieldId}:${key.index}:$value" }
        override fun setBound(key: Key, value: Int) { calls += "bound:${key.fieldId}:${key.index}:$value" }
        override fun save() { calls += "save" }
        override fun revert() { calls += "revert" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    /** A board read this session whose speed row is staged at [speed] and whose flash holds [flash]. */
    private fun show(
        speed: List<Int>?,
        flash: List<Int> = speed.orEmpty(),
        armed: Boolean = false,
    ): Recorder {
        val gainValues = keysA.zip(listOf(6000, 2000, 40)).toMap()
        val gains = BoardGains(
            staged = gainValues + speedKeys.zip(speed.orEmpty()).toMap(),
            flash = gainValues + speedKeys.zip(flash).toMap(),
            stale = false,
        )
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme {
                TuneScreen(
                    state = TuneState(target = 0x01, boards = mapOf(0x01 to gains)),
                    armed = armed,
                    telemetry = null,
                    actions = r,
                )
            }
        }
        return r
    }

    private fun inRow(index: Int, text: String) =
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneSpeedRowTag(index))))

    @Test
    fun theLoopOffAtSpeedKpZeroIsSaidOutright() {
        show(listOf(0, 0, 0))

        compose.onNodeWithTag(TUNE_SPEED_TAG).performScrollTo().assertExists()
        compose.onNodeWithTag(TUNE_SPEED_STATE_TAG).performScrollTo().assertExists()
        compose.onNodeWithText(s(R.string.tune_speed_off)).assertExists()
        compose.onNodeWithText(s(R.string.tune_speed_on)).assertDoesNotExist()
    }

    /** A zero that is not the off switch: `speed_max` 0 is a working setting and the row says so. */
    @Test
    fun aRunningLoopIsSaidToBeRunningEvenWithSpeedMaxAtZero() {
        show(listOf(100, 50, 0))

        compose.onNodeWithText(s(R.string.tune_speed_on)).assertExists()
        compose.onNodeWithText(s(R.string.tune_speed_off)).assertDoesNotExist()
        inRow(Gains.SPEED_MAX, s(R.string.tune_speed_max_note)).assertExists()
    }

    @Test
    fun anUnreadBoardClaimsNothingAboutTheLoop() {
        show(speed = null, flash = listOf(0, 0, 0))

        compose.onNodeWithText(s(R.string.tune_speed_unread)).assertExists()
        compose.onNodeWithTag(tuneSpeedSliderTag(Gains.SPEED_KP)).performScrollTo().assertIsNotEnabled()
    }

    @Test
    fun eachWordSaysWhatItIsInTheUnitsTheLoopTakesIt() {
        show(listOf(100, 50, 80))

        inRow(Gains.SPEED_KP, s(R.string.tune_speed_kp)).assertExists()
        inRow(Gains.SPEED_KP, s(R.string.tune_speed_kp_note)).assertExists()
        inRow(Gains.SPEED_KI, s(R.string.tune_speed_ki)).assertExists()
        inRow(Gains.SPEED_MAX, s(R.string.tune_speed_max)).assertExists()
    }

    /** The range is the compiled one, stated as every board's, and the slider cannot leave it. */
    @Test
    fun theRangeIsTheCompiledOneAndTheSliderCannotPassIt() {
        val max = Gains.SPEED_MAXIMA[Gains.SPEED_KP]
        val r = show(listOf(100, 50, 80))

        inRow(Gains.SPEED_KP, s(R.string.tune_speed_range, max.toString())).assertExists()
        compose.onNodeWithTag(tuneSpeedSliderTag(Gains.SPEED_KP)).performScrollTo()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(9_000f) }

        assertTrue(
            "the slider offered a value past the compiled maximum: ${r.calls}",
            r.calls.all { !it.startsWith("slide:") || it.endsWith(":$max") },
        )
    }

    @Test
    fun aTypedValuePastTheCompiledMaximumCannotBeSent() {
        val max = Gains.SPEED_MAXIMA[Gains.SPEED_MAX]
        val r = show(listOf(100, 50, 80))
        val entry = compose.onNodeWithTag(tuneSpeedEntryTag(Gains.SPEED_MAX))

        entry.performScrollTo().performTextClearance()
        entry.performTextInput("9000")
        compose.onNodeWithTag(tuneSpeedSetTag(Gains.SPEED_MAX)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.tune_speed_entry_invalid, max.toString())).assertExists()

        entry.performTextClearance()
        entry.performTextInput("160")
        compose.onNodeWithTag(tuneSpeedSetTag(Gains.SPEED_MAX)).assertIsEnabled().performClick()

        assertEquals("end:${Gains.CONTROL_SPEED_LOOP}:${Gains.SPEED_MAX}:160", r.calls.last())
    }

    @Test
    fun anUnsavedSpeedWordIsMarkedAgainstItsFlashValue() {
        show(listOf(100, 50, 80), flash = listOf(0, 0, 0))

        inRow(Gains.SPEED_KP, s(R.string.tune_flash, "0")).assertExists()
        inRow(Gains.SPEED_KP, " *").assertExists()
        compose.onNodeWithText(
            context.resources.getQuantityString(R.plurals.tune_unsaved_count, 3, 3),
        ).performScrollTo().assertExists()
    }

    @Test
    fun armedTheWordsStayLiveAndTheScreenSaysWhatChangingSpeedKpCosts() {
        show(listOf(100, 50, 80), armed = true)

        compose.onNodeWithTag(tuneSpeedSliderTag(Gains.SPEED_KP)).performScrollTo().assertIsEnabled()
        compose.onNodeWithText(s(R.string.tune_speed_armed)).assertExists()
    }
}
