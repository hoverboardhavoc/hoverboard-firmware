package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.semantics.SemanticsActions
import androidx.compose.ui.semantics.SemanticsProperties
import androidx.compose.ui.semantics.getOrNull
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertTextEquals
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performSemanticsAction
import androidx.compose.ui.test.performTextClearance
import androidx.compose.ui.test.performTextInput
import androidx.compose.ui.test.performTouchInput
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.ui.screens.GAIN_BACK_OFF_PERCENTS
import com.hoverboard.remote.ui.screens.GAIN_SLIDER_SPANS
import com.hoverboard.remote.ui.screens.GAIN_STEPS
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.gainBackedOff
import com.hoverboard.remote.ui.screens.gainSliderTop
import com.hoverboard.remote.ui.screens.gainStepped
import com.hoverboard.remote.ui.screens.tuneBackOffTag
import com.hoverboard.remote.ui.screens.tuneEntryTag
import com.hoverboard.remote.ui.screens.tunePendingTag
import com.hoverboard.remote.ui.screens.tuneRowTag
import com.hoverboard.remote.ui.screens.tuneSetTag
import com.hoverboard.remote.ui.screens.tuneSliderTag
import com.hoverboard.remote.ui.screens.tuneStagedTag
import com.hoverboard.remote.ui.screens.tuneStepDownTag
import com.hoverboard.remote.ui.screens.tuneStepUpTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * A gain row's instruments: the pending value shown beside the read-back while it is on its way,
 * the minus and plus steps, the typed entry, and the slider over a useful span that never passes
 * the board's maximum. Every one ends in the release call ([TuneActions.slideEnd]) the slider's
 * release makes, so each works armed as the slider does.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneGainRowTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }
    private val f = Gains.CONTROL_SF_GAINS

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

    private fun state(staged: List<Int> = START, maxima: Map<Int, Int> = emptyMap()) = TuneState(
        target = 0x01,
        boards = mapOf(
            0x01 to BoardGains(
                staged = keys.zip(staged).toMap(),
                flash = keys.zip(START).toMap(),
                stale = false,
                maxima = maxima,
            ),
        ),
    )

    private val shown = mutableStateOf(state())

    private fun show(s: TuneState = state(), armed: Boolean = false): Recorder {
        shown.value = s
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme { TuneScreen(state = shown.value, armed = armed, actions = r) }
        }
        return r
    }

    private fun pending(index: Int) = compose.onNodeWithTag(tunePendingTag(index), useUnmergedTree = true)

    /** The number after the arrow of gain [index]'s pending readout. */
    private fun pendingValue(index: Int): Int {
        val text = pending(index).fetchSemanticsNode().config.getOrNull(SemanticsProperties.Text)
            ?.joinToString("") { it.text }.orEmpty()
        return text.substringAfter("→").trim().toInt()
    }

    @Test
    fun aDragShowsTheDraggedValueBesideTheReadBackWhichStaysTheBoards() {
        val r = show()
        val slider = compose.onNodeWithTag(tuneSliderTag(Gains.GP))

        slider.performTouchInput {
            down(center)
            moveBy(Offset(width / 4f, 0f))
        }

        val dragged = pendingValue(Gains.GP)
        assertTrue("dragged to $dragged", dragged in 1..GAIN_SLIDER_SPANS[Gains.GP])
        compose.onNodeWithTag(tuneStagedTag(Gains.GP)).assertTextEquals(START[Gains.GP].toString())
        assertEquals("slide:$f:${Gains.GP}:$dragged", r.calls.last())

        slider.performTouchInput { up() }
        assertEquals("end:$f:${Gains.GP}:$dragged", r.calls.last())

        // The board reports the value: the read-back takes over and the pending one goes.
        shown.value = state(staged = START.toMutableList().also { it[Gains.GP] = dragged })
        compose.waitForIdle()
        pending(Gains.GP).assertDoesNotExist()
        compose.onNodeWithTag(tuneStagedTag(Gains.GP)).assertTextEquals(dragged.toString())
    }

    /** A value the board never reports does not stay on screen as if it were on its way. */
    @Test
    fun aPendingValueTheBoardNeverReportsGivesWay() {
        show()
        compose.onNodeWithTag(tuneStepUpTag(Gains.GP)).performClick()
        pending(Gains.GP).assertTextEquals(s(R.string.tune_pending, "3541"))

        compose.mainClock.advanceTimeBy(2_000)
        pending(Gains.GP).assertDoesNotExist()
    }

    @Test
    fun theStepButtonsStepFromThePendingValueAndStayInsideTheBoardsMaximum() {
        val r = show(state(maxima = mapOf(Gains.GX to 200)))

        // Two taps before any read-back: the second steps from the first, not from staged again.
        compose.onNodeWithTag(tuneStepUpTag(Gains.GP)).performClick()
        compose.onNodeWithTag(tuneStepUpTag(Gains.GP)).performClick()
        val gp = START[Gains.GP]
        val step = GAIN_STEPS[Gains.GP]
        assertEquals(
            listOf("end:$f:${Gains.GP}:${gp + step}", "end:$f:${Gains.GP}:${gp + 2 * step}"),
            r.calls.filter { it.startsWith("end:$f:${Gains.GP}:") },
        )

        // gx at 197 under a maximum of 200: plus sends 200, not 202, and is then off.
        val up = compose.onNodeWithTag(tuneStepUpTag(Gains.GX)).performScrollTo()
        up.performClick()
        assertEquals("end:$f:${Gains.GX}:200", r.calls.last())
        up.assertIsNotEnabled()
        compose.onNodeWithTag(tuneStepDownTag(Gains.GX)).performClick()
        assertEquals("end:$f:${Gains.GX}:${200 - GAIN_STEPS[Gains.GX]}", r.calls.last())
    }

    @Test
    fun minusIsOffAtZero() {
        show(state(staged = listOf(3491, 4002, 197, 0)))
        compose.onNodeWithTag(tuneStepDownTag(Gains.GV)).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(tuneStepUpTag(Gains.GV)).assertIsEnabled()
    }

    /** The entry reaches the board's maximum past the slider's span, and nothing past it. */
    @Test
    fun theTypedEntryTakesAnyValueUpToTheBoardsMaximumAndNoMore() {
        val r = show()
        val entry = compose.onNodeWithTag(tuneEntryTag(Gains.GP)).performScrollTo()
        val set = compose.onNodeWithTag(tuneSetTag(Gains.GP))
        val max = Gains.DEFAULT_MAX[Gains.GP]

        entry.performTextClearance()
        entry.performTextInput((max + 1).toString())
        set.assertIsNotEnabled()
        compose.onNode(
            hasText(s(R.string.tune_reference_entry_invalid, "0", max.toString())) and
                hasAnyAncestor(hasTestTag(tuneRowTag(Gains.GP))),
        ).assertExists()

        entry.performTextClearance()
        entry.performTextInput("abc")
        set.assertIsNotEnabled()

        entry.performTextClearance()
        entry.performTextInput("15000")
        set.assertIsEnabled().performClick()
        assertEquals("end:$f:${Gains.GP}:15000", r.calls.last())
    }

    @Test
    fun theTypedEntryFollowsAMaximumReadFromTheBoard() {
        show(state(maxima = mapOf(Gains.GP to 6050)))
        val entry = compose.onNodeWithTag(tuneEntryTag(Gains.GP)).performScrollTo()
        entry.performTextClearance()
        entry.performTextInput("7000")
        compose.onNodeWithTag(tuneSetTag(Gains.GP)).assertIsNotEnabled()
    }

    /** The slider covers the useful span, and says the other two instruments go further. */
    @Test
    fun theSliderCoversTheUsefulSpanAndNeverPassesTheBoardsMaximum() {
        val r = show()
        compose.onNodeWithTag(tuneSliderTag(Gains.GP))
            .performSemanticsAction(SemanticsActions.SetProgress) { it(20_000f) }
        assertEquals("end:$f:${Gains.GP}:${GAIN_SLIDER_SPANS[Gains.GP]}", r.calls.last())
        compose.onNode(
            hasText(s(R.string.tune_slider_span, GAIN_SLIDER_SPANS[Gains.GP].toString())) and
                hasAnyAncestor(hasTestTag(tuneRowTag(Gains.GP))),
        ).assertExists()
    }

    @Test
    fun aBoardMaximumBelowTheSpanCutsTheSlider() {
        val low = show(state(maxima = mapOf(Gains.GP to 6050)))
        compose.onNodeWithTag(tuneSliderTag(Gains.GP))
            .performSemanticsAction(SemanticsActions.SetProgress) { it(20_000f) }
        assertEquals("end:$f:${Gains.GP}:6050", low.calls.last())
    }

    @Test
    fun everySpanIsInsideTheDefaultMaximumAndTheTopIsTheLowerOfTheTwo() {
        assertEquals(Gains.COUNT, GAIN_SLIDER_SPANS.size)
        assertEquals(Gains.COUNT, GAIN_STEPS.size)
        for (i in 0 until Gains.COUNT) {
            assertTrue("span $i", GAIN_SLIDER_SPANS[i] in 1..Gains.DEFAULT_MAX[i])
            assertTrue("step $i", GAIN_STEPS[i] in 1..GAIN_SLIDER_SPANS[i])
            assertEquals(GAIN_SLIDER_SPANS[i], gainSliderTop(i, Gains.DEFAULT_MAX[i]))
            assertEquals(100, gainSliderTop(i, 100))
        }
        assertEquals(0, gainStepped(3, -5, 100))
        assertEquals(100, gainStepped(98, 5, 100))
        assertEquals(0, gainStepped(10, 5, -1))
    }

    @Test
    fun theBackOffValueRoundsToTheNearestCountAndStaysInsideTheBoardsRange() {
        assertEquals(listOf(75, 65, null, null), GAIN_BACK_OFF_PERCENTS)
        assertEquals(2618, gainBackedOff(3491, 75, 20_000)) // 2618.25
        assertEquals(2601, gainBackedOff(4002, 65, 20_000)) // 2601.3
        assertEquals(2, gainBackedOff(2, 75, 100)) // 1.5 rounds up
        assertEquals(0, gainBackedOff(0, 65, 100))
        assertEquals(1000, gainBackedOff(20_000, 75, 1000))
        assertEquals(0, gainBackedOff(100, 75, -1))
    }

    /** Only the rows the guide gives an onset rule carry the button. */
    @Test
    fun onlyTheGpAndGrRowsCarryABackOffButton() {
        show()
        compose.onNodeWithTag(tuneBackOffTag(Gains.GP)).performScrollTo()
            .assertTextEquals(s(R.string.tune_back_off, 75))
        compose.onNodeWithTag(tuneBackOffTag(Gains.GR)).performScrollTo()
            .assertTextEquals(s(R.string.tune_back_off, 65))
        compose.onNodeWithTag(tuneBackOffTag(Gains.GX)).assertDoesNotExist()
        compose.onNodeWithTag(tuneBackOffTag(Gains.GV)).assertDoesNotExist()
    }

    /** A press takes its share of the board's read-back, not of a pending value, and shows it pending. */
    @Test
    fun backOffTakesItsShareOfTheReadBackNotThePendingValue() {
        val r = show()
        compose.onNodeWithTag(tuneStepUpTag(Gains.GP)).performClick()
        compose.onNodeWithTag(tuneBackOffTag(Gains.GP)).performScrollTo().performClick()
        val sent = gainBackedOff(START[Gains.GP], 75, Gains.DEFAULT_MAX[Gains.GP])
        assertEquals("end:$f:${Gains.GP}:$sent", r.calls.last())
        pending(Gains.GP).assertTextEquals(s(R.string.tune_pending, sent.toString()))
    }

    @Test
    fun backOffHoldsTheBoardsMaximum() {
        val r = show(state(staged = listOf(3491, 4002, 197, 926), maxima = mapOf(Gains.GR to 2000)))
        compose.onNodeWithTag(tuneBackOffTag(Gains.GR)).performScrollTo().performClick()
        assertEquals("end:$f:${Gains.GR}:2000", r.calls.last())
    }

    @Test
    fun backOffIsOffWhileTheValueIsUnread() {
        val unread = TuneState(
            target = 0x01,
            boards = mapOf(
                0x01 to BoardGains(
                    staged = mapOf(keys[Gains.GR] to 4002),
                    flash = keys.zip(START).toMap(),
                    stale = false,
                    maxima = emptyMap(),
                ),
            ),
        )
        show(unread)
        compose.onNodeWithTag(tuneBackOffTag(Gains.GP)).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(tuneBackOffTag(Gains.GR)).performScrollTo().assertIsEnabled()
    }

    @Test
    fun backOffIsOffWhileTheBoardIsNotTunable() {
        show(state().copy(boards = state().boards.mapValues { it.value.copy(stale = true) }))
        compose.onNodeWithTag(tuneBackOffTag(Gains.GP)).performScrollTo().assertIsNotEnabled()
    }

    @Test
    fun armedBackOffSendsOnTheLiveLane() {
        val r = show(armed = true)
        compose.onNodeWithTag(tuneBackOffTag(Gains.GR)).performScrollTo().assertIsEnabled().performClick()
        val sent = gainBackedOff(START[Gains.GR], 65, Gains.DEFAULT_MAX[Gains.GR])
        assertEquals("end:$f:${Gains.GR}:$sent", r.calls.last())
    }

    /** Armed, the buttons and the entry are live and send on the same lane as the slider. */
    @Test
    fun armedTheStepsAndTheEntryWorkAsTheSliderDoes() {
        val r = show(armed = true)
        compose.onNodeWithTag(tuneStepDownTag(Gains.GR)).performScrollTo().assertIsEnabled().performClick()
        assertEquals("end:$f:${Gains.GR}:${START[Gains.GR] - GAIN_STEPS[Gains.GR]}", r.calls.last())

        val entry = compose.onNodeWithTag(tuneEntryTag(Gains.GR)).performScrollTo()
        entry.performTextClearance()
        entry.performTextInput("3000")
        compose.onNodeWithTag(tuneSetTag(Gains.GR)).assertIsEnabled().performClick()
        assertEquals("end:$f:${Gains.GR}:3000", r.calls.last())
    }
}

/** The section-7 start gains, `[gp, gr, gx, gv]`, as a board staged for the rover shows them. */
private val START = listOf(3491, 4002, 197, 926)
