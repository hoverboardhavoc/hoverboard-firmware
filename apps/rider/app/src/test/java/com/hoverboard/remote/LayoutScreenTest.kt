package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
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
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.board.BoardField
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.FieldRef
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPreset
import com.hoverboard.protocol.board.LayoutPresets
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.Pin
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ui.screens.LAYOUT_APPLY_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LATCH_CONFIRM_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LATCH_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LOCK_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_POWER_CYCLE_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_STORED_INVALID_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_VERDICT_TAG
import com.hoverboard.remote.ui.screens.LayoutScreen
import com.hoverboard.remote.ui.screens.layoutChangeTag
import com.hoverboard.remote.ui.screens.layoutPartTag
import com.hoverboard.remote.ui.screens.layoutPinTag
import com.hoverboard.remote.ui.screens.layoutPresetTag
import com.hoverboard.remote.ui.screens.layoutRowTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the layout editor says (`specs/rider-ui.md` section 3.5): the verdict in the user's terms,
 * Apply refused outright while that verdict is a refusal, a picker that offers only pins the board
 * would accept, and the power latch shown but never offered.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class LayoutScreenTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    /** Records what the screen asked for. */
    private class Recorder : LayoutActions {
        val calls = mutableListOf<String>()
        val staged = mutableListOf<Pair<Key, Int>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun refresh() { calls += "refresh" }
        override fun selectPart(part: ChipFamily) { calls += "part:${part.name}" }
        override fun stage(slot: LayoutSlot, raw: Int) { staged += slot.key to raw }
        override fun stagePreset(preset: LayoutPreset) { calls += "preset:${preset.name}" }
        override fun confirmLatchChange() { calls += "confirmLatch" }
        override fun cancelLatchChange() { calls += "cancelLatch" }
        override fun revert(slot: LayoutSlot) { calls += "revert:${slot.key}" }
        override fun revertAll() { calls += "revertAll" }
        override fun apply() { calls += "apply" }
        override fun confirmPowerCycled() { calls += "confirmPowerCycled" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    private val values: Map<Key, Value> =
        Layout.SLOTS.associate { it.key to it.def.default } + (Fields.LINK_SET.key() to Value.U8(0))

    private val blank: BoardFields = checkNotNull(Layout.fieldsFrom(values))

    private fun slot(field: BoardField, motor: Int? = null): LayoutSlot =
        checkNotNull(Layout.forField(FieldRef(field, motor)))

    private fun pin(name: String) = checkNotNull(Pin.byName(name)).packed

    private fun state(
        part: ChipFamily? = ChipFamily.F103C8,
        staged: BoardFields? = blank,
        written: Set<Key> = emptySet(),
        linkDropped: Boolean = false,
    ) = LayoutState(
        board = 0x01,
        part = part,
        values = values,
        linkSet = 0,
        staged = staged,
        written = written,
        linkDroppedSinceApply = linkDropped,
    )

    private fun show(state: LayoutState, armed: Boolean = false, actions: LayoutActions = Recorder()) =
        compose.setContent {
            HoverboardRemoteTheme { LayoutScreen(state = state, armed = armed, actions = actions) }
        }

    @Test
    fun aCleanVerdictSaysSoAndSaysWhatOnlyARebootCanTell() {
        show(state())

        compose.onNodeWithTag(LAYOUT_VERDICT_TAG).assertExists()
        compose.onNodeWithText(s(R.string.layout_verdict_valid)).assertExists()
        // The claim that earns the screen: this is the board's own function, not a heuristic.
        compose.onNodeWithText(s(R.string.layout_verdict_exact)).assertExists()
        // And its limit, stated next to it.
        compose.onNodeWithText(s(R.string.layout_verdict_after_boot)).assertExists()
        compose.onNodeWithText(s(R.string.layout_verdict_reachable)).assertExists()
    }

    @Test
    fun aRefusalNamesTheFieldAndTheReasonAndBlocksApply() {
        // A layout that would steal the inter-board link's pin.
        val staged = slot(BoardField.BUZZER).on(blank, pin("PA2"))
        val actions = Recorder()
        show(state(staged = staged), actions = actions)

        compose.onNodeWithText(
            s(
                R.string.layout_verdict_invalid,
                s(R.string.layout_field_buzzer),
                s(R.string.layout_reason_reserved, "PA2"),
            ),
        ).assertExists()
        compose.onNodeWithTag(LAYOUT_APPLY_TAG).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.layout_apply_blocked)).assertExists()
        assertEquals("nothing was asked of the model", emptyList<String>(), actions.calls - "shown")
    }

    @Test
    fun aCleanDeltaOffersApply() {
        val staged = slot(BoardField.BUZZER).on(blank, pin("PB5"))
        val actions = Recorder()
        show(state(staged = staged), actions = actions)

        compose.onNodeWithText(
            s(R.string.layout_delta_line, s(R.string.layout_field_buzzer), "PB9", "PB5"),
        ).assertExists()
        compose.onNodeWithTag(LAYOUT_APPLY_TAG).performScrollTo().assertIsEnabled().performClick()
        assertEquals(listOf("shown", "apply"), actions.calls)
    }

    @Test
    fun withoutAPartThereIsNoVerdictToShow() {
        val actions = Recorder()
        show(state(part = null), actions = actions)

        compose.onNodeWithText(s(R.string.layout_verdict_unknown)).assertExists()
        compose.onNodeWithTag(layoutPartTag(ChipFamily.F130C8)).performScrollTo().performClick()
        assertEquals(listOf("shown", "part:F130C8"), actions.calls)
    }

    @Test
    fun thePickerOffersOnlyPinsTheBoardWouldAccept() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(BoardField.LED_GREEN).key

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()

        // Free, bonded, not gate-capable: offered.
        compose.onNodeWithTag(layoutPinTag(key, pin("PB5"))).assertExists()
        // A live link port's pin, another function's pin, an advanced-timer pin, and a pin this part
        // does not bond: each withheld, which is what makes the invalid layout hard to express.
        compose.onNodeWithTag(layoutPinTag(key, pin("PA2"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutPinTag(key, pin("PB9"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutPinTag(key, pin("PA8"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutPinTag(key, pin("PF0"))).assertDoesNotExist()
        // Unsetting the function is always available: an absent function is a valid board state.
        compose.onNodeWithTag(layoutPinTag(key, PIN_ABSENT)).assertExists()

        compose.onNodeWithTag(layoutPinTag(key, pin("PB5"))).performScrollTo().performClick()
        assertEquals(listOf(key to pin("PB5")), actions.staged)
    }

    @Test
    fun theGateSlotsOfferThePartsTimerPinsAndTheOthersDoNot() {
        show(state())
        val key = slot(BoardField.GATE_HI_A, 0).key

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()

        for (packed in ChipFamily.GATES_T0_HI + ChipFamily.GATES_T0_LO) {
            compose.onNodeWithTag(layoutPinTag(key, packed)).assertExists()
        }
        compose.onNodeWithTag(layoutPinTag(key, pin("PB5"))).assertDoesNotExist()
    }

    @Test
    fun thePowerLatchIsShownAndNeverOffered() {
        show(state())
        val key = slot(BoardField.SELF_HOLD).key

        compose.onNodeWithTag(layoutRowTag(key)).performScrollTo().assertIsDisplayed()
        compose.onNode(hasText("PB12") and hasAnyAncestor(hasTestTag(layoutRowTag(key)))).assertExists()
        compose.onNodeWithTag(layoutChangeTag(key)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.layout_note_self_hold)).assertExists()
    }

    @Test
    fun armedShowsTheLockAndDisablesThePickers() {
        show(state(), armed = true)

        compose.onNodeWithTag(LAYOUT_LOCK_TAG).assertIsDisplayed()
        compose.onNodeWithTag(layoutChangeTag(slot(BoardField.LED_GREEN).key)).performScrollTo().assertIsNotEnabled()
    }

    @Test
    fun aHalfWrittenLayoutIsReportedAsAnInvalidBoard() {
        // The board holds one hall of three: not progress towards a layout, an invalid board. The
        // staged layout completes it, so the verdict above is clean and this warning is the one that
        // describes the board.
        val partial = values + (Fields.MOTOR_HALL_A.key(0) to Value.U8(pin("PC13")))
        val whole = slot(BoardField.HALL_B, 0).on(
            slot(BoardField.HALL_C, 0).on(checkNotNull(Layout.fieldsFrom(partial)), pin("PC14")),
            pin("PA1"),
        )
        show(
            LayoutState(
                board = 0x01,
                part = ChipFamily.F103C8,
                values = partial,
                linkSet = 0,
                staged = whole,
                written = setOf(Fields.MOTOR_HALL_A.key(0)),
            ),
        )

        compose.onNodeWithTag(LAYOUT_STORED_INVALID_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_stored_invalid, s(R.string.layout_field_hall_b))).assertExists()
        compose.onNodeWithText(s(R.string.layout_verdict_valid)).assertExists()
    }

    @Test
    fun aPresetIsOneTapAndSaysWhatItDoesNotTouch() {
        val actions = Recorder()
        show(state(), actions = actions)

        compose.onNodeWithText(s(R.string.layout_preset_body)).assertExists()
        compose.onNodeWithTag(layoutPresetTag(LayoutPresets.BENCH_MASTER)).performScrollTo().performClick()

        assertEquals(listOf("shown", "preset:${LayoutPresets.BENCH_MASTER.name}"), actions.calls)
    }

    @Test
    fun aLatchChangeIsConfirmedInWordsOrDeclined() {
        // The consequence is specific, so the panel states it rather than warning in the abstract.
        val actions = Recorder()
        show(state().copy(pendingLatch = pin("PB5")), actions = actions)

        compose.onNodeWithTag(LAYOUT_LATCH_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_latch_body, "PB12", "PB5")).assertExists()
        compose.onNodeWithText(s(R.string.layout_latch_cancel, "PB12")).assertExists()
        compose.onNodeWithTag(LAYOUT_LATCH_CONFIRM_TAG).performScrollTo().performClick()

        assertEquals(listOf("shown", "confirmLatch"), actions.calls)
    }

    @Test
    fun aWrittenLayoutAsksForOnePowerCycle() {
        val actions = Recorder()
        show(
            state(written = setOf(Fields.BOARD_BUZZER.key()), linkDropped = true),
            actions = actions,
        )

        compose.onNodeWithTag(LAYOUT_POWER_CYCLE_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_power_cycle, 1)).assertExists()
        compose.onNodeWithText(s(R.string.layout_power_cycled)).performClick()
        assertEquals(listOf("shown", "confirmPowerCycled"), actions.calls)
    }
}
