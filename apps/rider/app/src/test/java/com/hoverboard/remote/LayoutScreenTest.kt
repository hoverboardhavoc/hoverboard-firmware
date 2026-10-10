package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasSetTextAction
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performTextReplacement
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.board.BoardField
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.FieldRef
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPreset
import com.hoverboard.protocol.board.LayoutPresets
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.Pin
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.LayoutGroup
import com.hoverboard.remote.ui.screens.LAYOUT_ADD_MOTOR_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_APPLY_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LATCH_CONFIRM_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LATCH_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_LOCK_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_PART_UNKNOWN_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_POWER_CYCLE_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_STORED_INVALID_TAG
import com.hoverboard.remote.ui.screens.LAYOUT_VERDICT_TAG
import com.hoverboard.remote.ui.screens.LayoutScreen
import com.hoverboard.remote.ui.screens.layoutChangeTag
import com.hoverboard.remote.ui.screens.layoutDialogTag
import com.hoverboard.remote.ui.screens.layoutOptionTag
import com.hoverboard.remote.ui.screens.layoutPresetTag
import com.hoverboard.remote.ui.screens.layoutRowTag
import com.hoverboard.remote.ui.screens.layoutTabTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the layout editor says (`specs/rider-ui.md` sections 3.5 and 3.5a): the verdict in the
 * user's terms, Apply refused outright while that verdict is a refusal, a picker that offers only
 * pins the board would accept, the power latch shown but never offered, and the editing done in
 * dialogs from tabbed groups rather than in the rows.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class LayoutScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    /** Records what the screen asked for. */
    private class Recorder : LayoutActions {
        val calls = mutableListOf<String>()
        val staged = mutableListOf<Pair<Key, Int>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun refresh() { calls += "refresh" }
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
        chip: ChipTag? = ChipTag.F103C8,
        staged: BoardFields? = blank,
        written: Set<Key> = emptySet(),
        linkDropped: Boolean = false,
    ) = LayoutState(
        board = 0x01,
        chip = chip,
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
    fun aBoardThatSendsNoPartGetsNoVerdictAndNoWayToStateOne() {
        val actions = Recorder()
        show(state(chip = null), actions = actions)

        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_part_unsent)).assertExists()
        compose.onNodeWithText(s(R.string.layout_part_unknown_body)).assertExists()
        // No verdict of any kind, clean or refused, and nothing to pick a part with.
        compose.onNodeWithTag(LAYOUT_VERDICT_TAG).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.layout_verdict_valid)).assertDoesNotExist()
        assertEquals("nothing was asked of the model", listOf("shown"), actions.calls)
    }

    @Test
    fun aPartTheAppDoesNotKnowSaysThatRatherThanTheOtherReason() {
        show(state(chip = ChipTag.Unknown))

        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_part_unnamed)).assertExists()
        compose.onNodeWithText(s(R.string.layout_part_unsent)).assertDoesNotExist()
        compose.onNodeWithTag(LAYOUT_VERDICT_TAG).assertDoesNotExist()
    }

    @Test
    fun aKnownPartWithNoLayoutReadYetHasNothingToJudge() {
        // The part is the board's and the layout is not in hand, so the panel says which half is
        // missing rather than reporting a verdict it cannot have.
        show(state(staged = null))

        compose.onNodeWithTag(LAYOUT_VERDICT_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_verdict_unknown)).assertExists()
    }

    @Test
    fun aKnownPartIsStatedAsTheBoardsOwn() {
        show(state())

        compose.onNodeWithText(s(R.string.layout_part, ChipFamily.F103C8.label)).assertExists()
        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).assertDoesNotExist()
    }

    @Test
    fun thePickerOffersOnlyPinsTheBoardWouldAccept() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(BoardField.LED_GREEN).key

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutDialogTag(key)).assertExists()

        // Free, bonded, not gate-capable: offered.
        compose.onNodeWithTag(layoutOptionTag(key, pin("PB5"))).assertExists()
        // A live link port's pin, another function's pin, an advanced-timer pin, and a pin this part
        // does not bond: each withheld, which is what makes the invalid layout hard to express.
        compose.onNodeWithTag(layoutOptionTag(key, pin("PA2"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, pin("PB9"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, pin("PA8"))).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, pin("PF0"))).assertDoesNotExist()
        // Unsetting the function is always available: an absent function is a valid board state.
        compose.onNodeWithTag(layoutOptionTag(key, PIN_ABSENT)).assertExists()

        compose.onNodeWithTag(layoutOptionTag(key, pin("PB5"))).performScrollTo().performClick()
        assertEquals(listOf(key to pin("PB5")), actions.staged)
        // Tapping one stages it and closes, which is the simple-dialog pattern: no confirm step.
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun theGateSlotsOfferThePartsTimerPinsAndTheOthersDoNot() {
        show(state())
        val key = slot(BoardField.GATE_HI_A, 0).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 0)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()

        for (packed in ChipFamily.GATES_T0_HI + ChipFamily.GATES_T0_LO) {
            compose.onNodeWithTag(layoutOptionTag(key, packed)).assertExists()
        }
        compose.onNodeWithTag(layoutOptionTag(key, pin("PB5"))).assertDoesNotExist()
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
                chip = ChipTag.F103C8,
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

    @Test
    fun aRowThatCannotBeEditedWithoutThePartSaysThat() {
        show(state(chip = null))
        val pinRow = slot(BoardField.LED_GREEN).key

        // The first build rendered no control and no reason on a row it could not offer, which read
        // as a broken screen. Nothing can be staged without the part, and the row says that much.
        compose.onNodeWithTag(layoutRowTag(pinRow)).performScrollTo().assertIsDisplayed()
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(pinRow))),
        ).assertExists()
        compose.onNodeWithTag(layoutChangeTag(pinRow)).assertDoesNotExist()

        // The power latch is not editable with a part either, so it keeps its own standing note
        // rather than being told the part is missing.
        val latch = slot(BoardField.SELF_HOLD).key
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(latch))),
        ).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.layout_note_self_hold)).assertExists()

        // A preset cannot be staged against a part nobody knows either: it could never be applied.
        compose.onNodeWithTag(layoutPresetTag(LayoutPresets.BENCH_MASTER)).performScrollTo().assertIsNotEnabled()

        // Nor can the one row that is not a pin at all.
        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()
        val choiceRow = slot(BoardField.IMU_MODEL).key
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(choiceRow))),
        ).assertExists()
        compose.onNodeWithTag(layoutChangeTag(choiceRow)).assertDoesNotExist()
    }

    @Test
    fun aChoiceIsStagedFromItsOwnDialog() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(BoardField.IMU_MODEL).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutOptionTag(key, Fields.ImuModel.CLONE_2E)).performClick()

        assertEquals(listOf(key to Fields.ImuModel.CLONE_2E), actions.staged)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun theDeadTimeIsTypedIntoAnEditDialog() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(BoardField.DEAD_TIME, 0).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 0)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        // The one field that is free text rather than a list, so it gets the edit dialog's two
        // actions: the entry is staged on Set and nowhere else.
        compose.onNodeWithTag(layoutDialogTag(key)).assertExists()
        compose.onNode(hasSetTextAction()).performTextReplacement("64")
        assertEquals(emptyList<Pair<Key, Int>>(), actions.staged)
        compose.onNodeWithText(s(R.string.layout_set)).performClick()

        assertEquals(listOf(key to 64), actions.staged)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun aSecondMotorThatHoldsAPinHasItsOwnTab() {
        show(state(staged = slot(BoardField.HALL_A, 1).on(blank, pin("PC13"))))

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).performScrollTo().assertIsDisplayed()
        compose.onNodeWithTag(LAYOUT_ADD_MOTOR_TAG).assertDoesNotExist()
    }

    @Test
    fun addingASecondMotorIsTheOnlyWayItsTabFirstAppears() {
        // A bare board: every motor-1 field is unset, so the tab would never appear on its own.
        show(state())

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).assertDoesNotExist()
        compose.onNodeWithTag(layoutRowTag(slot(BoardField.HALL_A, 1).key)).assertDoesNotExist()

        compose.onNodeWithTag(LAYOUT_ADD_MOTOR_TAG).performScrollTo().performClick()

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).performScrollTo().assertIsDisplayed()
        // And the tab it added is the one shown, rather than leaving it to be found.
        compose.onNodeWithTag(layoutRowTag(slot(BoardField.HALL_A, 1).key)).performScrollTo().assertIsDisplayed()
    }

    @Test
    fun theGroupsAreTabsSoOnlyOneGroupIsOnScreen() {
        show(state())

        compose.onNodeWithTag(layoutRowTag(slot(BoardField.SELF_HOLD).key)).assertExists()
        compose.onNodeWithTag(layoutRowTag(slot(BoardField.IMU_SCL).key)).assertDoesNotExist()

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()

        compose.onNodeWithTag(layoutRowTag(slot(BoardField.IMU_SCL).key)).assertExists()
        compose.onNodeWithTag(layoutRowTag(slot(BoardField.SELF_HOLD).key)).assertDoesNotExist()
    }
}
