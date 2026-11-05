package com.hoverboardhavoc.remote

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
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.checks.Check
import com.hoverboard.protocol.checks.Refusal
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboardhavoc.remote.model.LayoutGroup
import com.hoverboardhavoc.remote.ui.screens.LAYOUT_ADD_MOTOR_TAG
import com.hoverboardhavoc.remote.ui.screens.LAYOUT_APPLY_TAG
import com.hoverboardhavoc.remote.ui.screens.LAYOUT_LOCK_TAG
import com.hoverboardhavoc.remote.ui.screens.LAYOUT_PART_UNKNOWN_TAG
import com.hoverboardhavoc.remote.ui.screens.LAYOUT_POWER_CYCLE_TAG
import com.hoverboardhavoc.remote.ui.screens.LayoutScreen
import com.hoverboardhavoc.remote.ui.screens.layoutChangeTag
import com.hoverboardhavoc.remote.ui.screens.layoutClearTag
import com.hoverboardhavoc.remote.ui.screens.layoutDialogTag
import com.hoverboardhavoc.remote.ui.screens.layoutOptionTag
import com.hoverboardhavoc.remote.ui.screens.layoutRowTag
import com.hoverboardhavoc.remote.ui.screens.layoutTabTag
import com.hoverboardhavoc.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the layout editor says (`specs/remote-ui.md` sections 3.5 and 3.5a): the delta and Apply, a
 * check's refusal named by its field and reason, a picker that offers what the part's table gives
 * the slot from the pins no other field holds, the power latch shown but never offered, and the
 * editing done in dialogs from tabbed groups rather than in the rows.
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
        val staged = mutableListOf<Pair<Key, Value>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun refresh() { calls += "refresh" }
        override fun stage(slot: LayoutSlot, value: Value) { staged += slot.key to value }
        override fun revert(slot: LayoutSlot) { calls += "revert:${slot.key}" }
        override fun clear(slot: LayoutSlot) { calls += "clear:${slot.key}" }
        override fun revertAll() { calls += "revertAll" }
        override fun apply() { calls += "apply" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    /** A board that stores nothing: every key read, each answered with its default. */
    private val values: Map<Key, Value?> = Layout.SLOTS.associate { it.key to null }

    private val blank: BoardFields = checkNotNull(Layout.fieldsFrom(values))

    private fun slot(field: FieldDef, motor: Int? = null): LayoutSlot =
        checkNotNull(Layout.forField(field, motor))

    private fun pin(name: String) = Value.Pin(name[1] - 'A', name.drop(2).toInt())

    private fun state(
        chip: ChipTag? = ChipTag.F103C8,
        staged: BoardFields? = blank,
        written: Set<Key> = emptySet(),
        notice: LayoutNotice? = null,
    ) = LayoutState(
        board = 0x01,
        chip = chip,
        values = values,
        staged = staged,
        written = written,
        notice = notice,
    )

    private fun show(state: LayoutState, armed: Boolean = false, actions: LayoutActions = Recorder()) =
        compose.setContent {
            HoverboardRemoteTheme { LayoutScreen(state = state, armed = armed, actions = actions) }
        }

    @Test
    fun aRefusedApplyNamesTheFieldAndTheReason() {
        val refusal = Refusal(
            Check.GROUP,
            Fields.MOTOR_HALL_A.key(0),
            "motors[0].hall.a: the group is incomplete: motors[0].hall.b is not stored",
        )
        show(state(notice = LayoutNotice.CheckRefused(refusal)))

        compose.onNodeWithText(s(R.string.layout_notice_check_refused, refusal.msg)).assertExists()
    }

    @Test
    fun aCleanDeltaOffersApply() {
        val staged = blank.with(slot(Fields.BOARD_BUZZER), pin("PB5"))
        val actions = Recorder()
        show(state(staged = staged), actions = actions)

        compose.onNodeWithText(
            s(R.string.layout_delta_line, s(R.string.layout_field_buzzer), s(R.string.layout_absent), "PB5"),
        ).assertExists()
        compose.onNodeWithTag(LAYOUT_APPLY_TAG).performScrollTo().assertIsEnabled().performClick()
        assertEquals(listOf("shown", "apply"), actions.calls)
    }

    @Test
    fun aBoardThatSendsNoPartHasNoWayToStateOne() {
        val actions = Recorder()
        show(state(chip = null), actions = actions)

        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_part_unsent)).assertExists()
        compose.onNodeWithText(s(R.string.layout_part_unknown_body)).assertExists()
        // Nothing to pick a part with.
        assertEquals("nothing was asked of the model", listOf("shown"), actions.calls)
    }

    @Test
    fun aPartTheAppDoesNotKnowSaysThatRatherThanTheOtherReason() {
        show(state(chip = ChipTag.Unknown))

        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_part_unnamed)).assertExists()
        compose.onNodeWithText(s(R.string.layout_part_unsent)).assertDoesNotExist()
    }

    @Test
    fun aKnownPartIsStatedAsTheBoardsOwn() {
        show(state())

        compose.onNodeWithText(s(R.string.layout_part, Chip.GD32F103C8.name)).assertExists()
        compose.onNodeWithTag(LAYOUT_PART_UNKNOWN_TAG).assertDoesNotExist()
    }

    @Test
    fun thePickerOffersThePartsFreePins() {
        val actions = Recorder()
        show(state(staged = blank.with(slot(Fields.BOARD_BUZZER), pin("PB9"))), actions = actions)
        val key = slot(Fields.LED_GREEN).key

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutDialogTag(key)).assertExists()

        // Free and bonded: offered, an advanced-timer pin among them, since an LED is a plain pin.
        compose.onNodeWithTag(layoutOptionTag(key, "PB5")).assertExists()
        compose.onNodeWithTag(layoutOptionTag(key, "PA8")).assertExists()
        // Another function's pin, an SWD pin, and a pin this part does not bond: each withheld.
        compose.onNodeWithTag(layoutOptionTag(key, "PB9")).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, "PA13")).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, "PF0")).assertDoesNotExist()

        compose.onNodeWithTag(layoutOptionTag(key, "PB5")).performScrollTo().performClick()
        assertEquals(listOf(key to pin("PB5")), actions.staged)
        // Tapping one stages it and closes, which is the simple-dialog pattern: no confirm step.
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    /** `specs/no-fleet-pins.md` section 2: a pin that holds a value can be cleared; one that holds none cannot. */
    @Test
    fun thePickerOffersClearOnlyForAPinThatHoldsOne() {
        val key = slot(Fields.LED_GREEN).key
        val actions = Recorder()
        show(state(staged = blank.with(slot(Fields.LED_GREEN), pin("PB3"))), actions = actions)

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutClearTag(key)).performClick()
        assertEquals(listOf("shown", "clear:$key"), actions.calls)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun aPinThatHoldsNoneHasNoClear() {
        val key = slot(Fields.LED_GREEN).key
        show(state())

        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutDialogTag(key)).assertExists()
        compose.onNodeWithTag(layoutClearTag(key)).assertDoesNotExist()
    }

    @Test
    fun aGateSlotOffersTheRoutesOfItsChannelWithTheirAlternateFunction() {
        val actions = Recorder()
        show(state(chip = ChipTag.F130C8), actions = actions)
        val key = slot(Fields.MOTOR_GATE_HI_A, 0).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 0)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()

        // TIMER0 channel 0 is PA8 at AF2 on the F130, and no other pin carries it.
        compose.onNodeWithTag(layoutOptionTag(key, "PA9 AF2")).assertDoesNotExist()
        compose.onNodeWithTag(layoutOptionTag(key, "PA8 AF2")).performClick()
        assertEquals(listOf(key to Value.PinAf(0, 8, 2)), actions.staged)
    }

    @Test
    fun thePowerLatchIsShownAndNeverOffered() {
        show(state())
        val key = slot(Fields.BOARD_SELF_HOLD).key

        compose.onNodeWithTag(layoutRowTag(key)).performScrollTo().assertIsDisplayed()
        // Not stored, it reads as not set: no fleet pin stands in for it (`specs/no-fleet-pins.md` section 2).
        val inRow = hasAnyAncestor(hasTestTag(layoutRowTag(key)))
        compose.onNode(hasText(s(R.string.layout_absent)) and inRow).assertExists()
        compose.onNode(hasText("PB12") and inRow).assertDoesNotExist()
        compose.onNodeWithTag(layoutChangeTag(key)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.layout_note_self_hold)).assertExists()
    }

    @Test
    fun armedShowsTheLockAndDisablesThePickers() {
        show(state(), armed = true)

        compose.onNodeWithTag(LAYOUT_LOCK_TAG).assertIsDisplayed()
        compose.onNodeWithTag(layoutChangeTag(slot(Fields.LED_GREEN).key)).performScrollTo().assertIsNotEnabled()
    }

    @Test
    fun aWrittenLayoutAsksForOnePowerCycleAndNothingElse() {
        val actions = Recorder()
        show(state(written = setOf(Fields.BOARD_BUZZER.key())), actions = actions)

        compose.onNodeWithTag(LAYOUT_POWER_CYCLE_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.layout_power_cycle, 1)).assertExists()
        // The board's boot counter is what clears this, so the screen asks for nothing back: the
        // button that took the operator's word for a restart is gone.
        compose.onNodeWithText(s(R.string.layout_power_cycle_watch)).assertExists()
        assertEquals(listOf("shown"), actions.calls)
    }

    @Test
    fun aPinRowThatCannotBePickedWithoutThePartSaysThat() {
        show(state(chip = null))
        val pinRow = slot(Fields.LED_GREEN).key

        // The first build rendered no control and no reason on a row it could not offer, which read
        // as a broken screen. No pin can be picked without the part, and the row says that much.
        compose.onNodeWithTag(layoutRowTag(pinRow)).performScrollTo().assertIsDisplayed()
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(pinRow))),
        ).assertExists()
        compose.onNodeWithTag(layoutChangeTag(pinRow)).assertDoesNotExist()

        // The power latch is not editable with a part either, so it keeps its own standing note
        // rather than being told the part is missing.
        val latch = slot(Fields.BOARD_SELF_HOLD).key
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(latch))),
        ).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.layout_note_self_hold)).assertExists()

        // A row that is not a pin needs no part to be staged, so it keeps its control.
        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()
        val numberRow = slot(Fields.IMU_I2C).key
        compose.onNode(
            hasText(s(R.string.layout_row_part_unknown)) and hasAnyAncestor(hasTestTag(layoutRowTag(numberRow))),
        ).assertDoesNotExist()
        compose.onNodeWithTag(layoutChangeTag(numberRow)).performScrollTo().assertIsEnabled()
    }

    @Test
    fun aChoiceIsStagedFromItsOwnDialog() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(Fields.IMU_MODEL).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutOptionTag(key, Fields.ImuModel.CLONE_2E.toString())).performClick()

        assertEquals(listOf(key to Value.U8(Fields.ImuModel.CLONE_2E)), actions.staged)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun anInstanceIsTypedIntoAnEditDialog() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(Fields.IMU_I2C).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        compose.onNode(hasSetTextAction()).performTextReplacement("1")
        compose.onNodeWithText(s(R.string.layout_set)).performClick()

        assertEquals(listOf(key to Value.U8(1)), actions.staged)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun theDeadTimeIsTypedIntoAnEditDialog() {
        val actions = Recorder()
        show(state(), actions = actions)
        val key = slot(Fields.MOTOR_DEAD_TIME, 0).key

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 0)).performScrollTo().performClick()
        compose.onNodeWithTag(layoutChangeTag(key)).performScrollTo().performClick()
        // The one field that is free text rather than a list, so it gets the edit dialog's two
        // actions: the entry is staged on Set and nowhere else.
        compose.onNodeWithTag(layoutDialogTag(key)).assertExists()
        compose.onNode(hasSetTextAction()).performTextReplacement("64")
        assertEquals(emptyList<Pair<Key, Value>>(), actions.staged)
        compose.onNodeWithText(s(R.string.layout_set)).performClick()

        assertEquals(listOf(key to Value.U8(64)), actions.staged)
        compose.onNodeWithTag(layoutDialogTag(key)).assertDoesNotExist()
    }

    @Test
    fun aSecondMotorThatHoldsAPinHasItsOwnTab() {
        show(state(staged = blank.with(slot(Fields.MOTOR_HALL_A, 1), pin("PC13"))))

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).performScrollTo().assertIsDisplayed()
        compose.onNodeWithTag(LAYOUT_ADD_MOTOR_TAG).assertDoesNotExist()
    }

    @Test
    fun addingASecondMotorIsTheOnlyWayItsTabFirstAppears() {
        // A bare board: every motor-1 field is unset, so the tab would never appear on its own.
        show(state())

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).assertDoesNotExist()
        compose.onNodeWithTag(layoutRowTag(slot(Fields.MOTOR_HALL_A, 1).key)).assertDoesNotExist()

        compose.onNodeWithTag(LAYOUT_ADD_MOTOR_TAG).performScrollTo().performClick()

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.MOTOR, 1)).performScrollTo().assertIsDisplayed()
        // And the tab it added is the one shown, rather than leaving it to be found.
        compose.onNodeWithTag(layoutRowTag(slot(Fields.MOTOR_HALL_A, 1).key)).performScrollTo().assertIsDisplayed()
    }

    @Test
    fun theGroupsAreTabsSoOnlyOneGroupIsOnScreen() {
        show(state())

        compose.onNodeWithTag(layoutRowTag(slot(Fields.BOARD_SELF_HOLD).key)).assertExists()
        compose.onNodeWithTag(layoutRowTag(slot(Fields.IMU_SCL_PIN).key)).assertDoesNotExist()

        compose.onNodeWithTag(layoutTabTag(LayoutGroup.IMU)).performScrollTo().performClick()

        compose.onNodeWithTag(layoutRowTag(slot(Fields.IMU_SCL_PIN).key)).assertExists()
        compose.onNodeWithTag(layoutRowTag(slot(Fields.BOARD_SELF_HOLD).key)).assertDoesNotExist()
    }
}
