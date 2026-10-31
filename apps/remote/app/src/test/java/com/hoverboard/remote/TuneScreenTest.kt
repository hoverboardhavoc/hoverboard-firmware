package com.hoverboard.remote

import android.app.Application
import androidx.compose.material3.Text
import androidx.compose.ui.semantics.SemanticsActions
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertTextEquals
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onFirst
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.compose.ui.test.performSemanticsAction
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.screens.TUNE_BOUNDS_TAG
import com.hoverboard.remote.ui.screens.TUNE_NODE_TAG
import com.hoverboard.remote.ui.screens.TUNE_REVERT_TAG
import com.hoverboard.remote.ui.screens.TUNE_RIDE_TAG
import com.hoverboard.remote.ui.screens.TUNE_SAVE_TAG
import com.hoverboard.remote.ui.screens.TUNE_STALE_TAG
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.nodeChipTag
import com.hoverboard.remote.ui.screens.tuneBoundSliderTag
import com.hoverboard.remote.ui.screens.tuneBoundTag
import com.hoverboard.remote.ui.screens.tuneRowTag
import com.hoverboard.remote.ui.screens.tuneSliderTag
import com.hoverboard.remote.ui.screens.tuneStagedTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Tune screen says (`specs/remote-ui.md` section 3.3, `specs/state-feedback.md` section 10):
 * four gain rows with their units, staged and flash as numbers, engaged never as one, the converging
 * and unsaved marks, Save off while armed with the reason, the sliders live while armed, and a stale
 * board greyed with writes off.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class TuneScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val keys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }

    /** Records what the screen asked for. */
    private class Recorder : TuneActions {
        val calls = mutableListOf<String>()
        val bounds = mutableListOf<Pair<Key, Int>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun selectTarget(address: Int) { calls += "target:$address" }
        override fun refresh() { calls += "refresh" }
        override fun slide(key: Key, value: Int) { calls += "slide:${key.fieldId}:${key.index}:$value" }
        override fun slideEnd(key: Key, value: Int) { calls += "end:${key.fieldId}:${key.index}:$value" }
        override fun setBound(key: Key, value: Int) { bounds += key to value }
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
            staged = keys.zip(staged).toMap(),
            flash = keys.zip(flash).toMap(),
            stale = stale,
            converging = converging.mapKeys { keys[it.key] },
        )

    private fun read(
        g: BoardGains = gains(START, START),
        motors: List<Int> = listOf(0x01, 0x02),
        bounds: Map<Key, Int> = boundDefaults,
    ) = TuneState(
        target = 0x01,
        motors = motors.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, "Hoverboard") },
        boards = mapOf(0x01 to g.copy(bounds = bounds)),
    )

    /** What a board that never staged a bound answers, which is every row's starting point. */
    private val boundDefaults: Map<Key, Int> =
        SetupFields.DRIVE_BOUNDS.associate { it.key to (it.def.default as Value.I16).v }

    private fun show(state: TuneState, armed: Boolean = false, ride: Boolean = false): Recorder {
        val r = Recorder()
        compose.setContent {
            HoverboardRemoteTheme {
                TuneScreen(
                    state = state,
                    armed = armed,
                    actions = r,
                    ride = { if (ride) Text(RIDE_SLOT) },
                )
            }
        }
        return r
    }

    private fun inBound(field: SetupField, text: String) =
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneBoundTag(field.key))))

    /** Gain [index]'s read-back readout: the entry under it holds the same number, so by tag. */
    private fun staged(index: Int) = compose.onNodeWithTag(tuneStagedTag(index))

    private fun inRow(index: Int, text: String) =
        compose.onNode(hasText(text) and hasAnyAncestor(hasTestTag(tuneRowTag(index))))

    @Test
    fun stagedAndFlashAreShownAsNumbersAndEngagedIsSaidToBeUnreadable() {
        show(read(gains(MOVED, START)))

        staged(0).assertTextEquals("3591")
        inRow(0, s(R.string.tune_flash, "3491")).assertExists()
        inRow(0, " *").assertExists()
        inRow(1, " *").assertDoesNotExist()
        compose.onNodeWithText(s(R.string.tune_legend_engaged)).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(context.resources.getQuantityString(R.plurals.tune_unsaved_count, 1, 1))
            .performScrollTo().assertIsDisplayed()
    }

    @Test
    fun aConvergingGainCarriesTheMarkAndTheBoundNotAClaimThatItArrived() {
        show(read(gains(MOVED, START, converging = mapOf(0 to 400L))))

        inRow(0, " ~").assertExists()
        inRow(0, s(R.string.tune_converging_row, "0.4")).assertExists()
        inRow(1, " ~").assertDoesNotExist()
    }

    @Test
    fun armedTheSlidersAndRevertWorkAndSaveIsOffSayingWhy() {
        val r = show(read(gains(MOVED, START)), armed = true)

        compose.onNodeWithTag(tuneSliderTag(0)).assertIsEnabled()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(6500f) }
        compose.onNodeWithTag(TUNE_REVERT_TAG).performScrollTo().assertIsEnabled()
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.tune_save_armed)).assertExists()
        compose.onNodeWithText(s(R.string.tune_save_armed_note)).assertExists()

        // Both halves reach the model, at the value the control was taken to: the samples the model
        // sends on a cadence, and the one release that carries the committed value.
        val f = Gains.CONTROL_SF_GAINS
        assertEquals(listOf("shown", "slide:$f:0:6500", "end:$f:0:6500"), r.calls)
    }

    /** The slider's extent is the board's own maximum as read, not a constant (3.3a). */
    @Test
    fun theSliderSpansTheRangeReadFromThisBoard() {
        val g = gains(START, START).copy(maxima = mapOf(0 to 6050))
        show(read(g))

        inRow(0, s(R.string.tune_range, "6050")).assertExists()
        inRow(1, s(R.string.tune_range, Gains.DEFAULT_MAX[1].toString())).assertExists()
    }

    @Test
    fun disarmedWithUnsavedGainsSaveIsOffered() {
        val r = show(read(gains(MOVED, START)))
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsEnabled().performClick()
        assertEquals("save", r.calls.last())
    }

    @Test
    fun aStaleBoardShowsItsLastValuesWithEveryWriteOff() {
        show(read(gains(MOVED, START, stale = true)))

        compose.onNodeWithTag(TUNE_STALE_TAG).assertIsDisplayed()
        staged(0).assertTextEquals("3591")
        compose.onNodeWithTag(tuneSliderTag(0)).assertIsNotEnabled()
        compose.onNodeWithTag(TUNE_SAVE_TAG).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(TUNE_REVERT_TAG).assertIsNotEnabled()
    }

    /** One gain set: no profile chips, no rider-set line, four rows each saying what its gain multiplies. */
    @Test
    fun fourGainRowsWithTheirUnitsAndNoChoiceOfSet() {
        show(read())
        for ((i, unit) in listOf(
            R.string.tune_gain_gp_unit,
            R.string.tune_gain_gr_unit,
            R.string.tune_gain_gx_unit,
            R.string.tune_gain_gv_unit,
        ).withIndex()) {
            inRow(i, listOf("gp", "gr", "gx", "gv")[i]).performScrollTo().assertExists()
            inRow(i, s(unit)).assertExists()
            staged(i).assertTextEquals(START[i].toString())
        }
        compose.onNodeWithText("With a rider").assertDoesNotExist()
        compose.onNodeWithText("No rider").assertDoesNotExist()
    }

    /** One chip per motor controller the walk found, labelled by name and address (`specs/remote-ui.md` 3.3). */
    @Test
    fun theSelectorOffersEveryMotorControllerByNameAndAddress() {
        val r = show(read())
        compose.onNodeWithTag(nodeChipTag(TUNE_NODE_TAG, 0x01)).assertExists()
        compose.onNodeWithText("Hoverboard 0x02").assertExists()
        compose.onNodeWithTag(nodeChipTag(TUNE_NODE_TAG, 0x02)).performClick()
        assertEquals("target:2", r.calls.last())
    }

    @Test
    fun aMachineWithOneMotorControllerOffersOneChip() {
        show(read(motors = listOf(0x01)))
        compose.onNodeWithTag(nodeChipTag(TUNE_NODE_TAG, 0x02)).assertDoesNotExist()
        compose.onNodeWithTag(nodeChipTag(TUNE_NODE_TAG, 0x01)).assertExists()
    }

    // --- the drive bounds (`specs/remote-ui.md` 3.3b) ---------------------------------------------

    /**
     * Each bound row says what the board stores, what the row takes, and when the board will start
     * using a value written here. The last line is generated from the firmware's arm row, so it is
     * asserted as the same string every arm-time Setup row shows.
     */
    @Test
    fun eachDriveBoundShowsItsStoredValueItsRangeAndWhenItTakesEffect() {
        show(read(bounds = boundDefaults + (SetupFields.POS_CLAMP.key to 60)))

        compose.onNodeWithTag(TUNE_BOUNDS_TAG).performScrollTo().assertIsDisplayed()
        // Twice over: the row's readout, and the entry prefilled with it so an exact number is
        // typed from where the slider left off.
        compose.onAllNodes(hasText("60") and hasAnyAncestor(hasTestTag(tuneBoundTag(SetupFields.POS_CLAMP.key))))
            .assertCountEquals(2)
        inBound(SetupFields.POS_CLAMP, s(R.string.setup_range, "0", "450")).assertExists()
        inBound(SetupFields.POS_CLAMP, s(R.string.setup_effect_arm)).assertExists()
        inBound(SetupFields.TURN_MAX, s(R.string.setup_range, "0", "28500")).assertExists()
        inBound(SetupFields.TURN_SLEW, s(R.string.setup_range, "1", "1000")).assertExists()
    }

    /** Letting go of a bound slider is the write, and the value sent is where the finger left it. */
    @Test
    fun releasingABoundSliderWritesThatValueOnce() {
        val r = show(read())

        compose.onNodeWithTag(tuneBoundSliderTag(SetupFields.TURN_MAX.key)).performScrollTo()
            .performSemanticsAction(SemanticsActions.SetProgress) { it(12_000f) }

        assertEquals(listOf(SetupFields.TURN_MAX.key to 12_000), r.bounds)
    }

    /**
     * Armed, the bounds are off and the screen says why, while the gain sliders above stay live: the
     * bounds are stored values and the board refuses to store anything while armed.
     */
    @Test
    fun armedTheBoundsAreOffAndSayWhyWhileTheGainSlidersStayLive() {
        show(read(), armed = true)

        compose.onNodeWithText(s(R.string.tune_bounds_armed)).performScrollTo().assertExists()
        compose.onNodeWithTag(tuneBoundSliderTag(SetupFields.POS_CLAMP.key)).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(tuneSliderTag(0)).performScrollTo().assertIsEnabled()
    }

    /** A bound the session has not read offers nothing to drag and says so rather than showing a 0. */
    @Test
    fun anUnreadBoundIsNotDraggable() {
        show(read(bounds = emptyMap()))

        compose.onAllNodesWithText(s(R.string.tune_bound_unread)).onFirst().assertExists()
        compose.onNodeWithTag(tuneBoundSliderTag(SetupFields.POS_CLAMP.key)).performScrollTo().assertIsNotEnabled()
    }

    /** The ride controls are the caller's, under their own heading, so the screen can stop the machine. */
    @Test
    fun theRideControlsAreOfferedOnThisScreen() {
        show(read(), ride = true)

        compose.onNodeWithTag(TUNE_RIDE_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(RIDE_SLOT).performScrollTo().assertExists()
    }
}

/** The section-7 start gains, `[gp, gr, gx, gv]`, as a board staged for the rover shows them. */
private val START = listOf(3491, 4002, 197, 926)

/** The same with `gp` moved 100 up and not saved. */
private val MOVED = listOf(3591, 4002, 197, 926)

/** What the ride slot renders in a test: the screen only has to place what it is given. */
private const val RIDE_SLOT = "ride slot"

