package com.hoverboard.remote

import android.app.Application
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.semantics.SemanticsProperties
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertIsSelected
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasClickAction
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
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SetupGroup
import com.hoverboard.remote.ui.screens.AppTab
import com.hoverboard.remote.ui.screens.ConnectedScreen
import com.hoverboard.remote.ui.screens.SETUP_APPLY_TAG
import com.hoverboard.remote.ui.screens.SETUP_FRAME_HOLD_TAG
import com.hoverboard.remote.ui.screens.SETUP_LOCK_TAG
import com.hoverboard.remote.ui.screens.SETUP_POWER_CYCLE_TAG
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.setupAdvancedTag
import com.hoverboard.remote.ui.screens.setupRowTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import androidx.compose.material3.Text
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Setup screen says (`specs/rider-ui.md` section 3.4): read-only and saying why while armed,
 * "stored" and "staged, not applied" after a verified write and never "live", and the orientation
 * refusal and rotation check in the user's terms.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupScreenTest {

    @get:Rule
    val compose = createComposeRule()

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    /** Records what the screen asked for. */
    private class Recorder : SetupActions {
        val calls = mutableListOf<String>()
        val staged = mutableListOf<Pair<Key, Value>>()
        override fun onShown() { calls += "shown" }
        override fun onHidden() { calls += "hidden" }
        override fun refresh() { calls += "refresh" }
        override fun stage(key: Key, value: Value) { staged += key to value }
        override fun discard(key: Key) { calls += "discard" }
        override fun discardAll() { calls += "discardAll" }
        override fun apply() { calls += "apply" }
        override fun stageFrame(roles: List<Int>, signs: List<Int>) { calls += "frame:$roles:$signs" }
        override fun setLevel() { calls += "setLevel" }
        override fun checkLevel() { calls += "checkLevel" }
        override fun checkForwardLean() { calls += "checkForwardLean" }
        override fun confirmPowerCycled() { calls += "confirmPowerCycled" }
        override fun dismissNotice() { calls += "dismiss" }
    }

    private val stored: Map<Key, Value> = SetupFields.ALL.associate { it.key to it.def.default }

    private fun show(state: SetupState, armed: Boolean = false, actions: SetupActions = Recorder()) =
        compose.setContent {
            HoverboardRemoteTheme { SetupScreen(state = state, armed = armed, telemetry = null, actions = actions) }
        }

    @Test
    fun armedShowsTheLockAndDisablesEveryEdit() {
        val pending = mapOf(SetupFields.CONTROL_MODE.key to Value.U8(1))
        val state = SetupState(board = 0x01, values = stored, pending = pending)
        show(state, armed = true)

        compose.onNodeWithTag(SETUP_LOCK_TAG).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_locked_body)).assertIsDisplayed()
        compose.onNodeWithTag(SETUP_APPLY_TAG).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_choice_balance)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_level_action)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_heading_rear)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_face_component_down)).assertIsNotEnabled()
        // Every text entry too, not just the ones this test names.
        compose.onAllNodes(hasSetTextAction()).fetchSemanticsNodes().forEach {
            assertTrue("an editable text field while armed", it.config.contains(SemanticsProperties.Disabled))
        }
    }

    @Test
    fun disarmedHasNoLockAndAChipStagesItsByte() {
        val actions = Recorder()
        show(SetupState(board = 0x01, values = stored), actions = actions)

        compose.onNodeWithTag(SETUP_LOCK_TAG).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_choice_balance)).performScrollTo().performClick()

        assertEquals(listOf(SetupFields.CONTROL_MODE.key to Value.U8(1)), actions.staged)
        assertTrue("the screen did not ask for its read pass", "shown" in actions.calls)
    }

    /**
     * Includes a verified rotation write: the stored map is then NOT what the board runs, so the
     * orientation panel must not say "Runs as" of it (P0-1 of the Setup audit).
     */
    @Test
    fun aVerifiedWriteSaysStoredAndStagedNeverLive() {
        val mode = SetupFields.CONTROL_MODE.key
        // Component side down, stock-rear forward: the identity map under the compiled roles.
        val pose = Orientation.Pose(Orientation.Face.COMPONENT_DOWN, Orientation.Heading.STOCK_REAR)
        val rotation = SetupFields.AXIS_SIGN.mapIndexed { i, f ->
            f.key to Value.I32(Orientation.frameOf(pose).signs[i])
        }.toMap()
        val written = rotation + (mode to Value.U8(1))
        val state = SetupState(board = 0x01, values = stored + written, staged = written)
        show(state)

        compose.onNodeWithTag(SETUP_POWER_CYCLE_TAG).assertIsDisplayed()
        val instruction = context.resources.getQuantityString(R.plurals.setup_power_cycle, written.size, written.size)
        compose.onNodeWithText(instruction).assertIsDisplayed()
        val name = s(R.string.setup_pose_name, s(R.string.setup_face_component_down), s(R.string.setup_heading_rear))
        compose.onNodeWithText(s(R.string.setup_orientation_stored_as, name)).assertExists()
        val runsAs = s(R.string.setup_orientation_runs, "").trim()
        assertEquals(0, compose.onAllNodes(hasText(runsAs, substring = true)).fetchSemanticsNodes().size)
        compose.onNodeWithText(s(R.string.setup_staged_mark)).assertExists()
        // The confirmation waits for the link drop a real power-cycle causes.
        compose.onNodeWithText(s(R.string.setup_power_cycled)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_power_cycle_wait)).assertExists()
        val live = compose.onAllNodes(hasText("live", substring = true, ignoreCase = true))
        assertEquals(0, live.fetchSemanticsNodes().size)
    }

    @Test
    fun theStandingCaveatsAreShown() {
        show(SetupState(board = 0x01, values = stored))

        compose.onNodeWithText(s(R.string.setup_note_motor_method)).assertExists()
        compose.onNodeWithText(s(R.string.setup_note_device_name)).assertExists()
        compose.onNodeWithText(s(R.string.setup_note_current_limit)).assertExists()
    }

    @Test
    fun walkOwnedFieldsAreDisplayedWithNoEditor() {
        show(SetupState(board = 0x01, values = stored + (SetupFields.NODE_ADDRESS.key to Value.U8(1))))

        val row = setupRowTag(SetupFields.NODE_ADDRESS.key)
        compose.onNodeWithTag(row).assertExists()
        val editors = compose.onAllNodes(
            androidx.compose.ui.test.hasAnyAncestor(androidx.compose.ui.test.hasTestTag(row)) and
                (hasSetTextAction() or hasClickAction()),
        ).fetchSemanticsNodes()
        assertEquals(0, editors.size)
    }

    @Test
    fun aMirroredStoredMapIsCalledOut() {
        val mirrored = listOf(-1, 1, 1, -1, 1, -1)
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(mirrored[i]) }
        show(SetupState(board = 0x01, values = stored + signs))

        compose.onNodeWithText(s(R.string.setup_orientation_mirrored)).assertExists()
    }

    private fun stockName() =
        s(R.string.setup_pose_name, s(R.string.setup_face_component_up), s(R.string.setup_heading_forward))

    @Test
    fun anUnsetBoardIsNamedAsTheStockPoseAndAHeadingStagesTheFrame() {
        val actions = Recorder()
        show(SetupState(board = 0x01, values = stored), actions = actions)

        // All six signs and both roles unset: the board runs the reference, the stock pose.
        compose.onNodeWithText(s(R.string.setup_orientation_runs, stockName())).assertExists()
        compose.onNodeWithText(s(R.string.setup_orientation_unset)).assertExists()
        compose.onNodeWithText(s(R.string.setup_orientation_roles, "Z", "Y")).assertExists()
        compose.onNodeWithText(s(R.string.setup_face_component_up)).assertIsSelected()
        compose.onNodeWithText(s(R.string.setup_heading_forward)).assertIsSelected()
        // The flat face offers the four stock edges, never a side.
        compose.onNodeWithText(s(R.string.setup_heading_component)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_heading_right)).performScrollTo().performClick()

        val pose = Orientation.Pose(Orientation.Face.COMPONENT_UP, Orientation.Heading.STOCK_RIGHT)
        val frame = Orientation.frameOf(pose)
        assertTrue("frame:${frame.roles}:${frame.signs}" in actions.calls)
    }

    /** A face only chooses the headings offered; the heading stages the whole frame, roles too. */
    @Test
    fun anEdgeDownFaceOffersItsFourHeadingsAndOneStagesBothFields() {
        val actions = Recorder()
        show(SetupState(board = 0x01, values = stored), actions = actions)

        compose.onNodeWithText(s(R.string.setup_face_forward_edge_down)).performScrollTo().performClick()
        assertTrue("a face alone staged a frame", actions.calls.none { it.startsWith("frame:") })
        compose.onNodeWithText(s(R.string.setup_face_forward_edge_down)).assertIsSelected()
        for (h in listOf(R.string.setup_heading_left, R.string.setup_heading_right, R.string.setup_heading_component)) {
            compose.onNodeWithText(s(h)).assertExists()
        }
        compose.onNodeWithText(s(R.string.setup_heading_rear)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_heading_back)).performScrollTo().performClick()

        val pose = Orientation.Pose(Orientation.Face.STOCK_FORWARD_EDGE_DOWN, Orientation.Heading.BACK_SIDE)
        val frame = Orientation.frameOf(pose)
        assertEquals(1, frame.roles[0])
        assertTrue("frame:${frame.roles}:${frame.signs}" in actions.calls)
    }

    /** A legal stored frame that is none of the 24 shows as custom with its raw values. */
    @Test
    fun aLegalFrameOutsideThePosesIsCustom() {
        val mixed = listOf(-1, 1, -1, -1, -1, 1)
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(mixed[i]) }
        show(SetupState(board = 0x01, values = stored + signs))

        val custom = s(R.string.setup_pose_custom, "[0, 0]", mixed.toString())
        compose.onNodeWithText(s(R.string.setup_orientation_runs, custom)).assertExists()
    }

    @Test
    fun aRoleRowOffersNoEditor() {
        show(SetupState(board = 0x01, values = stored))
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.IMU)).performScrollTo().performClick()
        val row = setupRowTag(SetupFields.AXIS_ROLE[0].key)
        compose.onNodeWithTag(row).performScrollTo().assertExists()
        val editors = compose.onAllNodes(
            hasAnyAncestor(hasTestTag(row)) and (hasSetTextAction() or hasClickAction()),
        ).fetchSemanticsNodes()
        assertEquals(0, editors.size)
    }

    /** P1-2: staged changes over a mirrored stored map get no power-cycle instruction. */
    @Test
    fun aMirroredStoredMapWithStagedChangesHoldsThePowerCycleBack() {
        val partial = listOf(1, 1, -1, -1, 1, -1) // ax of an identity Apply written, az not
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(partial[i]) }.toMap()
        val ax = SetupFields.AXIS_SIGN[0].key
        show(SetupState(board = 0x01, values = stored + signs, staged = mapOf(ax to Value.I32(1))))

        compose.onNodeWithTag(SETUP_POWER_CYCLE_TAG).assertDoesNotExist()
        compose.onNodeWithTag(SETUP_FRAME_HOLD_TAG).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_frame_incomplete_body)).assertIsDisplayed()
    }

    @Test
    fun aFieldWhoseReadFailedSaysNotRead() {
        val name = SetupFields.DEVICE_NAME.key
        show(SetupState(board = 0x01, values = stored - name, unread = setOf(name)))

        compose.onNode(hasAnyAncestor(hasTestTag(setupRowTag(name))) and hasText(s(R.string.setup_unread)))
            .assertExists()
    }

    /** P2-8: a name too long for one write is refused in the row, with the limit, before any send. */
    @Test
    fun aTooLongNameSaysWhyAndCannotBeStaged() {
        val row = setupRowTag(SetupFields.DEVICE_NAME.key)
        show(SetupState(board = 0x01, values = stored))

        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasSetTextAction())
            .performTextReplacement("n".repeat(CONFIG_VALUE_MAX + 1))

        compose.onNodeWithText(s(R.string.setup_too_long, CONFIG_VALUE_MAX)).assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_stage))).assertIsNotEnabled()
    }

    @Test
    fun theFrameRefusalIsStatedAsNothingWritten() {
        val refused = SetupNotice.FrameRefused(Orientation.Refusal.ACCEL_MIRRORED)
        show(SetupState(board = 0x01, values = stored, notice = refused))

        compose.onNodeWithText(s(R.string.setup_notice_accel_mirrored)).assertIsDisplayed()
    }

    @Test
    fun theRotationCheckWaitsForThePowerCycleAndThenReportsEachStep() {
        val sign = SetupFields.AXIS_SIGN[0].key
        show(SetupState(board = 0x01, values = stored, staged = mapOf(sign to Value.I32(-1))))
        compose.onNodeWithText(s(R.string.setup_check_blocked)).assertExists()
        compose.onNodeWithText(s(R.string.setup_check_level)).assertDoesNotExist()
    }

    @Test
    fun theRotationCheckShowsResults() {
        val actions = Recorder()
        show(
            SetupState(
                board = 0x01,
                values = stored,
                rotationCheck = RotationCheck(level = CheckResult.PASS, forwardLean = CheckResult.FAIL),
            ),
            actions = actions,
        )

        compose.onNodeWithText(s(R.string.setup_check_pass)).assertExists()
        compose.onNodeWithText(s(R.string.setup_check_fail)).assertExists()
        compose.onNodeWithText(s(R.string.setup_check_level)).performScrollTo().performClick()
        assertTrue("checkLevel" in actions.calls)
    }

    @Test
    fun theRiderRequirementIsAChoiceInDrive() {
        val actions = Recorder()
        show(SetupState(board = 0x01, values = stored), actions = actions)
        val row = setupRowTag(SetupFields.RIDER_REQUIRED.key)
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_choice_not_required)))
            .performScrollTo().performClick()
        assertEquals(listOf(SetupFields.RIDER_REQUIRED.key to Value.U8(0)), actions.staged)
    }

    @Test
    fun theBatteryCalibrationSitsBehindItsGroupsAdvancedToggle() {
        show(SetupState(board = 0x01, values = stored))
        val slope = setupRowTag(SetupFields.VBATT_CAL[0].key)
        compose.onNodeWithTag(slope).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.CALIBRATION)).performScrollTo().performClick()
        compose.onNodeWithTag(slope).performScrollTo().assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "25200")).assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "-5")).assertExists()
    }

    @Test
    fun advancedRevealsTheRawEditorsRenderedFromTheirType() {
        show(SetupState(board = 0x01, values = stored + (SetupFields.GYRO_BIAS[2].key to Value.I32(-88))))

        compose.onNodeWithTag(setupRowTag(SetupFields.GYRO_BIAS[2].key)).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.IMU)).performScrollTo().performClick()
        compose.onNodeWithTag(setupRowTag(SetupFields.GYRO_BIAS[2].key)).assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "-88")).assertExists()
    }

    @Test
    fun theTabRowOffersRideTuneAndSetupAndLeavingRideReleasesTheThrottle() {
        var released = 0
        compose.setContent {
            var tab by androidx.compose.runtime.remember { mutableStateOf(AppTab.RIDE) }
            HoverboardRemoteTheme {
                ConnectedScreen(
                    tab = tab,
                    onTab = { tab = it },
                    onLeaveRide = { released++ },
                    ride = { Text("RIDE-CONTENT") },
                    tune = { Text("TUNE-CONTENT") },
                    setup = { Text("SETUP-CONTENT") },
                )
            }
        }
        compose.onNodeWithText("RIDE-CONTENT").assertIsDisplayed()

        compose.onNodeWithText(s(R.string.tab_tune)).performClick()
        compose.onNodeWithText("TUNE-CONTENT").assertIsDisplayed()
        compose.onNodeWithText("RIDE-CONTENT").assertDoesNotExist()
        assertEquals(1, released)

        compose.onNodeWithText(s(R.string.tab_setup)).performClick()
        compose.onNodeWithText("SETUP-CONTENT").assertIsDisplayed()
        assertEquals("leaving Tune released the throttle again", 1, released)
    }
}
