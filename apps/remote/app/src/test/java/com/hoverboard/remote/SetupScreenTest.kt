package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.semantics.SemanticsProperties
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertIsNotSelected
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
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.CONFIG_VALUE_MAX
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SetupGroup
import com.hoverboard.remote.ui.screens.MACHINE_SIDE_TAG
import com.hoverboard.remote.ui.screens.SETUP_FRAME_BAD_TAG
import com.hoverboard.remote.ui.screens.SETUP_FRAME_WRITING_TAG
import com.hoverboard.remote.ui.screens.SETUP_LOCK_TAG
import com.hoverboard.remote.ui.screens.SETUP_NODE_TAG
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.nodeChipTag
import com.hoverboard.remote.ui.screens.setupAdvancedTag
import com.hoverboard.remote.ui.screens.setupRetryTag
import com.hoverboard.remote.ui.screens.setupRowTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/** Records what the screen asked for, for every Setup screen test. */
internal class Recorder : SetupActions {
    val calls = mutableListOf<String>()
    val edited = mutableListOf<Pair<Key, Value>>()
    override fun onShown() { calls += "shown" }
    override fun onHidden() { calls += "hidden" }
    override fun selectTarget(address: Int) { calls += "target:$address" }
    override fun refresh() { calls += "refresh" }
    override fun edit(key: Key, value: Value) { edited += key to value }
    override fun retry(key: Key) { calls += "retry:$key" }
    override fun chooseFrame(value: Int) { calls += "chooseFrame:$value" }
    override fun chooseDriveMode(value: Int) { calls += "chooseDriveMode:$value" }
    override fun chooseRightWheel(address: Int) { calls += "chooseRightWheel:$address" }
    override fun editFrame(roles: List<Int>, signs: List<Int>) { calls += "frame:$roles:$signs" }
    override fun setLevel() { calls += "setLevel" }
    override fun checkLevel() { calls += "checkLevel" }
    override fun checkForwardLean() { calls += "checkForwardLean" }
    override fun dismissNotice() { calls += "dismiss" }
}

/**
 * What the Setup screen says (`specs/remote-ui.md` sections 3.4 and 3.6a): no Apply, a write result
 * per row with its own retry, when each field takes effect, nothing asking for a power-cycle, and
 * the orientation refusal and rotation check in the user's terms.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val stored: Map<Key, Value> = SetupFields.ALL.associate { it.key to it.def.default }

    /**
     * A state with [board] selected and [other] found beside it, holding one [BoardSetup] for the
     * selected board. Its class reads motor controller unless [values] says otherwise, so the screen
     * shows every row, as it does for a board.
     */
    private fun setupState(
        board: Int,
        values: Map<Key, Value> = emptyMap(),
        writes: Map<Key, FieldWrite> = emptyMap(),
        notice: SetupNotice? = null,
        other: Int? = null,
    ) = SetupState(
        addresses = listOfNotNull(board, other).sorted(),
        attached = other ?: board,
        target = board,
        boards = mapOf(board to BoardSetup(values = withClass(values), writes = writes)),
        notice = notice,
    )

    /** [values] with a motor controller's `node.class` in it, unless it already names one. */
    private fun withClass(values: Map<Key, Value>): Map<Key, Value> =
        mapOf(SetupFields.NODE_CLASS.key to Value.U8(Fields.NodeClass.MOTOR_CONTROLLER)) + values

    /** The state with [f] folded into the selected board's part of it. */
    private fun SetupState.withBoard(f: (BoardSetup) -> BoardSetup) = copy(boards = mapOf(board!! to f(current)))

    /**
     * The node selector's chip reading [text]. The Machine section's Right wheel picker carries the
     * same labels (`specs/remote-ui.md` 3.7a), so a plain text match finds both.
     */
    private fun selector(text: String) = hasText(text) and !hasAnyAncestor(hasTestTag(MACHINE_SIDE_TAG))

    private fun show(
        state: SetupState,
        armed: Boolean = false,
        actions: SetupActions = Recorder(),
        odometers: List<Pair<String, OdometerReading>> = emptyList(),
    ) = compose.setContent {
        HoverboardRemoteTheme {
            SetupScreen(
                state = state,
                armed = armed,
                telemetry = null,
                odometers = odometers,
                actions = actions,
            )
        }
    }

    @Test
    fun armedShowsTheLockAndDisablesEveryEdit() {
        val state = setupState(board = 0x01, values = stored)
        show(state, armed = true)

        compose.onNodeWithTag(SETUP_LOCK_TAG).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_locked_body)).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_choice_balance)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_level_action)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_face_component_up)).assertIsNotEnabled()
        compose.onNodeWithText(s(R.string.setup_face_component_down)).assertIsNotEnabled()
        // Every text entry too, not just the ones this test names.
        compose.onAllNodes(hasSetTextAction()).fetchSemanticsNodes().forEach {
            assertTrue("an editable text field while armed", it.config.contains(SemanticsProperties.Disabled))
        }
    }

    /**
     * 3.6a: `CFG_ARMED` is per row, not one banner. A rider who opens Setup armed sees every
     * writable row say so, and the read-only rows say nothing (nothing here writes them).
     */
    @Test
    fun everyWritableRowSaysItIsReadOnlyWhileArmed() {
        show(setupState(board = 0x01, values = stored), armed = true)

        val writable = SetupFields.ALL.count { it.takesEffect != null && !it.advanced }
        val said = compose.onAllNodes(hasText(s(R.string.setup_row_armed))).fetchSemanticsNodes().size
        assertEquals(writable, said)
        val readOnly = setupRowTag(SetupFields.NODE_ADDRESS.key)
        compose.onNode(hasAnyAncestor(hasTestTag(readOnly)) and hasText(s(R.string.setup_row_armed)))
            .assertDoesNotExist()
    }

    @Test
    fun disarmedHasNoLockAndAChipWritesItsByte() {
        val actions = Recorder()
        show(setupState(board = 0x01, values = stored), actions = actions)

        compose.onNodeWithTag(SETUP_LOCK_TAG).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_choice_balance)).performScrollTo().performClick()

        assertEquals(listOf(SetupFields.CONTROL_MODE.key to Value.U8(1)), actions.edited)
        assertTrue("the screen did not ask for its read pass", "shown" in actions.calls)
        // No Apply, no basket, no power-cycle instruction anywhere on the screen. (The device-name
        // note's power-cycle is the Bluetooth module's own, and stays.)
        for (gone in listOf("Apply", "Power-cycle the board", "I power-cycled", "pending", "Staged")) {
            val nodes = compose.onAllNodes(hasText(gone, substring = true)).fetchSemanticsNodes()
            assertEquals("the screen still says \"$gone\"", 0, nodes.size)
        }
    }

    /**
     * 3.6a's second half, which was wrong at the bench on 2026-10-10: the commutation method is in
     * the arm-time value row, so its row says the next arm and never a restart, while a boot-read
     * field (the IMU model) says a restart, and its written mark is what reports that no restart has
     * happened yet.
     */
    @Test
    fun aRowSaysWhenItsFieldTakesEffectAndTheMethodNeedsNoRestart() {
        val method = SetupFields.MOTOR_METHOD.key
        val imu = SetupFields.IMU_MODEL.key
        show(
            setupState(
                board = 0x01,
                values = stored + (method to Value.U8(2)),
                writes = mapOf(
                    method to FieldWrite(Value.U8(2), WriteState.Written),
                    imu to FieldWrite(Value.U8(1), WriteState.Written),
                ),
            ),
        )

        val methodRow = setupRowTag(method)
        compose.onNode(hasAnyAncestor(hasTestTag(methodRow)) and hasText(s(R.string.setup_effect_arm)))
            .performScrollTo().assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(methodRow)) and hasText(s(R.string.setup_write_written)))
            .assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(methodRow)) and hasText(s(R.string.setup_write_awaiting_restart)))
            .assertDoesNotExist()

        val imuRow = setupRowTag(imu)
        compose.onNode(hasAnyAncestor(hasTestTag(imuRow)) and hasText(s(R.string.setup_effect_boot)))
            .performScrollTo().assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(imuRow)) and hasText(s(R.string.setup_write_awaiting_restart)))
            .assertExists()
        // Nothing asks the operator to assert the restart: the boot tag on the wire does that.
        compose.onAllNodes(hasText("I power-cycled", substring = true)).fetchSemanticsNodes().let {
            assertEquals(0, it.size)
        }
    }

    /** A refusal belongs to its row, with that row's own retry, and never to a banner. */
    @Test
    fun aRefusalIsShownOnItsOwnRowWithItsOwnRetry() {
        val actions = Recorder()
        val method = SetupFields.MOTOR_METHOD.key
        show(
            setupState(
                board = 0x01,
                values = stored,
                writes = mapOf(method to FieldWrite(Value.U8(2), WriteState.Refused(CfgRefusal.ARMED))),
            ),
            actions = actions,
        )

        val row = setupRowTag(method)
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_write_refused_armed)))
            .performScrollTo().assertExists()
        compose.onNodeWithTag(setupRetryTag(method)).performScrollTo().performClick()
        assertTrue("retry:$method" in actions.calls)
    }

    @Test
    fun aMismatchNamesWhatWasWrittenAndWhatTheBoardHolds() {
        val limit = SetupFields.MOTOR_CURRENT_LIMIT.key
        show(
            setupState(
                board = 0x01,
                values = stored,
                writes = mapOf(limit to FieldWrite(Value.U32(15_000), WriteState.Mismatch(Value.U32(9_000)))),
            ),
        )

        compose.onNodeWithText(s(R.string.setup_write_mismatch, "15000", "9000")).performScrollTo().assertExists()
    }

    @Test
    fun theStandingCaveatsAreShown() {
        show(setupState(board = 0x01, values = stored))

        compose.onNodeWithText(s(R.string.setup_note_motor_method)).assertExists()
        compose.onNodeWithText(s(R.string.setup_note_device_name)).assertExists()
        compose.onNodeWithText(s(R.string.setup_note_current_limit)).assertExists()
    }

    @Test
    fun walkOwnedFieldsAreDisplayedWithNoEditor() {
        show(setupState(board = 0x01, values = stored + (SetupFields.NODE_ADDRESS.key to Value.U8(1))))

        val row = setupRowTag(SetupFields.NODE_ADDRESS.key)
        compose.onNodeWithTag(row).assertExists()
        val editors = compose.onAllNodes(
            hasAnyAncestor(hasTestTag(row)) and (hasSetTextAction() or hasClickAction()),
        ).fetchSemanticsNodes()
        assertEquals(0, editors.size)
    }

    @Test
    fun aMirroredStoredMapIsCalledOut() {
        val mirrored = listOf(-1, 1, 1, -1, 1, -1)
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(mirrored[i]) }
        show(setupState(board = 0x01, values = stored + signs))

        compose.onNodeWithText(s(R.string.setup_orientation_mirrored)).assertExists()
        compose.onNodeWithTag(SETUP_FRAME_BAD_TAG).assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_frame_incomplete_body)).assertIsDisplayed()
    }

    /**
     * 3.6a's third consequence: the frame's two role indices are separate writes, so the screen says
     * what it is doing while the pair is in flight rather than leaving the operator to meet it as a
     * refused arm.
     */
    @Test
    fun aFrameInFlightSaysWhatItIsDoing() {
        val sending = SetupFields.FRAME.associate { it.key to FieldWrite(Value.I32(1), WriteState.Sending) }
        show(setupState(board = 0x01, values = stored, writes = sending))

        compose.onNodeWithTag(SETUP_FRAME_WRITING_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_frame_writing)).assertExists()
    }

    /**
     * A written frame is not what the board is running: it takes the stored map up at its next arm,
     * so the panel must not say "Runs as" of it (P0-1 of the Setup audit, restated for 3.6a).
     */
    @Test
    fun aWrittenFrameIsStoredNotRunning() {
        // Component side down, stock-rear forward: a flat pose, so the roles stay unset.
        val pose = Orientation.Pose(Orientation.Face.COMPONENT_DOWN, Orientation.Heading.STOCK_REAR)
        val frame = Orientation.frameOf(pose)
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(frame.signs[i]) }
        val writes = signs.associate { (k, v) -> k to FieldWrite(v, WriteState.Written) }
        show(setupState(board = 0x01, values = stored + signs, writes = writes))

        val name = s(R.string.setup_pose_name, s(R.string.setup_face_component_down), s(R.string.setup_heading_rear))
        compose.onNodeWithText(s(R.string.setup_orientation_stored_as, name)).assertExists()
        val runsAs = s(R.string.setup_orientation_runs, "").trim()
        assertEquals(0, compose.onAllNodes(hasText(runsAs, substring = true)).fetchSemanticsNodes().size)
        val live = compose.onAllNodes(hasText("live", substring = true, ignoreCase = true))
        assertEquals(0, live.fetchSemanticsNodes().size)
    }

    /**
     * Unset is not the stock pose on this family: it is named as the fallback, nothing is selected,
     * and picking the stock pose writes the family yaw explicitly.
     */
    @Test
    fun anUnsetBoardSelectsNoPoseAndTheStockPoseWritesTheFamilyYaw() {
        val actions = Recorder()
        show(setupState(board = 0x01, values = stored), actions = actions)

        val unset = s(R.string.setup_pose_unset, s(R.string.setup_face_component_down))
        compose.onNodeWithText(s(R.string.setup_orientation_runs, unset)).assertExists()
        compose.onNodeWithText(s(R.string.setup_orientation_unset)).assertExists()
        compose.onNodeWithText(s(R.string.setup_orientation_roles, "Z", "Y")).assertExists()
        compose.onNodeWithText(s(R.string.setup_face_component_up)).assertIsNotSelected()
        compose.onNodeWithText(s(R.string.setup_face_component_down)).assertIsNotSelected()
        compose.onNodeWithText(s(R.string.setup_pose_heading_title)).assertDoesNotExist()
        // The family note names the measured yaw the poses compose with, never the compiled fallback
        // the line above calls upside down.
        val family = s(R.string.setup_orientation_family)
        assertTrue(family, "(the measured yaw, not the firmware's compiled fallback)" in family)
        assertTrue(family, "compiled reference" !in family)
        compose.onNodeWithText(family).assertExists()

        compose.onNodeWithText(s(R.string.setup_face_component_up)).performScrollTo().performClick()
        // The flat face offers the four stock edges, never a side.
        compose.onNodeWithText(s(R.string.setup_heading_component)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_heading_forward)).performScrollTo().performClick()

        assertTrue("frame:[3, 2]:[-1, -1, 1, -1, -1, 1]" in actions.calls)
    }

    @Test
    fun theStoredFamilyYawIsNamedAndSelectedAsTheStockPose() {
        val yaw = listOf(-1, -1, 1, -1, -1, 1)
        val signs = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(yaw[i]) }
        show(setupState(board = 0x01, values = stored + signs))

        val name = s(R.string.setup_pose_name, s(R.string.setup_face_component_up), s(R.string.setup_heading_forward))
        compose.onNodeWithText(s(R.string.setup_orientation_runs, name)).assertExists()
        compose.onNodeWithText(s(R.string.setup_face_component_up)).assertIsSelected()
        compose.onNodeWithText(s(R.string.setup_heading_forward)).assertIsSelected()
    }

    /** A face only chooses the headings offered; the heading writes the whole frame, roles too. */
    @Test
    fun anEdgeDownFaceOffersItsFourHeadingsAndOneWritesBothFields() {
        val actions = Recorder()
        show(setupState(board = 0x01, values = stored), actions = actions)

        compose.onNodeWithText(s(R.string.setup_face_forward_edge_down)).performScrollTo().performClick()
        assertTrue("a face alone wrote a frame", actions.calls.none { it.startsWith("frame:") })
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
        show(setupState(board = 0x01, values = stored + signs))

        val custom = s(R.string.setup_pose_custom, "[0, 0]", mixed.toString())
        compose.onNodeWithText(s(R.string.setup_orientation_runs, custom)).assertExists()
    }

    /** The frame's own rows are displayed, never edited: one sign cannot be a rotation on its own. */
    @Test
    fun aSignOrRoleRowOffersNoEditor() {
        show(setupState(board = 0x01, values = stored))
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.IMU)).performScrollTo().performClick()
        for (key in listOf(SetupFields.AXIS_ROLE[0].key, SetupFields.AXIS_SIGN[0].key)) {
            val row = setupRowTag(key)
            compose.onNodeWithTag(row).performScrollTo().assertExists()
            val editors = compose.onAllNodes(
                hasAnyAncestor(hasTestTag(row)) and (hasSetTextAction() or hasClickAction()),
            ).fetchSemanticsNodes()
            assertEquals("$key offers an editor", 0, editors.size)
        }
    }

    /**
     * P1-2, restated for 3.6a: a frame write that stopped with one index left unread is held the
     * same way a mirrored map is, because the rest of the frame is then in doubt.
     */
    @Test
    fun aFrameWriteThatStoppedWithAnIndexUnreadIsCalledOut() {
        val ax = SetupFields.AXIS_SIGN[0].key
        show(
            setupState(
                board = 0x01,
                values = stored - ax,
                writes = mapOf(ax to FieldWrite(Value.I32(1), WriteState.Written)),
            ).withBoard { it.copy(unread = setOf(ax)) },
        )

        compose.onNodeWithTag(SETUP_FRAME_BAD_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_frame_incomplete_body)).assertIsDisplayed()
    }

    @Test
    fun aFieldWhoseReadFailedSaysNotRead() {
        val name = SetupFields.DEVICE_NAME.key
        show(setupState(board = 0x01, values = stored - name).withBoard { it.copy(unread = setOf(name)) })

        compose.onNode(hasAnyAncestor(hasTestTag(setupRowTag(name))) and hasText(s(R.string.setup_unread)))
            .assertExists()
    }

    /**
     * A board that answered with bytes this app cannot read says so, and does NOT say "Not read",
     * which would be a false statement about a board that replied. A `DEVICE_NAME` holding bytes
     * that are not valid UTF-8 is the case that produces it (`specs/ble.md`, the name's source).
     */
    @Test
    fun aFieldTheBoardAnsweredUnreadablySaysSoRatherThanNotRead() {
        val name = SetupFields.DEVICE_NAME.key
        show(setupState(board = 0x01, values = stored - name).withBoard { it.copy(garbled = setOf(name)) })

        val row = hasAnyAncestor(hasTestTag(setupRowTag(name)))
        compose.onNode(row and hasText(s(R.string.setup_garbled))).assertExists()
        compose.onAllNodes(row and hasText(s(R.string.setup_unread))).assertCountEquals(0)
    }

    /** P2-8: a name too long for one write is refused in the row, with the limit, before any send. */
    @Test
    fun aTooLongNameSaysWhyAndCannotBeSent() {
        val row = setupRowTag(SetupFields.DEVICE_NAME.key)
        show(setupState(board = 0x01, values = stored))

        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasSetTextAction())
            .performTextReplacement("n".repeat(CONFIG_VALUE_MAX + 1))

        compose.onNodeWithText(s(R.string.setup_too_long, CONFIG_VALUE_MAX)).assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_set))).assertIsNotEnabled()
    }

    @Test
    fun theFrameRefusalIsStatedAsNothingWritten() {
        val refused = SetupNotice.FrameRefused(Orientation.Refusal.ACCEL_MIRRORED)
        show(setupState(board = 0x01, values = stored, notice = refused))

        compose.onNodeWithText(s(R.string.setup_notice_accel_mirrored)).assertIsDisplayed()
    }

    /** The rotation check needs the arm that puts a written orientation into effect, not a restart. */
    @Test
    fun theRotationCheckWaitsForTheArmThatAppliesTheOrientation() {
        val sign = SetupFields.AXIS_SIGN[0].key
        show(
            setupState(
                board = 0x01,
                values = stored,
                writes = mapOf(sign to FieldWrite(Value.I32(-1), WriteState.Written)),
            ),
        )
        val blocked = s(R.string.setup_check_blocked)
        assertTrue(blocked, "power-cycle" !in blocked && "Apply" !in blocked)
        compose.onNodeWithText(blocked).assertExists()
        compose.onNodeWithText(s(R.string.setup_check_level)).assertDoesNotExist()
    }

    @Test
    fun theRotationCheckShowsResults() {
        val actions = Recorder()
        show(
            setupState(board = 0x01, values = stored).withBoard {
                it.copy(rotationCheck = RotationCheck(level = CheckResult.PASS, forwardLean = CheckResult.FAIL))
            },
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
        show(setupState(board = 0x01, values = stored), actions = actions)
        val row = setupRowTag(SetupFields.RIDER_REQUIRED.key)
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_choice_not_required)))
            .performScrollTo().performClick()
        assertEquals(listOf(SetupFields.RIDER_REQUIRED.key to Value.U8(0)), actions.edited)
    }

    @Test
    fun theBatteryCalibrationSitsBehindItsGroupsAdvancedToggle() {
        show(setupState(board = 0x01, values = stored))
        val slope = setupRowTag(SetupFields.VBATT_CAL[0].key)
        compose.onNodeWithTag(slope).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.CALIBRATION)).performScrollTo().performClick()
        compose.onNodeWithTag(slope).performScrollTo().assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "25200")).assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "-5")).assertExists()
    }

    @Test
    fun theGainMaximaSitInTuningBehindAdvancedWithTheirDefaults() {
        show(setupState(board = 0x01, values = stored + (SetupFields.GAIN_MAX[0].key to Value.I16(30_000))))
        val gp = setupRowTag(SetupFields.GAIN_MAX[0].key)
        compose.onNodeWithTag(gp).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.TUNING)).performScrollTo().performClick()
        compose.onNodeWithTag(gp).performScrollTo().assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(gp)) and hasText(s(R.string.setup_stored, "30000"))).assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(gp)) and hasText(s(R.string.setup_default, "20000"))).assertExists()
        val gx = setupRowTag(SetupFields.GAIN_MAX[2].key)
        compose.onNode(hasAnyAncestor(hasTestTag(gx)) and hasText(s(R.string.setup_default, "2000")))
            .performScrollTo().assertExists()
        val gv = setupRowTag(SetupFields.GAIN_MAX[3].key)
        compose.onNode(hasAnyAncestor(hasTestTag(gv)) and hasText(s(R.string.setup_default, "10000")))
            .performScrollTo().assertExists()
    }

    @Test
    fun advancedRevealsTheRawEditorsRenderedFromTheirType() {
        show(setupState(board = 0x01, values = stored + (SetupFields.GYRO_BIAS[2].key to Value.I32(-88))))

        compose.onNodeWithTag(setupRowTag(SetupFields.GYRO_BIAS[2].key)).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.IMU)).performScrollTo().performClick()
        compose.onNodeWithTag(setupRowTag(SetupFields.GYRO_BIAS[2].key)).assertExists()
        compose.onNodeWithText(s(R.string.setup_stored, "-88")).assertExists()
    }

    /** One chip per node, labelled by name and address, and a tap selects that node. */
    @Test
    fun theSelectorOffersEveryNodeByNameAndAddressAndATapSelectsIt() {
        val actions = Recorder()
        show(setupState(board = 0x01, values = stored, other = 0x02), actions = actions)

        compose.onNode(selector("Hoverboard 0x01")).assertExists()
        // The other node's store is not read yet, so its name is unknown and its address stands alone.
        compose.onNode(selector("0x02")).assertExists()
        compose.onNodeWithTag(nodeChipTag(SETUP_NODE_TAG, 0x02)).assertIsEnabled().performClick()

        assertTrue("target:2" in actions.calls)
    }

    /** A session with one node offers one chip and nothing for a board it did not find. */
    @Test
    fun aSessionWithOneNodeOffersOneChip() {
        show(setupState(board = 0x01, values = stored))

        compose.onNodeWithTag(nodeChipTag(SETUP_NODE_TAG, 0x01)).assertExists()
        compose.onNodeWithTag(nodeChipTag(SETUP_NODE_TAG, 0x02)).assertDoesNotExist()
    }

    /** A board not on the phone's link is edited like any other, attitude flows included: it reports too. */
    @Test
    fun aBoardBehindTheAttachedOneEditsItsStoreAndKeepsTheAttitudeFlows() {
        val actions = Recorder()
        show(setupState(board = 0x02, values = stored, other = 0x01), actions = actions)

        compose.onNodeWithText(s(R.string.setup_level_action)).performScrollTo().assertExists()
        compose.onNodeWithText(s(R.string.setup_check_level)).performScrollTo().assertExists()
        compose.onNodeWithText(s(R.string.setup_choice_balance)).performScrollTo().performClick()
        assertEquals(listOf(SetupFields.CONTROL_MODE.key to Value.U8(1)), actions.edited)
    }

    /** A write to the selected board is reported on that board's own rows. */
    @Test
    fun theSelectedBoardsOwnWriteIsReportedOnItsRow() {
        val deadTime = SetupFields.MOTOR_DEAD_TIME.key
        show(
            setupState(
                board = 0x02,
                values = stored + (deadTime to Value.U8(32)),
                writes = mapOf(deadTime to FieldWrite(Value.U8(32), WriteState.Written)),
                other = 0x01,
            ),
        )

        val row = setupRowTag(deadTime)
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_write_awaiting_restart)))
            .performScrollTo().assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_effect_boot))).assertExists()
    }

    /**
     * A bridge shows the keys every node has and nothing of a board's (`specs/hoverboard-bridge.md`, "The bridge
     * as a node"): its name, its class read-only, and its link set. No drive, IMU or calibration row,
     * and no attitude flow.
     */
    @Test
    fun aBridgeShowsOnlyItsIdentityRowsWithItsClassReadOnly() {
        val bridge = mapOf(
            SetupFields.NODE_CLASS.key to Value.U8(Fields.NodeClass.BRIDGE),
            SetupFields.DEVICE_NAME.key to Value.Str("hb-bridge-1a2b"),
            SetupFields.LINK_SET.key to Value.U8(1),
        )
        show(setupState(board = 0x01, values = bridge, other = 0x02))

        for (row in SetupFields.BRIDGE) compose.onNodeWithTag(setupRowTag(row.key)).performScrollTo().assertExists()
        for (row in SetupFields.ALL - SetupFields.BRIDGE.toSet()) {
            compose.onNodeWithTag(setupRowTag(row.key)).assertDoesNotExist()
        }
        compose.onNodeWithText(s(R.string.setup_level_action)).assertDoesNotExist()
        // The class is shown and cannot be edited: its row offers no control that writes.
        val classRow = hasAnyAncestor(hasTestTag(setupRowTag(SetupFields.NODE_CLASS.key)))
        compose.onAllNodes(classRow and hasSetTextAction()).assertCountEquals(0)
        compose.onAllNodes(classRow and hasText(s(R.string.setup_set))).assertCountEquals(0)
        compose.onNode(selector("hb-bridge-1a2b 0x01")).assertExists()
    }
}
