package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.assertIsNotSelected
import androidx.compose.ui.test.assertIsSelected
import androidx.compose.ui.test.hasAnyAncestor
import androidx.compose.ui.test.hasTestTag
import androidx.compose.ui.test.hasText
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithTag
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onFirst
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SetupGroup
import com.hoverboard.remote.ui.screens.MACHINE_BADGE_TAG
import com.hoverboard.remote.ui.screens.MACHINE_MODE_TAG
import com.hoverboard.remote.ui.screens.MACHINE_SECTION_TAG
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.machineModeTag
import com.hoverboard.remote.ui.screens.machineModeUseTag
import com.hoverboard.remote.ui.screens.machineSideTag
import com.hoverboard.remote.ui.screens.machineWriteTag
import com.hoverboard.remote.ui.screens.setupAdvancedTag
import com.hoverboard.remote.ui.screens.setupRowTag
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * The Machine section of the Setup screen (`specs/remote-ui.md` 3.2 and 3.7a): the drive mode, the
 * frame and the right wheel, read across every motor controller, written to every one, and badged
 * when the copies disagree. Apart from [SetupScreenTest] only for its size.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupMachineScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    private val stored: Map<Key, Value> = SetupFields.ALL.associate { it.key to it.def.default }

    private fun show(
        state: SetupState,
        armed: Boolean = false,
        actions: SetupActions = Recorder(),
    ) = compose.setContent {
        HoverboardRemoteTheme {
            SetupScreen(
                state = state,
                armed = armed,
                telemetry = null,
                odometers = emptyList(),
                actions = actions,
            )
        }
    }

    // --- the Machine section (`specs/remote-ui.md` 3.7a) ----------------------------------------

    /**
     * A state holding two motor controllers' stores, 0x01 (attached) and 0x02, which the Machine
     * section reads across. A null [slaveValues] is a session that found only 0x01.
     */
    private fun machineState(
        masterValues: Map<Key, Value> = stored,
        slaveValues: Map<Key, Value>? = stored,
        masterWrites: Map<Key, FieldWrite> = emptyMap(),
        slaveWrites: Map<Key, FieldWrite> = emptyMap(),
    ) = SetupState(
        addresses = listOfNotNull(0x01, slaveValues?.let { 0x02 }),
        attached = 0x01,
        boards = buildMap {
            put(0x01, BoardSetup(values = masterValues, writes = masterWrites))
            if (slaveValues != null) put(0x02, BoardSetup(values = slaveValues, writes = slaveWrites))
        },
    )

    private fun modeIs(mode: Int) = mapOf(SetupFields.DRIVE_MODE.key to Value.U8(mode))

    private fun frameIs(rigid: Boolean) =
        mapOf(SetupFields.CONTROL_FRAME.key to Value.U8(if (rigid) 1 else 0))

    private fun sideIs(right: Boolean) =
        mapOf(SetupFields.CONTROL_SIDE.key to Value.U8(if (right) 1 else 0))

    @Test
    fun theMachineSectionOffersItsRowsAndSaysTheyGoToEveryBoard() {
        val actions = Recorder()
        show(machineState(), actions = actions)

        compose.onNodeWithTag(MACHINE_SECTION_TAG).performScrollTo().assertIsDisplayed()
        compose.onNodeWithText(s(R.string.setup_machine_body)).assertExists()
        compose.onNodeWithText(s(R.string.setup_machine_order)).assertExists()
        // The take-effect line is the generated one, the same as every other row's.
        compose.onAllNodesWithText(s(R.string.setup_effect_arm)).onFirst().assertExists()

        compose.onNodeWithText(s(R.string.setup_choice_frame_rigid)).performScrollTo().performClick()
        assertTrue("chooseFrame:1" in actions.calls)
        // No button to press afterwards and nothing to discard: the tap is the write.
        for (gone in listOf("Write to both boards", "Discard")) {
            assertEquals(0, compose.onAllNodes(hasText(gone, substring = true)).fetchSemanticsNodes().size)
        }
    }

    /** Picking the right wheel is the write, and the model is the one that orders the pair. */
    @Test
    fun pickingTheRightWheelAsksForThePairWrite() {
        val actions = Recorder()
        show(machineState(), actions = actions)

        compose.onNodeWithTag(machineSideTag(0x02)).performScrollTo().performClick()
        assertTrue("chooseRightWheel:2" in actions.calls)
    }

    /** The verdict a pair that agrees reads, and the badge is absent. */
    @Test
    fun anAgreeingPairReadsAgreeWithNoBadge() {
        show(machineState(masterValues = stored + frameIs(true), slaveValues = stored + frameIs(true)))

        val verdict = s(R.string.setup_machine_frame_agree, s(R.string.setup_choice_frame_rigid))
        compose.onNodeWithText(verdict).performScrollTo().assertExists()
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).assertCountEquals(0)
    }

    /** A pair that differs names each board's value and carries the badge. */
    @Test
    fun aDifferingPairNamesBothValuesAndBadges() {
        show(machineState(masterValues = stored + frameIs(true), slaveValues = stored + frameIs(false)))

        val verdict = s(R.string.setup_machine_value, "Hoverboard 0x01", s(R.string.setup_choice_frame_rigid)) +
            " " + s(R.string.setup_machine_value, "Hoverboard 0x02", s(R.string.setup_choice_frame_pivoting))
        compose.onNodeWithText(verdict).performScrollTo().assertExists()
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).onFirst().assertExists()
    }

    /** The picker cannot express two right wheels, but a raw edit can, and it is badged. */
    @Test
    fun twoRightWheelsReadBothRightAndSelectNothing() {
        show(machineState(masterValues = stored + sideIs(true), slaveValues = stored + sideIs(true)))

        compose.onNodeWithText(s(R.string.setup_machine_side_both)).performScrollTo().assertExists()
        compose.onNodeWithTag(machineSideTag(0x01)).assertIsNotSelected()
        compose.onNodeWithTag(machineSideTag(0x02)).assertIsNotSelected()
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).onFirst().assertExists()
    }

    /** A fresh pair: no right wheel, said plainly, and nothing preselected. */
    @Test
    fun afreshPairReadsNoRightWheelAndPreselectsNothing() {
        show(machineState())

        compose.onNodeWithText(s(R.string.setup_machine_side_none)).performScrollTo().assertExists()
        compose.onNodeWithTag(machineSideTag(0x01)).assertIsNotSelected()
        compose.onNodeWithTag(machineSideTag(0x02)).assertIsNotSelected()
    }

    /**
     * With one motor controller found the Right wheel picker is disabled (`specs/remote-ui.md` 3.7a:
     * fewer than two) and the section says a write reaches one board.
     */
    @Test
    fun withOneMotorControllerTheRightWheelPickerIsDisabled() {
        show(machineState(slaveValues = null))

        compose.onNodeWithText(s(R.string.setup_machine_no_peer)).performScrollTo().assertExists()
        compose.onNodeWithTag(machineSideTag(0x01)).performScrollTo().assertIsNotEnabled()
        compose.onNodeWithTag(machineSideTag(0x02)).assertDoesNotExist()
        // Frame and side: neither verdict can be reached with half the machine out of sight.
        compose.onAllNodesWithText(s(R.string.setup_machine_unknown)).assertCountEquals(2)
    }

    /**
     * Each board reports its own write under the row (`specs/remote-ui.md` 3.7a), so a pair write
     * that one board refused says which board refused it.
     */
    @Test
    fun eachBoardReportsItsOwnMachineWrite() {
        val frame = SetupFields.CONTROL_FRAME.key
        show(
            machineState(
                masterValues = stored + frameIs(true),
                masterWrites = mapOf(frame to FieldWrite(Value.U8(1), WriteState.Written)),
                slaveWrites = mapOf(frame to FieldWrite(Value.U8(1), WriteState.Refused(CfgRefusal.ARMED))),
            ),
        )

        compose.onNodeWithTag(machineWriteTag(frame, 0x01)).performScrollTo().assertExists()
        val refused = s(
            R.string.setup_machine_board_line,
            "Hoverboard 0x02",
            s(R.string.setup_write_refused_armed),
        )
        compose.onNodeWithText(refused).performScrollTo().assertExists()
    }

    /**
     * The right wheel's write is held back when the left wheel's did not land, so the pair is never
     * left with two right wheels (`specs/remote-ui.md` 3.7a).
     */
    @Test
    fun aHeldBackRightWheelWriteSaysWhyItWasNotSent() {
        val side = SetupFields.CONTROL_SIDE.key
        show(
            machineState(
                masterWrites = mapOf(side to FieldWrite(Value.U8(1), WriteState.Skipped)),
                slaveWrites = mapOf(side to FieldWrite(Value.U8(0), WriteState.Unanswered)),
            ),
        )

        compose.onNodeWithTag(machineWriteTag(side, 0x01)).performScrollTo().assertExists()
        compose.onNodeWithText(
            s(
                R.string.setup_machine_board_line,
                "Hoverboard 0x01",
                s(R.string.setup_write_skipped),
            ),
        ).performScrollTo().assertExists()
    }

    /** The raw rows under ADVANCED carry the pair verdict, so a raw edit that breaks it is badged. */
    @Test
    fun theRawMachineRowsCarryThePairVerdict() {
        show(machineState(masterValues = stored + frameIs(true), slaveValues = stored + frameIs(false)))

        val row = setupRowTag(SetupFields.CONTROL_FRAME.key)
        compose.onNodeWithTag(row).assertDoesNotExist()
        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.DRIVE)).performScrollTo().performClick()
        compose.onNodeWithTag(row).performScrollTo().assertExists()
        // One verdict in the Machine section and one under the raw row.
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).assertCountEquals(2)
    }

    /** An arm-time field is written, not waiting on a restart, and it asks for no power-cycle. */
    @Test
    fun aWrittenMachineFieldSaysItTakesEffectAtTheNextArm() {
        val frame = SetupFields.CONTROL_FRAME.key
        show(
            machineState(
                masterValues = stored + (frame to Value.U8(1)),
                slaveValues = null,
                masterWrites = mapOf(frame to FieldWrite(Value.U8(1), WriteState.Written)),
            ),
        )

        compose.onNodeWithTag(setupAdvancedTag(SetupGroup.DRIVE)).performScrollTo().performClick()
        val row = setupRowTag(frame)
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_write_written)))
            .performScrollTo().assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_effect_arm)))
            .assertExists()
        compose.onNode(hasAnyAncestor(hasTestTag(row)) and hasText(s(R.string.setup_write_awaiting_restart)))
            .assertDoesNotExist()
    }

    // --- the drive mode under Machine (`specs/remote-ui.md` 3.2) ---------------------------------

    /** A drive-mode chip is the write: the model sends it to every motor controller. */
    @Test
    fun tappingADriveModeAsksForTheWriteToEveryBoard() {
        val actions = Recorder()
        show(machineState(), actions = actions)

        compose.onNodeWithTag(MACHINE_MODE_TAG).performScrollTo().assertExists()
        compose.onNodeWithTag(machineModeTag(DriveMode.SINGLE)).assertIsSelected()
        compose.onNodeWithTag(machineModeTag(DriveMode.BOUND)).performScrollTo().performClick()
        assertTrue("chooseDriveMode:${DriveMode.BOUND.byte}" in actions.calls)
    }

    /**
     * Copies that disagree are badged, named per board, select no chip, and offer a one-tap write of
     * each value the boards hold to every board.
     */
    @Test
    fun differingDriveModesBadgeAndOfferEitherValueToAll() {
        val actions = Recorder()
        show(
            machineState(
                masterValues = stored + modeIs(DriveMode.BOUND.byte),
                slaveValues = stored + modeIs(DriveMode.RIGID.byte),
            ),
            actions = actions,
        )

        val verdict = s(R.string.setup_machine_value, "Hoverboard 0x01", s(R.string.drive_mode_bound)) +
            " " + s(R.string.setup_machine_value, "Hoverboard 0x02", s(R.string.drive_mode_rigid))
        compose.onNodeWithText(verdict).performScrollTo().assertExists()
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).onFirst().assertExists()
        for (mode in DriveMode.entries) compose.onNodeWithTag(machineModeTag(mode)).assertIsNotSelected()

        compose.onNodeWithTag(machineModeUseTag(DriveMode.RIGID.byte)).performScrollTo().performClick()
        compose.onNodeWithTag(machineModeUseTag(DriveMode.BOUND.byte)).performScrollTo().performClick()
        assertEquals(
            listOf("chooseDriveMode:${DriveMode.RIGID.byte}", "chooseDriveMode:${DriveMode.BOUND.byte}"),
            actions.calls.filter { it.startsWith("chooseDriveMode") },
        )
    }

    /** Copies that agree select their chip, read agree, and carry no badge and no use-value button. */
    @Test
    fun agreeingDriveModesSelectTheirChipWithNoBadge() {
        val both = stored + modeIs(DriveMode.DIFFERENTIAL.byte)
        show(machineState(masterValues = both, slaveValues = both))

        compose.onNodeWithTag(machineModeTag(DriveMode.DIFFERENTIAL)).performScrollTo().assertIsSelected()
        compose.onNodeWithText(s(R.string.setup_machine_frame_agree, s(R.string.drive_mode_differential)))
            .performScrollTo().assertExists()
        compose.onAllNodesWithTag(MACHINE_BADGE_TAG).assertCountEquals(0)
        compose.onNodeWithTag(machineModeUseTag(DriveMode.DIFFERENTIAL.byte)).assertDoesNotExist()
    }

    /** A drive-mode copy left unread reads unknown rather than claiming a mode. */
    @Test
    fun anUnreadDriveModeReadsUnknown() {
        show(machineState(slaveValues = stored - SetupFields.DRIVE_MODE.key))

        compose.onNodeWithText(s(R.string.setup_machine_mode_unknown)).performScrollTo().assertExists()
        for (mode in DriveMode.entries) compose.onNodeWithTag(machineModeTag(mode)).assertIsNotSelected()
    }
}
