package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.MachineArm
import com.hoverboard.remote.R
import com.hoverboard.remote.RiderWaiver
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.MissingSetting
import com.hoverboard.remote.ui.components.ArmToggle
import com.hoverboard.remote.ui.components.JoystickPad
import com.hoverboard.remote.ui.components.ThrottlePad
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Tone
import com.hoverboard.remote.ui.theme.ToneText

/**
 * The two controls that move a machine and stop it, the pad under the thumb and the arm toggle
 * under it, with the statements about arming under that (`specs/remote-ui.md` 3.1).
 *
 * They are one composable because they belong together wherever they appear: the Ride screen, and
 * the Tune screen's copy (3.3b), which takes the same drawing and the same statements so the gains
 * and the drive bounds can be found by riding the machine. A screen that can move a machine and not
 * stop it would be the wrong half to copy, so neither is offered without the other.
 *
 * Nothing here is per screen: the state, the arming and the demand are the one session's, so a pad
 * on either screen drives the same board through the same callbacks.
 *
 * @param grid what the screen draws in the pad's bounds in place of the pad, the fault grid while it
 *   shows (`specs/remote-ui.md` 3.1b), or null for the pad. The grid says why the arm is refused, so
 *   no statement is drawn under the control while it is there.
 * @param padShown false while the screen draws something else in the pad's place and above it (the
 *   motor-controller-off card, 3.1): no pad, no grid and no statements.
 * @param statementsHeight the fixed height of the statements' slot on Ride (3.1), so the slots under
 *   it never move; null on Tune, where the statements take what they need.
 * @param onOpenLamp open a lamp's sheet, which the fault-disarm statement does when tapped. The
 *   caller releases the drive first: the sheet's scrim covers the pad.
 * @param onOpenSetup open Setup on a board, scrolled to a row, which a missing setting's statement
 *   does when tapped (3.1, "Missing settings, in plain words").
 */
@Composable
internal fun RideControls(
    state: UiState,
    onArmToggle: () -> Unit,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onJoystickMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
    modifier: Modifier = Modifier,
    grid: (@Composable () -> Unit)? = null,
    padShown: Boolean = true,
    statementsHeight: Dp? = null,
    onOpenLamp: (FaultLamp) -> Unit = {},
    onOpenSetup: (board: Int, row: Key) -> Unit = { _, _ -> },
) {
    Column(modifier = modifier.fillMaxWidth()) {
        if (padShown) {
            if (grid != null) {
                Box(Modifier.fillMaxWidth().height(PAD_HEIGHT).testTag(RIDE_PAD_SLOT_TAG)) { grid() }
            } else {
                DriveControl(
                    state = state,
                    onThrottleMove = onThrottleMove,
                    onJoystickMove = onJoystickMove,
                    onThrottleRelease = onThrottleRelease,
                )
            }
            Spacer(modifier = Modifier.height(SLOT_GAP))
        }
        ArmToggle(armed = state.armed, enabled = state.canArm, onToggle = onArmToggle)
        val shown = if (grid != null || !padShown) emptyList() else statements(state).take(MAX_STATEMENTS)
        if (statementsHeight != null) {
            Spacer(modifier = Modifier.height(STATEMENTS_GAP))
            Column(Modifier.fillMaxWidth().height(statementsHeight)) {
                for (s in shown) StatementLine(s, onOpenLamp, onOpenSetup)
            }
        } else {
            if (shown.isNotEmpty()) Spacer(modifier = Modifier.height(SLOT_GAP))
            for (s in shown) StatementLine(s, onOpenLamp, onOpenSetup)
        }
    }
}

/**
 * One statement line, 12 sp, one line, ellipsised: tapping it opens its lamp's sheet, or Setup at
 * its missing setting's row, whose "Set it ›" ends the line in [Primary].
 */
@Composable
private fun StatementLine(
    s: Statement,
    onOpenLamp: (FaultLamp) -> Unit,
    onOpenSetup: (board: Int, row: Key) -> Unit,
) {
    val click = when {
        s.lamp != null -> Modifier.clickable { onOpenLamp(s.lamp) }
        s.setup != null -> Modifier.clickable { onOpenSetup(s.setup.first, s.setup.second) }
        else -> Modifier
    }
    Row(
        modifier = Modifier.fillMaxWidth().then(click).testTag(RIDE_STATEMENT_TAG),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        ToneText(
            s.text,
            s.tone,
            style = ride(12.sp),
            maxLines = 1,
            modifier = Modifier.weight(1f, fill = false),
        )
        if (s.setup != null) {
            Spacer(Modifier.width(6.dp))
            Text(
                stringResource(R.string.ride_set_it),
                style = ride(12.sp, FontWeight.Bold),
                color = Primary,
                maxLines = 1,
            )
        }
    }
}

/**
 * One statement under the ARM control: its text, its tone, and what a tap opens, if anything: a
 * lamp's sheet, or Setup on a board at a row.
 */
private data class Statement(
    val text: String,
    val tone: Tone,
    val lamp: FaultLamp? = null,
    val setup: Pair<Int, Key>? = null,
)

/**
 * Every statement that applies to [state], in the order the Ride screen shows them
 * (`specs/remote-ui.md` 3.1); the screen shows the first [MAX_STATEMENTS]. The long forms are on
 * Setup's machine status section (3.1c).
 */
@Composable
private fun statements(state: UiState): List<Statement> =
    ridingStatements(state) + disarmStatements(state) + armStatements(state) + machineStatements(state) +
        missingStatements(state)

/**
 * What is wrong while riding: a driven wheel that does not turn, and a lamp lit on a board the arm
 * does not drive, which the grid does not show while armed (`specs/remote-ui.md` 3.1b).
 */
@Composable
private fun ridingStatements(state: UiState): List<Statement> = buildList {
    for (board in state.notTurning.sorted()) {
        add(Statement(stringResource(R.string.ride_not_turning, boardName(state, board, inline = false)), Tone.WARNING))
    }
    if (state.armed) {
        for ((board, word) in state.lampWords.toSortedMap()) {
            val lamp = FaultLamp.lit(word).firstOrNull() ?: continue
            val text = stringResource(
                R.string.ride_lamp_lit_armed,
                stringResource(lamp.title),
                boardName(state, board, inline = true),
            )
            add(Statement(text, Tone.WARNING, lamp))
        }
    }
}

/** The settings a figure lacks, in plain words (`specs/remote-ui.md` 3.1, "Missing settings"). */
@Composable
private fun missingStatements(state: UiState): List<Statement> = state.missing.map { m ->
    val text = stringResource(
        when (m.setting) {
            MissingSetting.WHEEL_SIZE -> R.string.ride_need_wheel_size
            MissingSetting.WHEEL_READING -> R.string.ride_reading_wheel
            MissingSetting.CELLS -> R.string.ride_need_cells
            MissingSetting.CHEMISTRY_READING -> R.string.ride_reading_chemistry
            MissingSetting.CHEMISTRY_UNKNOWN -> R.string.ride_need_chemistry
            MissingSetting.CURRENT_CAL -> R.string.ride_need_current_cal
            MissingSetting.CURRENT_CAL_READING -> R.string.ride_reading_current_cal
        },
    )
    val row = m.setting.row
    Statement(text, Tone.MUTED, setup = if (row != null && m.board != null) m.board to row.key else null)
}

/** Why the machine is not armed when the app took the level away: a fault, or leaving the screen. */
@Composable
private fun disarmStatements(state: UiState): List<Statement> = buildList {
    state.faultDisarm?.let {
        val text = stringResource(
            R.string.ride_disarmed_fault,
            stringResource(it.lamp.title),
            boardName(state, it.address, inline = true),
        )
        add(Statement(text, Tone.WARNING, it.lamp))
    }
    if (state.selfDisarmed) add(Statement(stringResource(R.string.ride_self_disarmed), Tone.MUTED))
}

/** What refuses the arm: frames that differ, a drive refusal, or no motor controller found. */
@Composable
private fun armStatements(state: UiState): List<Statement> = buildList {
    if (state.machineArm is MachineArm.FramesDiffer) {
        add(Statement(stringResource(R.string.ride_frames_differ), Tone.WARNING))
    }
    val refusal = state.drive.refusal
    if (state.attachedNode != null && refusal != null) {
        val tone = if (refusal == DriveRefusal.UNREAD) Tone.MUTED else Tone.WARNING
        add(Statement(refusalText(state, refusal, long = false), tone))
    }
    if (state.isConnected && refusal == null && state.drive.targets.boards.isEmpty()) {
        add(Statement(stringResource(R.string.ride_no_board), Tone.MUTED))
    }
}

/** What needs care when arming: the rider requirement waived, an unread frame, the right wheel. */
@Composable
private fun machineStatements(state: UiState): List<Statement> = buildList {
    when (state.riderWaiver) {
        RiderWaiver.WAIVED -> add(Statement(stringResource(R.string.ride_rider_waived), Tone.CAUTION))
        RiderWaiver.POSSIBLY -> add(Statement(stringResource(R.string.ride_rider_maybe_waived), Tone.CAUTION))
        RiderWaiver.NONE -> Unit
    }
    val line = when (val machine = state.machineArm) {
        is MachineArm.FrameUnread -> if (machine.peerFound) R.string.ride_frame_unread else R.string.ride_frame_no_peer
        is MachineArm.SideNotOneRight -> when (machine.pair) {
            SidePair.BothRight -> R.string.ride_side_both_right
            SidePair.NoneRight -> R.string.ride_side_none_right
            else -> R.string.ride_side_unknown
        }
        MachineArm.None, is MachineArm.FramesDiffer -> null
    }
    line?.let { add(Statement(stringResource(it), Tone.CAUTION)) }
}

/** A drive refusal's text: the Ride statement, or with [long] Setup's line (`specs/remote-ui.md` 3.1c). */
@Composable
internal fun refusalText(state: UiState, refusal: DriveRefusal, long: Boolean): String {
    val mode = state.drive.stored?.name.orEmpty()
    return when (refusal) {
        DriveRefusal.UNREAD ->
            stringResource(if (long) R.string.drive_refusal_unread_long else R.string.drive_refusal_unread)
        DriveRefusal.DISAGREE ->
            stringResource(if (long) R.string.drive_refusal_disagree_long else R.string.drive_refusal_disagree)
        DriveRefusal.UNKNOWN_MODE ->
            stringResource(if (long) R.string.drive_refusal_unknown_mode_long else R.string.drive_refusal_unknown_mode)
        DriveRefusal.NOT_TWO -> stringResource(
            if (long) R.string.drive_refusal_not_two_long else R.string.drive_refusal_not_two,
            mode,
            state.motorControllers.size,
        )
        DriveRefusal.NO_LEADER ->
            stringResource(if (long) R.string.drive_refusal_no_leader_long else R.string.drive_refusal_no_leader)
        DriveRefusal.NO_RIGHT_WHEEL ->
            stringResource(if (long) R.string.drive_refusal_no_right_long else R.string.drive_refusal_no_right)
    }
}

/**
 * The control the mode puts under the thumb: the two-axis stick in the modes that steer
 * ([com.hoverboard.remote.model.DriveMode.hasTurnAxis]) and the throttle pad in the ones that send
 * one value per board. One or the other, never both, and under a drive refusal the stored mode's.
 *
 * Both are live only while connected AND armed, and both release through the same callback, so
 * whichever is on screen the finger-up path is one path.
 */
@Composable
private fun DriveControl(
    state: UiState,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onJoystickMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
) {
    val size = Modifier.fillMaxWidth().height(PAD_HEIGHT)
    val live = state.isConnected && state.armed
    if (state.driveMode.hasTurnAxis) {
        JoystickPad(
            speed = state.throttleSpeed,
            steer = state.steer,
            engaged = state.engaged,
            enabled = live,
            blocked = state.armBlocked,
            onMove = onJoystickMove,
            onRelease = onThrottleRelease,
            modifier = size,
        )
    } else {
        ThrottlePad(
            speed = state.throttleSpeed,
            engaged = state.engaged,
            enabled = live,
            blocked = state.armBlocked,
            onMove = onThrottleMove,
            onRelease = onThrottleRelease,
            modifier = size,
        )
    }
}

/** Test tag on each statement line under the ARM control. */
const val RIDE_STATEMENT_TAG = "ride_statement"

/** Test tag on the pad's bounds while something else is drawn in them (the fault grid). */
const val RIDE_PAD_SLOT_TAG = "ride_pad_slot"

/** The gap between the ARM control and the statements' fixed slot on Ride. */
private val STATEMENTS_GAP = 8.dp

/**
 * The drive pad's height: its y range is the demand's resolution, and it is taller than the 260 dp
 * floor the throttle had.
 */
internal val PAD_HEIGHT = 280.dp

/** The gap between the Ride column's fixed slots. */
internal val SLOT_GAP = 12.dp

/** The statements shown at once. */
private const val MAX_STATEMENTS = 2
