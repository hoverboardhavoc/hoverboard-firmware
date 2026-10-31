package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.FramePair
import com.hoverboard.remote.MachineArm
import com.hoverboard.remote.R
import com.hoverboard.remote.RiderWaiver
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.BoardMode
import com.hoverboard.remote.model.DriveRefusal
import com.hoverboard.remote.model.FaultCause
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.components.CurrentSection
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.Tone
import com.hoverboard.remote.ui.theme.ToneText
import com.hoverboard.remote.ui.theme.WarningText

/** Test tag on the machine status section. */
const val MACHINE_STATUS_TAG = "machine_status"

/** Test tag on the section's drive line. */
const val STATUS_DRIVE_TAG = "status_drive"

/** Test tag on the line that explains a rigid pair whose boards are not leading yet. */
const val RIGID_NOT_LEADING_TAG = "rigid_not_leading"

/** Test tag on the state line of the motor controller at [address]. */
fun statusBoardTag(address: Int): String = "status_board_${boardHex(address)}"

/** Test tag for the bench rider-simulation row. */
const val SIM_RIDER_TAG = "simulate_rider"

/**
 * Setup's machine status section (`specs/remote-ui.md` 3.1c): what the Ride screen used to carry
 * above its pad, for every board at once and independent of the node selector. In order: the drive
 * in force or why the app refuses it, what the arm control knows about the machine, the
 * rigid-not-leading line, one row per motor controller with that board's own lines, the current,
 * and in debug builds the SIMULATE RIDER row.
 *
 * @param showSimulateRider whether the build carries the bench rider control (debug only).
 */
@Composable
fun MachineStatusSection(
    state: UiState,
    showSimulateRider: Boolean,
    onSimulateRider: (Boolean) -> Unit,
    modifier: Modifier = Modifier,
) {
    Section(stringResource(R.string.status_title)) {
        Column(
            modifier = modifier.testTag(MACHINE_STATUS_TAG),
            verticalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            DriveLine(state)
            MachineLines(state)
            if (rigidIsConfiguredButNotRunning(state)) {
                CautionText(
                    stringResource(R.string.drive_mode_rigid_not_leading),
                    modifier = Modifier.testTag(RIGID_NOT_LEADING_TAG),
                )
            }
            for (node in state.motorControllers) BoardRow(state, node.address, nodeLabel(node))
            CurrentSection(state.motorControllers.map { nodeLabel(it) to state.currentOf(it.address) })
            if (showSimulateRider) SimulateRiderRow(on = state.simulateRider, onChange = onSimulateRider)
        }
    }
}

/** "Drive: BOUND, Hoverboard 0x01 and Hoverboard 0x02", or the drive refusal's long text. */
@Composable
private fun DriveLine(state: UiState) {
    val modifier = Modifier.testTag(STATUS_DRIVE_TAG)
    val refusal = state.drive.refusal
    when {
        refusal != null -> ToneText(
            refusalText(state, refusal, long = true),
            if (refusal == DriveRefusal.UNREAD) Tone.MUTED else Tone.WARNING,
            modifier = modifier,
        )
        state.drive.targets.boards.isEmpty() ->
            Text(stringResource(R.string.status_drive_none), color = FgMuted, modifier = modifier)
        else -> {
            val boards = state.drive.targets.boards.map { a -> state.nodeAt(a)?.let { nodeLabel(it) } ?: boardHex(a) }
            val named = boards.reduce { a, b -> stringResource(R.string.status_and, a, b) }
            Text(stringResource(R.string.status_drive, state.drive.mode.name, named), color = Fg, modifier = modifier)
        }
    }
}

/**
 * What the arm control knows about the machine, in full: the [MachineArm] statement (warning for a
 * frame difference, caution for the rest) and the rider-waiver line.
 */
@Composable
private fun MachineLines(state: UiState) {
    when (val machine = state.machineArm) {
        MachineArm.None -> Unit
        is MachineArm.FramesDiffer -> WarningText(
            stringResource(
                R.string.arm_frames_differ,
                machine.rigid.toSortedMap().map { (address, rigid) ->
                    "${boardHex(address)} ${stringResource(frameName(rigid))}"
                }.joinToString(", "),
            ),
        )
        is MachineArm.FrameUnread ->
            CautionText(
                stringResource(if (machine.peerFound) R.string.arm_frame_unread else R.string.arm_frame_no_peer),
            )
        is MachineArm.SideNotOneRight -> CautionText(
            stringResource(
                when (machine.pair) {
                    SidePair.BothRight -> R.string.arm_side_both_right
                    SidePair.NoneRight -> R.string.arm_side_none_right
                    else -> R.string.arm_side_unknown
                },
            ),
        )
    }
    when (state.riderWaiver) {
        RiderWaiver.WAIVED -> CautionText(stringResource(R.string.arm_rider_waived))
        RiderWaiver.POSSIBLY -> CautionText(stringResource(R.string.arm_rider_maybe_waived))
        RiderWaiver.NONE -> Unit
    }
}

private fun frameName(rigid: Boolean): Int =
    if (rigid) R.string.setup_choice_frame_rigid else R.string.setup_choice_frame_pivoting

/**
 * Whether the machine is stored as one rigid machine and yet no board is leading one: the frame was
 * set under Machine, so the owner is expecting the mode, and the boards are still resolving to
 * something else, which on this firmware means the control mode is not balance or the frame has not
 * been armed in yet. An ordinary pivoting machine reaches none of this and is told nothing, which is
 * why the test is the stored frame rather than the absent leader.
 */
private fun rigidIsConfiguredButNotRunning(state: UiState): Boolean {
    val frame = state.machine.frame
    return frame is FramePair.Agree && frame.rigid && state.leader == null
}

/**
 * One motor controller's row: its label and the state word its own frames give, then its own
 * lines: the rider flag, LOCKDOWN, one line per set fault cause, the bit-6 mode readback, which
 * is not a fault, and the app's own reading of a driven wheel that does not turn
 * (`specs/remote-ui.md` 3.1). A silent board has its word and nothing under it.
 */
@Composable
private fun BoardRow(state: UiState, address: Int, label: String) {
    val telemetry = state.telemetryOf(address)
    val (word, tone) = boardSays(telemetry)
    ToneText(
        stringResource(R.string.board_chip, label, stringResource(word)),
        tone,
        style = MaterialTheme.typography.titleSmall,
        modifier = Modifier.padding(top = 6.dp).testTag(statusBoardTag(address)),
    )
    if (telemetry == null || !telemetry.hasState) return
    val small = MaterialTheme.typography.bodySmall
    if (telemetry.riderPresent) {
        Text(stringResource(R.string.status_rider), style = small, color = Primary)
    } else {
        Text(stringResource(R.string.status_no_rider), style = small, color = FgMuted)
    }
    if (telemetry.lockdown) CautionText(stringResource(R.string.status_lockdown), style = small)
    for (cause in telemetry.faultCauses) {
        WarningText(stringResource(R.string.telemetry_fault_cause, label, stringResource(cause.label())), style = small)
    }
    if ((telemetry.cyclic?.fault ?: 0) and CyclicState.EV_MODE_FAULT != 0) {
        CautionText(stringResource(R.string.board_mode_demoted, label), style = small)
    }
    if (address in state.notTurning) {
        WarningText(stringResource(R.string.status_not_turning), style = small)
    }
}

/**
 * What a board's row says about it, from that board's own frames: SILENT until it has been heard
 * from and again once it has gone quiet, then a fault or a lockdown ahead of the mode, because
 * either one is what an owner needs off a glance and the mode alone would hide it.
 *
 * Never the word ARMED: the arm control is the one armed statement in the app. A board in
 * [BoardMode.RUN] is a board whose motors are live, which is the fact.
 */
private fun boardSays(telemetry: TelemetryUi?): Pair<Int, Tone> = when {
    telemetry == null || !telemetry.hasState -> R.string.board_state_quiet to Tone.MUTED
    telemetry.anyFault -> R.string.board_state_fault to Tone.WARNING
    telemetry.lockdown -> R.string.board_state_lockdown to Tone.CAUTION
    else -> when (telemetry.mode) {
        BoardMode.OFF -> R.string.board_state_off to Tone.MUTED
        BoardMode.INIT, BoardMode.READY -> R.string.board_state_starting to Tone.MUTED
        BoardMode.RUN -> R.string.board_state_live to Tone.PRIMARY
        BoardMode.SHUTDOWN -> R.string.board_state_stopping to Tone.MUTED
        else -> R.string.board_state_unrecognised to Tone.MUTED
    }
}

/** The string a [FaultCause] is named by. */
@StringRes
private fun FaultCause.label(): Int = when (this) {
    FaultCause.COMMS_LOSS -> R.string.fault_cause_comms_loss
    FaultCause.TILT_STOP -> R.string.fault_cause_tilt_stop
    FaultCause.IMU_LOSS -> R.string.fault_cause_imu_loss
    FaultCause.LATCH_A -> R.string.fault_cause_latch_a
    FaultCause.LATCH_B -> R.string.fault_cause_latch_b
    FaultCause.MOTOR_HALL -> R.string.fault_cause_motor_hall
    FaultCause.MOTOR_CAL -> R.string.fault_cause_motor_cal
    FaultCause.MOTOR_PERIOD -> R.string.fault_cause_motor_period
    FaultCause.MOTOR_ARM -> R.string.fault_cause_motor_arm
}

/**
 * The bench affordance that stands in for foot pads.
 *
 * The app no longer asserts the `INPUTS` rider bit as a copy of the arm level, because the boards
 * have real pads and a remote that claims a rider overrides them
 * ([com.hoverboard.remote.model.RiderCommand.inputs]). A bench board with no pads wired reads no
 * rider at all, though, so this is the switch that puts the level back on the wire deliberately.
 *
 * ## Why it is gated twice, and why it dies with the link
 *
 * On a board that DOES have pads, this control is not a display preference. The folded rider level
 * is a term in the FSM's engage conjunction and its inverse is what enables the wind-down, so a
 * remote asserting rider holds a machine engaged through the moment the rider steps off.
 *
 * So: the row exists only in debug builds ([MachineStatusSection]'s `showSimulateRider` is
 * `BuildConfig.DEBUG` at the call site), because the bench flow installs a debug build and a rider's
 * build should not carry the control at all; and inside that build it takes a LONG PRESS, not a
 * tap. The label says HOLD, so the affordance is not a hidden gesture. It is also cleared whenever
 * the link drops ([com.hoverboard.remote.MainViewModel]), so it can never follow the app from the
 * padless board it was set for onto one that has pads.
 */
@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun SimulateRiderRow(on: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .combinedClickable(
                // A tap deliberately does nothing: this is the control that can hold a machine
                // engaged.
                onClick = {},
                onLongClick = { onChange(!on) },
            )
            .testTag(SIM_RIDER_TAG)
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            text = stringResource(R.string.sim_rider_label),
            style = MaterialTheme.typography.labelMedium,
            color = FgMuted,
        )
        Text(
            text = stringResource(if (on) R.string.sim_rider_on else R.string.sim_rider_off),
            style = MaterialTheme.typography.labelMedium,
            color = if (on) Primary else FgFaint,
        )
    }
}
