package com.hoverboard.remote.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.MachineArm
import com.hoverboard.remote.R
import com.hoverboard.remote.RiderWaiver
import com.hoverboard.remote.SidePair
import com.hoverboard.remote.ui.screens.boardHex
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * The arm toggle: the app's only way to assert `power_request`, and the reason the motors are ever
 * live. One tap arms, one tap disarms.
 *
 * It is both the control and the indicator, deliberately. A separate button and lamp can disagree;
 * one surface that is either loud red and reads ARMED or flat grey and reads TAP TO ARM cannot. It
 * is also the ONLY arm surface: the screen used to carry a full-width ARMED banner above it, reading
 * the same `armed` flag off the same state, and a second indicator that can say nothing this one
 * cannot is duplication rather than redundancy. Glanceability is kept by tinting the telemetry panel
 * and the throttle pad red while armed, not by a second piece of text
 * ([com.hoverboard.remote.ui.screens.ControlScreen]).
 *
 * [enabled] is false only when arming is refused (the throttle is held, the link is down, or the
 * pair holds two different frames). It never gates disarming: when [armed] is true this is always
 * tappable, because a stop control that can be unavailable is not a stop control.
 *
 * [machine] is what the pair's frame and side come to (`specs/rider-ui.md` 3.7a). A frame the two
 * boards disagree about is the one statement here that REFUSES rather than warning, so it takes
 * over the control's own label; the rest read like the rider-waiver line below it.
 *
 * [noBoard] is true while the drive mode in force addresses no motor controller: no node has yet
 * answered that it is one, or the walk found only a bridge. Arming is refused then, and the label
 * says why instead of naming the throttle.
 *
 * [selfDisarmed] is the one line here that reports rather than warns: the app took the arm level
 * away itself when it stopped being on screen, which by construction happened while nobody was
 * looking at this control. It is first among the statements because it is the answer to the
 * question a rider arrives with, why is this machine not armed any more, and because a rider who
 * cannot tell it from a fault or a dropped link will go looking at the machine for a cause that
 * is not there (`specs/ble-reliability.md`, final section).
 */
@Composable
fun ArmToggle(
    armed: Boolean,
    enabled: Boolean,
    onToggle: () -> Unit,
    modifier: Modifier = Modifier,
    riderWaiver: RiderWaiver = RiderWaiver.NONE,
    machine: MachineArm = MachineArm.None,
    selfDisarmed: Boolean = false,
    noBoard: Boolean = false,
) {
    val live = armed || enabled
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(20.dp))
            .background(if (armed) AccentRed else PanelSurface)
            .clickable(enabled = live) { onToggle() }
            .padding(vertical = 22.dp)
            .testTag(ARM_TAG),
        contentAlignment = Alignment.Center,
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = when {
                    armed -> stringResource(R.string.arm_disarm_action)
                    machine.refuses -> stringResource(R.string.arm_blocked)
                    noBoard -> stringResource(R.string.arm_no_board)
                    !enabled -> stringResource(R.string.arm_unavailable)
                    else -> stringResource(R.string.arm_action)
                },
                style = MaterialTheme.typography.titleLarge,
                fontWeight = FontWeight.Bold,
                textAlign = TextAlign.Center,
                color = armColor(armed = armed, enabled = enabled),
            )
            if (selfDisarmed) Statement(stringResource(R.string.arm_self_disarmed), armed)
            // `specs/control.md` (i): with the rider requirement waived, arming a balancing board is
            // the engage act, and the operator holds the frame upright when arming. The control that
            // does the arming is where that is said; when the app cannot tell what the board runs,
            // it says the requirement may be waived rather than staying silent or claiming it is.
            val waiverLine = when (riderWaiver) {
                RiderWaiver.NONE -> null
                RiderWaiver.WAIVED -> R.string.arm_rider_waived
                RiderWaiver.POSSIBLY -> R.string.arm_rider_maybe_waived
            }
            if (waiverLine != null) Statement(stringResource(waiverLine), armed)
            machineLine(machine)?.let { Statement(it, armed, refusal = machine.refuses) }
        }
    }
}

/** One line of what the arm control knows, under its label. */
@Composable
private fun Statement(text: String, armed: Boolean, refusal: Boolean = false) {
    Text(
        text = text,
        style = MaterialTheme.typography.bodySmall,
        textAlign = TextAlign.Center,
        color = when {
            armed -> TextPrimary
            refusal -> AccentRed
            else -> AccentYellow
        },
        modifier = Modifier.padding(top = 4.dp, start = 12.dp, end = 12.dp),
    )
}

/**
 * What the machine's frame and side say here (`specs/rider-ui.md` 3.7a). The frame difference names
 * every board's value, because "they differ" without them leaves the operator to go and look.
 */
@Composable
private fun machineLine(machine: MachineArm): String? = when (machine) {
    MachineArm.None -> null
    is MachineArm.FramesDiffer -> stringResource(
        R.string.arm_frames_differ,
        machine.rigid.toSortedMap().map { (address, rigid) ->
            "${boardHex(address)} ${stringResource(frameName(rigid))}"
        }.joinToString(", "),
    )
    is MachineArm.FrameUnread ->
        stringResource(if (machine.peerFound) R.string.arm_frame_unread else R.string.arm_frame_no_peer)
    is MachineArm.SideNotOneRight -> stringResource(
        when (machine.pair) {
            SidePair.BothRight -> R.string.arm_side_both_right
            SidePair.NoneRight -> R.string.arm_side_none_right
            else -> R.string.arm_side_unknown
        },
    )
}

private fun frameName(rigid: Boolean): Int =
    if (rigid) R.string.setup_choice_frame_rigid else R.string.setup_choice_frame_pivoting

private fun armColor(armed: Boolean, enabled: Boolean): Color = when {
    armed -> TextPrimary
    !enabled -> ZeroLine
    else -> AccentRed
}

const val ARM_TAG = "arm_toggle"
