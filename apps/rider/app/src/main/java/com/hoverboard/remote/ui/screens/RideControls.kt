package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.ui.components.ArmToggle
import com.hoverboard.remote.ui.components.JoystickPad
import com.hoverboard.remote.ui.components.ThrottlePad

/**
 * The two controls that move a machine and stop it: the arm toggle and the pad under the thumb.
 *
 * They are one composable because they belong together wherever they appear. The Ride screen has
 * carried both since it existed, and the Tune screen carries them now too (`specs/rider-ui.md`
 * 3.3b), so that the gains and the drive bounds can be found by riding the machine rather than by
 * walking back to another screen between every change. A screen that can move a machine and not
 * stop it would be the wrong half to copy, so neither is offered without the other.
 *
 * Nothing here is per screen: the state, the arming and the demand are the one session's, so a pad
 * on either screen drives the same board through the same callbacks.
 */
@Composable
internal fun RideControls(
    state: UiState,
    onArmToggle: () -> Unit,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onJoystickMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(modifier = modifier.fillMaxWidth()) {
        ArmToggle(
            armed = state.armed,
            enabled = state.canArm,
            onToggle = onArmToggle,
            riderWaiver = state.riderWaiver,
            machine = state.machineArm,
            selfDisarmed = state.selfDisarmed,
        )
        Spacer(modifier = Modifier.height(12.dp))
        DriveControl(
            state = state,
            onThrottleMove = onThrottleMove,
            onJoystickMove = onJoystickMove,
            onThrottleRelease = onThrottleRelease,
        )
    }
}

/**
 * The control the mode puts under the thumb: the two-axis stick in the modes that steer
 * ([DriveMode.hasTurnAxis]) and the throttle pad in the ones that send one value per board. One or
 * the other, never both.
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
    val size = Modifier.fillMaxWidth().heightIn(min = DRIVE_CONTROL_MIN_HEIGHT)
    if (state.driveMode.hasTurnAxis) {
        JoystickPad(
            speed = state.throttleSpeed,
            steer = state.steer,
            engaged = state.engaged,
            enabled = state.isConnected && state.armed,
            armed = state.armed,
            onMove = onJoystickMove,
            onRelease = onThrottleRelease,
            modifier = size,
        )
    } else {
        ThrottlePad(
            speed = state.throttleSpeed,
            engaged = state.engaged,
            enabled = state.isConnected && state.armed,
            armed = state.armed,
            onMove = onThrottleMove,
            onRelease = onThrottleRelease,
            modifier = size,
        )
    }
}

/** The drive control's guaranteed height: its y range is the demand's resolution. */
private val DRIVE_CONTROL_MIN_HEIGHT = 260.dp
