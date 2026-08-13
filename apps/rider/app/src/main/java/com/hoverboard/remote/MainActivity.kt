package com.hoverboard.remote

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.PermissionScreen
import com.hoverboard.remote.ui.theme.DarkBackground
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.koin.androidx.compose.koinViewModel

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            HoverboardRemoteTheme {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(DarkBackground)
                        .systemBarsPadding(),
                ) {
                    HoverboardRoot()
                }
            }
        }
    }
}

/**
 * Disarm the board whenever this app stops being the thing on screen.
 *
 * `ON_STOP` rather than `ON_PAUSE`: a permission dialog or the notification shade pauses the
 * activity without the rider letting go, and disarming under a transient overlay would be a stall
 * mid-ride. `ON_STOP` is the app genuinely losing the foreground.
 *
 * This is a real path, not a belt-and-braces one. Android does not promise a pointer-cancel when it
 * takes the window away, so the arm level latches until something clears it, and the transport keeps streaming from the
 * background: without this, a home-button press could leave a board armed with nobody looking at
 * it. Under a latching toggle this is the main protection against exactly that.
 */
@Composable
private fun DisarmWhenBackgrounded(viewModel: MainViewModel) {
    val owner = LocalLifecycleOwner.current
    DisposableEffect(owner, viewModel) {
        val observer = LifecycleEventObserver { _, event ->
            if (event == Lifecycle.Event.ON_STOP) viewModel.onAppBackgrounded()
        }
        owner.lifecycle.addObserver(observer)
        onDispose { owner.lifecycle.removeObserver(observer) }
    }
}

/** Runtime BLE permissions for this device's API level (SPEC §8.1, house stack §Permissions). */
private fun blePermissions(): Array<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    } else {
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }

@Composable
private fun HoverboardRoot() {
    val viewModel: MainViewModel = koinViewModel()
    val state by viewModel.uiState.collectAsStateWithLifecycle()
    val context = LocalContext.current
    val permissions = remember { blePermissions() }

    DisarmWhenBackgrounded(viewModel)

    val initiallyGranted = remember {
        permissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }
    }
    var permissionResolved by remember { mutableStateOf(initiallyGranted) }

    when {
        !permissionResolved -> {
            PermissionScreen(
                permissions = permissions,
                onGrant = { permissionResolved = true },
                onSkip = { permissionResolved = true },
            )
        }

        state.connectionState == ConnectionState.CONNECTED -> {
            ControlScreen(
                state = state,
                onArmToggle = viewModel::onArmToggle,
                onThrottleMove = viewModel::onThrottleMove,
                onThrottleRelease = viewModel::onThrottleRelease,
                onDisconnect = viewModel::disconnect,
            )
        }

        else -> {
            ConnectScreen(
                connectionState = state.connectionState,
                deviceName = state.deviceName,
                onDeviceNameChange = viewModel::setDeviceName,
                onConnect = viewModel::connect,
                onDisconnect = viewModel::disconnect,
            )
        }
    }
}
