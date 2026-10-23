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
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalView
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.screens.AppDestination
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ConnectedScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.LayoutScreen
import com.hoverboard.remote.ui.screens.PermissionScreen
import com.hoverboard.remote.ui.screens.RideControls
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.TuneScreen
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

/**
 * Hold the display on for as long as a board is attached.
 *
 * This exists because of the disarm above, and it is the difference between that disarm meaning
 * what it says and firing on its own. **A display timing out is an `ON_STOP`**, so an armed machine
 * that nobody is touching, which is every measurement run and most of a ride, disarmed itself after
 * the phone's screen timeout and said nothing. It was measured three times on the owner's phone at
 * 300, 306 and 307 seconds from resume to stop, against a 300,000 ms `screen_off_timeout`
 * (`specs/ble-reliability.md`, final section, and `specs/bench-evidence/2026-10-23/rover-rigid/`).
 * A display that cannot time out cannot manufacture an `ON_STOP`, and `ON_STOP` then means what the
 * disarm path assumes it means: the rider left.
 *
 * It is the screen that has to be held, not the processor. `PowerManager.PARTIAL_WAKE_LOCK`
 * "ensures that the CPU is running; the screen and keyboard backlight will be allowed to go off"
 * (its own documentation), so a partial lock leaves the screen dying, the activity stopping and the
 * disarm firing, having bought nothing; the evidence says the processor was never the problem
 * anyway, since the pump kept writing at 40 per second with the phone dozing. It is also the
 * cheaper of the two to ask for: `FLAG_KEEP_SCREEN_ON` is a window flag and needs no permission,
 * while a wake lock of any level needs `WAKE_LOCK` in the manifest.
 *
 * Setting it on the composition's own view rather than on the activity's window scopes it to this
 * composable and to the view being visible: `ViewRootImpl.applyKeepScreenOnFlag` ORs
 * `FLAG_KEEP_SCREEN_ON` into the window's parameters while any attached view asks for it, so the
 * window flag is what the system ends up seeing either way.
 *
 * CONNECTED rather than armed. Arming is a tap on a screen the rider has to be able to see, and a
 * display that dies between arming attempts is a display that dies while the rider is reaching for
 * the control.
 *
 * `internal` rather than private because the hold is the point of this and is asserted directly,
 * off the view it is set on (`KeepScreenOnTest`).
 */
@Composable
internal fun KeepScreenOnWhileConnected(connected: Boolean) {
    val view = LocalView.current
    DisposableEffect(view, connected) {
        view.keepScreenOn = connected
        onDispose { view.keepScreenOn = false }
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
    KeepScreenOnWhileConnected(state.isConnected)

    val initiallyGranted = remember {
        permissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }
    }
    var permissionResolved by remember { mutableStateOf(initiallyGranted) }
    var destination by rememberSaveable { mutableStateOf(AppDestination.ROOT) }

    when {
        !permissionResolved -> {
            PermissionScreen(
                permissions = permissions,
                onGrant = { permissionResolved = true },
                onSkip = { permissionResolved = true },
            )
        }

        state.connectionState == ConnectionState.CONNECTED ->
            Connected(viewModel, state, destination) { destination = it }

        else -> {
            ConnectScreen(
                connectionState = state.connectionState,
                deviceName = state.deviceName,
                selectedBoard = state.selectedBoard,
                findingBoards = state.findingBoards,
                boardsSeen = state.boardsSeen,
                onFindBoards = viewModel::findBoards,
                onStopFindingBoards = viewModel::stopFindingBoards,
                onSelectBoard = viewModel::selectBoard,
                onDeviceNameChange = viewModel::setDeviceName,
                onConnect = viewModel::connect,
                onDisconnect = viewModel::disconnect,
            )
        }
    }
}

/**
 * The connected app: Ride, and the Tune, Setup and Board layout destinations it pushes, over one
 * [MainViewModel].
 */
@Composable
private fun Connected(
    viewModel: MainViewModel,
    state: UiState,
    destination: AppDestination,
    onDestination: (AppDestination) -> Unit,
) {
    val setupState by viewModel.setup.state.collectAsStateWithLifecycle()
    val tuneState by viewModel.tune.state.collectAsStateWithLifecycle()
    val layoutState by viewModel.layout.state.collectAsStateWithLifecycle()
    ConnectedScreen(
        destination = destination,
        onDestination = onDestination,
        onLeaveDrive = viewModel::onThrottleRelease,
        armed = state.armed,
        ride = {
            ControlScreen(
                state = state,
                onArmToggle = viewModel::onArmToggle,
                onThrottleMove = viewModel::onThrottleMove,
                onJoystickMove = viewModel::onJoystickMove,
                onThrottleRelease = viewModel::onThrottleRelease,
                onDisconnect = viewModel::disconnect,
                onSimulateRider = viewModel::setSimulateRider,
                onDriveMode = viewModel::setDriveMode,
                // Debug builds only. The control can hold a padded board engaged through a
                // step-off (ControlScreen has the argument), and a rider's build has no
                // business carrying it.
                showSimulateRider = BuildConfig.DEBUG,
            )
        },
        tune = {
            TuneScreen(
                state = tuneState,
                armed = state.armed,
                telemetry = state.telemetry,
                actions = viewModel.tune,
                // The same arm toggle and pad the Ride screen has, over the same session state and
                // the same callbacks (`specs/rider-ui.md` 3.3b): the gains and the drive bounds are
                // found by riding the machine, so the screen that sets them can ride it and stop it.
                ride = {
                    RideControls(
                        state = state,
                        onArmToggle = viewModel::onArmToggle,
                        onThrottleMove = viewModel::onThrottleMove,
                        onJoystickMove = viewModel::onJoystickMove,
                        onThrottleRelease = viewModel::onThrottleRelease,
                    )
                },
            )
        },
        setup = {
            SetupScreen(
                state = setupState,
                armed = state.armed,
                // The board the selector names, not the attached one: each board's readings
                // arrive under its own address, so the screen shows the one it is editing.
                telemetry = state.telemetryOf(setupState.target),
                actions = viewModel.setup,
            )
        },
        layout = {
            LayoutScreen(
                state = layoutState,
                armed = state.armed,
                actions = viewModel.layout,
            )
        },
    )
}
