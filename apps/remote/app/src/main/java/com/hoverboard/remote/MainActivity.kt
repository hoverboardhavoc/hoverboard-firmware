package com.hoverboard.remote

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.SystemBarStyle
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.MutableState
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
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.screens.AppDestination
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ConnectedScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.LayoutScreen
import com.hoverboard.remote.ui.screens.MachineStatusSection
import com.hoverboard.remote.ui.screens.RideTopBarStatus

import com.hoverboard.remote.ui.screens.PermissionScreen
import com.hoverboard.remote.ui.screens.RideControls
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.nodeLabel
import com.hoverboard.remote.ui.theme.Bg
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.koin.androidx.compose.koinViewModel

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Light icons on black whatever the phone's own theme: the default style follows the
        // system and would draw dark icons on the black bars in light mode (`specs/remote-ui.md` 3.10).
        enableEdgeToEdge(
            statusBarStyle = SystemBarStyle.dark(Color.TRANSPARENT),
            navigationBarStyle = SystemBarStyle.dark(Color.TRANSPARENT),
        )
        setContent {
            HoverboardRemoteTheme {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Bg)
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
 * 300, 306 and 307 seconds from resume to stop, against the system's 300,000 ms screen-off
 * timeout (`specs/ble-reliability.md`, final section, and
 * `specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`, the run log).
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

/**
 * Where the connected app is, held across a configuration change and PUT BACK TO RIDE whenever the
 * link is not up.
 *
 * A link drop ends the session: the view model drops the arm level for that reason, and the
 * destination is the same kind of state. Without this, a drop while the owner was on Setup or the
 * layout editor reconnected straight back into an editor, with a live machine one screen away and
 * nothing saying the session had restarted. Ride is the root and the place the machine is live, so
 * that is where a new session starts.
 *
 * It also covers the app coming back from process death: the saved destination is restored first,
 * and the reset then takes it to Ride unless a board is already attached.
 *
 * `internal` and hoisted out of the root composable because the reset is the point of it and is
 * asserted directly ([com.hoverboard.remote.DestinationResetTest]), the way the screen hold above
 * is.
 */
@Composable
internal fun rememberDestination(connected: Boolean): MutableState<AppDestination> {
    val destination = rememberSaveable { mutableStateOf(AppDestination.ROOT) }
    LaunchedEffect(connected) {
        if (!connected) destination.value = AppDestination.ROOT
    }
    return destination
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
    var destination by rememberDestination(state.isConnected)

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
    // The row a missing-setting statement opened Setup at (`specs/remote-ui.md` 3.1).
    var setupRow by remember { mutableStateOf<Key?>(null) }
    val openSetup = { board: Int, row: Key ->
        viewModel.onThrottleRelease()
        viewModel.setup.selectTarget(board)
        setupRow = row
        onDestination(AppDestination.SETUP)
    }
    val tuneState by viewModel.tune.state.collectAsStateWithLifecycle()
    val layoutState by viewModel.layout.state.collectAsStateWithLifecycle()
    ConnectedScreen(
        destination = destination,
        onDestination = {
            setupRow = null
            onDestination(it)
        },
        onLeaveDrive = viewModel::onThrottleRelease,
        armed = state.armed,
        onDisconnect = viewModel::disconnect,
        rideStatus = { RideTopBarStatus(state) },
        ride = {
            ControlScreen(
                state = state,
                onArmToggle = viewModel::onArmToggle,
                onThrottleMove = viewModel::onThrottleMove,
                onJoystickMove = viewModel::onJoystickMove,
                onThrottleRelease = viewModel::onThrottleRelease,
                // Debug builds only (`specs/remote-ui.md` 3.1): the per-source frame count and
                // longest gap, a bench instrument with nothing to tell a rider.
                showFrameStats = BuildConfig.DEBUG,
                onOpenSetup = openSetup,
            )
        },
        tune = {
            TuneScreen(
                state = tuneState,
                armed = state.armed,
                actions = viewModel.tune,
                // The same arm toggle and pad the Ride screen has, over the same session state and
                // the same callbacks (`specs/remote-ui.md` 3.3b): the gains and the drive bounds are
                // found by riding the machine, so the screen that sets them can ride it and stop it.
                ride = {
                    RideControls(
                        state = state,
                        onArmToggle = viewModel::onArmToggle,
                        onThrottleMove = viewModel::onThrottleMove,
                        onJoystickMove = viewModel::onJoystickMove,
                        onThrottleRelease = viewModel::onThrottleRelease,
                        onOpenSetup = openSetup,
                    )
                },
            )
        },
        setup = { Setup(state, setupState, viewModel.setup, viewModel::setSimulateRider, setupRow) },
        layout = {
            LayoutScreen(
                state = layoutState,
                armed = state.armed,
                actions = viewModel.layout,
            )
        },
    )
}

/**
 * Setup for the node its selector names, with the machine status section at its top
 * (`specs/remote-ui.md` 3.1c), which covers every board whatever the selector says.
 */
@Composable
private fun Setup(
    state: UiState,
    setupState: SetupState,
    actions: SetupActions,
    onSimulateRider: (Boolean) -> Unit,
    scrollTo: Key?,
) {
    SetupScreen(
        state = setupState,
        armed = state.armed,
        // The node the selector names, not the attached one: each board's readings arrive
        // under its own address, so the screen shows the one it is editing.
        telemetry = setupState.board?.let { state.telemetryOf(it) },
        // Every motor controller, unlike the line above: the Travel section's machine
        // figure is the mean of the wheels, so it needs them all whichever node is edited.
        odometers = state.motorControllers.map { nodeLabel(it) to state.odometerOf(it.address) },
        actions = actions,
        scrollTo = scrollTo,
        status = {
            MachineStatusSection(
                state = state,
                // Debug builds only. The control can hold a padded board engaged through a
                // step-off (MachineStatusSection has the argument), and a rider's build has
                // no business carrying it.
                showSimulateRider = BuildConfig.DEBUG,
                onSimulateRider = onSimulateRider,
            )
        },
    )
}
