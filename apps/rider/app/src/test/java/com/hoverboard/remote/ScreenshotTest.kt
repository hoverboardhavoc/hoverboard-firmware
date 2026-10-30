package com.hoverboard.remote

import android.app.Application
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onRoot
import androidx.compose.ui.test.performClick
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.github.takahirom.roborazzi.captureRoboImage
import com.github.takahirom.roborazzi.captureScreenRoboImage
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SpeedFacts
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.model.TripBaseline
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.ui.screens.AppDestination
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.ConnectedScreen
import com.hoverboard.remote.ui.screens.ControlScreen
import com.hoverboard.remote.ui.screens.LayoutScreen
import com.hoverboard.remote.ui.screens.MachineStatusSection
import com.hoverboard.remote.ui.screens.RideTopBarStatus
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.TuneScreen
import com.hoverboard.remote.ui.screens.lampTag
import com.hoverboard.remote.ui.theme.Bg
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import kotlin.math.PI
import kotlin.math.roundToInt
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode

/**
 * Roborazzi baseline screenshots of the Connect screen, the 8a Ride screen and the Tune, Setup and
 * board layout screens in the app theme (`specs/rider-ui.md` 3.1d and 3.10; house stack §Testing,
 * SPEC §12.2 layer 1). Composables are captured directly (no Activity), so no view hierarchy or
 * Koin graph is needed.
 *
 * Captures go through a compose test rule, not the standalone `captureRoboImage { }` overload.
 * The standalone overload syncs through Espresso, whose Robolectric idle loop never sees the
 * main looper idle while the busy spinner's infinite transition (ConnectScreen's
 * StatusIndicator) keeps scheduling frames, so the scanning and attaching cases spun until the
 * test JVM ran out of heap. Under the rule those cases pause the test clock and advance it a
 * fixed [SPINNER_PHASE_MS], so the spinner is captured at the same visible phase every run.
 */
@RunWith(AndroidJUnit4::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ScreenshotTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    @Test
    fun connectScreen_disconnected() {
        capture("connect_disconnected") {
            ConnectScreen(
                connectionState = ConnectionState.DISCONNECTED,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                selectedBoard = null,
                findingBoards = false,
                boardsSeen = emptyList(),
                onFindBoards = {},
                onStopFindingBoards = {},
                onSelectBoard = {},
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_scanning() {
        capture("connect_scanning", advanceMs = SPINNER_PHASE_MS) {
            ConnectScreen(
                connectionState = ConnectionState.SCANNING,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                selectedBoard = null,
                findingBoards = false,
                boardsSeen = emptyList(),
                onFindBoards = {},
                onStopFindingBoards = {},
                onSelectBoard = {},
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_attaching() {
        capture("connect_attaching", advanceMs = SPINNER_PHASE_MS) {
            ConnectScreen(
                connectionState = ConnectionState.ATTACHING,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                selectedBoard = null,
                findingBoards = false,
                boardsSeen = emptyList(),
                onFindBoards = {},
                onStopFindingBoards = {},
                onSelectBoard = {},
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    @Test
    fun connectScreen_attachFailed() {
        capture("connect_attach_failed") {
            ConnectScreen(
                connectionState = ConnectionState.ATTACH_FAILED,
                deviceName = LinkConfig.DEFAULT_DEVICE_NAME,
                selectedBoard = null,
                findingBoards = false,
                boardsSeen = emptyList(),
                onFindBoards = {},
                onStopFindingBoards = {},
                onSelectBoard = {},
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    // --- Ride, 8a (`specs/rider-ui.md` 3.1 and 3.1d) -------------------------------------------
    //
    // Every Ride capture is taken at the class qualifier and again at the ASUS ROG's 1080 x 1920 px
    // (ASUS), named with the suffix `_asus`.

    /** Two boards, sides left and right, BOUND, armed, throttle forward, 7.4 km/h, a trip and a total. */
    @Test
    fun ride_riding() = capture("ride_riding") { Ride(riding()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_riding_asus() = capture("ride_riding_asus") { Ride(riding()) }

    /** As riding, with the pack under 15 percent: the ring in caution and the BATTERY LOW chip. */
    @Test
    fun ride_lowBattery() = capture("ride_low_battery") { Ride(lowBattery()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_lowBattery_asus() = capture("ride_low_battery_asus") { Ride(lowBattery()) }

    /** Disarmed, HALL on the left board and IMU on the right: the grid in the pad's bounds, the ARM label faint. */
    @Test
    fun ride_fault() = capture("ride_fault") { Ride(fault()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_fault_asus() = capture("ride_fault_asus") { Ride(fault()) }

    /** The fault with HALL's sheet open: the sheet is its own window, so the whole screen is captured. */
    @Test
    fun ride_faultSheet() = faultSheet("ride_fault_sheet")

    @Test
    @Config(qualifiers = ASUS)
    fun ride_faultSheet_asus() = faultSheet("ride_fault_sheet_asus")

    private fun faultSheet(name: String) {
        setRide(fault())
        compose.onNodeWithTag(lampTag(FaultLamp.HALL)).performClick()
        compose.waitForIdle()
        captureScreenRoboImage("src/test/screenshots/$name.png")
    }

    /** One second into the lamp test, on the test clock: the grid in the pad's bounds. */
    @Test
    fun ride_lampTest() = capture("ride_lamp_test", advanceMs = LAMP_TEST_PHASE_MS) {
        Ride(sided(armed = false).copy(lampTest = true))
    }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_lampTest_asus() = capture("ride_lamp_test_asus", advanceMs = LAMP_TEST_PHASE_MS) {
        Ride(sided(armed = false).copy(lampTest = true))
    }

    /**
     * One board silent, the other with no wheel diameter, no pack voltage, its current calibration
     * unread and no IMU fitted: every figure that has no value says so, and the statements say why.
     */
    @Test
    fun ride_unknown() = capture("ride_unknown") { Ride(unknown()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_unknown_asus() = capture("ride_unknown_asus") { Ride(unknown()) }

    /** Both boards reporting, no cell count and no pole pairs: the voltage alone and two "Set it ›" lines. */
    @Test
    fun ride_unset() = capture("ride_unset") { Ride(unset()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_unset_asus() = capture("ride_unset_asus") { Ride(unset()) }

    /** Riding backwards at 3.2 km/h: the magnitude and "km/h R". */
    @Test
    fun ride_reverse() = capture("ride_reverse") { Ride(reverse()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_reverse_asus() = capture("ride_reverse_asus") { Ride(reverse()) }

    /** Riding, with the right wheel judged not turning. */
    @Test
    fun ride_notTurning() = capture("ride_not_turning") { Ride(notTurning()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_notTurning_asus() = capture("ride_not_turning_asus") { Ride(notTurning()) }

    /** Connected to `offroad-master`, the walk having found only the bridge: the card. */
    @Test
    fun ride_boardsOff() = capture("ride_boards_off") { Ride(boardsOff()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_boardsOff_asus() = capture("ride_boards_off_asus") { Ride(boardsOff()) }

    /** DIFFERENTIAL, armed, the joystick deflected on both axes. */
    @Test
    fun ride_joystickArmed() = capture("ride_joystick_armed") { Ride(joystick()) }

    @Test
    @Config(qualifiers = ASUS)
    fun ride_joystickArmed_asus() = capture("ride_joystick_armed_asus") { Ride(joystick()) }

    private fun lowBattery(): UiState {
        val riding = riding()
        val low = riding.telemetryByBoard.mapValues { (_, t) -> t.withBattery(LOW_CV) }
        return riding.copy(telemetryByBoard = low).also { check(it.batteryIsLow) }
    }

    private fun unknown(): UiState = sided(armed = false).copy(
        telemetryByBoard = mapOf(0x01 to board(battery = 0, pitch = 120, roll = -40)),
        countsPerAmp = mapOf(0x02 to DRIVING_CAL),
        imuModel = mapOf(0x01 to 0, 0x02 to IMU_FITTED),
        speedFacts = listOf(
            SpeedFacts(0x01, 0, polePairs = POLE_PAIRS, diameterMm = 0, side = Fields.ControlSide.LEFT),
        ),
    )

    private fun unset(): UiState = sided(armed = false).copy(
        packCells = mapOf(0x01 to 0, 0x02 to 0),
        speedFacts = listOf(
            SpeedFacts(0x01, 0, polePairs = 0, diameterMm = DIAMETER_MM, side = Fields.ControlSide.LEFT),
            SpeedFacts(0x02, 0, polePairs = 0, diameterMm = DIAMETER_MM, side = Fields.ControlSide.RIGHT),
        ),
    )

    private fun reverse(): UiState {
        val rate = (REVERSE_KMH / KMH_PER_RATE).roundToInt()
        return riding().copy(
            throttleSpeed = -Throttle.MAX_SPEED / 4,
            speedFacts = listOf(
                SpeedFacts(0x01, -rate, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.LEFT),
                SpeedFacts(0x02, rate, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.RIGHT),
            ),
        )
    }

    private fun notTurning(): UiState = riding().copy(notTurning = setOf(0x02))

    private fun boardsOff(): UiState = UiState(
        connectionState = ConnectionState.CONNECTED,
        nodes = listOf(NodeInfo(0x01, Fields.NodeClass.BRIDGE, "offroad-master")),
        attachedNode = 0x01,
        deviceName = "offroad-master",
        boardsOff = true,
    )

    private fun joystick(): UiState {
        val targets = DriveTargets(DriveMode.DIFFERENTIAL, listOf(0x01, 0x02), right = 0x02)
        return riding().copy(
            drive = DriveInForce(targets, DriveMode.DIFFERENTIAL, refusal = null),
            driving = targets,
            throttleSpeed = Throttle.MAX_SPEED * 2 / 5,
            steer = Throttle.MAX_SPEED / 3,
        )
    }

    // --- the other screens, in the theme -------------------------------------------------------

    @Test
    fun tune_board() {
        val keys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }
        val bounds = SetupFields.DRIVE_BOUNDS.associate { it.key to (it.def.default as Value.I16).v }
        val state = TuneState(
            target = 0x01,
            motors = listOf(0x01, 0x02).map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME) },
            boards = mapOf(
                0x01 to BoardGains(
                    staged = keys.zip(Gains.DEFAULT).toMap(),
                    flash = keys.zip(Gains.DEFAULT).toMap(),
                    stale = false,
                    bounds = bounds,
                ),
            ),
        )
        capture("tune_board") {
            Pushed(AppDestination.TUNE) {
                TuneScreen(state = state, armed = false, actions = NoTune, ride = {})
            }
        }
    }

    /** Setup for a motor controller, with the machine status section: one board's fault line, one bit-6 MODE line. */
    @Test
    fun setup_motorController() {
        val values = SetupFields.ALL.associate { it.key to it.def.default } +
            (SetupFields.NODE_CLASS.key to Value.U8(Fields.NodeClass.MOTOR_CONTROLLER))
        val setup = SetupState(
            addresses = listOf(0x01, 0x02),
            attached = 0x01,
            target = 0x01,
            boards = mapOf(0x01 to BoardSetup(values = values)),
        )
        val status = sided(armed = false)
            .reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL))
            .reporting(0x02, board(fault = CyclicState.EV_MODE_FAULT, flags = CyclicState.FLAG_RIDER))
        capture("setup_motor_controller") {
            Pushed(AppDestination.SETUP) {
                SetupScreen(
                    state = setup,
                    armed = false,
                    telemetry = null,
                    odometers = emptyList(),
                    actions = Recorder(),
                    status = { MachineStatusSection(status, showSimulateRider = false, onSimulateRider = {}) },
                )
            }
        }
    }

    @Test
    fun layout_editor() {
        val values = Layout.SLOTS.associate { it.key to it.def.default } + (Fields.LINK_SET.key() to Value.U8(0))
        val state = LayoutState(
            board = 0x01,
            chip = ChipTag.F103C8,
            values = values,
            linkSet = 0,
            staged = checkNotNull(Layout.fieldsFrom(values)),
        )
        capture("layout_editor") {
            Pushed(AppDestination.LAYOUT) { LayoutScreen(state = state, armed = false, actions = NoLayout) }
        }
    }

    // --- fixtures ------------------------------------------------------------------------------

    /** The Ride screen under the Ride top bar, so the link dots and the chip are in the image. */
    @Composable
    private fun Ride(state: UiState) {
        ConnectedScreen(
            destination = AppDestination.RIDE,
            onDestination = {},
            onLeaveDrive = {},
            armed = state.armed,
            ride = {
                ControlScreen(
                    state = state,
                    onArmToggle = {},
                    onThrottleMove = { _, _ -> },
                    onJoystickMove = { _, _, _, _ -> },
                    onThrottleRelease = {},
                )
            },
            tune = {},
            setup = {},
            layout = {},
            rideStatus = { RideTopBarStatus(state) },
        )
    }

    /** [content] as the pushed [destination], under its top bar. */
    @Composable
    private fun Pushed(destination: AppDestination, content: @Composable () -> Unit) {
        ConnectedScreen(
            destination = destination,
            onDestination = {},
            onLeaveDrive = {},
            armed = false,
            ride = {},
            tune = content,
            setup = content,
            layout = content,
        )
    }

    private fun setRide(state: UiState) = compose.setContent {
        HoverboardRemoteTheme {
            Box(modifier = Modifier.fillMaxSize().background(Bg)) { Ride(state) }
        }
    }

    /** A board's frame in [mode]: [fault] its word, [battery] centivolts, its attitude and window. */
    private fun board(
        fault: Int = 0,
        battery: Int = RIDING_CV,
        flags: Int = 0,
        mode: Int = MODE_OFF,
        pitch: Int = 0,
        roll: Int = 0,
        obs: CyclicObs = QUIET_OBS,
    ) = TelemetryUi().merge(
        CyclicState(
            pitch = pitch,
            roll = roll,
            wheelSpeed = 0,
            battery = battery,
            mode = mode,
            fault = fault,
            flags = flags,
            peerDemand = 0,
            wheelPos = 0,
            wheelRate = 0,
            obs = obs,
        ),
    )

    private fun TelemetryUi.withBattery(cv: Int) = merge(checkNotNull(cyclic).copy(battery = cv))

    /** This state with [address] having last reported [telemetry]. */
    private fun UiState.reporting(address: Int, telemetry: TelemetryUi) =
        copy(telemetryByBoard = telemetryByBoard + (address to telemetry))

    /**
     * The pair at 0x01 (the left wheel) and 0x02 (the right), BOUND, both reporting at rest, every
     * ride fact read: a ten-cell Li-ion pack, a 15 A limit, an IMU on each.
     */
    private fun sided(armed: Boolean): UiState {
        val boards = listOf(0x01, 0x02)
        val targets = DriveTargets(DriveMode.BOUND, boards)
        return UiState(
            connectionState = ConnectionState.CONNECTED,
            armed = armed,
            nodes = boards.map { NodeInfo(it, Fields.NodeClass.MOTOR_CONTROLLER, BOARD_NAME) },
            attachedNode = 0x01,
            selected = 0x01,
            drive = DriveInForce(targets, stored = DriveMode.BOUND, refusal = null),
            driving = targets,
            machine = Machine(
                FramePair.Agree(rigid = false),
                SidePair.OneRight(0x02),
                ModePair.Agree(DriveMode.BOUND.byte),
            ),
            telemetryByBoard = boards.associateWith { board() },
            countsPerAmp = boards.associateWith { DRIVING_CAL },
            packCells = boards.associateWith { PACK_CELLS },
            packChemistry = boards.associateWith { Fields.BatteryChemistry.LI_ION },
            sides = mapOf(0x01 to Fields.ControlSide.LEFT, 0x02 to Fields.ControlSide.RIGHT),
            currentLimit = boards.associateWith { LIMIT_MA },
            imuModel = boards.associateWith { IMU_FITTED },
            batteryFloor = boards.associateWith { 0 },
            wheelGeometry = boards.associateWith { WheelGeometry(POLE_PAIRS, DIAMETER_MM) },
            speedFacts = listOf(
                SpeedFacts(0x01, 0, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.LEFT),
                SpeedFacts(0x02, 0, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.RIGHT),
            ),
        )
    }


    /** Riding: armed, throttle forward, 7.4 km/h, 6.8 A and 5.1 A, a trip and a total. */
    private fun riding(): UiState {
        val wheel = WheelGeometry(POLE_PAIRS, DIAMETER_MM)
        val rate = (RIDING_KMH / KMH_PER_RATE).roundToInt()
        fun driving(amps: Float, pitch: Int, roll: Int) = board(
            mode = MODE_RUN,
            pitch = pitch,
            roll = roll,
            obs = DRIVING_OBS.copy(phaseMean = (amps * DRIVING_CAL).roundToInt(), odometer = ODOMETER_EDGES),
        )
        val state = sided(armed = true).copy(
            throttleSpeed = Throttle.MAX_SPEED * 2 / 5,
            engaged = true,
            telemetryByBoard = mapOf(
                0x01 to driving(LEFT_AMPS, pitch = 230, roll = -80),
                0x02 to driving(RIGHT_AMPS, pitch = 210, roll = -60),
            ),
            wheelGeometry = mapOf(0x01 to wheel, 0x02 to wheel),
            speedFacts = listOf(
                SpeedFacts(0x01, rate, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.LEFT),
                SpeedFacts(0x02, -rate, POLE_PAIRS, DIAMETER_MM, Fields.ControlSide.RIGHT),
            ),
        )
        val total = (state.machineTravel as MachineTravel.Distance).millimetres
        return state.copy(trip = TripBaseline(pick = "AA:BB:CC:DD:EE:FF", millimetres = total - TRIP_MM))
            .also { check((it.ring.second as BatteryReading.Charge).percent in 70..80) }
    }

    /** Disarmed, HALL on the left board and IMU on the right. */
    private fun fault(): UiState = sided(armed = false)
        .reporting(0x01, board(fault = CyclicState.EV_MOTOR_HALL, pitch = 20, roll = -10))
        .reporting(0x02, board(fault = CyclicState.EV_IMU_LOSS))

    /** No-op Tune actions: a capture taps nothing. */
    private object NoTune : TuneActions {
        override fun onShown() = Unit
        override fun onHidden() = Unit
        override fun selectTarget(address: Int) = Unit
        override fun refresh() = Unit
        override fun slide(key: Key, value: Int) = Unit
        override fun slideEnd(key: Key, value: Int) = Unit
        override fun setBound(key: Key, value: Int) = Unit
        override fun save() = Unit
        override fun revert() = Unit
        override fun dismissNotice() = Unit
    }

    /** No-op layout actions. */
    private object NoLayout : LayoutActions {
        override fun onShown() = Unit
        override fun onHidden() = Unit
        override fun refresh() = Unit
        override fun stage(slot: LayoutSlot, raw: Int) = Unit
        override fun revert(slot: LayoutSlot) = Unit
        override fun revertAll() = Unit
        override fun apply() = Unit
        override fun dismissNotice() = Unit
    }

    /**
     * With [advanceMs] set, the test clock is paused (auto-advance off, which also lets the
     * infinite spinner run) and stepped exactly that far before the capture.
     */
    private fun capture(name: String, advanceMs: Long? = null, content: @Composable () -> Unit) {
        if (advanceMs != null) compose.mainClock.autoAdvance = false
        compose.setContent {
            HoverboardRemoteTheme {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Bg),
                ) {
                    content()
                }
            }
        }
        if (advanceMs != null) compose.mainClock.advanceTimeBy(advanceMs)
        compose.onRoot().captureRoboImage("src/test/screenshots/$name.png")
    }

    private companion object {
        /** Spinner phase for the busy screenshots: far enough in to draw a visible arc. */
        const val SPINNER_PHASE_MS = 500L

        /** One second into the two-second lamp test. */
        const val LAMP_TEST_PHASE_MS = 1_000L

        const val BOARD_NAME = "Hoverboard"
        const val MODE_OFF = 0
        const val MODE_RUN = 3

        /** 39.00 V on ten Li-ion cells is 3.90 V a cell: 72 percent. */
        const val RIDING_CV = 3_900

        /** 34.70 V on ten Li-ion cells: 12 percent, under the 15 percent low edge. */
        const val LOW_CV = 3_470

        const val LIMIT_MA = 15_000
        const val LEFT_AMPS = 6.8f
        const val RIGHT_AMPS = 5.1f
        const val IMU_FITTED = 1
        const val POLE_PAIRS = 15
        const val DIAMETER_MM = 165
        const val RIDING_KMH = 7.4
        const val REVERSE_KMH = 3.2

        /** The ASUS ROG's 1080 x 1920 px at its density of 3.0. */
        const val ASUS = "w360dp-h640dp-xxhdpi"

        /** km/h per unit of `wheel_rate` on this wheel (`specs/rider-ui.md` 3.1a). */
        const val KMH_PER_RATE = 50.0 / 4096 / (6 * POLE_PAIRS) * PI * DIAMETER_MM * 0.0036

        /** About 413 km of lifetime travel on each wheel, and a 3.1 km trip. */
        const val ODOMETER_EDGES = 71_700_000L
        const val TRIP_MM = 3_100_000L
    }
}
