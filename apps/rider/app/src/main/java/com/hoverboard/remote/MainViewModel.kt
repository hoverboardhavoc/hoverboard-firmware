package com.hoverboard.remote

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.hoverboard.remote.ble.BoardScanner
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.ble.ScanCandidate
import com.hoverboard.remote.ble.ScanList
import com.hoverboard.remote.ble.SelectedBoard
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.DriveTargets
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.model.batteryReading
import com.hoverboard.remote.model.currentReading
import com.hoverboard.remote.model.rigidLeader
import com.hoverboard.remote.model.odometerReading
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/**
 * UI state surfaced to Compose.
 *
 * @param connectionState BLE link state.
 * @param armed whether the arm control is currently held, i.e. whether the app is asserting
 *   `power_request` and the boards' motors are enabled.
 * @param throttleSpeed current commanded demand (-MAX..MAX), 0 whenever not armed. In DIFFERENTIAL
 *   this is the mix's `v`, the stick's forward axis, and not what either wheel is told.
 * @param steer current commanded turn (-MAX..MAX), 0 whenever not armed and 0 in the two modes
 *   with nowhere to spend it. DIFFERENTIAL spends it on the difference between two per-wheel
 *   demands; RIGID puts it on the wire for the leader to split.
 * @param engaged whether the throttle pad is currently held.
 * @param disconnecting whether [MainViewModel.disconnect] is mid-teardown: disarmed, but still
 *   CONNECTED while the disarming command reaches the boards.
 * @param simulateRider whether the bench affordance that asserts the `INPUTS` rider bit is on. Off
 *   by default; see [RiderCommand.inputs].
 * @param selfDisarmed whether the app is the thing that took the arm level away, rather than a
 *   fault or a lost link ([MainViewModel.onAppBackgrounded]). Set when the app disarms a machine
 *   that was armed, and cleared by the next arm, so the arm control can say it happened.
 * @param riderWaiver whether a board the arm control arms runs, or may run, with
 *   `CONTROL_RIDER_REQUIRED` at 0 ([SetupState.riderWaiver]), which the arm control states.
 * @param machine the machine's frame, side and drive mode as this session has read them on every
 *   motor controller ([SetupState.machine]), which the arm control acts on and the strip badges
 *   (`specs/rider-ui.md` 3.7a).
 * @param nodes every node the walk found, in address order, with its class and name as far as they
 *   are read ([SetupState.nodes], `specs/rider-ui.md` section 2).
 * @param attachedNode the node on the phone's own link, or null while none is attached.
 * @param selected the motor controller the Tune/Setup selectors point at: the one SINGLE drives and
 *   the panel reports ([SetupState.selectedMotor]).
 * @param drive the drive mode in force and the boards it addresses, read from the machine's stored
 *   setting ([DriveInForce]).
 * @param driving what the drive addresses NOW: the targets latched when the machine was armed, so a
 *   copy or a leader flag that changes under an armed machine does not move the arm to other
 *   boards, and [drive]'s targets while disarmed.
 * @param telemetryByBoard what each board last said about itself, by its address
 *   ([HoverboardTransport.telemetry]). A board absent from the map is silent.
 * @param selectedBoard the board picked off the Connect screen's scan list and remembered by
 *   address, or null while the app has only ever been pointed at a name.
 * @param findingBoards whether the Connect screen's scan for nearby boards is running.
 * @param boardsSeen what that scan has turned up so far, one row per address, in the order the
 *   boards were first heard from ([ScanList]).
 * @param countsPerAmp each board's `motor.current_cal`, as the attach-time ride-facts read got it
 *   ([SetupState.countsPerAmpOf]), by address: the scale belongs to the sense chain fitted to that
 *   board.
 * @param packCells each board's `board.pack_cells`, from the same read ([SetupState.packCellsOf]).
 * @param wheelGeometry each board's pole-pair count and wheel diameter, from the same read
 *   ([SetupState.wheelGeometryOf]): what turns that wheel's counted hall edges into a distance.
 */
data class UiState(
    val connectionState: ConnectionState = ConnectionState.DISCONNECTED,
    val armed: Boolean = false,
    val throttleSpeed: Int = 0,
    val steer: Int = 0,
    val engaged: Boolean = false,
    val deviceName: String = LinkConfig.DEFAULT_DEVICE_NAME,
    val disconnecting: Boolean = false,
    val selfDisarmed: Boolean = false,
    val simulateRider: Boolean = false,
    val riderWaiver: RiderWaiver = RiderWaiver.NONE,
    val machine: Machine = Machine.UNKNOWN,
    val nodes: List<NodeInfo> = emptyList(),
    val attachedNode: Int? = null,
    val selected: Int? = null,
    val drive: DriveInForce = DriveInForce.NONE,
    val driving: DriveTargets = DriveTargets.NONE,
    val telemetryByBoard: Map<Int, TelemetryUi> = emptyMap(),
    /** Per source address, the frames heard and the longest gap: the debug-build stream line. */
    val frameStats: Map<Int, FrameStats> = emptyMap(),
    val selectedBoard: SelectedBoard? = null,
    val findingBoards: Boolean = false,
    val boardsSeen: List<ScanCandidate> = emptyList(),
    val countsPerAmp: Map<Int, Int> = emptyMap(),
    val packCells: Map<Int, Int> = emptyMap(),
    val wheelGeometry: Map<Int, WheelGeometry> = emptyMap(),
) {
    val isConnected: Boolean get() = connectionState == ConnectionState.CONNECTED

    /** The motor controllers this session found, in address order. Never a bridge. */
    val motorControllers: List<NodeInfo> get() = nodes.filter { it.isMotorController }

    /** The node at [address], or null when the walk did not find one there. */
    fun nodeAt(address: Int): NodeInfo? = nodes.firstOrNull { it.address == address }

    /** What the drive mode the arm control and the stick act on is: [driving]'s. */
    val driveMode: DriveMode get() = driving.mode

    /**
     * What the SELECTED motor controller last said about itself, or null before its first frame,
     * once it has gone quiet, and while no motor controller is known. The panel's reading.
     */
    val telemetry: TelemetryUi? get() = selected?.let { telemetryByBoard[it] }

    /**
     * What the arm control says about the machine's frame and side (`specs/rider-ui.md` 3.7a).
     * Nothing while no node is attached: an app that has read no board has nothing to warn about yet.
     */
    val machineArm: MachineArm get() = if (attachedNode == null) MachineArm.None else machine.armStatement

    /**
     * What the board at [address] last said about itself, or null when it is not reporting. One
     * board's readings are never another's.
     */
    fun telemetryOf(address: Int): TelemetryUi? = telemetryByBoard[address]

    /**
     * Which motor controller is leading a rigid frame, read off what each says about itself
     * ([rigidLeader]), or null while the app cannot name one. The one board a RIGID frame drives.
     */
    val leader: Int? get() = rigidLeader(motorControllers.map { it.address }, telemetryByBoard)

    /**
     * What the board at [address]'s current display shows (`specs/rider-ui.md` 3.6): that board's
     * own window words against that board's own scale, or the reason there is no figure.
     */
    fun currentOf(address: Int): CurrentReading = currentReading(telemetryOf(address), countsPerAmp[address])

    /**
     * What the board at [address]'s battery display shows (`specs/rider-ui.md` 3.6b): its reported
     * pack voltage against the pack it stores, or the reason there is no state of charge.
     */
    fun batteryOf(address: Int): BatteryReading = batteryReading(telemetryOf(address), packCells[address])

    /**
     * What the board at [address]'s odometer shows (`specs/rider-ui.md` 3.6c): that wheel's lifetime
     * travel as a distance where the board has been told its own wheel, as the hall edges it counted
     * where it has not, or silence from a board that has sent nothing.
     */
    fun odometerOf(address: Int): OdometerReading = odometerReading(telemetryOf(address), wheelGeometry[address])

    /**
     * Whether the arm control will accept a press right now.
     *
     * False while the throttle is held, which is [MainViewModel.onArmToggle]'s refusal rendered for
     * the user: you cannot arm into a throttle that is already deflected. False as well while
     * [disconnecting], for the reason spelled out on [MainViewModel.disconnect]: the link is about
     * to go and an arm level delivered now would outlive it. It gates ARMING only; disarming is
     * always available.
     *
     * False too while the boards are known to hold different frames ([MachineArm.FramesDiffer]),
     * and while the drive mode in force addresses no motor controller at all: there is nothing to
     * arm until a node has answered that it is one.
     */
    val canArm: Boolean
        get() = isConnected && !engaged && !disconnecting && !machineArm.refuses && drive.targets.boards.isNotEmpty()

    /** Commanded throttle as a percent of MAX_SPEED, signed (-100..100). */
    val throttlePercent: Int get() = (throttleSpeed * PERCENT) / Throttle.MAX_SPEED

    private companion object {
        const val PERCENT = 100
    }
}

/**
 * Owns the arm/throttle safety logic and bridges the [HoverboardTransport].
 *
 * # The arm scheme
 *
 * A latching **arm** toggle plus a held **throttle**. One tap arms, one tap disarms, and the
 * throttle is the deadman: it is held to command travel and releasing it commands zero. The rider
 * needs one thumb, on the throttle, which is the hand position they are in anyway.
 *
 * This started as a two-control deadman, arm held under one thumb and throttle under the other. It
 * was rejected on the bench for the right reason: the throttle is ALREADY being held, so requiring
 * a second sustained touch buys nothing a rider can use, and costs them the hand they need.
 *
 * What the toggle keeps:
 *
 * - **Arming is a deliberate act.** It is its own control, which does nothing but arm, and it says
 *   what state it is in. Connecting does not arm. Touching the throttle does not arm. There is no
 *   path from "the app is open" to "the motors are live" that does not go through a tap whose only
 *   meaning is "arm this machine".
 * - **You cannot arm into a deflected throttle.** [onArmToggle] refuses to ARM while
 *   [UiState.engaged], so the machine never comes alive already being asked for travel. It never
 *   refuses to DISARM: a stop control that can be unavailable is not a stop control.
 * - **The throttle is inert unarmed.** An unarmed touch commands zero and displays zero, so the pad
 *   cannot be used to discover whether the board is armed by moving it.
 *
 * ## What the toggle gives up, and why that is the right trade
 *
 * A held control cannot drift from the firmware's level, because the finger IS the state. A toggle
 * can: the machine will sit armed at rest with nobody holding anything. That is a real loss and it
 * is worth naming rather than glossing.
 *
 * It is the right trade because the thing a deadman protects against is *unintended motion*, and
 * the throttle is still a deadman. An armed board at rest has its motor enables set and a zero
 * reference; it moves only while a finger is held on the pad, and lifting that finger commands zero
 * on the next pump tick whether or not the app is still running (and if the app is gone entirely,
 * the firmware stops honouring the last demand after 204 ms of silence). It is also how the machine
 * itself behaves: the physical power button latches, and the rider controls motion, not power.
 *
 * The remaining risk is a board left armed and forgotten, and that is what the paths below are for:
 * backgrounding the app, disconnecting, and losing the link all disarm or refuse to stay armed, so
 * "walked away from it" is covered even though "let go of it" no longer is.
 *
 * # What stops the machine
 *
 * Every path ends disarmed, but not all of them by the same mechanism, and one of them is not the
 * app's to guarantee:
 *
 *  1. **Tapping the arm control off** -> [onArmToggle] -> [RiderCommand.DISARMED] on the next pump
 *     tick. `power_request` false, `Run -> Shutdown -> Off`, motor enables cleared. Releasing the
 *     THROTTLE does not disarm, by design; it zeroes the demand and leaves the machine live.
 *  2. **Backgrounding the app** -> [onAppBackgrounded] from the activity's `ON_STOP`. This is the
 *     path that matters most under a latching toggle, because it is the one that catches a rider
 *     who put the phone in a pocket while the board was still armed. The
 *     link is deliberately left up: the pump keeps re-sending the disarmed command, which keeps the
 *     board pinned disarmed for as long as the app is backgrounded. It is also the only path here
 *     the rider did not ask for, so it latches [UiState.selfDisarmed] and the arm control says so
 *     when the app comes back.
 *  3. **Disconnecting** -> [disconnect] disarms and holds the link open [DISARM_SETTLE_MS] so the
 *     disarming command actually reaches the board BEFORE the link goes. Going quiet is not enough:
 *     see the note on [HoverboardTransport.disconnect]. The window is closed to arming for its whole
 *     length, because a link that is still up is still a link a tap can arm over.
 *  4. **Losing the link** -> the app stops being able to send at all, and the firmware unwinds on
 *     its own, in that order. The demand goes first: unhonoured after 204 ms of silence
 *     (`linkctl::DRIVE_TIMEOUT_TICKS`) and ramped down from there (~133 ms more from full travel),
 *     so the wheels stop and there is no runaway. The arm level goes second: the remote `INPUTS`
 *     mirror expires 1.5 s after the app was last heard from at all
 *     (`linkctl::INPUTS_TIMEOUT_TICKS`, applied at the read in
 *     `orchestrator::LinkInbox::remote_stale`), every level it carried then reads as released, and
 *     the board disarms itself. The ordering is compile-time asserted in the firmware rather than
 *     left to the two numbers happening to be chosen well, so the machine is always stopped before
 *     it is disarmed. Expiry is also one-way: only a fresh `INPUTS` revives the mirror, so a
 *     reconnecting demand stream cannot walk a disarmed board back to `Run` on a level nobody
 *     restated. On top of all that the app refuses to come back armed: [connectionState] leaving
 *     CONNECTED forces the local state disarmed, so a reconnect requires a fresh, deliberate press.
 */
@Suppress("TooManyFunctions") // one entry point per thing the screens offer, plus the two private send steps
class MainViewModel(
    private val transport: HoverboardTransport,
    private val settings: LinkSettings,
    private val scanner: BoardScanner,
    private val maxSpeed: Int = Throttle.MAX_SPEED,
) : ViewModel() {

    private val local = MutableStateFlow(LocalState())

    /**
     * The Setup screen's model. It refuses every edit while the arm level is held, read from the
     * same [local] state the arm control writes, so there is no second copy of "armed" to drift.
     */
    val setup = SetupModel(transport, viewModelScope) { local.value.armed }

    /**
     * The Tune screen's model. SAVE is refused while armed off the same [local] state; tune writes
     * are not (they touch no flash). Its motor controllers, its selection and what each board runs
     * for `CONTROL_RIDER_REQUIRED` are the Setup model's, the one owner of all three, so the board
     * Tune tunes is the board SINGLE drives.
     */
    val tune = TuneModel(
        transport,
        viewModelScope,
        { local.value.armed },
        setup.state.map { st ->
            val mcs = st.nodes.filter { it.isMotorController }
            TuneSelection(mcs, st.selectedMotor, mcs.associate { it.address to st.riderWaiverOf(it.address) })
        }.distinctUntilChanged(),
        setup::selectMotor,
    )

    /**
     * The board layout editor's model. Like Setup it refuses every edit while armed off the same
     * [local] state: a layout is written to flash, which an armed board refuses outright.
     */
    val layout = LayoutModel(transport, viewModelScope) { local.value.armed }

    /**
     * Set for the whole of [disconnect]'s settle window: the link is still up, but it is leaving.
     *
     * It exists because CONNECTED is not enough to decide whether arming is allowed. Inside the
     * window the transport is still CONNECTED, the control screen is still on screen and a finger is
     * still on the glass, so without this the arm control would accept a press whose level the board
     * would then hold until the mirror timed out, delivered on a link the app is dropping.
     *
     * Owned by [disconnect] and nothing else: it is set there before the disarm and released only
     * in that function's `finally`, after the link has actually been dropped. One setter, one
     * releaser, so there is no second path that could clear it while the drop is still pending.
     */
    private val disconnecting = MutableStateFlow(false)

    /**
     * The Connect screen's scan, while one is running. Held so a second press cannot start a
     * second radio scan over the first, and so starting a connect can stop it: the transport runs
     * its own scan to reach the board, and two scans racing on one adapter is how Android starts
     * dropping advertisements.
     */
    private var findJob: Job? = null
    private val findingBoards = MutableStateFlow(false)
    private val boardsSeen = MutableStateFlow<List<ScanCandidate>>(emptyList())

    /**
     * What the session knows about its nodes, gathered for [uiState]: the node list, the machine
     * statement, the drive in force, and each board's own telemetry by the address its frames
     * arrive under. The one place those are put together, so no screen has to work any of them out.
     */
    private data class Boards(
        val attached: Int?,
        val nodes: List<NodeInfo>,
        val selected: Int?,
        val machine: Machine,
        val drive: DriveInForce,
        val setup: SetupState,
        val telemetry: Map<Int, TelemetryUi>,
    )

    /** What the Connect screen's board picker knows, gathered for [uiState]. */
    private data class Picker(
        val selected: SelectedBoard?,
        val finding: Boolean,
        val seen: List<ScanCandidate>,
    )

    private val picker = combine(settings.selectedBoard, findingBoards, boardsSeen) { sel, f, seen ->
        Picker(selected = sel, finding = f, seen = seen)
    }

    private val boards =
        combine(setup.state, transport.telemetry) { st, byBoard ->
            Boards(
                attached = st.attached,
                nodes = st.nodes,
                selected = st.selectedMotor,
                machine = st.machine,
                drive = st.driveInForce(byBoard),
                setup = st,
                telemetry = byBoard,
            )
        }

    val uiState: StateFlow<UiState> =
        combine(
            transport.connectionState,
            local,
            settings.deviceName,
            disconnecting,
        ) { conn, l, name, leaving ->
            UiState(
                connectionState = conn,
                armed = l.armed,
                throttleSpeed = l.throttleSpeed,
                steer = l.steer,
                engaged = l.engaged,
                deviceName = name,
                disconnecting = leaving,
                selfDisarmed = l.selfDisarmed,
                simulateRider = l.simulateRider,
                driving = l.armedTargets ?: DriveTargets.NONE,
            )
        }.combine(boards) { ui, b ->
            val driving = if (ui.armed) ui.driving else b.drive.targets
            val mcs = b.nodes.filter { it.isMotorController }.map { it.address }
            ui.copy(
                attachedNode = b.attached,
                nodes = b.nodes,
                selected = b.selected,
                machine = b.machine,
                drive = b.drive,
                driving = driving,
                riderWaiver = b.setup.riderWaiver(driving.boards),
                telemetryByBoard = b.telemetry,
                countsPerAmp = mcs.mapNotNull { a -> b.setup.countsPerAmpOf(a)?.let { a to it } }.toMap(),
                packCells = mcs.mapNotNull { a -> b.setup.packCellsOf(a)?.let { a to it } }.toMap(),
                wheelGeometry = mcs.mapNotNull { a -> b.setup.wheelGeometryOf(a)?.let { a to it } }.toMap(),
            )
        }.combine(picker) { ui, p ->
            ui.copy(selectedBoard = p.selected, findingBoards = p.finding, boardsSeen = p.seen)
        }.combine(transport.frameStats) { ui, stats ->
            ui.copy(frameStats = stats)
        }
            .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(STATE_TIMEOUT_MS),
            initialValue = UiState(),
        )

    init {
        // A link that is not CONNECTED cannot be carrying an arm level, so the app must not go on
        // believing it holds one. This covers the reconnect case in particular: a session that
        // drops and comes back must not resume armed off a finger that never lifted.
        //
        // A FULL reset, not [LocalState.disarmed]: losing the link ends the session, and the bench
        // rider simulation must not survive into the next one. Carrying it across a reconnect is
        // how a setting made for a padless board follows the app onto a board that has pads, where
        // it holds the FSM's engage conjunction and suppresses the step-off wind-down. A setting
        // that outlives the board it was made for is not a setting, it is a latch.
        transport.connectionState
            .onEach { if (it != ConnectionState.CONNECTED) local.value = LocalState() }
            .launchIn(viewModelScope)
        // What the drive addresses follows what the session reads: the nodes' classes, the stored
        // mode and side, and the leader flags. Re-sent on every change, so a disarmed frame reaches
        // the boards the mode in force names as soon as the reads that name them answer, and an
        // armed RIGID frame follows its leader.
        combine(setup.state, transport.telemetry, ::Pair)
            .onEach { sendCurrent() }
            .launchIn(viewModelScope)
    }

    /** Begin scanning + connecting to the peripheral. */
    fun connect() {
        stopFindingBoards()
        transport.connect()
    }

    /**
     * Start listing the boards in range, so the user can pick one instead of typing its name.
     *
     * The list is emptied first: a fresh press means "what is here now", and rows left over from a
     * scan minutes ago are boards that may well be gone. Nothing times the scan out, because the
     * board the user is waiting for may be the one still being switched on; the screen's own stop
     * action ends it, and so does starting a connect.
     */
    fun findBoards() {
        if (findJob?.isActive == true) return
        boardsSeen.value = emptyList()
        findingBoards.value = true
        findJob = viewModelScope.launch {
            try {
                scanner.scan().collect { sighting ->
                    boardsSeen.update { ScanList.merge(it, sighting) }
                }
            } catch (e: CancellationException) {
                // Stopped by the user, or the ViewModel went away. Not a failure: absorbing it
                // would leave the rest of this coroutine running inside a cancelled scope.
                throw e
            } catch (_: Throwable) {
                // The radio refused: permission revoked while the screen was open, Bluetooth
                // switched off under us, an adapter that threw. There is nothing the app can do
                // about any of them, and the screen already says what it has: the scan stops and
                // the button comes back, with whatever rows were gathered still tappable.
            } finally {
                findingBoards.value = false
            }
        }
    }

    /** Stop the picker's scan, keeping the rows it gathered so one can still be tapped. */
    fun stopFindingBoards() {
        findJob?.cancel()
        findJob = null
        findingBoards.value = false
    }

    /**
     * Pin the app to the board whose row the user tapped, and connect to it.
     *
     * The pick is persisted before the connect rather than after it succeeds, because the pick is
     * a statement about which machine this phone drives and has to hold even when that machine is
     * the one that will not answer.
     */
    fun selectBoard(board: ScanCandidate) {
        stopFindingBoards()
        settings.selectBoard(board.address, board.displayName)
        transport.connect()
    }

    /**
     * Retarget the app at a differently-named board, the path for a board that will not show up in
     * a scan. Persisted immediately, and it drops any pinned board ([LinkSettings.setDeviceName]);
     * the transport reads the target at the start of each scan, so this takes effect on the next
     * connect without a restart.
     */
    fun setDeviceName(name: String) = settings.setDeviceName(name)

    /**
     * Disarm, let the disarming command reach the board, then drop the link.
     *
     * The wait is the whole point. Tearing the link down first would leave the board armed for the
     * 1.5 s it takes the remote `INPUTS` mirror to expire, with nothing left that could tell it
     * otherwise sooner; the timeout is a backstop for a phone that dies, not a way to hang up.
     * [DISARM_SETTLE_MS] is several pump ticks, so an ordinary lost frame on a best-effort link
     * still leaves a later one arriving.
     *
     * And for exactly the same reason the window has to be *closed to arming*. It is not a quiet
     * period: the link is up, the control screen is up, and the rider's hands are still on the
     * glass. A tap accepted here would put an arm level on the wire moments before the link went,
     * and the board would hold that level until the mirror timed it out while the app reset itself
     * to disarmed and rendered "disconnected". So [disconnecting] latches for the whole window and both
     * [onArmToggle] and [UiState.canArm] refuse it. Re-entry is refused too: a second tap on
     * Disconnect must not start a second window that outlives the first drop.
     */
    fun disconnect() {
        if (disconnecting.value) return
        disconnecting.value = true
        forceDisarm(selfDisarm = false)
        viewModelScope.launch {
            try {
                delay(DISARM_SETTLE_MS)
            } finally {
                // In a finally, so the link goes whether the window ran out or the ViewModel was
                // cleared inside it: clearing cancels this coroutine, and without this the settle
                // would take the pending drop with it and leave the link up with no owner.
                // Dropping BEFORE releasing the latch leaves no instant in which the app is armable
                // over a link it has decided to drop; afterwards the connection state carries the
                // refusal on its own.
                transport.disconnect()
                disconnecting.value = false
            }
        }
    }

    /**
     * The arm control was tapped: arm if disarmed, disarm if armed.
     *
     * Arming is refused while the throttle is held, while [disconnect] is settling, while the
     * boards are known to hold different frames, and while the drive mode in force addresses no
     * motor controller ([UiState.canArm]), so the machine never comes alive already being asked for
     * travel, never comes alive onto a link that is about to go, and never comes alive with one
     * wheel balancing and the other coasting. Disarming is never refused: a stop control that can
     * be unavailable is not a stop control.
     *
     * The arm LATCHES the boards the mode in force addresses ([LocalState.armedTargets]): each is
     * armed directly at its own address, never a bridge, and they stay the armed set until the
     * disarm, whatever a later read says (`specs/rider-ui.md` section 2).
     */
    fun onArmToggle() {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        if (local.value.armed) {
            forceDisarm(selfDisarm = false)
            return
        }
        // The boards hold two different frames: half an actuator, and nothing on either board can
        // see it (`specs/rider-ui.md` 3.7a). Refused here as well as on the control, so the gate is
        // not the rendering's. So is a drive that addresses no motor controller: nothing to arm.
        val st = setup.state.value
        val targets = st.driveInForce(transport.telemetry.value).targets
        val refused = local.value.engaged || disconnecting.value || st.machine.armStatement.refuses
        if (refused || targets.boards.isEmpty()) return
        // A fresh arm answers the "the app disarmed this" line, so it goes with the press that
        // makes it stale rather than lingering over a machine that is armed again.
        local.update { it.copy(armed = true, selfDisarmed = false, armedTargets = targets) }
        sendCurrent()
    }

    /**
     * Handle a throttle touch at [y] within a pad of [height] (pixels, y down). Call on touch-down
     * and on every move while held.
     *
     * The demand is zero unless armed, and the displayed speed is the demand rather than what the
     * finger position would ask for, so the pad never shows travel the board is not being asked for.
     */
    fun onThrottleMove(y: Float, height: Float) {
        val engaged = Throttle.isEngaged(height)
        val armed = local.value.armed
        val speed = if (engaged && armed) Throttle.speedFor(y, height, maxSpeed) else 0
        local.update { it.copy(throttleSpeed = speed, engaged = engaged) }
        sendCurrent()
    }

    /**
     * Throttle finger-up: demand goes to zero immediately.
     *
     * The arm level is deliberately NOT dropped here. `power_request` is a level meaning "this
     * machine is live", and a rider who lets the throttle rest at a stop has not stopped riding.
     * Dropping it would cycle the mode machine through `Off` and re-run the bring-up on every pause.
     */
    fun onThrottleRelease() {
        local.update { it.copy(throttleSpeed = 0, steer = 0, engaged = false) }
        sendCurrent()
    }

    /**
     * Handle a joystick touch at [x], [y] within a pad of [width] by [height] (pixels, y down). Call
     * on touch-down and on every move while held; finger-up goes through [onThrottleRelease], which
     * zeroes both axes.
     *
     * The control of the two modes that steer ([DriveMode.hasTurnAxis]): the vertical axis is the
     * forward demand and the horizontal one is the turn. [DriveFrame] decides what becomes of them,
     * a mixed pair in DIFFERENTIAL and one command to the leader in RIGID. Like the throttle pad,
     * both are zero unless armed, so an unarmed stick cannot be used to find out whether the board is
     * live by moving it.
     *
     * Nothing here gates on the mode. The stick is only on screen in those two modes
     * ([com.hoverboard.remote.ui.screens.ControlScreen]), and a turn reaches no wire in the other
     * two because [DriveFrame.of] drops it.
     */
    fun onJoystickMove(x: Float, y: Float, width: Float, height: Float) {
        val engaged = Throttle.isEngaged(height)
        val live = engaged && local.value.armed
        local.update {
            it.copy(
                throttleSpeed = if (live) Throttle.speedFor(y, height, maxSpeed) else 0,
                steer = if (live) Throttle.steerFor(x, width, maxSpeed) else 0,
                engaged = engaged,
            )
        }
        sendCurrent()
    }

    /**
     * The app is no longer in the foreground (the activity's `ON_STOP`).
     *
     * Disarms unconditionally. A rider cannot hold a deadman they cannot see, and Android does not
     * promise a pointer-cancel when it takes the window away, so nothing else here would fire.
     *
     * This is the one disarm nobody asked for, so it is also the one the rider has to be told
     * about: it latches [LocalState.selfDisarmed], which the arm control states when the app is
     * back on screen. Only when there was an arm level to take, because an app that backgrounds
     * while disarmed has disarmed nothing and has nothing to report.
     */
    fun onAppBackgrounded() = forceDisarm(selfDisarm = local.value.armed)

    /**
     * Turn the bench rider simulation on or off, and hand the new command to the pump.
     *
     * On the wire on the next `INPUTS` tick, not instantly: [sendCurrent] updates the value
     * [com.hoverboard.remote.ble.CommandPump] holds, and the pump sends `INPUTS` when its payload
     * differs from what it last delivered. A rider-level change counts as a difference, so this
     * goes out on the next tick with the same repeat burst an arm change gets.
     *
     * It is not rider intent, so it does NOT reset with [forceDisarm]: an operator who set it
     * before arming should not have to set it again after every stop. Losing the link DOES clear
     * it; see [LocalState.disarmed] and the `connectionState` collector. It only reaches the wire on
     * an armed command ([RiderCommand.DISARMED] states nothing), so toggling it while disarmed
     * stages it for the next arm.
     */
    fun setSimulateRider(on: Boolean) {
        local.update { it.copy(simulateRider = on) }
        sendCurrent()
    }

    /**
     * Drop the arm level and the demand together, and put that on the wire.
     *
     * [selfDisarm] is whether this disarm is the app's own doing and worth saying out loud. It is
     * taken here, and has no default, so that every disarm leaves one fully stated state behind:
     * there is no instant in which the machine is disarmed and the reason is still missing, and no
     * way to add a disarming path without deciding which kind it is.
     */
    private fun forceDisarm(selfDisarm: Boolean) {
        local.update { it.disarmed(selfDisarm) }
        sendCurrent()
    }

    /**
     * Hand the current intent to the transport as one [RiderCommand].
     *
     * One send site, so the arm level and the demand are always taken from the same snapshot of
     * state: there is no path that updates one without the other reaching the wire with it.
     */
    private fun sendCurrent() {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        val l = local.value
        val command = if (l.armed) {
            RiderCommand.armed(l.throttleSpeed, simulatingRider = l.simulateRider)
        } else {
            RiderCommand.DISARMED
        }
        // To the boards the mode in force addresses: SINGLE the selected motor controller, the
        // other three both. One value each in BOUND, which is also the pivoting balance mapping
        // (`specs/control.md` (h)); the mixed pair in DIFFERENTIAL, where the turn is spent on the
        // pair and never put on the wire; and in RIGID one command to the leader carrying both
        // axes, with the follower told only that it is armed (`specs/rover-one-imu.md` section 7).
        // Armed, the boards are the ones the arm latched; disarmed, the ones the mode names now.
        // Taken from the models' own latest state rather than from [uiState], which is a
        // `WhileSubscribed` flow whose value is the initial one until something collects it.
        val telemetry = transport.telemetry.value
        val targets = l.armedTargets ?: setup.state.value.driveInForce(telemetry).targets
        val leader = rigidLeader(targets.boards, telemetry)
        transport.sendCommand(DriveFrame.of(targets, command, steer = l.steer, leader = leader))
    }

    /**
     * The app's own idea of rider intent, the operator setting that rides with it, and how the
     * last arm level ended.
     *
     * [disarmed] rather than a bare default: every RIDER-INTENT field resets to its default there,
     * so [forceDisarm] stays a reset rather than a field-by-field clear that a later field could be
     * added behind, while [simulateRider] is carried across because it is an operator setting and
     * not something the rider is doing. It resets with the link, and so does [selfDisarmed],
     * because a link that went is its own explanation.
     *
     * [armedTargets] is the set of boards the arm latched, and part of the arm: it is null whenever
     * [armed] is false.
     */
    private data class LocalState(
        val armed: Boolean = false,
        val throttleSpeed: Int = 0,
        val steer: Int = 0,
        val engaged: Boolean = false,
        val selfDisarmed: Boolean = false,
        val simulateRider: Boolean = false,
        val armedTargets: DriveTargets? = null,
    ) {
        /**
         * The all-stop state: no arm level, no demand, no touch. Operator settings survive, and
         * [selfDisarm] records whether the app took the arm level by itself rather than the rider
         * asking for it, which is the one thing about a disarm the rider cannot see for themselves.
         */
        fun disarmed(selfDisarm: Boolean): LocalState =
            LocalState(selfDisarmed = selfDisarm, simulateRider = simulateRider)
    }

    companion object {
        /**
         * How long [disconnect] holds the link open after disarming.
         *
         * Long enough for the pump to emit the whole burst of repeats a CHANGED arm level gets
         * ([LinkConfig.INPUTS_CHANGE_REPEATS]), plus a tick of slack for one landing just after a
         * tick boundary. Sizing it off the repeat count rather than picking a number keeps it
         * correct if either the cadence or the burst length is retuned.
         */
        const val DISARM_SETTLE_MS: Long = (LinkConfig.INPUTS_CHANGE_REPEATS + 1) * LinkConfig.SEND_INTERVAL_MS

        private const val STATE_TIMEOUT_MS = 5_000L
    }
}
