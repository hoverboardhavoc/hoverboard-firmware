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
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.DriveMode
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.Throttle
import com.hoverboard.remote.model.leaderNode
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
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
 * @param telemetry what the ATTACHED board last said about itself, or null before its first frame
 *   and once it has gone quiet. Keyed out of [HoverboardTransport.telemetry] by that board's own
 *   address, so it is never the other board's reading.
 * @param slaveTelemetry what the SLAVE last said about itself, the same way: the attached board
 *   re-emits its peer's state under the peer's address, so the second board on the strip reports
 *   rather than being guessed at. Null when no slave was found, before its first frame, and once
 *   its stream has stopped.
 * @param armed whether the arm control is currently held, i.e. whether the app is asserting
 *   `power_request` and the board's motors are enabled.
 * @param throttleSpeed current commanded demand (-MAX..MAX), 0 whenever not armed. In DIFFERENTIAL
 *   this is the mix's `v`, the stick's forward axis, and not what either wheel is told.
 * @param steer current commanded turn (-MAX..MAX), 0 whenever not armed and 0 in the two modes
 *   with nowhere to spend it. DIFFERENTIAL spends it on the difference between two per-wheel
 *   demands; RIGID puts it on the wire for the leader to split.
 * @param engaged whether the throttle pad is currently held.
 * @param disconnecting whether [MainViewModel.disconnect] is mid-teardown: disarmed, but still
 *   CONNECTED while the disarming command reaches the board.
 * @param simulateRider whether the bench affordance that asserts the `INPUTS` rider bit is on. Off
 *   by default; see [RiderCommand.inputs].
 * @param selfDisarmed whether the app is the thing that took the arm level away, rather than a
 *   fault or a lost link ([MainViewModel.onAppBackgrounded]). Set when the app disarms a machine
 *   that was armed, and cleared by the next arm, so the arm control can say it happened.
 * @param riderWaiver whether the attached board runs, or may run, with `CONTROL_RIDER_REQUIRED` at 0
 *   ([SetupState.riderWaiver]), which the arm control states.
 * @param machine the pair's frame and side as this session has read them ([SetupState.machine]),
 *   which the arm control acts on and the board chips badge (`specs/rider-ui.md` 3.7a).
 * @param driveMode which boards the demand goes to ([DriveMode]); SINGLE unless the operator chose
 *   otherwise this session.
 * @param masterBoard the attached board's L3 address, or null while none is attached.
 * @param slaveBoard the discovered slave's L3 address, or null when this session found none.
 * @param selectedBoard the board picked off the Connect screen's scan list and remembered by
 *   address, or null while the app has only ever been pointed at a name.
 * @param findingBoards whether the Connect screen's scan for nearby boards is running.
 * @param boardsSeen what that scan has turned up so far, one row per address, in the order the
 *   boards were first heard from ([ScanList]).
 */
data class UiState(
    val connectionState: ConnectionState = ConnectionState.DISCONNECTED,
    val telemetry: TelemetryUi? = null,
    val slaveTelemetry: TelemetryUi? = null,
    val armed: Boolean = false,
    val throttleSpeed: Int = 0,
    val steer: Int = 0,
    val engaged: Boolean = false,
    val deviceName: String = LinkConfig.DEFAULT_DEVICE_NAME,
    val disconnecting: Boolean = false,
    val selfDisarmed: Boolean = false,
    val simulateRider: Boolean = false,
    val riderWaiver: RiderWaiver = RiderWaiver.NONE,
    val machine: MachinePair = MachinePair.UNKNOWN,
    val driveMode: DriveMode = DriveMode.SINGLE,
    val masterBoard: Int? = null,
    val slaveBoard: Int? = null,
    val selectedBoard: SelectedBoard? = null,
    val findingBoards: Boolean = false,
    val boardsSeen: List<ScanCandidate> = emptyList(),
) {
    val isConnected: Boolean get() = connectionState == ConnectionState.CONNECTED

    /**
     * What the arm control says about the pair's frame and side (`specs/rider-ui.md` 3.7a).
     * Nothing while no board is attached: an app that has read neither board has nothing to warn
     * about yet.
     */
    val machineArm: MachineArm get() = if (masterBoard == null) MachineArm.None else machine.armStatement

    /**
     * What [node] last said about itself, or null when that board is not reporting. One board's
     * readings are never the other's: a screen aimed at the slave asks for the slave's.
     */
    fun telemetryOf(node: Node): TelemetryUi? =
        if (node == Node.MASTER) telemetry else slaveTelemetry

    /**
     * Which board is leading a rigid frame, read off what the two boards say about themselves
     * ([leaderNode]), or null while the app cannot name one. The one board a RIGID frame drives.
     */
    val leader: Node? get() = leaderNode(telemetry, slaveTelemetry)

    /**
     * Whether [mode] can be chosen right now. One answer, read by the chip that offers it and by
     * [MainViewModel.setDriveMode], so a tap cannot reach a mode the row shows as unavailable.
     */
    fun canSelect(mode: DriveMode): Boolean =
        canChangeDriveMode &&
            (!mode.needsSlave || slaveBoard != null) &&
            (!mode.needsLeader || leader != null)

    /**
     * Whether the arm control will accept a press right now.
     *
     * False while the throttle is held, which is [MainViewModel.onArmToggle]'s refusal rendered for
     * the user: you cannot arm into a throttle that is already deflected. False as well while
     * [disconnecting], for the reason spelled out on [MainViewModel.disconnect]: the link is about
     * to go and an arm level delivered now would outlive it. It gates ARMING only; disarming is
     * always available.
     *
     * False too while the two boards are known to hold different frames ([MachineArm.FramesDiffer]):
     * one wheel would balance and the other coast, and no board can see its peer's store, so this
     * refusal is the app's alone to make.
     */
    val canArm: Boolean get() = isConnected && !engaged && !disconnecting && !machineArm.refuses

    /**
     * Whether the drive mode can be changed now: connected and disarmed. Changing which boards are
     * driven while armed would leave a board armed that the app has stopped streaming to; the pump
     * would disarm it ([com.hoverboard.remote.ble.CommandPump]), but a mode change is not a way to
     * stop a board, so it is not offered as one.
     */
    val canChangeDriveMode: Boolean get() = isConnected && !armed && !disconnecting

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
     * are not (they touch no flash). It hides Profile B on the master by the Setup model's knowledge
     * of what the master runs for `CONTROL_RIDER_REQUIRED`, the one owner of that fact.
     */
    val tune = TuneModel(transport, viewModelScope, { local.value.armed }, setup.state.map { it.riderWaiver })

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
     * What the session knows about the boards, gathered for [uiState]: the two addresses, the pair's
     * machine statement, and each board's own telemetry picked out of the keyed map by the address
     * that board's frames arrive under. The one place an address is turned into a board's readings,
     * so no screen has to guess which of the two streams it is looking at.
     */
    private data class Boards(
        val master: Int?,
        val slave: Int?,
        val riderWaiver: RiderWaiver,
        val machine: MachinePair,
        val telemetry: TelemetryUi?,
        val slaveTelemetry: TelemetryUi?,
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
        combine(
            transport.attachedBoard,
            transport.slaveBoard,
            setup.state,
            transport.telemetry,
        ) { m, sl, st, byBoard ->
            Boards(
                master = m,
                slave = sl,
                riderWaiver = st.riderWaiver,
                machine = st.machine,
                telemetry = m?.let { byBoard[it] },
                slaveTelemetry = sl?.let { byBoard[it] },
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
                driveMode = l.driveMode,
            )
        }.combine(boards) { ui, b ->
            ui.copy(
                masterBoard = b.master,
                slaveBoard = b.slave,
                riderWaiver = b.riderWaiver,
                machine = b.machine,
                telemetry = b.telemetry,
                slaveTelemetry = b.slaveTelemetry,
            )
        }.combine(picker) { ui, p ->
            ui.copy(selectedBoard = p.selected, findingBoards = p.finding, boardsSeen = p.seen)
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
     * Arming is refused while the throttle is held, while [disconnect] is settling, and while the
     * two boards are known to hold different frames ([UiState.canArm]), so the machine never comes
     * alive already being asked for travel, never comes alive onto a link that is about to go, and
     * never comes alive with one wheel balancing and the other coasting. Disarming is never
     * refused: a stop control that can be unavailable is not a stop control.
     */
    fun onArmToggle() {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        if (local.value.armed) {
            forceDisarm(selfDisarm = false)
            return
        }
        if (local.value.engaged || disconnecting.value) return
        // The pair holds two different frames: half an actuator, and nothing on either board can
        // see it (`specs/rider-ui.md` 3.7a). Refused here as well as on the control, so the gate is
        // not the rendering's.
        if (setup.state.value.machine.armStatement.refuses) return
        // A fresh arm answers the "the app disarmed this" line, so it goes with the press that
        // makes it stale rather than lingering over a machine that is armed again.
        local.update { it.copy(armed = true, selfDisarmed = false) }
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
     * Choose what the stick means and which boards hear it (`specs/rider-ui.md` 3.2). Refused while
     * armed ([UiState.canChangeDriveMode]), because changing what a stick means mid-drive is not a
     * thing to allow, and a mode that drives the slave ([DriveMode.needsSlave]) is refused while
     * discovery has found none. RIGID is refused on top of that while no board reports itself the
     * leader ([DriveMode.needsLeader]): it is the one mode that names a board by its role, and a
     * role the app cannot see is a board it cannot address. An operator setting like
     * [setSimulateRider]: it survives a disarm and is reset when the link drops, since the slave
     * address and the roles it relies on are session-scoped.
     */
    fun setDriveMode(mode: DriveMode) {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        if (local.value.armed || disconnecting.value) return
        if (mode.needsSlave && transport.slaveBoard.value == null) return
        if (mode.needsLeader && visibleLeader() == null) return
        local.update { it.copy(driveMode = mode) }
        sendCurrent()
    }

    /**
     * Which board is reporting itself the rigid frame's leader, from the transport's own latest
     * state rather than from [uiState].
     *
     * The same read as [UiState.leader] and off the same words; it is taken here because
     * [uiState] is a `WhileSubscribed` flow whose value is the initial one until something
     * collects it, and the two gates above are taken from the transport for that reason.
     */
    private fun visibleLeader(): Node? {
        val byBoard = transport.telemetry.value
        return leaderNode(
            master = transport.attachedBoard.value?.let { byBoard[it] },
            slave = transport.slaveBoard.value?.let { byBoard[it] },
        )
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
        // To the boards the mode names: SINGLE the master, the other three both. One value each in
        // BOUND, which is also the pivoting balance mapping (`specs/control.md` (h)); the mixed
        // pair in DIFFERENTIAL, where the turn is spent on the pair and never put on the wire; and
        // in RIGID one command to the leader carrying both axes, with the follower told only that
        // it is armed (`specs/rover-one-imu.md` section 7).
        transport.sendCommand(DriveFrame.of(l.driveMode, command, steer = l.steer, leader = visibleLeader()))
    }

    /**
     * The app's own idea of rider intent, the two operator settings that ride with it, and how the
     * last arm level ended.
     *
     * [disarmed] rather than a bare default: every RIDER-INTENT field resets to its default there,
     * so [forceDisarm] stays a reset rather than a field-by-field clear that a later field could be
     * added behind, while [simulateRider] and [driveMode] are carried across because they are
     * operator settings and not something the rider is doing. Both reset with the link, and so
     * does [selfDisarmed], because a link that went is its own explanation.
     */
    private data class LocalState(
        val armed: Boolean = false,
        val throttleSpeed: Int = 0,
        val steer: Int = 0,
        val engaged: Boolean = false,
        val selfDisarmed: Boolean = false,
        val simulateRider: Boolean = false,
        val driveMode: DriveMode = DriveMode.SINGLE,
    ) {
        /**
         * The all-stop state: no arm level, no demand, no touch. Operator settings survive, and
         * [selfDisarm] records whether the app took the arm level by itself rather than the rider
         * asking for it, which is the one thing about a disarm the rider cannot see for themselves.
         */
        fun disarmed(selfDisarm: Boolean): LocalState =
            LocalState(selfDisarmed = selfDisarm, simulateRider = simulateRider, driveMode = driveMode)
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
