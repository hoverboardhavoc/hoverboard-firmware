package com.hoverboard.remote

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.ble.LinkConfig
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.Throttle
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/**
 * UI state surfaced to Compose.
 *
 * @param connectionState BLE link state.
 * @param telemetry latest merged telemetry, or null before the first frame.
 * @param armed whether the arm control is currently held, i.e. whether the app is asserting
 *   `power_request` and the board's motors are enabled.
 * @param throttleSpeed current commanded demand (-MAX..MAX), 0 whenever not armed.
 * @param engaged whether the throttle pad is currently held.
 * @param disconnecting whether [MainViewModel.disconnect] is mid-teardown: disarmed, but still
 *   CONNECTED while the disarming command reaches the board.
 */
data class UiState(
    val connectionState: ConnectionState = ConnectionState.DISCONNECTED,
    val telemetry: TelemetryUi? = null,
    val armed: Boolean = false,
    val throttleSpeed: Int = 0,
    val engaged: Boolean = false,
    val deviceName: String = LinkConfig.DEFAULT_DEVICE_NAME,
    val disconnecting: Boolean = false,
) {
    val isConnected: Boolean get() = connectionState == ConnectionState.CONNECTED

    /**
     * Whether the arm control will accept a press right now.
     *
     * False while the throttle is held, which is [MainViewModel.onArmToggle]'s refusal rendered for
     * the user: you cannot arm into a throttle that is already deflected. False as well while
     * [disconnecting], for the reason spelled out on [MainViewModel.disconnect]: the link is about
     * to go and an arm level delivered now would outlive it. It gates ARMING only; disarming is
     * always available.
     */
    val canArm: Boolean get() = isConnected && !engaged && !disconnecting

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
 *     board pinned disarmed for as long as the app is backgrounded.
 *  3. **Disconnecting** -> [disconnect] disarms and holds the link open [DISARM_SETTLE_MS] so the
 *     disarming command actually reaches the board BEFORE the link goes. Going quiet is not enough:
 *     see the note on [HoverboardTransport.disconnect]. The window is closed to arming for its whole
 *     length, because a link that is still up is still a link a tap can arm over.
 *  4. **Losing the link** -> the app stops being able to send at all. The firmware stops honouring
 *     the last demand after 204 ms of silence and ramps the reference down from there (~133 ms more
 *     from full travel), so the wheels stop and there is no runaway. But the board **stays armed**,
 *     because the firmware's remote `INPUTS` slot has no staleness of any kind
 *     (`crates/orchestrator/src/lib.rs:223`): it holds the last level delivered, forever. No amount
 *     of Kotlin fixes that; it needs an age on that slot in the firmware, exactly as `DRIVE_CMD`
 *     already has. What the app does do is refuse to come back armed: [connectionState] leaving
 *     CONNECTED forces the local state disarmed, so a reconnect requires a fresh, deliberate press.
 */
class MainViewModel(
    private val transport: HoverboardTransport,
    private val settings: LinkSettings,
    private val maxSpeed: Int = Throttle.MAX_SPEED,
) : ViewModel() {

    private val local = MutableStateFlow(LocalState())

    /**
     * Set for the whole of [disconnect]'s settle window: the link is still up, but it is leaving.
     *
     * It exists because CONNECTED is not enough to decide whether arming is allowed. Inside the
     * window the transport is still CONNECTED, the control screen is still on screen and a finger is
     * still on the glass, so without this the arm control would accept a press whose level the board
     * would then hold forever, delivered on a link the app is in the middle of dropping.
     *
     * Owned by [disconnect] and [dropLink] and nothing else: one setter and one clearer, so there is
     * no second path that could clear it while the drop is still pending.
     */
    private val disconnecting = MutableStateFlow(false)

    val uiState: StateFlow<UiState> =
        combine(
            transport.connectionState,
            transport.telemetry,
            local,
            settings.deviceName,
            disconnecting,
        ) { conn, telem, l, name, leaving ->
            UiState(
                connectionState = conn,
                telemetry = telem,
                armed = l.armed,
                throttleSpeed = l.throttleSpeed,
                engaged = l.engaged,
                deviceName = name,
                disconnecting = leaving,
            )
        }.stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(STATE_TIMEOUT_MS),
            initialValue = UiState(),
        )

    init {
        // A link that is not CONNECTED cannot be carrying an arm level, so the app must not go on
        // believing it holds one. This covers the reconnect case in particular: a session that
        // drops and comes back must not resume armed off a finger that never lifted.
        transport.connectionState
            .onEach { if (it != ConnectionState.CONNECTED) local.value = LocalState() }
            .launchIn(viewModelScope)
    }

    /** Begin scanning + connecting to the peripheral. */
    fun connect() = transport.connect()

    /**
     * Retarget the app at a differently-named board. Persisted immediately; the transport reads the
     * name at the start of each scan, so this takes effect on the next connect without a restart.
     */
    fun setDeviceName(name: String) = settings.setDeviceName(name)

    /**
     * Disarm, let the disarming command reach the board, then drop the link.
     *
     * The wait is the whole point. The board holds the last `power_request` level it was delivered
     * with no staleness, so tearing the link down first would leave it armed with nothing left that
     * could tell it otherwise. [DISARM_SETTLE_MS] is several pump ticks, so an ordinary lost frame
     * on a best-effort link still leaves a later one arriving.
     *
     * And for exactly the same reason the window has to be *closed to arming*. It is not a quiet
     * period: the link is up, the control screen is up, and the rider's hands are still on the
     * glass. A tap accepted here would put an arm level on the wire moments before the link went,
     * and the board would hold that level forever while the app reset itself to disarmed and
     * rendered "disconnected". So [disconnecting] latches for the whole window and both
     * [onArmToggle] and [UiState.canArm] refuse it. Re-entry is refused too: a second tap on
     * Disconnect must not start a second window that outlives the first drop.
     */
    fun disconnect() {
        if (disconnecting.value) return
        disconnecting.value = true
        forceDisarm()
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
     * Arming is refused while the throttle is held and while [disconnect] is settling
     * ([UiState.canArm]), so the machine never comes alive already being asked for travel, and never
     * comes alive onto a link that is about to go. Disarming is never refused: a stop control that
     * can be unavailable is not a stop control.
     */
    fun onArmToggle() {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        if (local.value.armed) {
            forceDisarm()
            return
        }
        if (local.value.engaged || disconnecting.value) return
        local.update { it.copy(armed = true) }
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
        local.update { it.copy(throttleSpeed = 0, engaged = false) }
        sendCurrent()
    }

    /**
     * The app is no longer in the foreground (the activity's `ON_STOP`).
     *
     * Disarms unconditionally. A rider cannot hold a deadman they cannot see, and Android does not
     * promise a pointer-cancel when it takes the window away, so nothing else here would fire.
     */
    fun onAppBackgrounded() = forceDisarm()

    /** Drop the arm level and the demand together, and put that on the wire. */
    private fun forceDisarm() {
        local.value = LocalState()
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
        transport.sendCommand(
            if (l.armed) RiderCommand.armed(l.throttleSpeed) else RiderCommand.DISARMED,
        )
    }

    /**
     * The app's own idea of rider intent. Its default IS the disarmed state, so [forceDisarm] is a
     * reset to it rather than a field-by-field clear that a later field could be added behind.
     */
    private data class LocalState(
        val armed: Boolean = false,
        val throttleSpeed: Int = 0,
        val engaged: Boolean = false,
    )

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
