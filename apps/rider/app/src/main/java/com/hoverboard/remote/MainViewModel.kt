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
 */
data class UiState(
    val connectionState: ConnectionState = ConnectionState.DISCONNECTED,
    val telemetry: TelemetryUi? = null,
    val armed: Boolean = false,
    val throttleSpeed: Int = 0,
    val engaged: Boolean = false,
    val deviceName: String = LinkConfig.DEFAULT_DEVICE_NAME,
) {
    val isConnected: Boolean get() = connectionState == ConnectionState.CONNECTED

    /**
     * Whether the arm control will accept a press right now.
     *
     * False while the throttle is held, which is [MainViewModel.onArmPress]'s refusal rendered for
     * the user: you cannot arm into a throttle that is already deflected.
     */
    val canArm: Boolean get() = isConnected && !engaged

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
 * Two controls, both held, both required. The rider holds an **arm** control with one thumb and the
 * **throttle** pad with the other; the board moves only while both are held. This is the classic
 * two-control deadman, and each half answers a different requirement:
 *
 * - **Arming is a deliberate act.** It is its own control, in its own place on the screen, which
 *   does nothing but arm. Connecting does not arm. Touching the throttle does not arm. There is no
 *   path from "the app is open" to "the motors are live" that does not go through a press whose
 *   only meaning is "arm this machine".
 * - **It is held, because the firmware semantics are a level.** `power_request` is sampled every
 *   4 ms tick and `Run -> Shutdown` fires the moment it reads false
 *   (`crates/state/src/mode.rs:206-208`). A toggle would model that level as an edge and leave the
 *   app's idea of armed and the board's able to disagree; a held control cannot drift from it,
 *   because the finger IS the state.
 * - **You cannot arm into a deflected throttle.** [onArmPress] refuses while [UiState.engaged], so
 *   the machine never comes alive already being asked for full travel. The rider arms at rest and
 *   then asks for motion, in that order, every time.
 * - **The throttle is inert unarmed.** An unarmed touch commands zero and displays zero, so the pad
 *   cannot be used to discover whether the board is armed by moving it.
 *
 * ## Why two hands rather than one
 *
 * Two hands is the point, not a limitation worked around. Any one-handed version has to give up one
 * of the two properties above: either the arm control latches (so it is no longer a level, and a
 * pocketed phone stays armed), or it sits under the same hand as the throttle, close enough that
 * one grip covers both, and a single accidental contact arms and drives together. Two sustained
 * touches at opposite edges of the screen is a posture almost nothing produces by accident, and it
 * has a useful side effect: the rider is holding the phone in two hands, so dropping it releases
 * both controls at once.
 *
 * # What stops the machine
 *
 * Every path ends disarmed, but not all of them by the same mechanism, and one of them is not the
 * app's to guarantee:
 *
 *  1. **Releasing the arm control** -> [onArmRelease] -> [RiderCommand.DISARMED] on the next pump
 *     tick (<=100 ms). `power_request` false, `Run -> Shutdown -> Off`, motor enables cleared.
 *  2. **Backgrounding the app** -> [onAppBackgrounded] from the activity's `ON_STOP`. The touch is
 *     not reliably cancelled when Android takes the window away, so this does not wait for one. The
 *     link is deliberately left up: the pump keeps re-sending the disarmed command, which keeps the
 *     board pinned disarmed for as long as the app is backgrounded.
 *  3. **Disconnecting** -> [disconnect] disarms and holds the link open [DISARM_SETTLE_MS] so the
 *     disarming command actually reaches the board BEFORE the link goes. Going quiet is not enough:
 *     see the note on [HoverboardTransport.disconnect].
 *  4. **Losing the link** -> the app stops being able to send at all. The drive reference decays to
 *     zero within 200 ms and the wheels stop, so there is no runaway. But the board **stays armed**,
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

    val uiState: StateFlow<UiState> =
        combine(
            transport.connectionState,
            transport.telemetry,
            local,
            settings.deviceName,
        ) { conn, telem, l, name ->
            UiState(
                connectionState = conn,
                telemetry = telem,
                armed = l.armed,
                throttleSpeed = l.throttleSpeed,
                engaged = l.engaged,
                deviceName = name,
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
     */
    fun disconnect() {
        forceDisarm()
        viewModelScope.launch {
            delay(DISARM_SETTLE_MS)
            transport.disconnect()
        }
    }

    /**
     * The arm control went down. Asserts `power_request` and holds it until [onArmRelease].
     *
     * Refused while the throttle is held ([UiState.canArm]): arming into a deflected throttle would
     * make the machine's first act a demand the rider set before it was live.
     */
    fun onArmPress() {
        if (transport.connectionState.value != ConnectionState.CONNECTED) return
        if (local.value.engaged) return
        local.update { it.copy(armed = true) }
        sendCurrent()
    }

    /** The arm control came up: `power_request` drops and the demand goes neutral in one frame. */
    fun onArmRelease() = forceDisarm()

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
         * Three pump ticks. One would be a race against the pump's own cadence; three means the
         * disarming command is staged and flushed at least twice even if the first attempt lands
         * just after a tick, which matters on a link with no retransmit.
         */
        const val DISARM_SETTLE_MS: Long = 3 * LinkConfig.SEND_INTERVAL_MS

        private const val STATE_TIMEOUT_MS = 5_000L
    }
}
