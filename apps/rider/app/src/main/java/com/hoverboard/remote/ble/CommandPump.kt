package com.hoverboard.remote.ble

import com.hoverboard.protocol.linkctl.Inputs
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

/**
 * Serialised, rate-limited sender for rider [Inputs] frames (the app streams at ~10 Hz).
 *
 * Why this exists: BLE allows only one outstanding GATT operation at a time. Launching a
 * coroutine per touch-move event (a finger fires ~90/s) produced overlapping concurrent
 * writes to the same characteristic, which the BLE stack rejects. This pump instead holds the
 * *latest* [Inputs] and writes it on a single coroutine at a fixed cadence:
 *  - exactly one write in flight at a time (no concurrency), and
 *  - the held value is re-sent every tick so a stationary finger keeps the link fed (otherwise
 *    the board's no-command timeout would stop the motors mid-drive).
 *
 * A failed individual write is swallowed (the next tick retries); the deadman watchdog on the
 * board is the backstop. [start]/[stop] bracket a connection's lifetime.
 */
class CommandPump(
    private val scope: CoroutineScope,
    private val intervalMs: Long,
    private val write: suspend (Inputs) -> Unit,
) {
    private val pending = MutableStateFlow(STOP)
    private var job: Job? = null

    /** Update the [Inputs] to be streamed. Cheap; safe to call at UI event rate. */
    fun set(inputs: Inputs) {
        pending.value = inputs
    }

    /** Begin streaming the latest [Inputs] at [intervalMs]. Idempotent per connection. */
    fun start() {
        if (job?.isActive == true) return
        job = scope.launch {
            while (isActive) {
                try {
                    write(pending.value)
                } catch (e: CancellationException) {
                    throw e
                } catch (_: Exception) {
                    // Transient write failure (link blip, permission revoked, op in flight):
                    // drop this frame and retry on the next tick.
                }
                delay(intervalMs)
            }
        }
    }

    /** Stop streaming and reset the held value to the all-stop deadman [Inputs]. */
    fun stop() {
        job?.cancel()
        job = null
        pending.value = STOP
    }

    companion object {
        /** All-stop, rider disengaged: throttle 0, no buttons, rider bit clear (deadman). */
        val STOP: Inputs = Inputs(throttle = 0, buttons = 0, rider = 0)
    }
}
