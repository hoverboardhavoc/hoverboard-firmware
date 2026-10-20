package com.hoverboard.remote

import com.hoverboard.remote.ble.BoardScanner
import com.hoverboard.remote.ble.ScanCandidate
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.emitAll
import kotlinx.coroutines.flow.flow

/**
 * In-memory [BoardScanner] for unit tests: a scan that runs until it is cancelled and reports
 * exactly the advertisements a test hands it, with no Android BLE stack.
 *
 * [scans] counts the scans started, which is how a test asserts that starting a connect stopped
 * the picker's scan rather than leaving two running on one adapter.
 */
class FakeBoardScanner : BoardScanner {

    private val sightings = MutableSharedFlow<ScanCandidate>(extraBufferCapacity = BUFFER)

    var scans: Int = 0
        private set

    override fun scan(): Flow<ScanCandidate> = flow {
        scans++
        // Never completes on its own, like the radio: the collector decides when it ends.
        emitAll(sightings)
    }

    /** Deliver one advertisement to whatever is collecting. */
    fun advertise(candidate: ScanCandidate) {
        check(sightings.tryEmit(candidate)) { "the fake scan's buffer is full" }
    }

    private companion object {
        const val BUFFER = 32
    }
}
