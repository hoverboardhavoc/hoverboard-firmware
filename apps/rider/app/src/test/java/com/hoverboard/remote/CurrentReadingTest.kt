package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.currentReading
import com.hoverboard.remote.model.machineSupplyAmps
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertInstanceOf
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The two amp figures the panel shows (`specs/rider-ui.md` 3.6), and the four states that are not
 * a figure.
 *
 * The arithmetic half is one line of physics: the board measures the PHASE current, a supply reads
 * the DC LINK, and in six-step one phase pair conducts, so the link is the phase current scaled by
 * the duty. The other half is what this type exists for, which is refusing to produce a number
 * out of a window nothing is refreshing.
 */
class CurrentReadingTest {

    /** 455 counts per amp, the registry default and the figure measured on the bench. */
    private val cal = 455

    @Test
    fun `the phase current is the window mean over the board's counts per amp`() {
        val reading = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL), cal)

        val amps = assertInstanceOf(CurrentReading.Amps::class.java, reading)
        assertEquals(2.0f, amps.motor, 0.001f)
        // A fully conducting period makes the two equal: the link carries the phase current whole.
        assertEquals(2.0f, amps.supply, 0.001f)
    }

    /**
     * The link figure is the phase current scaled by the duty that was APPLIED, which is the whole
     * reason both numbers are shown: at half duty the supply reads half what the motor takes, and
     * reading one for the other is the mistake that cost a morning on 2026-10-09.
     */
    @Test
    fun `the link current is the phase current scaled by the duty`() {
        val reading = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL / 2), cal)

        val amps = assertInstanceOf(CurrentReading.Amps::class.java, reading)
        assertEquals(2.0f, amps.motor, 0.001f)
        assertEquals(1.0f, amps.supply, 0.01f)
    }

    /** The mean, not the peak: the peak is a maximum over ADC samples and reads high. */
    @Test
    fun `the peak is not what the figures are derived from`() {
        val reading = currentReading(telemetry(mean = 455, peak = 4_550, duty = CyclicObs.DUTY_FULL), cal)

        val amps = assertInstanceOf(CurrentReading.Amps::class.java, reading)
        assertEquals(1.0f, amps.motor, 0.001f)
    }

    @Test
    fun `a board nothing has arrived from has no reading`() {
        assertEquals(CurrentReading.Silent, currentReading(null, cal))
        assertEquals(CurrentReading.Silent, currentReading(TelemetryUi(), cal))
    }

    /**
     * The stale-number case, and the reason this is a mode check rather than a value check. The
     * current window is written by the period interrupt alone, which does not run while the motor
     * path is down, so a board sitting in OFF keeps sending whatever its last live window held.
     * Drawn as amps that would be a current the machine is not taking, minutes after it stopped.
     */
    @Test
    fun `a board whose motors are down shows no figure, however large its last window was`() {
        val loud = telemetry(mean = 9_000, duty = CyclicObs.DUTY_FULL, mode = OFF)

        assertEquals(CurrentReading.MotorsOff, currentReading(loud, cal))
        for (mode in listOf(SHUTDOWN, UNNAMED)) {
            val quiet = telemetry(mean = 9_000, duty = CyclicObs.DUTY_FULL, mode = mode)
            assertEquals(CurrentReading.MotorsOff, currentReading(quiet, cal), "mode $mode")
        }
    }

    /** Powered is powered: the window is live from bring-up, before any torque is asked for. */
    @Test
    fun `the figures appear as soon as the motor path is up`() {
        for (mode in listOf(INIT, READY, RUN)) {
            val reading = currentReading(telemetry(mean = 455, duty = CyclicObs.DUTY_FULL, mode = mode), cal)
            assertInstanceOf(CurrentReading.Amps::class.java, reading, "mode $mode")
        }
    }

    /**
     * A period that drove nothing. No phase conducted, so the link current is nothing and the
     * measured magnitude is the sense chain's rest noise; neither is worth a number, and 4 A of
     * noise on a resting machine is worse than no number at all.
     */
    @Test
    fun `a coasting period shows no figure rather than its noise`() {
        assertEquals(CurrentReading.Coasting, currentReading(telemetry(mean = 2_000, duty = 0), cal))
    }

    /** No scale, no amps: a figure at the registry default would be a different board's constant. */
    @Test
    fun `a board whose calibration this session has not read shows no figure`() {
        val live = telemetry(mean = 455, duty = CyclicObs.DUTY_FULL)

        assertEquals(CurrentReading.Unscaled, currentReading(live, null))
        assertEquals(CurrentReading.Unscaled, currentReading(live, 0))
    }

    /**
     * The sum is what a bench supply's own reading is compared against: the supply feeds both
     * boards, so one board's figure is a share of what it displays and nothing more.
     */
    @Test
    fun `the machine's supply current is the two boards' link figures added`() {
        val master = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL / 2), cal)
        val slave = currentReading(telemetry(mean = 455, duty = CyclicObs.DUTY_FULL / 2), cal)

        assertEquals(1.5f, machineSupplyAmps(listOf(master, slave))!!, 0.01f)
    }

    /** Each board's own counts, its own scale and its own duty: two derivations that are added. */
    @Test
    fun `each board's figure uses that board's own calibration`() {
        val master = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL), 455)
        val slave = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL), 910)

        assertEquals(3.0f, machineSupplyAmps(listOf(master, slave))!!, 0.01f)
    }

    /** A board that drove nothing contributes a real zero: its bridge is not on the supply. */
    @Test
    fun `a board that is off or coasting contributes nothing rather than blocking the sum`() {
        val master = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL), cal)

        assertEquals(2.0f, machineSupplyAmps(listOf(master, CurrentReading.MotorsOff))!!, 0.01f)
        assertEquals(2.0f, machineSupplyAmps(listOf(master, CurrentReading.Coasting))!!, 0.01f)
    }

    /**
     * And a board that is not reporting blocks it outright. A sum that quietly became one board's
     * figure would be a wrong number presented confidently, against a supply reading that carries
     * both boards.
     */
    @Test
    fun `a board that is not reporting leaves no sum at all`() {
        val master = currentReading(telemetry(mean = 910, duty = CyclicObs.DUTY_FULL), cal)

        assertNull(machineSupplyAmps(listOf(master, CurrentReading.Silent)))
        assertNull(machineSupplyAmps(listOf(master, CurrentReading.Unscaled)))
    }

    private fun telemetry(
        mean: Int,
        duty: Int,
        peak: Int = mean,
        mode: Int = RUN,
    ) = TelemetryUi().merge(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = 2_900,
            mode = mode,
            fault = 0,
            flags = 0,
            peerDemand = 0,
            wheelPos = 0,
            wheelRate = 0,
            obs = CyclicObs(
                phasePeak = peak,
                phaseMean = mean,
                dutyOn = duty,
                bootTag = 0,
                chip = ChipTag.F130C8,
                odometer = 0L,
            ),
        ),
    )

    private companion object {
        /** `motorcontroller/state/src/mode.rs`, `Mode`, in discriminant order. */
        const val OFF = 0
        const val INIT = 1
        const val READY = 2
        const val RUN = 3
        const val SHUTDOWN = 4

        /** A mode byte past the five, which this build cannot name. */
        const val UNNAMED = 9
    }
}
