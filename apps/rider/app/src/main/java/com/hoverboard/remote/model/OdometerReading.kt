package com.hoverboard.remote.model

/**
 * What one board's odometer display says right now (`specs/rider-ui.md` 3.6c).
 *
 * A count of hall edges or nothing at all, and there is only one way to have nothing: a board that
 * has sent no state. Zero is NOT that way. Zero edges is a real and sayable reading, a board whose
 * wheel has never turned, and folding it into the absent case would hide a board that is reporting
 * perfectly well.
 */
sealed interface OdometerReading {

    /** Nothing has arrived from this board ([TelemetryUi.hasState]). */
    data object Silent : OdometerReading

    /**
     * This wheel's lifetime travel, in ABSOLUTE hall edges: every counted commutation edge adds 1
     * whichever way the wheel turned, so the figure never winds back.
     *
     * EDGES, not a distance. Converting needs the motor's pole-pair count and the wheel's
     * diameter, and neither `motor.pole_pairs` nor a wheel diameter is a registered field, so the
     * app has no way to be told either and shows what the board measured.
     */
    data class Edges(val edges: Long) : OdometerReading
}

/** What [telemetry] reports for its own wheel. */
fun odometerReading(telemetry: TelemetryUi?): OdometerReading {
    val edges = telemetry?.odometerEdges ?: return OdometerReading.Silent
    return OdometerReading.Edges(edges)
}

/**
 * The MACHINE's travel: the MEAN of the boards' figures, not their sum.
 *
 * Both wheels cover the same ground, so adding them reports twice the travel. That is the opposite
 * of the current section, which sums because the two boards draw from one pack and their draws
 * genuinely add.
 *
 * Null when any board has not reported. A one-wheel mean is not a machine's travel, and nothing
 * distinguishes a second board that has not sent a sample yet from one that is not fitted, so the
 * figure is withheld rather than taken from the wheel that did report.
 *
 * The two wheels need not agree, and a difference between them is a real fact about the machine:
 * one wheel may have been fitted later, or spun on a stand. The mean is what the pair covered
 * together as far as the counts can say.
 */
fun machineOdometerEdges(boards: List<OdometerReading>): Long? {
    if (boards.isEmpty()) return null
    var total = 0L
    for (board in boards) {
        when (board) {
            is OdometerReading.Edges -> total += board.edges
            OdometerReading.Silent -> return null
        }
    }
    return total / boards.size
}
