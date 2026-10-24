package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.machineOdometer
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the Travel section. */
const val TRAVEL_SECTION_TAG = "setup_travel"

/**
 * How far each wheel has travelled (`specs/rider-ui.md` 3.6c).
 *
 * **It is on Setup rather than on the Ride panel, because the Ride screen has no room left.**
 * That screen's drive control has a height floor and `ControlScreenTest`
 * holds it on screen without scrolling; measured against that test, the panel has between 24 dp
 * and 30 dp of slack at 411x891 dp, and one row of this block is about as tall as that on its own.
 * Three rows and the note are four times what is there. Setup scrolls, so the block fits whole,
 * and the figure is a lifetime total that nobody reads while riding.
 *
 * **It does not follow the board selector**, for the reason the Machine section above it does not:
 * the machine's figure needs both boards, and the two wheels side by side are the comparison worth
 * drawing.
 *
 * **A distance only for a board that has been told its own wheel, and counts for one that has
 * not.** The board records absolute hall edges and can record nothing else: converting needs the
 * motor's pole-pair count and the wheel's rolling diameter, both measured on the machine and
 * written at a bench. A board told neither, or only one of the two, shows the count, because a
 * number captioned as a distance that is really a count reads as authoritative and is wrong by
 * whatever factor the wheel happens to be. Every row carries its own unit in its text, so no
 * reader has to work out which of the two they are looking at.
 *
 * **The machine figure is the MEAN of the two wheels, not their sum.** Both wheels cover the same
 * ground. That is the opposite of the Ride panel's current rows, where the two boards draw from
 * one pack and their currents genuinely add. It is a distance only when BOTH wheels converted:
 * two rows in different units cannot be averaged into one number.
 *
 * **One way to have no figure**, and zero is not it: a board that has sent nothing. Zero edges is
 * a board reporting that its wheel has never turned, which is a reading and is drawn as one.
 */
@Composable
internal fun TravelSection(odometer: OdometerReading, peerOdometer: OdometerReading?) {
    val readings = listOfNotNull(odometer, peerOdometer)
    Section(stringResource(R.string.setup_travel_title)) {
        Panel(modifier = Modifier.testTag(TRAVEL_SECTION_TAG)) {
            BoardOdometer(stringResource(R.string.setup_travel_master), odometer)
            if (peerOdometer != null) {
                BoardOdometer(stringResource(R.string.setup_travel_slave), peerOdometer)
                MachineOdometer(machineOdometer(readings))
            }
            Caption(
                if (allConverted(readings)) {
                    R.string.setup_travel_note_converted
                } else {
                    R.string.setup_travel_note
                },
            )
        }
    }
}

/**
 * Whether every board that has reported turned its count into a distance, which is the condition
 * for the converted note. A pair where one wheel is known and the other is not is reading in two
 * units, and the note that fits it is the one saying a distance needs the wheel's size.
 */
private fun allConverted(readings: List<OdometerReading>): Boolean =
    readings.any { it is OdometerReading.Travelled } && readings.none { it is OdometerReading.Edges }

/** One wheel's row: a distance, an edge count, or the statement that the board has not reported. */
@Composable
private fun BoardOdometer(tag: String, reading: OdometerReading) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            text = tag,
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
            modifier = Modifier.width(ODOMETER_TAG_WIDTH),
        )
        when (reading) {
            is OdometerReading.Travelled -> Text(
                text = distance(
                    reading.millimetres,
                    R.string.setup_travel_metres,
                    R.string.setup_travel_kilometres,
                ),
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface,
            )
            is OdometerReading.Edges -> Text(
                text = stringResource(R.string.setup_travel_value, reading.edges),
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface,
            )
            OdometerReading.Silent -> Text(
                text = stringResource(R.string.setup_travel_silent),
                style = MaterialTheme.typography.bodyMedium,
                color = TextSecondary,
            )
        }
    }
}

/**
 * The pair's travel: the two wheels averaged, drawn for a pair alone, in whichever unit both
 * wheels could give it in.
 *
 * Null is a board that has not reported, and the row says so rather than passing one wheel's
 * figure off as the machine's.
 */
@Composable
private fun MachineOdometer(travel: MachineTravel?) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            text = stringResource(R.string.setup_travel_machine),
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
            modifier = Modifier.width(ODOMETER_TAG_WIDTH),
        )
        Text(
            text = when (travel) {
                null -> stringResource(R.string.setup_travel_machine_incomplete)
                is MachineTravel.Distance -> distance(
                    travel.millimetres,
                    R.string.setup_travel_machine_metres,
                    R.string.setup_travel_machine_kilometres,
                )
                is MachineTravel.Edges -> stringResource(R.string.setup_travel_machine_value, travel.edges)
            },
            style = MaterialTheme.typography.bodyMedium,
            color = if (travel == null) TextSecondary else MaterialTheme.colorScheme.onSurface,
        )
    }
}

/**
 * [millimetres] in the unit that suits its magnitude: whole [metres] below a kilometre, and
 * [kilometres] to two decimals from there up.
 *
 * Both units, because neither reads well over the whole range a lifetime counter covers. A board
 * that has done fifty metres reading "0.05 km" throws away what the counter actually knows, and a
 * machine that has done four hundred kilometres reading "400,000 m" is a number nobody can take
 * in at a glance.
 */
@Composable
private fun distance(millimetres: Long, metres: Int, kilometres: Int): String =
    if (millimetres < MM_PER_KM) {
        stringResource(metres, millimetres / MM_PER_METRE)
    } else {
        stringResource(kilometres, millimetres.toDouble() / MM_PER_KM)
    }

/** Millimetres in a metre and in a kilometre, the two scales the rows are drawn at. */
private const val MM_PER_METRE = 1_000L
private const val MM_PER_KM = 1_000_000L

/** Room for the longest of the three row tags, so every row's figure starts in the same place. */
private val ODOMETER_TAG_WIDTH = 76.dp
