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
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.machineOdometerEdges
import com.hoverboard.remote.ui.theme.TextSecondary

/** Test tag on the Travel section. */
const val TRAVEL_SECTION_TAG = "setup_travel"

/**
 * How far each wheel has travelled, in hall edges (`specs/rider-ui.md` 3.6c).
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
 * **Counts, and the rows say they are counts.** The board records absolute hall edges and can
 * record nothing else: a distance needs the motor's pole-pair count and the wheel's diameter, and
 * neither is a registered field, so neither board nor app has been told either. A number captioned
 * as a distance that is really a count reads as authoritative and is wrong by whatever factor the
 * wheel happens to be, so the note says what is missing instead of the display guessing it.
 *
 * **The machine figure is the MEAN of the two wheels, not their sum.** Both wheels cover the same
 * ground. That is the opposite of the Ride panel's current rows, where the two boards draw from
 * one pack and their currents genuinely add.
 *
 * **One way to have no figure**, and zero is not it: a board that has sent nothing. Zero edges is
 * a board reporting that its wheel has never turned, which is a reading and is drawn as one.
 */
@Composable
internal fun TravelSection(odometer: OdometerReading, peerOdometer: OdometerReading?) {
    Section(stringResource(R.string.setup_travel_title)) {
        Panel(modifier = Modifier.testTag(TRAVEL_SECTION_TAG)) {
            BoardOdometer(stringResource(R.string.setup_travel_master), odometer)
            if (peerOdometer != null) {
                BoardOdometer(stringResource(R.string.setup_travel_slave), peerOdometer)
                MachineOdometer(machineOdometerEdges(listOf(odometer, peerOdometer)))
            }
            Caption(R.string.setup_travel_note)
        }
    }
}

/** One wheel's row: its edge count, or the statement that the board has not reported. */
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
 * The pair's travel: the two wheels' counts averaged, drawn for a pair alone.
 *
 * Null is a board that has not reported, and the row says so rather than passing one wheel's
 * count off as the machine's.
 */
@Composable
private fun MachineOdometer(edges: Long?) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(
            text = stringResource(R.string.setup_travel_machine),
            style = MaterialTheme.typography.labelMedium,
            color = TextSecondary,
            modifier = Modifier.width(ODOMETER_TAG_WIDTH),
        )
        Text(
            text = if (edges == null) {
                stringResource(R.string.setup_travel_machine_incomplete)
            } else {
                stringResource(R.string.setup_travel_machine_value, edges)
            },
            style = MaterialTheme.typography.bodyMedium,
            color = if (edges == null) TextSecondary else MaterialTheme.colorScheme.onSurface,
        )
    }
}

/** Room for the longest of the three row tags, so every row's figure starts in the same place. */
private val ODOMETER_TAG_WIDTH = 76.dp
