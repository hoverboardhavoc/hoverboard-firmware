package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.background
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.foundation.layout.Box
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.totalKmText
import com.hoverboard.remote.model.tripKmText
import com.hoverboard.remote.ui.theme.CautionGlyph
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import java.util.Locale

/**
 * The Ride screen, the 8a "Dial" design (`specs/rider-ui.md` 3.1): riding information and the
 * controls that move and stop the machine, with as little text as the facts allow.
 *
 * **The drive pad does not move.** The screen is a column of fixed-height slots from the top of the
 * content area: the instruments (276 dp), the pad (280 dp), the ARM control (52 dp), 12 dp apart.
 * Nothing above the pad changes height with state, and everything that changes with state sits
 * below it (the statements, the fault grid, the trip line, the debug line), so a thumb on the pad
 * means the same demand whatever the screen is saying. The column scrolls, so a phone shorter than
 * the layout scrolls the bottom away rather than squeezing the pad, whose HEIGHT is its resolution.
 *
 * Laid out for ONE thumb: the pad is the control being modulated and the only one that is held; the
 * arm toggle is a tap under it. See [com.hoverboard.remote.MainViewModel] for the safety argument.
 *
 * Per-board state, the drive mode and the machine's configuration statements are on Setup's
 * machine status section (3.1c); the board link dots and the BATTERY LOW and LAMP TEST chips are in
 * the top bar ([RideTopBarStatus]).
 */
@Composable
fun ControlScreen(
    state: UiState,
    onArmToggle: () -> Unit,
    onThrottleMove: (y: Float, height: Float) -> Unit,
    onJoystickMove: (x: Float, y: Float, width: Float, height: Float) -> Unit,
    onThrottleRelease: () -> Unit,
    modifier: Modifier = Modifier,
    showFrameStats: Boolean = false,
) {
    var sheet by rememberSaveable { mutableStateOf<FaultLamp?>(null) }
    val open = { lamp: FaultLamp ->
        onThrottleRelease()
        sheet = lamp
    }
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState()),
    ) {
        RideInstruments(state)
        Spacer(modifier = Modifier.height(SLOT_GAP))
        RideControls(
            state = state,
            onArmToggle = onArmToggle,
            onThrottleMove = onThrottleMove,
            onJoystickMove = onJoystickMove,
            onThrottleRelease = onThrottleRelease,
            gridShowing = state.gridShows,
            onOpenLamp = open,
            modifier = Modifier.padding(horizontal = GUTTER),
        )
        Column(modifier = Modifier.padding(horizontal = GUTTER).padding(bottom = 16.dp)) {
            if (state.gridShows) {
                Spacer(modifier = Modifier.height(SLOT_GAP))
                FaultGrid(state, onOpen = open)
            }
            Spacer(modifier = Modifier.height(SLOT_GAP))
            TripLine(state)
            if (showFrameStats && state.frameStats.isNotEmpty()) {
                Text(
                    text = frameStatsLine(state.frameStats),
                    style = ride(10.sp),
                    color = FgFaint,
                    modifier = Modifier.testTag(FRAME_STATS_TAG),
                )
            }
        }
    }
    sheet?.let { LampSheet(state, it, onDismiss = { sheet = null }) }
}

/**
 * "Trip 3.1 km     Total 413 km": the machine's travel and that less the trip's baseline, blank
 * and keeping its height while there is no machine distance.
 */
@Composable
private fun TripLine(state: UiState) {
    val total = state.machineTravel as? MachineTravel.Distance
    val trip = state.trip.trip(total)
    val text = if (total == null || trip == null) {
        ""
    } else {
        stringResource(R.string.ride_trip, tripKmText(trip), totalKmText(total.millimetres))
    }
    Text(
        text = text,
        style = ride(14.sp),
        color = FgMuted,
        textAlign = TextAlign.Center,
        modifier = Modifier.fillMaxWidth().testTag(RIDE_TRIP_TAG),
    )
}

/**
 * What Ride's top bar carries (`specs/rider-ui.md` 3.1): a link dot per board slot at the start,
 * in [FgMuted] while the phone hears that board and the caution glyph while it is silent, each with
 * its slot label; and at the end, before the overflow icon, LAMP TEST during the lamp test, else
 * BATTERY LOW with the caution glyph while the pack is low, else nothing.
 */
@Composable
fun RideTopBarStatus(state: UiState, modifier: Modifier = Modifier) {
    Row(modifier = modifier.padding(start = 16.dp), verticalAlignment = Alignment.CenterVertically) {
        for (slot in state.slots.all) {
            val heard = state.telemetryOf(slot.address) != null
            Box(Modifier.size(DOT_BOX), contentAlignment = Alignment.Center) {
                if (heard) {
                    Box(Modifier.size(8.dp).background(FgMuted, CircleShape))
                } else {
                    CautionGlyph(DOT_BOX)
                }
            }
            Spacer(Modifier.width(4.dp))
            Text(
                slotLabel(slot),
                style = ride(11.sp),
                color = FgMuted,
                modifier = Modifier.testTag(linkDotTag(slot.address)),
            )
            Spacer(Modifier.width(10.dp))
        }
        Spacer(Modifier.weight(1f))
        when {
            state.lampTest -> Text(
                stringResource(R.string.ride_lamp_test),
                style = ride(12.sp, letterSpacing = 1.sp),
                color = FgMuted,
                modifier = Modifier.testTag(RIDE_CHIP_TAG),
            )
            state.batteryIsLow -> CautionText(
                stringResource(R.string.ride_battery_low),
                style = ride(12.sp, FontWeight.Bold, letterSpacing = 0.8.sp),
                modifier = Modifier.testTag(RIDE_CHIP_TAG),
            )
        }
    }
}

/**
 * The debug-build stream line under the strip (`specs/rider-ui.md` 3.1; `specs/link-control.md`,
 * "Counting what is staged (2026-10-25)"): for each source address heard this session, in address
 * order, the frames received and the longest gap between two of them, e.g.
 * `0x01 412 · gap 0.41 s   0x02 206 · gap 1.31 s`. A gap past the 1.2 s quiet window on the
 * forward's source is the SILENT flicker, and the board's own count of the forwards it staged
 * (`CTRL_OBS` word 40) says which side of the link lost them.
 */
internal fun frameStatsLine(stats: Map<Int, FrameStats>): String =
    stats.toSortedMap().entries.joinToString("   ") { (addr, s) ->
        "${boardHex(addr)} ${s.frames} · gap ${"%.2f".format(Locale.ROOT, s.longestGapMs / MS_PER_S)} s"
    }

private const val MS_PER_S = 1000.0

/** 8a's side gutter. */
private val GUTTER = 24.dp

/** The link dot's box: the 10 dp caution glyph, or the 8 dp dot centred in it. */
private val DOT_BOX = 10.dp

/** Test tag for the debug-build per-source frame line under the strip. */
const val FRAME_STATS_TAG = "frame_stats"

/** Test tag on the trip line. */
const val RIDE_TRIP_TAG = "ride_trip"

/** Test tag on the top bar's chip (LAMP TEST or BATTERY LOW). */
const val RIDE_CHIP_TAG = "ride_chip"

/** Test tag on the link dot label of the board at [address]. */
fun linkDotTag(address: Int): String = "link_dot_${boardHex(address)}"
