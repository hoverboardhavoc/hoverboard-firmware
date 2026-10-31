package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.background
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.foundation.layout.Box
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.FaultLamp
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.MachineTravel
import com.hoverboard.remote.model.totalKmText
import com.hoverboard.remote.model.tripKmText
import com.hoverboard.remote.ui.theme.BgCode
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.CautionGlyph
import com.hoverboard.remote.ui.theme.CautionText
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import java.util.Locale

/**
 * The Ride screen, the 8a "Dial" design (`specs/remote-ui.md` 3.1): riding information and the
 * controls that move and stop the machine, with as little text as the facts allow.
 *
 * **The drive pad does not move, and nothing scrolls.** From the bottom of the content area up: the
 * debug line (debug builds), the trip line, the statements' two-line slot, the ARM control and the
 * pad (280 dp), each of fixed height; the instruments take what is left above, at most 276 dp, and
 * shrink to fit it ([RideInstruments]). So on a given phone the pad has one place, whatever the
 * screen is saying, and a thumb on it means the same demand. While the fault grid shows (disarmed)
 * it is drawn in the pad's bounds, and while no motor controller reports, the motor-controller-off
 * card ([BoardsOffCard]) takes the instruments' and the pad's place.
 *
 * Laid out for ONE thumb: the pad is the control being modulated and the only one that is held; the
 * arm toggle is a tap under it. See [com.hoverboard.remote.MainViewModel] for the safety argument.
 *
 * Per-board state, the drive mode and the machine's configuration statements are on Setup's
 * machine status section (3.1c); the board link dots and the BATTERY LOW and LAMP TEST chips are in
 * the top bar ([RideTopBarStatus]).
 *
 * @param onOpenSetup open Setup on a board, scrolled to a row: a missing setting's statement.
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
    onOpenSetup: (board: Int, row: Key) -> Unit = { _, _ -> },
) {
    var sheet by rememberSaveable { mutableStateOf<FaultLamp?>(null) }
    val open = { lamp: FaultLamp ->
        onThrottleRelease()
        sheet = lamp
    }
    // A pad that leaves composition under a finger never sees its finger-up, so the demand is
    // released when the grid or the card takes its place, as on a destination change (2a).
    val padGone = state.boardsOff || (state.gridShows && !state.armed)
    val release by rememberUpdatedState(onThrottleRelease)
    LaunchedEffect(padGone) { if (padGone) release() }
    Column(modifier = modifier.fillMaxSize()) {
        if (state.boardsOff) {
            BoardsOffCard(
                state,
                Modifier.padding(horizontal = GUTTER).fillMaxWidth().weight(1f),
            )
            Spacer(modifier = Modifier.height(SLOT_GAP))
        } else {
            RideInstruments(state, Modifier.weight(1f))
            Spacer(modifier = Modifier.height(SLOT_GAP))
        }
        // The grid takes the pad's bounds only while disarmed: while armed the pad keeps its place
        // (`specs/remote-ui.md` 3.1b).
        val grid: (@Composable () -> Unit)? = if (state.gridShows && !state.armed) {
            { FaultGrid(state, onOpen = open) }
        } else {
            null
        }
        RideControls(
            state = state,
            onArmToggle = onArmToggle,
            onThrottleMove = onThrottleMove,
            onJoystickMove = onJoystickMove,
            onThrottleRelease = onThrottleRelease,
            grid = grid,
            padShown = !state.boardsOff,
            statementsHeight = STATEMENTS_HEIGHT,
            onOpenLamp = open,
            onOpenSetup = { board, row ->
                onThrottleRelease()
                onOpenSetup(board, row)
            },
            modifier = Modifier.padding(horizontal = GUTTER),
        )
        Column(modifier = Modifier.padding(horizontal = GUTTER).padding(bottom = BOTTOM_MARGIN)) {
            TripLine(state, blank = state.boardsOff)
            if (showFrameStats) {
                Text(
                    text = frameStatsLine(state.frameStats),
                    style = ride(10.sp),
                    color = FgFaint,
                    maxLines = 1,
                    modifier = Modifier.height(DEBUG_LINE_HEIGHT).testTag(FRAME_STATS_TAG),
                )
            }
        }
    }
    sheet?.let { LampSheet(state, it, onDismiss = { sheet = null }) }
}

/**
 * The motor-controller-off card (`specs/remote-ui.md` 3.1): the link is up and no motor controller
 * has reported for 1 s, which is the boards unpowered behind a bridge that runs from USB. It takes
 * the instruments' and the pad's place, down to the pad's bottom.
 */
@Composable
private fun BoardsOffCard(state: UiState, modifier: Modifier = Modifier) {
    val shape = RoundedCornerShape(24.dp)
    Column(
        modifier = modifier
            .testTag(RIDE_BOARDS_OFF_TAG)
            .clip(shape)
            .background(BgCode)
            .border(1.dp, Border, shape)
            .padding(24.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        PowerGlyph(40.dp)
        Spacer(Modifier.height(16.dp))
        Text(stringResource(R.string.ride_off_title), style = ride(22.sp, FontWeight.Bold), color = Fg)
        Spacer(Modifier.height(6.dp))
        Text(stringResource(R.string.ride_off_action), style = ride(16.sp), color = Fg)
        Spacer(Modifier.height(16.dp))
        Text(
            stringResource(R.string.ride_off_waiting, state.linkName),
            style = ride(14.sp),
            color = FgMuted,
            textAlign = TextAlign.Center,
            maxLines = 2,
            overflow = TextOverflow.Ellipsis,
        )
    }
}

/** A power glyph: a 300 degree arc open at the top, and a bar down through the gap, in [FgMuted]. */
@Composable
private fun PowerGlyph(size: Dp) {
    Canvas(Modifier.size(size)) {
        val w = 3.dp.toPx()
        val r = this.size.minDimension / 2f - w
        val c = Offset(this.size.width / 2f, this.size.height / 2f + w / 2)
        drawArc(
            color = FgMuted,
            startAngle = POWER_ARC_START,
            sweepAngle = POWER_ARC_SWEEP,
            useCenter = false,
            topLeft = Offset(c.x - r, c.y - r),
            size = Size(r * 2, r * 2),
            style = Stroke(w, cap = StrokeCap.Round),
        )
        drawLine(FgMuted, Offset(c.x, c.y - r - w / 2), Offset(c.x, c.y - r * POWER_BAR_END), w, cap = StrokeCap.Round)
    }
}

/**
 * "Trip 3.1 km     Total 413 km": the machine's travel and that less the trip's baseline, blank
 * and keeping its height while there is no machine distance.
 */
@Composable
private fun TripLine(state: UiState, blank: Boolean) {
    val total = state.machineTravel as? MachineTravel.Distance
    val trip = state.trip.trip(total)
    val text = if (blank || total == null || trip == null) {
        ""
    } else {
        stringResource(R.string.ride_trip, tripKmText(trip), totalKmText(total.millimetres))
    }
    Text(
        text = text,
        style = ride(14.sp),
        color = FgMuted,
        textAlign = TextAlign.Center,
        maxLines = 1,
        modifier = Modifier.fillMaxWidth().height(TRIP_LINE_HEIGHT).testTag(RIDE_TRIP_TAG),
    )
}

/**
 * What Ride's top bar carries (`specs/remote-ui.md` 3.1): a link dot per board slot at the start,
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
 * The debug-build stream line under the strip (`specs/remote-ui.md` 3.1; `specs/link-control.md`,
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

/** The fixed slots under the ARM control (`specs/remote-ui.md` 3.1): statements, trip line, debug line, margin. */
private val STATEMENTS_HEIGHT = 32.dp
private val TRIP_LINE_HEIGHT = 20.dp
private val DEBUG_LINE_HEIGHT = 14.dp
private val BOTTOM_MARGIN = 8.dp

/** The power glyph's arc: 300 degrees from the top's right side, and the bar's end, as a part of the radius. */
private const val POWER_ARC_START = -60f
private const val POWER_ARC_SWEEP = 300f
private const val POWER_BAR_END = 0.1f

/** Test tag on the motor-controller-off card. */
const val RIDE_BOARDS_OFF_TAG = "ride_boards_off"

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
