package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipPath
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.LineHeightStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverboard.remote.R
import com.hoverboard.remote.UiState
import com.hoverboard.remote.model.BatteryReading
import com.hoverboard.remote.model.CurrentReading
import com.hoverboard.remote.model.RING_SEGMENTS
import com.hoverboard.remote.model.RideSlot
import com.hoverboard.remote.model.SlotSide
import com.hoverboard.remote.model.SpeedReading
import com.hoverboard.remote.model.angleText
import com.hoverboard.remote.model.litSegments
import com.hoverboard.remote.model.phaseFill
import com.hoverboard.remote.model.ringScale
import com.hoverboard.remote.model.speedText
import com.hoverboard.remote.ui.theme.Atkinson
import com.hoverboard.remote.ui.theme.BgCode
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.Caution
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgFaint
import com.hoverboard.remote.ui.theme.FgMuted
import com.hoverboard.remote.ui.theme.Neon
import com.hoverboard.remote.ui.theme.Primary
import com.hoverboard.remote.ui.theme.TABULAR_FIGURES
import kotlin.math.abs

/*
 * The Ride screen's instruments slot (`specs/remote-ui.md` 3.1): the battery ring with the speed in
 * it, the pack line, two attitude corners and the phase bars, in a box at most 276 dp tall. Every
 * figure is what a board sent in its own `CYCLIC_STATE`, or a stored field read with the ride
 * facts, and a figure with no value shows its own words, never a stale or a default number.
 *
 * The box is as tall as the screen leaves it, and nothing scrolls: the ring and the speed scale by
 * k = (h - 64) / 212, at most 1, while the pack line, the corners and the phase bars keep their size
 * and are placed from the box's bottom, so their text stays readable on the short phone. At 276 dp
 * every position is 8a's.
 */

/** Test tag on the speed figure. */
const val RIDE_SPEED_TAG = "ride_speed"

/** Test tag on the pack line. */
const val RIDE_PACK_TAG = "ride_pack"

/** Test tag on the phase bar figure of the board in [side]'s slot. */
fun ridePhaseTag(side: SlotSide): String = "ride_phase_${side.name.lowercase()}"

/**
 * The instruments slot: full width, and as tall as the room it is given up to
 * [INSTRUMENTS_HEIGHT], centred vertically in that room.
 */
@Composable
internal fun RideInstruments(state: UiState, modifier: Modifier = Modifier) {
    val (_, battery) = state.ring
    val low = state.batteryIsLow
    BoxWithConstraints(modifier = modifier.fillMaxWidth(), contentAlignment = Alignment.Center) {
        val h = minOf(maxHeight, INSTRUMENTS_HEIGHT)
        val k = ringScale(h.value)
        Box(modifier = Modifier.fillMaxWidth().height(h).testTag(RIDE_INSTRUMENTS_TAG)) {
            BatteryRing(battery, low, k)
            Speed(state.speed, k)
            PackLine(battery, low, k)
            val cornerTop = h - CORNER_BOTTOM - CORNER_HEIGHT
            state.slots.left?.let { AttitudeCorner(state, it, Modifier.offset(x = CORNER_INSET, y = cornerTop)) }
            state.slots.right?.let {
                AttitudeCorner(state, it, Modifier.align(Alignment.TopEnd).offset(x = -CORNER_INSET, y = cornerTop))
            }
            PhaseBars(state, top = h - PHASE_FROM_BOTTOM)
        }
    }
}

/** Test tag on the instruments box. */
const val RIDE_INSTRUMENTS_TAG = "ride_instruments"

/**
 * Ten segments over 240 degrees from 150 degrees (Compose `drawArc` angles: 0 is three o'clock,
 * clockwise), so the open 120 degrees face down; `ceil(percent / 10)` lit, in [Primary] or in
 * [Caution] while the pack is low, the rest [BgCode]. A reading with no state of charge lights none.
 */
@Composable
private fun BatteryRing(battery: BatteryReading, low: Boolean, k: Float) {
    val lit = (battery as? BatteryReading.Charge)?.let { litSegments(it.percent) } ?: 0
    val on = if (low) Caution else Primary
    Canvas(Modifier.fillMaxWidth().height(INSTRUMENTS_HEIGHT * k)) {
        val c = Offset(size.width / 2f, (RING_CENTRE_Y * k).toPx())
        val r = (RING_RADIUS * k).toPx()
        val stroke = Stroke((RING_STROKE * k).toPx(), cap = StrokeCap.Butt)
        val seg = RING_SWEEP / RING_SEGMENTS
        for (i in 0 until RING_SEGMENTS) {
            drawArc(
                color = if (i < lit) on else BgCode,
                startAngle = RING_START + i * seg + RING_GAP,
                sweepAngle = seg - 2 * RING_GAP,
                useCenter = false,
                topLeft = Offset(c.x - r, c.y - r),
                size = Size(r * 2, r * 2),
                style = stroke,
            )
        }
    }
}

/**
 * The speed's magnitude and its unit, with "R" after the unit while the speed is below zero, in a
 * box 132k dp tall centred on the ring's centre; the words that stand for no speed wrap inside the
 * ring. A missing setting is said in the statements, not here.
 */
@Composable
private fun BoxScope.Speed(speed: SpeedReading, k: Float) {
    Column(
        modifier = Modifier
            .align(Alignment.TopCenter)
            .offset(y = (RING_CENTRE_Y - SPEED_BOX / 2) * k)
            .fillMaxWidth()
            .height(SPEED_BOX * k),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        val figure = when (speed) {
            is SpeedReading.Kmh -> speedText(speed.shown)
            SpeedReading.Silent, is SpeedReading.NoSpeed -> stringResource(R.string.ride_figure_none)
        }
        Text(
            figure,
            style = speedStyle(k),
            color = if (speed is SpeedReading.Kmh) Fg else FgFaint,
            modifier = Modifier.testTag(RIDE_SPEED_TAG),
        )
        when (speed) {
            is SpeedReading.Kmh -> Row(verticalAlignment = Alignment.CenterVertically) {
                Text(stringResource(R.string.ride_kmh), style = ride(14.sp), color = FgMuted)
                if (speed.kmh < 0) {
                    Spacer(Modifier.width(4.dp))
                    Text(
                        stringResource(R.string.ride_reverse),
                        style = ride(14.sp, FontWeight.Bold),
                        color = Fg,
                        modifier = Modifier.testTag(RIDE_REVERSE_TAG),
                    )
                }
            }
            is SpeedReading.NoSpeed -> Text(stringResource(R.string.ride_kmh), style = ride(14.sp), color = FgFaint)
            SpeedReading.Silent -> Text(
                stringResource(R.string.ride_no_board_reporting),
                style = ride(12.sp),
                color = FgMuted,
                textAlign = TextAlign.Center,
                maxLines = 2,
                overflow = TextOverflow.Ellipsis,
                modifier = Modifier.widthIn(max = minOf(SPEED_WORDS_WIDTH, RING_INNER * k)),
            )
        }
    }
}

/** Test tag on the "R" beside the unit. */
const val RIDE_REVERSE_TAG = "ride_reverse"

/** "31.2 V   72 %", or the voltage alone, or no pack voltage, centred under the ring's centre. */
@Composable
private fun BoxScope.PackLine(battery: BatteryReading, low: Boolean, k: Float) {
    val text = when (battery) {
        BatteryReading.NoReading -> stringResource(R.string.ride_no_pack_voltage)
        is BatteryReading.Unjudged -> stringResource(R.string.ride_pack_volts, battery.volts)
        is BatteryReading.Charge -> stringResource(R.string.ride_pack, battery.volts, battery.percent)
    }
    Text(
        text,
        style = ride(18.sp, FontWeight.Bold),
        color = when {
            low -> Caution
            battery is BatteryReading.Charge -> Fg
            else -> FgMuted
        },
        textAlign = TextAlign.Center,
        modifier = Modifier
            .align(Alignment.TopCenter)
            .offset(y = PACK_TOP * k)
            .testTag(RIDE_PACK_TAG),
    )
}

/**
 * One board's attitude: its slot label in [Neon], the mini horizon, and its own pitch and roll to
 * 0.1 degree; the horizon's ring alone and dashes for a silent board, and "no IMU" for a board whose
 * `imu.model` reads 0.
 */
@Composable
private fun AttitudeCorner(state: UiState, slot: RideSlot, modifier: Modifier = Modifier) {
    val cyclic = state.telemetryOf(slot.address)?.cyclic
    val noImu = state.imuModel[slot.address] == 0
    Column(modifier = modifier.width(CORNER_WIDTH), horizontalAlignment = Alignment.CenterHorizontally) {
        Text(slotLabel(slot), style = ride(11.sp, FontWeight.Bold), color = Neon)
        Spacer(Modifier.height(2.dp))
        MiniHorizon(
            attitude = cyclic?.takeIf { !noImu }?.let { it.pitch / CENTI to it.roll / CENTI },
            size = HORIZON_SIZE,
        )
        Spacer(Modifier.height(4.dp))
        val small = ride(10.sp)
        when {
            noImu -> Text(stringResource(R.string.ride_no_imu), style = small, color = FgMuted)
            cyclic == null -> {
                Text(stringResource(R.string.ride_pitch_none), style = small, color = FgMuted)
                Text(stringResource(R.string.ride_roll_none), style = small, color = FgMuted)
            }
            else -> {
                Text(stringResource(R.string.ride_pitch, angleText(cyclic.pitch)), style = small, color = FgMuted)
                Text(stringResource(R.string.ride_roll, angleText(cyclic.roll)), style = small, color = FgMuted)
            }
        }
    }
}

/**
 * 8a's mini horizon: a circle clipping a sky of [Neon] at 35 percent above the horizon and
 * [BgElevated] below, the horizon rotated by minus the roll about the centre and moved down by the
 * pitch at 2 dp per degree, clamped to the radius, with ladder ticks at plus and minus 10 degrees;
 * a 1 dp [Border] ring and a fixed board symbol in [Fg]. With no [attitude] (pitch, roll), the ring
 * alone.
 */
@Composable
private fun MiniHorizon(attitude: Pair<Float, Float>?, size: Dp) {
    Canvas(Modifier.size(size)) {
        val r = this.size.minDimension / 2f
        val c = Offset(this.size.width / 2, this.size.height / 2)
        if (attitude != null) {
            val (pitch, roll) = attitude
            val circle = Path().apply { addOval(Rect(c.x - r, c.y - r, c.x + r, c.y + r)) }
            val perDegree = HORIZON_DP_PER_DEGREE.toPx()
            clipPath(circle) {
                rotate(-roll, c) {
                    val horizonY = c.y + (pitch * perDegree).coerceIn(-r, r)
                    drawRect(
                        Neon.copy(alpha = SKY_ALPHA),
                        Offset(c.x - r * 2, c.y - r * 2),
                        Size(r * 4, horizonY - (c.y - r * 2)),
                    )
                    drawRect(BgElevated, Offset(c.x - r * 2, horizonY), Size(r * 4, r * 4))
                    drawLine(FgMuted, Offset(c.x - r * 2, horizonY), Offset(c.x + r * 2, horizonY), 1.5.dp.toPx())
                    for (d in listOf(-LADDER_DEGREES, LADDER_DEGREES)) {
                        val y = horizonY - d * perDegree
                        drawLine(
                            FgMuted.copy(alpha = LADDER_ALPHA),
                            Offset(c.x - r * LADDER_HALF, y),
                            Offset(c.x + r * LADDER_HALF, y),
                            1.dp.toPx(),
                        )
                    }
                }
            }
        }
        drawCircle(Border, r, c, style = Stroke(1.dp.toPx()))
        if (attitude != null) {
            val w = 2.dp.toPx()
            for (sign in listOf(-1f, 1f)) {
                val inner = Offset(c.x + sign * r * SYMBOL_INNER, c.y)
                val outer = Offset(c.x + sign * r * SYMBOL_OUTER, c.y)
                drawLine(Fg, inner, outer, w, cap = StrokeCap.Round)
            }
            drawCircle(Fg, 1.6.dp.toPx(), c)
        }
    }
}

/** One phase bar per slot, the row inset 40 dp from each edge, the two 32 dp apart. */
@Composable
private fun BoxScope.PhaseBars(state: UiState, top: Dp) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .offset(y = top)
            .padding(horizontal = PHASE_INSET),
        horizontalArrangement = Arrangement.spacedBy(32.dp),
    ) {
        for (slot in listOf(state.slots.left, state.slots.right)) {
            Box(Modifier.weight(1f)) { if (slot != null) PhaseBar(state, slot) }
        }
    }
}

/**
 * One board's PHASE current, the current the limiter acts on, with its bar filled against that
 * board's own current limit ([phaseFill]). The derived supply figure stays on Setup, because it
 * over-reads under Sine and FOC.
 */
@Composable
private fun PhaseBar(state: UiState, slot: RideSlot) {
    val reading = if (state.telemetryOf(slot.address) == null) CurrentReading.Silent else state.currentOf(slot.address)
    val figure = when (reading) {
        is CurrentReading.Amps -> stringResource(R.string.ride_phase_amps, abs(reading.motor))
        CurrentReading.MotorsOff -> stringResource(R.string.ride_phase_off)
        CurrentReading.Coasting -> stringResource(R.string.ride_phase_amps, 0f)
        CurrentReading.Silent, CurrentReading.Unscaled -> stringResource(R.string.ride_phase_none)
    }
    val fill = phaseFill(reading, state.currentLimit[slot.address])
    Column {
        Row(verticalAlignment = Alignment.Bottom) {
            Text(slotLabel(slot), style = ride(12.sp), color = FgMuted)
            Spacer(Modifier.weight(1f))
            Text(
                figure,
                style = ride(15.sp, FontWeight.Bold),
                color = Fg,
                modifier = Modifier.testTag(ridePhaseTag(slot.side)),
            )
            Spacer(Modifier.width(4.dp))
            Text(
                stringResource(R.string.ride_phase_word),
                style = ride(11.sp),
                color = FgFaint,
                modifier = Modifier.padding(bottom = 1.dp),
            )
        }
        Spacer(Modifier.height(6.dp))
        Box(
            Modifier
                .fillMaxWidth()
                .height(6.dp)
                .clip(CircleShape)
                .background(BgCode),
        ) {
            if (fill != null && fill > 0f) {
                Box(
                    Modifier
                        .fillMaxWidth(fill)
                        .fillMaxHeight()
                        .clip(CircleShape)
                        .background(Primary),
                )
            }
        }
    }
}

/** The speed's own style: 88k sp bold on a line just tall enough for the figures. */
private fun speedStyle(k: Float) = ride((SPEED_SP * k).sp, FontWeight.Bold).copy(
    lineHeight = (SPEED_LINE_SP * k).sp,
    lineHeightStyle = LineHeightStyle(LineHeightStyle.Alignment.Center, LineHeightStyle.Trim.Both),
)

private const val SPEED_SP = 88f
private const val SPEED_LINE_SP = 96f

/** The slot's height, from the top of the content area. */
internal val INSTRUMENTS_HEIGHT = 276.dp

private val RING_CENTRE_Y = 118.dp
private val RING_RADIUS = 104.dp
private val RING_STROKE = 12.dp
private const val RING_START = 150f
private const val RING_SWEEP = 240f
private const val RING_GAP = 1.2f
private val SPEED_BOX = 132.dp
private val SPEED_WORDS_WIDTH = 168.dp

/** The ring's inner diameter at k = 1: twice the radius less the stroke. */
private val RING_INNER = 196.dp

private val PACK_TOP = 198.dp
/** A corner's height (label, horizon, two lines), and its bottom's distance above the slot's bottom. */
private val CORNER_HEIGHT = 86.dp
private val CORNER_BOTTOM = 40.dp
private val CORNER_WIDTH = 72.dp
private val CORNER_INSET = 20.dp
private val HORIZON_SIZE = 40.dp
private val HORIZON_DP_PER_DEGREE = 2.dp
private const val SKY_ALPHA = 0.35f
private const val LADDER_DEGREES = 10f
private const val LADDER_HALF = 0.3f
private const val LADDER_ALPHA = 0.6f
private const val SYMBOL_OUTER = 0.55f
private const val SYMBOL_INNER = 0.15f
private const val CENTI = 100f
/** The phase row's top, above the slot's bottom. */
private val PHASE_FROM_BOTTOM = 32.dp
private val PHASE_INSET = 40.dp

