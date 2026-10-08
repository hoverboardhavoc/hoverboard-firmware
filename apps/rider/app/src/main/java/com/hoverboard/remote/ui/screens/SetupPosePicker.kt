package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.selection.selectable
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.imu.Orientation.Face
import com.hoverboard.protocol.imu.Orientation.Heading
import com.hoverboard.protocol.imu.Orientation.Pose
import com.hoverboard.remote.R
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary
import com.hoverboard.remote.ui.theme.ThrottleTrack
import com.hoverboard.remote.ui.theme.ZeroLine

/**
 * The two-step pose picker (`specs/rider-ui.md` 3.4): which face of the board points up, then what
 * points forward. A face only chooses which four headings are offered; picking a heading stages the
 * whole frame through [onPick]. [intended] is the pose the basket and the store make together
 * (`SetupState.intendedPose`), highlighted when it is one of the 24; unset fields read as the stock
 * pose.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun PosePicker(intended: Pose?, editable: Boolean, onPick: (Orientation.Frame) -> Unit) {
    var chosenFace by rememberSaveable { mutableStateOf<Face?>(null) }
    val face = chosenFace ?: intended?.face
    PanelTitle(R.string.setup_pose_face_title)
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        for (f in Face.entries) {
            // Each face drawn in the intended pose when that is the face, else in its first heading.
            val shown = intended?.takeIf { it.face == f } ?: Pose(f, Orientation.headingsFor(f).first())
            PoseGlyph(shown, faceLabel(f), selected = f == face, enabled = editable) { chosenFace = f }
        }
    }
    if (face == null) return
    PanelTitle(R.string.setup_pose_heading_title)
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        for (h in Orientation.headingsFor(face)) {
            val pose = Pose(face, h)
            PoseGlyph(pose, headingLabel(h), selected = pose == intended, enabled = editable) {
                onPick(Orientation.frameOf(pose))
            }
        }
    }
    Caption(R.string.setup_pose_drawing)
}

/** A pose's name: its face and its heading. */
@Composable
internal fun poseName(pose: Pose): String = stringResource(
    R.string.setup_pose_name,
    stringResource(faceLabel(pose.face)),
    stringResource(headingLabel(pose.heading)),
)

@StringRes
private fun faceLabel(face: Face): Int = when (face) {
    Face.COMPONENT_UP -> R.string.setup_face_component_up
    Face.COMPONENT_DOWN -> R.string.setup_face_component_down
    Face.STOCK_FORWARD_EDGE_DOWN -> R.string.setup_face_forward_edge_down
    Face.STOCK_REAR_EDGE_DOWN -> R.string.setup_face_rear_edge_down
    Face.STOCK_LEFT_EDGE_DOWN -> R.string.setup_face_left_edge_down
    Face.STOCK_RIGHT_EDGE_DOWN -> R.string.setup_face_right_edge_down
}

@StringRes
private fun headingLabel(heading: Heading): Int = when (heading) {
    Heading.STOCK_FORWARD -> R.string.setup_heading_forward
    Heading.STOCK_REAR -> R.string.setup_heading_rear
    Heading.STOCK_LEFT -> R.string.setup_heading_left
    Heading.STOCK_RIGHT -> R.string.setup_heading_right
    Heading.COMPONENT_SIDE -> R.string.setup_heading_component
    Heading.BACK_SIDE -> R.string.setup_heading_back
}

/** One selectable board drawing with its label under it. */
@Composable
private fun PoseGlyph(pose: Pose, @StringRes label: Int, selected: Boolean, enabled: Boolean, onClick: () -> Unit) {
    val shape = RoundedCornerShape(6.dp)
    Column(
        modifier = Modifier
            .width(GLYPH_WIDTH)
            .border(if (selected) 2.dp else 1.dp, if (selected) AccentGreen else ZeroLine, shape)
            .selectable(selected = selected, enabled = enabled, role = Role.RadioButton, onClick = onClick)
            .padding(4.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Canvas(Modifier.size(GLYPH_DRAWING)) { drawPose(pose) }
        Text(
            stringResource(label),
            style = MaterialTheme.typography.labelSmall,
            color = if (selected) TextPrimary else TextSecondary,
            textAlign = TextAlign.Center,
        )
    }
}

/**
 * The board as a box in [pose], in an oblique projection of the BODY frame: forward to the right, up
 * at the top, left receding up and to the right. A box face is drawn when its outward normal, taken
 * through `Orientation.boardToBody`, points toward the viewer ([TOWARD_VIEWER]); the component side
 * (board `+Z`) is shaded, and an arrow leaves the stock-forward edge (board `+X`).
 */
private fun DrawScope.drawPose(pose: Pose) {
    val r = Orientation.boardToBody(pose)
    fun toBody(b: List<Float>): List<Float> =
        r.map { row -> row.indices.sumOf { (row[it] * b[it]).toDouble() }.toFloat() }
    val scale = size.minDimension / GLYPH_SPAN
    fun project(b: List<Float>): Offset {
        val (x, y, z) = toBody(b)
        return Offset(center.x + (x + OBLIQUE_X * y) * scale, center.y - (z + OBLIQUE_Z * y) * scale)
    }
    val edge = Stroke(width = 1.dp.toPx())
    for (axis in BOX.indices) for (sign in listOf(-1, 1)) {
        val normal = List(BOX.size) { if (it == axis) sign.toFloat() else 0f }
        val facing = toBody(normal).zip(TOWARD_VIEWER).sumOf { (a, b) -> (a * b).toDouble() }
        if (facing <= 0.0) continue
        val (u, v) = BOX.indices.filter { it != axis }
        val path = Path()
        CORNERS.forEachIndexed { i, (su, sv) ->
            val corner = MutableList(BOX.size) { 0f }
            corner[axis] = sign * BOX[axis]
            corner[u] = su * BOX[u]
            corner[v] = sv * BOX[v]
            val p = project(corner)
            if (i == 0) path.moveTo(p.x, p.y) else path.lineTo(p.x, p.y)
        }
        path.close()
        val component = axis == 2 && sign == 1
        drawPath(path, if (component) AccentGreen.copy(alpha = COMPONENT_ALPHA) else ThrottleTrack)
        drawPath(path, TextSecondary, style = edge)
    }
    val tail = project(listOf(BOX[0], 0f, 0f))
    val tip = project(listOf(BOX[0] + ARROW, 0f, 0f))
    val wingA = project(listOf(BOX[0] + ARROW - HEAD, HEAD, 0f))
    val wingB = project(listOf(BOX[0] + ARROW - HEAD, -HEAD, 0f))
    val arrow = 2.dp.toPx()
    drawLine(AccentYellow, tail, tip, arrow)
    drawLine(AccentYellow, tip, wingA, arrow)
    drawLine(AccentYellow, tip, wingB, arrow)
}

/**
 * The live board model (`specs/rider-ui.md` 3.4): a side view (forward to the right) tilted by the
 * reported pitch and a view from behind tilted by the reported roll, both of the frame the board
 * booted with. The firmware's signs (`specs/attitude.md`, "Outputs": lean forward is negative pitch,
 * roll right is positive) give: a forward lean drops the side view's right end (a clockwise turn by
 * `-pitch`), and a roll to the right drops the rear view's right end (clockwise by `roll`).
 */
@Composable
internal fun BoardModel(telemetry: TelemetryUi?) {
    PanelTitle(R.string.setup_model_title)
    Caption(R.string.setup_model_body)
    val live = telemetry?.takeIf { it.hasState }
    Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        TiltView(R.string.setup_model_side, clockwise = -(live?.pitchDegrees ?: 0f), length = SIDE_LENGTH, nose = true)
        TiltView(R.string.setup_model_behind, clockwise = live?.rollDegrees ?: 0f, length = REAR_LENGTH, nose = false)
    }
    AttitudeNow(telemetry)
}

/** One projection: a dashed level line and the board as a slab turned [clockwise] degrees. */
@Composable
private fun TiltView(@StringRes label: Int, clockwise: Float, length: Float, nose: Boolean) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Canvas(Modifier.size(MODEL_WIDTH, MODEL_HEIGHT)) {
            val half = size.width * length / 2
            drawLine(
                ZeroLine,
                Offset(0f, center.y),
                Offset(size.width, center.y),
                1.dp.toPx(),
                pathEffect = PathEffect.dashPathEffect(floatArrayOf(4.dp.toPx(), 4.dp.toPx())),
            )
            rotate(clockwise) {
                val thick = SLAB.toPx()
                drawRect(AccentGreen, Offset(center.x - half, center.y - thick / 2), Size(half * 2, thick))
                if (nose) {
                    val n = NOSE.toPx()
                    val w = 2.dp.toPx()
                    val tip = Offset(center.x + half + n, center.y)
                    drawLine(AccentYellow, Offset(center.x + half, center.y), tip, w)
                    drawLine(AccentYellow, tip, Offset(tip.x - n / 2, center.y - n / 2), w)
                    drawLine(AccentYellow, tip, Offset(tip.x - n / 2, center.y + n / 2), w)
                }
            }
        }
        Text(stringResource(label), style = MaterialTheme.typography.labelSmall, color = TextSecondary)
    }
}

private val GLYPH_WIDTH = 72.dp
private val GLYPH_DRAWING = 56.dp
private val MODEL_WIDTH = 140.dp
private val MODEL_HEIGHT = 72.dp
private val SLAB = 8.dp
private val NOSE = 10.dp
private const val SIDE_LENGTH = 0.7f
private const val REAR_LENGTH = 0.45f

/** Board half extents in board axes (`X` stock-forward, `Y` stock-left, `Z` component side). */
private val BOX = listOf(0.5f, 0.32f, 0.06f)
private const val ARROW = 0.3f
private const val HEAD = 0.1f
private const val COMPONENT_ALPHA = 0.6f

/** The drawing's extent in board units across the glyph's smaller side. */
private const val GLYPH_SPAN = 2.1f

/** The oblique projection: body `Y` (left) recedes this far right and up per unit. */
private const val OBLIQUE_X = 0.45f
private const val OBLIQUE_Z = 0.3f

/** Body-frame direction toward the viewer: the reverse of the projection's receding direction. */
private val TOWARD_VIEWER = listOf(OBLIQUE_X, -1f, OBLIQUE_Z)

/** A face's four corners, in order around it, as signs on its two in-plane axes. */
private val CORNERS = listOf(-1 to -1, 1 to -1, 1 to 1, -1 to 1)

