package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneNotice
import com.hoverboard.remote.model.SetupFields
import java.util.Locale

/**
 * The four gains' names in key-index order, as the law names them (`specs/state-feedback.md`
 * section 2). Used where a notice has to name one; the rows carry the unit beside the name.
 */
internal val GAIN_NAMES = listOf("gp", "gr", "gx", "gv")

/**
 * What each gain multiplies, in the unit of `specs/state-feedback.md` section 2's table, in
 * key-index order.
 */
internal val GAIN_UNITS = listOf(
    R.string.tune_gain_gp_unit,
    R.string.tune_gain_gr_unit,
    R.string.tune_gain_gx_unit,
    R.string.tune_gain_gv_unit,
)

/**
 * The top of each gain's SLIDER, in key-index order: the span the useful values sit in (about gp
 * 1000 to 6000, gr 50 to 4000 with the rover's start at 4002, gx 0 to 400, gv 0 to 2000), so a
 * pixel of thumb travel moves a gain a few counts rather than the twenty a track over the board's
 * whole `CONTROL_GAIN_MAX` gives. An app choice and not a firmware fact, which is why it is here
 * and not in [Gains]. The slider never reaches past the board's maximum ([gainSliderTop]); the
 * typed entry and the step buttons reach the whole of it.
 */
internal val GAIN_SLIDER_SPANS = listOf(8000, 5000, 500, 3000)

/** What one press of a gain row's minus or plus button moves the gain by, in key-index order. */
internal val GAIN_STEPS = listOf(50, 10, 5, 10)

/**
 * The share of the onset value each gain's back-off button keeps, in percent, in key-index order.
 * The tuning guide says to raise gp until a fast shake starts and keep about 75% of that, and to
 * raise gr until a buzz starts and keep about 65% (`docs/state-feedback-tuning.md`, Order and increments). It gives gx
 * and gv no onset rule, so they have no button.
 */
internal val GAIN_BACK_OFF_PERCENTS: List<Int?> = listOf(75, 65, null, null)

/**
 * [percent] of [staged], rounded to the nearest count (a half rounds up) and held inside the range a
 * board whose maximum is [max] accepts: what a back-off button sends.
 */
internal fun gainBackedOff(staged: Int, percent: Int, max: Int): Int =
    ((staged.toLong() * percent + PERCENT / 2) / PERCENT).toInt().coerceIn(Gains.MIN, maxOf(max, Gains.MIN))

private const val PERCENT = 100L

/** The top of gain [index]'s slider on a board whose maximum there is [boardMax]. */
internal fun gainSliderTop(index: Int, boardMax: Int): Int = minOf(GAIN_SLIDER_SPANS[index], boardMax)

/**
 * [value] moved by [delta] and held inside the range a board whose maximum is [max] accepts, so a
 * step button never asks for a value the board would refuse.
 */
internal fun gainStepped(value: Int, delta: Int, max: Int): Int =
    (value + delta).coerceIn(Gains.MIN, maxOf(max, Gains.MIN))

/** The reference row's two words in key-index order (`specs/state-feedback.md` section 4). */
internal val REFERENCE_NAMES = listOf("speed_max", "speed_slew")

@Composable
internal fun tuneNoticeText(n: TuneNotice): String = when (n) {
    TuneNotice.SaveWhileArmed -> stringResource(R.string.tune_notice_save_armed)
    TuneNotice.BoundWhileArmed -> stringResource(R.string.tune_notice_bound_armed)
    is TuneNotice.BoundOutOfRange -> stringResource(R.string.tune_notice_bound_range, rowLabel(n.key))
    is TuneNotice.BoardRefused -> stringResource(
        when (n.refusal) {
            CfgRefusal.BAD -> R.string.tune_notice_refused_bad
            CfgRefusal.UNKNOWN_KEY -> R.string.tune_notice_refused_unknown
            else -> R.string.tune_notice_refused_other
        },
        rowLabel(n.key),
    )
    is TuneNotice.Mismatch -> stringResource(
        R.string.tune_notice_mismatch,
        rowLabel(n.key),
        n.wrote.toString(),
        n.echoed?.toString() ?: "?",
    )
    is TuneNotice.Unanswered -> stringResource(R.string.tune_notice_unanswered, rowLabel(n.key))
    is TuneNotice.Garbled -> stringResource(R.string.tune_notice_garbled, rowLabel(n.key))
    TuneNotice.SlotBusy -> stringResource(R.string.tune_notice_slot_busy)
    TuneNotice.NotAttached -> stringResource(R.string.tune_notice_not_attached)
}

/**
 * What the screen calls [key]: a drive bound by its own label, a gain or a reference word by its
 * name, or the raw key if it names none of them.
 */
@Composable
private fun rowLabel(key: Key): String =
    SetupFields.DRIVE_BOUNDS.firstOrNull { it.key == key }?.let { stringResource(it.label) }
        ?: when (key.fieldId) {
            Gains.CONTROL_SF_GAINS -> GAIN_NAMES.getOrNull(key.index)
            Gains.CONTROL_REFERENCE -> REFERENCE_NAMES.getOrNull(key.index)
            else -> null
        }
        ?: key.toString()

/** Milliseconds as seconds, rounded UP to a tenth so a bound is never shown shorter than it is. */
internal fun seconds(ms: Long): String {
    val tenths = (ms + MS_PER_TENTH - 1) / MS_PER_TENTH
    return "%d.%d".format(Locale.ROOT, tenths / TENTHS_PER_S, tenths % TENTHS_PER_S)
}

private const val MS_PER_TENTH = 100L
private const val TENTHS_PER_S = 10L
