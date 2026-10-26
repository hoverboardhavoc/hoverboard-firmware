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
