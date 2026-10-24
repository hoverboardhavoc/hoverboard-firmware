package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneNotice
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TelemetryUi
import java.util.Locale

/** Gain names in key-index order, as the cascade names them (`specs/rider-ui.md` section 4). */
internal val GAIN_NAMES = listOf("kp", "bk", "pr")

/**
 * The speed loop's three words in key-index order, as the loop names them
 * (`crates/control/src/outer.rs`, `OuterSpeed`). Used where a notice has to name one; the rows
 * themselves carry the longer label with the units in it.
 */
internal val SPEED_NAMES = listOf("speed_kp", "speed_ki", "speed_max")

/**
 * The line under the two chips (`specs/rider-ui.md` section 4, "What the screen calls the two
 * sets"): the board switches sets on its own, and this says so and says which one that leaves it
 * on. Where one set is all the board can reach, there are no chips and the line says that instead.
 */
@Composable
internal fun profileLine(state: TuneState, telemetry: TelemetryUi?): String =
    if (state.profileBHidden) {
        stringResource(R.string.tune_profile_waived)
    } else {
        stringResource(R.string.tune_profile_auto, liveClause(state, telemetry))
    }

/**
 * Which set the board is on NOW, said only as far as the app knows it: the pads level is not known
 * before the board's first cyclic, and a board that may be running its rider requirement waived may
 * be on the rider set whatever the pads read.
 */
@Composable
private fun liveClause(state: TuneState, telemetry: TelemetryUi?): String {
    val withRider = stringResource(R.string.tune_profile_a)
    return when {
        telemetry?.hasState != true -> stringResource(R.string.tune_profile_live_unknown)
        else -> {
            val pads = if (telemetry.riderPresent) {
                stringResource(R.string.tune_profile_live_rider, withRider)
            } else {
                stringResource(R.string.tune_profile_live_none, stringResource(R.string.tune_profile_b))
            }
            if (state.riderMaybeWaived) {
                stringResource(R.string.tune_profile_live_maybe_waived, pads, withRider)
            } else {
                pads
            }
        }
    }
}

@Composable
internal fun tuneNoticeText(n: TuneNotice, state: TuneState): String = when (n) {
    TuneNotice.SaveWhileArmed -> stringResource(R.string.tune_notice_save_armed)
    TuneNotice.BoundWhileArmed -> stringResource(R.string.tune_notice_bound_armed)
    is TuneNotice.BoundOutOfRange -> stringResource(R.string.tune_notice_bound_range, rowLabel(n.key, state))
    is TuneNotice.BoardRefused -> stringResource(
        when (n.refusal) {
            CfgRefusal.BAD -> R.string.tune_notice_refused_bad
            CfgRefusal.UNKNOWN_KEY -> R.string.tune_notice_refused_unknown
            else -> R.string.tune_notice_refused_other
        },
        rowLabel(n.key, state),
    )
    is TuneNotice.Mismatch -> stringResource(
        R.string.tune_notice_mismatch,
        rowLabel(n.key, state),
        n.wrote.toString(),
        n.echoed?.toString() ?: "?",
    )
    is TuneNotice.Unanswered -> stringResource(R.string.tune_notice_unanswered, rowLabel(n.key, state))
    is TuneNotice.Garbled -> stringResource(R.string.tune_notice_garbled, rowLabel(n.key, state))
    TuneNotice.SlotBusy -> stringResource(R.string.tune_notice_slot_busy)
    TuneNotice.NotAttached -> stringResource(R.string.tune_notice_not_attached)
}

/**
 * What the screen calls [key]: one of the four drive bounds by its own label, else a gain by
 * [gainLabel]. The two kinds of row the screen writes, so the two kinds of key its notices name.
 */
@Composable
private fun rowLabel(key: Key, state: TuneState): String =
    SetupFields.DRIVE_BOUNDS.firstOrNull { it.key == key }?.let { stringResource(it.label) }
        ?: gainLabel(key, state)

/**
 * "kp (With a rider)", "pr (No rider)": a gain key in the screen's terms, naming the set by the
 * same words its chip does, or the raw key if it names no gain. Where one set is all the board can
 * reach there are no chips and no set to disambiguate, so the gain stands alone.
 *
 * A speed-loop word is in no set at all (`specs/rider-ui.md` 3.3c), so it is named on its own.
 */
@Composable
private fun gainLabel(key: Key, state: TuneState): String {
    if (key.fieldId == Gains.CONTROL_SPEED_LOOP) return SPEED_NAMES.getOrNull(key.index) ?: key.toString()
    val set = when (key.fieldId) {
        Gains.CONTROL_GAIN_A -> R.string.tune_profile_a
        Gains.CONTROL_GAIN_B -> R.string.tune_profile_b
        else -> return key.toString()
    }
    val name = GAIN_NAMES.getOrNull(key.index) ?: return key.toString()
    return if (state.profileBHidden) name else stringResource(R.string.tune_gain_in_set, name, stringResource(set))
}

/** Milliseconds as seconds, rounded UP to a tenth so a bound is never shown shorter than it is. */
internal fun seconds(ms: Long): String {
    val tenths = (ms + MS_PER_TENTH - 1) / MS_PER_TENTH
    return "%d.%d".format(Locale.ROOT, tenths / TENTHS_PER_S, tenths % TENTHS_PER_S)
}

private const val MS_PER_TENTH = 100L
private const val TENTHS_PER_S = 10L
