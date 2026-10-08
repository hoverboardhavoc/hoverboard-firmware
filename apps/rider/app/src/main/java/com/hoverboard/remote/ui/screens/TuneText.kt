package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.R
import com.hoverboard.remote.TuneNotice
import com.hoverboard.remote.TuneState
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.TelemetryUi
import java.util.Locale

/** Gain names in key-index order, as the cascade names them (`specs/rider-ui.md` section 4). */
internal val GAIN_NAMES = listOf("kp", "bk", "pr")

@Composable
internal fun riderLine(state: TuneState, telemetry: TelemetryUi?): String = when {
    state.profileBHidden -> stringResource(R.string.tune_rider_waived)
    state.target == Node.SLAVE -> stringResource(R.string.tune_rider_slave)
    telemetry?.hasState != true -> stringResource(R.string.tune_rider_unknown)
    telemetry.riderPresent -> stringResource(R.string.tune_rider_on)
    else -> stringResource(R.string.tune_rider_off)
}

@Composable
internal fun tuneNoticeText(n: TuneNotice, state: TuneState): String = when (n) {
    TuneNotice.SaveWhileArmed -> stringResource(R.string.tune_notice_save_armed)
    is TuneNotice.BoardRefused -> stringResource(
        when (n.refusal) {
            CfgRefusal.BAD -> R.string.tune_notice_refused_bad
            CfgRefusal.UNKNOWN_KEY -> R.string.tune_notice_refused_unknown
            else -> R.string.tune_notice_refused_other
        },
        gainLabel(n.key, state),
    )
    is TuneNotice.Mismatch -> stringResource(
        R.string.tune_notice_mismatch,
        gainLabel(n.key, state),
        n.wrote.toString(),
        n.echoed?.toString() ?: "?",
    )
    is TuneNotice.Unanswered -> stringResource(R.string.tune_notice_unanswered, gainLabel(n.key, state))
    is TuneNotice.Garbled -> stringResource(R.string.tune_notice_garbled, gainLabel(n.key, state))
    TuneNotice.NotAttached -> stringResource(R.string.tune_notice_not_attached)
}

/** "A kp", "B pr": a gain key in the screen's terms, or the raw key if it names no gain. */
private fun gainLabel(key: Key, state: TuneState): String {
    val profile = when (key.fieldId) {
        Gains.CONTROL_GAIN_A -> "A"
        Gains.CONTROL_GAIN_B -> "B"
        else -> return key.toString()
    }
    val name = GAIN_NAMES.getOrNull(key.index) ?: return key.toString()
    return if (state.profileBHidden) name else "$profile $name"
}

/** Milliseconds as seconds, rounded UP to a tenth so a bound is never shown shorter than it is. */
internal fun seconds(ms: Long): String {
    val tenths = (ms + MS_PER_TENTH - 1) / MS_PER_TENTH
    return "%d.%d".format(Locale.ROOT, tenths / TENTHS_PER_S, tenths % TENTHS_PER_S)
}

private const val MS_PER_TENTH = 100L
private const val TENTHS_PER_S = 10L
