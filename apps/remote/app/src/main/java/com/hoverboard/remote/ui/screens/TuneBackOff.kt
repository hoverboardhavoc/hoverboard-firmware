package com.hoverboard.remote.ui.screens

import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import com.hoverboard.remote.R

/** Test tag on gain [index]'s back-off button (gp and gr only, [GAIN_BACK_OFF_PERCENTS]). */
fun tuneBackOffTag(index: Int): String = "tune_back_off_$index"

/**
 * The tuning guide's back-off step for gains that have one ([GAIN_BACK_OFF_PERCENTS]): sends
 * [gainBackedOff] of the board's read-back [staged] value, not of a pending one, so a press keeps the
 * stated share of what the board holds. Off while the row is not tunable or the value is unread.
 */
@Composable
internal fun BackOff(index: Int, staged: Int?, max: Int, enabled: Boolean, onSend: (Int) -> Unit) {
    val percent = GAIN_BACK_OFF_PERCENTS[index] ?: return
    OutlinedButton(
        onClick = { staged?.let { onSend(gainBackedOff(it, percent, max)) } },
        enabled = enabled,
        modifier = Modifier.testTag(tuneBackOffTag(index)),
    ) { Text(stringResource(R.string.tune_back_off, percent)) }
}
