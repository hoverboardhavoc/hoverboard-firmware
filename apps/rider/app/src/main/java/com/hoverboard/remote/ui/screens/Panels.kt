package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.ui.theme.BgElevated
import com.hoverboard.remote.ui.theme.Border
import com.hoverboard.remote.ui.theme.Fg
import com.hoverboard.remote.ui.theme.FgMuted

/**
 * The pieces every grouped screen is built from: a titled section, a panel, a panel's heading and
 * a caption under a control.
 *
 * They started in the Setup screen's row file and moved here when the Tune screen's drive bounds
 * became the fourth caller (`specs/rider-ui.md` 3.3b): they carry no screen's logic, only the
 * house style for a block of rows, and a shared thing belongs where every caller can see it.
 */
@Composable
internal fun Section(title: String, content: @Composable ColumnScope.() -> Unit) {
    Spacer(modifier = Modifier.height(4.dp))
    Text(title, style = MaterialTheme.typography.titleLarge, color = Fg)
    Column(verticalArrangement = Arrangement.spacedBy(12.dp), content = content)
}

/** One row's surface: the rounded panel every field sits on. */
@Composable
internal fun Panel(modifier: Modifier = Modifier, content: @Composable ColumnScope.() -> Unit) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .background(BgElevated, RoundedCornerShape(8.dp))
            .border(1.dp, Border, RoundedCornerShape(8.dp))

            .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp),
        content = content,
    )
}

/** A panel's heading. */
@Composable
internal fun PanelTitle(@StringRes text: Int) {
    Text(stringResource(text), style = MaterialTheme.typography.titleSmall, color = Fg)
}

/** A standing explanation under a heading or a control. */
@Composable
internal fun Caption(@StringRes text: Int, args: List<Any> = emptyList()) {
    val body = if (args.isEmpty()) stringResource(text) else stringResource(text, *args.toTypedArray())
    Text(body, style = MaterialTheme.typography.bodySmall, color = FgMuted)
}
