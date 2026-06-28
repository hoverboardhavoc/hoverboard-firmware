package com.hoverboard.app.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.app.R
import com.hoverboard.app.model.ConnectionState
import com.hoverboard.app.ui.theme.Bg
import com.hoverboard.app.ui.theme.Flame2
import com.hoverboard.app.ui.theme.Neon
import com.hoverboard.app.ui.theme.StatusError
import com.hoverboard.app.ui.theme.flameBrush

/**
 * Connect screen (slice 1): scan for the board's module, connect, and show the BLE status. The
 * brand flame gradient marks the title and the primary action; the neon cyan marks the live link.
 * The drive UI arrives in a later slice.
 */
@Composable
fun ConnectScreen(
    connectionState: ConnectionState,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(32.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(
            text = stringResource(R.string.connect_title),
            style = MaterialTheme.typography.displaySmall.copy(
                brush = flameBrush(),
                fontWeight = FontWeight.Bold,
            ),
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(8.dp))

        Text(
            text = stringResource(R.string.connect_subtitle),
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(48.dp))

        StatusIndicator(connectionState)

        Spacer(modifier = Modifier.height(16.dp))

        Text(
            text = stringResource(statusLabel(connectionState)),
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
            modifier = Modifier.testTag("status_text"),
        )

        Spacer(modifier = Modifier.height(48.dp))

        ConnectAction(
            connectionState = connectionState,
            onConnect = onConnect,
            onDisconnect = onDisconnect,
        )
    }
}

@Composable
private fun StatusIndicator(connectionState: ConnectionState) {
    val busy = connectionState == ConnectionState.SCANNING ||
        connectionState == ConnectionState.CONNECTING
    if (busy) {
        CircularProgressIndicator(
            modifier = Modifier.size(48.dp),
            color = Flame2,
        )
    } else {
        val color = when (connectionState) {
            ConnectionState.CONNECTED -> Neon
            ConnectionState.ERROR -> StatusError
            else -> MaterialTheme.colorScheme.outline
        }
        Surface(
            modifier = Modifier.size(48.dp),
            shape = CircleShape,
            color = color,
            content = {},
        )
    }
}

@Composable
private fun ConnectAction(
    connectionState: ConnectionState,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
) {
    when (connectionState) {
        ConnectionState.DISCONNECTED, ConnectionState.ERROR -> {
            FlameButton(
                text = stringResource(R.string.connect_scan),
                onClick = onConnect,
            )
        }

        ConnectionState.SCANNING, ConnectionState.CONNECTING -> {
            OutlinedButton(
                onClick = onDisconnect,
                modifier = Modifier
                    .fillMaxWidth()
                    .testTag("connect_button"),
            ) {
                Text(stringResource(R.string.connect_stop_scan))
            }
        }

        ConnectionState.CONNECTED -> {
            OutlinedButton(
                onClick = onDisconnect,
                modifier = Modifier
                    .fillMaxWidth()
                    .testTag("connect_button"),
            ) {
                Text(stringResource(R.string.connect_disconnect), color = Neon)
            }
        }
    }
}

/** The primary action, painted with the brand flame gradient (Material 3 Button is solid-only). */
@Composable
private fun FlameButton(text: String, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(52.dp)
            .clip(RoundedCornerShape(26.dp))
            .background(flameBrush())
            .clickable(onClick = onClick)
            .testTag("connect_button"),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = text,
            color = Bg,
            fontWeight = FontWeight.Bold,
            style = MaterialTheme.typography.titleMedium,
        )
    }
}

private fun statusLabel(state: ConnectionState): Int = when (state) {
    ConnectionState.DISCONNECTED -> R.string.connect_status_idle
    ConnectionState.SCANNING -> R.string.connect_status_scanning
    ConnectionState.CONNECTING -> R.string.connect_status_connecting
    ConnectionState.CONNECTED -> R.string.connect_status_connected
    ConnectionState.ERROR -> R.string.connect_status_error
}
