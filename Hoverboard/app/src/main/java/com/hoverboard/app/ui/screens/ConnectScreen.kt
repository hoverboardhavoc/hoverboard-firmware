package com.hoverboard.app.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
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
import com.hoverboard.app.DiscoveryUiState
import com.hoverboard.app.R
import com.hoverboard.app.model.ConnectionState
import com.hoverboard.app.ui.theme.Bg
import com.hoverboard.app.ui.theme.BgElevated
import com.hoverboard.app.ui.theme.Border
import com.hoverboard.app.ui.theme.Flame2
import com.hoverboard.app.ui.theme.Neon
import com.hoverboard.app.ui.theme.StatusError
import com.hoverboard.app.ui.theme.TextMuted
import com.hoverboard.app.ui.theme.flameBrush

/**
 * Connect screen: scan for the board's module, connect, show the BLE status, and (slice 4) walk the
 * fleet over the live BLE link. The brand flame gradient marks the title + primary actions; the neon
 * cyan marks the live link + the discovered boards. The drive UI arrives in a later slice.
 */
@Composable
fun ConnectScreen(
    connectionState: ConnectionState,
    discovery: DiscoveryUiState,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    onDiscover: () -> Unit,
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

        if (connectionState == ConnectionState.CONNECTED) {
            Spacer(modifier = Modifier.height(16.dp))
            DiscoverSection(discovery = discovery, onDiscover = onDiscover)
        }
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
                testTag = "connect_button",
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

/** The Discover (L3 walk) action + the discovered-fleet panel; shown only while connected. */
@Composable
private fun DiscoverSection(discovery: DiscoveryUiState, onDiscover: () -> Unit) {
    if (discovery is DiscoveryUiState.Running) {
        OutlinedButton(
            onClick = {},
            enabled = false,
            modifier = Modifier
                .fillMaxWidth()
                .testTag("discover_button"),
        ) {
            CircularProgressIndicator(modifier = Modifier.size(18.dp), color = Flame2)
            Spacer(modifier = Modifier.size(8.dp))
            Text(stringResource(R.string.discover_running))
        }
    } else {
        FlameButton(
            text = stringResource(R.string.connect_discover),
            onClick = onDiscover,
            testTag = "discover_button",
        )
    }

    when (discovery) {
        is DiscoveryUiState.Done -> {
            Spacer(modifier = Modifier.height(16.dp))
            FleetPanel(discovery)
        }

        is DiscoveryUiState.Failed -> {
            Spacer(modifier = Modifier.height(16.dp))
            Text(
                text = stringResource(R.string.discover_failed, discovery.reason),
                style = MaterialTheme.typography.bodyMedium,
                color = StatusError,
                textAlign = TextAlign.Center,
                modifier = Modifier.testTag("discover_result"),
            )
        }

        else -> Unit
    }
}

/** The discovered fleet: the app's own (guest) address, each board + how it is reached, + the CONFIG echo. */
@Composable
private fun FleetPanel(done: DiscoveryUiState.Done) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(12.dp))
            .background(BgElevated)
            .border(1.dp, Border, RoundedCornerShape(12.dp))
            .padding(16.dp)
            .testTag("discover_result"),
    ) {
        Text(
            text = stringResource(R.string.discover_boards_title),
            style = MaterialTheme.typography.titleSmall,
            color = MaterialTheme.colorScheme.onBackground,
            fontWeight = FontWeight.Bold,
        )
        Spacer(modifier = Modifier.height(8.dp))

        // This app is the (transient) controller; show its own guest address so the who's-who is clear.
        Text(
            text = stringResource(
                R.string.discover_this_app,
                "0x${done.outcome.controllerAddr.toString(16).padStart(2, '0')}",
            ),
            style = MaterialTheme.typography.bodyMedium,
            color = TextMuted,
            modifier = Modifier
                .padding(vertical = 2.dp)
                .testTag("discover_controller"),
        )

        val boards = done.outcome.boards
        if (boards.isEmpty()) {
            Text(
                text = stringResource(R.string.discover_none),
                style = MaterialTheme.typography.bodyMedium,
                color = TextMuted,
            )
        } else {
            boards.forEachIndexed { i, addr ->
                BoardRow(addr = addr, isEntry = addr == done.outcome.entryAddr, index = i)
            }
        }

        done.outcome.configEcho?.let { echo ->
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                text = echo,
                style = MaterialTheme.typography.bodySmall,
                color = TextMuted,
            )
        }
    }
}


/** One discovered board: its address + how it is reached (entry over BLE, or downstream over UART). */
@Composable
private fun BoardRow(addr: Int, isEntry: Boolean, index: Int) {
    // The entry board is reached directly over this BLE link; the rest sit downstream of it, reached
    // through it over the inter-board UART. (No "gateway" - that term collides with the app/controller.)
    val reach = if (isEntry) {
        stringResource(R.string.discover_reach_entry)
    } else {
        stringResource(R.string.discover_reach_downstream)
    }
    Text(
        text = "0x${addr.toString(16).padStart(2, '0')}  •  $reach",
        style = MaterialTheme.typography.bodyMedium,
        color = Neon,
        modifier = Modifier
            .padding(vertical = 2.dp)
            .testTag("discover_board_$index"),
    )
}

/** The primary action, painted with the brand flame gradient (Material 3 Button is solid-only). */
@Composable
private fun FlameButton(text: String, onClick: () -> Unit, testTag: String) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(52.dp)
            .clip(RoundedCornerShape(26.dp))
            .background(flameBrush())
            .clickable(onClick = onClick)
            .testTag(testTag),
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
