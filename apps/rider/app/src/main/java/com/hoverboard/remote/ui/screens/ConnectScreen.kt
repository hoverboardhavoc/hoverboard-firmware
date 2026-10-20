package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.ble.ScanCandidate
import com.hoverboard.remote.ble.SelectedBoard
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow
import com.hoverboard.remote.ui.theme.PanelSurface
import com.hoverboard.remote.ui.theme.TextPrimary
import com.hoverboard.remote.ui.theme.TextSecondary

/**
 * Connect screen (`specs/rider-ui.md` 3.0): pick the board out of what is nearby, connect, show
 * status.  The deadman control lives on [ControlScreen]; the caller navigates there once connected.
 *
 * The board is PICKED rather than typed. A scan lists what is advertising, each row carrying the
 * name, the MAC and the signal strength, and tapping a row pins that address and connects to it.
 * The pick is by address because the address is the board's identity: it survives a rename, and it
 * tells two identically named masters apart, neither of which a name can do.
 *
 * The name box stays, below the list, as the second way in. It is for the board that will not turn
 * up in a scan at all, which is the case no picker can serve; typing in it drops the pinned board
 * ([com.hoverboard.remote.ble.LinkSettings.setDeviceName]), so the two paths cannot both be in
 * force. Both are only live while the link is idle: changing the target mid-scan would apply to
 * the next attempt, not the running one, which reads as the app ignoring the change.
 *
 * The whole column scrolls rather than hosting the rows in a `LazyColumn`, which cannot be nested
 * in a scrolling parent. A scan in a room sees tens of devices, not thousands, so composing every
 * row costs nothing worth a second scroll container.
 */
@Composable
fun ConnectScreen(
    connectionState: ConnectionState,
    deviceName: String,
    selectedBoard: SelectedBoard?,
    findingBoards: Boolean,
    boardsSeen: List<ScanCandidate>,
    onFindBoards: () -> Unit,
    onStopFindingBoards: () -> Unit,
    onSelectBoard: (ScanCandidate) -> Unit,
    onDeviceNameChange: (String) -> Unit,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val idle = connectionState == ConnectionState.DISCONNECTED ||
        connectionState == ConnectionState.ERROR ||
        connectionState == ConnectionState.ATTACH_FAILED
    // What the status line and the connect button call the board: the pinned board's label, its
    // address when it is advertising no name, and the typed name until one has been picked.
    val targetLabel = selectedBoard?.let { it.name ?: it.address } ?: deviceName

    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 24.dp, vertical = 32.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(
            text = stringResource(R.string.connect_title),
            style = MaterialTheme.typography.headlineMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(32.dp))

        StatusIndicator(connectionState)

        Spacer(modifier = Modifier.height(16.dp))

        Text(
            text = stringResource(statusLabel(connectionState), targetLabel),
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(24.dp))

        ConnectActions(
            connectionState = connectionState,
            targetLabel = targetLabel,
            pinned = selectedBoard != null,
            onConnect = onConnect,
            onDisconnect = onDisconnect,
        )

        if (idle) {
            Spacer(modifier = Modifier.height(24.dp))
            BoardPicker(
                finding = findingBoards,
                seen = boardsSeen,
                selectedAddress = selectedBoard?.address,
                onFindBoards = onFindBoards,
                onStopFindingBoards = onStopFindingBoards,
                onSelectBoard = onSelectBoard,
            )
            Spacer(modifier = Modifier.height(24.dp))
            Text(
                text = stringResource(R.string.connect_by_name_caption),
                style = MaterialTheme.typography.bodySmall,
                color = TextSecondary,
                textAlign = TextAlign.Center,
            )
            Spacer(modifier = Modifier.height(8.dp))
            DeviceNameField(deviceName = deviceName, onDeviceNameChange = onDeviceNameChange)
        }
    }
}

/**
 * The scan and its results: a button to start looking, the rows it turns up, and a button to stop.
 *
 * Rows stay tappable after the scan is stopped. The addresses do not expire with the scan, and a
 * user who saw their board and pressed stop has made their pick already.
 */
@Composable
private fun BoardPicker(
    finding: Boolean,
    seen: List<ScanCandidate>,
    selectedAddress: String?,
    onFindBoards: () -> Unit,
    onStopFindingBoards: () -> Unit,
    onSelectBoard: (ScanCandidate) -> Unit,
) {
    if (finding) {
        OutlinedButton(onClick = onStopFindingBoards, modifier = Modifier.fillMaxWidth()) {
            Text(stringResource(R.string.connect_stop_find))
        }
    } else {
        OutlinedButton(onClick = onFindBoards, modifier = Modifier.fillMaxWidth()) {
            Text(stringResource(R.string.connect_find_boards))
        }
    }

    Spacer(modifier = Modifier.height(12.dp))

    if (seen.isEmpty()) {
        Text(
            text = stringResource(
                if (finding) R.string.connect_finding else R.string.connect_nothing_seen,
            ),
            style = MaterialTheme.typography.bodyMedium,
            color = TextSecondary,
            textAlign = TextAlign.Center,
        )
        return
    }

    for (board in seen) {
        BoardRow(
            board = board,
            picked = selectedAddress != null && board.address.equals(selectedAddress, ignoreCase = true),
            onClick = { onSelectBoard(board) },
        )
        Spacer(modifier = Modifier.height(8.dp))
    }
}

/**
 * One board the scan heard: what it calls itself, its address, and how strong it is.
 *
 * The address is always shown, never only on the nameless rows. It is the thing the app actually
 * stores, and it is the only way to tell two boards apart when both answer to "Hoverboard".
 */
@Composable
private fun BoardRow(board: ScanCandidate, picked: Boolean, onClick: () -> Unit) {
    Surface(
        modifier = Modifier
            .fillMaxWidth()
            .clickable(onClick = onClick),
        shape = RoundedCornerShape(8.dp),
        color = PanelSurface,
        border = if (picked) BorderStroke(1.dp, AccentGreen) else null,
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 14.dp, vertical = 12.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = board.displayName ?: stringResource(R.string.connect_unnamed_board),
                    style = MaterialTheme.typography.titleMedium,
                    color = TextPrimary,
                )
                Text(
                    text = board.address,
                    style = MaterialTheme.typography.bodySmall,
                    color = TextSecondary,
                )
            }
            if (picked) {
                Text(
                    text = stringResource(R.string.connect_board_picked),
                    style = MaterialTheme.typography.labelMedium,
                    color = AccentGreen,
                )
            }
            // Only when the advertisement carried it: a scan result with no data has no strength
            // to report, and a made-up zero would read as a board right next to the phone.
            board.rssi?.let { rssi ->
                Text(
                    text = stringResource(R.string.connect_rssi, rssi),
                    style = MaterialTheme.typography.bodySmall,
                    color = TextSecondary,
                )
            }
        }
    }
}

/**
 * The advertised name to scan for, when a board cannot be picked off the list.
 *
 * The field keeps its own text so a half-typed name (including an empty box mid-edit) renders as
 * typed, while every keystroke is handed upward; the settings store drops blank values, so clearing
 * the box never persists a name that would match nothing. [deviceName] is the key on the remembered
 * text so a change from outside (a fresh read of the store) replaces what is shown.
 */
@Composable
private fun DeviceNameField(deviceName: String, onDeviceNameChange: (String) -> Unit) {
    var text by remember(deviceName) { mutableStateOf(deviceName) }
    OutlinedTextField(
        value = text,
        onValueChange = {
            text = it
            onDeviceNameChange(it)
        },
        label = { Text(stringResource(R.string.connect_device_name_label)) },
        singleLine = true,
        modifier = Modifier.fillMaxWidth(),
    )
}

@Composable
private fun StatusIndicator(connectionState: ConnectionState) {
    val busy = connectionState == ConnectionState.SCANNING ||
        connectionState == ConnectionState.CONNECTING ||
        connectionState == ConnectionState.ATTACHING
    if (busy) {
        CircularProgressIndicator(
            modifier = Modifier.size(48.dp),
            color = AccentYellow,
        )
    } else {
        val color = when (connectionState) {
            ConnectionState.CONNECTED -> AccentGreen
            ConnectionState.ERROR, ConnectionState.ATTACH_FAILED -> AccentRed
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
private fun ConnectActions(
    connectionState: ConnectionState,
    targetLabel: String,
    pinned: Boolean,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
) {
    when (connectionState) {
        ConnectionState.DISCONNECTED, ConnectionState.ERROR, ConnectionState.ATTACH_FAILED -> {
            Button(
                onClick = onConnect,
                modifier = Modifier.fillMaxWidth(),
            ) {
                // A pinned board is reconnected to, by address; an unpinned one is still hunted
                // for by name, and the button says which of the two is about to happen.
                val label = if (pinned) R.string.connect_connect_to else R.string.connect_scan
                Text(stringResource(label, targetLabel))
            }
        }

        ConnectionState.SCANNING, ConnectionState.CONNECTING, ConnectionState.ATTACHING -> {
            OutlinedButton(
                onClick = onDisconnect,
                modifier = Modifier.fillMaxWidth(),
            ) {
                Text(stringResource(R.string.connect_stop_scan))
            }
        }

        ConnectionState.CONNECTED -> {
            OutlinedButton(
                onClick = onDisconnect,
                modifier = Modifier.fillMaxWidth(),
            ) {
                Text(stringResource(R.string.connect_disconnect))
            }
        }
    }
}

private fun statusLabel(state: ConnectionState): Int = when (state) {
    ConnectionState.DISCONNECTED -> R.string.connect_status_idle
    ConnectionState.SCANNING -> R.string.connect_status_scanning
    ConnectionState.CONNECTING -> R.string.connect_status_connecting
    ConnectionState.ATTACHING -> R.string.connect_status_attaching
    ConnectionState.CONNECTED -> R.string.connect_status_connected
    ConnectionState.ATTACH_FAILED -> R.string.connect_status_attach_failed
    ConnectionState.ERROR -> R.string.connect_status_error
}
