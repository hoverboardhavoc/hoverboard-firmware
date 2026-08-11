package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
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
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.theme.AccentGreen
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.AccentYellow

/**
 * Connect screen (SPEC §8.1): scan for the configured board, connect, show status.
 * The deadman control lives on [ControlScreen]; the caller navigates there once connected.
 *
 * The board name is editable here rather than behind a settings screen because it is the one setting
 * that decides whether the scan can find anything at all, so it belongs next to the button that runs
 * the scan. It is only editable while idle: changing the target mid-scan would apply to the next
 * attempt, not the running one, which reads as the app ignoring the change.
 */
@Composable
fun ConnectScreen(
    connectionState: ConnectionState,
    deviceName: String,
    onDeviceNameChange: (String) -> Unit,
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
            style = MaterialTheme.typography.headlineMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(40.dp))

        StatusIndicator(connectionState)

        Spacer(modifier = Modifier.height(16.dp))

        Text(
            text = stringResource(statusLabel(connectionState), deviceName),
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
        )

        Spacer(modifier = Modifier.height(32.dp))

        DeviceNameField(
            deviceName = deviceName,
            onDeviceNameChange = onDeviceNameChange,
            enabled = connectionState == ConnectionState.DISCONNECTED ||
                connectionState == ConnectionState.ERROR ||
                connectionState == ConnectionState.ATTACH_FAILED,
        )

        Spacer(modifier = Modifier.height(16.dp))

        ConnectActions(
            connectionState = connectionState,
            deviceName = deviceName,
            onConnect = onConnect,
            onDisconnect = onDisconnect,
        )
    }
}

/**
 * The advertised name to scan for.
 *
 * The field keeps its own text so a half-typed name (including an empty box mid-edit) renders as
 * typed, while every keystroke is handed upward; the settings store drops blank values, so clearing
 * the box never persists a name that would match nothing. [deviceName] is the key on the remembered
 * text so a change from outside (a fresh read of the store) replaces what is shown.
 */
@Composable
private fun DeviceNameField(
    deviceName: String,
    onDeviceNameChange: (String) -> Unit,
    enabled: Boolean,
) {
    var text by remember(deviceName) { mutableStateOf(deviceName) }
    OutlinedTextField(
        value = text,
        onValueChange = {
            text = it
            onDeviceNameChange(it)
        },
        label = { Text(stringResource(R.string.connect_device_name_label)) },
        singleLine = true,
        enabled = enabled,
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
    deviceName: String,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
) {
    when (connectionState) {
        ConnectionState.DISCONNECTED, ConnectionState.ERROR, ConnectionState.ATTACH_FAILED -> {
            Button(
                onClick = onConnect,
                modifier = Modifier.fillMaxWidth(),
            ) {
                Text(stringResource(R.string.connect_scan, deviceName))
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
