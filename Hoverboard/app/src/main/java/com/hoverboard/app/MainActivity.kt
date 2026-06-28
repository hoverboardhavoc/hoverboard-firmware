package com.hoverboard.app

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.core.content.ContextCompat
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.hoverboard.app.ui.screens.ConnectScreen
import com.hoverboard.app.ui.screens.PermissionScreen
import com.hoverboard.app.ui.theme.Bg
import com.hoverboard.app.ui.theme.HoverboardTheme
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            HoverboardTheme {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Bg)
                        .systemBarsPadding(),
                ) {
                    HoverboardRoot()
                }
            }
        }
    }
}

/** Runtime BLE permissions for this device's API level. */
private fun blePermissions(): Array<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    } else {
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }

@Composable
private fun HoverboardRoot() {
    val viewModel: ConnectViewModel = hiltViewModel()
    val connectionState by viewModel.connectionState.collectAsStateWithLifecycle()
    val discovery by viewModel.discovery.collectAsStateWithLifecycle()
    val context = LocalContext.current
    val permissions = remember { blePermissions() }

    val initiallyGranted = remember {
        permissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }
    }
    var permissionResolved by remember { mutableStateOf(initiallyGranted) }

    if (!permissionResolved) {
        PermissionScreen(
            permissions = permissions,
            onGrant = { permissionResolved = true },
            onSkip = { permissionResolved = true },
        )
    } else {
        ConnectScreen(
            connectionState = connectionState,
            discovery = discovery,
            onConnect = viewModel::connect,
            onDisconnect = viewModel::disconnect,
            onDiscover = viewModel::discover,
        )
    }
}
