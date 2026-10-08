package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import com.hoverboard.remote.R

/**
 * The connected app: the selected tab's screen above a bottom tab row.
 *
 * Leaving the Ride tab calls [onLeaveRide] so the throttle cannot stay held by a gesture the
 * departing screen no longer sees: a pad that leaves composition mid-touch never gets its
 * finger-up.
 */
@Composable
fun ConnectedScreen(
    tab: AppTab,
    onTab: (AppTab) -> Unit,
    onLeaveRide: () -> Unit,
    ride: @Composable () -> Unit,
    tune: @Composable () -> Unit,
    setup: @Composable () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(modifier = modifier.fillMaxSize()) {
        Box(modifier = Modifier.weight(1f)) {
            when (tab) {
                AppTab.RIDE -> ride()
                AppTab.TUNE -> tune()
                AppTab.SETUP -> setup()
            }
        }
        NavigationBar {
            for (t in AppTab.entries) {
                NavigationBarItem(
                    selected = t == tab,
                    onClick = {
                        if (tab == AppTab.RIDE && t != AppTab.RIDE) onLeaveRide()
                        onTab(t)
                    },
                    icon = {},
                    label = { Text(stringResource(t.label)) },
                )
            }
        }
    }
}
