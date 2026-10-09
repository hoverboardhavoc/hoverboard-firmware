package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Menu
import androidx.compose.material3.DrawerValue
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.ModalDrawerSheet
import androidx.compose.material3.ModalNavigationDrawer
import androidx.compose.material3.NavigationDrawerItem
import androidx.compose.material3.Text
import androidx.compose.material3.rememberDrawerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextPrimary
import kotlinx.coroutines.launch

/** Test tag on the drawer icon in the top bar: on RIDE, the only way into the drawer. */
const val DRAWER_ICON_TAG = "drawer_icon"

/** Test tag on [destination]'s item in the drawer. */
fun drawerItemTag(destination: AppDestination): String = "drawer_item_${destination.name.lowercase()}"

/**
 * Whether the drawer may be opened or closed by a DRAG (`specs/rider-ui.md` section 2a).
 *
 * False on RIDE while the drawer is closed, and that is the whole reason the drawer needed a
 * decision rather than a theme change. A drawer conventionally opens on an edge swipe, and the
 * Ride screen's two controls deliberately CONSUME drags
 * ([com.hoverboard.remote.ui.components.ThrottlePad]: "claim the drag so the system/parents don't
 * steal it"), because a throttle that loses its gesture mid-drive is a safety problem. The conflict
 * is resolved HERE, by the drawer not offering the gesture on RIDE at all, rather than there, by
 * weakening what the pad claims: the pad's consumption stays exactly as it was. On RIDE the drawer
 * opens from its icon and nothing else.
 *
 * True while the drawer IS open, on RIDE as everywhere: the scrim is over the pad by then, so a
 * drag that closes the drawer is taking nothing from a control the rider can reach, and a drawer
 * that cannot be swiped shut is a drawer the one-thumb layout has to reach the scrim to dismiss.
 *
 * On the other destinations nothing claims a drag at the edge, so the swipe is free.
 */
internal fun drawerGesturesEnabled(destination: AppDestination, drawerOpen: Boolean): Boolean =
    destination != AppDestination.RIDE || drawerOpen

/**
 * The drawer icon's tint: [AccentRed] while armed (`specs/rider-ui.md` section 2a).
 *
 * The Ride screen makes armed state legible by tinting its two big surfaces
 * ([com.hoverboard.remote.ui.screens.ControlScreen], section 3.1), and a drawer HIDES the other
 * destinations instead of showing them beside RIDE, so on Tune or Setup none of those surfaces is
 * on screen. The icon is the one element present on every destination, so it carries the tint and
 * the armed state reads from the top bar wherever the rider has navigated. The same colour as the
 * outlines, so it reads as the same signal.
 */
internal fun drawerIconTint(armed: Boolean): Color = if (armed) AccentRed else TextPrimary

/**
 * The connected app: the current destination's screen under a top bar whose icon opens the
 * navigation drawer (`specs/rider-ui.md` section 2a).
 *
 * Leaving the Ride destination calls [onLeaveRide] so the throttle cannot stay held by a gesture
 * the departing screen no longer sees: a pad that leaves composition mid-touch never gets its
 * finger-up.
 *
 * @param armed whether the board is armed, which the drawer icon carries as a tint.
 */
@Composable
fun ConnectedScreen(
    destination: AppDestination,
    onDestination: (AppDestination) -> Unit,
    onLeaveRide: () -> Unit,
    armed: Boolean,
    ride: @Composable () -> Unit,
    tune: @Composable () -> Unit,
    setup: @Composable () -> Unit,
    layout: @Composable () -> Unit,
    modifier: Modifier = Modifier,
) {
    val drawer = rememberDrawerState(DrawerValue.Closed)
    val scope = rememberCoroutineScope()
    ModalNavigationDrawer(
        drawerState = drawer,
        gesturesEnabled = drawerGesturesEnabled(destination, drawer.isOpen),
        drawerContent = {
            ModalDrawerSheet {
                Text(
                    stringResource(R.string.nav_title),
                    style = MaterialTheme.typography.titleMedium,
                    color = TextPrimary,
                    modifier = Modifier.padding(16.dp),
                )
                for (d in AppDestination.entries) {
                    NavigationDrawerItem(
                        label = { Text(stringResource(d.label)) },
                        selected = d == destination,
                        onClick = {
                            scope.launch { drawer.close() }
                            if (destination == AppDestination.RIDE && d != AppDestination.RIDE) onLeaveRide()
                            onDestination(d)
                        },
                        modifier = Modifier
                            .padding(horizontal = 12.dp)
                            .testTag(drawerItemTag(d)),
                    )
                }
            }
        },
        modifier = modifier,
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            TopBar(destination, armed) { scope.launch { drawer.open() } }
            Box(modifier = Modifier.weight(1f)) {
                when (destination) {
                    AppDestination.RIDE -> ride()
                    AppDestination.TUNE -> tune()
                    AppDestination.SETUP -> setup()
                    AppDestination.LAYOUT -> layout()
                }
            }
        }
    }
}

/** The top bar: the drawer icon (armed-tinted) and the destination's name. */
@Composable
private fun TopBar(destination: AppDestination, armed: Boolean, onOpen: () -> Unit) {
    Row(modifier = Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        // The tag is on the button, not the icon inside it: the button merges its descendants'
        // semantics, so a tag below it is reachable only in the unmerged tree.
        IconButton(onClick = onOpen, modifier = Modifier.testTag(DRAWER_ICON_TAG)) {
            Icon(
                imageVector = Icons.Filled.Menu,
                // The armed state is said in words here as well as in the tint, because a tint is
                // the one indication a screen reader cannot read out.
                contentDescription = stringResource(if (armed) R.string.nav_open_armed else R.string.nav_open),
                tint = drawerIconTint(armed),
            )
        }
        Text(
            stringResource(destination.label),
            style = MaterialTheme.typography.titleLarge,
            color = TextPrimary,
            modifier = Modifier.padding(start = 4.dp),
        )
    }
}
