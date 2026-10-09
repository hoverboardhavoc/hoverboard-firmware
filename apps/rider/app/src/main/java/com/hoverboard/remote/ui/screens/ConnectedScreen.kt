package com.hoverboard.remote.ui.screens

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.ui.theme.ARMED_OUTLINE
import com.hoverboard.remote.ui.theme.AccentRed
import com.hoverboard.remote.ui.theme.TextPrimary

/** Test tag on the top bar surface, which carries the armed outline on every destination. */
const val TOP_BAR_TAG = "top_bar"

/** Test tag on the overflow icon in the top bar: the only way into the menu, and only on Ride. */
const val OVERFLOW_ICON_TAG = "overflow_icon"

/** Test tag on the up arrow in the top bar's leading slot: a pushed destination's way back. */
const val UP_ICON_TAG = "up_icon"

/** Test tag on [destination]'s item in the overflow menu. */
fun menuItemTag(destination: AppDestination): String = "menu_item_${destination.name.lowercase()}"

/**
 * The top bar surface's armed outline: [AccentRed] while armed (`specs/rider-ui.md` section 2a).
 *
 * The Ride screen makes armed state legible by tinting its two big surfaces
 * ([com.hoverboard.remote.ui.screens.ControlScreen], section 3.1), and the pushed destinations show
 * none of those surfaces. The tint used to sit on the overflow icon, which a pushed destination does
 * not have, so it sits on the bar SURFACE instead and reads on every screen. Drawn as the outline
 * the telemetry panel and the throttle pad already use, in the same colour, so it reads as the same
 * signal, and it keeps the title and the icons legible where a filled red bar would not.
 */
internal fun armedBarOutline(armed: Boolean): Color = if (armed) AccentRed else Color.Transparent

/**
 * The connected app: RIDE under a top bar whose three-dot icon pushes the other three destinations,
 * and a pushed destination under a top bar whose leading slot brings it back
 * (`specs/rider-ui.md` section 2a).
 *
 * The navigation is hierarchical, not lateral, and one destination deep: Ride pushes, a pushed
 * destination returns, and nothing pushes further. So the whole stack is "which destination, if not
 * the root", which [destination] already says, and [BackHandler] is all system back needs. A
 * navigation component would add a library, a graph and route strings to hold the same one bit.
 *
 * The menu is behind a tap target and offers no gesture of its own, which is why it is the right
 * component here. The Ride screen's two controls deliberately CONSUME drags
 * ([com.hoverboard.remote.ui.components.ThrottlePad]: "claim the drag so the system/parents don't
 * steal it"), because a throttle that loses its gesture mid-drive is a safety problem. A navigation
 * drawer conventionally opens on an edge swipe and therefore needed a Ride-specific rule to withhold
 * that swipe and a second rule to allow the close drag back; a menu needs neither, and the pad's
 * consumption is untouched either way.
 *
 * Pushing calls [onLeaveRide] so the throttle cannot stay held by a gesture the departing screen no
 * longer sees: a pad that leaves composition mid-touch never gets its finger-up. Every push leaves
 * Ride, because Ride is the only screen with a menu.
 *
 * @param armed whether the board is armed, which the top bar surface carries as an outline.
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
    // System back pops a pushed destination back to the root. On the root it is left unhandled, so
    // it leaves the app, which is the Android contract for a top-level destination.
    BackHandler(enabled = !destination.isRoot) { onDestination(AppDestination.ROOT) }
    Column(modifier = modifier.fillMaxSize()) {
        TopBar(
            destination = destination,
            armed = armed,
            onPush = { picked ->
                onLeaveRide()
                onDestination(picked)
            },
            onUp = { onDestination(AppDestination.ROOT) },
        )
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

/**
 * The top bar: the destination's name on an armed-outlined surface, with the root's overflow menu
 * or a pushed destination's up arrow.
 *
 * Exactly one of the two slots is filled, which is the navigation model made visible: the root has
 * no higher destination to go up to, and a pushed destination has no menu to hop sideways with.
 */
@Composable
private fun TopBar(
    destination: AppDestination,
    armed: Boolean,
    onPush: (AppDestination) -> Unit,
    onUp: () -> Unit,
) {
    // The armed state is said in words as well as drawn, because an outline is the one indication a
    // screen reader cannot read out. It goes on the title, which every destination has, now that the
    // tint is off the icon one of them lacks.
    val title = stringResource(destination.label)
    val spoken = if (armed) stringResource(R.string.nav_title_armed, title) else null
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .testTag(TOP_BAR_TAG)
            .border(ARMED_OUTLINE, armedBarOutline(armed)),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        if (!destination.isRoot) {
            IconButton(onClick = onUp, modifier = Modifier.testTag(UP_ICON_TAG)) {
                Icon(
                    imageVector = Icons.AutoMirrored.Filled.ArrowBack,
                    contentDescription = stringResource(R.string.nav_up),
                    tint = TextPrimary,
                )
            }
        }
        Text(
            title,
            style = MaterialTheme.typography.titleLarge,
            color = TextPrimary,
            modifier = Modifier
                .weight(1f)
                .padding(start = if (destination.isRoot) TITLE_INSET else 0.dp)
                .semantics {
                    if (spoken != null) contentDescription = spoken
                },
        )
        if (destination.isRoot) {
            OverflowMenu(onPush)
        }
    }
}

/** Ride's three-dot menu: the three destinations it pushes, behind a tap target and no gesture. */
@Composable
private fun OverflowMenu(onPush: (AppDestination) -> Unit) {
    var open by remember { mutableStateOf(false) }
    Box {
        // The tag is on the button, not the icon inside it: the button merges its descendants'
        // semantics, so a tag below it is reachable only in the unmerged tree.
        IconButton(onClick = { open = true }, modifier = Modifier.testTag(OVERFLOW_ICON_TAG)) {
            Icon(
                imageVector = Icons.Filled.MoreVert,
                contentDescription = stringResource(R.string.nav_open),
                tint = TextPrimary,
            )
        }
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            for (d in AppDestination.PUSHED) {
                DropdownMenuItem(
                    text = { Text(stringResource(d.label)) },
                    onClick = {
                        open = false
                        onPush(d)
                    },
                    modifier = Modifier.testTag(menuItemTag(d)),
                )
            }
        }
    }
}

/** The title's inset on the root, where no up arrow has already indented it. */
private val TITLE_INSET = 16.dp
