package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import com.hoverboard.remote.R

/**
 * The connected app's destinations (`specs/rider-ui.md` section 2a: the tab row is gone, and the
 * destinations are not peers).
 *
 * [RIDE] is the ROOT: it is where the machine is live. The other three are things done while it is
 * not, so they are pushed onto the stack from Ride's overflow menu and return with an up arrow.
 * The hierarchy is a fact about the destinations, so it is stated here ([isRoot], [PUSHED]) and
 * every screen that has to honour it asks rather than deciding for itself.
 *
 * [ConnectedScreen] lists the menu from [PUSHED] and picks its content with an exhaustive `when`,
 * so a destination and its screen arrive together and the compiler names every place that has to
 * grow.
 *
 * LAYOUT is the board layout editor (section 3.5). It is its own destination rather than a section
 * of Setup because the editing model is different in kind: Setup is a list of independent values
 * and a layout is one object with ownership rules across its fields. It is also touched when a
 * board is built or rewired rather than when a machine is tuned, which is rare, consequential, and
 * worth a deliberate door.
 */
enum class AppDestination(@StringRes val label: Int) {
    RIDE(R.string.nav_ride),
    TUNE(R.string.nav_tune),
    SETUP(R.string.nav_setup),
    LAYOUT(R.string.nav_layout),
    ;

    /**
     * Whether this destination sits at the root of the stack.
     *
     * The root carries the overflow menu and no up arrow; a pushed destination carries an up arrow
     * and no menu, which is what stops a sideways hop between the three.
     */
    val isRoot: Boolean get() = this == ROOT

    companion object {
        /** The root destination, where the machine is live. */
        val ROOT = RIDE

        /** What the root's overflow menu pushes, in the order it lists them. */
        val PUSHED: List<AppDestination> = entries.filter { it != ROOT }
    }
}
