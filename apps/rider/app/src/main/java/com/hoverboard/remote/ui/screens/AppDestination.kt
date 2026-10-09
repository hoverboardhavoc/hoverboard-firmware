package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import com.hoverboard.remote.R

/**
 * The connected app's destinations, as the navigation drawer lists them (`specs/rider-ui.md`
 * section 2a: the tab row is gone, a drawer replaces it).
 *
 * Four destinations is where a tab row starts to crowd a phone, and these are not peers: RIDE is
 * where the machine is live and the others are things done while it is not.
 *
 * [ConnectedScreen] lists the drawer from [entries] and picks its content with an exhaustive
 * `when`, so a destination and its screen arrive together and the compiler names every place that
 * has to grow.
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
}
