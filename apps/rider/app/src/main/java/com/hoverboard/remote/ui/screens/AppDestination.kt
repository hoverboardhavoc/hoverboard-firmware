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
 * The board layout editor (section 3.5) is the fourth destination and is not here yet, because
 * there is no screen to show for it. [ConnectedScreen] lists the drawer from [entries] and picks
 * its content with an exhaustive `when`, so adding the entry and its screen together is the whole
 * change, and the compiler names every place that has to grow.
 */
enum class AppDestination(@StringRes val label: Int) {
    RIDE(R.string.nav_ride),
    TUNE(R.string.nav_tune),
    SETUP(R.string.nav_setup),
}
