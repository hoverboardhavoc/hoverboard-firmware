package com.hoverboard.remote.ui.screens

import androidx.annotation.StringRes
import com.hoverboard.remote.R

/** The connected app's tabs (`specs/rider-ui.md` section 2). Tune joins once its firmware exists. */
enum class AppTab(@StringRes val label: Int) {
    RIDE(R.string.tab_ride),
    SETUP(R.string.tab_setup),
}
