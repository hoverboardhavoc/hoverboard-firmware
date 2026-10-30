package com.hoverboard.remote.ui.screens

import androidx.compose.runtime.Composable
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import com.hoverboard.remote.R
import com.hoverboard.remote.model.RideSlot
import com.hoverboard.remote.model.SlotSide
import com.hoverboard.remote.ui.theme.Atkinson
import com.hoverboard.remote.ui.theme.TABULAR_FIGURES

/*
 * The Ride screen's text (`specs/rider-ui.md` 3.1): its readout styles, which are sizes on the
 * app's one family and feature (3.10), and the slot labels every per-board figure carries.
 */

/** A slot's label: "L" or "R" for a board placed by its side, its address otherwise. */
@Composable
internal fun slotLabel(slot: RideSlot): String = when {
    !slot.bySide -> boardHex(slot.address)
    slot.side == SlotSide.LEFT -> stringResource(R.string.ride_slot_left)
    else -> stringResource(R.string.ride_slot_right)
}

/** A Ride readout style: [size] on [Atkinson] with tabular figures, in [weight]. */
internal fun ride(
    size: TextUnit,
    weight: FontWeight = FontWeight.Normal,
    letterSpacing: TextUnit = TextUnit.Unspecified,
) = TextStyle(
        fontFamily = Atkinson,
        fontWeight = weight,
        fontSize = size,
        letterSpacing = letterSpacing,
        fontFeatureSettings = TABULAR_FIGURES,
    )
