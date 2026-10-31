package com.hoverboard.remote.model

/*
 * The settings the Ride figures lack, said in plain words (`specs/remote-ui.md` 3.1, "Missing
 * settings, in plain words"), as a pure function of what the figures are.
 */

/**
 * A setting a Ride figure needs and lacks, or has not read yet (`specs/remote-ui.md` 3.1, "Missing
 * settings, in plain words"), with the Setup row a tap opens ([row]), or null where Setup has none.
 */
enum class MissingSetting(val row: SetupField?) {
    /**
     * A reporting board holds `motors.0.pole_pairs` or `motors.0.wheel_diameter_mm` as 0. Neither
     * has a row yet (section 6 item 10), so Setup opens at the top of the motor rows.
     */
    WHEEL_SIZE(SetupFields.MOTOR_METHOD),

    /** No speed fact is unset, and one is not read yet. */
    WHEEL_READING(null),

    /** `Unjudged.NO_CELLS`. */
    CELLS(SetupFields.PACK_CELLS),

    /** `Unjudged.NO_CHEMISTRY`. */
    CHEMISTRY_READING(null),

    /** `Unjudged.UNKNOWN_CHEMISTRY`. */
    CHEMISTRY_UNKNOWN(SetupFields.BATTERY_CHEMISTRY),

    /** A slot board's `motor.current_cal` reads 0: no screen edits it (3.6c). */
    CURRENT_CAL(null),

    /** A slot board's `motor.current_cal` is not read yet. */
    CURRENT_CAL_READING(null),
}

/** One missing setting and the board it is missing on, which a tap opens Setup on. */
data class Missing(val setting: MissingSetting, val board: Int?)

/**
 * Every missing setting the Ride figures show, in the order the statements list them
 * (`specs/remote-ui.md` 3.1):
 *
 * - the speed: [speed] lacking a fact; an unset one wins over one not read yet, and names the first
 *   of [speedFacts]' boards (address order) holding pole pairs or diameter as 0;
 * - the battery: the ring's [battery] reading, unjudged, on [ringBoard];
 * - the phase current: the first of [slotBoards] whose reading is [CurrentReading.Unscaled], by its
 *   [countsPerAmp] (absent while unread, 0 when unset).
 */
fun missingSettings(
    speed: SpeedReading,
    speedFacts: List<SpeedFacts>,
    ringBoard: Int?,
    battery: BatteryReading,
    slotBoards: List<Pair<Int, CurrentReading>>,
    countsPerAmp: Map<Int, Int>,
): List<Missing> = buildList {
    if (speed is SpeedReading.NoSpeed) {
        if (speed.unset.isNotEmpty()) {
            val board = speedFacts.sortedBy { it.address }.firstOrNull { f ->
                (f.polePairs ?: 1) <= 0 || (f.diameterMm ?: 1) <= 0
            }?.address
            add(Missing(MissingSetting.WHEEL_SIZE, board))
        } else if (speed.unread.isNotEmpty()) {
            add(Missing(MissingSetting.WHEEL_READING, null))
        }
    }
    if (battery is BatteryReading.Unjudged) {
        val setting = when (battery.why) {
            BatteryReading.Unjudged.Why.NO_CELLS -> MissingSetting.CELLS
            BatteryReading.Unjudged.Why.NO_CHEMISTRY -> MissingSetting.CHEMISTRY_READING
            BatteryReading.Unjudged.Why.UNKNOWN_CHEMISTRY -> MissingSetting.CHEMISTRY_UNKNOWN
        }
        add(Missing(setting, ringBoard))
    }
    slotBoards.firstOrNull { it.second == CurrentReading.Unscaled }?.let { (board, _) ->
        val setting = when (countsPerAmp[board]) {
            null -> MissingSetting.CURRENT_CAL_READING
            else -> MissingSetting.CURRENT_CAL
        }
        add(Missing(setting, board))
    }
}
