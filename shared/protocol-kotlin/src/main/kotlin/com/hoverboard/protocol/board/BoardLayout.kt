package com.hoverboard.protocol.board

/**
 * The board crate's one number a client needs (`motorcontroller/board/src/lib.rs, DEAD_TIME_MIN_DTG`): the dead-time
 * floor the configuration checks hold a gate group to.
 */

/**
 * The dead-time FLOOR a CONFIGURED gate group must meet, in `motor.dead_time` DTG ticks
 * (`motorcontroller/board/src/lib.rs`, `DEAD_TIME_MIN_DTG`, which carries the full derivation). The
 * configuration checks refuse a configured gate group below it before it is written
 * ([com.hoverboard.protocol.checks.Check.DEAD_TIME]); the firmware applies what is stored.
 *
 * The arithmetic in short: the gate timer is clocked at the 72 MHz sysclk with its dead-time
 * generator on the timer clock divided by 2, so one tick is 27.8 ns, and the 500 ns reference
 * figure, the shortest dead time any board in the fleet is measured running, is 18 ticks
 * (`specs/commutation.md`, the dead-time contracts). The floor admits every value the fleet runs
 * (25 on the 6-FET split board, 28 in the offroad stock dump, 32 on the 12-FET) and refuses the
 * range below it: 1 tick is 28 ns, roughly 25x too short, which is shoot-through.
 *
 * Zero is not measured against it. That is the right value for a board with no motor, so it is
 * legal with the gate group unset and the floor applies only where the gates are claimed.
 *
 * The editor range of any client that offers this field FOLLOWS this constant rather than
 * restating it, so a client cannot offer a value the checks refuse.
 */
const val DEAD_TIME_MIN_DTG = 18
