package com.hoverboard.protocol.board

/**
 * The SWD pins, PA13 and PA14, packed: the one pair no layout field may hold. The HAL refuses them
 * for every function, because SWD is the path every board is configured over
 * (`specs/configured-links.md` section 3), so a field staged on one is recorded in the board's status as [BoardStatus.Reason.SWD_PIN] and its
 * function stays absent. A picker never offers them ([candidatePins]).
 */
val SWD_PINS = listOf(0x0D, 0x0E)
