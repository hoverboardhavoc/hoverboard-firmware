Thanks, this is a good report and the mechanism you identified is right.

The chain you describe was fixed one commit after the image you built. `c8374b6` is the parent of `c8604b9`, which removed the shutdown's counter-stop step: the period ISR now runs from bring-up to power-off, so `SPEED` keeps being recomputed while the board is disarmed, and a board that trips with the wheel turning recovers on its own as soon as the wheel stops. Your SWD readings are all consistent with your image, including the frozen `motor_periods` and the clear `OBS_PERIOD_LIVE`.

Your two structural observations survived that fix, though, and have now been acted on in `89b43b4`. `off_inhibit` was still `speed != 0`, with nothing asking whether that word was a measurement, which left the same deadlock reachable through anything else that stops the ISR. The gate now reads the period-liveness observation beside the speed word and inhibits outright on a shortfall, without consulting the speed at all.

We went with inhibit rather than the OFF dwell you suggested, and the reasoning is worth stating since it goes the other way to your proposal. A frozen non-zero word costs a reboot, which is what you hit. A frozen zero word is worse: a board parked at `SPEED` 0 whose vector wedges and is then pushed would read as stationary, and arming would energise the bridge into a rolling wheel. A board held in OFF this way is not silent either, since the sustained form of the same shortfall raises the period-liveness fault 20 ms later, which gates arming and is published.

The `FAULT_DEMAND_STALE` bit is separate and already known: a boot-window latch set before the tick source comes up, present on every boot of both our bench boards, and with no causal link to the trip. It is tracked on its own.

The 29 inter-board UART line errors are not explained. If you still have the board in that state, the sequencing of those against the trip would be useful.
