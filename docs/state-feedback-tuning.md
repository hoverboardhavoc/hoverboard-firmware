# Tuning the state-feedback balance controller

For the four-term law in `specs/state-feedback.md`: torque = pitch + pitch rate + position + speed.

## Before you start

- On the Tune screen, select the **right-wheel board** (the master, node 0x03) in the board chips.
  Only that board runs the law; values set on the left-wheel board do nothing.
- The Setup tab must not warn "BOTH BOARDS ARE SET AS THE RIGHT WHEEL". If it does, fix `control.side`
  first (master 1, slave 0).
- Flat floor, hand near the frame, disarm from the app if it runs away.
- Change **one value at a time**, and test after every change.
- **Save only while disarmed**: the board refuses a flash write while armed. Write the values down.

## Starting set

| Row | Meaning | Start |
|---|---|---|
| gp | pitch | 3000 |
| gr | pitch rate (damping) | 110 |
| gx | position (return to the spot) | 0 |
| gv | speed (stop rolling) | 0 |
| speed_max | stick top speed, sixteenths of a count (80 is about 1.4 m/s) | 0 |
| speed_slew | how fast the target speed changes | 80 |

The standard push: a gentle shove to walking-slow (about 0.3 m/s), then let go.

## Order and increments

| Step | Row | Step size | Too low looks like | Too high looks like | Keep |
|---|---|---|---|---|---|
| 1 | gp | 250 | sags, slow to catch, falls over | fast shake, a few times a second | about 75% of where the shake starts; with the shake's onset staged, the row's "Back off to 75%" button sends it |
| 2 | gr | 50 up to 1000, then 250 | wobbles and overshoots after a nudge | buzz or high-pitched vibration | about 65% of where the buzz starts; with the buzz's onset staged, the row's "Back off to 65%" button sends it |
| 3 | gv | 50 | rolls on after a push, runs off | slow rocking about once a second, twitch at rest | the lowest value that stops a standard push in about 1 s |
| 4 | gx | 5 | stops but does not come back | comes back and overshoots past the start, slow swing | the highest value with no overshoot |
| 5 | gr | +50 | | | only if step 4 left a swing |
| 6 | speed_max | 20 | stick feels dead | too fast for the room | 40 to 80 |
| 7 | speed_slew | 10 | keeps rolling after the stick is released, sluggish starts | lurches or rocks on stick moves | stop time about 1 s (below) |

## Limits worth knowing

- **Stop time after releasing the stick is set by speed_slew, not by the gains.** The target speed
  falls to zero at speed_slew per tick, so from full stick:
  `stop time = speed_max x 1.02 / speed_slew` seconds (speed_max 80, slew 80: about 1 s;
  speed_max 163, slew 41: about 4 s). Raise speed_slew for a quicker stop; above about 120 the
  body may rock on starts and stops.

- **gv above about 1780 buys nothing.** One count of speed error (0.29 m/s) then asks for more than
  the 28,500 output clamp, so a push saturates the motors whatever the gain. Stop time on a hard
  push is set by the motors, not by gv.
- **A harder push always takes longer to stop.** The rover can only lean back so far to brake.
  Judge the tune on the standard push.
- **pos_clamp (45 edges, about 26 cm)** caps how far the rover pulls itself back after a held push.
- If gx makes it swing, add gr before lowering gx.

## When done

Disarm, Save, and read the values back (the Tune screen shows the board's own values, and the
flash line under each row shows what is saved).
