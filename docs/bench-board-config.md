# Reading a board's stored configuration over SWD

Everything a board here knows about itself is in its own flash, not in the build: one universal
image runs on every part, and the pin map, the IMU wiring, the gains and the limits are store
records read at boot and at every arm. This page is how to read and write them, and how to decode
what comes back.

**It deliberately holds no per-board values.** An earlier version of this page was a table of what
the two bench boards had staged on a particular afternoon. A table like that is a second copy of
state the boards themselves answer in one command, and it was wrong within days of each capture,
twice. The live values are on the boards; dated captures belong in `specs/bench-evidence/<date>/`
with the session that took them; the compiled defaults are in `crates/store/src/field.rs` and
change with the tree. What is below is the part that does not rot.

## Reading and writing a value

Host tool, from the repo root on the Mac that owns the bench:

```
crates/swd-bridge/target/<host-triple>/release/swd-mailbox-config <ocd-host:port> FIELD[:INDEX] ...
```

Add `=VALUE` to write, and the tool reads the value back and confirms it. `--dst ADDR` reaches a
board through the other one over the inter-board link (two hops). Writes are refused while a board
is armed (`CFG_ARMED`).

**Which probe, which board, which OpenOCD.** `tools/flash.sh` carries the one probe table: the
board name (`master`, `slave`, `offroad-master`, `offroad-slave`, `rover-master`, `rover-slave`),
its probe, its transport and the host that runs OpenOCD for it. Read it there rather than from a
list in a document; the probes move and the addresses drift. `~/notes/bench-overview.md` has the
physical map and the ground-path trap on the ESP32-C3 SuperMini probes.

**Which node answers.** A node address is WALK-ASSIGNED and is not a role. `node.address` (0x01)
persists, so what a board holds reflects whichever walk assigned it: the rover pair runs master
0x02 and slave 0x01, and the offroad pair runs the other way round. Take the address of the
attached node from what the walk reports, never from a convention, and `--dst` the other one.

## Decoding key

- **Pin fields** are a packed byte, `(port << 4) | pin`, port 0 = PA, 1 = PB, 2 = PC. `0xFF` = the
  pin is absent on this board. Pin fields are 0x40 through 0x55; `motor.dead_time` (0x64) is a raw
  timer value in that same numeric range and is NOT a pin.
- **`link.set` (0x02)** is a bit per port: `6` = `0b110` = the inter-board link and the BLE port
  are live.
- **`control.mode` (0x22)**: 0 = Throttle (works with no IMU), 1 = Balance.
- **`control.rider_required` (0x23)**: 1 = the engage gate requires a rider level, 0 = it does not,
  which is how a padless machine balances (`specs/control.md` (i)).
- **`control.frame` (0x25)**: 0 = pivoting platforms, each board balancing its own half; 1 = rigid
  body, one loop on the leader and the follower taking its wheel demand off the cyclic frame
  (`specs/control.md` (k), `specs/rover-one-imu.md`).
- **`motor.method` (0x21)**: 0 = six-step, 1 = sine, 2 = FOC. FOC is a capability request: a board
  without accepted current-sense offsets runs six-step and raises the method-demotion fault bit
  rather than substituting silently, and the running image publishes the method it is actually
  running.
- **Gain triples (0x71 profile A, 0x72 profile B, 0x74 maxima)**: index 0 = `kp`, 1 = `bk`,
  2 = `pr`.
- **`control.drive_lean` (0x73)**: index 0 = `lean_max` in centidegrees (0 = drive lean disabled),
  index 1 = `lean_slew`.
- **`imu.axis_sign` (0x65)**: indices 0..5 = `[ax, ay, az, gx, gy, gz]`. **0 means unset**, and that
  index then falls back to the compiled reference map `[-1, 1, -1, -1, 1, -1]`
  (`crates/imu/src/lib.rs`).
- **`imu.axis_role` (0x68)**: index 0 = which chip axis is UP, index 1 = which is PITCH_RATE,
  1 = X, 2 = Y, 3 = Z. 0 = unset, falling back to UP = Z, PITCH_RATE = Y. The roles are a second
  field beside the sign map because a board standing on edge is a 90 degree rotation from the flat
  stock mount, which six signs cannot express (`specs/imu.md`).
- **`imu.model` (0x60) = 2** is the WHO_AM_I 0x2E MPU-6050 clone fitted to every board here.
- **Units**: current in mA, battery in cV, angles in centidegrees, gyro bias in raw counts.

## Things worth knowing before trusting a reading

- **`swd-mailbox-walk` leaves `motor.current_limit` (0x20) overwritten.** The walk round-trips that
  field as its `CONFIG` self-test and does not restore it, so a walk leaves 15,000 mA behind (its
  `config-value` default) unless someone puts the bench value back. Check 0x20 after any walk.
- **Index 1 of every per-motor field reads its default**, because each board drives one wheel and
  only index 0 is staged. A default at `0x20:1` means "absent", not "configured".
- **A staged sign map that differs from the compiled default is not a mistake.** The compiled map
  came from the stock board's mount. Both bench boards measured wrong against it on 2026-07-31
  (level and the right way up, the firmware reported roll near 180 degrees), so their real mount is
  stored per board. Do not "correct" a sign map from a single static reading without knowing the
  board's actual physical attitude, and note that a board mounted upside down for a bench session
  carries a sign map that is wrong the moment it is turned over.
- **`board.vbatt` (0x41) is master-only.** The pin is fleet-uniform (PA4) but only a master senses
  the rail; a slave reads that pin as a stuck ~2.0 V and gets the battery over the link, so the
  field defaults ABSENT and a master is staged `0x04` once (`specs/sensing-and-safety.md`).
- **`pad.a` (0x46) and `pad.b` (0x47) default ABSENT** as of 2026-10-23. The pins are fleet-uniform
  but no board here has pad hardware fitted, and a defaulted pin would sample a floating line the
  active-high debouncer reads as a foot (`specs/board-model.md`).
- **The IDCODE does not tell you the chip family.** Every part here reads `0x13030410` with
  `set CPUTAPID 0`, which these GD32s require. The firmware detects the family itself at boot and
  publishes it; `tools/flash.sh`'s probe table records each board's family and UID.

## Taking a snapshot

When a session needs the staged state recorded, read it with `swd-mailbox-config` and file the
output under `specs/bench-evidence/<date>/<session>/` with the record that used it, dated and
attached to what it was evidence for. That is where a capture stays true: as a statement about one
afternoon, next to the run that depended on it.

## Source of truth in the repo

- `crates/store/src/field.rs`: every field id, its type, its compiled default, and the prose
  explaining why it exists. The `REGISTRY` array at the bottom is the complete list.
- `crates/swd-bridge/src/bin/config_write.rs`: the reader/writer tool used here.
- `tools/flash.sh`: the probe table, which board runs on which host, and the flash guards.
