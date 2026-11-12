# Documentation

Public, hardware-facing documentation for this firmware project. These docs describe **observable**
behavior (what you can see on the wire or over the air) and **how to build and run** the tooling. They are
deliberately implementation-agnostic and contain no third-party firmware internals.

## Contents

- [Getting started](getting-started.md): from a stock motor controller to driving: wire and install
  the bridge, flash the boards, apply a board file from [`boards/`](../boards/).
- [hoverboard-cli reference](cli-reference.md): every command.
- [Building](building.md): build the images, the CLI and the app, and run the tests.
- [Onboard BLE module](onboard-ble-module.md): the observable interface of the onboard CC2541-class
  transparent UART-to-BLE module found on these GD32 hoverboard mainboards: its AT bring-up sequence, its
  quirks, and the GATT layout a central sees once it is in transparent data mode.
- [Reading a board's stored configuration over SWD](bench-board-config.md): how to read and write
  a board's store with `swd-mailbox-config`, and how to decode what comes back: the pin packing, the
  mode and frame bytes, the gain triples, the IMU sign and role maps, and the traps (a walk
  overwrites the current limit; a node address is walk-assigned, not a role).
- [Tuning the state-feedback balance controller](state-feedback-tuning.md): the four-term
  law of `specs/state-feedback.md` (pitch, pitch rate, position, speed): a starting set, the tuning
  order and increments, and the rover's tune.

## Scope and provenance

This documents **RoboDurden-style ClassyWalk 2-1-20 / 2-2-20 GD32 hoverboard mainboards**, some of which
carry the onboard Bluetooth module described here (a chip stamped **`TTC2541 F256`** has been observed). The
open firmware for these mainboards is
[RoboDurden's Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x)
(see the [GD32F130 v20 board folder](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/tree/main/target_1%3DGD32F130/v20));
the onboard-Bluetooth support for these boards was added in a separate branch on top of that firmware by
this project, not by RoboDurden upstream.

The module behavior documented here was **observed on the bench**: the AT/UART exchange was captured by
instrumenting the firmware (recording the module's replies and reading them back over SWD), and the
advertising and GATT layout were seen by scanning and discovering services from a BLE central. The
over-the-air side is what any BLE scanner app shows.
