# hoverboard-firmware

> ⚠️ **Work in progress, not properly tested.** Beware of
> [shoot-through](https://en.wikipedia.org/wiki/Shoot-through): a bug in commutation, dead-time, or the
> fault/arming state machine can turn both FETs of a phase leg on at once, shorting the battery across the
> bridge. Bench it on a current-limited supply with the motor disconnected.

**The ambition: one configurable firmware for building vehicles, self-balancing or not, from a single
binary that runs across the whole spread of hoverboard boards, 12-FET and 6-FET, single, twin/split, and
side.** Build a hoverboard you ride on its foot pads, or a non-balancing machine (a kart, a scooter, a robot
base) driven by a remote or a throttle: the same image, reconfigured rather than recompiled.

Everything board-specific comes from configuration, not the build: the firmware detects the MCU at boot,
then reads the board layout, IMU model, drive mode, and tuning from settings stored on the board, which you
can change at any time without recompiling or reflashing. The [Status](#status) below is the honest picture
of how much of this exists today.

Every other open hoverboard firmware picks the board at **compile time** (board `#define`s, a per-board
header, an online build generator). Change the board, swap the IMU, or even move which pin a wire is on, and
you build a different binary. This one is configured at runtime instead.

## One binary, two architectures

Two things differ between boards, and they are **independent axes**: the MCU and the board layout. Each is
resolved at runtime, not baked into the build.

- **The MCU is detected at boot.** The fleet spans two genuinely different peripheral architectures, not
  minor variants of one: an **STM32F103 / GD32F103** class (APB-bus GPIO, the legacy `CRL`/`CRH` config
  registers) and a **GD32F130** class (AHB-bus GPIO with an explicit alternate-function mux, an ST-style
  peripheral set). Same Cortex-M3 core, different register model. One image detects which it is on and
  drives the right model either way, on the [runtime-hal](https://github.com/hoverboardhavoc/runtime-hal)
  foundation. (The small auxiliary sideboards are F130 parts too, so the same image runs on them.)
- **The board layout cannot be detected:** which pins drive the phases, where the halls, IMU, buttons, and
  LEDs sit, which IMU model is fitted, and whether it is a single mainboard driving both wheels or a split
  board with one controller per wheel. Every hoverboard firmware needs this; here it is **configuration stored on the board**, written
  and changed in the field over the board's link, which runs over **Bluetooth, UART, or SWD**, from a
  configurator.

The two axes are orthogonal: the same chip turns up in very different layouts, and a split board can be
either MCU. The combinations multiply, and one configurable image covers them all instead of a build per
board.

Six-step commutation already spins a wheel on both an F103 and an F130 from a single image; wiring up the
runtime-config path is the current focus (see [Status](#status)).

## Status

Active development, built foundation-up: each piece is host-tested, then brought up on real hardware
before the next.

Legend: ✅ working on hardware &nbsp; 🚧 in progress &nbsp; 🔜 next &nbsp; 📋 planned

| Component | | What it is |
|---|:--:|---|
| Foundations (`crates/base`) | ✅ | CRC-16/MODBUS, fixed-point conventions, the shared error vocabulary |
| HAL ([runtime-hal]) | ✅ | chip detection (GD32F103 / F130), GPIO / USART / timer / ADC / I2C / SPI, on-chip flash erase/program; commutation spins a wheel on real boards |
| Config storage (`crates/store`) | ✅ | log-structured key/value config store in flash; host-tested and running on a board |
| Universal firmware binary (`crates/firmware`) | ✅ | one image: detect the GD32 at boot, mount the store, run the housekeeping loop; valid fleet-wide |
| Onboard Bluetooth (`crates/ble` + harness) | ✅ | AT bring-up to a transparent BLE byte-pipe, a loopback test firmware, and an Android throughput harness; proven on real phones |
| DMA UART ([runtime-hal]) | ✅ | interrupt- and DMA-driven buffered USART receive behind embedded-io adapters; proven on both chip families |
| Config over Bluetooth | ✅ | a phone (the in-repo Android app) writes a setting into a board's on-board storage over the board's built-in Bluetooth and reads it back: the runtime-config path end to end (a browser configurator is planned) |
| IMU + attitude | 📋 | a configurable I2C IMU driver (multiple IMU models) and the pitch/roll attitude filter (an IMU already answers over I2C on the bench) |
| Link stack (`crates/link` + `crates/net`) | ✅ | frame format, addressing, discovery, multi-hop forwarding, and the config register protocol wired to the store; framing proven over the inter-board UART and Bluetooth, discovery + config proven over Bluetooth (forwarding is host-tested) |
| Config over SWD (`crates/swd-mailbox` + `crates/swd-bridge`) | ✅ | a RAM-mailbox link transport: a host sets and reads board config through a debug probe, no UART or radio needed |
| Board model | 📋 | the per-board pin map (halls, gates, LEDs, timers) selected at boot |
| Sensing + safety | 📋 | the scheduler and the mode/fault/arming state machine |
| Foot pads (rider detection) | 📋 | read the foot-pad sensors to detect the rider and arm balancing (you only balance with someone on the board) |
| Balance control | 📋 | the PID cascade |
| Motor hot path | 📋 | commutation: six-step first, then sinusoidal and FOC, gated behind the safety state machine |
| Provisioning + auto-detect | 📋 | a fresh board finds its link and is configured over it (deferred until a board is proven working) |
| Android app controller | 📋 | drive a board from a phone over Bluetooth (the in-repo app already talks the link; driving needs the link-control payloads and the motor path) |
| Full configurator + flash/backup bridge | 📋 | the complete browser configurator (board layout + tuning) over the board's link, plus an ESP32 bridge for flashing and backup |
| Firmware update / bootloader | 📋 | a small immutable bootloader (flash + boot-select), updatable over SWD, Bluetooth, and mesh-routed over the link |

[runtime-hal]: https://github.com/hoverboardhavoc/runtime-hal

## Commutation

Support is planned for the same commutation methods as EFeru's FOC firmware:

- **Trapezoidal / block commutation**: classic six-step. No current sensor required.
- **Sinusoidal**: open-loop sine commutation. No current sensor required.
- **FOC (Field-Oriented Control)**: closed-loop, the highest-quality method. Requires a current
  sensor.

Boards **with** a current sensor can run all three. Boards **without** one run trapezoidal and
sinusoidal; FOC (and any current/torque-based mode that depends on it) is unavailable there.

## Configuration

Configuration lives on the board, not in the build:

- **One binary, fleet-wide.** The image detects the MCU at boot and drives either architecture; the board
  layout and tuning are config, not a rebuild. (A few features may still be compile-time flags where flash
  space is tight.)
- **Config in flash** with a versioned struct, a magic/CRC header, and a fall back to safe
  defaults when the sector is blank or corrupt.
- **Live editing from the configurator** (over the board's link: Bluetooth, UART, or SWD) to read and write
  parameters without recompiling or reflashing.

## Builds

CI builds the universal `firmware` image on every push to `main` (one `firmware.bin`, valid on any
supported part), so you can flash without a local toolchain:

- **Latest `main` build** (no login): [download `firmware.zip`](https://nightly.link/hoverboardhavoc/hoverboard-firmware/workflows/ci/main/firmware.zip)
  — the `firmware.bin` + ELF from the most recent green `main` build (served by
  [nightly.link](https://nightly.link)).
- Or open any commit's run in the [Actions](https://github.com/hoverboardhavoc/hoverboard-firmware/actions)
  tab and grab the **firmware** artifact (needs a GitHub login).

## Flashing

The firmware is written to the mainboard MCU over SWD, using either a wireless ESP32-C3 debug probe
or an ST-Link V2 clone. See [docs/flashing.md](docs/flashing.md).

## Documentation

See [docs/](docs/) for hardware-facing references:

- [Onboard BLE module](docs/onboard-ble-module.md): the observable interface of the built-in
  CC2541-class Bluetooth module (AT bring-up, quirks, GATT layout, central integration notes).
- [BLE throughput harness](crates/ble/test-harness/README.md): how to build and run the Android +
  board-side loopback harness that measures the raw BLE byte-pipe throughput.

## Prior art

- [NiklasFauth/hoverboard-firmware-hack](https://github.com/NiklasFauth/hoverboard-firmware-hack)
  (lucysrausch / NiklasFauth): the original hack (trapezoidal commutation).
- [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)
  (EmanuelFeru): the FOC standard everyone forks (commutation / sinusoidal / FOC, field weakening,
  voltage / speed / torque modes).
- [RoboDurden/hoverboard-firmware-hack-FOC](https://github.com/RoboDurden/hoverboard-firmware-hack-FOC):
  forks of the above, per-board defines, [online config generator](https://pionierland.de/hoverhack/),
  C++ rewrite.

All compile-time configured. This project is the runtime-config alternative.
