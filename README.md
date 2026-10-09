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
can change at any time without recompiling or reflashing. The [Status](#status) below says how much of
this is built.

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

The two axes are orthogonal: the same chip turns up in different layouts, and a split board can be
either MCU. The combinations multiply, and one configurable image covers them all instead of a build per
board.

Six-step commutation drives a wheel on both an F103 and an F130 from a single image, under an arming gate
and a current limit, and a phone drives both wheels of a split pair over the board's own Bluetooth. The
runtime-config path is wired end to end: settings written from the phone take effect at the next arm. The
current focus is the balance loop and the machine it is going into (see [Status](#status)).

## Status

Active development, built foundation-up: each piece is host-tested, then brought up on real hardware
before the next.

Legend: ✅ working on hardware &nbsp; 🚧 in progress &nbsp; 📋 planned &nbsp; ❌ abandoned

| Component | | What it is |
|---|:--:|---|
| Foundations (`crates/base`) | ✅ | CRC-16/MODBUS, fixed-point conventions, the shared error vocabulary |
| HAL ([runtime-hal]) | ✅ | chip detection (GD32F103 / F130), GPIO / USART / timer / ADC / I2C / SPI, on-chip flash erase/program; commutation spins a wheel on real boards |
| Config storage (`crates/store`) | ✅ | log-structured key/value config store in flash; host-tested and running on a board |
| Universal firmware binary (`crates/firmware`) | ✅ | one image: detect the GD32 at boot, read and validate the board layout, mount the store, run the 250 Hz control pipeline and the 16 kHz motor ISR; valid fleet-wide |
| Onboard Bluetooth (`crates/ble` + harness) | ✅ | AT bring-up to a transparent BLE byte-pipe, a loopback test firmware, and an Android throughput harness; proven on real phones |
| DMA UART ([runtime-hal]) | ✅ | interrupt- and DMA-driven buffered USART receive behind embedded-io adapters; proven on both chip families |
| Config over Bluetooth | ✅ | a phone (the in-repo Android app) writes a setting into a board's on-board storage over the board's built-in Bluetooth and reads it back: the runtime-config path end to end (a browser configurator is planned) |
| IMU + attitude | ✅ | a configurable I2C IMU driver (multiple models, per-board axis map and gyro bias) and the fused pitch/roll filter; both bench boards read their IMU and publish attitude every control tick |
| Link stack (`crates/link` + `crates/net`) | ✅ | frame format, addressing, discovery, multi-hop forwarding, and the config register protocol wired to the store; framing proven over the inter-board UART and Bluetooth, discovery + config proven over Bluetooth (forwarding is host-tested) |
| Config over SWD (`crates/swd-mailbox` + `crates/swd-bridge`) | ✅ | a RAM-mailbox link transport: a host sets and reads board config through a debug probe, no UART or radio needed |
| Board model | ✅ | the per-board pin map (gates, halls, LEDs, timers, IMU bus, button, pads) read from config at boot, validated against the chip's own capability tables, and applied; a layout that would energize the wrong pin is refused rather than driven |
| Sensing + safety | ✅ | the 250 Hz scheduler, the OFF/INIT/READY/RUN/SHUTDOWN mode machine, fault latches, and the arming gate that is the one place in the image able to energize a bridge; arming, fault escalation and self-shutdown proven on both boards |
| Foot pads (rider detection) | 🚧 | the pad lines are configurable and debounced, and the rider level they produce is what the mode machine refuses to balance without (you only balance with someone on the board). Proven from the phone's rider mirror; no physical pads are wired to the bench boards yet |
| Balance control | 🚧 | the PID cascade, the gain schedule and the engagement machine are built and host-tested, and gains tune live from the phone while running. No machine has balanced yet: the frame it is going into is still being built |
| Motor hot path | ✅ | six-step commutation in a 16 kHz ISR behind the arming gate, with a measured phase-current limit, an over-current trip, and hall-loss and period-liveness faults that shut the bridge down by themselves. Sine and FOC are written and host-validated, but the image folds them to six-step (see [Commutation](#commutation)) |
| Android app controller | ✅ | drive a board from the phone over Bluetooth: arm, a throttle pad, live gain tuning, and per-board setup. Both wheels of a split pair obey one pad, the slave driven over the inter-board link |
| Provisioning + auto-detect | 📋 | a fresh board finds its link and is configured over it (deferred until a board is proven working) |
| Full configurator + flash/backup bridge | 📋 | the complete browser configurator (board layout and tuning) over the board's link, plus an ESP32 bridge for flashing and backup. The Android app already covers setup and tuning; the board-layout editor is designed but not built |
| ~~Firmware update / bootloader~~ | ❌ | **Abandoned: it does not fit in the flash these boards have.** A bootloader and the config store would leave 47,104 B for the app, and the image is 57,904 B. See [Why there is no bootloader](#why-there-is-no-bootloader) |

[runtime-hal]: https://github.com/hoverboardhavoc/runtime-hal


### Why there is no bootloader

Every board in the fleet maps only 64 KiB of flash, the GD32F103RCT6 "256 KB" 12-FET board included:
above 64 KiB it reads back `0x00` rather than firmware, so no part in the fleet has room the others
lack.

A self-update path needs an immutable bootloader at the bottom of flash, below the app, because it
has to bring up the Bluetooth module, receive and validate an image, erase and program flash, and
give feedback through an LED, all without being able to update itself. The design reserved 16 KiB for
it. With the 2 KiB config store above that, the app slot is 47,104 B:

```
0x08000000  bootloader (16 KiB, immutable)
0x08004000  config store (2 KiB)
0x08004800  app slot (47,104 B)   <- the image needs 57,904 B
```

The image is 10,800 B too large for that slot, and nothing in reach closes a gap that size. Two
dedicated shrink rounds have found about 3.6 KiB between them; the measured candidates left are worth
a few hundred bytes each; and the largest single lever is removing Bluetooth entirely, worth about
2.1 KiB, which is the control link. Even cutting the reservation to the 6 KiB once thought
achievable leaves the image over the slot, with nothing left to grow into.

So firmware is written over SWD, which is what [Flashing](#flashing) describes, and the configurator
reaches a board's settings over Bluetooth, UART, or SWD without reflashing it. The bootloader is
written and host-tested on a branch, including a web tool that flashes over Bluetooth, and it is kept
for a board with genuinely more than 64 KiB of mapped flash rather than for this fleet.

## Commutation

All three of EFeru's FOC firmware's commutation methods are written and host-validated against
behaviour fixtures recovered from the stock firmware:

- **Trapezoidal / block commutation**: classic six-step. No current sensor required.
- **Sinusoidal**: open-loop sine commutation. No current sensor required.
- **FOC (Field-Oriented Control)**: closed-loop, the highest-quality method. Requires a current
  sensor.

Boards **with** a current sensor can run all three. Boards **without** one run trapezoidal and
sinusoidal; FOC (and any current or torque based mode that depends on it) is unavailable there.

**The image runs six-step.** `motor.method` is a stored setting, and selecting sine or
FOC is accepted and then folded back to six-step in the running image, which publishes the method it
is actually running rather than the one it was asked for. The two are parked deliberately: the
machine being built does not need them, and a commutation change is the kind of thing that gets
bench-validated on its own rather than carried along.

## Configuration

Configuration lives on the board, not in the build:

- **One binary, fleet-wide.** The image detects the MCU at boot and drives either architecture; the board
  layout and tuning are config, not a rebuild. (A few features may still be compile-time flags where flash
  space is tight.)
- **Config in flash** as a log-structured key/value store: each setting is an appended record with
  its own header and CRC, the newest record for a key wins, and a key with no record reads the
  default its field registry declares. A blank or corrupt region therefore reads as defaults rather
  than failing, and a torn write costs the one record it was writing.
- **Live editing over the board's link** (Bluetooth, UART, or SWD) to read and write parameters
  without recompiling or reflashing. Writes are refused while the board is armed, and a changed
  setting takes effect at the next arm, so nothing is reconfigured under a live bridge.

## Builds

CI builds the universal `firmware` image on every push to `main` (one `firmware.bin`, valid on any
supported part), so you can flash without a local toolchain:

- **Latest `main` build** (no login): [download `firmware.zip`](https://nightly.link/hoverboardhavoc/hoverboard-firmware/workflows/ci/main/firmware.zip)
  (the `firmware.bin` and ELF from the most recent green `main` build, served by
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
