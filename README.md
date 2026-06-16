# hoverboard-firmware

Open firmware for hoverboard mainboards, controllable via an app, with optional self-balancing.

Control it two ways:

- **Built-in Bluetooth module** on the board.
- **An external microcontroller** over UART.

Settings are configurable from an Android app. The first target is a **GD32F103-class hoverboard (GD32F103-class
board) with the built-in Bluetooth module**, but the goal is a universal firmware for hoverboard
mainboards.

## Commutation

The firmware supports the same commutation methods as EFeru's FOC firmware:

- **Trapezoidal / block commutation** — classic six-step. No current sensor required.
- **Sinusoidal** — open-loop sine commutation. No current sensor required.
- **FOC (Field-Oriented Control)** — closed-loop, the highest-quality method. Requires a current
  sensor.

Boards **with** a current sensor can run all three. Boards **without** one run trapezoidal and
sinusoidal; FOC (and any current/torque-based mode that depends on it) is unavailable there.

## Configuration

Configuration lives on the board, not in the build:

- **One binary per MCU**. Boards sharing an MCU run the same image; tuning is config, not a
  rebuild. Some features may still be compile-time flags where flash space is tight.
- **Config in flash** with a versioned struct, a magic/CRC header, and a fall back to safe
  defaults when the sector is blank or corrupt.
- **Live editing over serial** to read and write parameters without recompiling or reflashing.

## Status

Early. Nothing here is built or flashed yet. Architecture is being laid out.

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
