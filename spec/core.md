# Core architecture spec

Status: draft. This is the foundational spec: target MCUs, the layering that makes one codebase
serve several boards, the fixed-point math basis, where the per-MCU seams fall, and where the
compile-time vs runtime line is drawn. Later specs (control, commutation, config, links) build on
the decisions here.

## Targets

| Target | Family / PAC | Package | Flash / RAM | Motors | Advanced timers |
|---|---|---|---|---|---|
| GD32F130C8 | F1x0 / `gd32f1` | 48-pin | 64K / 8K | 1 | TIMER0 |
| GD32F103C8 | F10x / `stm32f1` | 48-pin | 64K / 20K | 1 | TIMER0 |
| GD32F103RCT6 | F10x / `stm32f1` (high-density) | 64-pin | 256K / 48K | 2 (12-FET dual) | TIMER0 + TIMER7 |

All targets are Cortex-M3 with no FPU, and all three are committed. First bench bring-up is a
GD32F130 blue pill; the hoverboard controllers are the rest. The F130C8 and F103C8 are single-motor;
the RCT6 is the 12-FET dual-motor board. The topology is N-motor throughout (see Module shape and
seams), so the single- and dual-motor builds share one control path.

Concrete implementations on more than one part keep the abstraction honest: the seams are designed
against real F130 and F103 code, not guessed from one example. The only things fixed per build are
the PAC family and the memory map, so it is one binary per MCU part, with board setup in runtime
config (see Configuration model).

## Layered stack

From the compiler down to the application, each layer narrows from "any Cortex-M" to "this chip":

1. **Target triple** `thumbv7m-none-eabi`. All targets are Cortex-M3. No FPU, so no hardware float
   (see Math).
2. **`cortex-m` / `cortex-m-rt`** for core peripherals (NVIC, SysTick, SCB), the reset/vector
   runtime, and a per-MCU `memory.x` (flash/RAM origins and sizes).
3. **PAC** (register definitions, svd2rust-generated):
   - F130: `gd32f1` with the `gd32f130` feature (the `gd32f1` crate covers F1x0 only: F130/F150/
     F170/F190).
   - F103: the `stm32f1` PAC used by `stm32f1xx-hal` (the GD32F103 is register/flash compatible
     with the STM32F103). `gd32f1` does **not** include F103.
4. **HAL**:
   - F130: `gd32f1x0-hal` (gd32-rust org), built on the `gd32f1` PAC.
   - F103: `stm32f1xx-hal` (high-density feature for the RCT6).
   - Both implement `embedded-hal` 1.0, so cold-path code compiles unchanged on either.
5. **`embedded-hal` 1.0 traits** as the cold-path seam (digital GPIO, SPI, I2C, PWM duty, delay;
   UART via `embedded-io`).
6. **Pure control-math layer** (fixed-point), MCU-independent. Operates on numbers only; no
   hardware. Host-testable and bit-exact across targets.

## Cold path vs hot path

The split decides what goes through `embedded-hal` and what drops to the PAC.

- **Cold path, via the HAL / embedded-hal traits.** LEDs, buzzer, enable/standby lines, the button,
  the two UART links (BLE module + external MCU), and the IMU (I2C/SPI). Written once against the
  traits; reusable community drivers (e.g. an IMU driver) work as-is.
- **Hot path, via the PAC.** The advanced-timer motor drive (center-aligned PWM, complementary
  outputs, dead-time, break input) and the timer-triggered injected ADC current sampling.
  `embedded-hal` cannot express this (no ADC trait in 1.0; PWM is per-channel duty only), and the
  HAL crates do not fully wrap it, so this init and the per-loop edge use the PAC directly. Both HAL
  crates expose the raw PAC, so HAL cold path and PAC hot path coexist in one project.

## Configuration model

The only things fixed at compile time are what silicon forces: the PAC/HAL register family (F1x0 vs
F10x) and the memory map (flash/RAM size). That is one binary per MCU part. Everything else,
including the full motor wiring and the bridge/motor count, is **runtime config** stored in flash
and editable from the app, with no recompile to define a board.

This is made safe without a database of known boards. The firmware carries the per-MCU
alternate-function capability table (a property of the silicon, not the board), validates the
user's raw pin config against it (rejecting anything that is not a true complementary timer pair,
loudly, at boot), backs it with a hardware break/overcurrent net, and proves the wiring with a
current-limited commissioning self-test before arming. The detailed config schema, validation, and
safety/arming design are not yet specified.

## Math: fixed-point Q

No FPU on either part, so software float (tens to hundreds of cycles per op) is unacceptable in the
ADC-rate control loop. All control math is **fixed-point Q-format**, matching the reference FOC
firmware (generated as fixed-point for the same reason).

- The M3 has a single-cycle 32x32 multiply and hardware integer divide (`SDIV`/`UDIV`), so Q
  multiply-accumulate and scaling are cheap. It lacks the M4 DSP SIMD/saturating intrinsics, which
  are not needed.
- **Crate: `fixed` (recommended), pending confirmation vs `q-num`.** `fixed` is `no_std`, has
  type-level binary point (`I1F15`, `I16F16`, ...) and built-in saturating/wrapping arithmetic
  (overflow should saturate in a motor loop, not wrap). `q-num` is the literal ARM `Qm.n`-notation
  crate but thinner. There is no crate literally named `q`. See Open questions.
- **Trig** (`sin`/`cos` for Park / inverse-Park, `atan2` for attitude): sine lookup table (matches
  reference FOC, deterministic) or CORDIC (`cordic` crate, integrates with `fixed`). See Open
  questions.
- The math layer is orthogonal to the HAL: the crate choice does not touch the per-MCU seams.
- Drop `libm` (software float) from the hot path; keep it only for cold-path/calibration if at all.

## Module shape and seams

The architecture separates pure-logic modules (commutation, control, attitude, leds, scheduler,
...), which are math and state only, from a thin integration edge that mirrors modeled state onto
real I/O. Portability comes from keeping that split and making the edge the per-MCU boundary.

- **Pure logic**: shared, generic, no hardware. Reused verbatim across F130 and F103.
- **Hot-path edge**: small per-MCU glue (apply phase duties, read phase currents, read position).
  A trait here is optional and small; start with concrete per-MCU glue calling the shared math, and
  extract a trait only when host testing or the second implementation makes it pay off.
- **Trait seams** (signatures not fixed yet):
  `ClockInit`, `MotorDrive`, `CurrentSampler`, `HallInput`, `DigitalIo`, `SerialLink`,
  `ConfigStore`, `Timebase`. Commutation is **sensored**: hall-sensor input is required, so
  `HallInput` (rotor position from the halls) is a confirmed seam, not optional.
- **N motors, not one.** The hot-path seams (`MotorDrive`, `CurrentSampler`, `HallInput`) are driven
  from a runtime motor topology with one or more motors, so nothing is hardcoded to a single bridge.
  The F130C8/F103C8 case is N=1; the RCT6 12-FET board is N=2 on TIMER0+TIMER7.

## Known per-target divergences

These are the differences the multi-target approach forces into the design up front:

- **GPIO model**: F1x0 uses `CTL`/`AFSEL`, AHB-clocked; F10x uses `CRL`/`CRH`, APB2-clocked, with
  AFIO remap. Largest divergence.
- **ADC structure**: F1x0 has one ADC; F10x has two (dual/simultaneous sampling possible). Affects
  phase-current acquisition.
- **Flash/RAM map**: F130C8 = 64 KiB / 8 KiB; F103C8 = 64 KiB / 20 KiB; F103RCT6 = 256 KiB / 48 KiB.
  Affects config sector and linker script.
- **Advanced timers**: F130C8 and F103C8 have one (TIMER0), so single motor; the high-density
  F103RCT6 adds TIMER7, enabling the 12-FET dual-motor board.
- **Clock tree and timer register details**: same 72 MHz target, different setup.

## Open questions

- Fixed-point crate: `fixed` (recommended) vs `q-num`.
- Trig: sine LUT vs CORDIC.
- Hot-path edge: concrete per-MCU glue now vs a trait from the start.
- One F103 binary for both C8 and RCT6? The motor/timer count is runtime-gated by part detection,
  but `memory.x` is compile-time. A binary linked for the C8's 64K flash / 20K RAM could run on both
  and enable the second motor on the RCT6, if dual-motor fits in 64K/20K. If it does not fit, the
  RCT6 needs its own `memory.x` and stays a separate binary. Decide empirically once the dual-motor
  footprint is known; if it fits, prefer the single F103 binary.
