# Core architecture spec

Status: draft. Foundational decisions: the targets, the runtime HAL that lets one binary serve every
target, the memory model, the fixed-point math basis, the runtime configuration model, and the module
seams. Later specs (control, commutation, config, links) build on these.

## Targets

| Target | Family | Package | Flash / RAM | Motors | Advanced timers |
|---|---|---|---|---|---|
| GD32F130C8 | F1x0 | 48-pin | 64K / 8K | 1 | TIMER0 |
| GD32F103C8 | F10x | 48-pin | 64K / 20K | 1 | TIMER0 |
| GD32F103RCT6 | F10x (high-density) | 64-pin | 256K / 48K | 2 (12-FET dual) | TIMER0 + TIMER7 |

All three are committed, all Cortex-M3, no FPU. First bench bring-up is a GD32F103C8T6 bluepill; the
hoverboard controllers are the rest. The goal is one universal firmware (see README), and the
decision below is to take that literally: **one binary for all targets.**

## One binary, runtime HAL

The whole firmware is a single image that runs on every target. The board config (in flash) specifies
the MCU family and peripheral layout, and the runtime HAL is parameterized from it: peripheral
register access is dispatched to the right family layout per config. There is no chip auto-detection;
the config is the single source of truth for the hardware. This is what the project's "one binary"
promise means once you stop drawing the line at the MCU part.

The enabling fact: a real single-motor hoverboard firmware (the RoboDurden Gen2.x F103 build,
measured) uses about **4.3 KB of RAM**. So the smallest part's 8 KB is not the wall it looked like;
memory does not force per-part binaries. What is left is register-layout differences, which a runtime
HAL absorbs.

Consequences, recorded honestly:

- **We do not use the per-family community HALs** (`stm32f1xx-hal`, `gd32f1x0-hal`) in the production
  firmware. They are compile-time bound to one family's PAC. Instead we own a small HAL whose register
  access is parameterized by the family the config specifies. (The bring-up blinky used `stm32f1xx-hal`
  as a shortcut; production replaces it.)
- **We keep `embedded-hal` 1.0 traits as the public interface**, so external drivers (e.g. an IMU
  driver) still work against our GPIO/I2C/SPI.
- **Family from config, not detection**: the config carries the MCU family / peripheral layout, and the
  runtime HAL dispatches divergent peripheral ops on it (enum dispatch, no `dyn`, no alloc). Early boot
  runs on the reset-default clock until the config is read (flash is memory-mapped and reads identically
  across families); then clocks and peripherals are brought up per the config. A wrong family in config
  fails safe: the bridge will not arm until the commissioning self-test passes (see Configuration model).

What actually diverges, and how much:

- **GPIO**: large. F1x0 is AHB `CTL`/`AFSEL` at `0x48000000`; F10x is APB `CRL`/`CRH` at `0x40010800`.
  Two real code paths.
- **RCC/RCU clock enables**: different registers/bits. Two paths.
- **I2C / USART / SPI / timers / FMC**: shared (confirmed from the User Manuals). I2C is the classic
  event-based model (`CTL0`/`CTL1`/`STAT0`/`STAT1`/`CKCFG`/`RT`) on both families, no `TIMINGR`; the
  flash controller (FMC) is the same single-bank key/page-erase controller. One driver each,
  parameterized by base address, plus a flash page-size parameter (F1x0 and F10x-MD = 1 KB, F10x-HD
  RCT6 = 2 KB).
- **ADC**: the register core is shared, but F1x0 has one ADC (no dual mode, plus an oversampling
  register) while F10x has two ADCs with dual/simultaneous mode. The phase-current acquisition strategy
  differs, so this is two paths (single vs dual).
- **Interrupt vectors**: positions can differ per family. The vector table is populated with the union
  of positions used by any target; each handler fires only on the chip that uses that slot.

The cost we are taking on: we own the peripheral drivers (GPIO, RCC, timers, ADC, I2C, USART, SPI,
flash), including the finicky F1-style I2C; every supported family's divergent code ships in the one
image (small flash cost on 64K); a runtime branch per divergent op. The motor timer/ADC is the
safety-critical part and gets the most care, plus the arming/self-test safety model.

## Layered stack

1. **Target triple** `thumbv7m-none-eabi`. All targets are Cortex-M3, no FPU.
2. **`cortex-m` / `cortex-m-rt`** for core peripherals (NVIC, SysTick, SCB) and the reset/vector
   runtime. A single `memory.x` (see Memory model).
3. **Our runtime HAL**: config-selected family plus dispatched register access over our own register
   definitions, exposing `embedded-hal` traits at the top. No per-family PAC/HAL in production.
4. **`embedded-hal` 1.0 traits** as the seam (digital GPIO, SPI, I2C, PWM duty, delay; UART via
   `embedded-io`).
5. **Pure control-math layer** (fixed-point), MCU-independent, host-testable, bit-exact across targets.

Cold path vs hot path within the runtime HAL: the cold path (LEDs, buzzer, enable lines, the two UART
links, the IMU) is exposed through `embedded-hal` traits. The hot path (advanced-timer center-aligned
PWM, complementary outputs, dead-time, break, and timer-triggered injected ADC) is not expressible in
`embedded-hal`, so it is a direct register path inside the HAL, family-dispatched.

## Memory model

One binary, one static memory map, linked for the **smallest part**: F130C8, 64K flash / 8K RAM. All
statics and the default stack must fit there, and the linker enforces it.

- This is comfortable: the measured reference single-motor firmware is ~4.3 KB RAM. Dual-motor shares
  most state (remote, config, LEDs, buzzer, scheduler, IMU) and only duplicates per-motor control
  state, so it is expected to stay within 8 KB.
- Bigger parts' extra RAM/flash is unused by default. If ever needed (RCT6 dual-motor headroom), use it
  from the config-declared part profile: relocate the stack to the declared RAM top, and/or
  region-allocate extra buffers at init (never in the hot loop). Statics stay in the 8K-linked region.
  As with the family, this comes from config, not probing.
- Pretending the RAM is there (linking for a larger map) does not work: the reset stack pointer would
  point at nonexistent RAM and fault immediately, `cortex-m-rt` zeroes all of `.bss` at boot (faulting
  on any static above the real size), and the linker's overflow check is lost. So the static map stays
  at the smallest part.

## Configuration model

Board setup is runtime config in flash, editable from the app, no recompile to define a board. The
config is one self-contained binary blob with two parts:

- **Chip section**: per-subsystem **arch selectors** (which compiled code path to run for the divergent
  peripherals: gpio/af, clock/rcu, adc; the rest are shared), the peripheral **base addresses**, the
  memory and advanced-timer profile, and the chip **name** (identity only). No core field, the binary
  is already built for its core.
- **Board section**: the wiring in **logical** references (pins like `PA8`, peripherals like `TIMER0`),
  the motor topology (1..N), and tuning. The firmware resolves a logical pin against the chip section's
  base address and the selected path.

The blob is authored as JSON and compiled to the binary by a host tool; the firmware never parses JSON.
The **chip knowledge lives in that tool** (names, addresses, AF/capability tables), not on the device.
So validation is split: the tool validates the board against the chip's capability table at author time
(real complementary pairs, no duplicate pins, dead-time floor), and the firmware checks the structural
invariants it can from the blob, then relies on the current-limited commissioning self-test (and the
hardware break where the board has one) to catch a wrong-but-valid config before arming. There is no
chip auto-detection; the config is the single source of truth.

An arch selector can only name a path the firmware implements: the config picks among compiled code
paths, it does not describe new ones. A same-architecture chip is then pure data (new addresses plus
existing selectors); a genuinely new peripheral architecture, or a new core, needs new firmware code
and a new build.

## Math: fixed-point Q

No FPU, so software float (tens to hundreds of cycles per op) is unacceptable in the ADC-rate control
loop. All control math is **fixed-point Q-format**, matching the reference FOC firmware.

- The M3 has a single-cycle 32x32 multiply and hardware integer divide (`SDIV`/`UDIV`), so Q
  multiply-accumulate and scaling are cheap. It lacks the M4 DSP SIMD/saturating intrinsics, not needed.
- **Crate: `fixed` (recommended), pending confirmation vs `q-num`.** `fixed` is `no_std`, has type-level
  binary point (`I1F15`, `I16F16`, ...) and built-in saturating/wrapping arithmetic. `q-num` is the
  literal ARM `Qm.n`-notation crate but thinner. There is no crate literally named `q`.
- **Trig** (`sin`/`cos` for Park / inverse-Park, `atan2` for attitude): sine lookup table (matches
  reference FOC, deterministic) or CORDIC (`cordic` crate, integrates with `fixed`).
- The math layer is independent of the HAL. Drop `libm` (software float) from the hot path.

## Module shape and seams

Pure-logic modules (commutation, control, attitude, leds, scheduler, ...), which are math and state
only, are kept separate from a thin integration edge that mirrors modeled state onto real I/O. The
edge is the runtime HAL.

- **Pure logic**: shared, generic, no hardware. Reused verbatim across all targets.
- **Runtime HAL edge**: the family-dispatched register layer, exposing `embedded-hal` traits for the
  cold path and a direct register path for the hot path.
- **N motors, not one.** The hot-path code is driven from a runtime motor topology with one or more
  motors, so nothing is hardcoded to a single bridge. F130C8/F103C8 are N=1; the RCT6 is N=2 on
  TIMER0+TIMER7.
- Commutation is **sensored**: hall-sensor input is required, not optional.

## Known divergences the runtime HAL must bridge

- **GPIO model**: F1x0 AHB `CTL`/`AFSEL` vs F10x APB `CRL`/`CRH` + AFIO remap. Largest.
- **RCC/RCU clock-enable** registers and bits.
- **ADC structure**: F1x0 one ADC, F10x two.
- **Interrupt vector positions** per family (handled by populating the union).
- Per-part (not per-family): **memory size** (8/20/48 KiB) and **advanced-timer count** (1 vs 2, hence
  single vs dual motor).

## Open questions

- How the config encodes the MCU family / peripheral layout: a named profile (e.g. `F1x0` / `F10x`)
  vs explicit per-peripheral base addresses. Named profile is the likely answer.
- Interrupt vector unification: confirm the per-family IRQ position deltas and the union-table approach
  (a handler reads the config family to know which peripheral it serves).
- Register layer: our own minimal register definitions vs reusing a PAC's structs for the shared
  peripherals.
- Confirm I2C/USART/SPI/timers really need only base-address parameterization (check the bit-level
  deltas between F1x0 and F10x).
- Fixed-point crate: `fixed` vs `q-num`. Trig: sine LUT vs CORDIC.
- Finalize the per-MCU capability tables (FET/complementary pins, AF numbers) from the datasheets and
  Robo `HoverBoardGigaDevice`.
