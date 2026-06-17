# Core architecture spec

Status: draft. Foundational decisions: the targets, the dependency on the `runtime-hal` sibling crate
that lets one binary serve every target, the memory model, the fixed-point math basis, the runtime
configuration model, and the module seams. Later specs (control, commutation, config, links) build on
these.

## Targets

| Target | Family | Package | Flash / RAM | Motors | Advanced timers |
|---|---|---|---|---|---|
| GD32F130C8 | F1x0 | 48-pin | 64K / 8K | 1 | TIMER0 |
| GD32F103C8 | F10x | 48-pin | 64K / 8K | 1 | TIMER0 |
| GD32F103RCT6 | F10x (high-density) | 64-pin | 256K / 48K | 2 (12-FET dual) | TIMER0 + TIMER7 |

All three are committed, all Cortex-M3, no FPU. First bench bring-up is a GD32F103C8T6 bluepill; the
hoverboard controllers are the rest. The goal is one universal firmware (see README), and the decision
below is to take that literally: **one binary for all targets that share a core.**

## One binary, runtime HAL (the `runtime-hal` sibling crate)

The whole firmware is a single image that runs on every target of one core (Cortex-M3 here). The thing
that makes this possible is **`runtime-hal`**, a separate sibling crate (`../runtime-hal`) that this
firmware depends on. `runtime-hal` owns the MCU layer: it parses an **MCU descriptor** (the chip's code
paths, base addresses, and clock/memory profile, supplied at runtime as CBOR in flash) and dispatches
peripheral register access to the right family layout per that descriptor. There is no chip
auto-detection; the descriptor is the single source of truth for the silicon. This firmware does **not**
own register layouts, the chip database, or the family dispatch; that is entirely `runtime-hal`'s job.

This firmware is therefore a **consumer** of `runtime-hal`. It owns the board, the motor topology, the
control law, the links, and the board definition (see Configuration model). The chip/MCU half of the old
design has moved out into `runtime-hal`.

The enabling fact (recorded once, here): a real single-motor hoverboard firmware (the RoboDurden Gen2.x
F103 build, measured) uses about **4.3 KB of RAM**. So the smallest part's 8 KB is not the wall it
looked like; memory does not force per-part binaries. What is left is register-layout differences, which
`runtime-hal`'s runtime dispatch absorbs.

What `runtime-hal` provides this firmware:

- **Cold path** (LEDs, buzzer, enable lines, the two UART links, the IMU) behind `embedded-hal` 1.0 /
  `embedded-io` traits, so external drivers still work against our GPIO/I2C/SPI/serial.
- **Hot path** (advanced-timer center-aligned PWM, complementary outputs, dead-time, break, and
  timer-triggered injected ADC) behind `runtime-hal`'s own peripheral-capability traits, because the
  cross-peripheral timer-to-ADC synchronization is not expressible in `embedded-hal`.
- **Descriptor parsing**: given the config flash region, `runtime-hal` verifies the frame/CRC, decodes
  the CBOR into an `McuDescriptor`, validates selector-against-address, and configures the peripherals.

Why the divergences are not our problem anymore: the GD32 F1x0/F10x deltas (GPIO `CTL`/`AFSEL` vs
`CRL`/`CRH`, RCU vs RCC clock enables, one ADC vs two, per-family IRQ grouping and the RAM vector table)
all live behind `runtime-hal`'s path selectors. We name a chip in the descriptor; `runtime-hal` runs the
right code path. See [`../../runtime-hal/docs/SPEC.md`](../../runtime-hal/docs/SPEC.md) for the divergence
detail and the descriptor type.

We keep `embedded-hal` 1.0 traits (and `embedded-io` for serial) as the public seam, so reusable drivers
and the `control` crate never see a GD32. We do **not** use the per-family community HALs
(`stm32f1xx-hal`, `gd32f1x0-hal`) in production; they are compile-time bound to one family's PAC, which
the one-binary model rules out. (The bring-up blinky uses `stm32f1xx-hal` as a shortcut; production
replaces it with `runtime-hal`.)

## Layered stack

1. **Target triple** `thumbv7m-none-eabi`. All targets are Cortex-M3, no FPU.
2. **`cortex-m` / `cortex-m-rt`** for core peripherals (NVIC, SysTick, SCB) and the reset/vector runtime.
   A single `memory.x` (see Memory model).
3. **`runtime-hal`** (sibling crate): descriptor-selected family plus dispatched register access over its
   own register definitions, exposing `embedded-hal` traits at the top. No per-family PAC/HAL in
   production. This firmware depends on it by path (`runtime-hal = { path = "../../runtime-hal" }`).
4. **`embedded-hal` 1.0 traits** as the seam (digital GPIO, SPI, I2C, PWM duty, delay; UART via
   `embedded-io`).
5. **Pure control-math layer** (fixed-point), MCU-independent, host-testable, bit-exact across targets.

Cold path vs hot path are both `runtime-hal`'s, as above: the cold path through `embedded-hal`, the hot
path through `runtime-hal`'s timer/ADC capability traits. Nothing in `runtime-hal` is motor-specific; the
control law lives in this firmware's `control` crate, and the board semantics (which pin is a motor leg,
which is an LED) live in the board definition this firmware owns.

## Memory model

One binary, one static memory map, linked for the **smallest part**: F130C8, 64K flash / 8K RAM. All
statics and the default stack must fit there, and the linker enforces it.

- This is comfortable: the measured reference single-motor firmware is ~4.3 KB RAM. Dual-motor shares
  most state (remote, config, LEDs, buzzer, scheduler, IMU) and only duplicates per-motor control state,
  so it is expected to stay within 8 KB. `runtime-hal`'s descriptor + RAM vector table add a bounded,
  known cost (~100 bytes descriptor + ~300 bytes vector table).
- Bigger parts' extra RAM/flash is unused by default. If ever needed (RCT6 dual-motor headroom), use it
  from the descriptor-declared part profile: relocate the stack to the declared RAM top, and/or
  region-allocate extra buffers at init (never in the hot loop). Statics stay in the 8K-linked region.
  As with the family, this comes from the MCU descriptor, not probing.
- Pretending the RAM is there (linking for a larger map) does not work: the reset stack pointer would
  point at nonexistent RAM and fault immediately, `cortex-m-rt` zeroes all of `.bss` at boot (faulting on
  any static above the real size), and the linker's overflow check is lost. So the static map stays at
  the smallest part.

## Configuration model

Setup is runtime config in flash, editable from the app, no recompile to define a board. There are now
**two independent blobs**, each a framed canonical-CBOR document with a magic/version/length/CRC header:

- **MCU descriptor** (owned by `runtime-hal`, **not** by this firmware): per-subsystem arch selectors
  (the compiled code path for the divergent peripherals: gpio/af, clock/rcu, adc, irq), the peripheral
  base addresses, the memory + advanced-timer profile, and the chip name (identity only). This firmware
  hands the descriptor flash region to `runtime-hal::parse`, which returns a validated `McuDescriptor`.
  Authoring and the chip database for this blob live with `runtime-hal`'s config tool, not here. There is
  no `core` field; the binary is already built for its core.
- **Board definition** (owned by this firmware): the wiring in **logical** references (pins like `PA8`,
  peripherals like `TIMER0`), the motor topology (1..N), the link role, the control mode, and tuning. It
  carries **no chip section** at all: every chip fact (addresses, selectors, profile) comes from the MCU
  descriptor `runtime-hal` already parsed. The firmware resolves a logical board pin or peripheral
  against the `McuDescriptor`'s address table and selected path. See [../todo/board-config.md](../todo/board-config.md).

Both blobs are framed canonical CBOR (definite-length, minimal-width integers, no floats, bytewise
map-key ordering) behind CRC-16/MODBUS, the same discipline `runtime-hal` already uses for the MCU
descriptor. Neither blob is JSON, and the firmware never parses JSON; the board blob is authored from a
high-level schema and compiled to canonical CBOR by a host tool (board half) the same way `runtime-hal`'s
tool produces the MCU descriptor.

Validation is split across three places now:
- the **MCU descriptor** is validated by `runtime-hal` at parse (selector-vs-address consistency, frame/CRC);
- the **board definition** is validated at author time by the board tool against the chip's AF/capability
  table (real complementary pairs, no duplicate pins, dead-time floor, motor count within the descriptor's
  advanced-timer count) and at boot by the firmware for the structural invariants it can see from the blob;
- and the current-limited **commissioning self-test** (plus the hardware break where present) backstops a
  wrong-but-structurally-valid board before arming.

A board arch selector lives only in the MCU descriptor and can only name a path `runtime-hal` implements:
the config picks among compiled code paths, it does not describe new ones. A same-architecture chip is
pure data in the MCU descriptor (new addresses plus existing selectors); a genuinely new peripheral
architecture, or a new core, needs new `runtime-hal` code and a new build.

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

Pure-logic modules (commutation, control, attitude, leds, scheduler, ...), which are math and state only,
are kept separate from a thin integration edge that mirrors modeled state onto real I/O. The edge is
`runtime-hal`.

- **Pure logic**: shared, generic, no hardware. Reused verbatim across all targets. Lives in this
  firmware's crates (`control`, etc.).
- **Runtime HAL edge**: `runtime-hal`, the family-dispatched register layer, exposing `embedded-hal`
  traits for the cold path and its own timer/ADC capability traits for the hot path. External to this
  firmware.
- **Board definition**: this firmware's, mapping logical wiring (motor legs, halls, current-sense, LEDs,
  buzzer) onto the peripherals/pins the `McuDescriptor` resolves.
- **N motors, not one.** The hot-path code is driven from a runtime motor topology with one or more
  motors, so nothing is hardcoded to a single bridge. F130C8/F103C8 are N=1; the RCT6 is N=2 on
  TIMER0+TIMER7.
- Commutation is **sensored**: hall-sensor input is required, not optional.

## Where the chip/board line now sits

- **`runtime-hal` owns**: GD32 F1x0/F10x register models, the path selectors and base-address table, the
  RAM vector table, the clock/memory profile, and the MCU descriptor (parse + CBOR frame + CRC). One
  binary per core.
- **This firmware owns**: the board definition (motors, halls, current-sense, LEDs, buzzer, limits, link
  role, control mode) as its own CBOR blob, the control law, the links, and the safety/arming/commissioning
  logic.
- **Transitional note**: `runtime-hal`'s current M1 descriptor still carries a minimal `UsartWiring`
  record for its own inter-board USART bring-up. In the target split, link/board USART wiring belongs to
  this firmware's board definition; `runtime-hal` carving its M1 wiring out is tracked on its side.

## Open questions

- The board-definition CBOR schema and its versioning/migration story (the MCU descriptor schema is
  `runtime-hal`'s; this is the board half).
- How the board tool obtains the chip AF/capability table for author-time validation: shared from
  `runtime-hal`'s chip database vs a board-tool copy keyed by chip name.
- How the ADC channel for a current-sense pin is carried: resolved channel in the board definition vs a
  pin-to-channel map provided by `runtime-hal` from the descriptor.
- Fixed-point crate: `fixed` vs `q-num`. Trig: sine LUT vs CORDIC.
- Finalize the per-MCU capability tables (FET/complementary pins, AF numbers) from the datasheets and
  Robo `HoverBoardGigaDevice` (feeds the board tool's validation; the chip profiles feed `runtime-hal`).
