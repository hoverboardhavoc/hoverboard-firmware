# Battery voltage sensing is a sensing-board-only function

Confirmed 2026-06-20 on two independent twin-board pairs while validating the runtime-hal
`adc_voltage` example: one board of a pair senses pack voltage on **PA4 (ADC channel 4)** through a
resistor divider; the other does **not** sense it locally and receives it over the inter-board
USART link.

## Boards measured

| pair | board | MCU | PA4 behaviour |
|---|---|---|---|
| bench (wired ST-Links) | sensing | GD32F103 | senses VBATT, tracks supply |
| bench | non-sensing | GD32F130 | stuck ~2.0 V (2463 counts), does not track |
| classywalk offroad (Wi-Fi) | sensing | GD32F130 | senses VBATT, tracks supply |
| classywalk offroad | non-sensing | GD32F130 | stuck ~2.0 V (2480 counts), does not track |

## Sensing board: divider is family-independent (~31x)

Reconstructing the divider from `factor = bus_mv * 4095 / (vbatt_raw * vdda_mv)` (VDDA measured live
from VREFINT, ch17, ~3.32 V):

| sensing board | point | vbatt_raw | counts/V | factor |
|---|---|---|---|---|
| F103 bench | 30.0 V | 1196 | 39.9 | 30.9x |
| F130 offroad | 35.0 V | 1392 | 39.8 | 31.0x |
| F130 offroad | 19.0 V | 758 | 39.9 | 31.0x |

The F130 sensing board tracked exactly across a 35 V -> 19 V sweep (predicted 756 counts at 19 V,
read 758), linear through the origin. F103 and F130 agree to ~0.3%, both within resistor tolerance
of the RoboDurden `ADC_BATTERY_VOLT` factor 30. **One divider constant (~30, VDDA-referenced) is
correct on either chip family** — no per-board / per-family calibration. That is why the runtime-hal
`adc_voltage` example uses a single `VBATT_DIVIDER = 30`.

## Non-sensing board: PA4 not on the battery

Two different non-sensing boards both park PA4 at a fixed ~2.0 V (2463 / 2480 counts) regardless of
bus voltage; the offroad one did not move under a sweep. The value is too consistent across boards
to be a floating pin, so PA4 is tied to a defined node, just not the battery divider. This matches
the firmware architecture (BalanceAgain `analysis/offroad_integration_map.md`): a `battery_voltage_filter`
writes the filtered pack voltage to shared RAM, and the peer value arrives over the RX `0x78` link
parser — the non-sensing board consumes the sensing board's reading. So it cannot run a standalone
battery monitor; the `adc_voltage` example only produces a real reading on a sensing board.

## Two ADC pitfalls that masked this (general GD32, now fixed in runtime-hal)

1. **Pin must be ANALOG before sampling.** Reset state of a GPIO is a *digital* input on BOTH
   families (F10x floating input; F1x0 `CTL` = input `0`), not analog; sampling through the live
   digital input buffer reads a clamped / stuck value. Stock firmware does
   `pinMode(VBATT, GPIO_MODE_ANALOG)`. Fixed: `Chip::analog_pin` / `gpio::configure_analog`.
2. **Clear EOC after each conversion.** A back-to-back second channel read returns the prior
   conversion's data if end-of-conversion is left set (the SPL's `adc_flag_clear`), so the second
   channel mirrors the first. Fixed in `Adc::read_channel`.

Flash procedure for the offroad pair: `reference/flashing-offroad-wireless.md`.
