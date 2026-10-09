#!/usr/bin/env python3
"""Generate board-layout presets from RoboDurden's per-board `defines` files.

The requirement is `specs/robo-presets.md`. In one paragraph: the board layout editor
(`specs/rider-ui.md` 3.5) needs known-good field bundles per board variant, RoboDurden's firmware
repository is the most complete public record of hoverboard mainboard pinouts there is, and this
turns his per-layout pin maps into presets in OUR schema with OUR field ids, deterministically,
re-runnably against a newer upstream, and honest about what it could not map.

WHAT IT READS. A pristine clone of
`RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32`, in the gitignored `reference/` area and never in
this repository (the rule `specs/commutation.md` sets for GPL upstreams), checked out at a pinned
commit SHA with a clean working tree. "Latest" is not an input and a dirty tree is refused.

HOW IT READS IT. By C PREPROCESSOR EXPANSION of a key list we own, not by regex and not by the
objdump technique `~/dev/efferu-pin-dump/emit_pins.c` uses for EFeru. EFeru's pins are
`(port_base, pin_mask)` pairs only a compiler can resolve; RoboDurden's are tokens like `PC13`, and
the token IS the answer, so the preprocessor is the right tool. Two passes per layout:

  1. EXTRACTION. A translation unit that includes his `defines.h` with `TARGET` and `LAYOUT` set,
     then `#undef`s every pin token and emits one marker line per key we ask about, guarded so an
     absent key says so. `cc -E` resolves every `#if` the build would, including `defines.h`'s own
     post-processing (`BUTTON_PU` becoming `BUTTON`, `DISABLE_BUTTON` dropping `SELF_HOLD`) and
     every guard a layout file carries. His files hold commented-out defines, nested guards and
     keys defined in terms of other keys, all of which a regex gets wrong.
  2. CENSUS. `cc -E -dM` over the layout file alone, differenced against a baseline run with the
     same flags, which yields exactly the macros that FILE defines. Anything in it our key list
     does not ask about is reported, so an upstream addition becomes a diff rather than a silence.

His code never runs: nothing is executed, and the output is text the preprocessor produced.

THE PIN TOKENS ARE UNDEF'd ON PURPOSE. `target.h` defines `PA8` as `((uint32_t)GPIOA | 8)`, so a
marker emitted with those macros live would read as an address expression rather than `PA8`. The
extraction therefore undefines the pin tokens (and `TODO_PIN`) immediately before the markers, so
`HALL_A` expands to the token `PC13` and stops. `TODO_PIN` undefined is the same trick for a
different reason: his three files that mark a pin they do not know define `TODO_PIN PF4`, and
expanding it would turn "unknown" into a plausible PF4. Undefined, the marker reads `TODO_PIN` and
the preset carries the pin as UNKNOWN, which is the honesty this pipeline has to preserve.

HIS TREE CARRIES NO VENDOR SPL. `target.h` includes `gd32f1x0.h` / `gd32f10x.h`, which PlatformIO
supplies and the repository does not, so the extraction supplies EMPTY stubs for them. That also
keeps non-pin values symbolic (`TIMER_BLDC_PULLUP = GPIO_PUPD_NONE` rather than a vendor number),
which is what a human reading the census wants.

HIS config.h IS NEUTRALIZED, AND THE CONFIGURATION IS OURS. `-D CONFIG_H` makes his `config.h`
(which selects a layout, a remote and a role for HIS build) expand to nothing through its own include
guard, and this tool passes the configuration itself. That is the same principle as the key list
being ours: the generator decides what posture it is extracting, and records it in the output
(`expansion` in the emitted JSON). The posture is MASTER-or-single, because the facts that differ by
role are master facts (battery sense, and the buzzer on the layouts that put it on the slave), and
the layouts whose output changes under `SLAVE` are reported per preset rather than guessed at.

IN SCOPE: his targets 1 (GD32F130) and 2 (GD32F103). Targets 3 (GD32E230) and 4 (MM32SPIN05) are
parts this image cannot run on at all, so they are skipped DELIBERATELY and named in the output,
never silently dropped.

THE VERDICT. The generator finishes by running every preset through the board validator and
recording the verdict per preset. It asks the Kotlin mirror
(`protocol-kotlin/.../board/PresetVerdicts.kt`, which explains the choice) rather than the Rust:
`validate` is a pure function of the fields, the part's capabilities and the reserved set, and the
Kotlin module holds all three in ordinary code, while the Rust capability tables for the fleet's
parts are `#[cfg(test)]`-private. Using them from the host would mean writing a second capability
table for facts already modelled.

DETERMINISM is a property, not an intention: canonical key order, no timestamps, no paths, no
hostname, no toolchain identity, and generating twice from one SHA is byte-identical. That last one
is a test (`tools/tests/test_robo_presets.py`).

Usage:
  tools/robo-presets.py --clone reference/robo-gd32 --sha <40 hex> [--out FILE] [--no-verdicts]
  tools/robo-presets.py --selftest

stdlib only; `cc` for the expansion and `protocol-kotlin/gradlew` for the verdicts.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

# --------------------------------------------------------------------------------------------------
# Provenance
# --------------------------------------------------------------------------------------------------

#: Bumped when the SCHEMA or the MAPPING changes, so a consumer can tell one generation from another.
#: It is not a version of this file: a comment edit does not move it.
GENERATOR_VERSION = 1

#: The upstream the pins come from, named as a repository rather than a path (a path is machine
#: identity and would break determinism).
UPSTREAM = "RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32"

#: The documentation repository, which holds the layout photographs and PDF manuals. Split from the
#: firmware repository by RoboDurden himself because, in his words, it "has become to large".
DOCS_UPSTREAM = "RoboDurden/Hoverboard-Firmware-Hack-Gen2.x"

#: Where his layout files live inside the clone.
DEFINES_DIR = "HoverBoardGigaDevice/Inc/defines"

#: Where his `defines.h` lives, which is also the include root of the extraction.
INC_DIR = "HoverBoardGigaDevice/Inc"

# --------------------------------------------------------------------------------------------------
# His targets, and the parts of ours they are
# --------------------------------------------------------------------------------------------------

#: His TARGET numbering (`HoverBoardGigaDevice/Inc/config.h`), the chip each one is, and the parts of
#: that family our capability tables answer for. A target with no parts is out of scope: our image
#: cannot run on it, so a preset for one would be an invitation to brick a board.
#:
#: Target 2 lists TWO parts because his "GD32F103" spans packages: layout 2.2.7 is a Gen1 dual-motor
#: board whose halls sit on PC10-PC12 and whose second motor drives TIMER7, neither of which the
#: LQFP48 part has. Validating against every part of the family is what tells "this layout needs the
#: 64-pin part" apart from "this layout cannot be driven".
TARGETS = {
    1: {"chip": "GD32F130", "parts": ["F130C8"], "docs_dir": "target_1=GD32F130"},
    2: {"chip": "GD32F103", "parts": ["F103C8", "F103RC"], "docs_dir": "target_2=GD32F103"},
    3: {"chip": "GD32E230", "parts": [], "docs_dir": "target_3=GD32E230"},
    4: {"chip": "MM32SPIN05", "parts": [], "docs_dir": "target_4=MM32SPIN05"},
}

#: The non-layout files in his `defines` directory, and why each is skipped.
NOT_A_LAYOUT = {
    "defines_2-x-y.h": "his template: the pin values are placeholders for a new layout, not a board",
    "defines_2-ad.h": (
        "the REMOTE_AUTODETECT variant: defines.h includes it directly rather than through the "
        "TARGET/LAYOUT path, so it is a mode of his firmware and not a board layout"
    ),
}

# --------------------------------------------------------------------------------------------------
# The pin encoding we emit into
# --------------------------------------------------------------------------------------------------

#: The packed pin encoding (`specs/board-model.md`, "The field vocabulary"): one byte,
#: `(port << 4) | pin`, with no port E and nothing above F.
PORTS = {"A": 0, "B": 1, "C": 2, "D": 3, "F": 5}

#: `0xFF`: the function is absent on this board, which is a valid state and not an error.
PIN_ABSENT = 0xFF

_PIN_TOKEN = re.compile(r"^P([ABCDF])([0-9]{1,2})$")


def pack_pin(token):
    """The packed byte for a pin token like `PC13`, or None when the token names no pin."""
    m = _PIN_TOKEN.match(token)
    if not m:
        return None
    pin = int(m.group(2))
    if pin > 15:
        return None
    return (PORTS[m.group(1)] << 4) | pin


#: Every pin token `target.h` defines, which the extraction undefines before emitting its markers.
#: Built rather than listed: the ports and pin counts are the encoding's own, and his set is a subset.
PIN_TOKENS = ["P%s%d" % (p, n) for p in ("A", "B", "C") for n in range(16)] + [
    "PF%d" % n for n in (0, 1, 4, 6, 7)
]

#: Tokens undefined alongside the pins so the markers stay symbolic rather than expanding into
#: vendor numbers that only a real SPL header would give meaning to.
SYMBOLIC_TOKENS = [
    "TODO_PIN",  # his "I do not know this pin" marker; see the module docstring
    "GPIO_PUPD_NONE",
    "GPIO_PUPD_PULLUP",
    "GPIO_PUPD_PULLDOWN",
    "GPIO_MODE_AF_PP",
]

# --------------------------------------------------------------------------------------------------
# DECISION 1: his gate colour order against our phase order
# --------------------------------------------------------------------------------------------------

#: He names gates by motor wire colour (G green, B blue, Y yellow); we name them by phase (A, B, C).
#: Getting this wrong swaps two phases, which on a first arm is a board that jerks or trips rather
#: than spins, so it is established against three independent sources rather than assumed:
#:
#:   1. THE BENCH PAIR'S OWN WORKING CONFIGURATION, which is the oracle `specs/robo-presets.md` names.
#:      `defines_2-2-20.h` (the F103 master) and `defines_2-1-20.h` (the F130 slave) are the files our
#:      working pin map came from, and both read `BLDC_YH PA8`, `BLDC_BH PA9`, `BLDC_GH PA10` with
#:      `BLDC_YL PB13`, `BLDC_BL PB14`, `BLDC_GL PB15`. Our map for those boards
#:      (`specs/board-model.md` section 4, mirrored in `protocol-kotlin/.../LayoutPresets.kt`) is
#:      `gate_hi_a PA8`, `gate_hi_b PA9`, `gate_hi_c PA10`, `gate_lo_a PB13`, `gate_lo_b PB14`,
#:      `gate_lo_c PB15`. So A = yellow, B = blue, C = green.
#:   2. HIS OWN CHANNEL CONVENTION. `HoverBoardGigaDevice/Inc/defines.h` fixes
#:      `TIMER_BLDC_CHANNEL_Y TIMER_CH_0`, `..._B TIMER_CH_1`, `..._G TIMER_CH_2` for every Gen2
#:      board, and TIMER0's channels 0/1/2 are PA8/PA9/PA10, which our gate set takes in that order.
#:   3. HIS LAYOUT 2.2.1, which spells the pairing out field by field in the layout file itself:
#:      `TIMER_BLDC_CHANNEL_G TIMER_CH_2` beside `TIMER_BLDC_GH_PIN GPIO_PIN_10`, and so on.
#:
#: The mapping is a CONVENTION, and his colour labels are not always consistent with it: layout 2.1.3
#: puts green on channel 0 and yellow on channel 2 (with his own channel defines commented out
#: alongside, contradicting the pins), and 2.2.7's left motor reverses the order its right motor
#: uses. Those layouts are REFUSED by our validator with `InvalidGateSet`, which is the honest
#: outcome: reordering his pins to satisfy the timer would silently permute the phases against the
#: halls, which is the wrong commutation the decision exists to prevent. His own commit "Autodetect
#: now works with wrong bldc colors" says the labels are unreliable; the refusals say which.
GATE_COLOUR_FOR_PHASE = {"a": "Y", "b": "B", "c": "G"}

# --------------------------------------------------------------------------------------------------
# DECISION 2: ADC_BATTERY_VOLT to board.vbatt_cal
# --------------------------------------------------------------------------------------------------

#: The conversion, done once and recorded here.
#:
#: His `ADC_BATTERY_VOLT` is VOLTS PER ADC COUNT at the pack: 0.02507 on the bench layout, a divider
#: of about 30 against a 3.3 V reference over a 12-bit converter. Ours is `board.vbatt_cal`
#: (id 0x69, `specs/sensing-and-safety.md`, "The battery word"): index 0 is the slope in MICROVOLTS
#: PER COUNT and index 1 an offset in CENTIVOLTS, read as
#: `centivolts = raw12 * slope / 10_000 + offset`.
#:
#: Volts per count to microvolts per count is a factor of 1e6, and the division by 10_000 in our
#: formula is what turns microvolts back into centivolts (1 cV = 10_000 uV), so the slope is simply
#:
#:     slope_uV_per_count = round(ADC_BATTERY_VOLT * 1_000_000)
#:
#: 0.02507 V/count becomes 25070 uV/count, against the registry default of 25200 from the
#: 2026-10-08 four-point fit on the bench master: 0.5% apart, which is the check that the units are
#: right rather than merely plausible.
#:
#: THE OFFSET IS ZERO BECAUSE HIS MODEL HAS NO OFFSET TERM: his conversion is a pure gain. That is a
#: statement about his model and not a missing value, so the preset carries both indices with the
#: offset at zero rather than leaving half the field for a later patch to fill. Our own default
#: offset of -5 cV came from a measured fit and is NOT mixed in: a preset that took his slope and
#: our offset would be a calibration neither of us measured.
#:
#: AND IT IS A MASTER-ONLY FACT. `VBATT` senses the pack only on a master board; a slave's PA4 sits
#: at a fictitious 2.0 V (memory: battery sense is master-only; `specs/sensing-and-safety.md`), so a
#: slave carrying this calibration is noise at best. Every preset says so in its notes, and a slave
#: takes the battery over the link.
VBATT_CAL_FIELD = 0x69
VBATT_CAL_SLOPE_PER_VOLT = 1_000_000

#: The i16 the field holds. A slope outside it is a finding, not a value to truncate.
I16_MAX = 32767


def vbatt_cal_slope(volts_per_count):
    """The `board.vbatt_cal` index-0 slope, microvolts per count, for his volts-per-count figure."""
    return int(round(volts_per_count * VBATT_CAL_SLOPE_PER_VOLT))


# --------------------------------------------------------------------------------------------------
# DECISION 3: the IMU bus
# --------------------------------------------------------------------------------------------------

#: He expresses the IMU bus as a pair FLAG (`I2C_PB6PB7`, `I2C_PB8PB9`) rather than as pins, and
#: `defines_2-1-20.h`'s own comment says the IMU needs soldering to the tiny module on PB6/PB7,
#: which is also where `USART0` can live. The flag is therefore a claim about a SOLDERABLE OPTION
#: and not a fitted bus, so the presets leave `imu.scl_pin`, `imu.sda_pin` and `imu.model` ABSENT
#: rather than asserting them, and say so in the preset.
#:
#: Two more reasons it would be wrong to assert them. Our IMU group is all-or-none (both pins plus a
#: nonzero model), and the MODEL is a fact about the part someone soldered on, which no layout file
#: can state: the bench board's own fitted part answers `WHO_AM_I` 0x2E, not the MPU-6050 his define
#: implies. And `I2C_PB8PB9`, which two of his layouts carry, is not a pair our capability table
#: recognises as a hardware I2C instance at all (we answer PB6/PB7 = I2C0 and PB10/PB11 = I2C1), so
#: asserting it would produce a refusal manufactured by us rather than a fact about his board.
IMU_ABSENT_REASON = (
    "his layout states the IMU bus as a solderable pair flag, not a fitted bus, and the IMU model "
    "is a fact about the part soldered on that no layout file carries: staged by hand or by a "
    "per-board preset instead"
)

# --------------------------------------------------------------------------------------------------
# DECISION 4: the dead time, which the spec did not name
# --------------------------------------------------------------------------------------------------

#: `motor.dead_time` is required at or above `DEAD_TIME_MIN_DTG` (18) wherever a gate group is
#: configured (`crates/board/src/lib.rs`, `DEAD_TIME_MIN_DTG`), so a preset that omitted it would be
#: refused on the dead time before any check that could say something about HIS file: the group
#: completeness check runs before the capability stage, and the first failure wins. Every preset with
#: gates would read `DeadTimeBelowFloor` and the exercise would report nothing.
#:
#: His dead time is a single figure in `defines.h` rather than per-layout data: `DEAD_TIME 60`, with
#: his comment "60 = 1us, measured by oscilloscope". `HoverBoardGigaDevice/Src/setup.c` programs it
#: into TIMER0's break register with `clockdivision = TIMER_CKDIV_DIV1`, so one of his ticks is
#: `1 / 72 MHz` = 13.9 ns and 60 of them are 833 ns. Our field is the same register's DTG at the
#: `/2` divider our bring-up configures, where one tick is 27.8 ns, so his measured dead time is
#:
#:     round(60 * (1/72e6) / (2/72e6)) = 30 ticks
#:
#: 30 is above our floor of 18 and below the 32 the 12-FET runs, so it validates and it is safe.
#: It is HIS board-independent figure re-expressed for our divider, NOT a per-board measurement, and
#: the bench boards' own silicon-proven value is 25 (`specs/commutation.md`). Every preset says so.
DEAD_TIME_FIELD = 0x64
HIS_DEAD_TIME_DIVIDER = 1
OUR_DEAD_TIME_DIVIDER = 2


def dead_time_dtg(his_ticks):
    """His `DEAD_TIME` ticks re-expressed for the divider our gate timer runs."""
    return int(round(his_ticks * HIS_DEAD_TIME_DIVIDER / OUR_DEAD_TIME_DIVIDER))


# --------------------------------------------------------------------------------------------------
# The key list, which is OURS
# --------------------------------------------------------------------------------------------------

#: The fields a preset carries, in registry id order (which is the canonical order the output uses),
#: each with the macro of his it comes from.
#:
#: `motor` is the motor index for a per-motor field. `kind` is how the value is read:
#:   "pin"     the marker's token is a pin, packed into the field byte
#:   "derived" the generator computes it; the entry names what from
#:
#: The three fields the validator carries WITHOUT judging (`motor.direction`,
#: `motor.align_offset`, `motor.current_cal`) are deliberately absent from this table, and so are
#: machine-policy fields. A preset is board facts only (`specs/rider-ui.md` 3.7), and those three are
#: not facts a pin map can state: the drive direction is a hall-and-phase wiring fact, the align
#: offset is bench-swept, and the calibration belongs to the shunt chain fitted. The existing
#: hand-written presets take them from the board for the same reason
#: (`protocol-kotlin/.../LayoutPresets.kt`, `LayoutPreset.applyTo`).
FIELDS = [
    {"id": 0x40, "key": "board.self_hold", "kind": "pin", "frm": "SELF_HOLD"},
    {"id": 0x41, "key": "board.vbatt", "kind": "pin", "frm": "VBATT"},
    {"id": 0x42, "key": "board.buzzer", "kind": "pin", "frm": "BUZZER"},
    {"id": 0x43, "key": "led.green", "kind": "pin", "frm": "LED_GREEN"},
    {"id": 0x44, "key": "led.orange", "kind": "pin", "frm": "LED_ORANGE"},
    {"id": 0x45, "key": "led.red", "kind": "pin", "frm": "LED_RED"},
    {"id": 0x46, "key": "pad.a", "kind": "pin", "frm": "PHOTO_L"},
    {"id": 0x47, "key": "pad.b", "kind": "pin", "frm": "PHOTO_R"},
    {"id": 0x48, "key": "imu.scl_pin", "kind": "decision3"},
    {"id": 0x49, "key": "imu.sda_pin", "kind": "decision3"},
    {"id": 0x4A, "key": "motor.hall_a", "kind": "pin", "frm": "HALL_A", "motor": 0},
    {"id": 0x4B, "key": "motor.hall_b", "kind": "pin", "frm": "HALL_B", "motor": 0},
    {"id": 0x4C, "key": "motor.hall_c", "kind": "pin", "frm": "HALL_C", "motor": 0},
    {"id": 0x4D, "key": "motor.gate_hi_a", "kind": "pin", "frm": "BLDC_YH", "motor": 0},
    {"id": 0x4E, "key": "motor.gate_hi_b", "kind": "pin", "frm": "BLDC_BH", "motor": 0},
    {"id": 0x4F, "key": "motor.gate_hi_c", "kind": "pin", "frm": "BLDC_GH", "motor": 0},
    {"id": 0x50, "key": "motor.gate_lo_a", "kind": "pin", "frm": "BLDC_YL", "motor": 0},
    {"id": 0x51, "key": "motor.gate_lo_b", "kind": "pin", "frm": "BLDC_BL", "motor": 0},
    {"id": 0x52, "key": "motor.gate_lo_c", "kind": "pin", "frm": "BLDC_GL", "motor": 0},
    {"id": 0x53, "key": "board.button", "kind": "pin", "frm": "BUTTON"},
    {"id": 0x54, "key": "motor.phase_a", "kind": "pin", "frm": "PHASE_A", "motor": 0},
    {"id": 0x55, "key": "motor.phase_b", "kind": "pin", "frm": "PHASE_B", "motor": 0},
    {"id": 0x60, "key": "imu.model", "kind": "decision3"},
    {"id": 0x64, "key": "motor.dead_time", "kind": "dead_time", "frm": "DEAD_TIME", "motor": 0},
    {"id": 0x66, "key": "motor.current_sense", "kind": "current_sense", "motor": 0},
    {"id": 0x69, "key": "board.vbatt_cal", "kind": "vbatt_cal", "frm": "ADC_BATTERY_VOLT"},
]

#: The one field above the board validator does not judge, so it is not sent to the verdict oracle:
#: `board.vbatt_cal` is a calibration the sensing producer owns, not a layout slot
#: (`protocol-kotlin/.../LayoutSlots.kt`, `Layout`).
NOT_VALIDATED = {VBATT_CAL_FIELD}

#: The keys of his we ASK ABOUT and do not consume, each with why. Recording them is half the point
#: of owning the key list: every one is either a field we should add or a fact we have decided not to
#: model, and leaving them out would lose the only census of them we will ever get cheaply.
UNMAPPED_KEYS = {
    "BLDC_GH": None,  # consumed, listed so the census is complete
    "TIMER_BLDC_PULLUP": "no field: the gate pins' pull configuration, which our HAL owns",
    "TIMER_BLDC_EMERGENCY_SHUTDOWN": "no field: the TIMER0 break input, which our bring-up does not use",
    "TIMER_BLDC": "no field: which timer drives the gates, which our capability query derives",
    "RCU_TIMER_BLDC": "no field: the gate timer's clock enable, which our HAL owns",
    "TIMER_BLDC_CHANNEL_G": "no field: his colour-to-channel convention, which decision 1 reads rather than stores",
    "TIMER_BLDC_CHANNEL_B": "no field: his colour-to-channel convention, which decision 1 reads rather than stores",
    "TIMER_BLDC_CHANNEL_Y": "no field: his colour-to-channel convention, which decision 1 reads rather than stores",
    "CURRENT_DC": "no field: we sense per phase, not on the DC bus",
    "PHASE_C": "no field: our phase-current group is the injected group's two ranks, A and B only",
    "MOTOR_AMP_CONV_DC_AMP": (
        "no field yet: amps per count on the DC-BUS shunt chain, a different quantity from "
        "motor.current_cal (0x67), which is counts per amp on the PER-PHASE chain. Not convertible "
        "between chains, so it stays open"
    ),
    "UPPER_LED": "no field: a second LED bank",
    "UPPER_LED2": "no field: a second LED bank",
    "LOWER_LED": "no field: a second LED bank",
    "MOSFET_OUT": "no field: his onboard indicator LED output",
    "CHARGE_STATE": "no field: the charger-present sense",
    "BUTTON_PU": "no field: the button's pull direction (its PIN reaches board.button through his BUTTON alias)",
    "HAS_BUZZER": "not a pin: his own guard for the layouts that put the buzzer on the slave board",
    "DEBUG_PIN": "no field: a debug output his own comment says nothing uses any more",
    "DATA_PIN": "no field: a single-wire header pin of unknown function",
    "TODO_PIN": "not a pin: his marker for a pin he does not know",
    "I2C_PB6PB7": "decision 3: a solderable-option flag, not a fitted bus",
    "I2C_PB8PB9": (
        "decision 3, and PB8/PB9 is not a pair our capability table answers as a hardware I2C "
        "instance (we answer PB6/PB7 = I2C0, PB10/PB11 = I2C1)"
    ),
    "I2C0_SCL": "decision 3: a bus pin stated outside his pair-flag form",
    "I2C0_SDA": "decision 3: a bus pin stated outside his pair-flag form",
    "MPU_6050": "decision 3: an IMU model claim a layout file cannot make good on",
    "MPU_6050old": "decision 3: an IMU model claim a layout file cannot make good on",
    "MPU_6500": "decision 3: an IMU model claim a layout file cannot make good on",
    "BMI_160": "decision 3: an IMU model claim a layout file cannot make good on",
    "USART0_TX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "USART0_RX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "USART1_TX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "USART1_RX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "USART2_TX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "USART2_RX": "not a layout field: the link ports are a compiled safety fact, reached through LINK_SET",
    "HAS_USART0": "not a pin: his own declaration that the layout has that USART",
    "HAS_USART1": "not a pin: his own declaration that the layout has that USART",
    "HAS_USART2": "not a pin: his own declaration that the layout has that USART",
    "USART0_REMOTE": "not a pin: his choice of which USART carries the remote",
    "USART1_REMOTE": "not a pin: his choice of which USART carries the remote",
    "USART2_REMOTE": "not a pin: his choice of which USART carries the remote",
    "USART_STEER_COM": "not a pin: his pre-2025 spelling of the steering USART",
    "USART_STEER_COM_TX_PORT": "not a pin: his pre-2025 port/pin spelling of a USART pin",
    "USART_STEER_COM_TX_PIN": "not a pin: his pre-2025 port/pin spelling of a USART pin",
    "USART_STEER_COM_RX_PORT": "not a pin: his pre-2025 port/pin spelling of a USART pin",
    "USART_STEER_COM_RX_PIN": "not a pin: his pre-2025 port/pin spelling of a USART pin",
    "PHOTO_L_PIN": "not a pin token: his pre-2025 port/pin spelling of PHOTO_L",
    "PHOTO_R_PIN": "not a pin token: his pre-2025 port/pin spelling of PHOTO_R",
    "PHASE_MATCH": "not a pin: his autodetect's hall-to-phase permutation result",
    "PHASE_CURRENT_G": "no field: his colour-named phase-current pin, added by the autodetect work",
    "PHASE_CURRENT_B": "no field: his colour-named phase-current pin, added by the autodetect work",
    "PHASE_CURRENT_Y": "no field: his colour-named phase-current pin, added by the autodetect work",
    "MOTOR_LEFT": "not a pin: his switch for which motor of a dual-motor board the build drives",
    "STM32F103": "not a pin: his declaration that the board carries an STM32 rather than a GD32",
    "PIN_PACKAGE": "not a pin: the package pin count, which our capability table answers per part",
    "TIMER_TIMEOUT": "no field: his timeout timer, which our scheduler owns",
    "TIMER_TIMEOUT_IRQn": "no field: his timeout timer's interrupt",
    "RCU_TIMER_TIMEOUT": "no field: his timeout timer's clock enable",
    "TIMEOUT_IrqHandler": "no field: his timeout timer's handler name",
    "PIDINIT_a3o": "no field: his PID gains, which are tuning and not board data",
}
for _colour in ("G", "B", "Y"):
    for _side in ("H", "L"):
        UNMAPPED_KEYS.setdefault("BLDC_%s%s" % (_colour, _side), None)
        UNMAPPED_KEYS["TIMER_BLDC_%s%s_PORT" % (_colour, _side)] = (
            "not a pin token: his pre-2025 port/pin spelling of a gate pin"
        )
        UNMAPPED_KEYS["TIMER_BLDC_%s%s_PIN" % (_colour, _side)] = (
            "not a pin token: his pre-2025 port/pin spelling of a gate pin"
        )

#: A macro a layout file defines whose name starts with this is its own include guard, which is not
#: board data. Stated as a rule rather than listed, because his guard names vary per file
#: (`DEFINES_2_1_20_H`, `DEFINES_2_x_H`, `DEFINES_2_10_H`) and a new layout would otherwise report
#: its guard as an upstream addition on the day it arrives.
INCLUDE_GUARD_PREFIX = "DEFINES_"


def asked_keys():
    """Every macro name the extraction asks about: the mapped fields' plus the known-unmapped."""
    names = set(UNMAPPED_KEYS)
    for f in FIELDS:
        if f.get("frm"):
            names.add(f["frm"])
    return sorted(names)


# --------------------------------------------------------------------------------------------------
# The link set, and the reserved pins behind it
# --------------------------------------------------------------------------------------------------

#: The compiled safe-USART allowlist as the firmware declares it, mirrored from
#: `protocol-kotlin/.../ReservedPins.kt`: each wiring's `LINK_SET` bit and its two pins. The link's
#: own pins are deliberately NOT layout fields (the allowlist is what lets a BLANK board probe for a
#: controller before any field can exist), so this is how a layout is kept off them.
ALLOWLIST = [
    {"bit": 3, "pins": ("PB6", "PB7")},
    {"bit": 1, "pins": ("PA2", "PA3")},
    {"bit": 2, "pins": ("PB10", "PB11")},
]

#: His USART pin-pair macros, in the order a layout declares them.
HIS_USARTS = [("USART0_TX", "USART0_RX"), ("USART1_TX", "USART1_RX"), ("USART2_TX", "USART2_RX")]


def link_set_from_usarts(values):
    """The `LINK_SET` mask a layout implies, and the basis for each bit.

    A board's live link ports are at most the USART wirings its layout declares, so a wiring his file
    does not mention cannot be one and its pins are free for layout fields. The derivation is
    therefore: set an allowlist wiring's bit when some `USART<n>_TX`/`_RX` pair his layout declares
    is that wiring's pin pair (unordered, because which of the two is TX is his wiring's business).

    An EMPTY basis is the statement that his layout declares no USART on a wiring our allowlist
    carries, which leaves the mask at 0: the firmware reads that as UNCONFIGURED and probes every
    routable allowlist port, so the validator then reserves them all against the layout. That is the
    correct reading of "we do not know which ports are live", and the basis beside the mask is how a
    reader tells it from a mask that was derived and came out empty.

    Routability is NOT applied here. It is the part's answer, not the layout's, and `reservedSet`
    already asks it per part: PB6/PB7 reaches a USART only on the F1x0 and PB10/PB11 only on the
    F10x, so the same mask yields different reserved sets on the two families, which is correct.
    """
    declared = []
    for tx, rx in HIS_USARTS:
        a, b = values.get(tx), values.get(rx)
        if a is not None and b is not None:
            declared.append(({a, b}, (tx, rx)))
    mask, basis = 0, []
    for entry in ALLOWLIST:
        for pins, macros in declared:
            if pins == set(entry["pins"]):
                mask |= 1 << entry["bit"]
                basis.append(
                    {"bit": entry["bit"], "pins": list(entry["pins"]), "from": list(macros)}
                )
                break
    return mask, basis


# --------------------------------------------------------------------------------------------------
# The expansion: what configuration we feed his preprocessor
# --------------------------------------------------------------------------------------------------

#: The posture every preset is extracted in. MASTER-or-single, because the role-dependent facts in
#: his files are master facts: `HAS_BUZZER` is guarded on `MASTER_OR_SINGLE` in most layouts (and on
#: `SLAVE` in the three that put the buzzer on the slave board), and battery sense is master-only on
#: this hardware anyway. A generated SLAVE preset would also need facts his files do not carry (the
#: phase-pin order differs between the bench pair's two halves, and the slave takes the battery over
#: the link), so it would be a half-state rather than a preset; the layouts whose extraction CHANGES
#: under `SLAVE` are reported per preset instead.
ROLE_DEFINES = ["MASTER_OR_SINGLE"]

#: The role posture compared against, to report which layouts carry role-dependent board facts.
#: `MASTERSLAVE_USART` rides along because `defines.h` `#error`s on a MASTER or SLAVE build with no
#: inter-board USART selected, which his own `config.h` selects for exactly those two roles.
ROLE_ALTERNATE = ["SLAVE", "MASTERSLAVE_USART=1"]

#: The vendor SPL headers `target.h` includes and his repository does not carry. Stubbed EMPTY: the
#: extraction needs the layout tokens, not SPL semantics, and empty stubs are also what keeps the
#: non-pin values symbolic.
SPL_STUBS = [
    "gd32f1x0.h",
    "gd32f1x0_gpio.h",
    "gd32f1x0_exti.h",
    "gd32f1x0_rcu.h",
    "gd32f10x.h",
    "gd32e23x.h",
    "mm32_device.h",
    "hal_conf.h",
    "hal_device.h",
    "hal_rcc.h",
]

#: The configuration switches that change what a layout file expands to, each swept over every value
#: so no text scanning decides which layouts have variants. A layout whose extraction is the same for
#: every combination yields ONE preset; one that differs yields a preset per distinct result, named
#: by the switch values that differ.
#:
#: `LAYOUT_SUB` is his own sub-layout numbering, which he names in `config.h` ("Layout 2.1.7 exists
#: as 2.1.7.0 and 2.1.7.1"); the suffix is therefore his designation. `MOTOR_LEFT` is the switch
#: layout 2.2.7, a Gen1 dual-motor board, uses to choose WHICH of its two motors the build drives,
#: with his comment reading "motor right chosen" for the default.
VARIANT_AXES = [
    {"macro": "LAYOUT_SUB", "values": [("0", "0"), ("1", "1")], "join": "."},
    {"macro": "MOTOR_LEFT", "values": [(None, "right"), ("1", "left")], "join": "-"},
]


def variant_combos():
    """Every combination of the variant axes, in a fixed order."""
    combos = [[]]
    for axis in VARIANT_AXES:
        combos = [c + [(axis, v)] for c in combos for v in axis["values"]]
    return combos


# --------------------------------------------------------------------------------------------------
# The two preprocessor passes
# --------------------------------------------------------------------------------------------------

_MARK_VALUE = re.compile(r'^@@K "([A-Za-z_][A-Za-z0-9_]*)" = \|(.*)\|\s*$')
_MARK_ABSENT = re.compile(r'^@@A "([A-Za-z_][A-Za-z0-9_]*)"\s*$')
_DEFINE = re.compile(r"^#define ([A-Za-z_][A-Za-z0-9_]*)")


def _stub_dir(root):
    """A directory of empty vendor SPL headers, created under `root`."""
    path = os.path.join(root, "stubs")
    os.makedirs(path, exist_ok=True)
    for name in SPL_STUBS:
        open(os.path.join(path, name), "w").close()
    return path


def _flags(clone, stubs, target, layout, extra):
    """The preprocessor flags for one expansion.

    `-D CONFIG_H` neutralizes his `config.h` through its own include guard, so the configuration is
    ours and nothing of his fights us over `LAYOUT`. `-D GD32F103` is needed on target 2 because
    `target.h` selects its per-chip block on that name (target 1 is its `#else` branch, and the name
    is defined for symmetry).
    """
    chip = {1: "GD32F130", 2: "GD32F103", 3: "GD32E230", 4: "MM32SPIN05"}[target]
    flags = [
        "-I",
        os.path.join(clone, INC_DIR),
        "-I",
        stubs,
        "-D",
        "CONFIG_H",
        "-D",
        "TARGET=%d" % target,
        "-D",
        "LAYOUT=%s" % layout,
        "-D",
        chip,
    ]
    for d in extra:
        flags += ["-D", d]
    return flags


def _run_cc(flags, source, work):
    """`cc -E -P` over `source`, returning its stdout. A failure carries the compiler's own words."""
    path = os.path.join(work, "tu.c")
    with open(path, "w") as fh:
        fh.write(source)
    proc = subprocess.run(
        ["cc", "-E", "-P"] + flags + [path], capture_output=True, text=True, check=False
    )
    if proc.returncode != 0:
        raise RuntimeError("cc -E failed:\n%s" % proc.stderr.strip())
    return proc.stdout


def extract(clone, stubs, work, target, layout, extra):
    """The value of every asked key for one (target, layout, configuration).

    Returns `{key: value}` for the keys the expansion found defined (the value is the token, or the
    empty string for a flag-style define), with absent keys simply missing from the mapping.
    """
    lines = ['#include "defines.h"']
    for token in PIN_TOKENS + SYMBOLIC_TOKENS:
        lines.append("#undef %s" % token)
    for key in asked_keys():
        lines += [
            "#ifdef %s" % key,
            '@@K "%s" = |%s|' % (key, key),
            "#else",
            '@@A "%s"' % key,
            "#endif",
        ]
    out = _run_cc(_flags(clone, stubs, target, layout, extra), "\n".join(lines) + "\n", work)
    values, seen = {}, set()
    for line in out.splitlines():
        m = _MARK_VALUE.match(line.strip())
        if m:
            values[m.group(1)] = m.group(2).strip()
            seen.add(m.group(1))
            continue
        m = _MARK_ABSENT.match(line.strip())
        if m:
            seen.add(m.group(1))
    missing = set(asked_keys()) - seen
    if missing:
        raise RuntimeError("the expansion lost markers for %s" % sorted(missing))
    return values


def census(clone, stubs, work, target, layout, extra, include_path):
    """Every macro the LAYOUT FILE itself defines, by `cc -E -dM` differenced against a baseline.

    The second pass of the method, and the reason an upstream addition becomes a diff rather than a
    silence. The baseline run carries the same flags, so the generator's own `-D`s cancel and what is
    left is exactly what that file defines under this configuration.
    """
    flags = _flags(clone, stubs, target, layout, extra)

    def defines(source, name):
        path = os.path.join(work, name)
        with open(path, "w") as fh:
            fh.write(source)
        proc = subprocess.run(
            ["cc", "-E", "-dM"] + flags + [path], capture_output=True, text=True, check=False
        )
        if proc.returncode != 0:
            raise RuntimeError("cc -E -dM failed:\n%s" % proc.stderr.strip())
        return {m.group(1) for m in (_DEFINE.match(l) for l in proc.stdout.splitlines()) if m}

    base = defines("", "census_base.c")
    full = defines('#include "%s"\n' % include_path, "census_full.c")
    return sorted(full - base)


# --------------------------------------------------------------------------------------------------
# One layout to one preset
# --------------------------------------------------------------------------------------------------

#: What a field's value is when his file routes it through `TODO_PIN`: a pin he does not know. It
#: must travel as UNKNOWN and never as a plausible default, and the editor must refuse to apply a
#: preset with an unknown pin in a field the validator requires.
UNKNOWN_PIN = "TODO_PIN"


def gates_complete(values):
    """Does this layout give all six gate pins as encodable pins?

    The dead time is only emitted where it is, because a CONFIGURED gate group is what the floor
    applies to and zero is the correct dead time for a board with no motor
    (`crates/board/src/lib.rs`, `DEAD_TIME_MIN_DTG`).
    """
    macros = [
        "BLDC_%s%s" % (GATE_COLOUR_FOR_PHASE[p], s) for s in ("H", "L") for p in ("a", "b", "c")
    ]
    return all(pack_pin(values.get(m) or "") is not None for m in macros)


def field_entries(values, notes, layout_defines):
    """The `fields`, `absent` and staged-byte lists for one layout, in canonical (registry id) order.

    `layout_defines` is what the LAYOUT FILE itself defines, from the census pass. It matters for one
    field: `defines.h` carries an `#ifndef` fallback for `ADC_BATTERY_VOLT`, so the expansion always
    finds one, and taking it would hand every board a per-board calibration his layout never stated.
    A board fact has to come from the board's own file.

    The JSON carries a pin as HIS SYMBOLIC TOKEN and never as our packed byte
    (`specs/robo-presets.md`, the schema's second insistence): the app converts through the same pin
    encoding its editor already uses, so one encoder keeps one owner, and a human reading the file
    sees `PC13` rather than `0x2D`. The packed bytes are built HERE, beside the tokens that produced
    them, purely so the validator stage has something to judge; they do not reach the output.
    """
    fields, absent, staged = [], [], []

    def ref(f):
        out = {"id": "0x%02X" % f["id"], "key": f["key"], "index": f.get("motor", 0)}
        return out

    for f in FIELDS:
        if f["kind"] == "decision3":
            absent.append(dict(ref(f), reason=IMU_ABSENT_REASON))
            continue
        if f["kind"] == "pin":
            raw = values.get(f["frm"])
            if raw is None:
                absent.append(dict(ref(f), reason="his layout defines no %s" % f["frm"]))
                continue
            packed = pack_pin(raw)
            if packed is None:
                why = (
                    "his file marks this pin TODO_PIN: he does not know it"
                    if raw == UNKNOWN_PIN
                    else "his %s expands to %r, which names no pin this encoding holds"
                    % (f["frm"], raw)
                )
                fields.append(
                    dict(ref(f), value=None, unknown=True, **{"from": f["frm"]}, note=why)
                )
                staged.append((f["id"], f.get("motor", 0), PIN_ABSENT))
                notes.append(
                    "%s is UNKNOWN (%s). It is staged ABSENT, never guessed at, so the verdict "
                    "below is the verdict on this layout with that function absent."
                    % (f["key"], why)
                )
                continue
            fields.append(dict(ref(f), value=raw, **{"from": f["frm"]}))
            staged.append((f["id"], f.get("motor", 0), packed))
            continue
        if f["kind"] == "dead_time":
            raw = values.get(f["frm"])
            if raw is None:
                absent.append(dict(ref(f), reason="his tree defines no DEAD_TIME"))
                continue
            if not gates_complete(values):
                absent.append(
                    dict(
                        ref(f),
                        reason=(
                            "his layout gives no complete gate group, and zero is the right dead "
                            "time for a board with no motor configured"
                        ),
                    )
                )
                continue
            dtg = dead_time_dtg(int(raw))
            fields.append(
                dict(
                    ref(f),
                    value=dtg,
                    **{"from": f["frm"]},
                    note=(
                        "his DEAD_TIME %s ticks at his /%d gate-timer divider, re-expressed for our "
                        "/%d divider. HIS SINGLE GLOBAL FIGURE, not per-board data: the bench "
                        "boards' own silicon-proven value is DTG 25."
                        % (raw, HIS_DEAD_TIME_DIVIDER, OUR_DEAD_TIME_DIVIDER)
                    ),
                )
            )
            staged.append((f["id"], f.get("motor", 0), dtg))
            continue
        if f["kind"] == "current_sense":
            both = pack_pin(values.get("PHASE_A") or "") is not None and (
                pack_pin(values.get("PHASE_B") or "") is not None
            )
            fields.append(
                dict(
                    ref(f),
                    value=1 if both else 0,
                    **{"from": "PHASE_A + PHASE_B"},
                    note=(
                        "derived: the declaration and the pins that realize it may not disagree, so "
                        "it is 1 exactly where his layout gives both phase-sense pins"
                    ),
                )
            )
            staged.append((f["id"], f.get("motor", 0), 1 if both else 0))
            continue
        if f["kind"] == "vbatt_cal":
            raw = values.get(f["frm"]) if f["frm"] in layout_defines else None
            if raw is None:
                absent.append(
                    dict(
                        ref(f),
                        reason=(
                            "his layout file states no ADC_BATTERY_VOLT of its own, so it carries no "
                            "calibration: his firmware falls back to a global default in defines.h, "
                            "which is not a fact about this board"
                        ),
                    )
                )
                absent.append(
                    dict(
                        {"id": "0x%02X" % f["id"], "key": f["key"], "index": 1},
                        reason="no calibration to take an offset from",
                    )
                )
                continue
            slope = vbatt_cal_slope(float(raw))
            if abs(slope) > I16_MAX:
                notes.append(
                    "board.vbatt_cal slope %d does not fit the i16 the field holds, so his "
                    "ADC_BATTERY_VOLT %s is carried unconverted in the notes only." % (slope, raw)
                )
                absent.append(
                    dict(ref(f), reason="his %s = %s converts to %d, outside the field's i16"
                         % (f["frm"], raw, slope))
                )
                continue
            fields.append(
                dict(
                    ref(f),
                    value=slope,
                    **{"from": f["frm"]},
                    note=(
                        "his %s volts per ADC count x 1e6 = microvolts per count, the unit index 0 "
                        "holds. MASTER-ONLY: a slave's PA4 reads a fictitious 2.0 V, so a slave "
                        "preset drops board.vbatt and this calibration both." % raw
                    ),
                )
            )
            fields.append(
                dict(
                    {"id": "0x%02X" % f["id"], "key": f["key"], "index": 1},
                    value=0,
                    **{"from": f["frm"]},
                    note=(
                        "his conversion is a pure gain with no offset term, so the offset is zero "
                        "rather than unstated. Our own -5 cV default came from a measured fit and is "
                        "deliberately not mixed with his slope."
                    ),
                )
            )
            continue
        raise AssertionError("field %s has no kind handler" % f["key"])
    return fields, absent, staged


def unmapped_entries(values, defined_names):
    """The census of his keys this preset does not consume, with his value for each."""
    out = []
    for name in sorted(defined_names):
        if name not in UNMAPPED_KEYS:
            continue
        reason = UNMAPPED_KEYS[name]
        if reason is None:
            continue
        raw = values.get(name)
        entry = {"from": name, "reason": reason}
        if raw:
            entry["value"] = raw
        out.append(entry)
    return out


def layout_notes(values, target):
    """The notes every preset for this layout carries, beyond the per-field ones."""
    notes = [
        "Extracted in the MASTER-or-single posture. A SLAVE board of this layout drops "
        "board.vbatt and board.vbatt_cal (battery sense is master-only on this hardware; the slave "
        "takes the pack over the link), and its phase-sense pins may differ from the master's.",
        "The IMU is absent by decision, not by omission: see the absent entries for "
        "imu.scl_pin, imu.sda_pin and imu.model.",
    ]
    if any(values.get(k) is not None for k in ("I2C_PB6PB7", "I2C_PB8PB9", "I2C0_SCL")):
        notes.append(
            "His layout DOES state an IMU bus, as a solderable pair flag. It is still carried as "
            "absent: the flag is a claim about an option, and PB6/PB7 is also where USART0 can live."
        )
    if target == 2 and values.get("STM32F103") is not None:
        notes.append("His layout says this board may carry an STM32F103 rather than a GD32F103.")
    return notes


# --------------------------------------------------------------------------------------------------
# The verdict oracle
# --------------------------------------------------------------------------------------------------

#: Where the Kotlin mirror lives, relative to the repository root, and the Gradle task that runs it.
GRADLE_DIR = "protocol-kotlin"
GRADLE_TASK = "presetVerdicts"

#: The JDK this machine has when `JAVA_HOME` is unset: Android Studio's bundled JBR, which
#: `protocol-kotlin/build.gradle.kts` already names as the only one here.
FALLBACK_JAVA_HOME = "/Applications/Android Studio.app/Contents/jbr/Contents/Home"


def verdicts(repo_root, requests, work):
    """One verdict per request line, from the Kotlin mirror of `board::validate`.

    Fails LOUDLY: a verdict that could not be obtained is not recorded as unknown, because the whole
    point of this stage is that a refusal is a finding. The spec calls it the highest-value part of
    the exercise, and an exercise that silently skipped it would read as a clean run.
    """
    if not requests:
        return {}
    req_path = os.path.join(work, "verdict-requests.tsv")
    rep_path = os.path.join(work, "verdict-replies.tsv")
    with open(req_path, "w") as fh:
        fh.write("\n".join(requests) + "\n")
    env = dict(os.environ)
    if not env.get("JAVA_HOME") and os.path.isdir(FALLBACK_JAVA_HOME):
        env["JAVA_HOME"] = FALLBACK_JAVA_HOME
    proc = subprocess.run(
        [
            "./gradlew",
            "--offline",
            "-q",
            "--console=plain",
            GRADLE_TASK,
            "--args=%s %s" % (req_path, rep_path),
        ],
        cwd=os.path.join(repo_root, GRADLE_DIR),
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    if proc.returncode != 0 or not os.path.exists(rep_path):
        raise RuntimeError(
            "the verdict oracle failed (gradle exit %d):\n%s\n%s"
            % (proc.returncode, proc.stdout.strip(), proc.stderr.strip())
        )
    out = {}
    for line in open(rep_path):
        if not line.strip():
            continue
        col = line.rstrip("\n").split("\t")
        if len(col) != 8:
            raise RuntimeError("a verdict reply has %d columns, expected 8: %r" % (len(col), line))
        name, part, verdict, latch, obs, field, motor, reason = col
        entry = {
            "part": part,
            "verdict": verdict,
            "self_hold": None if latch == "-" else latch,
        }
        if verdict == "refused":
            entry["obs_result"] = int(obs)
            entry["field"] = field
            entry["motor"] = None if motor == "-" else int(motor)
            entry["reason"] = reason
        out.setdefault(name, []).append(entry)
    return out


def verdict_request(name, part, link_set, staged):
    """One request line for the oracle: the staged layout as `<fieldId>:<index>=<raw>` pairs."""
    pairs = [
        "%d:%d=%d" % (fid, index, raw) for fid, index, raw in staged if fid not in NOT_VALIDATED
    ]
    return "\t".join([name, part, str(link_set), ",".join(pairs)])


# --------------------------------------------------------------------------------------------------
# The generator
# --------------------------------------------------------------------------------------------------

_SHA = re.compile(r"^[0-9a-f]{40}$")


def check_clone(clone, sha):
    """Refuse anything but a pristine clone checked out at the pinned SHA."""
    if not _SHA.match(sha):
        raise SystemExit("--sha must be a full 40-character commit SHA, got %r" % sha)
    if not os.path.isdir(os.path.join(clone, ".git")):
        raise SystemExit("%s is not a git clone" % clone)
    dirty = subprocess.run(
        ["git", "-C", clone, "status", "--porcelain"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if dirty:
        raise SystemExit(
            "the upstream clone has a dirty working tree, so what it holds is not the pinned "
            "commit's content:\n%s" % dirty
        )
    head = subprocess.run(
        ["git", "-C", clone, "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if head != sha:
        raise SystemExit("the clone is at %s, not the pinned %s" % (head, sha))


_LAYOUT_FILE = re.compile(r"^defines_2-(\d+)-(\d+)\.h$")


def layout_files(clone):
    """His layout files, as `(target, layout, filename)`, plus the files that are not layouts."""
    found, skipped = [], []
    for name in sorted(os.listdir(os.path.join(clone, DEFINES_DIR))):
        if not name.endswith(".h"):
            continue
        if name in NOT_A_LAYOUT:
            skipped.append({"source_file": "%s/%s" % (DEFINES_DIR, name), "reason": NOT_A_LAYOUT[name]})
            continue
        m = _LAYOUT_FILE.match(name)
        if not m:
            skipped.append(
                {
                    "source_file": "%s/%s" % (DEFINES_DIR, name),
                    "reason": "its name is not defines_2-<target>-<layout>.h, so it names no target",
                }
            )
            continue
        found.append((int(m.group(1)), int(m.group(2)), name))
    found.sort(key=lambda t: (t[0], t[1]))
    return found, skipped


def generate(clone, sha, repo_root, want_verdicts=True):
    """The whole preset document for one pinned upstream commit."""
    check_clone(clone, sha)
    files, skipped = layout_files(clone)
    presets, staged, unknown = [], {}, {}
    work = tempfile.mkdtemp(prefix="robo-presets-")
    try:
        stubs = _stub_dir(work)
        for target, layout, name in files:
            source_file = "%s/%s" % (DEFINES_DIR, name)
            spec = TARGETS.get(target)
            if spec is None or not spec["parts"]:
                skipped.append(
                    {
                        "source_file": source_file,
                        "target": target,
                        "chip": spec["chip"] if spec else None,
                        "reason": (
                            "target %d is %s, a part this image cannot run on at all, so a preset "
                            "for it would be an invitation to brick a board"
                            % (target, spec["chip"] if spec else "a target config.h does not name")
                        ),
                    }
                )
                continue
            for entry, pairs in presets_for_layout(
                clone, stubs, work, target, layout, source_file, spec, unknown
            ):
                presets.append(entry)
                staged[entry["id"]] = pairs
        if want_verdicts:
            attach_verdicts(repo_root, presets, staged, work)
    finally:
        shutil.rmtree(work, ignore_errors=True)

    doc = {
        "generator_version": GENERATOR_VERSION,
        "upstream": UPSTREAM,
        "upstream_sha": sha,
        "expansion": {
            "method": (
                "C preprocessor expansion of a key list we own, with a second -dM pass reporting "
                "every #define his layout file carries that the list does not ask about"
            ),
            "posture": "MASTER_OR_SINGLE",
            "defines": sorted(ROLE_DEFINES + ["CONFIG_H"]),
            "config_h": (
                "neutralized through its own include guard: the layout, role and remote selection "
                "is ours and is recorded here, never his build's"
            ),
            "spl_stubs": sorted(SPL_STUBS),
            "variant_axes": [a["macro"] for a in VARIANT_AXES],
        },
        "decisions": decision_record(),
        "targets_in_scope": sorted(t for t, s in TARGETS.items() if s["parts"]),
        "skipped": sorted(skipped, key=lambda s: s["source_file"]),
        "unknown_keys": [
            {"from": k, "layouts": sorted(v), "note": "his #define, not in our key list"}
            for k, v in sorted(unknown.items())
        ],
        "presets": presets,
        "summary": summarize(presets),
    }
    return doc


#: The variant-axis macros themselves, which the generator defines and so must not read back as
#: board facts: a switch that selects a configuration is not a fact about the board.
AXIS_MACROS = {a["macro"] for a in VARIANT_AXES}


def axis_defines(combo):
    """The `-D` flags one variant combination adds: an axis with no value is simply not defined."""
    out = list(ROLE_DEFINES)
    for axis, (value, _label) in combo:
        if value is not None:
            out.append("%s=%s" % (axis["macro"], value))
    return out


def varying_axes(extracted):
    """Which variant axes actually change what a layout expands to.

    An axis varies when two combinations that differ ONLY in that axis extract differently. That is
    the exact question, and asking it this way means no text scanning decides which layouts have
    sub-variants: every axis is swept, and a layout that ignores one yields a single preset.
    """
    table = {tuple(sorted(labels.items())): values for labels, values in extracted}
    varying = set()
    for axis in VARIANT_AXES:
        for key_a, values_a in table.items():
            for key_b, values_b in table.items():
                a, b = dict(key_a), dict(key_b)
                if a[axis["macro"]] == b[axis["macro"]] or values_a == values_b:
                    continue
                rest_a = {k: v for k, v in a.items() if k != axis["macro"]}
                rest_b = {k: v for k, v in b.items() if k != axis["macro"]}
                if rest_a == rest_b:
                    varying.add(axis["macro"])
    return varying


def presets_for_layout(clone, stubs, work, target, layout, source_file, spec, unknown):
    """Every preset one layout file yields: one per distinct result over the variant axes."""
    include_path = "defines/%s" % os.path.basename(source_file)
    runs = []
    for combo in variant_combos():
        labels = {axis["macro"]: label for axis, (_v, label) in combo}
        extra = axis_defines(combo)
        found = extract(clone, stubs, work, target, layout, extra)
        runs.append((labels, extra, {k: v for k, v in found.items() if k not in AXIS_MACROS}))

    varying = varying_axes([(labels, values) for labels, _e, values in runs])

    # One preset per distinct setting of the axes that matter; a run is representative of its group.
    groups = {}
    for labels, extra, values in runs:
        signature = tuple(labels[a["macro"]] for a in VARIANT_AXES if a["macro"] in varying)
        groups.setdefault(signature, (labels, extra, values))

    out = []
    for signature in sorted(groups):
        labels, extra, values = groups[signature]
        suffix = "".join("-" + label for label in signature)
        display = "".join(
            axis["join"] + labels[axis["macro"]]
            for axis in VARIANT_AXES
            if axis["macro"] in varying
        )
        preset_id = "robo-2-%d-%d%s" % (target, layout, suffix)
        names = census(clone, stubs, work, target, layout, extra, include_path)
        for name in names:
            if name in UNMAPPED_KEYS or name.startswith(INCLUDE_GUARD_PREFIX):
                continue
            if any(f.get("frm") == name for f in FIELDS):
                continue
            unknown.setdefault(name, set()).add(preset_id)
        out.append(
            one_preset(
                preset_id=preset_id,
                name="RoboDurden 2.%d.%d%s" % (target, layout, display),
                target=target,
                layout=layout,
                source_file=source_file,
                spec=spec,
                values=values,
                census_names=names,
                variant={a: labels[a] for a in sorted(varying)} if varying else None,
                role_keys=role_difference(clone, stubs, work, target, layout, extra, values),
            )
        )
    return out


def role_difference(clone, stubs, work, target, layout, extra, values):
    """The FIELD macros whose value changes when the layout is expanded as a SLAVE instead.

    Restricted to the macros the preset's own fields come from. Selecting the slave role also moves
    things that are not board facts (his inter-board USART selection brings `HAS_USART1` with it),
    and a difference in one of those is not a difference between two boards' layouts.
    """
    alt = [d for d in extra if d not in ROLE_DEFINES] + list(ROLE_ALTERNATE)
    other = extract(clone, stubs, work, target, layout, alt)
    macros = {f["frm"] for f in FIELDS if f.get("frm")}
    return sorted(m for m in macros if values.get(m) != other.get(m))


def one_preset(
    preset_id, name, target, layout, source_file, spec, values, census_names, variant, role_keys
):
    """One preset entry."""
    notes = layout_notes(values, target)
    fields, absent, staged = field_entries(values, notes, set(census_names))
    link_set, basis = link_set_from_usarts(values)
    if role_keys:
        notes.append(
            "Role-dependent board facts: a SLAVE board of this layout differs in %s."
            % ", ".join(role_keys)
        )
    if variant and "MOTOR_LEFT" in variant:
        notes.append(
            "This is a DUAL-MOTOR board, and his file expresses its two motors as ALTERNATIVES "
            "under one MOTOR_LEFT switch rather than as a second named gate set, so each of the two "
            "presets carries one motor as motor 0 and neither populates our motor-1 field set. "
            "Merging them would be our invention rather than his record: the two halves also differ "
            "in facts we do not model (his DC-bus shunt and break input per side), and nothing on "
            "the bench exercises a two-motor preset."
        )
    entry = {
        "id": preset_id,
        "name": name,
        "alias": None,
        "chip": spec["chip"],
        "target": target,
        "layout": layout,
        "source_file": source_file,
        "link_set": link_set,
        "link_set_basis": basis,
        "docs": {
            "repo": DOCS_UPSTREAM,
            "target_dir": spec["docs_dir"],
            "layout_dir_prefix": "v%d" % layout,
            "url": "https://github.com/%s/tree/master/%s" % (DOCS_UPSTREAM, spec["docs_dir"]),
            "note": (
                "a layout directory is v<LAYOUT> and may carry an opaque alias suffix (v18=2.20), "
                "so the directory is found by prefix; the alias is not derivable from the firmware "
                "repository and is therefore null"
            ),
        },
        "notes": notes,
        "fields": fields,
        "absent": absent,
        "unmapped": unmapped_entries(values, census_names),
    }
    if variant:
        entry["variant"] = variant
    return entry, staged


def attach_verdicts(repo_root, presets, staged, work):
    """Run every preset through the validator, for every part of its family, and record the verdict."""
    requests, index = [], []
    for p in presets:
        for part in TARGETS[p["target"]]["parts"]:
            requests.append(
                verdict_request("%s|%s" % (p["id"], part), part, p["link_set"], staged[p["id"]])
            )
            index.append((p, part))
    replies = verdicts(repo_root, requests, work)
    for p, part in index:
        got = replies.get("%s|%s" % (p["id"], part))
        if not got:
            raise RuntimeError("the oracle returned no verdict for %s on %s" % (p["id"], part))
        p.setdefault("validate", {"parts": []})["parts"] += got
    for p in presets:
        parts = p["validate"]["parts"]
        unknown = [f["key"] for f in p["fields"] if f.get("unknown")]
        if unknown:
            p["validate"]["unknown_pins_staged_absent"] = sorted(unknown)
        accepted = [e for e in parts if e["verdict"] == "accepted"]
        p["validate"]["verdict"] = "accepted" if accepted else "refused"
        p["validate"]["accepted_on"] = [e["part"] for e in accepted]
        if not accepted:
            first = parts[0]
            p["validate"]["field"] = first["field"]
            p["validate"]["motor"] = first["motor"]
            p["validate"]["reason"] = first["reason"]
            p["validate"]["obs_result"] = first["obs_result"]


def summarize(presets):
    """The counts a reader wants before reading twenty-five entries.

    With the validator stage skipped there are no verdicts, and the counts say so rather than
    reporting every preset as accepted: an unasked question is not a pass.
    """
    by_target = {}
    for p in presets:
        label = "target_%d" % p["target"]
        by_target[label] = by_target.get(label, 0) + 1
    if presets and "validate" not in presets[0]:
        return {"presets": len(presets), "by_target": by_target, "validated": False}
    refused = sorted(
        (p for p in presets if p.get("validate", {}).get("verdict") == "refused"),
        key=lambda p: p["id"],
    )
    return {
        "presets": len(presets),
        "by_target": by_target,
        "validated": True,
        "accepted": len(presets) - len(refused),
        "refused": len(refused),
        "refusals": [
            {
                "id": p["id"],
                "chip": p["chip"],
                "field": p["validate"]["field"],
                "motor": p["validate"]["motor"],
                "reason": p["validate"]["reason"],
            }
            for p in refused
        ],
    }


def decision_record():
    """The mapping decisions, carried as data so a reader of the file sees them with the values."""
    return [
        {
            "decision": "gate colour order",
            "settled": "A = BLDC_Y (yellow), B = BLDC_B (blue), C = BLDC_G (green)",
            "oracle": (
                "the bench pair's own working configuration: defines_2-2-20.h and defines_2-1-20.h "
                "read BLDC_YH PA8 / BLDC_BH PA9 / BLDC_GH PA10, and our map for those boards is "
                "gate_hi_a PA8 / gate_hi_b PA9 / gate_hi_c PA10. Confirmed twice more by his fixed "
                "TIMER_BLDC_CHANNEL_Y/B/G = TIMER_CH_0/1/2 and by layout 2.2.1 spelling the "
                "channel-to-pin pairing out"
            ),
            "consequence": (
                "a layout whose colour labels contradict the convention is REFUSED with "
                "InvalidGateSet rather than silently reordered, because reordering would permute "
                "the phases against the halls"
            ),
        },
        {
            "decision": "ADC_BATTERY_VOLT to board.vbatt_cal",
            "settled": "slope_uV_per_count = round(ADC_BATTERY_VOLT * 1e6); offset = 0",
            "oracle": (
                "his figure is volts per ADC count; index 0 of board.vbatt_cal is microvolts per "
                "count and the formula's /10_000 returns centivolts. 0.02507 becomes 25070 against "
                "the registry default 25200 from the bench four-point fit, 0.5% apart"
            ),
            "consequence": (
                "the offset is zero because his model is a pure gain, not because it is unknown, "
                "and the calibration is MASTER-ONLY: a slave's PA4 reads a fictitious 2.0 V, so a "
                "slave preset carrying it is noise"
            ),
        },
        {
            "decision": "the IMU bus",
            "settled": "imu.scl_pin, imu.sda_pin and imu.model are left ABSENT",
            "oracle": (
                "he states the bus as a pair flag (I2C_PB6PB7, I2C_PB8PB9) and his own comment on "
                "2.1.20 says the IMU needs soldering to the tiny module on PB6/PB7, which is also "
                "where USART0 can live, so the flag is a solderable option and not a fitted bus"
            ),
            "consequence": (
                "the group is all-or-none, so the model goes absent with the pins; PB8/PB9 is not a "
                "hardware-I2C pair our capability table answers at all; and the model is a fact "
                "about the part soldered on, which no layout file carries"
            ),
        },
        {
            "decision": "motor.dead_time (not named by the spec)",
            "settled": "DTG = round(his DEAD_TIME * 1/2) = 30 ticks",
            "oracle": (
                "his DEAD_TIME 60 is programmed at TIMER_CKDIV_DIV1 (13.9 ns a tick, 833 ns) in "
                "setup.c; our bring-up runs the generator at /2 (27.8 ns a tick)"
            ),
            "consequence": (
                "a preset with no dead time is refused on DeadTimeBelowFloor before any check that "
                "could say something about his file, so every gated preset would report nothing. "
                "This is HIS single global figure, not per-board data: the bench boards' own "
                "silicon-proven value is 25"
            ),
        },
    ]


# --------------------------------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------------------------------

#: Where the generated presets land by default: the asset the app ships (`specs/robo-presets.md`,
#: "The app side"), committed so the determinism test has something to compare against.
DEFAULT_OUT = "tools/robo-presets/board-presets.json"


def render(doc):
    """The document as the bytes the file holds: sorted keys, two-space indent, one trailing newline."""
    return json.dumps(doc, sort_keys=True, indent=2) + "\n"


def repo_root():
    """This repository's root, which is where the Gradle wrapper and the default output sit."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--clone", help="the pristine upstream clone, in the gitignored reference/ area")
    ap.add_argument("--sha", help="the upstream commit the clone is checked out at (40 hex)")
    ap.add_argument("--out", default=None, help="where to write the presets (default %s)" % DEFAULT_OUT)
    ap.add_argument(
        "--no-verdicts",
        action="store_true",
        help="skip the validator stage (for a tree with no JDK; the verdicts are then absent, never guessed)",
    )
    args = ap.parse_args(argv)
    if not args.clone or not args.sha:
        ap.error("--clone and --sha are both required")
    root = repo_root()
    doc = generate(args.clone, args.sha, root, want_verdicts=not args.no_verdicts)
    out = args.out or os.path.join(root, DEFAULT_OUT)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as fh:
        fh.write(render(doc))
    s = doc["summary"]
    print("%d presets from %s" % (s["presets"], doc["upstream_sha"][:12]))
    for k in sorted(s["by_target"]):
        print("  %s: %d" % (k, s["by_target"][k]))
    if not s["validated"]:
        print("  the validator stage was skipped, so no preset carries a verdict")
    else:
        print("  accepted %d, refused %d" % (s["accepted"], s["refused"]))
        for r in s["refusals"]:
            print("  REFUSED %s (%s): %s %s" % (r["id"], r["chip"], r["field"], r["reason"]))
    if doc["unknown_keys"]:
        print("  upstream keys our list does not ask about:")
        for u in doc["unknown_keys"]:
            print("    %s (%s)" % (u["from"], ", ".join(u["layouts"])))
    print("  wrote %s" % os.path.relpath(out, root))
    return 0


if __name__ == "__main__":
    sys.exit(main())
