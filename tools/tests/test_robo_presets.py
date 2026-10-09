#!/usr/bin/env python3
"""Unit tests for tools/robo-presets.py, the RoboDurden layout-to-preset generator.

Run: python3 -m unittest discover -s tools/tests -p 'test_robo_presets.py'
 or: python3 -m pytest tools/tests -q

Covers `specs/robo-presets.md`: the pin encoding both ways, the three mapping decisions it names
plus the dead-time one it does not, the LINK_SET derivation, the marker grammar, the variant-axis
sweep, the skip rules for his non-layout files and his out-of-scope targets, and the key-list
invariants.

Two tests need the pinned upstream clone, a C preprocessor and a JDK, and SKIP without them:

  - THE BENCH-PAIR PIN TEST, which is the oracle for decision 1 (the gate colour order). It runs
    against the COMMITTED presets, so it needs none of those three, and it is the test that makes
    the colour mapping proven rather than assumed.
  - THE DETERMINISM TEST, which generates twice from one SHA and compares byte for byte. That is
    `specs/robo-presets.md`'s acceptance test, stated there as a property and not an intention.

stdlib only, no hardware.
"""

import importlib.util
import json
import os
import shutil
import sys
import tempfile
import unittest

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_TOOLS)
sys.path.insert(0, _TOOLS)
_spec = importlib.util.spec_from_file_location(
    "robo_presets", os.path.join(_TOOLS, "robo-presets.py")
)
rp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rp)

#: The pinned upstream, as the committed presets name it. The clone itself is in the gitignored
#: `reference/` area and is not present in a clean checkout, which is what the skips are for.
CLONE = os.path.join(_ROOT, "reference", "robo-gd32")

#: The committed presets, which are also the asset the app ships.
PRESETS = os.path.join(_ROOT, rp.DEFAULT_OUT)


def have_clone(sha):
    """Is the pinned clone here, pristine and at `sha`?"""
    if not os.path.isdir(os.path.join(CLONE, ".git")):
        return False
    try:
        rp.check_clone(CLONE, sha)
    except SystemExit:
        return False
    return True


def have_toolchain():
    """Is there a C preprocessor and a JDK the verdict oracle can use?"""
    if shutil.which("cc") is None:
        return False
    return bool(os.environ.get("JAVA_HOME")) or os.path.isdir(rp.FALLBACK_JAVA_HOME)


# --------------------------------------------------------------------------------------------------
# The pin encoding we emit into
# --------------------------------------------------------------------------------------------------


class PinEncoding(unittest.TestCase):
    def test_tokens_pack_to_the_board_models_encoding(self):
        # specs/board-model.md, "The field vocabulary": (port << 4) | pin, A=0 B=1 C=2 D=3 F=5.
        for token, packed in [
            ("PA0", 0x00),
            ("PA8", 0x08),
            ("PA15", 0x0F),
            ("PB0", 0x10),
            ("PB12", 0x1C),
            ("PC13", 0x2D),
            ("PC15", 0x2F),
            ("PF0", 0x50),
            ("PF4", 0x54),
        ]:
            self.assertEqual(rp.pack_pin(token), packed, token)

    def test_a_token_that_names_no_pin_packs_to_nothing(self):
        # Including port E, which the encoding deliberately does not define, and his TODO_PIN.
        for token in ("PE3", "TODO_PIN", "P??", "PA16", "GPIO_PUPD_NONE", ""):
            self.assertIsNone(rp.pack_pin(token), token)

    def test_the_undef_list_covers_every_port_his_tokens_use(self):
        # The extraction undefines the pin tokens so a marker reads `PC13` and not an address
        # expression. A token his files use and the list misses would silently read as one.
        for token in ("PA0", "PA15", "PB0", "PB15", "PC13", "PC15", "PF0", "PF1", "PF4", "PF6", "PF7"):
            self.assertIn(token, rp.PIN_TOKENS, token)
        self.assertIn("TODO_PIN", rp.SYMBOLIC_TOKENS)


# --------------------------------------------------------------------------------------------------
# Decision 1: his gate colour order against our phase order
# --------------------------------------------------------------------------------------------------


class GateColourOrder(unittest.TestCase):
    #: The bench pair's pin map (`specs/board-model.md` section 4, carried as preset data there, and
    #: mirrored in `protocol-kotlin/.../LayoutPresets.kt`, `LayoutPresets.splitMotor`). This is the
    #: ORACLE for decision 1: `defines_2-2-20.h` (the F103 master) and `defines_2-1-20.h` (the F130
    #: slave) are the files our working pin map came from, so a generated preset for those two
    #: layouts must equal it. A mapping that swapped two colours would swap two phases, which on a
    #: first arm is a jerk or a trip rather than a spin.
    BENCH = {
        "motor.hall_a": "PC13",
        "motor.hall_b": "PA1",
        "motor.hall_c": "PC14",
        "motor.gate_hi_a": "PA8",
        "motor.gate_hi_b": "PA9",
        "motor.gate_hi_c": "PA10",
        "motor.gate_lo_a": "PB13",
        "motor.gate_lo_b": "PB14",
        "motor.gate_lo_c": "PB15",
    }

    #: The benign fleet pins those two boards also share, which the same two files state
    #: (`specs/board-model.md` section 1's registry defaults: self-hold PB12, buzzer PB9, LEDs
    #: PB3/PA15/PB4). Checked alongside, because a mapping error in the singletons would be just as
    #: wrong and just as invisible.
    BENCH_BOARD = {
        "board.self_hold": "PB12",
        "board.buzzer": "PB9",
        "led.green": "PB3",
        "led.orange": "PA15",
        "led.red": "PB4",
        "board.vbatt": "PA4",
    }

    def test_the_colour_to_phase_map_is_the_one_his_channel_convention_fixes(self):
        # `HoverBoardGigaDevice/Inc/defines.h` fixes TIMER_BLDC_CHANNEL_Y/B/G to TIMER_CH_0/1/2 for
        # every Gen2 board, and our gate set takes TIMER0's channels 0/1/2 in that order.
        self.assertEqual(rp.GATE_COLOUR_FOR_PHASE, {"a": "Y", "b": "B", "c": "G"})

    def test_the_mapped_gate_fields_read_his_colours_in_that_order(self):
        gates = {f["key"]: f["frm"] for f in rp.FIELDS if f["key"].startswith("motor.gate_")}
        self.assertEqual(
            gates,
            {
                "motor.gate_hi_a": "BLDC_YH",
                "motor.gate_hi_b": "BLDC_BH",
                "motor.gate_hi_c": "BLDC_GH",
                "motor.gate_lo_a": "BLDC_YL",
                "motor.gate_lo_b": "BLDC_BL",
                "motor.gate_lo_c": "BLDC_GL",
            },
        )

    def test_the_bench_pairs_generated_presets_equal_the_bench_pin_map(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        by_id = {p["id"]: p for p in doc["presets"]}
        for preset_id, part in (("robo-2-1-20", "F130C8"), ("robo-2-2-20", "F103C8")):
            with self.subTest(preset=preset_id):
                self.assertIn(preset_id, by_id)
                p = by_id[preset_id]
                got = {f["key"]: f["value"] for f in p["fields"]}
                for key, pin in dict(self.BENCH, **self.BENCH_BOARD).items():
                    self.assertEqual(got.get(key), pin, "%s %s" % (preset_id, key))
                # And the whole layout is one our own boot judge accepts on that part, which is what
                # makes "known-good" a checked claim rather than a label.
                self.assertEqual(p["validate"]["verdict"], "accepted")
                self.assertIn(part, p["validate"]["accepted_on"])

    def test_the_bench_presets_carry_his_phase_sense_pins_as_his_file_states_them(self):
        # Recorded as a test rather than only in the report: his 2.1.20 gives PHASE_A PB0 and
        # PHASE_B PA0, which is the F103 MASTER's pair. Our own spec says the F130 slave senses on
        # PB0/PB1 (`specs/board-model.md` section 1, motor.phase_a), so the generated 2.1.20 preset
        # disagrees with the bench slave on phase B. The generator carries HIS value; the
        # disagreement is a finding about his file, and this test pins which value is in the file so
        # a future change to either side is visible.
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        by_id = {p["id"]: p for p in doc["presets"]}
        for preset_id in ("robo-2-1-20", "robo-2-2-20"):
            got = {f["key"]: f["value"] for f in by_id[preset_id]["fields"]}
            self.assertEqual(got["motor.phase_a"], "PB0")
            self.assertEqual(got["motor.phase_b"], "PA0")
            self.assertEqual(got["motor.current_sense"], 1)


# --------------------------------------------------------------------------------------------------
# Decision 2: ADC_BATTERY_VOLT to board.vbatt_cal
# --------------------------------------------------------------------------------------------------


class BatteryCalibration(unittest.TestCase):
    def test_volts_per_count_becomes_microvolts_per_count(self):
        # His bench figure, against the registry default 25200 from the 2026-10-08 four-point fit:
        # 0.5% apart, which is the check that the units are right rather than merely plausible.
        self.assertEqual(rp.vbatt_cal_slope(0.02507), 25070)
        # His other figures across the fleet, including a 20x divider and a 40-digit literal.
        self.assertEqual(rp.vbatt_cal_slope(0.0171862875), 17186)
        self.assertEqual(rp.vbatt_cal_slope(0.025392524927), 25393)
        self.assertEqual(rp.vbatt_cal_slope(0.02500961912134820371101460718516), 25010)
        # The default in defines.h, which a layout stating nothing of its own would fall back to.
        self.assertEqual(rp.vbatt_cal_slope(0.024169921875), 24170)

    def test_every_generated_slope_fits_the_i16_the_field_holds(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        for p in doc["presets"]:
            for f in p["fields"]:
                if f["key"] == "board.vbatt_cal":
                    self.assertLessEqual(abs(f["value"]), rp.I16_MAX, p["id"])

    def test_the_offset_index_is_zero_because_his_model_is_a_pure_gain(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        for p in doc["presets"]:
            cal = {f["index"]: f["value"] for f in p["fields"] if f["key"] == "board.vbatt_cal"}
            if not cal:
                continue
            self.assertEqual(sorted(cal), [0, 1], p["id"])
            self.assertEqual(cal[1], 0, p["id"])

    def test_every_preset_says_the_battery_sense_is_master_only(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        for p in doc["presets"]:
            self.assertTrue(
                any("master-only" in n for n in p["notes"]),
                "%s does not say battery sense is master-only" % p["id"],
            )


# --------------------------------------------------------------------------------------------------
# Decision 3: the IMU bus
# --------------------------------------------------------------------------------------------------


class ImuBus(unittest.TestCase):
    def test_the_imu_fields_are_never_asserted(self):
        imu = [f for f in rp.FIELDS if f["key"].startswith("imu.")]
        self.assertEqual({f["key"] for f in imu}, {"imu.scl_pin", "imu.sda_pin", "imu.model"})
        for f in imu:
            self.assertEqual(f["kind"], "decision3", f["key"])
            self.assertNotIn("frm", f)

    def test_every_preset_carries_the_imu_as_absent_with_a_reason(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        for p in doc["presets"]:
            absent = {a["key"]: a["reason"] for a in p["absent"]}
            for key in ("imu.scl_pin", "imu.sda_pin", "imu.model"):
                self.assertIn(key, absent, p["id"])
                self.assertIn("solderable", absent[key])
            self.assertFalse([f for f in p["fields"] if f["key"].startswith("imu.")], p["id"])

    def test_his_pair_flags_are_recorded_rather_than_dropped(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        flags = set()
        for p in doc["presets"]:
            flags |= {u["from"] for u in p["unmapped"] if u["from"].startswith(("I2C", "MPU", "BMI"))}
        # A layout that states a bus states it as one of these; the census must keep every one.
        self.assertTrue(flags, "no IMU bus claim of his survived into the census")
        self.assertTrue(flags <= set(rp.UNMAPPED_KEYS), flags)


# --------------------------------------------------------------------------------------------------
# Decision 4: the dead time, which the spec did not name
# --------------------------------------------------------------------------------------------------


class DeadTime(unittest.TestCase):
    def test_his_global_figure_is_re_expressed_for_our_divider(self):
        # His DEAD_TIME 60 at TIMER_CKDIV_DIV1 (13.9 ns a tick, 833 ns) against our /2 generator
        # (27.8 ns a tick): 30 ticks.
        self.assertEqual(rp.dead_time_dtg(60), 30)
        self.assertEqual(rp.dead_time_dtg(50), 25)

    def test_the_result_clears_the_validators_floor(self):
        # Below the floor every preset with gates would be refused on the dead time before any check
        # that could say something about his file, because the first failure wins.
        self.assertGreaterEqual(rp.dead_time_dtg(60), 18)

    def test_the_dead_time_rides_only_where_there_is_a_gate_group(self):
        gates = {"BLDC_%s%s" % (c, s): "PA8" for c in "GBY" for s in "HL"}
        self.assertTrue(rp.gates_complete(gates))
        self.assertFalse(rp.gates_complete(dict(gates, BLDC_GH="TODO_PIN")))
        self.assertFalse(rp.gates_complete({}))

    def test_every_gated_preset_carries_a_dead_time_and_says_where_it_came_from(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        for p in doc["presets"]:
            by_key = {f["key"]: f for f in p["fields"]}
            gated = all(
                "motor.gate_%s_%s" % (side, phase) in by_key
                for side in ("hi", "lo")
                for phase in ("a", "b", "c")
            )
            if not gated:
                continue
            self.assertIn("motor.dead_time", by_key, p["id"])
            dt = by_key["motor.dead_time"]
            self.assertEqual(dt["value"], 30, p["id"])
            self.assertEqual(dt["from"], "DEAD_TIME", p["id"])
            self.assertIn("not per-board data", dt["note"], p["id"])


# --------------------------------------------------------------------------------------------------
# The LINK_SET derivation and the reserved pins behind it
# --------------------------------------------------------------------------------------------------


class LinkSet(unittest.TestCase):
    def test_a_declared_wiring_sets_its_bit_whichever_pin_is_tx(self):
        mask, basis = rp.link_set_from_usarts(
            {"USART0_TX": "PB6", "USART0_RX": "PB7", "USART1_TX": "PA3", "USART1_RX": "PA2"}
        )
        self.assertEqual(mask, 0b1010)
        self.assertEqual([b["bit"] for b in basis], [3, 1])

    def test_a_wiring_he_does_not_declare_sets_no_bit(self):
        # Layout 2.2.1 declares USART0 on PB6/PB7 and USART2 on PB10/PB11 and no USART1, which is
        # why PA3 is free for its BUTTON field.
        mask, _ = rp.link_set_from_usarts(
            {"USART0_TX": "PB6", "USART0_RX": "PB7", "USART2_TX": "PB10", "USART2_RX": "PB11"}
        )
        self.assertEqual(mask, 0b1100)

    def test_a_usart_on_pins_the_allowlist_does_not_carry_sets_no_bit(self):
        # Layout 2.1.3's USART1 is PA2/PA15, which is not the allowlist's PA2/PA3 wiring.
        mask, basis = rp.link_set_from_usarts({"USART1_TX": "PA2", "USART1_RX": "PA15"})
        self.assertEqual(mask, 0)
        self.assertEqual(basis, [])

    def test_half_a_declaration_is_not_a_declaration(self):
        mask, _ = rp.link_set_from_usarts({"USART1_TX": "PA2"})
        self.assertEqual(mask, 0)

    def test_the_allowlist_mirrors_the_firmwares_three_wirings(self):
        # protocol-kotlin/.../ReservedPins.kt, SAFE_LINK_USARTS: two BLE wirings and the link.
        self.assertEqual(
            [(e["bit"], e["pins"]) for e in rp.ALLOWLIST],
            [(3, ("PB6", "PB7")), (1, ("PA2", "PA3")), (2, ("PB10", "PB11"))],
        )


# --------------------------------------------------------------------------------------------------
# The key list, which is ours
# --------------------------------------------------------------------------------------------------


class KeyList(unittest.TestCase):
    def test_the_mapped_fields_are_in_registry_id_order(self):
        ids = [f["id"] for f in rp.FIELDS]
        self.assertEqual(ids, sorted(ids))
        # One row per field, whatever its indices: board.vbatt_cal's two indices come from one row.
        self.assertEqual(len(ids), len(set(ids)))

    def test_every_field_has_a_handler_and_a_source(self):
        for f in rp.FIELDS:
            self.assertIn(
                f["kind"], {"pin", "decision3", "dead_time", "current_sense", "vbatt_cal"}, f["key"]
            )
            if f["kind"] in {"pin", "dead_time", "vbatt_cal"}:
                self.assertTrue(f.get("frm"), f["key"])

    def test_a_mapped_macro_is_not_also_a_known_unmapped_one(self):
        # Except the gate macros, which are listed with no reason precisely so the census stays
        # complete without reporting them twice.
        for f in rp.FIELDS:
            name = f.get("frm")
            if name and name in rp.UNMAPPED_KEYS:
                self.assertIsNone(rp.UNMAPPED_KEYS[name], name)

    def test_the_carried_but_unjudged_motor_facts_are_not_preset_data(self):
        # motor.direction (0x62), motor.align_offset (0x63) and motor.current_cal (0x67) are not
        # facts a pin map can state, so a preset must not carry them
        # (protocol-kotlin/.../LayoutPresets.kt, LayoutPreset.applyTo).
        self.assertFalse({f["id"] for f in rp.FIELDS} & {0x62, 0x63, 0x67})

    def test_only_the_calibration_is_withheld_from_the_validator(self):
        self.assertEqual(rp.NOT_VALIDATED, {0x69})


# --------------------------------------------------------------------------------------------------
# What is skipped, and loudly
# --------------------------------------------------------------------------------------------------


class Scope(unittest.TestCase):
    def test_only_his_targets_1_and_2_are_in_scope(self):
        self.assertEqual(sorted(t for t, s in rp.TARGETS.items() if s["parts"]), [1, 2])
        self.assertEqual(rp.TARGETS[3]["chip"], "GD32E230")
        self.assertEqual(rp.TARGETS[4]["chip"], "MM32SPIN05")

    def test_the_template_and_the_autodetect_variant_are_not_layouts(self):
        self.assertEqual(sorted(rp.NOT_A_LAYOUT), ["defines_2-ad.h", "defines_2-x-y.h"])

    def test_every_skip_is_named_with_a_reason(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        doc = json.load(open(PRESETS))
        files = {s["source_file"] for s in doc["skipped"]}
        for name in ("defines_2-3-1.h", "defines_2-3-2.h", "defines_2-3-3.h", "defines_2-4-1.h"):
            self.assertIn("%s/%s" % (rp.DEFINES_DIR, name), files, name)
        for name in rp.NOT_A_LAYOUT:
            self.assertIn("%s/%s" % (rp.DEFINES_DIR, name), files, name)
        for s in doc["skipped"]:
            self.assertTrue(s["reason"].strip(), s["source_file"])
        # And nothing out of scope slipped into the presets.
        self.assertEqual({p["target"] for p in doc["presets"]}, {1, 2})

    def test_the_variant_sweep_covers_every_combination(self):
        combos = rp.variant_combos()
        self.assertEqual(len(combos), 4)
        self.assertEqual(rp.AXIS_MACROS, {"LAYOUT_SUB", "MOTOR_LEFT"})
        # An axis with no value is simply not defined, which is how MOTOR_LEFT's default reads.
        self.assertNotIn("MOTOR_LEFT", "".join(rp.axis_defines(combos[0])))

    def test_an_axis_that_changes_nothing_yields_one_preset(self):
        same = [
            ({"LAYOUT_SUB": "0", "MOTOR_LEFT": "right"}, {"HALL_A": "PA0"}),
            ({"LAYOUT_SUB": "1", "MOTOR_LEFT": "right"}, {"HALL_A": "PA0"}),
            ({"LAYOUT_SUB": "0", "MOTOR_LEFT": "left"}, {"HALL_A": "PA0"}),
            ({"LAYOUT_SUB": "1", "MOTOR_LEFT": "left"}, {"HALL_A": "PA0"}),
        ]
        self.assertEqual(rp.varying_axes(same), set())

    def test_an_axis_that_changes_the_layout_is_named(self):
        split = [
            ({"LAYOUT_SUB": "0", "MOTOR_LEFT": "right"}, {"HALL_A": "PA0"}),
            ({"LAYOUT_SUB": "1", "MOTOR_LEFT": "right"}, {"HALL_A": "PA0"}),
            ({"LAYOUT_SUB": "0", "MOTOR_LEFT": "left"}, {"HALL_A": "PB5"}),
            ({"LAYOUT_SUB": "1", "MOTOR_LEFT": "left"}, {"HALL_A": "PB5"}),
        ]
        self.assertEqual(rp.varying_axes(split), {"MOTOR_LEFT"})


# --------------------------------------------------------------------------------------------------
# The committed document's own shape
# --------------------------------------------------------------------------------------------------


class Document(unittest.TestCase):
    def setUp(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        self.doc = json.load(open(PRESETS))

    def test_it_carries_its_provenance_and_nothing_machine_local(self):
        self.assertEqual(self.doc["upstream"], rp.UPSTREAM)
        self.assertRegex(self.doc["upstream_sha"], r"^[0-9a-f]{40}$")
        self.assertEqual(self.doc["generator_version"], rp.GENERATOR_VERSION)
        text = json.dumps(self.doc)
        for leak in (os.path.expanduser("~"), "/Users/", "/private/tmp", "clang", "gradle"):
            self.assertNotIn(leak, text, "the output names %r" % leak)

    def test_the_four_decisions_are_recorded_as_data(self):
        decisions = {d["decision"] for d in self.doc["decisions"]}
        self.assertEqual(len(decisions), 4)
        for d in self.doc["decisions"]:
            self.assertTrue(d["settled"] and d["oracle"] and d["consequence"], d["decision"])

    def test_every_field_names_the_macro_it_came_from(self):
        # The schema's first insistence: a preset that disagrees with a board on the bench is
        # traceable to one line of one file of his.
        for p in self.doc["presets"]:
            for f in p["fields"]:
                self.assertTrue(f.get("from"), "%s %s" % (p["id"], f["key"]))
                self.assertIn("id", f)
                self.assertIn("index", f)

    def test_a_pin_rides_as_his_token_and_never_as_our_packed_byte(self):
        # The schema's second insistence: one pin encoder keeps one owner, and a human reading the
        # file sees PC13 rather than 0x2D.
        for p in self.doc["presets"]:
            for f in p["fields"]:
                self.assertNotIn("packed", f, "%s %s" % (p["id"], f["key"]))
                if f["key"].endswith("_pin") or "hall" in f["key"] or "gate" in f["key"]:
                    if f["value"] is not None:
                        self.assertIsNotNone(rp.pack_pin(f["value"]), f["value"])

    def test_absent_is_distinguished_from_a_field_that_never_appeared(self):
        # The schema's third insistence. Every field our key list asks about appears in exactly one
        # of the two lists, so "his layout defines no hall pins" and "the generator forgot to ask"
        # are different statements.
        asked = set()
        for f in rp.FIELDS:
            asked.add((f["key"], f.get("motor", 0)))
            if f["kind"] == "vbatt_cal":
                asked.add((f["key"], 1))
        for p in self.doc["presets"]:
            seen = {(f["key"], f["index"]) for f in p["fields"]}
            seen |= {(a["key"], a["index"]) for a in p["absent"]}
            self.assertEqual(seen, asked, p["id"])
            for a in p["absent"]:
                self.assertTrue(a["reason"].strip(), "%s %s" % (p["id"], a["key"]))

    def test_an_unmapped_entry_carries_his_value(self):
        # The schema's fourth insistence: the unmapped entries are the queue for our own board
        # model, and they are the only census of them we will get cheaply.
        valued = 0
        for p in self.doc["presets"]:
            for u in p["unmapped"]:
                self.assertIn(u["from"], rp.UNMAPPED_KEYS, u["from"])
                self.assertTrue(u["reason"].strip(), u["from"])
                valued += "value" in u
        self.assertGreater(valued, 0)

    def test_the_ids_are_stable_machine_safe_and_unique(self):
        ids = [p["id"] for p in self.doc["presets"]]
        self.assertEqual(len(ids), len(set(ids)))
        for i in ids:
            self.assertRegex(i, r"^robo-2-[12]-\d+(-[0-9a-z]+)*$")

    def test_a_preset_names_the_part_it_was_judged_on(self):
        for p in self.doc["presets"]:
            parts = [e["part"] for e in p["validate"]["parts"]]
            self.assertEqual(parts, rp.TARGETS[p["target"]]["parts"], p["id"])
            if p["validate"]["verdict"] == "refused":
                self.assertTrue(p["validate"]["reason"], p["id"])
                self.assertTrue(p["validate"]["field"], p["id"])

    def test_the_upstream_census_leaves_nothing_unread(self):
        # The second pass's whole job: a #define of his our key list does not ask about is a diff,
        # never a silence. An entry here is not a failure, it is the report arriving; a non-empty
        # list means the key list wants a row for a key upstream has added.
        for u in self.doc["unknown_keys"]:
            self.assertTrue(u["layouts"], u["from"])
        self.assertEqual(
            [u["from"] for u in self.doc["unknown_keys"]],
            [],
            "upstream defines keys our list does not ask about: add them to FIELDS or UNMAPPED_KEYS",
        )


# --------------------------------------------------------------------------------------------------
# Determinism: the acceptance test
# --------------------------------------------------------------------------------------------------


class Determinism(unittest.TestCase):
    def test_generating_twice_from_one_sha_is_byte_identical(self):
        if not os.path.exists(PRESETS):
            self.skipTest("the committed presets are not here")
        sha = json.load(open(PRESETS))["upstream_sha"]
        if not have_clone(sha):
            self.skipTest("the pinned upstream clone is not here, pristine and at %s" % sha[:12])
        if not have_toolchain():
            self.skipTest("no C preprocessor and JDK for the extraction and the verdicts")
        first = rp.render(rp.generate(CLONE, sha, _ROOT))
        second = rp.render(rp.generate(CLONE, sha, _ROOT))
        self.assertEqual(first, second)
        # And the committed file is that same output, so the asset in the tree is reproducible from
        # the SHA it names rather than being a snapshot nobody can rebuild.
        self.assertEqual(first, open(PRESETS).read())

    def test_a_dirty_or_wrong_tree_is_refused(self):
        if not os.path.isdir(os.path.join(CLONE, ".git")):
            self.skipTest("the pinned upstream clone is not here")
        with self.assertRaises(SystemExit):
            rp.check_clone(CLONE, "0" * 40)
        with self.assertRaises(SystemExit):
            rp.check_clone(CLONE, "396a27b")

    def test_a_short_sha_is_not_a_pin(self):
        with self.assertRaises(SystemExit):
            rp.check_clone(CLONE, "deadbeef")


# --------------------------------------------------------------------------------------------------
# The extraction, against his real files
# --------------------------------------------------------------------------------------------------


class Extraction(unittest.TestCase):
    """The preprocessor passes, run against the pinned clone when it is here."""

    @classmethod
    def setUpClass(cls):
        if not os.path.exists(PRESETS):
            raise unittest.SkipTest("the committed presets are not here")
        cls.sha = json.load(open(PRESETS))["upstream_sha"]
        if not have_clone(cls.sha):
            raise unittest.SkipTest("the pinned upstream clone is not here")
        if shutil.which("cc") is None:
            raise unittest.SkipTest("no C preprocessor")
        cls.work = tempfile.mkdtemp(prefix="robo-presets-test-")
        cls.stubs = rp._stub_dir(cls.work)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(getattr(cls, "work", ""), ignore_errors=True)

    def extract(self, target, layout, extra=None):
        return rp.extract(
            CLONE, self.stubs, self.work, target, layout, extra or list(rp.ROLE_DEFINES)
        )

    def test_a_pin_comes_back_as_his_token(self):
        values = self.extract(1, 20)
        self.assertEqual(values["HALL_A"], "PC13")
        self.assertEqual(values["BLDC_YH"], "PA8")
        self.assertEqual(values["ADC_BATTERY_VOLT"], "0.02507")

    def test_a_flag_style_define_comes_back_as_an_empty_value(self):
        # His IMU pair flags carry no value, which is a different statement from being absent.
        values = self.extract(1, 20)
        self.assertEqual(values["I2C_PB6PB7"], "")
        self.assertNotIn("I2C_PB8PB9", values)

    def test_a_commented_out_define_is_absent_rather_than_regex_bait(self):
        # 2.1.20 carries `//#define CURRENT_DC P??` on a board with no DC shunt. This is the failure
        # mode the preprocessor method exists to avoid.
        self.assertNotIn("CURRENT_DC", self.extract(1, 20))

    def test_his_button_alias_resolves_through_defines_h(self):
        # 2.1.4 defines BUTTON_PU and no BUTTON; defines.h makes BUTTON the same pin. Nothing but
        # the preprocessor gets that right.
        values = self.extract(1, 4)
        self.assertEqual(values["BUTTON_PU"], "PA4")
        self.assertEqual(values["BUTTON"], "PA4")

    def test_his_unknown_pin_survives_as_unknown(self):
        # 2.1.9 routes PHOTO_L and PHOTO_R through TODO_PIN, which is `#define TODO_PIN PF4` in
        # three of his files. Expanded, it would read as a plausible PF4.
        values = self.extract(1, 9)
        self.assertEqual(values["PHOTO_L"], rp.UNKNOWN_PIN)
        self.assertEqual(values["PHOTO_R"], rp.UNKNOWN_PIN)

    def test_a_buzzer_his_own_guard_never_defines_is_absent(self):
        # 2.1.2 guards `#define BUZZER PB9` with `#ifdef BUZZER` rather than `#ifdef HAS_BUZZER`, so
        # the buzzer is never defined on that layout. The preprocessor reports what his build does.
        values = self.extract(1, 2)
        self.assertEqual(values.get("HAS_BUZZER"), "")
        self.assertNotIn("BUZZER", values)

    def test_the_role_posture_changes_the_buzzer_where_his_guard_says_it_should(self):
        master = self.extract(1, 1)
        slave = rp.extract(CLONE, self.stubs, self.work, 1, 1, list(rp.ROLE_ALTERNATE))
        self.assertEqual(master["BUZZER"], "PB10")
        self.assertNotIn("BUZZER", slave)

    def test_the_census_sees_only_what_that_file_defines(self):
        names = rp.census(
            CLONE, self.stubs, self.work, 1, 20, list(rp.ROLE_DEFINES), "defines/defines_2-1-20.h"
        )
        self.assertIn("HALL_A", names)
        self.assertIn("I2C_PB6PB7", names)
        self.assertIn("DEFINES_2_1_20_H", names)  # its own include guard
        # Nothing from defines.h or the configuration we passed leaks in.
        for leak in ("DEAD_TIME", "PWM_FREQ", "MASTER_OR_SINGLE", "CONFIG_H", "LAYOUT"):
            self.assertNotIn(leak, names, leak)

    def test_an_include_guard_is_not_reported_as_an_upstream_addition(self):
        self.assertTrue("DEFINES_2_1_20_H".startswith(rp.INCLUDE_GUARD_PREFIX))
        self.assertTrue("DEFINES_2_x_H".startswith(rp.INCLUDE_GUARD_PREFIX))


if __name__ == "__main__":
    unittest.main()
