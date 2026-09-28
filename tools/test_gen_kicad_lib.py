#!/usr/bin/env python3
"""The KiCad symbol layout rules, and that every real definition generates."""
import json
import os
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from gen_kicad_lib import DEFINITIONS_DIR, compact, layout, library, symbol, symbol_name  # noqa: E402


def fn(name, typ="digital", direction=""):
    return {"function": name, "type": typ, "direction": direction}


def tile(pads, power, family="Drive", name="X", rev="a"):
    return {"family": family, "name": name, "rev": rev,
            "package": {"type": "T44", "pads": len(pads)},
            "pads": [{"pad": str(i + 1), "functions": fs} for i, fs in enumerate(pads)],
            "power": power}


def rail(pos, gnd, direction="input", lo=1.8, hi=5.0):
    return {"positive_pad": [pos], "gnd_pad": [gnd], "direction": direction, "min": lo, "max": hi, "type": "system"}


# Drive.H-like: GND 1, signals 2-8 (6 unused), V_MOTOR 9, V+ 10.
DRIVE = tile(
    [[fn("GND", "power")], [fn("TRIG", direction="input")], [fn("EN", direction="input")],
     [fn("I2C.CLK", direction="bidirectional")], [fn("I2C.DAT", direction="bidirectional")], [],
     [fn("OUT+", "drive", "output")], [fn("OUT-", "drive", "output")],
     [fn("V_MOTOR", "power", "input")], [fn("V+", "power", "input")]],
    [rail("10", "1"), rail("9", "1", lo=2.5, hi=5.5)])

# Power.L.1T-like: each input rail has its own ground.
CHARGER = tile(
    [[fn("GND", "power")], [fn("LP")], [fn("SW")], [fn("I2C.CLK")], [fn("I2C.DAT")],
     [fn("BATT-", "power")], [fn("BATT+", "power")], [fn("SUPPLY+", "power")],
     [fn("SUPPLY-", "power")], [fn("V+", "power", "output")]],
    [rail("10", "1", "output"), rail("8", "9"), rail("7", "6")], family="Power", rev="b")

# Core-like: port names win over the function listed first.
CORE = tile(
    [[fn("GND", "power")], [fn("A0"), fn("SPI3.CLK"), fn("TIM1.2N"), fn("TIM3.3"), fn("G2.IO2")],
     [fn("BOOT0", "system"), fn("H3")], [fn("V+", "power")]],
    [rail("4", "1")], family="Core", name="ST.Q")


def names(pins):
    return [p["name"] for p in pins]


class Layout(unittest.TestCase):
    def test_supply_block_then_pad_order(self):
        power, rest = layout(DRIVE)
        self.assertEqual(names(power), ["V+", "V_MOTOR", "GND"])
        self.assertEqual(names(rest), ["TRIG", "EN", "I2C.CLK", "I2C.DAT", "", "OUT+", "OUT-"])
        self.assertEqual([p["rail"] for p in power], ["1.8-5V", "2.5-5.5V", ""])

    def test_rail_keeps_its_own_ground(self):
        power, _ = layout(CHARGER)
        self.assertEqual(names(power), ["V+", "GND", "BATT+", "BATT-", "SUPPLY+", "SUPPLY-"])

    def test_core_pins_are_named_by_port(self):
        _, rest = layout(CORE)
        self.assertEqual(names(rest), ["A0", "H3"])
        self.assertEqual(rest[1]["alts"], ["BOOT0"])

    def test_name_carries_rev_after_a(self):
        self.assertEqual(symbol_name(DRIVE), "Drive.X")
        self.assertEqual(symbol_name(CHARGER), "Power.X-b")

    def test_timers_compact(self):
        self.assertEqual(compact(["SPI3.CLK", "TIM1.2N", "ADC9", "TIM3.3"]), ["SPI3.CLK", "TIM1.2N/3.3", "ADC9"])


class Kicad(unittest.TestCase):
    def pins(self, d):
        return re.findall(r"\(pin (\S+) line\n\t+\(at [^)]*\)\n\t+\(length [^)]*\)(\n\t+\(hide yes\))?"
                          r"\n\t+\(name \"([^\"]*)\".*?\(number \"([^\"]*)\"", symbol(d), re.S)

    def test_pin_types(self):
        types = {num: t for t, _, _, num in self.pins(DRIVE)}
        self.assertEqual(types["10"], "power_in")
        self.assertEqual(types["1"], "power_in")
        self.assertEqual(types["3"], "input")
        self.assertEqual(types["7"], "output")
        self.assertEqual(types["4"], "bidirectional")
        self.assertEqual({num: t for t, _, _, num in self.pins(CHARGER)}["10"], "power_out")

    def test_unused_pad_is_a_hidden_nameless_no_connect(self):
        pad6 = [p for p in self.pins(DRIVE) if p[3] == "6"][0]
        self.assertEqual(pad6[:3], ("no_connect", "\n\t\t\t\t(hide yes)", ""))

    def test_every_function_is_a_pin_alternate(self):
        s = symbol(CORE)
        for f in ("SPI3.CLK", "TIM1.2N", "TIM3.3", "G2.IO2", "BOOT0"):
            self.assertIn(f'(alternate "{f}"', s)
        self.assertNotIn("G2.IO2,", s)  # touch-sense stays out of the body text

    def test_fields(self):
        s = symbol(DRIVE)
        self.assertIn('(property "Footprint" "Bergsonne Tiles:T44-10"', s)
        self.assertEqual(re.findall(r'\(property "([^"]+)"', s),
                         ["Reference", "Value", "Footprint", "Datasheet", "Description", "ki_keywords"])


class EveryDefinition(unittest.TestCase):
    """Every definition with pads must generate, one pin per pad, parens balanced."""

    def test_all(self):
        defs = [json.loads(p.read_text(encoding="utf-8")) for p in sorted(Path(DEFINITIONS_DIR).glob("*.json"))]
        defs = [d for d in defs if d.get("pads")]
        self.assertTrue(defs)
        text = library(defs)
        self.assertEqual(text.count("("), text.count(")"))
        for d in defs:
            with self.subTest(tile=symbol_name(d)):
                s = symbol(d)
                self.assertEqual(sorted(re.findall(r'\(number "([^"]+)"', s)),
                                 sorted(p["pad"] for p in d["pads"]))


if __name__ == "__main__":
    unittest.main()
