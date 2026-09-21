#!/usr/bin/env python3
"""Tests for the target-power decision rules.

`decide()` is pure so the safety rules can be checked without a probe or a
target attached — which matters, because the failure they prevent is
destructive and cannot be exercised on the bench.
"""
import unittest

import coreprobe_power
from coreprobe_power import (
    VSHIFT_AUTO,
    PowerError,
    available_supplies,
    classify,
    decide,
    read_declaration,
)


class TestClassify(unittest.TestCase):
    """Bands from CoreProbe docs/level-shifter.md."""

    def test_bands(self):
        self.assertEqual(classify(8), "absent")
        self.assertEqual(classify(1791), "1v8")
        self.assertEqual(classify(3295), "3v3")

    def test_between_the_classes_is_anomalous_not_rounded(self):
        # The sense reads SWDIO held up by the target's own pull-up, so a real
        # target sits near a rail. 1027 mV is what an unpowered pull-up reads;
        # calling it "1v8" would drive signalling at a board that is not up.
        self.assertEqual(classify(1027), "anomalous")
        self.assertEqual(classify(2500), "anomalous")
        self.assertEqual(classify(3601), "anomalous")


class TestDecide(unittest.TestCase):
    def test_self_powered_target_is_never_fed(self):
        # Even when the project declares a supply: a board with a rail of its
        # own must never be driven.
        self.assertEqual(decide(3295, "5v"), (None, "3v3"))
        self.assertEqual(decide(1791, "5v"), (None, "1v8"))

    def test_self_powered_level_comes_from_the_sense(self):
        self.assertEqual(decide(3295, None), (None, "3v3"))

    def test_unpowered_and_undeclared_refuses_with_the_json_to_add(self):
        with self.assertRaises(PowerError) as cm:
            decide(8, None)
        msg = str(cm.exception)
        self.assertIn("target_power", msg)
        self.assertIn('"5v"', msg)
        # The level is sensed now, so the fix must not ask for it.
        self.assertNotIn("target_logic", msg)

    def test_five_volts_is_the_supply(self):
        # The level is not known yet: the board has to be up before the probe
        # can read what it runs its logic at.
        self.assertEqual(decide(8, "5v"), ("5v", None))

    def test_supply_needs_no_logic_level(self):
        # A regulated board fed 5 V can come back at 1V8 or 3V3; the probe
        # senses which. Declaring it is no longer asked for.
        self.assertEqual(decide(8, "5v"), ("5v", None))

    def test_removed_switches_are_refused_with_the_reason(self):
        for supply in ("1v8", "3v3"):
            with self.subTest(supply=supply):
                with self.assertRaises(PowerError) as cm:
                    decide(8, supply)
                self.assertIn("removed", str(cm.exception))

    def test_nonsense_supply_is_refused(self):
        with self.assertRaises(PowerError):
            decide(8, "12v")

    def test_anomalous_self_powered_target_is_reported_not_guessed(self):
        with self.assertRaises(PowerError) as cm:
            decide(2500, None)
        self.assertIn("neither 1V8", str(cm.exception))


class TestLevelShifterIsNeverSetByTheHost(unittest.TestCase):
    """The shifter must never be set before the target is sensed.

    The firmware senses and sets it itself. The host's only job is to leave it
    in AUTO, so this pins that there is no way to send a fixed level.
    """

    def test_no_function_sets_a_fixed_level(self):
        self.assertFalse(hasattr(coreprobe_power, "set_vshift"))

    def test_auto_matches_the_firmware(self):
        # VSHIFT_SEL_AUTO in CoreProbe firmware/src/target.h.
        self.assertEqual(VSHIFT_AUTO, 0x02)


class TestAvailableSupplies(unittest.TestCase):
    def test_current_boards_supply_off_or_5v(self):
        self.assertEqual(available_supplies(), ["off", "5v"])


class TestReadDeclaration(unittest.TestCase):
    def test_missing_file_reads_as_undeclared(self):
        # A project with no config.json is self-powered by default, not an error.
        self.assertEqual(read_declaration("/nonexistent/config.json"), (None, None))

    def test_absent_block_reads_as_undeclared(self):
        import json
        import tempfile

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"core": "Core.ST.W5"}, f)
            path = f.name
        self.assertEqual(read_declaration(path), (None, None))

    def test_block_is_read_back(self):
        # target_logic is still read, so a stale one can be reported.
        import json
        import tempfile

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"probe": {"target_power": "5v", "target_logic": "1v8"}}, f)
            path = f.name
        self.assertEqual(read_declaration(path), ("5v", "1v8"))


if __name__ == "__main__":
    unittest.main()
