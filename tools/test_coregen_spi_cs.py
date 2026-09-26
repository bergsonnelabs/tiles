#!/usr/bin/env python3
"""Unit tests for SPI chip-select resolution in coregen.

Several SPI tiles can share one bus, each with its own chip-select pad
(config.json tiles[].cs_pad). coregen claims those pads, emits a per-bus
chip-select map keyed by tile instance (the `cs` value the driver passes to
the tile bridge), and refuses configurations where two tiles would answer the
same transfer. tests/hw-spi-loopback (TWO_TILES=1) covers the firmware side on
a bench; these tests cover every way the config can be wrong.

Run:
    python3 tools/test_coregen_spi_cs.py
"""

import contextlib
import copy
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools" / "coregen"))
import coregen  # noqa: E402

L4 = ROOT / "definitions" / "Core-ST-L4-b.json"
with open(L4, encoding="utf-8") as f:
    _TILE = json.load(f)
PAD_MAP = coregen.build_pad_map(_TILE["pads"])
MCU = coregen.MCU_DB[_TILE["components"][0]["part"]]

# Core.ST.L4 (not the L4.2): pad 9 = PA4 (SPI1 NSS pad), pad 4 = PB6, pad 5 = PC15, pad 6 = PA11 (USB D-)
BUS = {"2": "SPI1.MOSI", "3": "SPI1.CLK", "8": "SPI1.MISO"}


def cfg(pads=None, tiles=None):
    c = {"core": "Core.ST.L4", "pads": dict(BUS, **(pads or {})),
         "interfaces": {"SPI1": {"mode": 0, "prescaler": 8}}}
    if tiles is not None:
        c["tiles"] = tiles
    return c


def tile(instance, cs_pad=None, kind="Sense.I.6P6", bus="SPI1"):
    t = {"tile": kind, "bus": bus, "instance": instance}
    if cs_pad is not None:
        t["cs_pad"] = cs_pad
    return t


# Core.ST.W5, as on the bench: SPI3 on pads 2/6/7, I2C1 on pads 10/11,
# pad 3 = PA5 and pad 4 = PA6 (free GPIOs used as chip selects).
W5 = ROOT / "definitions" / "Core-ST-W5-b.json"
with open(W5, encoding="utf-8") as f:
    _W5 = json.load(f)
W5_PAD_MAP = coregen.build_pad_map(_W5["pads"])
W5_MCU = coregen.MCU_DB[_W5["components"][0]["part"]]
W5_BUS = {"2": "SPI3.CLK", "6": "SPI3.MOSI", "7": "SPI3.MISO", "10": "I2C1.CLK", "11": "I2C1.DAT"}


def w5cfg(tiles, pads=None):
    return {"core": "Core.ST.W5", "pads": dict(W5_BUS, **(pads or {})),
            "interfaces": {"SPI3": {"mode": 0, "prescaler": 8}}, "tiles": tiles}


def cam(cs_pad=None, cs_id=None, bus="I2C1", bus2="SPI3", instance=0):
    """A dual-bus tile: Sense.CAM.P, I2C for configuration and SPI for data."""
    t = {"tile": "Sense.CAM.P", "bus": bus, "bus2": bus2, "instance": instance}
    if cs_pad is not None:
        t["cs_pad"] = cs_pad
    if cs_id is not None:
        t["cs_id"] = cs_id
    return t


def spi_w5(config):
    """spi() for the W5: returns (SPI3 bus dict, errors, stderr)."""
    config = copy.deepcopy(config)
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            errors = coregen.validate_tile_buses(config) + coregen.merge_tile_cs_pads(config)
            _, verrs = coregen.validate_project_config(config, _W5, W5_PAD_MAP, W5_MCU)
            errors += verrs
            if errors:
                return None, errors, err.getvalue()
            buses = coregen.build_spi_config(config, W5_MCU, W5_PAD_MAP)
    except SystemExit:
        sys.stderr.write(err.getvalue())
        raise
    return next(b for b in buses if b["instance"] == "SPI3"), errors, err.getvalue()


def spi(config):
    """merge + validate + build_spi_config, as generate() runs them.
    Returns (bus dict, errors, stderr text); raises SystemExit on a build error."""
    config = copy.deepcopy(config)
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            errors = coregen.validate_tile_buses(config) + coregen.merge_tile_cs_pads(config)
            _, verrs = coregen.validate_project_config(config, _TILE, PAD_MAP, MCU)
            errors += verrs
            if errors:
                return None, errors, err.getvalue()
            buses = coregen.build_spi_config(config, MCU, PAD_MAP)
    except SystemExit:
        sys.stderr.write(err.getvalue())   # the caller's capture sees the ERROR
        raise
    return buses[0], errors, err.getvalue()


class TestSingleChipSelect(unittest.TestCase):
    def test_bus_cs_pad_without_tiles_is_unchanged(self):
        bus, errors, _ = spi(cfg({"9": "SPI1.CS"}))
        self.assertEqual(errors, [])
        self.assertEqual((bus["cs_pad"], bus["cs_port"], bus["cs_pin"]), ("9", "A", 4))
        self.assertEqual(bus["cs_map"], [])

    def test_one_tile_cs_pad_only_in_tiles_becomes_the_bus_cs(self):
        bus, errors, _ = spi(cfg(tiles=[tile(0, "9")]))
        self.assertEqual(errors, [])
        self.assertEqual(bus["cs_pad"], "9")
        self.assertEqual(bus["cs_map"], [])

    def test_one_tile_on_a_non_nss_gpio_pad(self):
        # PB6 has no SPI1 NSS function: a software CS is valid on any GPIO pad.
        bus, errors, _ = spi(cfg(tiles=[tile(0, 4)]))
        self.assertEqual(errors, [])
        self.assertEqual((bus["cs_pad"], bus["cs_port"], bus["cs_pin"]), ("4", "B", 6))

    def test_gpio_out_cs_pad_becomes_the_chip_select(self):
        c = cfg({"4": "GPIO.OUT"}, tiles=[tile(0, "4")])
        errs = coregen.merge_tile_cs_pads(c)
        self.assertEqual(errs, [])
        self.assertEqual(c["pads"]["4"], "SPI1.CS")

    def test_cs_pad_is_initialized_high(self):
        c = cfg(tiles=[tile(0, "4")])
        coregen.merge_tile_cs_pads(c)
        pad = next(p for p in coregen.build_pad_config(c, PAD_MAP) if p["pad"] == "4")
        self.assertEqual((pad["mode"], pad["default"]), ("output", "high"))

    def test_gpio_section_without_default_keeps_cs_high(self):
        c = cfg({"9": "SPI1.CS"})
        c["gpio"] = {"9": {"speed": "high"}}
        pad = next(p for p in coregen.build_pad_config(c, PAD_MAP) if p["pad"] == "9")
        self.assertEqual(pad["default"], "high")


class TestPerTileChipSelects(unittest.TestCase):
    def test_two_tiles_distinct_cs_get_a_map_keyed_by_instance(self):
        bus, errors, _ = spi(cfg(tiles=[tile(0, "9"), tile(1, "4")]))
        self.assertEqual(errors, [])
        self.assertIsNone(bus["cs_pad"])
        self.assertEqual([(e["id"], e["pad"], e["port"], e["pin"]) for e in bus["cs_map"]],
                         [(0, "9", "A", 4), (1, "4", "B", 6)])

    def test_several_bus_cs_pads_do_not_collapse_to_the_last(self):
        # Studio mirrors each tile's cs_pad into pads as "SPI1.CS".
        bus, errors, _ = spi(cfg({"9": "SPI1.CS", "4": "SPI1.CS"},
                                 tiles=[tile(0, "9"), tile(1, "4")]))
        self.assertEqual(errors, [])
        self.assertEqual({e["pad"] for e in bus["cs_map"]}, {"9", "4"})

    def test_mixed_tile_types(self):
        bus, _, _ = spi(cfg(tiles=[tile(0, "9", "Store.O.128"), tile(1, "4", "Sense.I.6P6")]))
        self.assertEqual([(e["id"], e["tile"]) for e in bus["cs_map"]],
                         [(0, "Store.O.128"), (1, "Sense.I.6P6")])

    def test_unclaimed_cs_pad_is_a_note(self):
        bus, errors, stderr = spi(cfg({"5": "SPI1.CS"}, tiles=[tile(0, "9"), tile(1, "4")]))
        self.assertEqual(errors, [])
        self.assertEqual(len(bus["cs_map"]), 2)
        self.assertIn("NOTE", stderr)
        self.assertIn("5", stderr)

    def test_several_cs_pads_and_no_tiles(self):
        bus, errors, stderr = spi(cfg({"9": "SPI1.CS", "4": "SPI1.CS"}))
        self.assertEqual(errors, [])
        self.assertIsNone(bus["cs_pad"])
        self.assertEqual(bus["cs_map"], [])
        self.assertIn("core_spi_select() drives none", stderr)


class TestRejected(unittest.TestCase):
    def assertExits(self, config, *needles):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            with self.assertRaises(SystemExit):
                bus, errors, _ = spi(config)
                if errors:          # validation errors exit in generate()
                    sys.stderr.write("\n".join(errors))
                    raise SystemExit(1)
        text = err.getvalue()
        for n in needles:
            self.assertIn(n, text)
        return text

    def test_duplicate_cs_pad(self):
        self.assertExits(cfg(tiles=[tile(0, "9"), tile(1, "9")]),
                         "both use cs_pad 9", "its own chip-select pad")

    def test_duplicate_instance(self):
        self.assertExits(cfg(tiles=[tile(0, "9"), tile(0, "4", "Store.O.128")]),
                         "both have chip-select id 0")

    def test_two_tiles_one_cs_pad(self):
        self.assertExits(cfg({"9": "SPI1.CS"}, tiles=[tile(0), tile(1)]),
                         "carries 2 tiles", "only one chip-select pad (9)")

    def test_missing_cs_pad_when_bus_has_several(self):
        self.assertExits(cfg({"4": "SPI1.CS"}, tiles=[tile(0, "9"), tile(1)]),
                         "has no cs_pad", "Name the tile's cs_pad")

    def test_cs_pad_taken_by_another_function(self):
        self.assertExits(cfg(tiles=[tile(0, "8")]), "is assigned 'SPI1.MISO'")

    def test_cs_pad_on_the_usb_pads(self):
        self.assertExits(cfg(tiles=[tile(0, "6")]), "USB")

    def test_cs_pad_does_not_invent_an_unconfigured_bus(self):
        c = cfg(tiles=[tile(0, "4", bus="SPI3")])
        self.assertEqual(coregen.merge_tile_cs_pads(c), [])
        self.assertNotIn("4", c["pads"])      # build_tiles_config then reports the bus

    def test_cs_pad_of_another_bus(self):
        self.assertExits(cfg({"4": "SPI3.CS"}, tiles=[tile(0, "4")]), "is assigned 'SPI3.CS'")


class TestDualBusTiles(unittest.TestCase):
    """A dual-bus tile (bus I2C + bus2 SPI) selects by its cs_id, not its
    instance, which is its I2C address slot."""

    def test_alone_keeps_the_bus_cs(self):
        # As before #294: the lone CS pad is the bus's own; spi_cs 0 works.
        bus, errors, _ = spi_w5(w5cfg([cam(cs_pad="4")]))
        self.assertEqual(errors, [])
        self.assertEqual((bus["cs_pad"], bus["cs_port"], bus["cs_pin"]), ("4", "A", 6))
        self.assertEqual(bus["cs_map"], [])

    def test_alone_with_bus_cs_pad_and_no_cs_pad(self):
        bus, errors, _ = spi_w5(w5cfg([cam()], pads={"3": "SPI3.CS"}))
        self.assertEqual(errors, [])
        self.assertEqual(bus["cs_pad"], "3")

    def test_cs_pad_is_claimed_on_bus2(self):
        c = w5cfg([cam(cs_pad="4")])
        self.assertEqual(coregen.merge_tile_cs_pads(c), [])
        self.assertEqual(c["pads"]["4"], "SPI3.CS")

    def test_with_a_plain_spi_tile_gets_the_next_free_id(self):
        bus, errors, _ = spi_w5(w5cfg([tile(0, "3", bus="SPI3"), cam(cs_pad="4")]))
        self.assertEqual(errors, [])
        self.assertIsNone(bus["cs_pad"])
        self.assertEqual([(e["id"], e["pad"], e["tile"]) for e in bus["cs_map"]],
                         [(0, "3", "Sense.I.6P6"), (1, "4", "Sense.CAM.P")])

    def test_default_id_skips_explicit_ids_in_any_order(self):
        # The camera comes first in tiles[], but the plain tile's instance 0
        # is taken before defaults are handed out.
        bus, _, _ = spi_w5(w5cfg([cam(cs_pad="4"), tile(0, "3", bus="SPI3")]))
        self.assertEqual({e["pad"]: e["id"] for e in bus["cs_map"]}, {"4": 1, "3": 0})

    def test_explicit_cs_id(self):
        bus, errors, _ = spi_w5(w5cfg([tile(0, "3", bus="SPI3"), cam(cs_pad="4", cs_id=5)]))
        self.assertEqual(errors, [])
        self.assertEqual({e["pad"]: e["id"] for e in bus["cs_map"]}, {"3": 0, "4": 5})

    def test_instance_is_not_the_key(self):
        # Camera instance 1 (I2C address slot) and a plain tile at instance 1:
        # no clash, the camera's id is its cs_id (default 0).
        bus, errors, _ = spi_w5(w5cfg([tile(1, "3", bus="SPI3"), cam(cs_pad="4", instance=1)]))
        self.assertEqual(errors, [])
        self.assertEqual({e["pad"]: e["id"] for e in bus["cs_map"]}, {"3": 1, "4": 0})

    def test_cs_id_collision(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
            spi_w5(w5cfg([tile(1, "3", bus="SPI3"), cam(cs_pad="4", cs_id=1)]))
        self.assertIn("both have chip-select id 1", err.getvalue())

    def test_dual_and_plain_sharing_one_cs_pad(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
            spi_w5(w5cfg([tile(0, bus="SPI3"), cam()], pads={"3": "SPI3.CS"}))
        self.assertIn("carries 2 tiles", err.getvalue())

    def assertBusError(self, config, needle):
        _, errors, _ = spi_w5(config)
        self.assertTrue(any(needle in e for e in errors), errors)

    def test_bus2_same_protocol(self):
        self.assertBusError(w5cfg([cam(bus="I2C1", bus2="I2C3", cs_pad="4")]), "same protocol")

    def test_bus2_not_configured(self):
        self.assertBusError(w5cfg([cam(bus2="SPI1", cs_pad="4")]), "bus2 'SPI1' is not configured")

    def test_bus2_not_i2c_or_spi(self):
        self.assertBusError(w5cfg([cam(bus2="USART2", cs_pad="4")]), "is not an I2C bus and an SPI bus")

    def test_cs_id_on_a_plain_tile_must_equal_instance(self):
        t = tile(0, "3", bus="SPI3")
        t["cs_id"] = 2
        self.assertBusError(w5cfg([t]), "differs from its instance")

    def test_cs_id_out_of_range(self):
        self.assertBusError(w5cfg([cam(cs_pad="4", cs_id=300)]), "must be an integer 0-255")


class TestGenerated(unittest.TestCase):
    """End to end: the emitted core_init.c attaches the map before any transfer."""

    def render(self, config, core=L4):
        with tempfile.TemporaryDirectory() as d:
            proj = os.path.join(d, "proj")
            os.makedirs(proj)
            path = os.path.join(proj, "config.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(config, f)
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                coregen.generate(str(core), os.path.join(d, "out"), path)
            with open(os.path.join(d, "out", "core_init.c"), encoding="utf-8") as f:
                return f.read()

    def test_map_emitted_after_high_pads(self):
        c = cfg(tiles=[tile(0, "9"), tile(1, "4")])
        src = self.render(c)
        self.assertIn("{ GPIOA, 4, 0 },", src)
        self.assertIn("{ GPIOB, 6, 1 },", src)
        self.assertIn("hal_spi_set_cs_map(&core_spi1, _core_spi1_cs_map, 2);", src)
        self.assertNotIn("hal_spi_set_cs(&core_spi1", src)
        # both pads are set high before they become outputs, before SPI init
        spi_init = src.index("hal_spi_init(&core_spi1")
        for port, pin in (("A", 4), ("B", 6)):
            hi = src.index(f"ll_gpio_set(GPIO{port}, 1UL << {pin});")
            out = src.index(f"ll_gpio_config_output(GPIO{port}, {pin});")
            self.assertLess(hi, out)
            self.assertLess(out, spi_init)

    def test_dual_bus_map_and_recipe(self):
        c = w5cfg([tile(0, "3", bus="SPI3"), cam(cs_pad="4")])
        src = self.render(c, W5)
        self.assertIn("{ GPIOA, 5, 0 },", src)
        self.assertIn("{ GPIOA, 6, 1 },", src)
        self.assertIn("hal_spi_set_cs_map(&core_spi3, _core_spi3_cs_map, 2);", src)

    def test_single_cs_unchanged(self):
        src = self.render(cfg({"9": "SPI1.CS"}))
        self.assertIn("hal_spi_set_cs(&core_spi1, GPIOA, 4);", src)
        self.assertNotIn("cs_map", src)


if __name__ == "__main__":
    unittest.main(verbosity=1)
