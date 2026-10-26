#!/usr/bin/env python3
"""
coregen — Generate C headers from Mosaic tile JSON definitions.

Usage:
    python3 coregen.py <tile.json> [output_dir] [--config config.json]
                       [--status core-x.json] [--port stm32]

Reads a tile JSON file and produces:
    core_caps.h        The Core's CORE_HAS_* from its status file (--status;
                       tools/coregen/core_caps.py)
    core_pads.h        Pad-to-GPIO mapping defines
    core_board.h       Board-level defines (LED, power, debug)
    core_interfaces.h  Interface convenience defines with AF numbers
    core_config.h      Project-specific pin and clock configuration
                       (only when --project is provided)

Output defaults to ./generated/ if not specified.
"""

import argparse
import importlib.util
import ipaddress
import json
import os
import re
import sys
from pathlib import Path

from jinja2 import Environment, FileSystemLoader

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core_caps  # noqa: E402  (tools/coregen/core_caps.py)


# ---- Console encoding ----------------------------------------------------
# Windows Python falls back to the legacy ANSI code page (cp1252) whenever
# stdout isn't a real console — which is exactly the case under make, where it
# is a pipe. Any non-ASCII in our progress output then raises
# UnicodeEncodeError and takes the build down with it. Force UTF-8 on the
# streams we own; errors="replace" keeps a terminal that genuinely cannot
# render a character from crashing over it.
for _stream in (sys.stdout, sys.stderr):
    try:
        if (getattr(_stream, "encoding", "") or "").lower().replace("-", "") != "utf8":
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError, ValueError):
        pass


def eprint(*args, **kwargs):
    """Print to stderr. coregen's stdout is redirected to /dev/null in quiet
    (V=0) builds — the path the Studio build service uses — so fatal errors MUST
    go to stderr or they vanish, leaving only a cryptic 'No rule to make target
    core_drivers.mk' from the swallowed failure. All ERROR diagnostics use this."""
    print(*args, file=sys.stderr, **kwargs)


# ---- Core naming ----
# Public names (Core.ST.<family>[.<n>], matching each board's silkscreen) are
# the standard; they map onto the DB-synced definition file stems. The Makefile
# carries the same map so either form resolves to the right definition.
#
# Core.ST.L4 and Core.ST.L4.2 are two DIFFERENT boards (same STM32L422; the
# L4.2 has more pads). Core.ST.L4 was called Core.ST.L4.1 until the 2026-09
# rename.
CORE_NAME_ALIASES = {
    "Core.ST.L0": "Core-ST-L0-a",
    "Core.ST.L4": "Core-ST-L4-b",  # rev b — superset of rev a (adds PA13/PA14)
    "Core.ST.L4.2": "Core-ST-L4-2-a",
    "Core.ST.W5": "Core-ST-W5-b",
    "Core.ST.H5": "Core-ST-H5-a",
}
CORE_STEM_TO_PUBLIC = {stem: name for name, stem in CORE_NAME_ALIASES.items()}

# Names retired by the 2026-09 rename (the Cores renamed to their silkscreens),
# old public name or old stem -> current one. Still accepted in config.json
# "core" (and as the Makefile's TILE), with a one-line NOTE, so older projects
# keep building.
CORE_DEPRECATED_NAMES = {
    "Core.ST.L0.1": "Core.ST.L0",
    "Core.ST.L4.1": "Core.ST.L4",
    "Core.ST.H5.1": "Core.ST.H5",
    "Core-ST-L0-1-a": "Core-ST-L0-a",
    "Core-ST-L4-1-a": "Core-ST-L4-a",
    "Core-ST-L4-1-b": "Core-ST-L4-b",
    "Core-ST-H5-1-a": "Core-ST-H5-a",
}


def core_public_name(stem):
    """Public name for a definition stem: the alias table first, else the stem
    itself read back (family-name-rev, dots in the name became dashes), so
    Core-ST-L4-a -> Core.ST.L4 and mCore-ST-C0-a -> mCore.ST.C0. Deliberately
    not the JSON's own family/name fields: those can lag a DB rename."""
    if stem in CORE_STEM_TO_PUBLIC:
        return CORE_STEM_TO_PUBLIC[stem]
    base = stem.rsplit("-", 1)[0] if "-" in stem else stem
    return base.replace("-", ".")


def resolve_core_name(name):
    """config.json "core" -> (definition stem, deprecation note or None)."""
    note = None
    if name in CORE_DEPRECATED_NAMES:
        new = CORE_DEPRECATED_NAMES[name]
        note = (f"NOTE: config.json \"core\": \"{name}\" is now \"{new}\" "
                f"(the old name still works; update config.json)")
        name = new
    return CORE_NAME_ALIASES.get(name, name), note


# ---- Back ends (ports) ----
#
# Everything vendor-specific (the MCU table, clock tree, pin mux, peripheral
# clock enables) lives in a back end, tools/coregen/ports/<port>.py, picked by
# --port (the Makefile passes $(PORT) from the Core's core.mk; default stm32).
# A port that is not public ships its back end in its private repo, at
# <overlay>/tools/coregen/ports/<port>.py, found through TILES_OVERLAY. The
# interface is documented at the top of ports/stm32.py.

DEFAULT_PORT = "stm32"

# What a back end must define (see ports/stm32.py).
BACKEND_HOOKS = (
    "PORT", "mcu_for_part", "template_dirs", "pad_gpio", "pad_mux",
    "timer_channels", "led_info", "validate", "usb_capable", "i3c_unsupported",
    "clock_config", "i2c_config", "spi_config", "usart_config", "pwm_config",
    "extra_context",
)

_BACKENDS = {}       # port name -> loaded module
_BACKEND = None      # the active back end


def backend_path(name):
    """tools/coregen/ports/<name>.py in this SDK, else in the first
    TILES_OVERLAY checkout that has one, or None."""
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ports", f"{name}.py")
    candidates = [here] + [
        os.path.join(os.path.expanduser(o), "tools", "coregen", "ports", f"{name}.py")
        for o in os.environ.get("TILES_OVERLAY", "").split()
    ]
    return next((c for c in candidates if os.path.isfile(c)), None)


def load_backend(name):
    """Import the back end for port `name` (by path; cached). Exits with an
    ERROR when there is none or it lacks part of the interface."""
    if name in _BACKENDS:
        return _BACKENDS[name]
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', str(name)):
        eprint(f"ERROR: --port '{name}' is not a port name (letters, digits, '_')")
        sys.exit(1)
    path = backend_path(name)
    if path is None:
        overlays = " ".join(os.environ.get("TILES_OVERLAY", "").split())
        eprint(f"ERROR: Unknown port '{name}': no coregen back end tools/coregen/ports/{name}.py "
               f"in this SDK" + (f" or in TILES_OVERLAY ({overlays})" if overlays else "")
               + f". A port that is not public has its back end in its private repo: "
               f"build with TILES_OVERLAY=/path/to/<that repo's checkout>")
        sys.exit(1)
    spec = importlib.util.spec_from_file_location(f"coregen_port_{name}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    # No ports/__pycache__/ next to the back end: .gitignore re-includes all
    # of tools/coregen/, and an overlay's checkout is not ours to litter.
    dont_write, sys.dont_write_bytecode = sys.dont_write_bytecode, True
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = dont_write
    missing = [h for h in BACKEND_HOOKS if not hasattr(mod, h)]
    if missing:
        eprint(f"ERROR: coregen back end {path} does not define {', '.join(missing)} "
               f"(the interface is documented in tools/coregen/ports/stm32.py)")
        sys.exit(1)
    if mod.PORT != name:
        eprint(f"ERROR: coregen back end {path} says PORT = {mod.PORT!r}, expected {name!r}")
        sys.exit(1)
    _BACKENDS[name] = mod
    return mod


def use_backend(port):
    """Make `port` (a name or a loaded back end) the active back end."""
    global _BACKEND
    _BACKEND = load_backend(port) if isinstance(port, str) else port
    return _BACKEND


def backend():
    """The active back end (stm32 unless --port / generate(port=) chose another)."""
    return _BACKEND or use_backend(DEFAULT_PORT)


def extract_system_pads(pads):
    """Find system pads: SWCLK, SWDIO, BOOT0, NRST, V+, GND."""
    system = {}
    for pad in pads:
        for func in pad.get("functions", []):
            fname = func.get("function", "")
            if fname in ("SWCLK", "SWDIO", "BOOT0", "NRST", "GND", "V+"):
                key = fname.replace("+", "PLUS")
                system[key] = pad["pad"]
    return system


def resolve_clock_block(tile):
    """Read the tile's clock data from the post-2026-05-01 schema.

    Capability (sources, boot_source) lives under `features.clock`; the
    user-pickable configurations live as options on the `config.clock`
    select knob, with each option's clock-tree state declared as a
    `firmware_contract` entry of type "clock". This helper rebuilds the
    legacy `(sources, configurations, knob_default)` shape for the rest
    of coregen to consume.
    """
    features_clock = (tile.get("features") or {}).get("clock") or {}
    config_clock = (tile.get("config") or {}).get("clock") or {}
    sources = features_clock.get("sources", [])
    configurations = []
    for opt in config_clock.get("options", []):
        # Pull clock state from firmware_contract[type=clock]; fall back
        # to legacy `derived` for tiles that haven't been migrated yet.
        clock_entry = next(
            (fc for fc in (opt.get("firmware_contract") or []) if fc.get("type") == "clock"),
            None,
        )
        cfg = dict(clock_entry or opt.get("derived") or {})
        cfg.pop("type", None)
        cfg["name"] = opt.get("value")
        configurations.append(cfg)
    knob_default = config_clock.get("default", "medium")
    return sources, configurations, knob_default


def extract_adc_channel(pad):
    """Extract ADC channel number from a pad's analog functions.

    Looks for functions like 'ADC7', 'ADC11', 'ADC_IN5', etc.
    Returns the channel number as int, or None if no ADC function exists.
    """
    for func in pad.get("functions", []):
        if func.get("type") == "analog":
            fname = func["function"]
            m = re.match(r'ADC_?(?:IN)?(\d+)', fname)
            if m:
                return int(m.group(1))
    return None



def build_pad_map(pads):
    """Build a list of pad info dicts for template rendering."""
    be = backend()
    pad_map = []
    for pad in pads:
        port, pin = be.pad_gpio(pad)

        # Collect all available functions. A function's pin-mux value (its
        # "af" here, which every reader below and the templates use) is the
        # back end's pad_mux(): `mux` in the definition, or the older `af`.
        all_functions = []
        af_functions = []
        for func in pad.get("functions", []):
            all_functions.append(func["function"])
            mux = be.pad_mux(func)
            if mux is not None:
                af_functions.append({
                    "function": func["function"],
                    "type": func["type"],
                    "af": mux,
                })

        # Extract ADC channel if present
        adc_channel = extract_adc_channel(pad)

        # Extract timer PWM channels if present
        timer_channels = be.timer_channels(pad)

        # Classify the pad
        pad_type = "gpio"
        for func in pad.get("functions", []):
            if func.get("function") in ("GND", "V+"):
                pad_type = "power"
                break
            if func.get("function") == "NRST":
                pad_type = "system"
                break

        pad_map.append({
            "number": pad["pad"],
            "port": port,
            "pin": pin,
            "type": pad_type,
            "all_functions": all_functions,
            "af_functions": af_functions,
            "adc_channel": adc_channel,
            "timer_channels": timer_channels,
        })

    return pad_map


def sanitize_signal_name(name):
    """Make a signal name safe for C identifiers."""
    name = name.replace("+", "P").replace("-", "M")
    name = re.sub(r'[^A-Za-z0-9_]', '_', name)
    return name


def build_interface_map(tile, pad_map):
    """Build interface info with resolved GPIO ports/pins/AFs."""
    pad_lookup = {p["number"]: p for p in pad_map}

    interfaces = []
    for iface in tile.get("interfaces", []):
        signals = []
        seen_signals = {}

        for assign in iface.get("pad_assignments", []):
            pad_num = assign["pad"]
            pad_info = pad_lookup.get(pad_num, {})
            fname = assign["function"]

            af = None
            for af_func in pad_info.get("af_functions", []):
                if af_func["function"] == fname:
                    af = af_func["af"]
                    break

            raw_signal = fname.split(".")[-1] if "." in fname else fname
            signal = sanitize_signal_name(raw_signal)

            is_required = assign.get("is_required", False)
            if signal in seen_signals:
                seen_signals[signal] += 1
                signal = f"{signal}_ALT{seen_signals[signal]}" if seen_signals[signal] > 1 else f"{signal}_ALT"
            else:
                seen_signals[signal] = 0

            signals.append({
                "pad": pad_num,
                "function": fname,
                "signal": signal,
                "port": pad_info.get("port"),
                "pin": pad_info.get("pin"),
                "af": af,
                "is_required": is_required,
            })

        interfaces.append({
            "name": iface["name"],
            "type": iface["type"],
            "parameters": iface.get("parameters", {}),
            "signals": signals,
        })

    return interfaces


# ---- Project config validation ----

def validate_project_config(config, tile, pad_map, mcu=None):
    """Validate a project config against a tile definition.

    Returns (warnings, errors) where each is a list of strings.
    """
    warnings = []
    errors = []

    # Build lookup of available functions per pad
    pad_lookup = {p["number"]: p for p in pad_map}

    # Validate pad assignments
    pins = config.get("pads", config.get("pins", {}))
    for pad_num, assigned_func in pins.items():
        if pad_num not in pad_lookup:
            errors.append(f"Pad {pad_num}: does not exist on this tile (has {len(pad_lookup)} pads)")
            continue

        pad_info = pad_lookup[pad_num]

        # GPIO.OUT and GPIO.IN are synthetic — always valid on GPIO pads. So is
        # SPIn.CS: a chip select is a GPIO output the SPI HAL drives (never the
        # hardware NSS), so any GPIO pad can be one (a tile's tiles[].cs_pad).
        if assigned_func in ("GPIO.OUT", "GPIO.IN") or re.match(r'^SPI\d+\.CS$', assigned_func):
            if pad_info["port"] is None:
                errors.append(f"Pad {pad_num}: cannot use {assigned_func} on a non-GPIO pad")
            continue

        # I3C is requested on an I2C bus ("interfaces": {"I2C1": {"i3c": true}}),
        # never by pad function: coregen switches the bus to I3C only when every
        # tile on it speaks I3C, and otherwise keeps the I2C bus the pads name.
        if re.match(r'^I3C\d+\.', str(assigned_func)):
            i2c_fn = next((f for f in pad_info["all_functions"]
                           if re.match(r'^I2C\d+\.', f) and f.split('.')[-1] == str(assigned_func).split('.')[-1]),
                          None)
            hint = f"assign '{i2c_fn}' and add \"i3c\": true to {i2c_fn.split('.')[0]} in \"interfaces\"" \
                if i2c_fn else "request I3C with \"i3c\": true on an I2C bus in \"interfaces\""
            errors.append(
                f"Pad {pad_num}: '{assigned_func}' can't be assigned directly. I3C runs on an "
                f"I2C bus: {hint} (config-json.md §7)."
            )
            continue

        # Check if the assigned function exists on this pad
        if assigned_func not in pad_info["all_functions"]:
            available = [f for f in pad_info["all_functions"]
                         if f not in ("GND", "V+", "NRST")]
            errors.append(
                f"Pad {pad_num}: '{assigned_func}' is not available. "
                f"Options: {', '.join(available)}"
            )

    # Port-specific pad rules (STM32: Core.ST.L4's always-on USB pads).
    errors.extend(backend().validate(config, tile, pad_map, mcu))

    # On-tile pull-ups switched by a GPIO (tile config "pullups", a DB-owned
    # multiselect; Core.ST.L4 has pad 4 via PC15 and pad 5 via PA9).
    for msg in validate_pullups(config, tile, pad_map):
        errors.append(msg)

    # Validate interface configs reference real interfaces
    iface_names = {i["name"] for i in tile.get("interfaces", [])}
    for iface_name in config.get("interfaces", {}):
        if iface_name not in iface_names:
            errors.append(
                f"Interface '{iface_name}': not found on this tile. "
                f"Available: {', '.join(sorted(iface_names))}"
            )

    # I3C request on an I2C bus: "i3c" is a bool, "i3c_speed" the push-pull SCL.
    for iface_name, icfg in (config.get("interfaces", {}) or {}).items():
        if not isinstance(icfg, dict) or ("i3c" not in icfg and "i3c_speed" not in icfg):
            continue
        if not re.match(r'^I2C\d+$', iface_name):
            errors.append(f"Interface '{iface_name}': \"i3c\" applies to an I2C bus (I2C1, I2C3, ...)")
            continue
        if "i3c" in icfg and not isinstance(icfg["i3c"], bool):
            errors.append(f"Interface '{iface_name}': \"i3c\" must be true or false, got {icfg['i3c']!r}")
        sp = icfg.get("i3c_speed")
        if sp is not None and (not isinstance(sp, int) or isinstance(sp, bool) or not I3C_MIN_SPEED <= sp <= I3C_MAX_SPEED):
            errors.append(f"Interface '{iface_name}': \"i3c_speed\" {sp!r} must be an integer "
                          f"{I3C_MIN_SPEED}-{I3C_MAX_SPEED} (Hz, push-pull SCL)")

    # Check that pin assignments are consistent with interface configs
    configured_ifaces = set(config.get("interfaces", {}).keys())
    assigned_ifaces = set()
    for pad_num, func in pins.items():
        if "." in func and func not in ("GPIO.OUT", "GPIO.IN"):
            iface = func.split(".")[0]
            assigned_ifaces.add(iface)

    for iface in configured_ifaces - assigned_ifaces:
        warnings.append(
            f"Interface '{iface}' configured but no pins assigned to it"
        )

    # Validate tile readdress_gpio straps reference a declared GPIO.OUT pad.
    # Without this, a typo'd or undeclared strap pad has no build-time signal —
    # the readdress dance just silently fails to mux the pin at runtime.
    for tile_entry in config.get("tiles", []):
        rg = tile_entry.get("readdress_gpio")
        if rg is None:
            continue
        rg_key = str(rg)
        if pins.get(rg_key) != "GPIO.OUT":
            tname = tile_entry.get("tile", tile_entry.get("type", "?"))
            found = pins.get(rg_key, "not declared")
            errors.append(
                f"Tile '{tname}': readdress_gpio pad {rg} must be declared as "
                f"GPIO.OUT in 'pads' (pad {rg} is: {found})"
            )

    # Validate clock performance level
    _sources, _configurations, _knob_default = resolve_clock_block(tile)
    clock = config.get("clock", _knob_default)
    # "default" is an alias for the tile's schema default (config.clock.default).
    if clock == "default":
        clock = _knob_default
    if clock:
        configs = {c["name"]: c for c in _configurations}
        if clock not in configs:
            available = ", ".join(sorted(configs.keys())) if configs else "(none defined)"
            errors.append(
                f"Clock level '{clock}' not available on this tile. "
                f"Options: {available}"
            )

    # Validate bootloader mode
    bootloader = config.get("bootloader", "none")
    valid_boot_modes = {"none", "custom", "rom"}
    if bootloader not in valid_boot_modes:
        errors.append(
            f"Bootloader mode '{bootloader}' is not valid. "
            f"Options: {', '.join(sorted(valid_boot_modes))}"
        )
    if bootloader != "none":
        mcu_define = mcu["define"] if mcu else ""
        if not backend().usb_capable(mcu):
            warnings.append(
                f"Bootloader '{bootloader}' requires USB — "
                f"{mcu_define} does not support USB CDC"
            )

    return warnings, errors


def _tile_type(tile_entry):
    return tile_entry.get("tile", tile_entry.get("type", ""))


def _configured_buses(pads):
    """Buses some pad creates (a chip select alone does not create one)."""
    out = set()
    for f in (pads or {}).values():
        m = re.match(r'^((?:SPI|I2C|I3C)\d+)\.(CLK|MOSI|MISO|DAT|SDA|SCL)$', str(f))
        if m:
            out.add(m.group(1))
    return out


def tile_spi_bus(tile_entry):
    """The SPI bus a tile's chip select is on: `bus` when that is SPI, else
    `bus2` when that is (a dual-bus tile such as Sense.CAM.P: I2C `bus` for
    configuration, SPI `bus2` for data). None for a tile with no SPI bus."""
    for key in ("bus", "bus2"):
        b = str(tile_entry.get(key) or "")
        if re.match(r'^SPI\d+$', b):
            return b
    return None


def is_dual_bus(tile_entry):
    return bool(tile_entry.get("bus2"))


def validate_tile_buses(config):
    """Check tiles[].bus2 and tiles[].cs_id. Returns a list of error strings.

    bus2 names a tile's second bus, of the other protocol from `bus` (I2C and
    SPI), and must be configured by pads like `bus`. cs_id is the chip-select id
    of a dual-bus tile on its SPI bus (see spi_cs_ids); a single-bus SPI tile's
    id is its instance, so a cs_id there must equal the instance.
    """
    errors = []
    key = "pads" if "pads" in config or "pins" not in config else "pins"
    configured = _configured_buses(config.get(key))
    proto = lambda b: re.sub(r'\d+$', '', b)
    for t in config.get("tiles", []) or []:
        name = f"Tile '{_tile_type(t)}' instance {t.get('instance', 0)}"
        bus = str(t.get("bus") or "")
        bus2 = t.get("bus2")
        if bus2 not in (None, ""):
            bus2 = str(bus2)
            if not re.match(r'^(I2C|SPI)\d+$', bus2) or not re.match(r'^(I2C|SPI)\d+$', bus):
                errors.append(f"{name}: bus '{bus}' + bus2 '{bus2}' is not an I2C bus and an SPI bus. "
                              f"A dual-bus tile has one of each.")
            elif proto(bus2) == proto(bus):
                errors.append(f"{name}: bus2 '{bus2}' is the same protocol as bus '{bus}'. "
                              f"bus2 must be the other one (I2C + SPI).")
            elif bus2 not in configured:
                have = ", ".join(sorted(configured)) or "(none)"
                errors.append(f"{name}: bus2 '{bus2}' is not configured (no pads assign it). "
                              f"Configured buses: {have}")
        cs_id = t.get("cs_id")
        if cs_id is None:
            continue
        if not isinstance(cs_id, int) or isinstance(cs_id, bool) or not 0 <= cs_id <= 255:
            errors.append(f"{name}: cs_id {cs_id!r} must be an integer 0-255")
        elif tile_spi_bus(t) is None:
            errors.append(f"{name}: cs_id is set but the tile has no SPI bus")
        elif not is_dual_bus(t) and cs_id != t.get("instance", 0):
            errors.append(f"{name}: cs_id {cs_id} differs from its instance. A single-bus SPI "
                          f"tile's chip-select id is its instance (its driver passes tile->id); "
                          f"drop cs_id or make it equal.")
    return errors


def spi_cs_ids(tiles):
    """Chip-select ids of the tiles on ONE SPI bus, as [(tile_entry, id)].

    The id is the `cs` value the tile's driver passes to the tiles_pal_t SPI
    calls, and the key of the bus's chip-select map:
      - a single-bus SPI tile: its instance (drivers set tile->id = instance);
      - a dual-bus tile (bus2): its cs_id, which the application passes to the
        driver as its SPI chip-select (Sense.CAM.P: cfg.spi_cs). When cs_id is
        omitted it is the smallest id no other tile on the bus holds, taken in
        tiles[] order after every explicit id (so 0 for a tile alone).
    Collisions are left to the caller.
    """
    fixed = []
    for t in tiles:
        if not is_dual_bus(t):
            fixed.append(t.get("instance", 0))
        elif t.get("cs_id") is not None:
            fixed.append(t["cs_id"])
    used = set(fixed)
    out = []
    for t in tiles:
        if not is_dual_bus(t):
            out.append((t, t.get("instance", 0)))
        elif t.get("cs_id") is not None:
            out.append((t, t["cs_id"]))
        else:
            k = 0
            while k in used:
                k += 1
            used.add(k)
            out.append((t, k))
    return out


def merge_tile_cs_pads(config):
    """Claim every SPI tile's tiles[].cs_pad in the project's pads as '<bus>.CS'.

    The bus is the tile's SPI bus: `bus`, or `bus2` for a dual-bus tile whose
    `bus` is I2C. A chip-select pad may appear only in tiles[].cs_pad (Studio
    also mirrors it into pads). Merging it here means the pad is validated like
    any other pad and configured by the pad init as a push-pull output,
    deasserted, before the SPI peripheral is enabled. A cs_pad already assigned
    'GPIO.OUT' becomes the chip select. Mutates config in memory only
    (config.json is not rewritten). Returns a list of error strings.
    """
    errors = []
    key = "pads" if "pads" in config or "pins" not in config else "pins"
    # Only buses some pad already creates: a cs_pad must not invent a bus (a
    # tile on an unconfigured bus fails later with its own clear error).
    configured = _configured_buses(config.get(key))
    for t in config.get("tiles", []) or []:
        bus = tile_spi_bus(t)
        cs_pad = t.get("cs_pad")
        if cs_pad in (None, "") or bus not in configured:
            continue
        pad = str(cs_pad)
        want = f"{bus}.CS"
        pads = config.setdefault(key, {})
        cur = pads.get(pad)
        if cur is None or cur == "GPIO.OUT":
            pads[pad] = want
        elif cur != want:
            errors.append(
                f"Tile '{_tile_type(t)}' instance {t.get('instance', 0)} on {bus}: cs_pad {pad} "
                f"is assigned '{cur}' in pads. A chip-select pad must be free, "
                f"'GPIO.OUT' or '{want}'."
            )
    return errors


def build_pad_config(config, pad_map):
    """Build the resolved pad configuration from project config.

    For each assigned pad, resolves the GPIO port/pin and AF number.
    Also merges per-pad GPIO settings from the 'gpio' section:
      pull:        "none" | "up" | "down"  (default: "none")
      output_type: "push-pull" | "open-drain" (default: "push-pull")
      exti:        "rising" | "falling" | "both" (default: none)
      default:     "high" | "low" (default: none — no explicit set/clear)
    """
    pad_lookup = {p["number"]: p for p in pad_map}
    gpio_section = config.get("gpio", {})
    pad_configs = []

    for pad_num, assigned_func in config.get("pads", config.get("pins", {})).items():
        pad_info = pad_lookup.get(pad_num)
        if pad_info is None:
            continue

        entry = {
            "pad": pad_num,
            "function": assigned_func,
            "port": pad_info["port"],
            "pin": pad_info["pin"],
            "af": None,
            "mode": "af",  # alternate function
            # GPIO-specific settings (populated from gpio section below)
            "pull": "none",
            "output_type": "push-pull",
            "speed": "medium",
            "exti": None,
            "default": None,
        }

        if assigned_func == "GPIO.OUT":
            entry["mode"] = "output"
            entry["af"] = None
        elif assigned_func == "GPIO.IN":
            entry["mode"] = "input"
            entry["af"] = None
        elif re.match(r'^SPI\d+\.CS$', assigned_func):
            # SPI CS pins are managed as GPIO output via hal_spi_set_cs(),
            # not as hardware NSS alternate function. Start deasserted: an
            # output pin comes up low, which would select the device from
            # boot until the SPI init ran.
            entry["mode"] = "output"
            entry["af"] = None
            bus_cfg = config.get("interfaces", {}).get(assigned_func.split(".")[0], {})
            entry["default"] = "low" if bus_cfg.get("cs_polarity") == "active-high" else "high"
        elif re.match(r'^ADC_?(?:IN)?\d+', assigned_func):
            # ADC input pads: set to analog mode (MODER=11).
            # No AF needed — analog functions bypass the AF mux entirely.
            entry["mode"] = "analog"
            entry["af"] = None
        elif re.match(r'^DAC\d+\.OUT', assigned_func):
            # DAC output: set to analog mode.
            entry["mode"] = "analog"
            entry["af"] = None
            entry["dac"] = True
        else:
            # Find AF for this function
            for af_func in pad_info["af_functions"]:
                if af_func["function"] == assigned_func:
                    entry["af"] = af_func["af"]
                    break

        # Merge per-pad GPIO settings from the 'gpio' section
        gpio_cfg = gpio_section.get(str(pad_num), gpio_section.get(pad_num, {}))
        if gpio_cfg:
            entry["pull"] = gpio_cfg.get("pull", "none")
            entry["output_type"] = gpio_cfg.get("output_type", "push-pull")
            entry["speed"] = gpio_cfg.get("speed", "medium")
            entry["exti"] = gpio_cfg.get("exti", None)
            # Keep an SPI chip select's deasserted default unless overridden.
            entry["default"] = gpio_cfg.get("default", entry["default"])

        pad_configs.append(entry)

    return pad_configs


def build_timer_config(config, pad_map):
    """Extract timer pad assignments from project config.

    Scans the assigned pads for TIMx.y patterns and returns a list of
    dicts describing each timer PWM output:
      [{"pad": "7", "timer": "TIM2", "channel": 3, "af": 1}, ...]

    This info is used to generate PAD_n_TIM / PAD_n_TIM_CH defines
    and the core_pad_timer_info() lookup in core_pads.h.
    """
    pad_lookup = {p["number"]: p for p in pad_map}
    timer_pads = []

    for pad_num, assigned_func in config.get("pads", config.get("pins", {})).items():
        m = re.match(r'(TIM\d+)\.(\d+)$', assigned_func)
        if not m:
            continue
        pad_info = pad_lookup.get(pad_num)
        if not pad_info:
            continue
        timer_name = m.group(1)
        channel = int(m.group(2))
        # Find AF number for this specific function
        af = None
        for af_func in pad_info["af_functions"]:
            if af_func["function"] == assigned_func:
                af = af_func["af"]
                break
        timer_pads.append({
            "pad": pad_num,
            "timer": timer_name,
            "channel": channel,
            "af": af,
        })

    return timer_pads


# ---- GPIO-switched on-tile pull-ups ----

def _pin_name(pin):
    m = re.match(r'^P([A-H])(\d+)$', str(pin))
    return (m.group(1), int(m.group(2))) if m else (None, None)


def tile_pullup_options(tile):
    """The tile's `config.pullups` options as dicts:
    {value, label, pad (str or None), pins: [(port, pin, drive)]}."""
    knob = (tile.get("config") or {}).get("pullups") or {}
    out = []
    for opt in knob.get("options", []) or []:
        value = str(opt.get("value", ""))
        m = re.match(r'^pad(\d+)$', value) or re.search(r'[Pp]ad (\d+)', str(opt.get("label", "")))
        pins = []
        for fc in opt.get("firmware_contract") or []:
            if fc.get("type") != "gpio":
                continue
            port, pin = _pin_name(fc.get("pin"))
            if port is None:
                continue
            pins.append((port, pin, fc.get("drive", "high")))
        out.append({"value": value, "label": opt.get("label", value),
                    "pad": m.group(1) if m else None, "pins": pins})
    return out


def validate_pullups(config, tile, pad_map):
    """Errors for the project's "pullups" list."""
    errors = []
    want = config.get("pullups")
    if want is None:
        return errors
    if not isinstance(want, list):
        return [f"pullups: expected a list of option names, got {want!r}"]
    opts = {o["value"]: o for o in tile_pullup_options(tile)}
    if not opts and want:
        return [f"pullups: this Core has no switchable pull-ups (tile config has no 'pullups' option)"]
    pad_gpio = {p["number"]: (p["port"], p["pin"]) for p in pad_map if p["port"]}
    assigned = config.get("pads", config.get("pins", {}))
    for v in want:
        o = opts.get(str(v))
        if o is None:
            errors.append(f"pullups: '{v}' is not an option here. Options: {', '.join(sorted(opts))}")
            continue
        for port, pin, _ in o["pins"]:
            for pad_num in assigned:
                if pad_gpio.get(pad_num) == (port, pin):
                    errors.append(f"pullups: '{v}' drives P{port}{pin}, which pad {pad_num} "
                                  f"is assigned to ({assigned[pad_num]})")
    return errors


def build_pullup_config(config, tile, i2c_buses):
    """GPIO writes for the enabled pull-ups, plus a warning for fast I2C on a
    pad whose on-tile pull-up is available but off."""
    opts = tile_pullup_options(tile)
    want = set(str(v) for v in (config.get("pullups") or []))
    pins = []
    for o in opts:
        if o["value"] in want:
            for port, pin, drive in o["pins"]:
                pins.append({"label": o["label"], "value": o["value"], "port": port,
                             "pin": pin, "high": drive != "low"})
    assigned = config.get("pads", config.get("pins", {}))
    speed = {b["instance"]: b.get("speed", 400000) for b in i2c_buses}
    for o in opts:
        if o["value"] in want or not o["pad"]:
            continue
        fn = str(assigned.get(o["pad"], ""))
        m = re.match(r'^(I2C\d+)\.(CLK|DAT)$', fn)
        if m and speed.get(m.group(1), 0) >= 400000:
            eprint(f"  WARNING: pad {o['pad']} is {fn} at {speed[m.group(1)] // 1000} kHz with only "
                   f"the MCU's internal pull-up (~40 kOhm), too weak for that speed on most buses. "
                   f"This Core has an on-tile pull-up for it: add \"{o['value']}\" to "
                   f"\"pullups\" in config.json, or fit external pull-ups.")
    return pins


# ---- Clock tree and peripheral clocks: the back end's ----
#
# These keep their pre-split names and signatures; each hands the work to the
# active back end (tools/coregen/ports/<port>.py).

def build_clock_config(config, tile, mcu):
    """Resolved clock configuration for config.json's "clock" level
    (back end: clock_config)."""
    return backend().clock_config(config, tile, mcu)


def build_i2c_config(config, mcu, clock_config):
    """One dict per I2C bus some pad uses (back end: i2c_config)."""
    return backend().i2c_config(config, mcu, clock_config)


def build_usart_config(config, mcu):
    """One dict per USART some pad uses (back end: usart_config)."""
    return backend().usart_config(config, mcu)


def build_pwm_config(config, mcu):
    """One dict per timer some pad uses for PWM (back end: pwm_config)."""
    return backend().pwm_config(config, mcu)


# ---- ADC config ----

# Pattern matches ADC function names in tile JSON / config.json:
#   "ADC"        (bare peripheral name, channel inferred from pad)
#   "ADC7"       (channel number, Core.ST.L0/L4-style)
#   "ADC7+"      (single-ended positive input, Core.ST.H5-style)
#   "ADC_IN3"    (legacy alias)
#   "ADCIN3"     (another legacy alias)
# Intentionally rejects the negative-input variant ("ADC3-") since single-ended
# differential mode isn't DSL-safe today.
_ADC_FUNC_RE = re.compile(r'^ADC(?:_?IN)?\d*\+?$')


def build_adc_config(config):
    """Detect ADC use from pad assignments and build the ADC config dict.

    Scans `config.pads` for analog functions whose name starts with "ADC"
    (single-ended positive inputs only — differential "-" variants are
    excluded until the HAL exposes them). Returns a dict shaped for the
    `core_init.{h,c}.j2` templates, or None when no ADC pad is configured.

    Today emits exactly one handle (`core_adc1`) regardless of which ADC
    peripheral a given pad is actually wired to. Correct for Core.ST.L0 / Core.ST.L4 /
    Core.ST.W5 (single ADC). For Core.ST.H5 the two ADC peripherals share channel-
    number namespaces and the tile JSON doesn't yet tag which peripheral
    each pad belongs to; when a multi-ADC project lands we'll extend this
    to emit `core_adc1` + `core_adc2` and dispatch per-pad. Until then this
    comment is the migration flag.
    """
    pads = config.get("pads", config.get("pins", {}))
    adc_pads = []
    for pad_num, func in sorted(pads.items(), key=lambda kv: int(kv[0])):
        if _ADC_FUNC_RE.match(func):
            adc_pads.append({"pad": pad_num, "function": func})
    if not adc_pads:
        return None
    return {
        "handle": "core_adc1",
        "instance_var": "core_adc1",  # same as handle for single-ADC mode
        "resolution": "HAL_ADC_RES_12BIT",
        "sampling": "HAL_ADC_SAMP_MED",
        "pads": adc_pads,
    }


def resolve_spi_chip_selects(config, bus_name, bus_cs_pads, pad_lookup):
    """Decide how one SPI bus drives its chip selects.

    bus_cs_pads: every pad assigned '<bus>.CS' (tile cs_pads merged in).
    Returns (bus_cs_pad, cs_map):
      - one CS pad (or none): bus_cs_pad is it (or None), cs_map is []. The
        bus's own CS, as before: core_spi_select() and every tile bridge call
        on this bus drive it, whatever `cs` the driver passes. At most one
        tile may sit on the bus.
      - several CS pads: bus_cs_pad is None and cs_map lists one entry per
        tile, {id, pad, port, pin, tile}. id is the tile's chip-select id (see
        spi_cs_ids): the `cs` value its driver passes to the tiles_pal_t SPI
        calls, which is its instance for a single-bus SPI tile and its cs_id
        for a dual-bus tile. Every tile on the bus must name its cs_pad, and
        the ids and cs_pads must be unique on the bus.
    The tiles on a bus include dual-bus tiles whose bus2 is this bus.
    Exits with an ERROR on a conflict.
    """
    tiles = [t for t in (config.get("tiles", []) or []) if tile_spi_bus(t) == bus_name]
    fail = []

    # Two tiles on one chip select would both answer every transfer.
    seen_pad = {}
    for t in tiles:
        cs = t.get("cs_pad")
        if cs in (None, ""):
            continue
        cs = str(cs)
        if cs in seen_pad:
            o = seen_pad[cs]
            fail.append(f"Tiles '{_tile_type(o)}' (instance {o.get('instance', 0)}) and "
                        f"'{_tile_type(t)}' (instance {t.get('instance', 0)}) on {bus_name} both use "
                        f"cs_pad {cs}. Each tile on a bus needs its own chip-select pad.")
        else:
            seen_pad[cs] = t

    pads = sorted(set(bus_cs_pads), key=lambda p: int(p) if p.isdigit() else 0)
    if len(pads) <= 1:
        if len(tiles) > 1 and not fail:
            where = f"only one chip-select pad ({pads[0]})" if pads else "no chip-select pad"
            fail.append(f"{bus_name} carries {len(tiles)} tiles but {where}. Give each tile "
                        f"its own cs_pad (tiles[].cs_pad) so each is selected on its own.")
        for m in fail:
            eprint(f"  ERROR: {m}")
        if fail:
            sys.exit(1)
        return (pads[0] if pads else None), []

    cs_map = []
    seen_id = {}
    for t, cid in spi_cs_ids(tiles):
        inst = t.get("instance", 0)
        what = f"Tile '{_tile_type(t)}' instance {inst}" + (f" (cs_id {cid})" if is_dual_bus(t) else "")
        cs = t.get("cs_pad")
        if cs in (None, ""):
            fail.append(f"{what} on {bus_name} has no cs_pad, and {bus_name} has several "
                        f"chip-select pads ({', '.join(pads)}). Name the tile's cs_pad.")
            continue
        if not isinstance(cid, int) or isinstance(cid, bool) or not 0 <= cid <= 255:
            fail.append(f"Tile '{_tile_type(t)}' on {bus_name}: instance {inst!r} must be an "
                        f"integer 0-255 (it is the tile's chip-select id)")
            continue
        if cid in seen_id:
            o = seen_id[cid]
            fail.append(f"Tiles '{_tile_type(o)}' and '{_tile_type(t)}' on {bus_name} both have "
                        f"chip-select id {cid}. The id is the `cs` a driver passes: a single-bus "
                        f"SPI tile's instance, or a dual-bus tile's cs_id. Give each tile on "
                        f"{bus_name} a different one.")
            continue
        seen_id[cid] = t
        info = pad_lookup.get(str(cs), {})
        if info.get("port") is None or info.get("pin") is None:
            fail.append(f"{what}: CS pad {cs} could not be resolved to a GPIO port/pin")
            continue
        cs_map.append({"id": cid, "pad": str(cs), "port": info["port"],
                       "pin": info["pin"], "tile": _tile_type(t),
                       "key": (f"cs_id {cid} (dual-bus)" if is_dual_bus(t) else f"instance {cid}")})
    for m in fail:
        eprint(f"  ERROR: {m}")
    if fail:
        sys.exit(1)
    claimed = {e["pad"] for e in cs_map}
    spare = [p for p in pads if p not in claimed]
    if spare:
        eprint(f"  NOTE: {bus_name} chip-select pad(s) {', '.join(spare)} belong to no tile: "
               f"they are held deasserted, and core_spi_select() drives none of them.")
    return None, cs_map


def build_spi_config(config, mcu, pad_map, clock_config=None):
    """Detect SPI buses from pin assignments and build SPI config list.

    Scans pads for patterns like 'SPI1.CLK', 'SPI1.MOSI', 'SPI1.MISO', 'SPI1.CS'
    and returns a list of dicts with bus configuration for template rendering.

    Per-bus settings (mode, clock, bit order) come from the 'interfaces'
    section of config.json and are the back end's (spi_config); its STM32
    defaults are mode=0 (CPOL=0/CPHA=0), prescaler=8 (÷8), bit_order="msb",
    and with clock_config a prescaler that puts SCK over the part's
    datasheet ceiling is reported.

    SPI1.CS pads are configured as GPIO output (software CS management via
    hal_spi_set_cs) rather than the hardware NSS alternate function. A bus
    with several CS pads gets a per-tile chip-select map instead (see
    resolve_spi_chip_selects).
    """
    iface_cfg = config.get("interfaces", {})
    pads = config.get("pads", config.get("pins", {}))
    pad_lookup = {p["number"]: p for p in pad_map}

    # Detect SPI buses and CS pad assignments from pad assignments (tile
    # cs_pads are already merged in, see merge_tile_cs_pads)
    bus_numbers = set()
    cs_pads = {}   # bus_num -> [pad_num string, ...]
    for pad_num, func in pads.items():
        m = re.match(r'^SPI(\d+)\.(CLK|MOSI|MISO|CS)$', func)
        if m:
            bus_num = int(m.group(1))
            bus_numbers.add(bus_num)
            if m.group(2) == "CS":
                cs_pads.setdefault(bus_num, []).append(str(pad_num))

    if not bus_numbers:
        return []

    spi_buses = []
    for bus_num in sorted(bus_numbers):
        bus_name = f"SPI{bus_num}"
        bus_cfg = iface_cfg.get(bus_name, {})
        port_cfg = backend().spi_config(bus_num, bus_cfg, mcu, clock_config)

        # Chip selects: one pad is the bus's own (hal_spi_set_cs); several
        # are a per-tile map (hal_spi_set_cs_map), keyed by tile instance.
        cs_pad_num, cs_map = resolve_spi_chip_selects(
            config, bus_name, cs_pads.get(bus_num, []), pad_lookup)
        cs_port = None
        cs_pin = None
        if cs_pad_num:
            pad_info = pad_lookup.get(cs_pad_num, {})
            cs_port = pad_info.get("port")
            cs_pin = pad_info.get("pin")

        cs_polarity = bus_cfg.get("cs_polarity", "active-low")
        cs_active_low = (cs_polarity != "active-high")

        spi_buses.append({
            "num": bus_num,
            "instance": bus_name,
            "handle": f"core_spi{bus_num}",
            **port_cfg,
            "cs_pad": cs_pad_num,
            "cs_port": cs_port,
            "cs_pin": cs_pin,
            "cs_map": cs_map,
            "cs_active_low": cs_active_low,
        })

    return spi_buses


# ---- I3C on an I2C bus (Core.ST.H5) ----
#
# A project asks for I3C on an I2C bus: "interfaces": {"I2C1": {"i3c": true}}.
# coregen honours it only when (1) the Core offers I3C on both of the bus's
# pads (Core.ST.H5: pad 4 I3C1.CLK, pad 5 I3C1.DAT, AF3), (2) the MCU has an
# I3C HAL (STM32H523), and (3) every tile on the bus speaks I3C on the pads it
# uses for I2C, per its definition's pad functions (interface "I3C"). Anything
# else keeps the whole bus on I2C, exactly as without the key, with a
# one-line NOTE. Mixed I2C/I3C buses are not supported. Whether the MCU has
# I3C is the back end's (i3c_unsupported).

I3C_DEFAULT_SPEED = 12500000
I3C_MIN_SPEED = 100000
I3C_MAX_SPEED = 12500000


def definition_dirs(tile_path=None):
    """Where tile definitions are looked up, in order: this SDK's public
    definitions/, the directory of the Core's own definition, then
    definitions/ in each private overlay checkout named in TILES_OVERLAY
    (space-separated; the Makefile exports it). Only public tiles are in this
    repo; the others live in the private repos (tiles-internal, tiles-alpha,
    tile-*)."""
    dirs = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "definitions")]
    if tile_path:
        dirs.append(os.path.dirname(os.path.abspath(tile_path)))
    for o in os.environ.get("TILES_OVERLAY", "").split():
        dirs.append(os.path.join(os.path.expanduser(o), "definitions"))
    out = []
    for d in dirs:
        d = os.path.normpath(d)
        if d not in out:
            out.append(d)
    return out


def _tile_definition(defs_dirs, tile_type):
    """The newest-revision <Family>-<Name>-<rev>.json for a tile name such as
    "Sense.I.6P6", across `defs_dirs` (a directory or a list; the first one
    holding a given file wins), or None."""
    if isinstance(defs_dirs, str):
        defs_dirs = [defs_dirs]
    stem = tile_type.replace(".", "-")
    found = {}
    for d in defs_dirs:
        try:
            names = os.listdir(d)
        except OSError:
            continue
        for n in names:
            if re.match(re.escape(stem) + r'-[a-z]+\.json$', n):
                found.setdefault(n, os.path.join(d, n))
    if not found:
        return None
    with open(found[sorted(found)[-1]], encoding="utf-8") as f:
        return json.load(f)


def _tile_speaks_i3c(defn):
    """True when every pad that carries I2C.CLK / I2C.DAT on the tile also
    offers I3C.CLK / I3C.DAT (interface "I3C")."""
    found = {"CLK": False, "DAT": False}
    for pad in defn.get("pads", []):
        fns = pad.get("functions", [])
        for sig in ("CLK", "DAT"):
            if any(f.get("function") == f"I2C.{sig}" for f in fns):
                if not any(f.get("interface") == "I3C" and f.get("function") == f"I3C.{sig}" for f in fns):
                    return False
                found[sig] = True
    return found["CLK"] and found["DAT"]


def resolve_i3c_buses(config, pad_map, mcu, defs_dirs):
    """Decide which requested I2C buses become I3C buses (see above).

    Returns a list of dicts, one per bus switched to I3C:
      {"i2c_name": "I2C1", "name": "I3C1", "num": 1, "handle": "core_i3c1",
       "speed": Hz, "pads": {pad: I3C function}, "attach": [(addr, tile, instance)]}
    NOTEs for requests that stay I2C go to stderr.
    """
    key = "pads" if "pads" in config or "pins" not in config else "pins"
    pads = config.get(key, {}) or {}
    pad_lookup = {p["number"]: p for p in pad_map}
    out = []
    for i2c_name, icfg in sorted((config.get("interfaces", {}) or {}).items()):
        if not isinstance(icfg, dict) or icfg.get("i3c") is not True:
            continue
        bus_pads = {sig: next((n for n, f in pads.items() if f == f"{i2c_name}.{sig}"), None)
                    for sig in ("CLK", "DAT")}

        def stay(reason):
            eprint(f"  NOTE: {i2c_name} asked for I3C (\"i3c\": true) but stays I2C: {reason}")

        if not all(bus_pads.values()):
            continue       # no pads for this bus: the usual "configured but no pins" warning covers it
        no_i3c = backend().i3c_unsupported(mcu)
        if no_i3c:
            stay(no_i3c)
            continue
        i3c_fns = {}
        for sig, pad in bus_pads.items():
            info = pad_lookup.get(pad, {})
            fn = next((f for f in info.get("all_functions", []) if re.match(rf'^I3C\d+\.{sig}$', f)), None)
            i3c_fns[sig] = fn
        if not all(i3c_fns.values()) or i3c_fns["CLK"].split(".")[0] != i3c_fns["DAT"].split(".")[0]:
            stay(f"pads {bus_pads['CLK']}/{bus_pads['DAT']} have no I3C function on this Core")
            continue
        blockers = []
        attach = []
        for t in config.get("tiles", []) or []:
            if i2c_name not in (t.get("bus"), t.get("bus2")):
                continue
            ttype = _tile_type(t)
            if is_dual_bus(t):
                blockers.append(f"{ttype} uses two buses (core_tiles_pal2 needs I2C)")
                continue
            defn = _tile_definition(defs_dirs, ttype)
            if defn is None:
                blockers.append(f"{ttype} has no definition to check")
                continue
            if not _tile_speaks_i3c(defn):
                blockers.append(f"{ttype} has no I3C on its bus pads")
                continue
            addrs = (tile_driver_map().get(ttype) or {}).get("i2c_addrs") or []
            inst = t.get("instance", 0)
            if isinstance(inst, int) and 0 <= inst < len(addrs):
                attach.append((addrs[inst], ttype, inst))
        if blockers:
            stay("; ".join(blockers))
            continue
        name = i3c_fns["CLK"].split(".")[0]
        speed = icfg.get("i3c_speed", I3C_DEFAULT_SPEED)
        out.append({
            "i2c_name": i2c_name,
            "name": name,
            "num": int(name[3:]),
            "handle": f"core_{name.lower()}",
            "speed": speed,
            "pads": {bus_pads["CLK"]: i3c_fns["CLK"], bus_pads["DAT"]: i3c_fns["DAT"]},
            "attach": attach,
        })
        print(f"  {i2c_name} -> {name} (I3C SDR, {speed} Hz): every tile on the bus speaks I3C")
    return out


def apply_i3c_buses(config, i3c_buses):
    """Swap the switched buses' pad functions from I2Cn.* to I3Cm.* (in memory)
    so the pad init and bus builders see an I3C bus and no I2C one."""
    key = "pads" if "pads" in config or "pins" not in config else "pins"
    for bus in i3c_buses:
        for pad, fn in bus["pads"].items():
            config[key][pad] = fn


# ---- Tile peripheral driver mapping ----
#
# Public tiles' drivers, in drivers/ here. A tile that is not public has its
# driver in its private overlay repo (tiles-internal, tiles-alpha,
# tile-<family>-<name>), which declares it in drivers/drivers.json with the
# same fields; see overlay_driver_map().

TILE_DRIVER_MAP = {
    "Sense.CAM.P": {"header": "tile_sense_cam_p.h",  "source": "tile_sense_cam_p",  "prefix": "tile_sense_cam_p"},
    "Sense.I.9":   {"header": "tile_sense_i_9.h",    "source": "tile_sense_i_9",    "prefix": "tile_sense_i_9", "extra_sources": ["tile_sense_i_9_dmp3"]},
    "Sense.I.6P8": {"header": "tile_sense_i_6p8.h",  "source": "tile_sense_i_6p8",  "prefix": "tile_sense_i_6p8"},
    # i2c_addrs: the driver's static address per instance (drivers/tile_sense_i_6p6.c
    # id_table), which an I3C bus uses to assign the tile's dynamic address (SETDASA).
    "Sense.I.6P6": {"header": "tile_sense_i_6p6.h",  "source": "tile_sense_i_6p6",  "prefix": "tile_sense_i_6p6",
                    "i2c_addrs": [0x69, 0x68]},
    "Sense.I.6D":  {"header": "tile_sense_i_6d.h",   "source": "tile_sense_i_6d",   "prefix": "tile_sense_i_6d"},
    "Drive.P":     {"header": "tile_drive_p.h",      "source": "tile_drive_p",      "prefix": "tile_drive_p"},
    "Drive.H":     {"header": "tile_drive_h.h",      "source": "tile_drive_h",      "prefix": "tile_drive_h"},
    "Drive.A.2":   {"header": "tile_drive_a_2.h",    "source": "tile_drive_a_2",    "prefix": "tile_drive_a_2"},
    "Drive.DC.H":  {"header": "tile_drive_dc_h.h",   "source": "tile_drive_dc_h",   "prefix": "tile_drive_dc_h"},
    "Power.L.1N":  {"header": "tile_power_l_1n.h",   "source": "tile_power_l_1n",   "prefix": "tile_power_l_1n"},
    "Power.L.1T":  {"header": "tile_power_l_1t.h",   "source": "tile_power_l_1t",   "prefix": "tile_power_l_1t"},
    "Power.P.N":   {"header": "tile_power_p_n.h",    "source": "tile_power_p_n",    "prefix": "tile_power_p_n"},
    "Display.RGBW": {"header": "tile_display_rgbw.h",   "source": "tile_display_rgbw",   "prefix": "tile_display_rgbw"},
    "Store.O.128": {"header": "tile_store_o_128.h",   "source": "tile_store_o_128",   "prefix": "tile_store_o_128"},
    "Sense.T.C":   {"header": "tile_sense_t_c.h",  "source": "tile_sense_t_c",  "prefix": "tile_sense_t_c"},
    "Sense.MIC":   {"header": "tile_sense_mic.h",  "source": "tile_sense_mic",  "prefix": "tile_sense_mic"},
    "Sense.TOF":   {"header": "tile_sense_tof.h", "source": "tile_sense_tof", "prefix": "tile_sense_tof"},
    "Sense.CAP":   {"header": "tile_sense_cap.h", "source": "tile_sense_cap", "prefix": "tile_sense_cap"},
    "Sense.M.3G":  {"header": "tile_sense_m_3g.h", "source": "tile_sense_m_3g", "prefix": "tile_sense_m_3g"},
    "Sense.HR":    {"header": "tile_sense_hr.h",   "source": "tile_sense_hr",   "prefix": "tile_sense_hr"},
}


# Where an overlay checkout declares its drivers, relative to its root.
OVERLAY_DRIVERS_FILE = os.path.join("drivers", "drivers.json")


def overlay_driver_map():
    """Drivers declared by the private overlay checkouts named in TILES_OVERLAY
    (space-separated; the Makefile exports it), as TILE_DRIVER_MAP entries.

    Each overlay's drivers/drivers.json is
      {"schema": "tiles-overlay-drivers/v1",
       "drivers": {"<Tile>": {"header", "source", "prefix", [extra_sources],
                              [i2c_addrs], "init", "version", "definition"}}}
    with the TILE_DRIVER_MAP fields plus what the manifest generators read
    (init, version, definition). Each entry gets "dir": the overlay's drivers/
    directory, where its header and sources are. Later overlays win."""
    out = {}
    for o in os.environ.get("TILES_OVERLAY", "").split():
        root = os.path.expanduser(o)
        path = os.path.join(root, OVERLAY_DRIVERS_FILE)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as f:
                drivers = json.load(f).get("drivers") or {}
        except (OSError, ValueError, AttributeError) as e:
            eprint(f"  ERROR: {path}: not a drivers file ({e})")
            sys.exit(1)
        for tile, entry in drivers.items():
            missing = [k for k in ("header", "source", "prefix") if not entry.get(k)]
            if missing:
                eprint(f"  ERROR: {path}: {tile} has no {', '.join(missing)}")
                sys.exit(1)
            out[tile] = {**entry, "dir": os.path.normpath(os.path.join(root, "drivers"))}
    return out


def tile_driver_map():
    """TILE_DRIVER_MAP with the TILES_OVERLAY drivers merged over it."""
    return {**TILE_DRIVER_MAP, **overlay_driver_map()}


def _pal2_expr(tile_entry):
    """core_tiles_pal2(&core_i2cN, &core_spiM) for a dual-bus tile, else None."""
    if not is_dual_bus(tile_entry):
        return None
    buses = [str(tile_entry.get("bus")), str(tile_entry.get("bus2"))]
    i2c = next((b for b in buses if b.startswith("I2C")), None)
    spi = next((b for b in buses if b.startswith("SPI")), None)
    if not i2c or not spi:
        return None
    return f"core_tiles_pal2(&core_{i2c.lower()}, &core_{spi.lower()})"


def build_tiles_config(config, i2c_buses, spi_buses=None, pad_map=None, i3c_buses=None):
    """Build tile peripheral config from 'tiles' list in project config.

    For each declared tile, looks up driver info, validates the bus assignment,
    and generates handle names. Returns (tiles_config, tile_pal_buses, tile_driver_sources).

    tiles_config: list of dicts with per-tile info for template rendering
    tile_pal_buses: list of dicts for unique buses needing tiles_pal_t handles
    tile_driver_sources: list of unique driver source names (for Makefile)

    SPI chip selects (tiles[].cs_pad) are resolved per bus by
    resolve_spi_chip_selects(), from build_spi_config().
    """
    tiles_list = config.get("tiles", [])
    if not tiles_list:
        return [], [], []

    # Build lookup of configured buses by name
    i2c_lookup = {bus["instance"]: bus for bus in i2c_buses}
    spi_lookup = {bus["instance"]: bus for bus in (spi_buses or [])}
    # A tile keeps naming its I2C bus ("bus": "I2C1") when coregen runs that
    # bus as I3C; it then gets the I3C handle and bridge.
    i3c_lookup = {bus["i2c_name"]: bus for bus in (i3c_buses or [])}
    all_bus_names = set(i2c_lookup) | set(spi_lookup) | set(i3c_lookup)

    driver_map = tile_driver_map()
    tiles_config = []
    # Chip-select id per SPI tile (id(entry) -> id), for the init recipe.
    cs_ids = {}
    for bus in {tile_spi_bus(t) for t in tiles_list} - {None}:
        for t, cid in spi_cs_ids([t for t in tiles_list if tile_spi_bus(t) == bus]):
            cs_ids[id(t)] = cid
    seen_buses = {}          # bus_name -> hal handle dict
    seen_drivers = set()

    for tile_entry in tiles_list:
        tile_type = tile_entry.get("tile", tile_entry.get("type", ""))
        bus_name = tile_entry["bus"]
        instance = tile_entry.get("instance", 0)

        # Look up driver info
        driver = driver_map.get(tile_type)
        if driver is None:
            overlays = " ".join(os.environ.get("TILES_OVERLAY", "").split())
            eprint(f"  ERROR: Unknown tile '{tile_type}'. "
                  f"Known tiles: {', '.join(sorted(driver_map.keys()))}. "
                  f"Only public tiles' drivers are in this SDK; a tile that is not "
                  f"public has its driver in its private repo: build with "
                  f"TILES_OVERLAY=/path/to/<that repo's checkout>"
                  + (f" (searched: {overlays})" if overlays else ""))
            sys.exit(1)

        # Validate bus exists in project config
        if bus_name not in all_bus_names:
            configured = ", ".join(sorted(all_bus_names)) if all_bus_names else "(none)"
            eprint(f"  ERROR: Tile '{tile_type}' references bus '{bus_name}' "
                  f"which is not configured. Configured buses: {configured}")
            sys.exit(1)

        # Generate handle name: prefix_bus_instance (e.g., tile_sense_i_9_i2c1_0)
        handle = f"{driver['prefix']}_{bus_name.lower()}_{instance}"

        is_spi = bus_name in spi_lookup

        # Track unique buses for HAL handle generation
        if bus_name not in seen_buses:
            pal_handle = f"core_pal_{bus_name.lower()}"
            if bus_name in i3c_lookup:
                i3c_bus = i3c_lookup[bus_name]
                seen_buses[bus_name] = {
                    "bus_name": i3c_bus["name"],
                    "pal_handle": f"core_pal_{i3c_bus['name'].lower()}",
                    "i3c_handle": i3c_bus["handle"],
                    "bus_type": "i3c",
                }
            elif is_spi:
                spi_bus = spi_lookup[bus_name]
                seen_buses[bus_name] = {
                    "bus_name": bus_name,
                    "pal_handle": pal_handle,
                    "spi_handle": spi_bus["handle"],
                    "bus_type": "spi",
                }
            else:
                i2c_bus = i2c_lookup[bus_name]
                seen_buses[bus_name] = {
                    "bus_name": bus_name,
                    "pal_handle": pal_handle,
                    "i2c_handle": i2c_bus["handle"],
                    "bus_type": "i2c",
                }

        seen_drivers.add(driver["source"])
        # Some tiles ship multiple .c files (e.g., Sense.I.9 carries
        # the ICM-20948 DMP3 firmware blob in a separate translation
        # unit so it can be lazy-included by build flags later). Pick
        # them up from the optional `extra_sources` field.
        for extra in driver.get("extra_sources", []):
            seen_drivers.add(extra)

        tiles_config.append({
            "type": tile_type,
            "bus_name": bus_name,
            "instance": instance,
            "handle": handle,
            "header": driver["header"],
            "source": driver["source"],
            "prefix": driver["prefix"],
            # An overlay driver's drivers/ directory (None for a public one):
            # core_drivers.mk names its sources by path.
            "driver_dir": driver.get("dir"),
            "pal_handle": seen_buses[bus_name]["pal_handle"],
            # For the iterable tile table: the bus HANDLE (so the consumer can
            # core_tiles_pal(bus) at runtime and group by pointer), plus optional
            # per-tile aux the table carries verbatim (e.g. Drive.P readdress strap).
            "bus_handle": (seen_buses[bus_name].get("i2c_handle")
                           or seen_buses[bus_name].get("spi_handle")
                           or seen_buses[bus_name].get("i3c_handle")),
            "bus_is_spi": 1 if seen_buses[bus_name]["bus_type"] == "spi" else 0,
            "bus_is_i3c": 1 if seen_buses[bus_name]["bus_type"] == "i3c" else 0,
            "readdress_gpio": tile_entry.get("readdress_gpio", -1),
            # Dual-bus tile (bus + bus2): the PAL carries both buses, and the
            # driver's SPI chip select is its cs_id (e.g. Sense.CAM.P cfg.spi_cs).
            "pal2": _pal2_expr(tile_entry),
            "cs_id": cs_ids.get(id(tile_entry)),
        })

    tile_pal_buses = list(seen_buses.values())
    tile_driver_sources = sorted(seen_drivers)

    return tiles_config, tile_pal_buses, tile_driver_sources


# ---- Smart tiles.h generation ----

COREGEN_BEGIN = "/* ---- coregen:begin ---- */"
COREGEN_END   = "/* ---- coregen:end ---- */"


def _extract_managed_block(text):
    """Extract the content between coregen markers, or None if not found."""
    begin = text.find(COREGEN_BEGIN)
    end = text.find(COREGEN_END)
    if begin < 0 or end < 0 or end <= begin:
        return None
    return text[begin:end + len(COREGEN_END)]


def generate_tiles_h(env, ctx, project_dir):
    """Generate or update tile_handles.h in the project directory.

    Named tile_handles.h (not tiles.h) to avoid shadowing tiles.h in
    the compiler include path, which would prevent driver headers from
    finding the framework tile_t / TILES_CHECK_VERSION definitions.

    - If tile_handles.h doesn't exist: write it fresh.
    - If it exists and the managed block matches: update the managed block.
    - If it exists but the managed block has been removed: skip (user-managed).
    """
    tiles_path = os.path.join(project_dir, "tile_handles.h")
    template = env.get_template("tiles.h.j2")
    fresh = template.render(**ctx)

    new_block = _extract_managed_block(fresh)
    if new_block is None:
        # Template didn't produce markers — shouldn't happen
        print(f"  WARNING: tile_handles.h template missing coregen markers, skipping")
        return

    if not os.path.exists(tiles_path):
        # First generation — write the whole file
        with open(tiles_path, "w", encoding="utf-8") as f:
            f.write(fresh)
        print(f"  tile_handles.h (new)")
        return

    # File exists — check the managed block
    with open(tiles_path, encoding="utf-8") as f:
        existing = f.read()

    old_block = _extract_managed_block(existing)

    if old_block is None:
        # Markers were removed — user fully owns the file now
        print(f"  tile_handles.h (skipped — coregen markers removed, file is user-managed)")
        return

    if old_block == new_block:
        # Already up to date
        print(f"  tile_handles.h (up to date)")
        return

    # Block differs — update the managed section, preserving anything
    # the user added outside the markers.
    updated = existing.replace(old_block, new_block)
    with open(tiles_path, "w", encoding="utf-8") as f:
        f.write(updated)
    print(f"  tile_handles.h (updated)")


# ---- Generation ----


# ---- BLE contract ------------------------------------------------------

BLE_ACCESS_FLAGS = {"read": "CORE_BLE_READ", "write": "CORE_BLE_WRITE",
                    "notify": "CORE_BLE_NOTIFY"}
BLE_SCALARS = {"bool": ("CORE_BLE_BOOL", "uint8_t"), "uint8": ("CORE_BLE_UINT8", "uint8_t"),
               "int8": ("CORE_BLE_INT8", "int8_t"), "uint16": ("CORE_BLE_UINT16", "uint16_t"),
               "int16": ("CORE_BLE_INT16", "int16_t"), "uint32": ("CORE_BLE_UINT32", "uint32_t"),
               "int32": ("CORE_BLE_INT32", "int32_t")}


def _ble_ident(name):
    """'Power Status' -> 'power_status'. Used for C symbols, so it must be stable.

    ASCII only. str.isalnum() is true for Unicode letters, so 'Cafe' spelled with
    an accent used to survive into ble_caf\u00e9_set(), which is not a portable C
    identifier. Anything outside ASCII becomes '_' like any other separator; two
    names that collide after folding are caught by the duplicate-symbol check.
    """
    out = "".join(c.lower() if (c.isalnum() and c.isascii()) else "_" for c in name)
    while "__" in out:
        out = out.replace("__", "_")
    out = out.strip("_")
    if not out:
        return "unnamed"
    # A C identifier cannot start with a digit, and '9 Lives' otherwise folds to
    # '9_lives' and emits ble_9_lives_set(), which will not compile.
    return out if out[0].isalpha() or out[0] == "_" else "_" + out


BLE_PUBLISH_DEFAULT_HZ = 10

# A DSL global lowers to `int` or `const char *`, and those are the only two
# storages a generated publisher can read. The characteristic's own type says
# how the value goes on the wire; the binding only has to name a variable whose
# C type the publisher can declare.
BLE_BIND_STORAGE = {"scalar": "int", "string": "const char *"}


def _ble_binding(ch, cname, c_type, is_string, notify, errors):
    """Parse `source` / `publish` into publisher context, or None for escape-to-C.

    None means "coregen emits a setter and nothing else", which is source "code"
    and also the default: a contract written before binding existed keeps
    behaving exactly as it did.
    """
    source = ch.get("source")
    if source is None or source == "code":
        return None

    if not isinstance(source, dict):
        errors.append(
            f"ble.contract: '{cname}' has source {source!r}. Expected \"code\", "
            f"{{\"var\": \"name\"}} or {{\"tile\": \"handle.method\"}}")
        return None

    if "tile" in source:
        errors.append(
            f"ble.contract: '{cname}' binds a tile reading ({source['tile']!r}), which "
            f"coregen cannot emit yet. Read the tile into a variable in your program and "
            f"bind that instead: \"source\": {{\"var\": \"...\"}}")
        return None

    var = source.get("var")
    if not isinstance(var, str) or not var:
        errors.append(f"ble.contract: '{cname}' has a source with no 'var'")
        return None
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", var):
        errors.append(
            f"ble.contract: '{cname}' binds variable '{var}', which is not a C identifier")
        return None

    if c_type:
        storage = BLE_BIND_STORAGE["scalar"]
    elif is_string:
        storage = BLE_BIND_STORAGE["string"]
    else:
        errors.append(
            f"ble.contract: '{cname}' is type 'bytes', which has no variable form to bind. "
            f"Use \"source\": \"code\" and call ble_{_ble_ident(cname)}_set() yourself")
        return None

    publish = ch.get("publish", "on_change")
    mode, hz = "on_change", BLE_PUBLISH_DEFAULT_HZ
    if publish == "always":
        mode = "always"
    elif isinstance(publish, dict) and "hz" in publish:
        hz = publish["hz"]
        if not isinstance(hz, int) or isinstance(hz, bool) or not 1 <= hz <= 1000:
            errors.append(
                f"ble.contract: '{cname}' declares publish hz {publish['hz']!r}; "
                f"expected a whole number from 1 to 1000")
            return None
    elif publish != "on_change":
        errors.append(
            f"ble.contract: '{cname}' has publish {publish!r}. Expected \"on_change\", "
            f"\"always\" or {{\"hz\": N}}")
        return None

    return {"var": var, "storage": storage, "mode": mode,
            "interval_ms": max(1, 1000 // hz), "hz": hz,
            # Nothing is published to nobody. A notify characteristic waits for a
            # subscriber; a plain read one only needs a connection, because the
            # central reads the stored value on demand rather than being pushed it.
            "gate": "subscribed" if notify else "connected"}


def build_ble_contract(project, config_path, errors, warnings=None):
    """Parse the `ble.contract` block into template context.

    Accepts the contract inline, or as a filename resolved relative to the
    project directory. The file form is for hand- and agent-authored projects
    (surgical edits, sane diffs); Studio always emits inline because the cloud
    build service materialises only main.c / config.json / Makefile and has no
    file map to carry a second document.

    Ids must be explicit. Studio assigns one when a characteristic is created
    and persists it (auto-assign-then-freeze), so by the time a contract reaches
    coregen every id is already pinned. coregen deliberately does not invent
    ids: a generated id would be positional, and inserting a service would then
    renumber everything after it and break already-deployed clients.
    """
    ble = project.get("ble") or {}
    contract = ble.get("contract")
    if contract is None:
        return None

    if isinstance(contract, str):
        base = os.path.dirname(config_path) if config_path else "."
        path = os.path.join(base, contract)
        if not os.path.isfile(path):
            errors.append(f"ble.contract: no such file '{contract}' (looked in {base})")
            return None
        try:
            with open(path, encoding="utf-8") as f:
                contract = json.load(f)
        except json.JSONDecodeError as e:
            errors.append(f"ble.contract: {contract} is not valid JSON - {e}")
            return None

    services_in = contract.get("services") if isinstance(contract, dict) else contract
    if not isinstance(services_in, list):
        errors.append("ble.contract: expected a list of services (or an object with 'services')")
        return None

    services, seen_ids, seen_syms = [], {}, {}
    for svc in services_in:
        sname = svc.get("name")
        if not sname:
            errors.append("ble.contract: every service needs a 'name'")
            continue
        sid, ssig = svc.get("id"), svc.get("sig")
        if (sid is None) == (ssig is None):
            errors.append(f"ble.contract: service '{sname}' needs exactly one of 'id' or 'sig'")
            continue
        chars = []
        for ch in svc.get("characteristics", []):
            cname = ch.get("name")
            if not cname:
                errors.append(f"ble.contract: a characteristic in '{sname}' has no 'name'")
                continue
            cid, csig = ch.get("id"), ch.get("sig")
            if (cid is None) == (csig is None):
                errors.append(
                    f"ble.contract: characteristic '{cname}' needs exactly one of 'id' or 'sig'")
                continue
            uuid = cid if cid is not None else csig
            key = str(uuid).lower()
            if key in seen_ids:
                errors.append(
                    f"ble.contract: id {uuid} used by both '{seen_ids[key]}' and '{cname}'")
            seen_ids[key] = cname

            access = ch.get("access") or ["read"]
            bad = [a for a in access if a not in BLE_ACCESS_FLAGS]
            if bad:
                errors.append(f"ble.contract: '{cname}' has unknown access {bad}")
                continue

            ctype = ch.get("type", "bytes")
            if ctype in BLE_SCALARS:
                size_expr, c_type = BLE_SCALARS[ctype]
                length = None
            else:
                length = ch.get("len") or ch.get("max_len")
                if not length:
                    errors.append(
                        f"ble.contract: '{cname}' is type '{ctype}' so it needs 'len'")
                    continue
                size_expr, c_type = f"CORE_BLE_BYTES({length})", None

            sym = _ble_ident(cname)
            if sym in seen_syms:
                errors.append(
                    f"ble.contract: '{cname}' and '{seen_syms[sym]}' both map to the C symbol "
                    f"'{sym}' - rename one")
            seen_syms[sym] = cname

            bind = _ble_binding(ch, cname, c_type, ctype == "string",
                                "notify" in access, errors)

            # A readable characteristic that names no source at all is the
            # forgotten-publish case the design calls out. Explicit "code" is
            # not: the author said they would publish it themselves, which is
            # what every hand-written contract does.
            if (warnings is not None and bind is None and "source" not in ch
                    and ("read" in access or "notify" in access)):
                warnings.append(
                    f"ble.contract: nothing publishes '{cname}' - it will always read zero. "
                    f"Bind it with \"source\": {{\"var\": \"...\"}}, or say "
                    f"\"source\": \"code\" if you publish it yourself")

            chars.append({
                "name": cname, "sym": sym, "uuid": uuid, "is_sig": csig is not None,
                "access_expr": " | ".join(BLE_ACCESS_FLAGS[a] for a in access),
                "writable": "write" in access,
                "notify": "notify" in access,
                "bind": bind,
                "size_expr": size_expr, "c_type": c_type, "len": length,
                # string and bytes are the same on the wire, but not in the API
                # we can offer: a string has a NUL-terminated form the DSL can
                # pass and receive, where a raw byte buffer does not.
                "is_string": ctype == "string",
                "define": "BLE_CH_" + sym.upper(),
            })
        services.append({
            "name": sname, "sym": _ble_ident(sname), "uuid": sid if sid is not None else ssig,
            "is_sig": ssig is not None, "note": svc.get("note"), "characteristics": chars,
            "define": "BLE_SVC_" + _ble_ident(sname).upper(),
        })

    all_chars = [c for s in services for c in s["characteristics"]]

    if warnings is not None:
        _ble_orphan_handlers(all_chars, config_path, warnings)

    return {"services": services,
            "chars": all_chars,
            "bound": [c for c in all_chars if c["bind"]]}


def _ble_orphan_handlers(chars, config_path, warnings):
    """Flag ble_*_on_write definitions in main.c that match no characteristic.

    A text scan, not a parse: it reads the project's main.c if there is one
    beside config.json (there is in the cloud path too, which materialises both).
    Renaming a characteristic leaves the old handler behind silently otherwise,
    because the weak default still links and the write just stops arriving.
    """
    if not config_path:
        return
    main_c = os.path.join(os.path.dirname(config_path) or ".", "main.c")
    if not os.path.isfile(main_c):
        return
    try:
        with open(main_c, encoding="utf-8", errors="replace") as f:
            src = f.read()
    except OSError:
        return
    known = {c["sym"] for c in chars}
    seen = set()
    for m in re.finditer(r"\bble_([A-Za-z0-9_]+)_on_write\s*\(", src):
        sym = m.group(1)
        if sym in known or sym in seen:
            continue
        seen.add(sym)
        warnings.append(
            f"main.c defines 'ble_{sym}_on_write' but no characteristic maps to the symbol "
            f"'{sym}' - it will never be called. Renamed or removed from the contract?")


BLE_TX_POWER = {"low": 0, "medium": 1, "high": 2}


def build_ble_radio(project, project_name, errors):
    """Radio-level settings from the `ble` block -> generated setter calls.

    Every field maps to a real core_ble setter; nothing here is decorative.
    Absent fields are simply not emitted, so the SDK default stands.
    """
    ble = project.get("ble") or {}
    if not ble.get("enabled"):
        return None

    # Always resolve a name. Studio's generated main.c advertises with
    # BLE_DEVICE_NAME, so the macro has to exist even when nobody typed one —
    # otherwise "enable BLE" produces firmware that will not compile. Falling
    # back to the project name means a project that enables the radio and
    # changes nothing else still shows up in a scanner under a recognisable
    # name, which is the whole point of the toggle.
    out = {"name": ble.get("name") or ble.get("device_name") or project_name}

    tx = ble.get("tx_power")
    if tx is not None:
        if isinstance(tx, str):
            if tx.lower() not in BLE_TX_POWER:
                errors.append(
                    f"ble.tx_power: '{tx}' is not one of low / medium / high")
            else:
                out["tx_power"] = BLE_TX_POWER[tx.lower()]
        elif isinstance(tx, int) and 0 <= tx <= 2:
            out["tx_power"] = tx
        else:
            errors.append(f"ble.tx_power: expected low/medium/high or 0-2, got {tx!r}")

    adv = ble.get("adv_interval_ms", ble.get("adv_interval"))
    if adv is not None:
        lo = hi = None
        if isinstance(adv, dict):
            lo, hi = adv.get("min"), adv.get("max")
        elif isinstance(adv, (int, float)):
            lo = hi = int(adv)
        if not isinstance(lo, int) or not isinstance(hi, int):
            errors.append("ble.adv_interval_ms: expected a number, or {min, max}")
        elif not (20 <= lo <= 10240 and 20 <= hi <= 10240):
            errors.append(
                f"ble.adv_interval_ms: {lo}-{hi} ms is outside the 20-10240 ms the radio allows")
        elif lo > hi:
            errors.append(f"ble.adv_interval_ms: min {lo} is greater than max {hi}")
        else:
            out["adv_min"], out["adv_max"] = lo, hi

    # Pairing is Just Works + bonding, and it is all-or-nothing: once on, every
    # characteristic sits behind an encrypted link.
    pairing = ble.get("pairing")
    if isinstance(pairing, str):
        pairing = pairing.lower() not in ("none", "off", "false", "")
    out["pairing"] = bool(pairing)

    return out


# Thread network defaults: DEVELOPMENT ONLY. Every node built with them joins
# the same network with a well-known key. sdk/core/core_thread.c falls back to
# the same values when a build forces THREAD_ENABLED on a config without a
# "thread" block; keep the two in step.
THREAD_DEFAULTS = {
    "channel": 15,
    "panid": 0x1234,
    "network_key": "00112233445566778899aabbccddeeff",
    "network_name": "tiles-dev",
    "ext_panid": "dead00beef00cafe",
    "mesh_local_prefix": "fdde:ad00:beef:0::",
}


def _hex_bytes(field, value, n, errors):
    """'<2n hex digits>' (':' / '-' / spaces allowed) -> list of n ints."""
    if not isinstance(value, str):
        errors.append(f"thread.{field}: expected a string of {2 * n} hex digits, got {value!r}")
        return None
    digits = re.sub(r"[\s:\-]", "", value)
    if digits.lower().startswith("0x"):
        digits = digits[2:]
    if len(digits) != 2 * n or not re.fullmatch(r"[0-9a-fA-F]+", digits):
        errors.append(f"thread.{field}: expected {2 * n} hex digits ({n} bytes), got {value!r}")
        return None
    return [int(digits[i:i + 2], 16) for i in range(0, 2 * n, 2)]


def thread_supported(mcu):
    """The back end's thread_supported(mcu): an optional hook (a back end
    without it has no Thread)."""
    hook = getattr(backend(), "thread_supported", None)
    return bool(hook and hook(mcu))


def build_thread_config(project, mcu, errors):
    """The `thread` block -> network credentials for coregen/thread_dataset.h.

    Milestone 1: a fixed network for development, no commissioning. Every key
    is optional; absent ones take THREAD_DEFAULTS.
    """
    th = project.get("thread") or {}
    if not th.get("enabled"):
        return None
    if not thread_supported(mcu):
        errors.append("thread: Thread is only supported on Core.ST.W5 (STM32WBA55). "
                      "Remove \"thread\" from config.json, or build for Core.ST.W5.")
        return None
    # "ble" may be enabled too: one image runs both (ST's concurrent link
    # layer; the Makefile's BLE + Thread block, stm32wba55hg_ble_thread.ld).

    out = {}
    role = str(th.get("role", "ftd")).lower()
    if role != "ftd":
        errors.append(f"thread.role: '{role}' is not supported yet; only \"ftd\" (a full, "
                      f"router-capable Thread device)")
    out["role"] = "ftd"

    ch = th.get("channel", THREAD_DEFAULTS["channel"])
    if not isinstance(ch, int) or isinstance(ch, bool) or not 11 <= ch <= 26:
        errors.append(f"thread.channel: expected an 802.15.4 channel 11-26, got {ch!r}")
        ch = THREAD_DEFAULTS["channel"]
    out["channel"] = ch

    pan = th.get("panid", THREAD_DEFAULTS["panid"])
    if isinstance(pan, str):
        try:
            pan = int(pan, 0)
        except ValueError:
            pan = None
    if not isinstance(pan, int) or isinstance(pan, bool) or not 0 <= pan <= 0xFFFE:
        errors.append(f"thread.panid: expected 0x0000-0xFFFE (0xFFFF is broadcast), got {th.get('panid')!r}")
        pan = THREAD_DEFAULTS["panid"]
    out["panid"] = pan

    key = _hex_bytes("network_key", th.get("network_key", THREAD_DEFAULTS["network_key"]), 16, errors)
    out["network_key"] = key or _hex_bytes("network_key", THREAD_DEFAULTS["network_key"], 16, [])

    name = th.get("network_name", THREAD_DEFAULTS["network_name"])
    if not isinstance(name, str) or not 1 <= len(name.encode("utf-8")) <= 16:
        errors.append(f"thread.network_name: expected 1-16 bytes of text, got {name!r}")
        name = THREAD_DEFAULTS["network_name"]
    elif any(c in name for c in '"\\') or not name.isprintable():
        errors.append(f"thread.network_name: printable text without quotes or backslashes, got {name!r}")
        name = THREAD_DEFAULTS["network_name"]
    out["network_name"] = name

    xpan = _hex_bytes("ext_panid", th.get("ext_panid", THREAD_DEFAULTS["ext_panid"]), 8, errors)
    out["ext_panid"] = xpan or _hex_bytes("ext_panid", THREAD_DEFAULTS["ext_panid"], 8, [])

    mlp = th.get("mesh_local_prefix", THREAD_DEFAULTS["mesh_local_prefix"])
    mlp_bytes = None
    if isinstance(mlp, str):
        try:
            net = ipaddress.IPv6Network(mlp if "/" in mlp else mlp + "/64", strict=False)
            if net.prefixlen != 64:
                raise ValueError("not /64")
            mlp_bytes = list(net.network_address.packed[:8])
        except ValueError:
            pass
    if mlp_bytes is None:
        errors.append(f"thread.mesh_local_prefix: expected an IPv6 /64 prefix such as "
                      f"\"fd00:1234:5678:9abc::\", got {mlp!r}")
        mlp_bytes = list(ipaddress.IPv6Network(THREAD_DEFAULTS["mesh_local_prefix"] + "/64").network_address.packed[:8])
    out["mesh_local_prefix"] = mlp_bytes
    out["mesh_local_prefix_str"] = str(ipaddress.IPv6Address(bytes(mlp_bytes) + bytes(8))) + "/64"

    out["default_key"] = "network_key" not in th
    if out["default_key"]:
        eprint("  NOTE: thread: no network_key in config.json, using the well-known "
               "development key (anyone can join this network)")

    # Run-time provisioning: "provisioning" lets core_thread_set_dataset() store
    # a network that then wins over this one at boot (config.json's becomes the
    # factory default); "ble_provisioning" adds the BLE service that lets a
    # phone do it (docs/thread-provisioning-protocol.md) and implies the first.
    prov, ble_prov = th.get("provisioning", False), th.get("ble_provisioning", False)
    for field, value in (("provisioning", prov), ("ble_provisioning", ble_prov)):
        if not isinstance(value, bool):
            errors.append(f"thread.{field}: expected true or false, got {value!r}")
    ble_prov = ble_prov is True
    if ble_prov and not (project.get("ble") or {}).get("enabled"):
        errors.append("thread.ble_provisioning: needs BLE in the same image; add "
                      "\"ble\": {\"enabled\": true} to config.json, or use \"provisioning\" "
                      "(core_thread_set_dataset() from C) without BLE")
        ble_prov = False
    out["provisioning"] = prov is True or ble_prov
    out["ble_provisioning"] = ble_prov
    if ble_prov:
        eprint("  NOTE: thread.ble_provisioning is DEVELOPMENT ONLY: any phone that "
               "connects can move this node to another Thread network (no pairing)")
    return out


def generate(tile_path, output_dir, config_path=None, port=None, status_path=None):
    """Generate all headers from a tile JSON and optional project config.

    port: the back end to use (a port name such as "stm32"); None keeps the
    active one (stm32 unless chosen before). status_path: the Core's status
    file (the Makefile passes core.mk's CORE_STATUS), from which core_caps.h
    is written; None writes no core_caps.h."""
    be = use_backend(port) if port else backend()
    with open(tile_path, encoding="utf-8") as f:
        tile = json.load(f)

    # Resolve MCU info
    part = tile["components"][0]["part"]
    mcu = be.mcu_for_part(part)
    if mcu is None:
        eprint(f"ERROR: Unknown MCU part '{part}' for port '{be.PORT}'. Add it to the "
               f"back end's part table (tools/coregen/ports/{be.PORT}.py).")
        sys.exit(1)

    # Build template context
    pad_map = build_pad_map(tile["pads"])
    system_pads = extract_system_pads(tile["pads"])
    led = be.led_info(tile)
    interfaces = build_interface_map(tile, pad_map)
    power = tile.get("power", [{}])[0] if tile.get("power") else {}

    # The public name (Core.ST.<family>[.<n>]) for user-visible output, from
    # the alias table or else the file stem. The identity defines below follow
    # it rather than the JSON's family/name fields, which can lag a DB rename.
    _stem = os.path.basename(tile_path).replace(".json", "")
    tile_name = core_public_name(_stem)
    _family, _, _variant = tile_name.partition(".")

    ctx = {
        "tile": tile,
        "tile_name": tile_name,
        "tile_family": _family,
        "tile_variant": _variant,
        "tile_rev": tile["rev"],
        "tile_headline": tile.get("headline", ""),
        "json_version": tile.get("json_version", ""),
        "mcu_part": part,
        "mcu": mcu,
        "pad_count": tile["package"]["pads"],
        "pads": pad_map,
        "system_pads": system_pads,
        "led": led,
        "interfaces": interfaces,
        "power": power,
        "source_file": os.path.basename(tile_path),
    }

    # Load and validate project config if provided
    templates = ["core_pads.h.j2", "core_board.h.j2", "core_interfaces.h.j2"]

    if config_path:
        with open(config_path, encoding="utf-8") as f:
            project = json.load(f)

        # Validate core matches. Public names (Core.ST.<family>[.<n>]) resolve
        # to the canonical definition stem, so a config written against the
        # public name doesn't spuriously warn. Pre-rename names still resolve,
        # with a NOTE on stderr (stdout is silenced in quiet builds).
        proj_core = project.get("core", "")
        proj_core_stem, rename_note = resolve_core_name(proj_core)
        if rename_note:
            eprint(f"  {rename_note}")
        tile_file_stem = os.path.basename(tile_path).replace(".json", "")
        if proj_core and proj_core_stem != tile_file_stem:
            print(f"  NOTE: config.json targets '{proj_core}', building for '{tile_file_stem}' (TILE= override)")
            # Allow override — this is the multi-tile portability path

        # SPI tiles' chip-select pads join the pad map first, so they are
        # validated and initialized (deasserted) like any other pad.
        cs_errors = validate_tile_buses(project) + merge_tile_cs_pads(project)

        # Validate pin/interface/clock assignments
        warnings, errors = validate_project_config(project, tile, pad_map, mcu)
        errors = cs_errors + errors

        # BLE contract: declared services/characteristics -> generated GATT.
        # Parsed alongside the other validation so a malformed contract fails
        # the build with a clear message rather than emitting broken C.
        ctx["ble_contract"] = build_ble_contract(project, config_path, errors, warnings)
        ctx["thread"] = build_thread_config(project, mcu, errors)
        # Thread joins on boot: core_init() starts it in a Thread-only build;
        # with BLE too, core_ble_init() does (sdk/core/core_ble.c), since the
        # radio has to come up BLE first.
        ctx["thread_autostart"] = bool(ctx["thread"]) and not (project.get("ble") or {}).get("enabled")
        ctx["ble_radio"] = build_ble_radio(
            project,
            os.path.basename(os.path.dirname(os.path.abspath(config_path)))
            if config_path
            else "Bergsonne",
            errors,
        )

        # stderr, not stdout: a normal build runs coregen with stdout sent to
        # /dev/null (Makefile V=0), so a warning printed to stdout is a warning
        # nobody ever sees.
        for w in warnings:
            eprint(f"  WARNING: {w}")
        if errors:
            for e in errors:
                eprint(f"  ERROR: {e}")
            sys.exit(1)

        # Build resolved configs.
        # Project name is sourced from the parent directory — config.json no
        # longer carries it (name + description are project identity, not
        # hardware configuration; see Studio X1a).
        ctx["project_name"] = os.path.basename(os.path.dirname(os.path.abspath(config_path)))
        # I3C requests ("i3c": true on an I2C bus) are settled before the pad
        # and bus builders run, so a switched bus reaches them as I3C only.
        ctx["i3c_buses"] = resolve_i3c_buses(project, pad_map, mcu, definition_dirs(tile_path))
        apply_i3c_buses(project, ctx["i3c_buses"])
        ctx["pad_config"] = build_pad_config(project, pad_map)
        ctx["clock_config"] = build_clock_config(project, tile, mcu)
        ctx["iface_config"] = project.get("interfaces", {})
        ctx["config_file"] = os.path.basename(config_path)
        ctx["i2c_buses"] = build_i2c_config(project, mcu, ctx["clock_config"])
        ctx["i2c_pullups"] = {bus["instance"]: bus["pullups"] for bus in ctx["i2c_buses"]}
        ctx["pullup_pins"] = build_pullup_config(project, tile, ctx["i2c_buses"])
        ctx["spi_buses"] = build_spi_config(project, mcu, pad_map, ctx["clock_config"])
        ctx["usart_buses"] = build_usart_config(project, mcu)
        ctx["pwm_timers"] = build_pwm_config(project, mcu)
        ctx["adc_config"] = build_adc_config(project)
        # Studio tick dispatcher — polled from main loop via hal_tick().
        # Null means "no tick configured" → coregen emits a no-op stub.
        _timer_cfg = project.get("timer", {})
        ctx["studio_tick_ms"] = _timer_cfg.get("tick_ms")
        # Port-specific context (STM32: the I2C / SAI kernel clock routing).
        ctx.update(be.extra_context(project, mcu))
        _usb_cfg = project.get("usb", {})
        ctx["usb_enabled"] = _usb_cfg.get("enabled", False)
        # Per-project USB identity → descriptor overrides in hal_usb_cdc.c
        # (each unset field falls back to the SDK default). CMSIS-DAP hosts
        # auto-detect by matching "CMSIS-DAP" in the product string.
        def _parse_usb_id(v):
            if v is None or v == "":
                return None
            return v if isinstance(v, int) else int(str(v), 16)
        ctx["usb_vid"] = _parse_usb_id(_usb_cfg.get("vid"))
        ctx["usb_pid"] = _parse_usb_id(_usb_cfg.get("pid"))
        ctx["usb_product"] = _usb_cfg.get("product")
        ctx["usb_manufacturer"] = _usb_cfg.get("manufacturer")
        ctx["usb_serial"] = _usb_cfg.get("serial")
        # USB-capable MCUs always get CDC init in core_init() so the
        # DFU bootloader's 1200-baud touch reboot always works — even
        # when main.c forgets to call core_usb_init().
        ctx["usb_capable"] = be.usb_capable(mcu)
        # Bootloader mode: "none", "custom", or "rom" (from config.json)
        ctx["bootloader_mode"] = project.get("bootloader", "none")
        # Watchdog-by-default (Part A) + strike→ROM-DFU brick recovery (Part B).
        # These are INDEPENDENT and the split matters:
        #   - The watchdog is plain IWDG (ll_iwdg) and works on EVERY Core. It is
        #     armed by default by the Studio starter scaffolding and by the
        #     starter templates, which write iwdg.enabled=true AND feed the dog.
        #   - The strike→SOS→ROM-DFU escape needs USB, so it only compiles into
        #     ROM_DFU builds of a usb_capable part (see core_init.c.j2).
        # A Core without USB (W5, L0) recovers over SWD instead — a probe can
        # always halt and reflash it — so arming the watchdog there is normal.
        # Still opt-in at the config level: absent iwdg config → OFF, so
        # regenerating an existing project never surprises it with resets.
        _iwdg_cfg = project.get("iwdg", {}) or {}
        ctx["iwdg_enabled"] = bool(_iwdg_cfg.get("enabled", False))
        ctx["iwdg_timeout_ms"] = int(_iwdg_cfg.get("timeout_ms", 5000))
        if ctx["iwdg_enabled"] and ctx["usb_capable"] and ctx["bootloader_mode"] != "rom":
            print(
                "  WARNING: iwdg is enabled on a USB-capable Core without ROM_DFU "
                "bootloader mode — an un-fed watchdog will reset-loop, giving up "
                'the strike→DFU recovery this Core could have had. Set '
                '"bootloader": "rom" to get it.'
            )
        ctx["timer_pads"] = build_timer_config(project, pad_map)

        # DAC: detect from pad config
        dac_pads = [p for p in ctx["pad_config"] if p.get("dac")]
        ctx["dac_enabled"] = len(dac_pads) > 0
        ctx["dac_pad"] = dac_pads[0] if dac_pads else None

        # Build tile peripheral driver config
        tiles_config, tile_pal_buses, tile_driver_sources = build_tiles_config(
            project, ctx["i2c_buses"], ctx["spi_buses"], pad_map, ctx["i3c_buses"]
        )
        ctx["tiles_config"] = tiles_config
        ctx["tile_pal_buses"] = tile_pal_buses
        ctx["tile_driver_sources"] = tile_driver_sources

        # Collect unique driver headers for includes
        seen_headers = []
        for tc in tiles_config:
            if tc["header"] not in seen_headers:
                seen_headers.append(tc["header"])
        ctx["tile_driver_headers"] = seen_headers

        templates.append("core_config.h.j2")
        templates.append("core_init.h.j2")
        templates.append("core_init.c.j2")
        templates.append("core.h.j2")
        if ctx.get("ble_contract") or ctx.get("ble_radio"):
            templates.append("ble_contract.h.j2")
            templates.append("ble_contract.c.j2")
        if ctx.get("thread"):
            templates.append("thread_dataset.h.j2")

    # Set up Jinja2. The back end's template directories come first, so a
    # port can replace any template by file name.
    templates_dir = os.path.join(os.path.dirname(__file__), "templates")
    env = Environment(
        loader=FileSystemLoader([*be.template_dirs(), templates_dir]),
        keep_trailing_newline=True,
        trim_blocks=True,
        lstrip_blocks=True,
    )

    # Generate each template
    os.makedirs(output_dir, exist_ok=True)

    for template_name in templates:
        out_name = template_name.replace(".j2", "")
        template = env.get_template(template_name)
        output = template.render(**ctx)

        # core.h goes next to main.c (project dir), everything else to generated dir
        if out_name == "core.h" and config_path:
            out_path = os.path.join(os.path.dirname(config_path), out_name)
        else:
            out_path = os.path.join(output_dir, out_name)

        with open(out_path, "w", encoding="utf-8") as f:
            f.write(output)
        print(f"  {out_name}")

    # core_caps.h: the Core's CORE_HAS_* and the functions it lacks, from its
    # status file (tools/coregen/core_caps.py).
    if status_path:
        try:
            core_caps.generate(status_path, os.path.join(output_dir, "core_caps.h"),
                               core_name=tile_name)
        except (core_caps.CapsError, OSError, ValueError) as e:
            eprint(f"ERROR: core_caps.h from {status_path}: {e}")
            sys.exit(1)
        print(f"  core_caps.h")

    # Generate core_drivers.mk if tiles are declared
    if ctx.get("tile_driver_sources"):
        mk_path = os.path.join(output_dir, "core_drivers.mk")
        with open(mk_path, "w", encoding="utf-8") as f:
            f.write("# AUTO-GENERATED by coregen — do not edit\n")
            # Overlay drivers by path (without .c), public ones by name.
            overlay = {}
            for tc in ctx.get("tiles_config") or []:
                if tc.get("driver_dir"):
                    extras = tile_driver_map()[tc["type"]].get("extra_sources", [])
                    for name in [tc["source"], *extras]:
                        overlay[name] = os.path.join(tc["driver_dir"], name).replace(os.sep, "/")
            public = [n for n in ctx["tile_driver_sources"] if n not in overlay]
            # The TILES_OVERLAY this was generated with: the Makefile
            # regenerates when it changes, so no stale overlay path survives.
            f.write("TILES_OVERLAY_GEN = " + " ".join(os.environ.get("TILES_OVERLAY", "").split()) + "\n")
            f.write("TILES_DRIVERS = " + " ".join(public) + "\n")
            if overlay:
                f.write("TILES_OVERLAY_DRIVERS = " + " ".join(overlay[n] for n in sorted(overlay)) + "\n")
        print(f"  core_drivers.mk")

    # Per-project WAMR natives: wraps the Tier 2 functions whose adapters
    # need coregen-emitted state (PAD_*_PORT macros, core_dac extern,
    # etc.). Always emitted when a config exists; the Makefile decides
    # whether to compile it (only when WAMR_ENABLED=1). Reuses the same
    # generator as the static SDK table — just runs it in `--mode project`.
    if config_path:
        # Import lazily so coregen.py doesn't carry a hard dependency on
        # the natives generator for projects that don't use WAMR.
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        try:
            import gen_studio_natives as gtn
        finally:
            sys.path.pop(0)
        all_fns = gtn.load_manifests(gtn.SDK_DOCS)
        kept = [
            fn for fn in all_fns
            if fn.skip_reason(mode="project") is None
        ]
        skipped = [
            (fn, fn.skip_reason(mode="project"))
            for fn in all_fns
            if fn.skip_reason(mode="project")
        ]
        c_text = gtn.emit_c(kept, skipped, mode="project")
        h_text = gtn.emit_h(kept, mode="project")
        c_path = os.path.join(output_dir, "studio_natives_project.c")
        h_path = os.path.join(output_dir, "studio_natives_project.h")
        with open(c_path, "w", encoding="utf-8") as f:
            f.write(c_text)
        with open(h_path, "w", encoding="utf-8") as f:
            f.write(h_text)
        print(f"  studio_natives_project.c ({len(kept)} natives)")
        print(f"  studio_natives_project.h")

    # Generate or update tiles.h (smart merge) in the project directory
    if config_path and ctx.get("tiles_config"):
        project_dir = os.path.dirname(config_path)
        generate_tiles_h(env, ctx, project_dir)

    # Generate project Makefile (once — skip if already exists)
    if config_path:
        project_dir = os.path.dirname(config_path)
        makefile_path = os.path.join(project_dir, "Makefile")
        if not os.path.exists(makefile_path):
            tile_stem = os.path.basename(tile_path).replace(".json", "")
            tile_public = core_public_name(tile_stem)
            tiles_line = "TILES_ENABLED := 1\n" if ctx.get("tiles_config") else ""
            with open(makefile_path, "w", encoding="utf-8") as f:
                f.write(f"# Project Makefile — run make from inside the project folder\n")
                f.write(f"TILE         := {tile_public}\n")
                f.write(f"{tiles_line}")
                f.write(f"PROJECT      := $(notdir $(CURDIR))\n")
                f.write(f"ROOT         := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))../..)\n\n")
                f.write(f".PHONY: all clean clean-all distclean flash flash-dfu size generate\n")
                f.write(f"all clean clean-all distclean flash flash-dfu size generate:\n")
                f.write(f"\t$(MAKE) -C $(ROOT) TILE=$(TILE) PROJECT=$(PROJECT) TILES_ENABLED=$(TILES_ENABLED) $@\n")
            print(f"  Makefile")
        else:
            print(f"  Makefile (exists, skipped)")

    print(f"  -> {output_dir}/")


def main():
    parser = argparse.ArgumentParser(
        description="Generate C headers from Mosaic tile JSON definitions."
    )
    parser.add_argument("tile_json", help="Path to the tile JSON definition")
    parser.add_argument("output_dir", nargs="?", default="generated",
                        help="Output directory (default: generated)")
    parser.add_argument("--config", "-c", metavar="FILE",
                        help="Path to config.json hardware definition")
    parser.add_argument("--status", metavar="FILE",
                        help="The Core's status file (sdk/status/core-*.json, the "
                             "Makefile passes core.mk's CORE_STATUS): writes "
                             "core_caps.h from it")
    parser.add_argument("--port", default=DEFAULT_PORT, metavar="PORT",
                        help="Back end: tools/coregen/ports/<PORT>.py here or in a "
                             f"TILES_OVERLAY checkout (default: {DEFAULT_PORT}; the "
                             "Makefile passes the Core's core.mk PORT)")

    args = parser.parse_args()

    if not os.path.exists(args.tile_json):
        eprint(f"ERROR: File not found: {args.tile_json}")
        sys.exit(1)

    if args.config and not os.path.exists(args.config):
        eprint(f"ERROR: Config file not found: {args.config}")
        sys.exit(1)

    if args.status and not os.path.exists(args.status):
        eprint(f"ERROR: Status file not found: {args.status} (the Core's core.mk CORE_STATUS)")
        sys.exit(1)

    use_backend(args.port)
    tile_name = os.path.basename(args.tile_json).replace(".json", "")
    print(f"coregen: {tile_name}")
    generate(args.tile_json, args.output_dir, args.config, status_path=args.status)


# ---- The STM32 back end, loaded at import ----
# A back end may `import coregen` for the front's helpers; run as a script,
# this module is __main__, so it is registered under "coregen" too (one copy).
# The STM32 names that lived here before the split (MCU_DB, solve_pll, the
# clock-enable maps, ...) stay importable from coregen.
sys.modules.setdefault("coregen", sys.modules[__name__])
_STM32 = load_backend(DEFAULT_PORT)
MCU_DB = _STM32.MCU_DB
solve_pll = _STM32.solve_pll
parse_gpio = _STM32.parse_gpio
extract_pad_gpio = _STM32.extract_pad_gpio
extract_led_info = _STM32.extract_led_info
extract_timer_channels = _STM32.extract_timer_channels
I2C_CLK_MAP = _STM32.I2C_CLK_MAP
I2C_TIMING_MAP = _STM32.I2C_TIMING_MAP
I2C_MIN_CLOCK = _STM32.I2C_MIN_CLOCK
USART_CLK_MAP = _STM32.USART_CLK_MAP
TIMER_CLK_MAP = _STM32.TIMER_CLK_MAP
SPI_CLK_MAP = _STM32.SPI_CLK_MAP
SPI_PRESCALER_MAP = _STM32.SPI_PRESCALER_MAP
SPI_MAX_SCK_MHZ = _STM32.SPI_MAX_SCK_MHZ
I3C_MCUS = _STM32.I3C_MCUS


if __name__ == "__main__":
    main()
