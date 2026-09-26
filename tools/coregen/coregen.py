#!/usr/bin/env python3
"""
coregen — Generate C headers from Mosaic tile JSON definitions.

Usage:
    python3 coregen.py <tile.json> [output_dir] [--config config.json]

Reads a tile JSON file and produces:
    core_pads.h        Pad-to-GPIO mapping defines
    core_board.h       Board-level defines (LED, power, debug)
    core_interfaces.h  Interface convenience defines with AF numbers
    core_config.h      Project-specific pin and clock configuration
                       (only when --project is provided)

Output defaults to ./generated/ if not specified.
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path

from jinja2 import Environment, FileSystemLoader


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
# Vendor-segmented public names (Core.ST.<family>.<n>) are the standard; they
# map onto the DB-synced definition file stems. The Makefile carries the same
# map so either form resolves to the right definition.
CORE_NAME_ALIASES = {
    "Core.ST.L0.1": "Core-ST-L0-1-a",
    "Core.ST.L4.1": "Core-ST-L4-1-b",  # rev b — superset of rev a (adds PA13/PA14)
    "Core.ST.L4.2": "Core-ST-L4-2-a",
    "Core.ST.W5": "Core-ST-W5-b",
    "Core.ST.H5.1": "Core-ST-H5-1-a",
}
CORE_STEM_TO_PUBLIC = {stem: name for name, stem in CORE_NAME_ALIASES.items()}

# ---- MCU database ----
# Maps part numbers to build-relevant properties.

MCU_DB = {
    "STM32L011E4": {
        "define": "STM32L011xx",
        "family": "stm32l0xx",
        "core": "cortex-m0plus",
        "cpu_flag": "-mcpu=cortex-m0plus",
        "fpu": None,
        "max_sysclk_mhz": 32,
        # RM0377 §7.2.4 / §7.3.3 (RCC_CFGR PLLMUL, PLLDIV): no input divider,
        # multiplier from a fixed set, output /2 /3 /4. Input 2-24 MHz, VCO at
        # most 96 MHz in range 1 (48 in range 2, 24 in range 3), SYSCLK <= 32.
        # (These limits were copied from the L4: "max" ran the VCO at 128 MHz.)
        "pll": {
            "m_range": (1, 1),
            "n_values": [3, 4, 6, 8, 12, 16, 24, 32, 48],
            "r_values": [2, 3, 4],
            "in_min_mhz": 2,
            "in_max_mhz": 24,
            "vco_min_mhz": 0,
            "vco_max_mhz": 96,
            "out_max_mhz": 32,
            "prefer_low_vco": True,
        },
    },
    "STM32L422TB": {
        "define": "STM32L422xx",
        "family": "stm32l4xx",
        "core": "cortex-m4",
        "cpu_flag": "-mcpu=cortex-m4",
        "fpu": "fpv4-sp-d16",
        "max_sysclk_mhz": 80,
        "pll": {
            "m_range": (1, 8),
            "n_range": (8, 86),
            "r_values": [2, 4, 6, 8],
            "vco_min_mhz": 64,
            "vco_max_mhz": 344,
        },
    },
    "STM32WBA55HGF6": {
        "define": "STM32WBA55xx",
        "family": "stm32wbaxx",
        "core": "cortex-m33",
        "cpu_flag": "-mcpu=cortex-m33",
        "fpu": "fpv5-sp-d16",
        "max_sysclk_mhz": 100,
        # RM0493 §12.4.3 / §12.8.7: ref_ck 4-16 MHz, VCO 128-544 MHz, pll1rclk
        # <= 100 MHz, and a PLL1R division factor is forbidden when
        # VCO / (2 x TRUNC(R / 2)) exceeds that maximum (so R = 3 at a 300 MHz
        # VCO, what the solver used to pick for 100 MHz, is out). Lowest VCO
        # first: the RM recommends it, and it is what CubeWBA uses (100 MHz =
        # 32 / 4 x 25 / 2, VCO 200 MHz).
        "pll": {
            "m_range": (1, 8),
            "n_range": (4, 512),
            "r_values": [1, 2, 3, 4, 5, 6, 7, 8],
            "in_min_mhz": 4,
            "in_max_mhz": 16,
            "vco_min_mhz": 128,
            "vco_max_mhz": 544,
            "out_max_mhz": 100,
            "r_trunc_rule": True,
            "prefer_low_vco": True,
        },
    },
    "STM32H523HE": {
        "define": "STM32H523xx",
        "family": "stm32h5xx",
        "core": "cortex-m33",
        "cpu_flag": "-mcpu=cortex-m33",
        "fpu": "fpv5-sp-d16",
        "max_sysclk_mhz": 250,
        "pll": {
            "m_range": (1, 63),
            "n_range": (4, 512),
            "r_values": [1, 2, 3, 4, 5, 6, 7, 128],
            "vco_min_mhz": 150,
            "vco_max_mhz": 836,
        },
    },
}


def solve_pll(source_mhz, target_mhz, pll_spec):
    """Find PLL M/N/R values to get from source_mhz to target_mhz.

    Returns (m, n, r) tuple or None if no valid combination exists.
    By default prefers the VCO closest to the middle of the valid range
    (best jitter performance) and lowest M (widest PLL bandwidth); a spec
    with "prefer_low_vco" takes the lowest legal VCO instead (lowest power).

    Optional spec keys (defaults keep the L4 / H5 behavior unchanged):
      n_values       explicit multiplier set instead of n_range (L0 PLLMUL)
      in_min_mhz /   PLL input (source / M) limits; default 1-16 MHz
        in_max_mhz
      out_max_mhz    ceiling on the PLL output
      r_trunc_rule   WBA rule: R is forbidden when VCO / (2 x (R // 2))
                     exceeds out_max_mhz (RM0493 §12.8.7)
    """
    m_min, m_max = pll_spec["m_range"]
    n_values = pll_spec.get("n_values")
    if n_values is None:
        n_min, n_max = pll_spec["n_range"]
    r_values = pll_spec["r_values"]
    vco_min = pll_spec["vco_min_mhz"]
    vco_max = pll_spec["vco_max_mhz"]
    vco_mid = (vco_min + vco_max) / 2
    in_min = pll_spec.get("in_min_mhz", 1)
    in_max = pll_spec.get("in_max_mhz", 16)
    out_max = pll_spec.get("out_max_mhz")
    prefer_low = pll_spec.get("prefer_low_vco", False)

    best = None
    best_score = float("inf")

    for m in range(m_min, m_max + 1):
        pll_input = source_mhz / m
        if pll_input < in_min or pll_input > in_max:
            continue

        for r in r_values:
            # target = source / m * n / r  →  n = target * m * r / source
            n_exact = target_mhz * m * r / source_mhz
            n = round(n_exact)

            if n_values is not None:
                if n not in n_values:
                    continue
            elif n < n_min or n > n_max:
                continue

            # Check we hit the target exactly
            actual = source_mhz / m * n / r
            if abs(actual - target_mhz) > 0.01:
                continue
            if out_max is not None and actual > out_max + 0.01:
                continue

            # Check VCO range
            vco = source_mhz / m * n
            if vco < vco_min or vco > vco_max:
                continue

            if pll_spec.get("r_trunc_rule"):
                half = 2 * (r // 2)
                if half == 0 or vco / half > out_max + 0.01:
                    continue

            if prefer_low:
                score = vco + m * 0.01
            else:
                # Prefer VCO near middle of range, then lowest M
                score = abs(vco - vco_mid) + m * 0.01
            if score < best_score:
                best = (m, n, r)
                best_score = score

    return best


def parse_gpio(function_str):
    """Parse a digital function name like 'A7' or 'B12' into (port, pin) or (None, None)."""
    m = re.match(r'^P?([A-H])(\d+)$', function_str)
    if m:
        return m.group(1), int(m.group(2))
    return None, None


def extract_pad_gpio(pad):
    """Extract the default GPIO port/pin from a pad's functions list."""
    for func in pad.get("functions", []):
        if func.get("type") == "digital":
            port, pin = parse_gpio(func["function"])
            if port is not None:
                return port, pin
    return None, None


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


def extract_led_info(tile):
    """Extract LED pin info from application_notes."""
    for note in tile.get("application_notes", []):
        text = note.get("details", "") + " " + note.get("heading", "")
        if "LED" in text.upper():
            m = re.search(r'P([A-H])(\d+)', note.get("details", ""))
            if m:
                port = m.group(1)
                pin = int(m.group(2))
                active_high = "active-high" in note.get("details", "").lower()
                return {"port": port, "pin": pin, "active_high": active_high}
    return None


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


def extract_timer_channels(pad):
    """Extract timer PWM channel info from a pad's timer functions.

    Looks for functions like 'TIM2.3', 'TIM1.1', etc. (ignoring
    complementary outputs like 'TIM1.2N', ETR, BKIN, and LPTIM).
    Returns a list of dicts: [{"timer": "TIM2", "channel": 3, "af": 1}, ...].
    """
    channels = []
    for func in pad.get("functions", []):
        if func.get("type") != "timer":
            continue
        fname = func["function"]
        # Match TIMx.y where y is a plain digit (no N suffix = not complementary)
        m = re.match(r'(TIM\d+)\.(\d+)$', fname)
        if m and "af" in func:
            channels.append({
                "timer": m.group(1),
                "channel": int(m.group(2)),
                "af": func["af"],
            })
    return channels


def build_pad_map(pads):
    """Build a list of pad info dicts for template rendering."""
    pad_map = []
    for pad in pads:
        port, pin = extract_pad_gpio(pad)

        # Collect all available functions
        all_functions = []
        af_functions = []
        for func in pad.get("functions", []):
            all_functions.append(func["function"])
            if "af" in func:
                af_functions.append({
                    "function": func["function"],
                    "type": func["type"],
                    "af": func["af"],
                })

        # Extract ADC channel if present
        adc_channel = extract_adc_channel(pad)

        # Extract timer PWM channels if present
        timer_channels = extract_timer_channels(pad)

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

        # GPIO.OUT and GPIO.IN are synthetic — always valid on GPIO pads
        if assigned_func in ("GPIO.OUT", "GPIO.IN"):
            if pad_info["port"] is None:
                errors.append(f"Pad {pad_num}: cannot use {assigned_func} on a non-GPIO pad")
            continue

        # Check if the assigned function exists on this pad
        if assigned_func not in pad_info["all_functions"]:
            available = [f for f in pad_info["all_functions"]
                         if f not in ("GND", "V+", "NRST")]
            errors.append(
                f"Pad {pad_num}: '{assigned_func}' is not available. "
                f"Options: {', '.join(available)}"
            )

    # Core.ST.L4: USB is always on (CDC console, 1200-baud DFU touch, serial
    # update), so its D+/D- pads can't be anything else. Assigning SPI1 /
    # USART1 / TIM1 / GPIO there silently fought the USB peripheral for PA11 /
    # PA12 (pads 6/7 on L4.1, 16/17 on L4.2).
    if mcu and mcu.get("define") == "STM32L422xx":
        for pad_num, assigned_func in pins.items():
            info = pad_lookup.get(pad_num)
            if not info:
                continue
            usb_fn = next((f for f in info["all_functions"] if f in ("USB.DP", "USB.DM")), None)
            if usb_fn and not str(assigned_func).startswith("USB."):
                errors.append(
                    f"Pad {pad_num}: '{assigned_func}' can't be used — pad {pad_num} is "
                    f"{'USB D+' if usb_fn == 'USB.DP' else 'USB D-'} (P{info['port']}{info['pin']}), "
                    f"and USB is always on on this Core (USB serial, DFU and serial "
                    f"update). Use another pad."
                )

    # On-tile pull-ups switched by a GPIO (tile config "pullups", a DB-owned
    # multiselect; Core.ST.L4.1 has pad 4 via PA9 and pad 5 via PC15).
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
        usb_capable = {"STM32L422xx", "STM32H523xx"}
        if mcu_define not in usb_capable:
            warnings.append(
                f"Bootloader '{bootloader}' requires USB — "
                f"{mcu_define} does not support USB CDC"
            )

    return warnings, errors


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
            entry["default"] = gpio_cfg.get("default", None)

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


# Exact MSI frequencies. The L0's MSI ranges are powers of two of 32.768 kHz
# (RM0377 §7.2.3): "1 MHz" is 1.048576 MHz, "2 MHz" is 2.097152 MHz. SYSCLK_HZ
# used to say 1000000 / 2000000, so SysTick (and every baud rate) ran ~5% fast
# at those levels. The L4's MSI ranges are whole MHz (RM0394 §6.2.3).
L0_MSI_HZ = {1: 1048576, 2: 2097152, 4: 4194304}

# WBA hclk5 (radio AHB) ceiling in range 1 and the HPRE5 dividers
# (RM0493 Table 99, §12.8.51).
WBA_HCLK5_MAX_MHZ = 32
WBA_HPRE5_DIVS = (1, 2, 3, 4, 6)


def _project_uses_i2c(config):
    pads = config.get("pads", config.get("pins", {}))
    return any(re.match(r'^I2C\d+\.(CLK|DAT)$', str(f)) for f in pads.values())


def build_clock_config(config, tile, mcu):
    """Build resolved clock configuration from a performance level.

    Accepts a performance level string ("low", "medium", "high", "max")
    which is resolved from the tile JSON's config.clock select knob.

    Auto-calculates PLL M/N/R if the target frequency requires it, and picks
    the voltage range, flash wait states and bus dividers each part's
    reference manual requires for it (see the per-family notes below).
    """
    sources, configurations, knob_default = resolve_clock_block(tile)
    level = config.get("clock", knob_default)
    # "default" is an alias for the tile's schema default (config.clock.default).
    if level == "default":
        level = knob_default

    # Resolve performance level to clock config
    configs = {c["name"]: c for c in configurations}
    if level not in configs:
        available = ", ".join(sorted(configs.keys())) if configs else "(none defined)"
        eprint(f"  ERROR: Clock level '{level}' not available. Options: {available}")
        sys.exit(1)

    resolved = configs[level]
    source = resolved["source"]
    target_mhz = resolved["sysclk_mhz"]
    define = mcu["define"]
    part = tile['components'][0]['part']

    # ---- Per-family level adjustments (build notices go to stderr) ----
    if define == "STM32L422xx" and source == "msi" and target_mhz < 10:
        # RM0394 §46.4: the USB needs an APB clock of at least 10 MHz, APB
        # can't run faster than HCLK, and every L4 build turns USB on. The
        # next MSI range up is 16 MHz (RM0394 §6.2.3).
        eprint(f"  NOTE: clock '{level}' runs MSI at 16 MHz, not {target_mhz} MHz, on the "
               f"{part}: USB is always on and needs APB >= 10 MHz (RM0394 §46.4). "
               f"'low' and 'medium' are the same clock on this Core.")
        target_mhz = 16

    ble_on = bool((config.get("ble") or {}).get("enabled"))
    if define == "STM32WBA55xx" and ble_on and target_mhz <= 16:
        eprint(f"  ERROR: BLE needs clock medium or higher. Clock '{level}' runs HSI16 "
               f"in voltage range 2, and the 2.4 GHz radio needs range 1 with an "
               f"undivided hclk5 of 16-32 MHz (RM0493 §12.4.6). Set \"clock\": \"medium\".")
        sys.exit(1)

    if resolved.get("lp_run"):
        # RM0377 §6.3.4: Low-power run needs SYSCLK <= MSI range 1 (~131 kHz)
        # and LPSDSR. At this level's MSI frequency it is out of spec, so it
        # is ignored; the level runs in voltage range 3 instead (below).
        eprint(f"  NOTE: clock '{level}' asks for Low-power run, which the {part} allows "
               f"only at <= 131 kHz (RM0377 §6.3.4). Running it as normal Run mode "
               f"in the lowest voltage range that fits.")

    print(f"  Clock: {level} → {source} @ {target_mhz}MHz")

    # Find frequency for the selected source.
    # For MSI, the tile JSON records the reset-default frequency (4MHz) but the
    # oscillator can be tuned to any range value — treat target_mhz as the MSI freq.
    source_mhz = target_mhz if source == "msi" else 16
    for src in sources:
        if src["type"] == source and source != "msi":
            source_mhz = src["frequency_mhz"]
            break

    pll_config = None

    # Auto-calculate PLL if needed (MSI without PLL: target == source, skip)
    if target_mhz != source_mhz and pll_config is None:
        max_mhz = mcu.get("max_sysclk_mhz", 80)
        if target_mhz > max_mhz:
            eprint(f"  ERROR: sysclk_mhz={target_mhz} exceeds max {max_mhz}MHz for {part}")
            sys.exit(1)

        pll_spec = mcu.get("pll")
        if pll_spec is None:
            eprint(f"  ERROR: PLL not available on {part}, cannot reach {target_mhz}MHz from {source}={source_mhz}MHz")
            sys.exit(1)

        result = solve_pll(source_mhz, target_mhz, pll_spec)
        if result is None:
            eprint(f"  ERROR: No valid PLL configuration found for {source_mhz}MHz → {target_mhz}MHz")
            sys.exit(1)

        m, n, r = result
        pll_config = {"m": m, "n": n, "r": r}
        vco = source_mhz / m * n
        print(f"  PLL: {source_mhz}MHz ÷{m} ×{n} ÷{r} = {target_mhz}MHz (VCO={vco:.0f}MHz)")

    # Exact SYSCLK in Hz
    if define == "STM32L011xx" and source == "msi":
        if target_mhz not in L0_MSI_HZ:
            eprint(f"  ERROR: No L0 MSI range for {target_mhz}MHz")
            sys.exit(1)
        sysclk_hz = L0_MSI_HZ[target_mhz]
    else:
        sysclk_hz = int(target_mhz * 1000000)
    sysclk_mhz_ceil = -(-sysclk_hz // 1000000)

    # Voltage range (vos_range) — L0 and WBA; H5 keeps needs_vos/vos_value.
    #  L0 (RM0377 §6.1.4, Tables 14/33/43): range 1 up to 32 MHz, range 2 up
    #  to 16 MHz, range 3 up to 4.2 MHz with no HSI16 and no flash/EEPROM
    #  program or erase. The L0 boots in range 2 and nothing used to change
    #  it, so 16 MHz ran with 0 WS and 32 MHz ran in range 2 (limit 16 MHz).
    #  "low" takes range 3 unless HSI16 has to run (I2C kernel clock);
    #  "medium" stays in range 2 (the reset range); HSI16/PLL take range 1.
    #  WBA (RM0493 Table 99): HSI16 at 16 MHz is range 2 (1 WS, hclk5 / 2);
    #  anything faster, and the radio, is range 1.
    needs_vos = False
    vos_value = 1  # default for WBA55
    vos_range = None
    hsi16_kernel = False
    hpre5 = None
    if define == "STM32L011xx":
        hsi16_kernel = _project_uses_i2c(config)
        if source == "msi":
            vos_range = 3 if (level == "low" and sysclk_hz <= 4200000) else 2
            if vos_range == 3 and hsi16_kernel:
                vos_range = 2
                eprint(f"  NOTE: clock '{level}': I2C1 runs from HSI16 (~100 µA while running) "
                       f"is set up), which voltage range 3 can't run (RM0377 Table 43), so "
                       f"this build uses range 2.")
        else:
            vos_range = 1
    elif define == "STM32WBA55xx":
        vos_range = 2 if (source == "hsi16" and target_mhz <= 16) else 1
        needs_vos = vos_range == 1
        vos_value = 1
        if pll_config:
            hpre5 = next(d for d in WBA_HPRE5_DIVS if target_mhz / d <= WBA_HCLK5_MAX_MHZ)
    elif define == "STM32H523xx" and target_mhz > 32:
        needs_vos = True
        # H5 VOS register encoding (inverted from scale number):
        # VOS=00(0)→Scale3(32MHz), 01(1)→Scale2(100MHz), 10(2)→Scale1(150MHz), 11(3)→Scale0(250MHz)
        if target_mhz <= 100:
            vos_value = 1   # VOS=01, Scale 2
        elif target_mhz <= 150:
            vos_value = 2   # VOS=10, Scale 1
        else:
            vos_value = 3   # VOS=11, Scale 0 (boost)

    # MSI range define (STM32L0/L4)
    _msi_range_map = {
        1: "LL_RCC_MSI_RANGE_1MHZ",  2: "LL_RCC_MSI_RANGE_2MHZ",
        4: "LL_RCC_MSI_RANGE_4MHZ",  8: "LL_RCC_MSI_RANGE_8MHZ",
        16: "LL_RCC_MSI_RANGE_16MHZ", 24: "LL_RCC_MSI_RANGE_24MHZ",
        32: "LL_RCC_MSI_RANGE_32MHZ", 48: "LL_RCC_MSI_RANGE_48MHZ",
    }
    msi_range = _msi_range_map.get(target_mhz) if source == "msi" else None
    if source == "msi" and msi_range is None:
        eprint(f"  ERROR: No MSI range constant for {target_mhz}MHz")
        sys.exit(1)

    return {
        "level": level,
        "source": source,
        "source_mhz": source_mhz,
        "sysclk_mhz": target_mhz,
        "sysclk_hz": sysclk_hz,
        "sysclk_mhz_ceil": sysclk_mhz_ceil,
        "pll": pll_config,
        "msi_range": msi_range,
        "lp_run": False,
        "ahb_div": 1,
        "apb1_div": 1,
        "apb2_div": 1,
        "needs_vos": needs_vos,
        "vos_value": vos_value,
        "vos_range": vos_range,
        "hsi16_kernel": hsi16_kernel,
        "hpre5": hpre5,
    }


# ---- I2C bus clock mapping per family ----

# Maps (family_define, bus_number) -> (clk_enable_func, clk_mask_define)
I2C_CLK_MAP = {
    # L0: I2C1 on APB1
    ("STM32L011xx", 1): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C1"),
    # L4: I2C1/I2C3 on APB1
    ("STM32L422xx", 1): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C1"),
    ("STM32L422xx", 3): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C3"),
    # WBA: I2C1 on APB1, I2C3 on APB7
    ("STM32WBA55xx", 1): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C1"),
    ("STM32WBA55xx", 3): ("ll_rcc_apb7_clk_enable", "LL_APB7_I2C3"),
    # H5: I2C1/I2C2 on APB1, I2C3 on APB3
    ("STM32H523xx", 1): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C1"),
    ("STM32H523xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_I2C2"),
    ("STM32H523xx", 3): ("ll_rcc_apb3_clk_enable", "LL_APB3_I2C3"),
}

# Maps (speed_hz, kernel_clk_mhz) -> timing constant define
# Speeds: 100kHz (Standard), 400kHz (Fast Mode), 1MHz (Fast Mode Plus)
# 1MHz entries only exist for kernel clocks >= 48MHz (16/32MHz don't have
# sufficient timing margin and no LL_I2C_TIMING_1M_16/32MHZ constants exist).
I2C_TIMING_MAP = {
    # Standard mode (100kHz) — minimum kernel clock: 1 MHz
    (100000,   1): "LL_I2C_TIMING_100K_1MHZ",
    (100000,   2): "LL_I2C_TIMING_100K_2MHZ",
    (100000,   4): "LL_I2C_TIMING_100K_4MHZ",
    (100000,   8): "LL_I2C_TIMING_100K_8MHZ",
    (100000,  16): "LL_I2C_TIMING_100K_16MHZ",
    (100000,  32): "LL_I2C_TIMING_100K_32MHZ",
    (100000,  48): "LL_I2C_TIMING_100K_48MHZ",
    (100000,  64): "LL_I2C_TIMING_100K_64MHZ",
    (100000,  80): "LL_I2C_TIMING_100K_80MHZ",
    (100000, 128): "LL_I2C_TIMING_100K_128MHZ",
    (100000, 144): "LL_I2C_TIMING_100K_144MHZ",
    (100000, 240): "LL_I2C_TIMING_100K_240MHZ",
    (100000, 248): "LL_I2C_TIMING_100K_248MHZ",
    # Fast mode (400kHz) — minimum kernel clock: 4 MHz
    (400000,   4): "LL_I2C_TIMING_400K_4MHZ",
    (400000,   8): "LL_I2C_TIMING_400K_8MHZ",
    (400000,  16): "LL_I2C_TIMING_400K_16MHZ",
    (400000,  32): "LL_I2C_TIMING_400K_32MHZ",
    (400000,  48): "LL_I2C_TIMING_400K_48MHZ",
    (400000,  64): "LL_I2C_TIMING_400K_64MHZ",
    (400000,  80): "LL_I2C_TIMING_400K_80MHZ",
    (400000, 128): "LL_I2C_TIMING_400K_128MHZ",
    (400000, 144): "LL_I2C_TIMING_400K_144MHZ",
    (400000, 240): "LL_I2C_TIMING_400K_240MHZ",
    (400000, 248): "LL_I2C_TIMING_400K_248MHZ",
    # Fast mode plus (1MHz) — minimum kernel clock: 16 MHz
    (1000000,  16): "LL_I2C_TIMING_1M_16MHZ",
    (1000000,  32): "LL_I2C_TIMING_1M_32MHZ",
    (1000000,  48): "LL_I2C_TIMING_1M_48MHZ",
    (1000000,  64): "LL_I2C_TIMING_1M_64MHZ",
    (1000000,  80): "LL_I2C_TIMING_1M_80MHZ",
    (1000000, 128): "LL_I2C_TIMING_1M_128MHZ",
    (1000000, 144): "LL_I2C_TIMING_1M_144MHZ",
    (1000000, 240): "LL_I2C_TIMING_1M_240MHZ",
    (1000000, 248): "LL_I2C_TIMING_1M_248MHZ",
}

# Minimum kernel clock (MHz) for each I2C speed (from CubeMX — below this, timing is not achievable)
I2C_MIN_CLOCK = {100000: 1, 400000: 4, 1000000: 16}


# ---- UART (USART) config ----
#
# Scope for the first pilot: USART1/2/3 only. LPUART has separate clock
# mux rules (wake-from-Stop, HSI16 vs PCLK) that deserve their own pass.
# Once that lands, add an LPUART_CLK_MAP alongside this one.

USART_CLK_MAP = {
    # (family_define, usart_num) → (clk_enable_func, clk_bitmask, pclk_symbol)
    ("STM32L011xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_USART2", "PCLK1_HZ"),

    ("STM32L422xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_USART1", "PCLK2_HZ"),
    ("STM32L422xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_USART2", "PCLK1_HZ"),

    ("STM32WBA55xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_USART1", "PCLK2_HZ"),
    ("STM32WBA55xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_USART2", "PCLK1_HZ"),

    ("STM32H523xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_USART1", "PCLK2_HZ"),
    ("STM32H523xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_USART2", "PCLK1_HZ"),
    ("STM32H523xx", 3): ("ll_rcc_apb1_clk_enable", "LL_APB1_USART3", "PCLK1_HZ"),
}


def build_usart_config(config, mcu):
    """Detect USART peripherals from pad assignments and build a config list.

    Scans `config.pads` for functions matching `USART<n>.(TX|RX)` and returns
    one dict per used peripheral. Per-peripheral baud (and any future
    parameters like parity / word-length) come from `interfaces.USART<n>`
    in config.json; default baud is 115200.

    Returns [] when no USART pad is configured. LPUART is intentionally
    out of scope for this pilot — its clock mux deserves its own pass.
    """
    family_define = mcu["define"]
    iface_cfg = config.get("interfaces", {})
    pads = config.get("pads", config.get("pins", {}))

    usart_numbers = set()
    for _, func in pads.items():
        m = re.match(r'^USART(\d+)\.(TX|RX)$', func)
        if m:
            usart_numbers.add(int(m.group(1)))

    if not usart_numbers:
        return []

    buses = []
    for num in sorted(usart_numbers):
        name = f"USART{num}"
        cfg = iface_cfg.get(name, {})
        baud = cfg.get("baud", 115200)
        rx_int = 1 if cfg.get("rx_interrupt", False) else 0

        key = (family_define, num)
        clk_info = USART_CLK_MAP.get(key)
        if clk_info is None:
            eprint(f"  ERROR: {name} clock mapping not defined for {family_define}")
            sys.exit(1)
        clk_func, clk_mask, pclk_symbol = clk_info

        buses.append({
            "num": num,
            "instance": name,
            "handle": f"core_usart{num}",
            "clk_func": clk_func,
            "clk_mask": clk_mask,
            "pclk_symbol": pclk_symbol,
            "baud": baud,
            "rx_interrupt": rx_int,
        })

    return buses


# ---- Timer / PWM config ----
#
# Scans pads for TIM<n>.<ch> assignments, groups by timer, and emits one
# handle per used timer with an auto-init in core_pads_init(). Also emits
# a pad→timer lookup function so the pad-oriented DSL wrappers in
# core_pwm.h can dispatch without the caller touching a handle.

TIMER_CLK_MAP = {
    # (family_define, timer_num) → (clk_func, clk_mask)
    ("STM32L011xx", 2):  ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM2"),
    ("STM32L011xx", 21): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM21"),

    ("STM32L422xx", 1):  ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM1"),
    ("STM32L422xx", 2):  ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM2"),
    ("STM32L422xx", 15): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM15"),
    ("STM32L422xx", 16): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM16"),

    ("STM32WBA55xx", 1):  ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM1"),
    ("STM32WBA55xx", 2):  ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM2"),
    ("STM32WBA55xx", 3):  ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM3"),
    ("STM32WBA55xx", 16): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM16"),
    ("STM32WBA55xx", 17): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM17"),

    ("STM32H523xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_TIM1"),
    ("STM32H523xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM2"),
    ("STM32H523xx", 3): ("ll_rcc_apb1_clk_enable", "LL_APB1_TIM3"),
}


def build_pwm_config(config, mcu):
    """Detect timer PWM usage from pad assignments and build a config list.

    Scans `project.pads` for functions matching `TIM<n>.<ch>` and returns
    one dict per used timer peripheral. All channels on the same timer
    share a frequency (hardware constraint); frequency comes from
    `interfaces.TIM<n>.freq` in config.json if specified, otherwise
    defaults to 1 kHz — a sensible starting point for LEDs and motors.
    """
    family_define = mcu["define"]
    iface_cfg = config.get("interfaces", {})
    pads = config.get("pads", config.get("pins", {}))

    # Collect pads per timer
    timer_pads = {}
    for pad_num, func in pads.items():
        m = re.match(r'^TIM(\d+)\.\d+$', func)
        if m:
            num = int(m.group(1))
            timer_pads.setdefault(num, []).append(pad_num)

    if not timer_pads:
        return []

    timers = []
    for num in sorted(timer_pads.keys()):
        name = f"TIM{num}"
        tcfg = iface_cfg.get(name, {})
        freq = tcfg.get("freq", 1000)

        key = (family_define, num)
        clk_info = TIMER_CLK_MAP.get(key)
        if clk_info is None:
            eprint(f"  ERROR: {name} clock mapping not defined for {family_define}")
            sys.exit(1)
        clk_func, clk_mask = clk_info

        timers.append({
            "num": num,
            "instance": name,
            "handle": f"core_tim{num}",
            "clk_func": clk_func,
            "clk_mask": clk_mask,
            "freq": freq,
            # Sort pads numerically so the generated lookup-switch is readable.
            "pads": sorted(timer_pads[num], key=int),
        })

    return timers


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


def build_i2c_config(config, mcu, clock_config):
    """Detect I2C buses from pin assignments and build I2C config list.

    Scans pads for patterns like 'I2C1.CLK', 'I2C3.DAT' and returns a list
    of dicts with bus configuration for template rendering.

    Per-bus speed and pullup settings come from the 'interfaces' section
    of config.json.  Defaults: speed=400000 (400kHz), pullups=true.

    On WBA55, I2C kernel clock is routed to HSI16 (16MHz) so timing is
    always computed for 16MHz regardless of SYSCLK.
    """
    family_define = mcu["define"]
    sysclk_mhz = clock_config["sysclk_mhz"]
    iface_cfg = config.get("interfaces", {})

    # On WBA55 and L011, the I2C kernel clock is routed to HSI16 (16MHz) regardless
    # of SYSCLK (the L0 so 400 kHz works at the 1-2 MHz MSI levels too).
    # H523 uses SYSCLK as I2C kernel clock — TIMINGR constants now exist for 144/240MHz.
    _hsi16_i2c_parts = {"STM32WBA55xx", "STM32L011xx"}
    i2c_clk_mhz = 16 if family_define in _hsi16_i2c_parts else sysclk_mhz

    # Detect which I2C buses are referenced in pad assignments
    bus_numbers = set()
    pads = config.get("pads", config.get("pins", {}))
    for pad_num, func in pads.items():
        m = re.match(r'^I2C(\d+)\.(CLK|DAT)$', func)
        if m:
            bus_numbers.add(int(m.group(1)))

    if not bus_numbers:
        return []

    i2c_buses = []
    for bus_num in sorted(bus_numbers):
        bus_name = f"I2C{bus_num}"
        bus_cfg = iface_cfg.get(bus_name, {})
        speed = bus_cfg.get("speed", 400000)
        pullups = bus_cfg.get("pullups", True)

        # Validate speed
        if speed not in (100000, 400000, 1000000):
            eprint(f"  ERROR: I2C{bus_num} speed {speed} not supported (use 100000, 400000, or 1000000)")
            sys.exit(1)

        # Check minimum clock for requested speed
        speed_label = {100000: "100kHz", 400000: "400kHz", 1000000: "1MHz"}[speed]
        min_clk = I2C_MIN_CLOCK.get(speed, 1)
        if i2c_clk_mhz < min_clk:
            eprint(f"  ERROR: I2C{bus_num} {speed_label} requires at least {min_clk}MHz kernel clock, but this config has {i2c_clk_mhz}MHz.")
            print(f"         Use a higher clock level or a lower I2C speed.")
            sys.exit(1)

        # Look up timing constant for this speed + I2C kernel clock combo
        timing = I2C_TIMING_MAP.get((speed, i2c_clk_mhz))
        if timing is None:
            eprint(f"  ERROR: I2C{bus_num} {speed_label} is not supported with a {i2c_clk_mhz}MHz I2C kernel clock.")
            if family_define == "STM32WBA55xx" and speed == 1000000:
                print(f"         Core.ST.W5 routes I2C to HSI16 (16MHz); maximum supported speed is 400kHz.")
            else:
                print(f"         No pre-computed TIMINGR for {speed_label} @ {i2c_clk_mhz}MHz — add it to I2C_TIMING_MAP or use a lower speed.")
            sys.exit(1)

        key = (family_define, bus_num)
        clk_info = I2C_CLK_MAP.get(key)
        if clk_info is None:
            eprint(f"  ERROR: I2C{bus_num} clock enable not defined for {family_define}")
            sys.exit(1)

        clk_func, clk_mask = clk_info
        i2c_buses.append({
            "num": bus_num,
            "instance": bus_name,
            "handle": f"core_i2c{bus_num}",
            "clk_func": clk_func,
            "clk_mask": clk_mask,
            "timing": timing,
            "pullups": pullups,
            "speed": speed,
        })

    return i2c_buses


# ---- SPI bus clock mapping per family ----

SPI_CLK_MAP = {
    # L0: SPI1 on APB2
    ("STM32L011xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_SPI1"),
    # L4: SPI1 on APB2
    ("STM32L422xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_SPI1"),
    # WBA: SPI1 on APB2, SPI3 on APB7
    ("STM32WBA55xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_SPI1"),
    ("STM32WBA55xx", 3): ("ll_rcc_apb7_clk_enable", "LL_APB7_SPI3"),
    # H5: SPI1 on APB2, SPI3 on APB3
    ("STM32H523xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_SPI1"),
    ("STM32H523xx", 3): ("ll_rcc_apb3_clk_enable", "LL_APB3_SPI3"),
}

SPI_PRESCALER_MAP = {
    2:   "LL_SPI_PRESCALER_2",   4:   "LL_SPI_PRESCALER_4",
    8:   "LL_SPI_PRESCALER_8",   16:  "LL_SPI_PRESCALER_16",
    32:  "LL_SPI_PRESCALER_32",  64:  "LL_SPI_PRESCALER_64",
    128: "LL_SPI_PRESCALER_128", 256: "LL_SPI_PRESCALER_256",
}


# Datasheet SCK ceilings for an SPI master, MHz: (VDD >= 2.7 V, below 2.7 V).
# STM32L422 DS Table 80 (master receiver / full duplex, voltage range 1);
# STM32WBA5x DS14127 Table 94 (master receiver). The Cores run the SPI kernel
# at SYSCLK (APB dividers 1), so the fastest setting is SYSCLK / 2.
SPI_MAX_SCK_MHZ = {
    "STM32L422xx": (40, 16),
    "STM32WBA55xx": (50, 33),
}


def build_spi_config(config, mcu, pad_map, clock_config=None):
    """Detect SPI buses from pin assignments and build SPI config list.

    Scans pads for patterns like 'SPI1.CLK', 'SPI1.MOSI', 'SPI1.MISO', 'SPI1.CS'
    and returns a list of dicts with bus configuration for template rendering.

    Per-bus mode, prescaler and bit order come from the 'interfaces' section
    of config.json.  Defaults: mode=0 (CPOL=0/CPHA=0), prescaler=8 (÷8),
    bit_order="msb". With clock_config, a prescaler that puts SCK over the
    part's datasheet ceiling is reported (a NOTE when only the low-VDD
    ceiling is exceeded, an ERROR above the 2.7-3.6 V one).

    SPI1.CS pads are configured as GPIO output (software CS management via
    hal_spi_set_cs) rather than the hardware NSS alternate function.
    """
    family_define = mcu["define"]
    iface_cfg = config.get("interfaces", {})
    pads = config.get("pads", config.get("pins", {}))
    pad_lookup = {p["number"]: p for p in pad_map}

    # Detect SPI buses and CS pad assignments from pad assignments
    bus_numbers = set()
    cs_pads = {}   # bus_num -> pad_num string
    for pad_num, func in pads.items():
        m = re.match(r'^SPI(\d+)\.(CLK|MOSI|MISO|CS)$', func)
        if m:
            bus_num = int(m.group(1))
            bus_numbers.add(bus_num)
            if m.group(2) == "CS":
                cs_pads[bus_num] = pad_num

    if not bus_numbers:
        return []

    spi_buses = []
    for bus_num in sorted(bus_numbers):
        bus_name = f"SPI{bus_num}"
        bus_cfg = iface_cfg.get(bus_name, {})
        mode = bus_cfg.get("mode", 0)
        prescaler = bus_cfg.get("prescaler", 8)

        if mode not in (0, 1, 2, 3):
            eprint(f"  ERROR: SPI{bus_num} mode {mode} not valid (use 0-3)")
            sys.exit(1)

        prescaler_define = SPI_PRESCALER_MAP.get(prescaler)
        if prescaler_define is None:
            eprint(f"  ERROR: SPI{bus_num} prescaler {prescaler} not valid "
                  f"(use 2, 4, 8, 16, 32, 64, 128, or 256)")
            sys.exit(1)

        bit_order = bus_cfg.get("bit_order", "msb")
        if bit_order not in ("msb", "lsb"):
            eprint(f"  ERROR: SPI{bus_num} bit_order '{bit_order}' not valid (use \"msb\" or \"lsb\")")
            sys.exit(1)

        limits = SPI_MAX_SCK_MHZ.get(family_define)
        if clock_config and limits:
            kernel_hz = clock_config["sysclk_hz"] // clock_config.get("apb2_div", 1)
            sck_mhz = kernel_hz / prescaler / 1e6
            hi, lo = limits
            if sck_mhz > hi:
                eprint(f"  ERROR: SPI{bus_num} SCK {sck_mhz:.1f} MHz (SYSCLK / {prescaler}) is over the "
                       f"{mcu['define']} master limit of {hi} MHz. Use a larger prescaler.")
                sys.exit(1)
            if sck_mhz > lo:
                eprint(f"  NOTE: SPI{bus_num} SCK {sck_mhz:.1f} MHz is legal at VDD 2.7-3.6 V "
                       f"(up to {hi} MHz) but over the {lo} MHz limit below 2.7 V.")

        key = (family_define, bus_num)
        clk_info = SPI_CLK_MAP.get(key)
        if clk_info is None:
            eprint(f"  ERROR: SPI{bus_num} clock enable not defined for {family_define}")
            sys.exit(1)

        clk_func, clk_mask = clk_info
        cpol = mode >> 1   # CPOL: bit 1 of mode
        cpha = mode & 1    # CPHA: bit 0 of mode
        cpol_define = "LL_SPI_CPOL_HIGH" if cpol else "LL_SPI_CPOL_LOW"
        cpha_define = "LL_SPI_CPHA_2EDGE" if cpha else "LL_SPI_CPHA_1EDGE"

        # Resolve CS pad GPIO port/pin for hal_spi_set_cs()
        cs_pad_num = cs_pads.get(bus_num)
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
            "clk_func": clk_func,
            "clk_mask": clk_mask,
            "prescaler": prescaler_define,
            "cpol": cpol_define,
            "cpha": cpha_define,
            "cs_pad": cs_pad_num,
            "cs_port": cs_port,
            "cs_pin": cs_pin,
            "cs_active_low": cs_active_low,
            "lsb_first": bit_order == "lsb",
        })

    return spi_buses


# ---- Tile peripheral driver mapping ----

TILE_DRIVER_MAP = {
    "Sense.CAM.P": {"header": "tile_sense_cam_p.h",  "source": "tile_sense_cam_p",  "prefix": "tile_sense_cam_p"},
    "Sense.I.9":   {"header": "tile_sense_i_9.h",    "source": "tile_sense_i_9",    "prefix": "tile_sense_i_9", "extra_sources": ["tile_sense_i_9_dmp3"]},
    "Sense.I.6P8": {"header": "tile_sense_i_6p8.h",  "source": "tile_sense_i_6p8",  "prefix": "tile_sense_i_6p8"},
    "Sense.I.6P6": {"header": "tile_sense_i_6p6.h",  "source": "tile_sense_i_6p6",  "prefix": "tile_sense_i_6p6"},
    "Sense.ADC.6": {"header": "tile_sense_adc_6.h",  "source": "tile_sense_adc_6",  "prefix": "tile_sense_adc_6"},
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
    "Sense.BP":    {"header": "tile_sense_bp.h",  "source": "tile_sense_bp",  "prefix": "tile_sense_bp"},
    "Sense.TOF":   {"header": "tile_sense_tof.h", "source": "tile_sense_tof", "prefix": "tile_sense_tof"},
    "Sense.ACP":   {"header": "tile_sense_acp.h", "source": "tile_sense_acp", "prefix": "tile_sense_acp"},
    "Sense.CAP":   {"header": "tile_sense_cap.h", "source": "tile_sense_cap", "prefix": "tile_sense_cap"},
    "Sense.M.3G":  {"header": "tile_sense_m_3g.h", "source": "tile_sense_m_3g", "prefix": "tile_sense_m_3g"},
    "Sense.HR":    {"header": "tile_sense_hr.h",   "source": "tile_sense_hr",   "prefix": "tile_sense_hr"},
}


def build_tiles_config(config, i2c_buses, spi_buses=None, pad_map=None):
    """Build tile peripheral config from 'tiles' list in project config.

    For each declared tile, looks up driver info, validates the bus assignment,
    and generates handle names. Returns (tiles_config, tile_pal_buses, tile_driver_sources).

    tiles_config: list of dicts with per-tile info for template rendering
    tile_pal_buses: list of dicts for unique buses needing tiles_pal_t handles
                    SPI buses include a 'cs_entries' list (one per tile instance)
    tile_driver_sources: list of unique driver source names (for Makefile)
    """
    tiles_list = config.get("tiles", [])
    if not tiles_list:
        return [], [], []

    # Build lookup of configured buses by name
    i2c_lookup = {bus["instance"]: bus for bus in i2c_buses}
    spi_lookup = {bus["instance"]: bus for bus in (spi_buses or [])}
    all_bus_names = set(i2c_lookup) | set(spi_lookup)

    # Pad number → GPIO port/pin lookup for CS resolution
    pad_lookup = {p["number"]: p for p in (pad_map or [])}

    tiles_config = []
    seen_buses = {}          # bus_name -> hal handle dict
    spi_cs_entries = {}      # bus_name -> list of {instance, port, pin}
    seen_drivers = set()

    for tile_entry in tiles_list:
        tile_type = tile_entry.get("tile", tile_entry.get("type", ""))
        bus_name = tile_entry["bus"]
        instance = tile_entry.get("instance", 0)

        # Look up driver info
        driver = TILE_DRIVER_MAP.get(tile_type)
        if driver is None:
            eprint(f"  ERROR: Unknown tile '{tile_type}'. "
                  f"Known tiles: {', '.join(sorted(TILE_DRIVER_MAP.keys()))}")
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
            if is_spi:
                spi_bus = spi_lookup[bus_name]
                seen_buses[bus_name] = {
                    "bus_name": bus_name,
                    "pal_handle": pal_handle,
                    "spi_handle": spi_bus["handle"],
                    "bus_type": "spi",
                    "cs_entries": [],   # populated below
                }
                spi_cs_entries[bus_name] = seen_buses[bus_name]["cs_entries"]
            else:
                i2c_bus = i2c_lookup[bus_name]
                seen_buses[bus_name] = {
                    "bus_name": bus_name,
                    "pal_handle": pal_handle,
                    "i2c_handle": i2c_bus["handle"],
                    "bus_type": "i2c",
                }

        # Resolve per-tile SPI CS pad → GPIO port/pin
        if is_spi:
            cs_pad_num = tile_entry.get("cs_pad")
            cs_port = None
            cs_pin = None
            if cs_pad_num:
                pad_info = pad_lookup.get(str(cs_pad_num), {})
                cs_port = pad_info.get("port")
                cs_pin = pad_info.get("pin")
                if cs_port is None or cs_pin is None:
                    eprint(f"  ERROR: Tile '{tile_type}' instance {instance}: "
                          f"CS pad {cs_pad_num} could not be resolved to a GPIO port/pin")
                    sys.exit(1)
            spi_cs_entries[bus_name].append({
                "instance": instance,
                "port": cs_port,
                "pin": cs_pin,
                "has_cs": cs_port is not None,
            })

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
            "pal_handle": seen_buses[bus_name]["pal_handle"],
            # For the iterable tile table: the bus HANDLE (so the consumer can
            # core_tiles_pal(bus) at runtime and group by pointer), plus optional
            # per-tile aux the table carries verbatim (e.g. Drive.P readdress strap).
            "bus_handle": (seen_buses[bus_name].get("i2c_handle")
                           or seen_buses[bus_name].get("spi_handle")),
            "bus_is_spi": 1 if seen_buses[bus_name]["bus_type"] == "spi" else 0,
            "readdress_gpio": tile_entry.get("readdress_gpio", -1),
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


def generate(tile_path, output_dir, config_path=None):
    """Generate all headers from a tile JSON and optional project config."""
    with open(tile_path, encoding="utf-8") as f:
        tile = json.load(f)

    # Resolve MCU info
    part = tile["components"][0]["part"]
    mcu = MCU_DB.get(part)
    if mcu is None:
        eprint(f"ERROR: Unknown MCU part '{part}'. Add it to MCU_DB in coregen.py.")
        sys.exit(1)

    # Build template context
    pad_map = build_pad_map(tile["pads"])
    system_pads = extract_system_pads(tile["pads"])
    led = extract_led_info(tile)
    interfaces = build_interface_map(tile, pad_map)
    power = tile.get("power", [{}])[0] if tile.get("power") else {}

    # Prefer the vendor-segmented public name (Core.ST.<family>.<n>) for
    # user-visible output; fall back to the DB short name (family.name) for
    # cores without a public alias yet.
    _stem = os.path.basename(tile_path).replace(".json", "")
    tile_name = CORE_STEM_TO_PUBLIC.get(_stem, f"{tile['family']}.{tile['name']}")

    ctx = {
        "tile": tile,
        "tile_name": tile_name,
        "tile_family": tile["family"],
        "tile_variant": tile["name"],
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

        # Validate core matches. Public names (Core.ST.<family>.<n>) resolve to
        # the canonical definition stem, so a config written against the public
        # name doesn't spuriously warn.
        proj_core = project.get("core", "")
        proj_core_stem = CORE_NAME_ALIASES.get(proj_core, proj_core)
        tile_file_stem = os.path.basename(tile_path).replace(".json", "")
        if proj_core and proj_core_stem != tile_file_stem:
            print(f"  NOTE: config.json targets '{proj_core}', building for '{tile_file_stem}' (TILE= override)")
            # Allow override — this is the multi-tile portability path

        # Validate pin/interface/clock assignments
        warnings, errors = validate_project_config(project, tile, pad_map, mcu)

        # BLE contract: declared services/characteristics -> generated GATT.
        # Parsed alongside the other validation so a malformed contract fails
        # the build with a clear message rather than emitting broken C.
        ctx["ble_contract"] = build_ble_contract(project, config_path, errors, warnings)
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
        # On WBA55 and L011, route I2C kernel clock to HSI16 (the L0 so 400 kHz
        # works at every clock level; RCC_CCIPR.I2C1SEL, RM0377 §7.3.19).
        # H523 uses SYSCLK — TIMINGR constants now cover 16/48/144/240MHz.
        _hsi16_i2c_parts = {"STM32WBA55xx", "STM32L011xx"}
        ctx["i2c_kernel_clk"] = "hsi16" if mcu["define"] in _hsi16_i2c_parts else None
        ctx["i2c_kernel_clk_mhz"] = 16 if mcu["define"] in _hsi16_i2c_parts else None
        # SAI kernel clock: if any pad carries a SAI1 function (PDM mic capture),
        # route the SAI1 kernel clock to HSI16 (16 MHz) — the simplest always-on
        # source, and what hal_sai's PDM clock math assumes. WBA55 only.
        _sai_pads = project.get("pads", project.get("pins", {}))
        _sai_used = any(str(f).startswith("SAI1.") for f in _sai_pads.values())
        ctx["sai_kernel_clk"] = "hsi16" if (_sai_used and mcu["define"] == "STM32WBA55xx") else None
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
        _usb_capable_parts = {"STM32L422xx", "STM32H523xx"}
        ctx["usb_capable"] = mcu["define"] in _usb_capable_parts
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
            project, ctx["i2c_buses"], ctx["spi_buses"], pad_map
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

    # Set up Jinja2
    templates_dir = os.path.join(os.path.dirname(__file__), "templates")
    env = Environment(
        loader=FileSystemLoader(templates_dir),
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

    # Generate core_drivers.mk if tiles are declared
    if ctx.get("tile_driver_sources"):
        mk_path = os.path.join(output_dir, "core_drivers.mk")
        with open(mk_path, "w", encoding="utf-8") as f:
            f.write("# AUTO-GENERATED by coregen — do not edit\n")
            f.write("TILES_DRIVERS = " + " ".join(ctx["tile_driver_sources"]) + "\n")
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
            tile_public = CORE_STEM_TO_PUBLIC.get(tile_stem, tile_stem)
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

    args = parser.parse_args()

    if not os.path.exists(args.tile_json):
        eprint(f"ERROR: File not found: {args.tile_json}")
        sys.exit(1)

    if args.config and not os.path.exists(args.config):
        eprint(f"ERROR: Config file not found: {args.config}")
        sys.exit(1)

    tile_name = os.path.basename(args.tile_json).replace(".json", "")
    print(f"coregen: {tile_name}")
    generate(args.tile_json, args.output_dir, args.config)


if __name__ == "__main__":
    main()
