"""
coregen back end for STM32 (PORT = "stm32"): every Core.ST.* board.

coregen is split in two. The front (tools/coregen/coregen.py) is
vendor-neutral: it validates config.json, builds the pad map, resolves buses,
SPI chip selects, pull-ups, ADC, I3C, tile drivers, the BLE contract and the
Thread dataset, renders the templates and writes core_drivers.mk. A back end
(this file, or tools/coregen/ports/<port>.py in public tiles or in a
TILES_OVERLAY checkout) knows one MCU vendor: its part table, clock tree,
pin mux and peripheral clock enables. coregen picks it with --port <port>;
the Makefile passes $(PORT) from the Core's core.mk.

Back-end interface: module attributes the front calls. Each is a plain
function of its arguments (no state). A back end may `import coregen` for
the front's helpers (eprint, resolve_clock_block); coregen is fully defined
before any back end loads.

  PORT                      the port name, equal to the file's stem
  mcu_for_part(part)        the MCU dict for the Core definition's
                            components[0].part, or None if unknown. Must
                            carry "define" (the C part define); the rest is
                            the back end's own and reaches templates as `mcu`
  template_dirs()           directories searched for templates before
                            tools/coregen/templates/, so a back end can
                            replace any template by file name ([] here)
  pad_gpio(pad)             (port, pin) of a definition pad's GPIO, or
                            (None, None)
  pad_mux(func)             the pin-mux value of one pad function (the
                            "af" templates read), or None when it has none
  timer_channels(pad)       [{"timer", "channel", "af"}] PWM outputs of a pad
  led_info(tile)            {"port", "pin", "active_high"} or None
  validate(config, tile, pad_map, mcu)
                            port-specific pad errors (list of str), reported
                            right after the generic pad checks
  usb_capable(mcu)          True when the Core always runs USB (CDC console,
                            1200-baud DFU touch)
  i3c_unsupported(mcu)      None when the MCU has I3C, else the reason an
                            "i3c": true bus stays I2C
  clock_config(config, tile, mcu)
                            the resolved clock tree (ctx clock_config)
  i2c_config(config, mcu, clock_config)
                            one dict per I2C bus some pad uses
  spi_config(bus_num, bus_cfg, mcu, clock_config)
                            one SPI bus's non-chip-select settings (mode,
                            prescaler, bit order, clock enable); the front
                            adds the chip selects
  usart_config(config, mcu) one dict per USART some pad uses
  pwm_config(config, mcu)   one dict per timer some pad uses
  extra_context(project, mcu)
                            extra template context (dict) for a project build

Fatal configuration errors print "  ERROR: ..." to stderr and exit 1, like
the front.
"""

import re
import sys

from coregen import eprint, resolve_clock_block

PORT = "stm32"


def mcu_for_part(part):
    return MCU_DB.get(part)


def template_dirs():
    """STM32 uses tools/coregen/templates/ as is."""
    return []


def pad_mux(func):
    """A pad function's alternate-function number: `mux` when it is
    {"kind": "af", "value": N}, else the older `af` key. None when the
    function has neither (GPIO, analog, power). 0 is a real AF."""
    mux = func.get("mux")
    if isinstance(mux, dict) and mux.get("kind") == "af":
        return mux.get("value")
    return func.get("af")


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
            # PLL1 in the wide VCO range, DS14540 Table 46: ref1_ck 2-16 MHz,
            # VCO 128-560 MHz. SYSCLK is pll1_p_ck, and P must be even
            # (RM0481 §11.8.10). This allowed odd P and a 150-836 MHz VCO.
            "m_range": (1, 63),
            "n_range": (4, 512),
            "r_values": list(range(2, 129, 2)),
            "in_min_mhz": 2,
            "in_max_mhz": 16,
            "vco_min_mhz": 128,
            "vco_max_mhz": 560,
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
        af = pad_mux(func)
        if m and af is not None:
            channels.append({
                "timer": m.group(1),
                "channel": int(m.group(2)),
                "af": af,
            })
    return channels


pad_gpio = extract_pad_gpio
led_info = extract_led_info
timer_channels = extract_timer_channels


def validate(config, tile, pad_map, mcu):
    """STM32 pad rules, run right after the generic pad checks."""
    errors = []
    pad_lookup = {p["number"]: p for p in pad_map}
    pins = config.get("pads", config.get("pins", {}))

    # Core.ST.L4: USB is always on (CDC console, 1200-baud DFU touch, serial
    # update), so its D+/D- pads can't be anything else. Assigning SPI1 /
    # USART1 / TIM1 / GPIO there silently fought the USB peripheral for PA11 /
    # PA12 (pads 6/7 on Core.ST.L4, 16/17 on Core.ST.L4.2).
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
    return errors


# USB-capable MCUs always get CDC init in core_init() so the DFU bootloader's
# 1200-baud touch reboot always works, even when main.c forgets to call
# core_usb_init(). A bootloader mode other than "none" needs it.
USB_CAPABLE = {"STM32L422xx", "STM32H523xx"}


def usb_capable(mcu):
    return bool(mcu) and mcu.get("define") in USB_CAPABLE


# I3C on an I2C bus needs an I3C HAL (see resolve_i3c_buses in coregen.py).
I3C_MCUS = {"STM32H523xx"}


def i3c_unsupported(mcu):
    if mcu["define"] not in I3C_MCUS:
        return f"the {mcu['define'][:-2]} on this Core has no I3C"
    return None


# On WBA55 and L011, the I2C kernel clock is routed to HSI16 (16MHz)
# regardless of SYSCLK (the L0 so 400 kHz works at the 1-2 MHz MSI levels
# too; RCC_CCIPR.I2C1SEL, RM0377 §7.3.19). H523 uses SYSCLK as I2C kernel
# clock — TIMINGR constants cover 16/48/144/240MHz.
HSI16_I2C_PARTS = {"STM32WBA55xx", "STM32L011xx"}


def extra_context(project, mcu):
    """Template context only the STM32 templates read."""
    ctx = {}
    ctx["i2c_kernel_clk"] = "hsi16" if mcu["define"] in HSI16_I2C_PARTS else None
    ctx["i2c_kernel_clk_mhz"] = 16 if mcu["define"] in HSI16_I2C_PARTS else None
    # SAI kernel clock: if any pad carries a SAI1 function (PDM mic capture),
    # route the SAI1 kernel clock to HSI16 (16 MHz) — the simplest always-on
    # source, and what hal_sai's PDM clock math assumes. WBA55 only.
    _sai_pads = project.get("pads", project.get("pins", {}))
    _sai_used = any(str(f).startswith("SAI1.") for f in _sai_pads.values())
    ctx["sai_kernel_clk"] = "hsi16" if (_sai_used and mcu["define"] == "STM32WBA55xx") else None
    return ctx


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


def clock_config(config, tile, mcu):
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

    if define == "STM32H523xx" and source == "csi":
        # RM0481 §55.5.2: the USB needs APB2 >= 12 MHz, and every H5 build
        # starts USB (the 1200-baud and serial-update paths depend on it). CSI
        # is 4 MHz, so 'low' runs the HSI through HSIDIV /4 instead. (The USB
        # driver used to switch SYSCLK behind the generated code's back,
        # leaving SYSCLK_HZ and every baud rate wrong.)
        eprint(f"  NOTE: clock '{level}' runs the HSI at 16 MHz, not CSI at {target_mhz} MHz, "
               f"on the {part}: USB is always on and needs APB2 >= 12 MHz (RM0481 §55.5.2).")
        source = "hsi"
        target_mhz = 16

    ble_on = bool((config.get("ble") or {}).get("enabled"))
    if define == "STM32WBA55xx" and ble_on and target_mhz <= 16:
        eprint(f"  ERROR: BLE needs clock medium or higher. Clock '{level}' runs HSI16 "
               f"in voltage range 2, and the 2.4 GHz radio needs range 1 with an "
               f"undivided hclk5 of 16-32 MHz (RM0493 §12.4.6). Set \"clock\": \"medium\".")
        sys.exit(1)
    thread_on = bool((config.get("thread") or {}).get("enabled"))
    if define == "STM32WBA55xx" and thread_on and target_mhz <= 16:
        eprint(f"  ERROR: Thread needs clock medium or higher. Clock '{level}' runs HSI16 "
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
    # H5: the HSI runs through HSIDIV (64 MHz >> 0..3, RM0481 §11.8.1). A
    # direct-HSI level at 8/16/32/64 MHz is the HSI divided down, not a PLL.
    h5_hsidiv = 0
    if define == "STM32H523xx" and source == "hsi" and target_mhz in (8, 16, 32):
        h5_hsidiv = {32: 1, 16: 2, 8: 3}[target_mhz]
        source_mhz = target_mhz

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
        # PWR_VOSCR.VOS field (RM0481 §10.11.4): 0 = VOS3 .. 3 = VOS0. The
        # scale limits are VOS3 100, VOS2 150, VOS1 200, VOS0 250 MHz (DS14540
        # Table 20); this stays one scale above that because the SPI kernel
        # clock (per_ck = HSI, 64 MHz) is over VOS3's 50 MHz SPI limit.
        if target_mhz <= 100:
            vos_value = 1   # VOS2
        elif target_mhz <= 150:
            vos_value = 2   # VOS1
        else:
            vos_value = 3   # VOS0
    if define == "STM32H523xx" and not needs_vos:
        vos_value = 0       # VOS3, the reset scale

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
        "h5_hsidiv": h5_hsidiv,
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


def usart_config(config, mcu):
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


def pwm_config(config, mcu):
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


def i2c_config(config, mcu, clock_config):
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
    i2c_clk_mhz = 16 if family_define in HSI16_I2C_PARTS else sysclk_mhz

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
    # H5: SPI1 on APB2, SPI2/SPI3 on APB1 (RM0481 §11.8.29/31; APB3 bit 5 is
    # SPI5EN, which this used to set for SPI3)
    ("STM32H523xx", 1): ("ll_rcc_apb2_clk_enable", "LL_APB2_SPI1"),
    ("STM32H523xx", 2): ("ll_rcc_apb1_clk_enable", "LL_APB1_SPI2"),
    ("STM32H523xx", 3): ("ll_rcc_apb1_clk_enable", "LL_APB1_SPI3"),
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


def spi_config(bus_num, bus_cfg, mcu, clock_config=None):
    """One SPI bus's settings from `interfaces.SPI<n>` (bus_cfg), without its
    chip selects (the front's build_spi_config adds those).

    Defaults: mode=0 (CPOL=0/CPHA=0), prescaler=8 (÷8), bit_order="msb".
    With clock_config, a prescaler that puts SCK over the part's datasheet
    ceiling is reported (a NOTE when only the low-VDD ceiling is exceeded,
    an ERROR above the 2.7-3.6 V one).
    """
    family_define = mcu["define"]
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

    return {
        "clk_func": clk_func,
        "clk_mask": clk_mask,
        "prescaler": prescaler_define,
        "cpol": cpol_define,
        "cpha": cpha_define,
        "lsb_first": bit_order == "lsb",
    }
