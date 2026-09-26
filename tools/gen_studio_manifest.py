#!/usr/bin/env python3
"""
Generate Studio manifests from cores + tiles headers.

Reads C headers, finds doxygen blocks containing `@studio expose ...`, and
emits JSON manifests consumed by the Studio frontend palette and build
service. Produces one merged core manifest and one manifest per tile driver.

`@studio` tag syntax (inside a doxygen block):

    @studio expose category=<str> icon=<glyph> name=<dsl_name> [availability=Core.X,Core.Y]

Param syntax (each `@param` line):

    @param <cname> [{dsl_type}] [[min..max]] [unit] <description>

Example:

    @studio expose category=led icon=☀ name=heartbeat
    @param period_ms [0..60000] ms Time between LED toggles.

Usage:
  tools/gen_studio_manifest.py          # write manifests
  tools/gen_studio_manifest.py --check  # verify manifests match headers
"""

import argparse
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path


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

ROOT = Path(__file__).resolve().parent.parent
CORE_OUT_DIR = ROOT / "manifests"
TILE_OUT_DIR = ROOT / "manifests"
SDK_DOCS_OUT_DIR = ROOT / "manifests" / "sdk-docs"

# Where hal_/ll_ headers live, keyed by layer.
LAYER_DIRS = {"hal": ROOT / "sdk/hal", "ll": ROOT / "sdk/ll"}

# Per-category lower-layer docs. Maps a docs category to the hal_*/ll_*
# headers that implement it, so the SDK reference can show the same surface at
# the Core / HAL / LL layer the reader works in. Each layer value is
# (header_filename, only), or a list of those when a layer spans headers (e.g.
# ll_gpio.h + ll_exti.h): `only=None` includes every documented function in
# the header (source order); a list curates + orders the output — required for
# sprawling headers like ll_rcc.h, and for register-level headers where only
# the public-facing calls belong in the reference. Categories absent here are
# Core-only. An entry may introduce a category that has no core header at all
# (e.g. clocks, which is config-driven) — give it a label/icon via the `meta`
# key.
LAYER_HEADERS = {
    "pad": {
        "hal": [("hal_gpio.h", None), ("hal_exti.h", None)],
        "ll": [
            ("ll_gpio.h", None),
            ("ll_exti.h",
             ["ll_exti_gpio_config", "ll_exti_config", "ll_exti_disable",
              "ll_exti_pending", "ll_exti_clear_pending", "ll_exti_sw_trigger",
              "ll_exti_set_source"]),
        ],
    },
    "led": {
        "ll": (
            "ll_gpio.h",
            ["ll_gpio_config_output", "ll_gpio_set", "ll_gpio_clear",
             "ll_gpio_toggle", "ll_gpio_read"],
        ),
    },
    "timing": {
        "hal": ("hal_common.h", ["hal_tick", "hal_timeout_expired"]),
        "ll": ("ll_systick.h", ["ll_systick_init", "ll_delay_ms", "ll_delay_us"]),
    },
    "watchdog": {
        "ll": (
            "ll_iwdg.h",
            ["ll_iwdg_init", "ll_iwdg_init_1s", "ll_iwdg_init_2s",
             "ll_iwdg_init_5s", "ll_iwdg_init_10s", "ll_iwdg_refresh",
             "ll_iwdg_caused_reset", "ll_rcc_clear_reset_flags"],
        ),
    },
    "clocks": {
        "meta": {"label": "Clocks", "icon": "⏱"},
        "ll": (
            "ll_rcc.h",
            ["ll_rcc_hsi16_enable", "ll_rcc_hse_enable", "ll_flash_set_latency",
             "ll_flash_latency_for_mhz", "ll_rcc_pll_config", "ll_rcc_pll_enable",
             "ll_rcc_pll_ready", "ll_rcc_set_sysclk", "ll_rcc_wait_sysclk",
             "ll_rcc_set_ahb_div", "ll_rcc_set_apb1_div", "ll_rcc_set_apb2_div"],
        ),
    },
    "serial": {
        "hal": ("hal_uart.h", None),
        "ll": ("ll_uart.h", None),
    },
    "i2c": {
        "hal": ("hal_i2c.h", None),
        "ll": (
            "ll_i2c.h",
            ["ll_i2c_init", "ll_i2c_init_fmp", "ll_i2c_write", "ll_i2c_read",
             "ll_i2c_timing_100k", "ll_i2c_timing_400k", "ll_i2c_timing_1m",
             "ll_i2c_target_init", "ll_i2c_target_disable",
             "ll_i2c_target_is_read", "ll_i2c_target_addcode",
             "ll_i2c_target_flush_tx", "ll_i2c_target_clear_addr"],
        ),
    },
    "spi": {
        "hal": ("hal_spi.h", None),
        "ll": ("ll_spi.h", None),
    },
    "timer": {
        "hal": ("hal_timer.h", None),
        "ll": ("ll_tim.h", None),
    },
    "adc": {
        "hal": ("hal_adc.h", None),
        "ll": ("ll_adc.h", None),
    },
    "audio": {
        # PDM microphone capture: SAI1 PDM master-RX + GPDMA (hal_sai), with the
        # CIC decimator surfaced from core_pdm.h (the category's core header).
        "hal": ("hal_sai.h", None),
        "ll": (
            "ll_sai.h",
            ["ll_sai_block_enable", "ll_sai_block_disable", "ll_sai_block_enable_dma",
             "ll_sai_flush", "ll_sai_read", "ll_sai_overrun", "ll_sai_clear_overrun"],
        ),
    },
    "dac": {
        "hal": ("hal_dac.h", None),
        "ll": ("ll_dac.h", None),
    },
    "power": {
        "ll": (
            "ll_pwr.h",
            ["ll_pwr_enable_backup_access", "ll_pwr_sleep_wfi", "ll_pwr_stop",
             "ll_pwr_standby", "ll_pwr_woke_from_standby", "ll_pwr_clear_standby_flag"],
        ),
    },
    "rng": {
        "ll": ("ll_rng.h", None),
    },
    "usb": {
        # Curated (not None): a documented typedef in the header otherwise
        # parses as a stray "void" entry.
        "hal": (
            "hal_usb_cdc.h",
            ["hal_usb_cdc_init", "hal_usb_cdc_connected", "hal_usb_cdc_write",
             "hal_usb_cdc_printf", "hal_usb_cdc_set_rx_callback", "hal_usb_cdc_rx_ready",
             "hal_usb_cdc_getc", "hal_usb_cdc_rx_try", "hal_usb_cdc_read",
             "hal_usb_cdc_available", "hal_usb_cdc_poll", "hal_usb_hid_send_report",
             "hal_usb_hid_set_rx_callback"],
        ),
        # Two USB IPs: USB Device FS on the L4 (ll_usb.h), USB DRD on the H5
        # (ll_usb_drd.h). Only the device-level calls; the endpoint / PMA /
        # buffer-descriptor helpers are the CDC/HID stack's plumbing.
        "ll": [
            ("ll_usb.h",
             ["ll_usb_power_on", "ll_usb_connect", "ll_usb_disconnect",
              "ll_usb_set_address", "ll_usb_ep_config", "ll_usb_pma_write",
              "ll_usb_pma_read"]),
            ("ll_usb_drd.h",
             ["ll_usb_drd_power_on", "ll_usb_drd_connect", "ll_usb_drd_disconnect",
              "ll_usb_drd_set_address", "ll_usb_drd_chep_config",
              "ll_usb_drd_pma_write", "ll_usb_drd_pma_read"]),
        ],
    },
    "nvm": {
        "ll": (
            "ll_flash.h",
            ["ll_flash_unlock", "ll_flash_lock", "ll_flash_erase_page",
             "ll_flash_program_dword", "ll_flash_program_qword",
             "ll_flash_wait_bsy", "ll_flash_clear_errors"],
        ),
    },
    "dma": {
        # No core_ layer: DMA is set up inside the ADC / SPI / SAI drivers.
        # Classic DMA (L0 / L4) and GPDMA (W5 / H5, incl. linked lists).
        "meta": {"label": "DMA", "icon": "⇶"},
        "ll": (
            "ll_dma.h",
            ["ll_dma_set_request", "ll_dma_config", "ll_dma_enable",
             "ll_dma_disable", "ll_dma_remaining", "ll_dma_transfer_complete",
             "ll_dma_half_transfer", "ll_dma_transfer_error", "ll_dma_clear_flags",
             "ll_gpdma_config", "ll_gpdma_enable", "ll_gpdma_disable",
             "ll_gpdma_transfer_complete", "ll_gpdma_half_transfer",
             "ll_gpdma_transfer_error", "ll_gpdma_clear_flags",
             "ll_gpdma_node_init", "ll_gpdma_node_link", "ll_gpdma_node_terminate",
             "ll_gpdma_list_start"],
        ),
    },
    "recovery": {
        "hal": (
            "hal_dfu.h",
            ["hal_dfu_jump_to_rom", "hal_dfu_reboot", "hal_recovery_strikes",
             "hal_recovery_valid", "hal_recovery_set_strikes",
             "hal_recovery_stash_cause", "hal_recovery_stashed_cause"],
        ),
    },
    "debug": {
        "hal": ("hal_debug.h", None),
    },
    "fault": {
        # Curated: the documented callback typedef would otherwise parse as
        # an entry.
        "hal": ("hal_fault.h", ["hal_fault_set_callback"]),
    },
    "rtc": {
        "ll": (
            "ll_rtc.h",
            ["ll_rtc_init", "ll_rtc_set_time", "ll_rtc_get_time", "ll_rtc_set_date",
             "ll_rtc_get_date", "ll_rtc_wakeup_config", "ll_rtc_wakeup_disable",
             "ll_rtc_alarm_a_set", "ll_rtc_alarm_a_disable", "ll_rtc_alarm_a_flag",
             "ll_rtc_alarm_a_clear_flag", "ll_rtc_bkp_read", "ll_rtc_bkp_write"],
        ),
    },
}

C_TO_DSL = {
    "int": "int",
    "int8_t": "int", "int16_t": "int", "int32_t": "int",
    "uint8_t": "int", "uint16_t": "int", "uint32_t": "int",
    "float": "float", "double": "float",
    "bool": "bool", "_Bool": "bool",
}

UNIT_VOCAB = {
    "ns", "us", "ms", "s",
    "hz", "khz", "mhz",
    "v", "mv", "ma", "a", "w",
    "c", "f", "k", "db",
    "rad", "deg", "pct",
    "rpm", "hpa", "pa", "g", "kg",
    "°c", "°f", "%",
}

DOXY_BLOCK_RE = re.compile(r"/\*\*(.*?)\*/", re.DOTALL)
# The return type ends in whitespace or `*`, so `hal_i2c_t *core_i2c_handle_for_bus(`
# (pointer glued to the name) parses as well as `uint8_t * f(`.
SIG_RE = re.compile(
    r"(?:static\s+inline\s+)?([\w\s\*]*?[\w\*][\s\*]+)(\w+)\s*\(([^;{]*)\)",
)
PARAM_RE = re.compile(
    r"^@param\s+(\S+)\s*(?:\{(\w+)\})?\s*(?:\[(-?[\d.]+)\.\.(-?[\d.]+)\])?\s*(.*)$"
)


def strip_doxy(body):
    out = []
    for line in body.splitlines():
        m = re.match(r"\s*\*\s?(.*)$", line)
        out.append(m.group(1) if m else line.strip())
    return out


def parse_studio_tags(lines):
    """Return every @studio tag in the block as a list of (verb, positional, attrs).

    Brace-delimited bodies (e.g. `enum {KEY=label, ...}`) are extracted
    before the rest of the line is split on whitespace so commas inside
    the braces don't tear the body apart. The parsed body appears in
    `attrs` under the key named by the token preceding the `{`.
    """
    out = []
    for line in lines:
        m = re.match(r"@studio\s+(\w+)\s*(.*)", line.strip())
        if not m:
            continue
        verb = m.group(1)
        rest = m.group(2)

        # Extract a single `<key> {...}` block so its contents (which
        # may include commas) survive the whitespace split below.
        # Only `enum` is recognised today; future brace keys (e.g.,
        # `range { ... }` for explicit value lists) can slot in here
        # without changing the outer tag shape.
        brace_attrs = {}
        brace_match = re.search(r"(\w+)\s*\{([^}]*)\}", rest)
        if brace_match:
            brace_key = brace_match.group(1)
            brace_body = brace_match.group(2)
            if brace_key == "enum":
                brace_attrs["enum"] = parse_enum_body(brace_body)
            else:
                # Unknown brace key — surface via stderr so a typo doesn't
                # silently disappear into the void.
                print(
                    f"warn: @studio {verb}: unknown brace key '{brace_key}' — only 'enum' is recognised",
                    file=sys.stderr,
                )
            rest = (rest[:brace_match.start()] + rest[brace_match.end():]).strip()

        positional = None
        attrs = {}
        # shlex keeps a quoted value together: label="Full-scale range".
        try:
            tokens = shlex.split(rest) if '"' in rest else rest.split()
        except ValueError:
            tokens = rest.split()
        for tok in tokens:
            if "=" in tok:
                k, v = tok.split("=", 1)
                attrs[k] = v
            elif positional is None:
                positional = tok
        attrs.update(brace_attrs)
        out.append((verb, positional, attrs))
    return out


def parse_enum_body(body):
    """Parse `K1=label1, K2=label2` into [{c_name, label}, ...].

    Entries with no `=` use the C identifier as its own label. Whitespace
    around keys and labels is stripped. Empty entries (trailing comma,
    etc.) are ignored silently — they're a formatting artifact, not a
    meaningful declaration.
    """
    out = []
    for part in body.split(","):
        part = part.strip()
        if not part:
            continue
        if "=" in part:
            k, v = part.split("=", 1)
            out.append({"c_name": k.strip(), "label": v.strip()})
        else:
            out.append({"c_name": part, "label": part})
    return out


def parse_studio_tag(lines):
    """Back-compat helper — return the first @studio expose tag's attrs."""
    for verb, _pos, attrs in parse_studio_tags(lines):
        if verb == "expose":
            return attrs
    return None


def parse_params(lines):
    out = []
    for raw in lines:
        line = raw.strip()
        m = PARAM_RE.match(line)
        if not m:
            continue
        name, type_override, lo, hi, rest = m.groups()
        entry = {"name": name}
        if type_override:
            entry["type_override"] = type_override
        if lo is not None and hi is not None:
            entry["range"] = [
                float(lo) if "." in lo else int(lo),
                float(hi) if "." in hi else int(hi),
            ]
        rest = rest.strip()
        if rest:
            first, _, tail = rest.partition(" ")
            if first.lower() in UNIT_VOCAB:
                entry["unit"] = first
                rest = tail.strip()
        if rest:
            entry["description"] = rest
        out.append(entry)
    return out


def first_brief(lines):
    # A brief runs until a blank line or the next @command, as Doxygen reads
    # it — so a @brief wrapped onto a second line keeps its ending instead of
    # stopping mid-sentence ("…so they survive a").
    parts = []
    capturing = True
    in_brief = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("@"):
            if in_brief:
                break
            if stripped.startswith("@brief"):
                parts.append(stripped[len("@brief"):].strip())
                in_brief = True
                continue
            capturing = False
        elif in_brief:
            if not stripped:
                break
            parts.append(stripped)
        elif capturing and stripped:
            parts.append(stripped)
    return " ".join(parts).strip() or None


def extract_signature(source, after_offset):
    # Skip whitespace past the closing `*/` of the doxy block. The next
    # non-whitespace character must be the start of a C declaration — not
    # another comment, not a preprocessor directive. If it is, the doxy
    # block wasn't attached to a declaration (e.g., a file-level header
    # comment sitting above `#ifndef GUARD`) and we bail out. This stops
    # the regex from skipping forward and matching text _inside_ a later
    # doxygen block.
    i = after_offset
    while i < len(source) and source[i].isspace():
        i += 1
    if i >= len(source):
        return None
    head = source[i:i + 2]
    if head.startswith("/") or head.startswith("#"):
        return None
    m = SIG_RE.match(source, i)
    if not m:
        return None
    ret = re.sub(r"\s+", " ", m.group(1)).strip()
    ret = re.sub(r"\s+\*", " *", ret)
    # A documented function-pointer typedef (`typedef void (*cb_t)(...)`) is
    # not a function; it used to surface as a bogus entry named "void".
    if ret.startswith("typedef"):
        return None
    ret = re.sub(r"^(?:static|inline|extern|const)\s+", "", ret)
    ret = re.sub(r"^(?:static|inline|extern|const)\s+", "", ret)
    name = m.group(2)
    raw = m.group(3).strip()
    variadic = False
    params = []
    # Docs see every parameter, including array ones (`uint32_t out[3]`),
    # which `params` has always skipped; hosts keep using `params` so the
    # palette doesn't change shape.
    doc_params = []
    if raw not in ("", "void"):
        for p in raw.split(","):
            p = p.strip()
            if p.startswith("..."):
                # `...` (the lazy match may drag a trailing
                # `__attribute__((format(...)))` along; ignore it).
                variadic = True
                continue
            am = re.match(r"(.+?)(\w+)\s*(\[[^\]]*\])\s*$", p)
            if am:
                ptype = re.sub(r"\s+", " ", am.group(1)).strip()
                doc_params.append({"name": am.group(2), "ctype": ptype, "array": am.group(3)})
                continue
            pm = re.match(r"(.+?)(\w+)\s*$", p)
            if not pm:
                continue
            ptype = re.sub(r"\s+", " ", pm.group(1)).strip()
            pname = pm.group(2).strip()
            params.append({"name": pname, "ctype": ptype})
            doc_params.append({"name": pname, "ctype": ptype})
    sig = {"returns": ret, "name": name, "params": params}
    if len(doc_params) != len(params):
        sig["doc_params"] = doc_params
    if variadic:
        # Docs only: printf-style functions keep their `...` in the published
        # signature. Palette hosts never see it (the DSL has no varargs).
        sig["variadic"] = True
    return sig


ENUM_TYPES = set()

# Enum type name → [{"name", "value", "label"}], from the header's own
# `typedef enum { A = 0x00, /**< label */ ... } name_t;`. Emitted as `choices`
# on every enum-typed param so Studio can offer a labelled list instead of a
# magic int (block editor dropdowns; Vibe's adjustable values).
ENUM_MEMBERS = {}
# Every enum constant the scanned headers declare, name → value (Vibe Settings
# expressions may name them). And the authoring errors that must fail the build.
ENUM_CONSTANTS = {}
VIBE_ERRORS = []

_ENUM_BLOCK_RE = re.compile(r"typedef\s+enum\s*\{([^}]*)\}\s*(\w+)\s*;")
_ENUM_MEMBER_RE = re.compile(
    r"(\w+)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\s*,?\s*(?:/\*\*?<?\s*(.*?)\s*\*/)?"
)


def _choice_label(text, fallback):
    """Member doc comment → a label a person would say ("+/- 2g" → "±2 g")."""
    label = (text or "").strip().rstrip(".")
    if not label:
        return fallback
    label = label.replace("+/-", "±").replace("± ", "±")
    # "±2g" / "500dps" → "±2 g" / "500 dps": a space between number and unit.
    label = re.sub(r"(\d)(g|dps|Hz|kHz|ms)\b", r"\1 \2", label)
    return label


def collect_enum_types(source):
    """Record header-declared `typedef enum {...} name_t;` type names.

    Enum-typed params pass as plain ints at the C ABI, and their member
    values are documented in the Doxygen briefs — so the DSL sees them
    as `int`. Without this every enum-typed param vendors as an
    unmappable `?<ctype>` and hard-errors in the Studio editor (first
    hit: drive_dc_h.set_current_regulation_mode's drive_dc_h_imode_t)."""
    ENUM_TYPES.update(
        re.findall(r"typedef\s+enum\s*\{[^}]*\}\s*(\w+)\s*;", source)
    )
    for body, type_name in _ENUM_BLOCK_RE.findall(source):
        members = []
        for name, value, doc in _ENUM_MEMBER_RE.findall(body):
            # `@studio value=<number>` in a member's doc comment is the physical
            # quantity it stands for (ODR_100HZ → 100), which Vibe Settings
            # expressions read through value(<arg>). It is not part of the label.
            quantity = None
            qm = re.search(r"@studio\s+value=(-?\d+(?:\.\d+)?)", doc or "")
            if qm:
                quantity = float(qm.group(1)) if "." in qm.group(1) else int(qm.group(1))
                doc = (doc[: qm.start()] + doc[qm.end():]).strip()
            member = {"name": name, "value": int(value, 0), "label": _choice_label(doc, name)}
            if quantity is not None:
                member["quantity"] = quantity
            members.append(member)
            ENUM_CONSTANTS[name] = int(value, 0)
        if members:
            ENUM_MEMBERS[type_name] = members


def dsl_type_of(ctype, override):
    if override:
        return override
    norm = ctype.replace(" *", "*").replace("* ", "*").strip()
    if norm in {"const char*", "char*"}:
        return "string"
    if ctype.strip() in C_TO_DSL:
        return C_TO_DSL[ctype.strip()]
    if norm in C_TO_DSL:
        return C_TO_DSL[norm]
    if ctype.strip() in ENUM_TYPES:
        return "int"
    return f"?{ctype}"


def build_host_entry(tag, doxy_lines, sig, header_name, scope, all_tags=()):
    description = first_brief(doxy_lines)
    doxy_params = parse_params(doxy_lines)

    # Per-parameter studio annotations — currently only `@studio param
    # <cname> enum {...}` carries useful metadata, but this is the seam
    # for future per-param attributes (e.g., bitmasks, unit hints).
    # Indexed by the positional C identifier so we can merge into the
    # dsl_params loop below without altering doxy_params shape.
    studio_params = {}
    for verb, positional, attrs in all_tags:
        if verb == "param" and positional:
            studio_params[positional] = attrs

    # `@studio out_buffer <cname> type=...` identifies which C parameter
    # is an output buffer the driver writes into. Two flavours:
    #
    #   Fixed-length:   length=<N>             → DSL sees `int[N]` return
    #   Cap (variable): cap_param=<other_cname> → DSL sees `int[]` writable
    #                                              param + scalar return for
    #                                              the actual count
    #
    # Fixed-length collapses the buffer param into a return value entirely
    # (the function's C return type is void; DSL sees an `int[N]` return).
    # Cap mode keeps the function returning a scalar count and exposes the
    # buffer as a writable array param the DSL caller hands in.
    out_buffers = {}
    out_buffer_caps = set()  # cnames of cap params (stripped from DSL)
    for verb, positional, attrs in all_tags:
        if verb != "out_buffer" or not positional:
            continue
        if "type" not in attrs:
            print(
                f"warn: {sig['name']}: @studio out_buffer {positional} missing type=",
                file=sys.stderr,
            )
            continue
        has_length = "length" in attrs
        has_cap_param = "cap_param" in attrs
        if not has_length and not has_cap_param:
            print(
                f"warn: {sig['name']}: @studio out_buffer {positional} needs either length=<N> (fixed) or cap_param=<name> (variable)",
                file=sys.stderr,
            )
            continue
        if has_length and has_cap_param:
            print(
                f"warn: {sig['name']}: @studio out_buffer {positional} can't carry both length= and cap_param=",
                file=sys.stderr,
            )
            continue
        entry = {"type": attrs["type"]}
        if has_length:
            try:
                entry["length"] = int(attrs["length"])
            except ValueError:
                print(
                    f"warn: {sig['name']}: @studio out_buffer {positional} length={attrs['length']!r} must be an integer",
                    file=sys.stderr,
                )
                continue
        else:
            entry["cap_param"] = attrs["cap_param"]
            out_buffer_caps.add(attrs["cap_param"])
        out_buffers[positional] = entry

    # `@studio out_scalar <cname> type=<ctype>` identifies a scalar pointer
    # parameter (`Type *<cname>`) that the driver writes into. The DSL caller
    # passes a *local* (lvalue Ident) into the slot and the value is back-
    # filled when the call returns. Multiple out_scalars per host are allowed
    # (this is how multi-out functions like `self_test(*accel, *gyro)` get
    # exposed faithfully). The param stays in the DSL-visible signature and
    # the entry carries `out_scalar: True` so consumers (type checker /
    # codegen) can enforce the lvalue rule and emit `&` (C) or stage memory
    # (Wasm) at the call site.
    out_scalars = {}
    for verb, positional, attrs in all_tags:
        if verb != "out_scalar" or not positional:
            continue
        if "type" not in attrs:
            print(
                f"warn: {sig['name']}: @studio out_scalar {positional} missing type=",
                file=sys.stderr,
            )
            continue
        out_scalars[positional] = {"type": attrs["type"]}

    # `@studio in_buffer <cname> type=<element> length_param=<other_cname>
    #     [length=<N>]`
    # identifies which C parameter is a caller-passed array buffer + which
    # adjacent param carries its length. Both flavours strip the buffer
    # param + length param from the DSL-facing list and emit a single
    # array DSL param in the buffer's position:
    #
    #   Fixed-length (length=<N>):    DSL sees `int[N]`. Codegen splices
    #                                 the literal N at the call site
    #                                 regardless of caller's array.
    #   Variable (no length=):        DSL sees `int[]`. Codegen reads the
    #                                 caller's array length at the call
    #                                 site and splices that into the C
    #                                 count slot.
    in_buffers = {}      # cname → { type, length?, length_param? }
    in_buffer_lengths = set()  # cnames of length params (stripped from DSL)
    for verb, positional, attrs in all_tags:
        if verb != "in_buffer" or not positional:
            continue
        if "type" not in attrs:
            print(
                f"warn: {sig['name']}: @studio in_buffer {positional} missing type=",
                file=sys.stderr,
            )
            continue
        has_length = "length" in attrs
        has_length_param = "length_param" in attrs
        # Three valid shapes:
        #   length=N + length_param=<name>  → fixed-length, C count arg present
        #   length_param=<name> only         → variable-length, count from caller
        #   length=N only                    → fixed-length, C function has no count arg
        if not has_length and not has_length_param:
            print(
                f"warn: {sig['name']}: @studio in_buffer {positional} needs length= (fixed without count arg), length_param= (variable), or both (fixed with count arg)",
                file=sys.stderr,
            )
            continue
        entry = {"type": attrs["type"]}
        if has_length_param:
            entry["length_param"] = attrs["length_param"]
        if has_length:
            try:
                entry["length"] = int(attrs["length"])
            except ValueError:
                print(
                    f"warn: {sig['name']}: @studio in_buffer {positional} length={attrs['length']!r} must be an integer",
                    file=sys.stderr,
                )
                continue
        in_buffers[positional] = entry
        if has_length_param:
            in_buffer_lengths.add(attrs["length_param"])

    receiver = None
    c_params = []
    for sp in sig["params"]:
        norm = sp["ctype"].replace(" *", "*").replace("* ", "*").strip()
        if norm == "tile_t*":
            receiver = sp["ctype"]
        elif sp["name"] in out_buffers:
            # Fixed-length out-buffer collapses entirely into a return
            # value; cap-mode out-buffer surfaces as a writable DSL array
            # param (handled in the dsl_params loop further down).
            ob = out_buffers[sp["name"]]
            if "length" in ob:
                continue
            else:
                c_params.append(sp)
        elif sp["name"] in in_buffer_lengths:
            # Length params paired with an `@studio in_buffer` are
            # implicit at the DSL layer (array.length supplies them).
            continue
        elif sp["name"] in out_buffer_caps:
            # Cap params paired with a cap-mode `@studio out_buffer`
            # are implicit (the writable array's declared length supplies
            # them).
            continue
        else:
            c_params.append(sp)

    # Warn when out_buffer / in_buffer annotations don't line up with
    # the C signature — catches typos (`@studio out_buffer buf` when
    # the param is named `buffer`).
    c_param_names = {sp["name"] for sp in sig["params"]}
    for cname in out_buffers:
        if cname not in c_param_names:
            print(
                f"warn: {sig['name']}: @studio out_buffer {cname} doesn't match any C parameter",
                file=sys.stderr,
            )
    for cname in in_buffers:
        if cname not in c_param_names:
            print(
                f"warn: {sig['name']}: @studio in_buffer {cname} doesn't match any C parameter",
                file=sys.stderr,
            )
    for cname in out_scalars:
        if cname not in c_param_names:
            print(
                f"warn: {sig['name']}: @studio out_scalar {cname} doesn't match any C parameter",
                file=sys.stderr,
            )
    for cname in in_buffer_lengths:
        if cname not in c_param_names:
            print(
                f"warn: {sig['name']}: @studio in_buffer length_param={cname} doesn't match any C parameter",
                file=sys.stderr,
            )

    # Cap-mode out_buffers stay in `c_params` (they're DSL-visible as
    # writable arrays); fixed-length out_buffers were stripped above.
    fixed_out_buffer_count = sum(1 for ob in out_buffers.values() if "length" in ob)
    expected_doxy = len(c_params) + fixed_out_buffer_count + len(in_buffer_lengths) + len(out_buffer_caps)
    if len(doxy_params) != expected_doxy:
        print(
            f"warn: {sig['name']}: @param count {len(doxy_params)} != C arg count "
            f"{len(c_params)} + fixed out_buffer {fixed_out_buffer_count} "
            f"+ in_buffer length params {len(in_buffer_lengths)} "
            f"+ out_buffer cap params {len(out_buffer_caps)}",
            file=sys.stderr,
        )

    for cname in out_buffer_caps:
        if cname not in c_param_names:
            print(
                f"warn: {sig['name']}: @studio out_buffer cap_param={cname} doesn't match any C parameter",
                file=sys.stderr,
            )

    # Index doxy params by their declared C name so we survive a
    # stripped out_buffer in the middle of the list (positional index
    # would misalign). Fall back to positional alignment for params
    # that don't match by name (e.g., author renamed the DSL param in
    # the @param line without touching the C signature).
    doxy_by_name = {p["name"]: p for p in doxy_params}
    dsl_params = []
    for i, cp in enumerate(c_params):
        meta = doxy_by_name.get(cp["name"])
        if meta is None:
            meta = doxy_params[i] if i < len(doxy_params) else {"name": cp["name"]}
        # Array IN / cap-mode OUT params get a DSL array type from the
        # @studio in_buffer / @studio out_buffer annotation,
        # overriding the underlying C pointer type. Fixed-length renders
        # as `int[N]`; variable / cap-mode renders as `int[]`.
        if cp["name"] in in_buffers:
            ib = in_buffers[cp["name"]]
            element_dsl = dsl_type_of(ib["type"], None)
            if "length" in ib:
                entry_type = f"{element_dsl}[{ib['length']}]"
            else:
                entry_type = f"{element_dsl}[]"
        elif cp["name"] in out_buffers and "length" not in out_buffers[cp["name"]]:
            ob = out_buffers[cp["name"]]
            element_dsl = dsl_type_of(ob["type"], None)
            entry_type = f"{element_dsl}[]"
        elif cp["name"] in out_scalars:
            os_ = out_scalars[cp["name"]]
            entry_type = dsl_type_of(os_["type"], None)
        else:
            entry_type = dsl_type_of(cp["ctype"], meta.get("type_override"))
        entry = {
            "name": meta["name"],
            "type": entry_type,
        }
        if cp["name"] in out_scalars:
            entry["out_scalar"] = True
        if "range" in meta:
            entry["range"] = meta["range"]
        if "unit" in meta:
            entry["unit"] = meta["unit"]
        if "description" in meta:
            entry["description"] = meta["description"]
        # Attach per-param studio annotations when present. Matched by
        # either the C identifier (`cp["name"]`) or the DSL-facing name
        # — the latter catches the common pattern where @param renames
        # a parameter for DSL friendliness.
        #
        #   - `type=<dsl_type>` overrides the C-inferred DSL type. Used
        #     for array-in-param annotations like `type=int[16]` where
        #     the C type is a pointer but the DSL sees a fixed-length
        #     array.
        #   - `enum {K=label, ...}` attaches friendly labels for
        #     int-valued enum C params.
        # Enum-typed C params carry their members as `choices`.
        enum_members = ENUM_MEMBERS.get(cp["ctype"].strip())
        if enum_members:
            entry["choices"] = enum_members
        tp = studio_params.get(cp["name"]) or studio_params.get(entry["name"])
        if tp:
            if "type" in tp:
                entry["type"] = tp["type"]
            if "enum" in tp:
                entry["enum"] = tp["enum"]
        dsl_params.append(entry)

    category = tag.get("category", "?")
    dsl_name = tag.get("name", sig["name"])
    qname = ["$slot", dsl_name] if scope == "tile" else [category, dsl_name]

    host = {
        "qname": qname,
        "symbol": sig["name"],
        "header": header_name,
        "category": category,
        "description": description,
        "params": dsl_params,
        "returns": sig["returns"],
    }
    # DSL-visible return type — explicitly declared on the @studio expose
    # tag via `returns=<int|bool|float|string>`. Drives the DSL's import
    # return-type clause (`import X.Y() -> int`) and makes the host
    # callable as a CallExpr. Void hosts (no `returns=`) stay statement-
    # only; this is orthogonal to the C-level `returns` field above which
    # records the underlying C return type verbatim.
    # `@studio control <cparam> label="…" tier=basic|advanced [default=<ENUM|n>]`
    # marks an argument of THIS function as a user-facing setting. A control is
    # always defined in terms of an exposed function — never a second API — so
    # the block editor, the compiler's checks and Vibe all read one source.
    #   tier=basic     worth surfacing for a plain ask ("read the IMU" → range)
    #   tier=advanced  only when the user asks for it
    controls = []
    dsl_names = {cp["name"]: dp["name"] for cp, dp in zip(c_params, dsl_params)}
    for verb, positional, attrs in all_tags:
        if verb != "control" or not positional:
            continue
        if positional not in dsl_names and positional not in dsl_names.values():
            print(
                f"warn: {sig['name']}: @studio control {positional} doesn't match any parameter",
                file=sys.stderr,
            )
            continue
        pname = dsl_names.get(positional, positional)
        tier = attrs.get("tier", "advanced")
        if tier not in ("basic", "advanced"):
            print(
                f"warn: {sig['name']}: @studio control {positional} tier={tier!r} must be basic|advanced",
                file=sys.stderr,
            )
            tier = "advanced"
        control = {"param": pname, "label": attrs.get("label", pname), "tier": tier}
        # scope=config (default): a pure setting — it has a value whether or not
        #   the program sets it, so Studio may write the call itself.
        # scope=usage: an argument of something the program DOES (a wait timeout, a
        #   detection threshold) — only a setting once the program makes the call.
        scope_attr = attrs.get("scope", "config")
        if scope_attr not in ("config", "usage"):
            VIBE_ERRORS.append(f"{sig['name']}: control {pname}: scope={scope_attr!r} must be config|usage")
        control["scope"] = scope_attr
        if attrs.get("type") == "bool":
            control["type"] = "bool"
        elif "type" in attrs:
            VIBE_ERRORS.append(f"{sig['name']}: control {pname}: type={attrs['type']!r} — only type=bool exists")
        if "role" in attrs:
            if attrs["role"] != "sample_rate":
                VIBE_ERRORS.append(f"{sig['name']}: control {pname}: role={attrs['role']!r} — only role=sample_rate exists")
            control["role"] = attrs["role"]
        # rate="<expr>": the sample rate in Hz this setting produces, when the argument
        # is not an enum whose members carry it (`@studio value=`): a period
        # (rate="1000 / period_ms"), a divider (rate="1125 / (1 + divider)").
        if "rate" in attrs:
            control["_rate_src"] = attrs["rate"]
        # scale=<n> unit=<u>: the argument is stored in sub-units (0.1 dB, 0.01 °C);
        # people read and type value × scale, in `unit`. Unlike show= this is
        # invertible, so it drives the INPUT as well as the display.
        if "scale" in attrs:
            try:
                k = float(attrs["scale"])
                if k <= 0:
                    raise ValueError
                control["scale"] = int(k) if k == int(k) else k
            except ValueError:
                VIBE_ERRORS.append(f"{sig['name']}: control {pname}: scale={attrs['scale']!r} must be a positive number")
            if "unit" in attrs and "show" not in attrs:
                control["display_unit"] = attrs["unit"]
        # when="<expr>": the setting only APPLIES while this holds (a filter on a
        # sensor that is switched off; a range on a powered-down axis). Studio
        # greys it out and keeps it, and the part it gates, out of the story.
        if "when" in attrs:
            control["_when_src"] = attrs["when"]
        if "allow" in attrs:
            control["_allow_src"] = [a for a in attrs["allow"].split(",") if a]
        if "show" in attrs:
            control["_show_src"] = attrs["show"]
            if "unit" in attrs:
                control["show_unit"] = attrs["unit"]
        if "default" in attrs:
            raw = attrs["default"]
            param_entry = next((dp for dp in dsl_params if dp["name"] == pname), {})
            member = next(
                (c for c in param_entry.get("choices", []) if c["name"] == raw), None
            )
            if member is not None:
                control["default"] = member["value"]
            else:
                try:
                    control["default"] = int(raw, 0)
                except ValueError:
                    print(
                        f"warn: {sig['name']}: @studio control {positional} default={raw!r} is neither an enum member nor an integer",
                        file=sys.stderr,
                    )
        controls.append(control)
    if controls:
        host["controls"] = controls
    # `@studio require expr="…" [when="…"] message="…"` — a relation that must hold
    # between this tile's settings. Parsed + resolved in resolve_vibe_settings().
    requires = [
        attrs for verb, _, attrs in all_tags if verb == "require"
    ]
    if requires:
        host["_requires_src"] = requires
    if "returns" in tag:
        host["dsl_returns"] = tag["returns"]
    if "icon" in tag:
        host["icon"] = tag["icon"]
    # Array-returning hosts: the frontend needs the element type +
    # length to declare the right stack buffer before the call. We
    # only support one out-buffer per host (matching the manifest
    # schema on the consumer side). Multiple annotations collapse
    # to the first with a warning.
    if out_buffers:
        if len(out_buffers) > 1:
            print(
                f"warn: {sig['name']}: multiple @studio out_buffer annotations — only one out-buffer per host is supported (using the first)",
                file=sys.stderr,
            )
        first_name = next(iter(out_buffers))
        ob = out_buffers[first_name]
        # Cap-mode out_buffers are DSL-visible writable params; the
        # frontend resolver needs to know the param's name. Fixed-length
        # out_buffers collapse into the host's return value, so the
        # name is irrelevant there.
        if "cap_param" in ob:
            host["c_out_buffer"] = {"name": first_name, **ob}
        else:
            host["c_out_buffer"] = ob
    if out_scalars:
        # Preserve C-signature order (dict iteration follows insertion =
        # parse order, but the parse loop visits @studio tags in source
        # order — sort by C param position so codegen emits args in the
        # right slot).
        order = {sp["name"]: i for i, sp in enumerate(sig["params"])}
        host["c_out_scalars"] = [
            {"name": n, **out_scalars[n]}
            for n in sorted(out_scalars.keys(), key=lambda n: order.get(n, 1 << 30))
        ]
    if in_buffers:
        if len(in_buffers) > 1:
            print(
                f"warn: {sig['name']}: multiple @studio in_buffer annotations — only one in-buffer per host is supported (using the first)",
                file=sys.stderr,
            )
        first_name = next(iter(in_buffers))
        # `name` is the DSL-facing param name, which equals the C param
        # name for any in_buffer flow that survived the doxy reconciliation.
        # `length_param` carries the C name of the count slot so codegen
        # can splice the literal length value into the C call in the right
        # position. `length` is the fixed array length (the array's `[N]`).
        host["c_in_buffer"] = {
            "name": first_name,
            **in_buffers[first_name],
        }
    if receiver:
        host["receiver"] = receiver.strip()
    if "availability" in tag:
        host["availability"] = {"cores": tag["availability"].split(",")}
    return host


def parse_header(path, scope):
    """Return (hosts, sections, docs, events).

    `hosts` — palette-facing entries for functions tagged `@studio expose`.
    `sections` — file-scope metadata from `@studio category`/`@studio tile`.
    `docs` — docs-facing entries for every documented function in the file,
             whether or not it's Studio-exposed. This is what feeds the
             SDK reference pages on the website.
    `events` — event declarations from `@studio event name=<id>
               [description="..."] [payload=n:t,n:t,...]`. Valid in both
               scopes; tile events carry extra `mask`/`read`/`read_type`
               attributes tied to the tile driver's on_event ABI, core
               events rely on coregen emitting a subsystem-specific
               dispatcher (see core_pad.h → pad-edge dispatcher).

    Core-scope returns `sections = {"<category>": {label, icon}, ...}`.
    Tile-scope returns `sections = {"<tile>": {label, icon}}` with a single
    entry keyed by the tile palette label.
    """
    source = path.read_text()
    collect_enum_types(source)
    hosts = []
    sections = {}
    docs = []
    events = []
    for m in DOXY_BLOCK_RE.finditer(source):
        lines = strip_doxy(m.group(1))
        tags = parse_studio_tags(lines)

        for verb, positional, attrs in tags:
            if verb == "category" and scope == "core" and positional:
                sections[positional] = {
                    "label": attrs.get("label", positional),
                    "icon": attrs.get("icon", ""),
                }
            elif verb == "tile" and scope == "tile":
                label = attrs.get("label", path.stem)
                sections[label] = {
                    "label": label,
                    "icon": attrs.get("icon", ""),
                }
            elif verb == "event":
                name = attrs.get("name")
                if not name:
                    print(
                        f"warn: {path.name}: @studio event missing name=",
                        file=sys.stderr,
                    )
                    continue
                entry = {
                    "name": name,
                    "payload": parse_event_payload(attrs.get("payload", "")),
                }
                if "description" in attrs:
                    entry["description"] = attrs["description"]
                if "icon" in attrs:
                    entry["icon"] = attrs["icon"]
                if "mask" in attrs:
                    # C expression the dispatcher AND's against the `events`
                    # bitmask to detect this event firing. Typically a #define
                    # from the tile driver header (e.g., ICM42686P_INT_TILT_DET).
                    entry["mask"] = attrs["mask"]
                if "read" in attrs:
                    # Driver function that populates a payload struct. The
                    # dispatcher calls it when the event fires and passes
                    # the struct fields as handler arguments by name.
                    # Signature is `void read(tile_t *, <read_type> *)`.
                    entry["read"] = attrs["read"]
                if "read_type" in attrs:
                    # C struct type whose fields match the event's payload
                    # parameter names. Declared as a stack local inside the
                    # dispatch branch before calling `read`.
                    entry["read_type"] = attrs["read_type"]
                events.append(entry)

        sig = extract_signature(source, m.end())
        if not sig:
            # Floating doxygen block with no following function — fine,
            # probably a file-level comment. Skip silently.
            continue

        expose = next((a for v, _, a in tags if v == "expose"), None)
        if expose:
            hosts.append(build_host_entry(expose, lines, sig, path.name, scope, tags))

        # Docs entry: collect every function with a preceding doxygen block
        # so the SDK reference page includes init / sos / etc. even though
        # they're not palette-exposed.
        docs.append(build_doc_entry(lines, sig, bool(expose), source, m.end()))
    resolve_vibe_settings(hosts, path.name)
    return hosts, sections, docs, events


def resolve_vibe_settings(hosts, where):
    """Second pass over ONE tile's hosts: turn every Vibe Settings expression into
    an AST, resolve names, and lint. Anything wrong is an authoring error that
    fails the build (VIBE_ERRORS) — a bad rule must never reach a user."""
    from studio_expr import ExprError, parse as parse_expr

    by_fn = {h["qname"][-1]: h for h in hosts}

    def resolver(this_fn):
        def resolve_ref(name):
            fn, _, arg = name.rpartition(".")
            fn = fn or this_fn
            host = by_fn.get(fn)
            if host is None:
                raise ExprError(f"no exposed function named {fn!r} on this tile")
            if not any(p["name"] == arg for p in host["params"]):
                raise ExprError(f"{fn} has no argument {arg!r}")
            return f"{fn}.{arg}"

        return resolve_ref

    def compile_expr(text, this_fn, what):
        try:
            return parse_expr(text, resolver(this_fn), ENUM_CONSTANTS.get)
        except ExprError as err:
            VIBE_ERRORS.append(f"{where}: {this_fn}: {what} {text!r}: {err}")
            return None

    labels = {}
    for host in hosts:
        fn = host["qname"][-1]
        for control in host.get("controls", []):
            param = next(p for p in host["params"] if p["name"] == control["param"])
            choices = param.get("choices") or []
            tag = f"{where}: {fn}: control {control['param']}"
            # one label, one setting
            if control["label"] in labels:
                VIBE_ERRORS.append(f"{tag}: label {control['label']!r} is already used by {labels[control['label']]}")
            labels[control["label"]] = f"{fn}.{control['param']}"
            # allow=<MEMBER,…>: the members THIS argument may take, when the enum type
            # is shared (one power-mode enum, but only the gyro has Standby) or gives
            # one register value two names. Studio offers exactly `choices`.
            allow_src = control.pop("_allow_src", None)
            offered = choices
            if allow_src is not None:
                for name in allow_src:
                    if not any(c["name"] == name for c in choices):
                        VIBE_ERRORS.append(f"{tag}: allow={name} is not a member of the argument's enum")
                offered = [c for c in choices if c["name"] in allow_src]
                control["choices"] = offered
            values = [c["value"] for c in offered]
            if len(values) != len(set(values)):
                VIBE_ERRORS.append(f"{tag}: two offered enum members share a value — pick with allow=…")
            if control.get("type") == "bool" and choices:
                VIBE_ERRORS.append(f"{tag}: type=bool on an enum argument")
            if "default" in control and offered and control["default"] not in values:
                VIBE_ERRORS.append(f"{tag}: default is not one of the offered members")
            # A config setting is in force even when the program never sets it, so
            # Studio must know what it is. (A usage setting has no value until used.)
            if control["scope"] == "config" and "default" not in control:
                VIBE_ERRORS.append(f"{tag}: scope=config needs default=… (the power-on / init value, from the datasheet)")
            if not choices and control.get("type") != "bool" and not param.get("range"):
                VIBE_ERRORS.append(f"{tag}: a numeric setting needs `@param {control['param']} [min..max]` so Studio can bound it")
            rate_src = control.pop("_rate_src", None)
            if rate_src is not None:
                if control.get("role") != "sample_rate":
                    VIBE_ERRORS.append(f"{tag}: rate=… only means something with role=sample_rate")
                ast = compile_expr(rate_src, fn, "rate")
                if ast is not None:
                    control["rate"] = ast
            elif control.get("role") == "sample_rate":
                # Without rate=, the rate IS the selected member's quantity — so every
                # offered member must have one, or the rule silently never fires.
                if not offered:
                    VIBE_ERRORS.append(f'{tag}: role=sample_rate on a numeric argument needs rate="<expr in Hz>"')
                elif any("quantity" not in c for c in offered):
                    missing = ", ".join(c["name"] for c in offered if "quantity" not in c)
                    VIBE_ERRORS.append(f"{tag}: role=sample_rate needs `@studio value=<Hz>` on every offered member (missing: {missing}) or rate=…")
            if "scale" in control and (choices or control.get("type") == "bool"):
                VIBE_ERRORS.append(f"{tag}: scale= only applies to a numeric argument")
            if "scale" in control and "show" in control:
                VIBE_ERRORS.append(f"{tag}: use scale= (invertible) OR show= (computed), not both")
            when_src = control.pop("_when_src", None)
            if when_src is not None:
                ast = compile_expr(when_src, fn, "when")
                if ast is not None:
                    control["when"] = ast
            show_src = control.pop("_show_src", None)
            if show_src is not None:
                ast = compile_expr(show_src, fn, "show")
                if ast is not None:
                    control["show"] = ast
        rules = []
        for attrs in host.pop("_requires_src", []):
            if "expr" not in attrs or "message" not in attrs:
                VIBE_ERRORS.append(f'{where}: {fn}: @studio require needs expr="…" and message="…"')
                continue
            rule = {"expr": compile_expr(attrs["expr"], fn, "require"), "message": attrs["message"]}
            if "when" in attrs:
                rule["when"] = compile_expr(attrs["when"], fn, "when")
            if rule["expr"] is not None and rule.get("when", True) is not None:
                rules.append(rule)
        if rules:
            host["requires"] = rules


def parse_event_payload(spec):
    """Split a `payload=name:type,name:type,...` attribute into a list of
    {name, type} dicts. Empty spec returns []; types default to int."""
    if not spec:
        return []
    out = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            name, typ = part.split(":", 1)
            out.append({"name": name.strip(), "type": typ.strip()})
        else:
            out.append({"name": part, "type": "int"})
    return out


def build_doc_entry(doxy_lines, sig, studio_exposed, source, doxy_end_offset):
    """Build a docs-facing entry for the SDK reference pages.

    Carries richer C-level detail than the palette `host` entry: C parameter
    types (not DSL-mapped), attributes like `noreturn` pulled off the
    declaration line, and the signature as a single string the website can
    format. Params without `@param` lines still appear — their description
    is empty — so hidden-but-documented functions (e.g., no-arg init) land
    cleanly on the page.
    """
    description = first_brief(doxy_lines)
    doxy_params = parse_params(doxy_lines)
    by_name = {p["name"]: p for p in doxy_params}

    doc_params = []
    c_params = sig.get("doc_params", sig["params"])
    for i, cp in enumerate(c_params):
        # Prefer name-matched doxy meta; fall back to positional; fall back
        # to the C param name when no doxy exists at all (e.g., a void-init).
        meta = by_name.get(cp["name"]) or (
            doxy_params[i] if i < len(doxy_params) else {"name": cp["name"]}
        )
        entry = {
            "name": meta.get("name", cp["name"]),
            "ctype": cp["ctype"] + cp.get("array", ""),
        }
        if "range" in meta:
            entry["range"] = meta["range"]
        if "unit" in meta:
            entry["unit"] = meta["unit"]
        if "description" in meta:
            entry["description"] = meta["description"]
        doc_params.append(entry)

    c_sig = format_c_params(c_params)
    if sig.get("variadic"):
        c_sig = "..." if c_sig == "void" else c_sig + ", ..."
    signature = f"{sig['name']}(" + c_sig + ")"
    attributes = detect_attributes(source, doxy_end_offset, sig["name"])

    entry = {
        "name": sig["name"],
        "signature": signature,
        "returns": sig["returns"],
        "brief": description,
        "params": doc_params,
        "studio_exposed": studio_exposed,
    }
    if attributes:
        entry["attributes"] = attributes
    return entry


def parse_layer_docs(path, layer, only=None):
    """Docs-facing entries for the hal_*/ll_* functions in `path`, tagged `layer`.

    Reuses the core docs path (Doxygen block -> build_doc_entry) but these are
    never Studio-exposed. Duplicate names — the family-gated `#if` variants in
    e.g. ll_rcc.h — collapse to the first occurrence. `only` (a list of names)
    curates and orders the output and is required for sprawling headers;
    without it, every documented function in the header is included in source
    order. Names in `only` that aren't found (or aren't Doxygen'd) are warned
    about, not silently dropped.
    """
    source = path.read_text()
    by_name = {}
    order = []
    for m in DOXY_BLOCK_RE.finditer(source):
        sig = extract_signature(source, m.end())
        if not sig or sig["name"] in by_name:
            continue
        entry = build_doc_entry(strip_doxy(m.group(1)), sig, False, source, m.end())
        entry["layer"] = layer
        by_name[sig["name"]] = entry
        order.append(sig["name"])
    if only is None:
        return [by_name[n] for n in order]
    missing = [n for n in only if n not in by_name]
    if missing:
        print(
            f"warn: {path.name}: layer-doc functions not found or undocumented: "
            f"{', '.join(missing)}",
            file=sys.stderr,
        )
    return [by_name[n] for n in only if n in by_name]


GAP_HEAD_RE = re.compile(r"^//\s*@studio\s+unsupported\s+(.*)$")
GAP_ATTR_RE = re.compile(r'(\w+)=("([^"]*)"|\S+)')


def parse_sdk_gaps(path):
    """The SDK's own known-gap notes in one core header.

    Returns None when the header declares none, else a dict with the header's
    `@studio coverage` identity (id / name / page), its `@studio category`
    (if any), and the gaps. A gap is a `//` comment block:

        // @studio unsupported tier=<1|2|3> value=<H|M|L> title="..."
        //   continuation lines, joined into `brief`, until a bare `//`,
        //   a non-comment line or the next @-directive.

    tier: which API tier the gap sits at (1 = C API, 2 = the DSL / Studio
    default instance, 3 = wire format / later). value: how much closing it
    is worth (H / M / L). These were written for an SDK coverage table and
    fed nothing until this; the website now renders them per subsystem.
    """
    source = path.read_text()
    lines = source.splitlines()
    gaps = []
    i = 0
    while i < len(lines):
        m = GAP_HEAD_RE.match(lines[i].strip())
        if not m:
            i += 1
            continue
        attrs = {k: (q if q else v) for k, v, q in GAP_ATTR_RE.findall(m.group(1))}
        brief = []
        j = i + 1
        while j < len(lines):
            t = lines[j].strip()
            if not t.startswith("//"):
                break
            body = t[2:].strip()
            if not body or body.startswith("@"):
                break
            brief.append(body)
            j += 1
        gap = {"title": attrs.get("title", ""), "brief": " ".join(brief)}
        if "tier" in attrs:
            gap["tier"] = int(attrs["tier"]) if attrs["tier"].isdigit() else attrs["tier"]
        if "value" in attrs:
            gap["value"] = attrs["value"]
        gaps.append(gap)
        i = j
    if not gaps:
        return None
    ident = {"id": path.stem.removeprefix("core_"), "name": path.stem}
    category = None
    for m in DOXY_BLOCK_RE.finditer(source):
        block = strip_doxy(m.group(1))
        for verb, positional, _attrs in parse_studio_tags(block):
            if verb == "category" and positional and category is None:
                category = positional
        in_cov = False
        for line in block:
            t = line.strip()
            if t.startswith("@studio coverage"):
                in_cov = True
                continue
            if in_cov:
                cm = re.match(r"(id|name|page):\s*(.+)$", t)
                if cm:
                    ident[cm.group(1)] = cm.group(2).strip()
                elif t.startswith("@") or not t:
                    in_cov = False
    entry = {"id": ident["id"], "name": ident["name"], "header": path.name}
    if "page" in ident:
        entry["page"] = ident["page"]
    if category:
        entry["category"] = category
    entry["gaps"] = gaps
    return entry


def format_c_params(params):
    if not params:
        return "void"
    return ", ".join(f"{p['ctype']} {p['name']}{p.get('array', '')}" for p in params)


def detect_attributes(source, after_offset, _fn_name):
    """Pick up `__attribute__((...))` annotations on the declaration that
    immediately follows the doxy block. Scans only up to the next `;` or
    `{` — that's the end of this declaration. A wider window would leak
    attributes from later functions (e.g., core_led_blink catching the
    noreturn off a later forward-declared core_led_sos)."""
    end = len(source)
    for ch in (";", "{"):
        idx = source.find(ch, after_offset)
        if idx != -1 and idx < end:
            end = idx
    window = source[after_offset:end]
    attrs = []
    for match in re.finditer(r"__attribute__\s*\(\s*\(\s*(\w+)\s*\)\s*\)", window):
        name = match.group(1)
        if name == "noreturn" and "noreturn" not in attrs:
            attrs.append("noreturn")
    return attrs


def source_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=ROOT, stderr=subprocess.DEVNULL,
        ).decode().strip()
    except Exception:
        return "unknown"


def serialize(data):
    return json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def load_bus_addresses(def_path):
    """Extract per-bus address variants from a tile-definition JSON.

    Returns a dict keyed by bus name (upper-cased, e.g. "I2C"), each
    value a list of `{address, is_default}` entries pulled verbatim
    from `interfaces[name=<bus>].parameters.addresses`. Buses without
    an `addresses` list (I3C's dynamic assignment, SPI's CS-based
    selection) are omitted. Empty dict when the file is missing, the
    JSON has no interfaces, or no interface carries addresses — the
    frontend treats that as "one fixed address, no user choice".

    No schema change to the tile-def format; the data is already
    present for every tile with `parameters.addresses`. This helper
    just surfaces it onto the tile manifest so Studio can cap bus
    capacity and render an address selector per-row.
    """
    if def_path is None:
        return {}
    try:
        raw = json.loads(Path(def_path).read_text())
    except FileNotFoundError:
        print(f"warn: tile definition not found: {def_path}", file=sys.stderr)
        return {}
    except json.JSONDecodeError as err:
        print(f"warn: tile definition {def_path} is invalid JSON: {err}", file=sys.stderr)
        return {}

    out = {}
    for iface in raw.get("interfaces", []) or []:
        name = iface.get("name")
        if not isinstance(name, str):
            continue
        addrs = iface.get("parameters", {}).get("addresses", []) or []
        if not addrs:
            continue
        # Copy each entry to strip any extra fields the tile-def schema
        # may add later (keeps the manifest shape stable).
        #
        # Blank entries are skipped: the portal's tile editor needs a second
        # address row before it will let you mark one as default, so a tile
        # with exactly one address picks up an empty sibling. Passing that
        # through would show a blank alternate address in Studio and the docs.
        out[name] = [
            {
                "address": a["address"],
                **({"is_default": True} if a.get("is_default") else {}),
            }
            for a in addrs
            if isinstance(a, dict) and str(a.get("address") or "").strip()
        ]
        if not out[name]:
            del out[name]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if manifests on disk are out of sync")
    args = ap.parse_args()

    commit = source_commit()

    # Auto-discover every `sdk/core/core_*.h` header; anything without a
    # `@studio category` tag contributes nothing, so new modules opt in
    # simply by adding the tag. No hand-maintained source list.
    core_sources = sorted((ROOT / "sdk/core").glob("core_*.h"))
    core_hosts = []
    core_events = []
    core_categories = {}
    # Per-category docs — functions documented in each tagged header,
    # keyed by the category's canonical name (led, usb, adc, ...). The
    # website SDK pages consume these JSON files directly.
    sdk_docs = {}
    for p in core_sources:
        hosts, sections, docs, events = parse_header(p, scope="core")
        core_hosts.extend(hosts)
        # Attach the surrounding category to each event so Studio's DSL
        # codegen + block palette can group them under the right header.
        # Core events declared outside a `@studio category` block are
        # skipped with a warning — the palette needs somewhere to show them.
        if events:
            if len(sections) == 1:
                cat = next(iter(sections.keys()))
                for e in events:
                    e["category"] = cat
                core_events.extend(events)
            else:
                print(
                    f"warn: {p.name}: {len(events)} @studio event(s) but "
                    f"{len(sections)} @studio category declarations — "
                    f"events dropped (need exactly one category per file)",
                    file=sys.stderr,
                )
        for name, meta in sections.items():
            # First declaration wins — later duplicates are ignored so two
            # files claiming the same category don't silently clobber each
            # other. Logged for visibility.
            if name in core_categories and core_categories[name] != meta:
                print(f"warn: category '{name}' redeclared in {p.name}", file=sys.stderr)
                continue
            core_categories[name] = meta
            for fn in docs:
                fn["layer"] = "core"
            if name in sdk_docs:
                # A category may span headers (audio = core_audio.h +
                # core_pdm.h, usb = core_usb.h + core_usb_hid.h): append, in
                # file order, and list every header.
                doc = sdk_docs[name]
                doc["functions"].extend(docs)
                doc["headers"]["core"] += ", " + p.name
                continue
            sdk_docs[name] = {
                "schema": "studio-sdk-docs/v2",
                "source": f"tiles@{commit}",
                "category": name,
                "label": meta["label"],
                "icon": meta["icon"],
                "header": p.name,
                "headers": {"core": p.name},
                "functions": docs,
            }

    # Augment categories with their HAL / LL surface so the SDK reference can
    # render at whichever layer the reader works in. A category may be brand
    # new here (no core header — e.g. clocks, which is config-driven). See
    # LAYER_HEADERS.
    for category, spec in LAYER_HEADERS.items():
        doc = sdk_docs.get(category)
        if doc is None:
            meta = spec.get("meta", {})
            doc = {
                "schema": "studio-sdk-docs/v2",
                "source": f"tiles@{commit}",
                "category": category,
                "label": meta.get("label", category),
                "icon": meta.get("icon", ""),
                "header": None,
                "headers": {},
                "functions": [],
            }
            sdk_docs[category] = doc
        for layer in ("hal", "ll"):
            if layer not in spec:
                continue
            entries = spec[layer] if isinstance(spec[layer], list) else [spec[layer]]
            for fname, only in entries:
                doc["functions"].extend(
                    parse_layer_docs(LAYER_DIRS[layer] / fname, layer, only))
            doc["headers"][layer] = ", ".join(fname for fname, _ in entries)

    core_manifest = {
        "schema": "studio-manifest/v1",
        "source": f"tiles@{commit}",
        "categories": core_categories,
        "hosts": core_hosts,
        "events": core_events,
    }

    tile_sources = [
        {
            "path": ROOT / "drivers/tile_sense_hr.h",
            "definition": ROOT / "definitions/Sense-HR-a.json",
            "prefix": "tile_sense_hr",
            "init": "tile_sense_hr_init",
            "version": "1.0.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_cam_p.h",
            "definition": ROOT / "definitions/Sense-CAM-P-a.json",
            "prefix": "tile_sense_cam_p",
            "init": "tile_sense_cam_p_init",
            "version": "1.0.0",
        },
        {
            "path": ROOT / "drivers/tile_display_rgbw.h",
            "definition": ROOT / "definitions/Display-RGBW-a.json",
            "prefix": "tile_display_rgbw",
            "init": "tile_display_rgbw_init",
            "version": "2.5.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_i_6p6.h",
            "definition": ROOT / "definitions/Sense-I-6P6-a.json",
            "prefix": "tile_sense_i_6p6",
            "init": "tile_sense_i_6p6_init",
            "version": "1.3.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_adc_6.h",
            "definition": ROOT / "definitions/Sense-ADC-6-a.json",
            "prefix": "tile_sense_adc_6",
            "init": "tile_sense_adc_6_init",
            "version": "1.1.0",
        },
        {
            "path": ROOT / "drivers/tile_drive_h.h",
            "definition": ROOT / "definitions/Drive-H-a.json",
            "prefix": "tile_drive_h",
            "init": "tile_drive_h_init",
            "version": "4.4.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_mic.h",
            "definition": ROOT / "definitions/Sense-MIC-a.json",
            "prefix": "tile_sense_mic",
            "init": "tile_sense_mic_init",
            "version": "2.4.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_m_3g.h",
            "definition": ROOT / "definitions/Sense-M-3G-a.json",
            "prefix": "tile_sense_m_3g",
            "init": "tile_sense_m_3g_init",
            "version": "1.0.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_i_9.h",
            "definition": ROOT / "definitions/Sense-I-9-c.json",
            "prefix": "tile_sense_i_9",
            "init": "tile_sense_i_9_init",
            "version": "3.2.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_t_c.h",
            "definition": ROOT / "definitions/Sense-T-C-a.json",
            "prefix": "tile_sense_t_c",
            "init": "tile_sense_t_c_init",
            "version": "1.4.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_cap.h",
            "definition": ROOT / "definitions/Sense-CAP-a.json",
            "prefix": "tile_sense_cap",
            "init": "tile_sense_cap_init",
            "version": "1.0.0",
        },
        {
            "path": ROOT / "drivers/tile_drive_a_2.h",
            "definition": ROOT / "definitions/Drive-A-2-a.json",
            "prefix": "tile_drive_a_2",
            "init": "tile_drive_a_2_init",
            "version": "3.3.0",
        },
        {
            "path": ROOT / "drivers/tile_drive_p.h",
            "definition": ROOT / "definitions/Drive-P-a.json",
            "prefix": "tile_drive_p",
            "init": "tile_drive_p_init",
            "version": "3.4.0",
        },
        {
            "path": ROOT / "drivers/tile_power_l_1t.h",
            "definition": ROOT / "definitions/Power-L-1T-b.json",
            "prefix": "tile_power_l_1t",
            "init": "tile_power_l_1t_init",
            "version": "3.3.0",
        },
        {
            "path": ROOT / "drivers/tile_power_l_1n.h",
            "definition": ROOT / "definitions/Power-L-1N-a.json",
            "prefix": "tile_power_l_1n",
            "init": "tile_power_l_1n_init",
            "version": "1.2.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_acp.h",
            "definition": ROOT / "definitions/Sense-ACP-b.json",
            "prefix": "tile_sense_acp",
            "init": "tile_sense_acp_init",
            "version": "1.0.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_bp.h",
            "definition": ROOT / "definitions/Sense-BP-a.json",
            "prefix": "tile_sense_bp",
            "init": "tile_sense_bp_init",
            "version": "1.3.0",
        },
        {
            "path": ROOT / "drivers/tile_sense_tof.h",
            "definition": ROOT / "definitions/Sense-TOF-a.json",
            "prefix": "tile_sense_tof",
            "init": "tile_sense_tof_init",
            "version": "1.6.0",
        },
        {
            "path": ROOT / "drivers/tile_drive_dc_h.h",
            "definition": ROOT / "definitions/Drive-DC-H-a.json",
            "prefix": "tile_drive_dc_h",
            "init": "tile_drive_dc_h_init",
            "version": "4.3.0",
        },
        {
            "path": ROOT / "drivers/tile_store_o_128.h",
            "definition": ROOT / "definitions/Store-O-128-a.json",
            "prefix": "tile_store_o_128",
            "init": "tile_store_o_128_init",
            "version": "1.1.0",
        },
    ]

    targets = [(CORE_OUT_DIR / "core.json", core_manifest)]

    # Per-category SDK docs JSONs — one file per `@studio category`.
    for category, doc in sdk_docs.items():
        targets.append((SDK_DOCS_OUT_DIR / f"{category}.json", doc))

    # The SDK's own known gaps (`// @studio unsupported` in core headers), one
    # file for the site: it renders them per subsystem and on the status page.
    # Not a category (no `functions`), so gen_studio_natives skips it.
    gap_subsystems = [g for g in (parse_sdk_gaps(p) for p in core_sources) if g]
    targets.append((SDK_DOCS_OUT_DIR / "gaps.json", {
        "schema": "studio-sdk-gaps/v1",
        "source": f"tiles@{commit}",
        "subsystems": gap_subsystems,
    }))

    for t in tile_sources:
        hosts, sections, _docs, events = parse_header(t["path"], scope="tile")
        if len(sections) != 1:
            print(
                f"warn: {t['path'].name}: expected exactly one @studio tile tag, "
                f"found {len(sections)}",
                file=sys.stderr,
            )
        palette = next(iter(sections.values())) if sections else {
            "label": t["path"].stem, "icon": "",
        }
        manifest = {
            "schema": "studio-manifest/v1",
            "source": f"tiles@{commit}",
            "tile": palette["label"],
            "palette": palette,
            "driver": {
                "prefix": t["prefix"],
                "header": t["path"].name,
                "version": t["version"],
            },
            "handle": {"type": "tile_t", "init": t["init"]},
            "hosts": hosts,
            "events": events,
        }
        # Carry per-bus address variants from the tile-def JSON so
        # the frontend can cap how many instances of this tile fit on a
        # bus (I2C: one per address) and surface a variant selector when
        # the tile supports more than one. Absent/empty when the tile
        # has no definition file or none of its interfaces declare
        # addresses — the frontend treats that as "one fixed address,
        # no user choice".
        bus_addrs = load_bus_addresses(t.get("definition"))
        if bus_addrs:
            manifest["bus_addresses"] = bus_addrs
        targets.append((TILE_OUT_DIR / f"{t['path'].stem}.json", manifest))

    if VIBE_ERRORS:
        # Authoring errors in `@studio control` / `@studio require`. These never
        # degrade to warnings: a rule Studio cannot trust is worse than no rule.
        for err in VIBE_ERRORS:
            print(f"error: vibe settings: {err}", file=sys.stderr)
        sys.exit(1)

    if args.check:
        # The `source` sha tracks the commit, not the content: it legitimately
        # differs from HEAD between a regen and the commit that lands it (and
        # always differs in CI, which runs on a later commit). Normalise it on
        # both sides so --check flags only *structural* drift — a header edited
        # without regenerating its manifest.
        def strip_source(s):
            return re.sub(r'"source": "tiles@[^"]*"', '"source": "tiles@<sha>"', s)

        drift = []
        for path, data in targets:
            want = strip_source(serialize(data))
            have = strip_source(path.read_text()) if path.exists() else ""
            if have != want:
                drift.append(path)
        if drift:
            for p in drift:
                print(f"drift: {p}", file=sys.stderr)
            sys.exit(1)
        print("manifests up to date")
        return

    for path, data in targets:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(serialize(data))
        # Palette manifests carry `hosts`; SDK-docs carry `functions`.
        if "hosts" in data:
            summary = f"{len(data['hosts'])} hosts, {len(data.get('events', []))} events"
        elif "subsystems" in data:
            n = sum(len(x["gaps"]) for x in data["subsystems"])
            summary = f"{n} gaps in {len(data['subsystems'])} headers"
        else:
            summary = f"{len(data.get('functions', []))} functions"
        print(f"wrote {path.relative_to(ROOT)}  ({summary})")


if __name__ == "__main__":
    main()
