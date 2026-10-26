#!/usr/bin/env python3
"""
gen_studio_natives.py — generate WAMR NativeSymbol table +
adapter wrappers from `manifests/sdk-docs/*.json`.

For every function with `studio_exposed: true`, emits:

  1. A `static void|int|double core_X_native(wasm_exec_env_t, ...)`
     adapter that forwards from WAMR's calling convention (exec_env
     first, scalars after) to a direct call into the SDK.
  2. An entry in `g_studio_natives[]` pairing the host symbol name
     with the adapter pointer and the WAMR signature string
     ("()", "(i)", "(iii)", "(F)", "(i)i", etc.).

The firmware embedder registers everything in one call:

  extern const NativeSymbol g_studio_natives[];
  extern const size_t g_studio_natives_count;
  wasm_runtime_register_natives("env", g_studio_natives,
                                g_studio_natives_count);

Running this is an SDK maintenance step, not a firmware-build step.
The generated files are checked into `sdk/wamr/` so embedders don't
need Python at build time. Re-run when manifests change (currently
hand-edited; long-term should flow through the `@studio` annotation
generator).

Out-of-scope today:
  - Pointer parameters (`const char *`, arrays) — these need WAMR's
    `wasm_runtime_validate_app_addr` + `addr_app_to_native` dance
    and a per-import scheme. Skipped with a comment; the skipped
    set will come back when a real `Core.USB.print(string)` path
    is designed.
  - Hosts that return a pointer.
  - Functions without a `signature` entry in the manifest (rare;
    older manifests).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


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


REPO_ROOT = Path(__file__).resolve().parent.parent
SDK_DOCS = REPO_ROOT / "manifests" / "sdk-docs"
DEFAULT_OUT_C = REPO_ROOT / "sdk" / "wamr" / "studio_natives.c"
DEFAULT_OUT_H = REPO_ROOT / "sdk" / "wamr" / "studio_natives.h"


# ---------------------------------------------------------------------------
# Type mapping
#
# Keep this table narrow on purpose. SDK ctypes that aren't here get the
# function skipped with a diagnostic — better to exclude loudly than to
# silently mis-lower. The C API's own types.h is authoritative; this mirror
# tracks it by hand for now.
# ---------------------------------------------------------------------------
I32_TYPES = {
    "int", "unsigned int", "bool",
    "int8_t", "int16_t", "int32_t",
    "uint8_t", "uint16_t", "uint32_t",
    "hal_status_t",
    "core_reset_reason_t",   # an enum (core_reset.h): an int across the boundary
}

# Categories whose adapter wrappers reach into coregen-generated
# per-project state (handles like `core_adc1`, dispatchers like
# `core_pwm_timer_for_pad`). Those can only be wrapped once we
# know the project's config.json — a later per-project generation
# step. For now we exclude them from the cross-project catalog so
# `studio_natives.c` links unconditionally.
#
# Safe categories: inline helpers that only reach into the ll_*
# layer (no config-generated globals) or into HAL code that's
# always compiled.
PROJECT_INDEPENDENT_CATEGORIES = {
    "led",
    "timing",
    "watchdog",
    "nvm",
    "usb",
    # rtc and backup were on the per-project list but a closer audit
    # (Phase D-0) showed every Tier-2 entry resolves through tal_rtc /
    # ll_rtc / ll_pwr — no coregen-emitted handles or PAD_*_PORT
    # macros. Safe to wrap directly here.
    "rtc",
    "backup",
    # rng has no per-project state either — core_rng_read calls
    # ll_rng_read which is plain SDK code. Added with the RNG Tier 2
    # `read32` exposure.
    "rng",
    # power exposes core_sleep / core_stop_for / core_woke_from_standby.
    # Each calls into ll_pwr / ll_rtc / ll_rcc only (no per-project
    # handles), so the static SDK table is the right home.
    "power",
}
F64_TYPES = {"double"}
F32_TYPES = {"float"}  # WAMR uses 'f' for f32, 'F' for f64 in sig strings

# Null-terminated string params. WAMR's `$` signature char tells the
# runtime to validate the wasm pointer (find the bounds + the null
# terminator) and pass a `const char *` straight through to the C
# adapter. No manual `validate_app_addr` needed; the wrapper just
# declares the param as `const char *` and forwards.
STRING_TYPES = {"const char *", "char *"}

# Pointer types we still don't have an ABI for. Buffer / out-pointer
# params (`uint8_t *`, `void *`) need the multi-scalar-out / array-OUT
# story already prototyped on the tile-driver side; not in scope here.
POINTER_TYPES_SKIP = {"void *", "uint8_t *"}


@dataclass
class HostFn:
    category: str
    name: str            # C symbol (e.g. "core_led_heartbeat")
    params: list[dict]   # each { "name", "ctype" }
    returns: str         # C return type; "void" if none

    @property
    def wamr_signature(self) -> str:
        pchars = "".join(wamr_char(p["ctype"]) for p in self.params)
        rt = self.returns.strip()
        if rt == "void":
            return f"({pchars})"
        return f"({pchars}){wamr_char(rt)}"

    @property
    def c_return(self) -> str:
        return self.returns.strip()

    def skip_reason(self, *, mode: str = "static") -> str | None:
        in_independent = self.category in PROJECT_INDEPENDENT_CATEGORIES
        if mode == "static" and not in_independent:
            return f"category '{self.category}' reaches into coregen-generated state; needs per-project generation"
        if mode == "project" and in_independent:
            return f"category '{self.category}' is project-independent; wrapped in the static table"
        for p in self.params:
            ct = p.get("ctype", "").strip()
            if ct in POINTER_TYPES_SKIP:
                return f"pointer param '{ct}' not yet supported"
            if wamr_char(ct) is None:
                return f"param type '{ct}' not in the i32/f32/f64 map"
        rt = self.c_return
        if rt != "void" and wamr_char(rt) is None:
            return f"return type '{rt}' not in the i32/f32/f64 map"
        return None


def wamr_char(ctype: str) -> str | None:
    t = ctype.strip()
    if t in I32_TYPES:
        return "i"
    if t in STRING_TYPES:
        return "$"
    if t in F32_TYPES:
        return "f"
    if t in F64_TYPES:
        return "F"
    return None


def c_param_type(ctype: str) -> str:
    """Adapter-side param type. i32s come in as int32_t, floats as
    double (WAMR promotes f32 native args to f64 on the C boundary for
    variadic safety — but the adapter just forwards, so we use the
    manifest's declared ctype directly)."""
    return ctype.strip()


def load_manifests(sdk_docs: Path) -> list[HostFn]:
    out: list[HostFn] = []
    for path in sorted(sdk_docs.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        cat = data.get("category", path.stem)
        for fn in data.get("functions", []):
            if not fn.get("studio_exposed"):
                continue
            out.append(HostFn(
                category=cat,
                name=fn["name"],
                params=[{"name": p.get("name", f"_p{i}"),
                         "ctype": p.get("ctype", "int")}
                        for i, p in enumerate(fn.get("params", []))],
                returns=fn.get("returns", "void"),
            ))
    return out


# ---------------------------------------------------------------------------
# Emission
# ---------------------------------------------------------------------------

WARNING = (
    "/* ---------------------------------------------------------------------\n"
    " * Generated by tools/gen_studio_natives.py - DO NOT EDIT BY HAND.\n"
    " * Regenerate when manifests/sdk-docs/ JSON files change.\n"
    " * ------------------------------------------------------------------ */\n"
)


def emit_wrapper(fn: HostFn) -> str:
    """A `static` wrapper that matches WAMR's native-call convention
    and forwards to the real SDK symbol. Marked `static` so LTO +
    gc-sections can drop wrappers for hosts the user's Wasm module
    never actually imports — the NativeSymbol table entry keeps
    them reachable from the register path, but `register_natives`
    only validates signatures, it doesn't force-link them."""
    sig = fn.wamr_signature
    rt = fn.c_return
    params = fn.params

    if not params:
        param_decl = "wasm_exec_env_t env"
    else:
        parts = ["wasm_exec_env_t env"]
        for p in params:
            parts.append(f"{c_param_type(p['ctype'])} {p['name']}")
        param_decl = ", ".join(parts)

    call_args = ", ".join(p["name"] for p in params)
    ret_decl = "void" if rt == "void" else rt
    ret_keyword = "" if rt == "void" else "return "

    lines = [
        f"/* {fn.name}  {sig}  (category: {fn.category}) */",
        f"static {ret_decl} {fn.name}_native({param_decl})",
        "{",
        "    (void)env;",
        f"    {ret_keyword}{fn.name}({call_args});",
        "}",
    ]
    return "\n".join(lines)


def table_symbol(mode: str) -> str:
    return "g_studio_natives_project" if mode == "project" else "g_studio_natives"


def emit_table(fns: list[HostFn], *, mode: str = "static") -> str:
    sym = table_symbol(mode)
    lines = [
        "/* NOT const: WAMR's `wasm_runtime_register_natives` calls",
        " * `qsort` on the table in place (see wasm_native.c's",
        " * register_natives). If the array sits in .rodata the",
        " * qsort writes silently no-op on Cortex-M and WAMR's",
        " * bsearch lookup then fails with \"failed to call unlinked",
        " * import function\" — even though every symbol is present.",
        " * Leaving it writable puts the table in .data (RAM-backed)",
        " * where the sort runs correctly at startup. */",
        f"NativeSymbol {sym}[] = {{",
    ]
    for fn in fns:
        guard = adapter_guard(fn) if mode == "project" else None
        if guard:
            lines.append(f"#ifdef {guard}")
        # Signature strings need one layer of parentheses exactly —
        # `wamr_signature` already wraps in `()`, so pass through.
        lines.append(
            f'    {{ "{fn.name}", (void *){fn.name}_native, "{fn.wamr_signature}", NULL }},'
        )
        if guard:
            lines.append("#endif")
    lines += [
        "};",
        "",
        f"const size_t {sym}_count =",
        f"    sizeof({sym}) / sizeof({sym}[0]);",
    ]
    return "\n".join(lines)


def emit_c(
    fns: list[HostFn],
    skipped: list[tuple[HostFn, str]],
    *,
    mode: str = "static",
) -> str:
    header_includes = sorted({f'#include "core_{fn.category}.h"' for fn in fns})

    parts = [
        WARNING,
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        '#include "wasm_export.h"',
        "",
    ]

    if mode == "static":
        parts += [
            "/* The LL layer's *_BASE macros (RCC_BASE, AHB1_BASE, …) are",
            " * defined via per-MCU branches in ll_common.h + ll_rcc.h and",
            " * referenced transitively by ll_pwr.h and friends. Projects",
            " * normally reach them via their auto-generated `core.h`, but",
            " * this TU is project-independent so we pull them in directly. */",
            '#include "ll_common.h"',
            '#include "ll_rcc.h"',
            "",
        ]
    else:
        parts += [
            "/* Project-side WAMR natives — wraps the Tier 2 functions whose",
            " * adapters reach into coregen-emitted state (PAD_*_PORT macros,",
            " * core_dac / core_adc1 externs, core_pwm_timer_for_pad dispatcher).",
            " * Compiled per project alongside core_init.c so all that state is",
            " * in scope. Registered via wasm_runtime_register_natives at the",
            " * same time as the static SDK table. */",
            '#include "core.h"   /* coregen umbrella: pulls in core_pads.h + externs */',
            "",
        ]

    parts.append("/* SDK headers providing the host symbols we wrap. */")
    if mode == "project":
        # Each per-project category guards its include with the matching
        # CORE_USES_* project flag. core_dac.h has a hard `#error` on non-H
        # cores, so guarding the include is mandatory there. PWM is
        # guarded because tal_timer.h references core_pad_timer_info,
        # which coregen only emits when the project has TIM pads.
        cats = sorted({fn.category for fn in fns})
        for cat in cats:
            guard = ADAPTER_GUARDS.get(cat)
            if guard:
                parts.append(f"#ifdef {guard}")
                parts.append(f'#include "core_{cat}.h"')
                parts.append("#endif")
            else:
                parts.append(f'#include "core_{cat}.h"')
    else:
        parts += header_includes
    parts.append("")

    if skipped:
        parts.append("/* Skipped at generation time:")
        for fn, reason in skipped:
            parts.append(f" *   - {fn.name}: {reason}")
        parts.append(" */")
        parts.append("")

    parts.append("/* ---- Adapters ---------------------------------------------------------- */")
    parts.append("")
    for fn in fns:
        guard = adapter_guard(fn) if mode == "project" else None
        if guard:
            parts.append(f"#ifdef {guard}")
        parts.append(emit_wrapper(fn))
        if guard:
            parts.append("#endif")
        parts.append("")

    parts.append("/* ---- NativeSymbol table ----------------------------------------------- */")
    parts.append("")
    parts.append(emit_table(fns, mode=mode))
    parts.append("")
    return "\n".join(parts)


# Per-project adapters compile only when the project actually configures
# the relevant peripheral. The matching CORE_USES_* project flags live in
# coregen's core_pads.h.j2 (TIMER_PADS / ADC_PADS / DAC). They say what the
# project uses; the Core's own capabilities are CORE_HAS_* (core_caps.h).
# Pad / GPIO has no guard — hal_pad_lookup() returns NULL for unmapped
# pads, so the adapter compiles cleanly even on projects with no GPIO.OUT
# pads.
ADAPTER_GUARDS = {
    "pwm": "CORE_USES_TIMER_PADS",
    "adc": "CORE_USES_ADC_PADS",
    "dac": "CORE_USES_DAC",
    # i2c Tier 2 wrappers consult core_i2c_handle_for_bus, which only
    # exists in coregen output when the project declares at least one
    # I2C bus. Sentinel emitted by core_pads.h.j2 alongside the others.
    "i2c": "CORE_USES_I2C_BUSES",
    "spi": "CORE_USES_SPI_BUSES",
    "serial": "CORE_USES_SERIAL_BUSES",
}


def adapter_guard(fn: HostFn) -> str | None:
    return ADAPTER_GUARDS.get(fn.category)


def emit_h(fns: list[HostFn], *, mode: str = "static") -> str:
    sym = table_symbol(mode)
    guard = "STUDIO_NATIVES_PROJECT_H" if mode == "project" else "STUDIO_NATIVES_H"
    parts = [
        WARNING,
        "",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stddef.h>",
        "",
        '#include "wasm_export.h"',
        "",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
        "",
        f"/* {len(fns)} Studio-exposed host symbols ({mode} table). */",
        f"extern NativeSymbol {sym}[];",
        f"extern const size_t {sym}_count;",
        "",
        "#ifdef __cplusplus",
        "}",
        "#endif",
        "",
        f"#endif /* {guard} */",
        "",
    ]
    return "\n".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdk-docs", type=Path, default=SDK_DOCS)
    ap.add_argument("--out-c", type=Path, default=DEFAULT_OUT_C)
    ap.add_argument("--out-h", type=Path, default=DEFAULT_OUT_H)
    ap.add_argument(
        "--mode",
        choices=("static", "project"),
        default="static",
        help="static: project-independent SDK table; project: coregen "
             "per-project sibling table (pad/pwm/adc/dac).",
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    all_fns = load_manifests(args.sdk_docs)
    kept: list[HostFn] = []
    skipped: list[tuple[HostFn, str]] = []
    for fn in all_fns:
        reason = fn.skip_reason(mode=args.mode)
        if reason:
            skipped.append((fn, reason))
        else:
            kept.append(fn)

    print(f"[gen_studio_natives:{args.mode}] {len(kept)} exposed, {len(skipped)} skipped",
          file=sys.stderr)
    for fn, reason in skipped:
        print(f"  skip {fn.name}: {reason}", file=sys.stderr)

    c_text = emit_c(kept, skipped, mode=args.mode)
    h_text = emit_h(kept, mode=args.mode)

    if args.dry_run:
        print("# --- header ---")
        print(h_text)
        print("# --- source ---")
        print(c_text)
        return 0

    args.out_c.parent.mkdir(parents=True, exist_ok=True)
    args.out_c.write_text(c_text, encoding="utf-8")
    args.out_h.write_text(h_text, encoding="utf-8")
    print(f"[gen_studio_natives] wrote {args.out_c.relative_to(REPO_ROOT)} + "
          f"{args.out_h.relative_to(REPO_ROOT)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
