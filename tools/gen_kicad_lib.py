#!/usr/bin/env python3
"""
gen_kicad_lib.py — KiCad 9 symbol library for tiles, from definitions/.

Writes one symbol per tile into a `.kicad_sym`. The public copy lives in
bergsonnelabs/bergsonne-kicad next to its hand-drawn footprints, and the
nightly `sync-definitions` workflow keeps it current. Footprints are not
generated: each symbol names `Bergsonne Tiles:<package>` (e.g. T44-10), and
the footprint library under that nickname supplies the pads.

Layout (agreed 2026-09-28):
  1. Supply pins on top, from power[]: V+ first, then the other rails in pad
     order, each followed by its own ground (a gnd_pad no other rail uses).
     Shared grounds end the block.
  2. A gap, then every other pad in pad order.
  3. Pin name: the port name on Cores (A0, H3), else the pad's first function.
     Unused pads are hidden no_connect pins with no name.
  4. Text beside each pin: the rail's range, then its alternate functions
     (filtered to the common peripherals). Every function is also a native
     KiCad pin alternate, pickable in the schematic.
  5. Name above the body; rev a is unsuffixed (Drive.H), later revs carry it
     (Core.ST.W5-b). Fields: Reference, Value, Footprint (hidden, just under
     the body) and search keywords only.

Which tiles go in is decided by status (production/beta for the public
library). Status lives only in the product DB, so it comes in as a JSON map
`{definition filename: status}` that `sync_definitions.py --statuses-out`
writes; it is deliberately not added to the public definition files.

`--symbols-dir` writes each tile revision's symbol block on its own instead,
as `<dir>/<Family.Name>/symbol-<rev>.kicad_sym`, for every definition with
pads in the `--definitions` dirs (a later dir's copy of a revision wins, as in
twins/scripts/build-bundle.mjs, which ships them in the Studio bundle). A block
is exactly what `library()` puts between its header and the closing paren, so
blocks concatenated inside that header make a library.

Usage:
    python3 tools/gen_kicad_lib.py --statuses s.json --include production,beta \\
        --footprints "<lib>/Bergsonne Tiles.pretty" --out "<lib>/Bergsonne Tiles.kicad_sym"
    python3 tools/gen_kicad_lib.py --tiles <Family>-<Name>-a.json --out private.kicad_sym
    python3 tools/gen_kicad_lib.py --symbols-dir <dir> --definitions definitions <overlay>/definitions ...
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import sys
from pathlib import Path

DEFINITIONS_DIR = Path(__file__).resolve().parent.parent / "definitions"

G = 2.54         # pin pitch and grid
FONT = 1.27
CHAR = 1.05      # rough advance of KiCad's stroke font at 1.27 mm (body text)
NAME_CHAR = 1.3  # pin names are mostly capitals, which run wider
PIN_LEN = 2.54

PORT = re.compile(r"^[A-K]\d{1,2}$")
# Body text shows only these peripherals; the pin's native alternates carry all.
SHOWN = re.compile(r"^(I2C|I3C|SPI|QSPI|OSPI|USART|UART|LPUART|ADC|DAC|TIM\d|SWD|SWCLK|SWDIO|BOOT|USB|CAN|FDCAN)")


# ---- layout ---------------------------------------------------------------

def rail_range(r: dict) -> str:
    lo, hi = r.get("min"), r.get("max")
    if lo in (None, "") and hi in (None, ""):
        return ""
    if lo == hi or hi in (None, ""):
        return f"{lo:g}V"
    return f"{lo:g}-{hi:g}V"


def symbol_name(d: dict) -> str:
    return f"{d['family']}.{d['name']}" + ("" if d["rev"] == "a" else f"-{d['rev']}")


def package(d: dict) -> str:
    return f"{d['package']['type']}-{d['package']['pads']}"


def layout(d: dict) -> tuple[list[dict], list[dict]]:
    """(power block, other pins), each pin {pad, name, rail, alts}."""
    pads = sorted(d["pads"], key=lambda p: int(p["pad"]))
    funcs = {p["pad"]: [f["function"] for f in p["functions"] if f["function"]] for p in pads}

    rails = [r for r in d.get("power", []) if r.get("positive_pad")]
    # V+ first; every other rail in pad order. (power[].type doesn't help:
    # most rails are "system", e.g. Sense.HR's VLED as well as its V+.)
    rails.sort(key=lambda r: (not any(funcs.get(x, [])[:1] == ["V+"] for x in r["positive_pad"]),
                              min(int(x) for x in r["positive_pad"])))
    gnd_use = collections.Counter(g for r in rails for g in set(r.get("gnd_pad", [])))
    note: dict[str, str] = {}
    block: list[str] = []
    for r in rails:
        for p in sorted(r["positive_pad"], key=int):
            if p not in block:
                block.append(p)
                note[p] = rail_range(r)
        for g in sorted(r.get("gnd_pad", []), key=int):
            if gnd_use[g] == 1 and g not in block:
                block.append(g)
    for r in rails:
        for g in sorted(r.get("gnd_pad", []), key=int):
            if g not in block:
                block.append(g)
    for p in pads:  # a GND pad with no power[] entry still belongs up here
        if p["pad"] not in block and funcs[p["pad"]][:1] == ["GND"]:
            block.append(p["pad"])

    def pin(p: str) -> dict:
        fs = funcs[p]
        port = [f for f in fs if PORT.match(f)] if d["family"] == "Core" else []
        name = port[0] if port else (fs[0] if fs else "")
        return {"pad": p, "name": name, "rail": note.get(p, ""), "alts": [f for f in fs if f != name]}

    return [pin(p) for p in block], [pin(p["pad"]) for p in pads if p["pad"] not in block]


# ---- KiCad ----------------------------------------------------------------

def q(s: str) -> str:
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def snap(v: float, step: float = G) -> float:
    return round(-(-v // step) * step, 4)


def fx(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") or "0"


def ftype(f: dict, multi: bool = False) -> str:
    if multi:
        return "bidirectional"
    dirn, typ = f.get("direction") or "", f.get("type") or ""
    if dirn in ("input", "output", "bidirectional"):
        return dirn
    return "bidirectional" if typ == "digital" else "passive"


def etype(d: dict, pad: str, name: str, rails: dict, gnds: set) -> str:
    """KiCad electrical type for a pad, from its power role or its functions."""
    if pad in gnds:
        return "power_in"
    if pad in rails:
        return {"output": "power_out", "input": "power_in"}.get(rails[pad].get("direction"), "passive")
    fs = [f for f in next(p["functions"] for p in d["pads"] if p["pad"] == pad) if f["function"]]
    if not fs:
        return "no_connect"
    return ftype(next((f for f in fs if f["function"] == name), fs[0]), multi=len(fs) > 1)


def compact(alts: list[str]) -> list[str]:
    """TIM1.2N, TIM3.3 -> TIM1.2N/3.3 (the old Eagle Core.W notation)."""
    out: list[str | None] = []
    tims: list[str] = []
    for a in alts:
        m = re.match(r"^TIM(\d+\..+)$", a)
        if m:
            if not tims:
                out.append(None)  # keeps the first timer's position
            tims.append(m.group(1))
        else:
            out.append(a)
    return [("TIM" + "/".join(tims)) if a is None else a for a in out]


def symbol(d: dict) -> str:
    power, signals = layout(d)
    rails: dict[str, dict] = {}
    gnds: set[str] = set()
    for r in d.get("power", []):
        for p in r.get("positive_pad", []):
            rails[p] = r
        gnds |= set(r.get("gnd_pad", []))
    for p in d["pads"]:
        if [f["function"] for f in p["functions"]][:1] == ["GND"]:
            gnds.add(p["pad"])

    rows: list[dict | None] = list(power) + ([None] if power and signals else []) + list(signals)
    pins_only = [p for p in rows if p]

    def body_text(p: dict) -> str:
        return ", ".join(([p["rail"]] if p["rail"] else []) + compact([a for a in p["alts"] if SHOWN.match(a)]))

    name = symbol_name(d)
    alt_x = snap(0.762 + max([len(p["name"]) for p in pins_only] + [1]) * NAME_CHAR + G, FONT)
    text_w = max([len(body_text(p)) * CHAR for p in pins_only] + [0])
    width = snap(max(alt_x + text_w + FONT, len(name) * CHAR * 1.2 + G, 5 * G))

    y = -G  # first pin row
    pins, texts = [], []
    for p in rows:
        if p is None:
            y -= G
            continue
        et = etype(d, p["pad"], p["name"], rails, gnds)
        fs = next(x for x in d["pads"] if x["pad"] == p["pad"])["functions"]
        alts = "".join(f"\n\t\t\t\t(alternate {q(f['function'])} {ftype(f)} line)"
                       for f in fs if f["function"] and f["function"] != p["name"])
        pins.append(
            f"\t\t\t(pin {et} line\n\t\t\t\t(at {fx(-PIN_LEN)} {fx(y)} 0)\n\t\t\t\t(length {fx(PIN_LEN)})"
            + ("\n\t\t\t\t(hide yes)" if et == "no_connect" else "")
            + f"\n\t\t\t\t(name {q(p['name'])}\n\t\t\t\t\t(effects (font (size {FONT} {FONT})))\n\t\t\t\t)"
            f"\n\t\t\t\t(number {q(p['pad'])}\n\t\t\t\t\t(effects (font (size {FONT} {FONT})))\n\t\t\t\t){alts}\n\t\t\t)"
        )
        bt = body_text(p)
        if bt:
            texts.append(f"\t\t\t(text {q(bt)}\n\t\t\t\t(at {fx(alt_x)} {fx(y)} 0)"
                         f"\n\t\t\t\t(effects (font (size {FONT} {FONT})) (justify left))\n\t\t\t)")
        y -= G
    bottom = y  # one row of margin under the last pin

    keywords = sorted({f["function"] for p in d["pads"] for f in p["functions"] if f["function"]})
    fields = [
        ("Reference", "T", (0, 3.81), False, "left"),
        ("Value", name, (0, FONT), False, "left"),
        ("Footprint", f"Bergsonne Tiles:{package(d)}", (0, bottom - FONT), True, "left"),
        ("Datasheet", "", (0, 0), True, None),
        ("Description", "", (0, 0), True, None),
        ("ki_keywords", " ".join(["tile", "bergsonne"] + keywords), (0, 0), True, None),
    ]
    fld = []
    for k, v, (x, fy), hide, just in fields:
        eff = f"(font (size {FONT} {FONT}))" + (f" (justify {just})" if just else "") + (" (hide yes)" if hide else "")
        fld.append(f"\t\t(property {q(k)} {q(v)}\n\t\t\t(at {fx(x)} {fx(fy)} 0)\n\t\t\t(effects {eff})\n\t\t)")

    return (
        f"\t(symbol {q(name)}\n\t\t(pin_names (offset 0.762))\n\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n"
        + "\n".join(fld)
        + f"\n\t\t(symbol {q(name + '_0_1')}"
        f"\n\t\t\t(rectangle (start 0 0) (end {fx(width)} {fx(bottom)})"
        f" (stroke (width 0.254) (type default)) (fill (type background)))\n\t\t)"
        f"\n\t\t(symbol {q(name + '_1_1')}\n" + "\n".join(pins + texts) + "\n\t\t)\n\t\t(embedded_fonts no)\n\t)"
    )


LIB_HEADER = ('(kicad_symbol_lib\n\t(version 20241209)\n\t(generator "bergsonne_gen_kicad_lib")\n'
              '\t(generator_version "9.0")\n')


def library(defs: list[dict]) -> str:
    syms = [symbol(d) for d in sorted(defs, key=symbol_name)]
    return LIB_HEADER + "\n".join(syms) + "\n)\n"


def write_symbol_blocks(dirs: list[Path], out: Path) -> int:
    """One `<out>/<Family.Name>/symbol-<rev>.kicad_sym` per tile revision with
    pads, from every definition in `dirs` (later dirs win). Each file is the
    block plus a newline, so LIB_HEADER + the files + ")\\n" is a library."""
    defs: dict[tuple[str, str], dict] = {}
    for d in dirs:
        for path in sorted(Path(d).glob("*.json")):
            x = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(x, dict) and x.get("family") and x.get("name") and x.get("rev"):
                defs[(f"{x['family']}.{x['name']}", x["rev"])] = x
    n = 0
    for (tile, rev), x in sorted(defs.items()):
        if not x.get("pads"):
            continue
        dest = out / tile / f"symbol-{rev}.kicad_sym"
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(symbol(x) + "\n", encoding="utf-8")
        n += 1
    return n


# ---- CLI ------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description="Generate the tiles KiCad symbol library.")
    ap.add_argument("--out", help="the .kicad_sym to write")
    ap.add_argument("--definitions", type=Path, nargs="+", default=[DEFINITIONS_DIR],
                    help="definition dirs; a later dir's file of the same name wins")
    ap.add_argument("--symbols-dir", type=Path,
                    help="write one symbol block per tile revision here instead of a library")
    ap.add_argument("--statuses", type=Path, help="JSON {definition filename: status}")
    ap.add_argument("--include", default="production,beta", help="statuses to include (with --statuses)")
    ap.add_argument("--tiles", nargs="+", help="explicit definition filenames instead of --statuses")
    ap.add_argument("--footprints", type=Path,
                    help="footprint library (.pretty); tiles whose package is missing there are skipped")
    args = ap.parse_args()

    if args.symbols_dir:
        if args.out or args.statuses or args.tiles or args.footprints:
            ap.error("--symbols-dir takes only --definitions")
        n = write_symbol_blocks(args.definitions, args.symbols_dir)
        print(f"wrote {n} symbol blocks to {args.symbols_dir}")
        return 0
    if not args.out:
        ap.error("--out is required (or give --symbols-dir)")
    if bool(args.statuses) == bool(args.tiles):
        ap.error("give exactly one of --statuses or --tiles")
    if args.tiles:
        wanted = list(args.tiles)
    else:
        include = {s.strip() for s in args.include.split(",") if s.strip()}
        statuses = json.loads(args.statuses.read_text(encoding="utf-8"))
        wanted = sorted(f for f, s in statuses.items() if s in include)

    packages = None
    if args.footprints:
        packages = {p.stem for p in args.footprints.glob("*.kicad_mod")}

    defs, skipped = [], []
    for fname in wanted:
        path = next((p for p in (d / fname for d in reversed(args.definitions)) if p.exists()),
                    args.definitions[-1] / fname)
        if not path.exists():
            skipped.append(f"{fname}: no definition file")
            continue
        d = json.loads(path.read_text(encoding="utf-8"))
        if not d.get("pads"):
            skipped.append(f"{fname}: no pads in the DB yet")
        elif packages is not None and package(d) not in packages:
            skipped.append(f"{fname}: no footprint {package(d)}")
        else:
            defs.append(d)

    Path(args.out).write_text(library(defs), encoding="utf-8")
    for s in skipped:
        print(f"  SKIP  {s}")
    print(f"wrote {len(defs)} symbols to {args.out} ({len(skipped)} skipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
