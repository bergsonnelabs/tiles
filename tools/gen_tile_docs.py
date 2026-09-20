#!/usr/bin/env python3
"""Generate per-driver tile-docs manifests from the driver headers.

Sibling of gen_studio_manifest.py: that one emits the Studio palette + SDK-docs
manifests; this one emits the *tile-driver* documentation the website renders
(one JSON per active driver, under manifests/tile-docs/). The heavy lifting —
parsing Doxygen + @studio annotations into the ParsedDriver shape (brief,
description, code examples, defines, enums, structs, functions, events,
unsupported gaps) — lives in parse_driver_header.py.

The website used to read this data from a MySQL table populated out-of-band;
this makes it a committed, reviewable, CI-checked snapshot instead — same model
as the SDK docs. Add a driver to ACTIVE_TILES to publish it.

Usage:
    python3 tools/gen_tile_docs.py            # write manifests/tile-docs/*.json
    python3 tools/gen_tile_docs.py --check    # fail if they're out of sync
"""

import argparse
import json
import subprocess
import sys
import re
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parse_driver_header import parse_header  # noqa: E402


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
DRIVERS_DIR = ROOT / "drivers"
OUT_DIR = ROOT / "manifests" / "tile-docs"

# Drivers published to the docs site. Matches the palette manifest set in
# gen_studio_manifest.py; add a stem here once a driver is doc-ready.
ACTIVE_TILES = [
    "tile_display_rgbw",
    "tile_drive_a_2",
    "tile_drive_dc_h",
    "tile_drive_h",
    "tile_drive_p",
    "tile_power_l_1n",
    "tile_power_l_1t",
    "tile_sense_acp",
    "tile_sense_bp",
    "tile_sense_cam_p",
    "tile_sense_cap",
    "tile_sense_i_6p6",
    "tile_sense_i_9",
    "tile_sense_m_3g",
    "tile_sense_mic",
    "tile_sense_t_c",
    "tile_sense_tof",
    "tile_store_o_128",
]


def source_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def serialize(data):
    return json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def strip_source(s):
    # The provenance sha tracks the commit, not the content — ignore it when
    # diffing so --check flags only real drift (same trick as gen_studio_manifest).
    return re.sub(r'"source": "tiles@[^"]*"', '"source": "tiles@<sha>"', s)


def build(stem, commit):
    path = DRIVERS_DIR / f"{stem}.h"
    if not path.exists():
        print(f"warn: {path.name} not found — skipping", file=sys.stderr)
        return None
    doc = parse_header(str(path))
    # Provenance + schema, mirroring the SDK-docs manifests.
    out = {"schema": "tile-docs/v1", "source": f"tiles@{commit}"}
    out.update(doc)
    out.update(vibe_settings(path))
    return out


def vibe_settings(path):
    """The driver's `@studio control` / `@studio require` annotations as a table the
    docs site renders under "Vibe Settings" — the same data Studio's Vibe inspector
    and its assistant read, taken from the SAME parser (gen_studio_manifest), so the
    docs cannot describe a setting Studio doesn't have. Keys are only emitted for
    drivers that declare settings."""
    import gen_studio_manifest as gsm

    gsm.collect_enum_types(path.read_text())
    hosts, _sections, _docs, _events = gsm.parse_header(path, scope="tile")
    rows, rules = [], []
    for host in hosts:
        fn = host["qname"][-1]
        for control in host.get("controls", []):
            param = next(p for p in host["params"] if p["name"] == control["param"])
            choices = control.get("choices") or param.get("choices") or []
            row = {
                "label": control["label"],
                "tier": control["tier"],
                "scope": control["scope"],
                "function": fn,
                "param": control["param"],
                "kind": "bool" if control.get("type") == "bool" else "choice" if choices else "number",
            }
            if choices:
                row["choices"] = [c["label"] for c in choices]
            if param.get("range"):
                row["range"] = param["range"]
            if param.get("unit"):
                row["unit"] = param["unit"]
            # A scaled setting is documented the way people read it (30 to 130 dB),
            # not in the argument's sub-units (300..1300).
            if "scale" in control and row.get("range"):
                k = control["scale"]
                row["range"] = [round(v * k, 6) for v in row["range"]]
                row["range"] = [int(v) if v == int(v) else v for v in row["range"]]
                if control.get("display_unit"):
                    row["unit"] = control["display_unit"]
            if "default" in control:
                named = next((c["label"] for c in choices if c["value"] == control["default"]), None)
                row["default"] = named if named is not None else control["default"]
            if "show" in control:
                row["computed_unit"] = control.get("show_unit", "")
            if "role" in control:
                row["role"] = control["role"]
            rows.append(row)
        for rule in host.get("requires", []):
            rules.append({"function": fn, "message": rule["message"]})
    rows.sort(key=lambda r: r["tier"] != "basic")
    out = {}
    if rows:
        out["vibe_settings"] = rows
    if rules:
        out["vibe_rules"] = rules
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if manifests on disk are out of sync")
    args = ap.parse_args()
    commit = source_commit()

    targets = []
    for stem in ACTIVE_TILES:
        doc = build(stem, commit)
        if doc is None:
            continue
        targets.append((OUT_DIR / f"{stem}.json", doc))

    if args.check:
        drift = []
        for path, data in targets:
            want = strip_source(serialize(data))
            have = strip_source(path.read_text()) if path.exists() else ""
            if have != want:
                drift.append(path)
        # Also flag stray files (a driver removed from ACTIVE_TILES).
        wanted = {p for p, _ in targets}
        for existing in OUT_DIR.glob("*.json") if OUT_DIR.exists() else []:
            if existing not in wanted:
                drift.append(existing)
        if drift:
            for p in drift:
                print(f"drift: {p}", file=sys.stderr)
            sys.exit(1)
        print(f"tile-docs up to date ({len(targets)} drivers)")
        return

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for path, data in targets:
        path.write_text(serialize(data))
        n = len(data.get("functions", []))
        print(f"wrote {path.relative_to(ROOT)}  ({n} functions)")


if __name__ == "__main__":
    main()
