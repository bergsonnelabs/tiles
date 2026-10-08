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

A private overlay checkout's drivers (declared in its drivers/drivers.json)
are published from that repo instead: --overlay DIR writes (or checks)
DIR/manifests/tile-docs/ for them and touches nothing public.

Usage:
    python3 tools/gen_tile_docs.py            # write manifests/tile-docs/*.json
    python3 tools/gen_tile_docs.py --check    # fail if they're out of sync
    python3 tools/gen_tile_docs.py --overlay DIR [--check]
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
    "tile_sense_cam_p",
    "tile_sense_cap",
    "tile_sense_hr",
    "tile_sense_i_6p6",
    "tile_sense_i_9",
    "tile_sense_m_3g",
    "tile_sense_mic",
    "tile_sense_t_c",
    "tile_sense_tof",
    "tile_store_o_128",
]


def source_commit(cwd=ROOT):
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=cwd, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def serialize(data):
    return json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def strip_source(s):
    # The provenance sha tracks the commit, not the content — ignore it when
    # diffing so --check flags only real drift (same trick as gen_studio_manifest).
    return re.sub(r'"source": "[\w.-]+@[^"]*"', '"source": "<repo>@<sha>"', s)


def overlay_stems(overlay):
    """Header stems of the drivers a private overlay checkout declares in
    drivers/drivers.json (see coregen's overlay_driver_map)."""
    path = overlay / "drivers" / "drivers.json"
    try:
        drivers = json.loads(path.read_text()).get("drivers") or {}
    except (OSError, ValueError) as e:
        sys.exit(f"error: {path}: {e}")
    return sorted(Path(d["header"]).stem for d in drivers.values())


def build(stem, commit, drivers_dir=DRIVERS_DIR, repo="tiles"):
    path = drivers_dir / f"{stem}.h"
    if not path.exists():
        print(f"warn: {path.name} not found — skipping", file=sys.stderr)
        return None
    doc = parse_header(str(path))
    # Provenance + schema, mirroring the SDK-docs manifests.
    out = {"schema": "tile-docs/v1", "source": f"{repo}@{commit}"}
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
            # A hazardous value is documented by its label, like the default.
            if "hazard" in control:
                hz = control["hazard"]
                row["hazard"] = {"reason": hz["reason"]}
                if "values" in hz:
                    row["hazard"]["values"] = [
                        next((c["label"] for c in choices if c["value"] == v), v)
                        for v in hz["values"]
                    ]
                if "above" in hz:
                    # in the units the row's range is documented in
                    above = round(hz["above"] * control.get("scale", 1), 6)
                    row["hazard"]["above"] = int(above) if above == int(above) else above
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
    ap.add_argument("--overlay", metavar="DIR", type=Path,
                    help="a private overlay checkout: its own drivers' docs, into "
                         "DIR/manifests/tile-docs/, and nothing public")
    args = ap.parse_args()

    out_dir, base = OUT_DIR, ROOT
    if args.overlay:
        overlay = args.overlay.expanduser().resolve()
        out_dir, base = overlay / "manifests" / "tile-docs", overlay
        # Parse the public headers first (and discard them) so the enum types
        # they declare are known exactly as when these drivers lived here.
        for stem in ACTIVE_TILES:
            vibe_settings(DRIVERS_DIR / f"{stem}.h")
        commit = source_commit(overlay)
        builds = [(stem, build(stem, commit, overlay / "drivers", overlay.name))
                  for stem in overlay_stems(overlay)]
    else:
        commit = source_commit()
        builds = [(stem, build(stem, commit)) for stem in ACTIVE_TILES]

    targets = [(out_dir / f"{stem}.json", doc) for stem, doc in builds if doc is not None]

    if args.check:
        drift = []
        for path, data in targets:
            want = strip_source(serialize(data))
            have = strip_source(path.read_text()) if path.exists() else ""
            if have != want:
                drift.append(path)
        # Also flag stray files (a driver removed from ACTIVE_TILES).
        wanted = {p for p, _ in targets}
        for existing in out_dir.glob("*.json") if out_dir.exists() else []:
            if existing not in wanted:
                drift.append(existing)
        if drift:
            for p in drift:
                print(f"drift: {p}", file=sys.stderr)
            sys.exit(1)
        print(f"tile-docs up to date ({len(targets)} drivers)")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    for path, data in targets:
        path.write_text(serialize(data))
        n = len(data.get("functions", []))
        print(f"wrote {path.relative_to(base)}  ({n} functions)")


if __name__ == "__main__":
    main()
