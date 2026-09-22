#!/usr/bin/env python3
"""
gen_reference_data.py — derive public tile reference tables from the canonical
tile definitions.

Walks definitions/*.json and emits a single committed JSON the website renders
on its Resources pages (I2C addresses + pad assignments). Core tiles are
excluded: they're fully programmable, so their pad functions and bus addresses
are config-defined per project (see coregen), not fixed product specs.

Same model as gen_tile_docs.py: author the definitions, derive the data, re-sync
and the pages update. No database at request time.

Usage:
    python3 tools/gen_reference_data.py            # write manifests/reference/tiles.json
    python3 tools/gen_reference_data.py --check     # CI: fail if the manifest is stale
"""
import glob
import json
import os
import re
import subprocess
import sys


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

OUT = "manifests/reference/tiles.json"

# Display order for families on the reference pages (Core excluded).
FAMILY_ORDER = ["Sense", "Drive", "Power", "Link", "Display", "Store", "Special"]


def family_rank(f):
    return FAMILY_ORDER.index(f) if f in FAMILY_ORDER else len(FAMILY_ORDER)


def i2c_interfaces(defn):
    """Default + alternate I2C addresses per I2C interface on the tile."""
    out = []
    for iface in defn.get("interfaces", []):
        if iface.get("type") != "I2C":
            continue
        addrs = (iface.get("parameters") or {}).get("addresses") or []
        # Skip blank rows: the portal editor requires a second address row to
        # mark one default, so single-address tiles carry an empty sibling.
        addrs = [a for a in addrs if str((a or {}).get("address") or "").strip()]
        if not addrs:
            continue
        if len(addrs) == 1:
            default, alt = addrs[0]["address"], []
        else:
            explicit = next((a for a in addrs if a.get("is_default")), None)
            default = explicit["address"] if explicit else addrs[0]["address"]
            alt = [a["address"] for a in addrs if a["address"] != default]
        out.append({"name": iface.get("name"), "default": default, "alt": alt})
    return out


def pad_functions(defn):
    """Per-pad default function + alternates (first listed function is default)."""
    pads = {}
    for p in defn.get("pads", []):
        fns = p.get("functions", [])
        if not fns:
            continue
        entries = [{"name": f.get("function"), "type": f.get("type")} for f in fns]
        pads[str(p.get("pad"))] = {"default": entries[0], "alt": entries[1:]}
    return pads


def build():
    tiles = []
    for path in sorted(glob.glob("definitions/*.json")):
        defn = json.load(open(path))
        if defn.get("family") == "Core":
            continue
        tiles.append(
            {
                "family": defn.get("family"),
                "name": defn.get("name"),
                "rev": defn.get("rev"),
                "headline": defn.get("headline", ""),
                "i2c": i2c_interfaces(defn),
                "pads": pad_functions(defn),
            }
        )
    tiles.sort(key=lambda t: (family_rank(t["family"]), t["name"]))

    commit = "unknown"
    try:
        commit = (
            subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
        )
    except Exception:
        pass
    return {"schema": "reference/v1", "source": f"tiles@{commit}", "tiles": tiles}


def strip_source(s):
    """Ignore the provenance sha when comparing for --check."""
    return re.sub(r'"source":\s*"tiles@[^"]*"', '"source": "tiles@*"', s)


def main():
    data = build()
    text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    if "--check" in sys.argv:
        if not os.path.exists(OUT):
            print(f"ERROR: {OUT} missing — run: python3 tools/gen_reference_data.py")
            sys.exit(1)
        current = open(OUT, encoding="utf-8").read()
        if strip_source(current) != strip_source(text):
            print(f"ERROR: {OUT} is stale — run: python3 tools/gen_reference_data.py")
            sys.exit(1)
        print(f"OK: {OUT} in sync ({len(data['tiles'])} tiles)")
        return
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, "w", encoding="utf-8").write(text)
    print(f"Wrote {OUT} ({len(data['tiles'])} tiles)")


if __name__ == "__main__":
    main()
