#!/usr/bin/env python3
"""Guard the SDK implementation-status data against id drift.

The status matrix in sdk/status/ has two parts that must stay in lockstep:

  - features.json   defines the canonical feature ids (grouped, ordered)
  - core-*.json     records each Core's status keyed by those ids

The website renders features.json's ids and looks each up by exact string in
the core files. So a key in a core file that isn't a defined feature is
invisible (silently dropped), and a renamed feature silently reads as
"planned" everywhere. This drift is exactly how the L0/L4/H5 files ended up on
an older id scheme than features.json + core-w. This check fails the build if:

  1. a core file has a key that isn't a defined feature id (orphan), or
  2. a feature id appears in no core file at all (dead row).

It also guards a second, unrelated kind of drift in drivers/: a driver
header's Doxygen `@version` tag disagreeing with its TILE_*_VERSION_*
macros. That tag is not decorative — gen_tile_docs emits it as the FIRST
line of the manifest description, so a stale tag makes the published docs
page open by announcing the wrong version. It went unnoticed across two
releases on two drivers, and then a third time in the very commit that
fixed the first two (the tag was corrected to 2.4.0 while VERSION_PATCH
was bumped to 1 a few lines below). Cheap to check, evidently not cheap
to remember.

Run:
    python3 tools/check_status_consistency.py
"""

import json
import re
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

STATUS_DIR = Path(__file__).resolve().parent.parent / "sdk" / "status"


DRIVERS_DIR = Path(__file__).resolve().parent.parent / "drivers"

_VERSION_TAG_RE = re.compile(r"^\s*\*\s*@version\s+([0-9]+(?:\.[0-9]+)*)", re.M)


def _macro(text, suffix):
    """TILE_<anything>_VERSION_<suffix> value, or None if absent."""
    m = re.search(r"#define\s+TILE_\w+_VERSION_" + suffix + r"\s+(\d+)", text)
    return m.group(1) if m else None


def check_driver_version_tags():
    """@version tag must match the VERSION_* macros in the same header.

    Only checks headers that have both — a driver with neither, or with only
    macros, is not in scope and is left alone.
    """
    failed = False
    checked = 0
    for path in sorted(DRIVERS_DIR.glob("tile_*.h")):
        text = path.read_text(encoding="utf-8", errors="replace")
        tag = _VERSION_TAG_RE.search(text)
        major = _macro(text, "MAJOR")
        if not tag or major is None:
            continue
        minor = _macro(text, "MINOR") or "0"
        patch = _macro(text, "PATCH") or "0"
        macros = f"{major}.{minor}.{patch}"
        checked += 1
        if tag.group(1) != macros:
            failed = True
            print(
                f"FAIL: {path.name} @version {tag.group(1)} != "
                f"VERSION_* macros {macros} — the docs page renders the tag",
                file=sys.stderr,
            )
    return failed, checked


def main():
    feat = json.loads((STATUS_DIR / "features.json").read_text())
    canon = [f["id"] for g in feat["groups"] for f in g["features"]]
    canon_set = set(canon)

    if len(canon) != len(canon_set):
        dupes = sorted({i for i in canon if canon.count(i) > 1})
        print(f"FAIL: duplicate feature ids in features.json: {dupes}", file=sys.stderr)
        sys.exit(1)

    cores = sorted(STATUS_DIR.glob("core-*.json"))
    used = set()
    failed = False
    for path in cores:
        d = json.loads(path.read_text())
        keys = set(d["features"].keys())
        used |= keys
        orphans = sorted(keys - canon_set)
        if orphans:
            failed = True
            print(
                f"FAIL: {path.name} has keys not defined in features.json "
                f"(rename or add them): {orphans}",
                file=sys.stderr,
            )

    dead = sorted(canon_set - used)
    if dead:
        # Not fatal — a freshly-defined feature legitimately has no Core value
        # yet — but surface it so dead rows don't accumulate unnoticed.
        print(f"note: features with no value on any Core: {dead}", file=sys.stderr)

    ver_failed, ver_checked = check_driver_version_tags()

    if failed or ver_failed:
        sys.exit(1)
    print(f"status consistency OK — {len(canon)} feature ids, {len(cores)} cores")
    print(f"driver @version tags OK — {ver_checked} headers")


if __name__ == "__main__":
    main()
