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

  1. a core file has a key that isn't a defined feature id (orphan),
  2. a core file lacks a defined feature id (the site renders a missing id
     as "Not implemented", so every Core states every feature), or
  3. a status is blank ("") or not one of the known values (a blank also
     rendered as "Not implemented" and hid real code; 2026-10), or
  4. a feature's "cap" / "functions" (coregen's core_caps.h) is malformed, a
     cap is claimed twice, or a function pattern names nothing sdk/core
     declares.

--overlay <repo> also checks a private repo's sdk/status/*.json (a Core that
is not public, e.g. tiles-alpha's core-n-lm20b.json) the same way.

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
    python3 tools/check_status_consistency.py [--overlay ../tiles-alpha]
"""

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "coregen"))
import core_caps  # noqa: E402  (features.json "cap" / "functions")


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

# The values the site knows (CLAUDE.md §5). A dict entry carries one in "status".
VALID_STATUS = {"verified", "compile", "partial", "wip", "none", "na"}


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


def check_core_files(paths, canon):
    """Orphan / missing ids and unknown statuses in core status files.
    Returns (failed, the ids the files use)."""
    canon_set = set(canon)
    used = set()
    failed = False
    for path in paths:
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
        missing = [i for i in canon if i not in keys]
        if missing:
            failed = True
            print(
                f"FAIL: {path.name} has no value for {missing} "
                f"(add each, \"none\" or \"na\" if nothing exists)",
                file=sys.stderr,
            )
        for fid, val in d["features"].items():
            status = val.get("status") if isinstance(val, dict) else val
            if status not in VALID_STATUS:
                failed = True
                print(
                    f"FAIL: {path.name} {fid}: status {status!r} is not one of "
                    f"{sorted(VALID_STATUS)}",
                    file=sys.stderr,
                )
    return failed, used


def check_caps(feat, paths):
    """features.json's "cap" / "functions" (coregen's core_caps.h): each cap
    well formed and backed by one feature, each function pattern naming a
    function sdk/core declares, and every Core's caps decidable."""
    try:
        core_caps.resolve_functions(feat)
        for path in paths:
            core_caps.caps_for(json.loads(path.read_text()), feat)
    except core_caps.CapsError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--overlay", metavar="REPO", action="append", default=[],
                    help="also check a private repo's <REPO>/sdk/status/*.json (a "
                         "non-public Core's status file) against this features.json; "
                         "repeatable")
    args = ap.parse_args()

    feat = json.loads((STATUS_DIR / "features.json").read_text())
    canon = [f["id"] for g in feat["groups"] for f in g["features"]]
    canon_set = set(canon)

    if len(canon) != len(canon_set):
        dupes = sorted({i for i in canon if canon.count(i) > 1})
        print(f"FAIL: duplicate feature ids in features.json: {dupes}", file=sys.stderr)
        sys.exit(1)

    cores = sorted(STATUS_DIR.glob("core-*.json"))
    failed, used = check_core_files(cores, canon)
    failed |= check_caps(feat, cores)

    dead = sorted(canon_set - used)
    if dead:
        # Unreachable while every Core must list every id; kept as a guard.
        print(f"note: features with no value on any Core: {dead}", file=sys.stderr)

    overlay_cores = []
    for repo in args.overlay:
        d = Path(repo) / "sdk" / "status"
        paths = sorted(p for p in d.glob("*.json") if p.name != "features.json")
        if not paths:
            print(f"note: {repo}: no status files in sdk/status/", file=sys.stderr)
        o_failed, _ = check_core_files(paths, canon)
        failed |= o_failed | check_caps(feat, paths)
        overlay_cores += paths

    ver_failed, ver_checked = check_driver_version_tags()

    if failed or ver_failed:
        sys.exit(1)
    print(f"status consistency OK — {len(canon)} feature ids, {len(cores)} cores"
          + (f" + {len(overlay_cores)} overlay ({', '.join(p.name for p in overlay_cores)})"
             if args.overlay else ""))
    print(f"driver @version tags OK — {ver_checked} headers")


if __name__ == "__main__":
    main()
