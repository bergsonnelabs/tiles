#!/usr/bin/env python3
"""
sync_definitions.py — One-way sync: canonical product DB → definitions/.

The product DB (the TileEditor app writes to it) is the single source of
truth for tile definitions. This script mirrors every tile's JSON into
`definitions/<stem>.json`, where the stem is derived from the DB columns:

    tile_families.name  +  tiles.name  +  tiles.rev
    "Core"              +  "ST.L4"     +  "b"        -> Core-ST-L4-b.json

It is a *faithful mirror*: it writes/updates every non-empty tile AND
deletes any `definitions/*.json` that no longer corresponds to a DB row.
(The previous archived sync only ever created/updated, never deleted —
which is why stems that were renamed in the DB lingered in git for so
long afterward.)

Dry-run by default; pass --apply to write. Intended to run both by hand
and from the scheduled `sync-definitions` GitHub Action (which opens a PR
whenever this produces a drift).

Connection comes from the environment (no credentials in the repo):
    TILES_DB_HOST, TILES_DB_PORT, TILES_DB_USER, TILES_DB_PASSWORD, TILES_DB_NAME
Host/port/user/name fall back to the known library instance; the password
has no default and must be supplied.

Usage:
    TILES_DB_PASSWORD=… python3 tools/sync_definitions.py            # dry-run
    TILES_DB_PASSWORD=… python3 tools/sync_definitions.py --apply
    TILES_DB_PASSWORD=… python3 tools/sync_definitions.py --only Core-ST-L4-b.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import mysql.connector


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

DEFINITIONS_DIR = Path(__file__).resolve().parent.parent / "definitions"

DB = dict(
    host=os.environ.get("TILES_DB_HOST", "library.c1cuew2e6bhd.eu-north-1.rds.amazonaws.com"),
    port=int(os.environ.get("TILES_DB_PORT", "3306")),
    user=os.environ.get("TILES_DB_USER", "web"),
    password=os.environ.get("TILES_DB_PASSWORD", ""),
    database=os.environ.get("TILES_DB_NAME", "library"),
)


def tile_filename(family: str, name: str, rev: str) -> str:
    """Core + ST.L4 + b -> Core-ST-L4-b.json (dots in name become dashes)."""
    return f"{family}-{name.replace('.', '-')}-{rev}.json"


def main() -> int:
    ap = argparse.ArgumentParser(description="Mirror the product DB into definitions/.")
    ap.add_argument("--apply", action="store_true", help="write changes (default: dry-run)")
    ap.add_argument("--only", help="restrict to a single filename, e.g. Core-ST-L4-b.json")
    args = ap.parse_args()

    if not DB["password"]:
        print("ERROR: set TILES_DB_PASSWORD (no credentials are stored in the repo).",
              file=sys.stderr)
        return 2
    if not DEFINITIONS_DIR.is_dir():
        print(f"ERROR: definitions dir not found: {DEFINITIONS_DIR}", file=sys.stderr)
        return 2

    conn = mysql.connector.connect(connection_timeout=20, **DB)
    cur = conn.cursor()
    cur.execute(
        """
        SELECT f.name AS family, t.name, t.rev, t.json
        FROM tiles t
        JOIN tile_families f ON t.tile_family_id = f.id
        ORDER BY f.name, t.name, t.rev
        """
    )
    rows = cur.fetchall()
    cur.close()
    conn.close()

    counts = {"create": 0, "update": 0, "identical": 0, "delete": 0, "skip_empty": 0}
    db_files: set[str] = set()

    for family, name, rev, raw in rows:
        fname = tile_filename(family, name, rev)
        if not raw or not str(raw).strip():
            counts["skip_empty"] += 1
            continue  # placeholder DB row with no JSON — not a real definition
        db_files.add(fname)
        if args.only and fname != args.only:
            continue

        try:
            db_obj = json.loads(raw)
        except (ValueError, TypeError) as e:
            print(f"  BADJSON {fname}  (unparseable DB json: {e})")
            continue
        text = json.dumps(db_obj, indent=2, ensure_ascii=False) + "\n"
        path = DEFINITIONS_DIR / fname

        if path.exists():
            if path.read_text(encoding="utf-8") == text:
                counts["identical"] += 1
                continue
            print(f"  {'UPDATE' if args.apply else 'DIFF  '}  {fname}")
            if args.apply:
                path.write_text(text, encoding="utf-8")
            counts["update"] += 1
        else:
            print(f"  {'CREATE' if args.apply else 'NEW   '}  {fname}")
            if args.apply:
                path.write_text(text, encoding="utf-8")
            counts["create"] += 1

    # Faithful mirror: delete any definition file with no corresponding DB row.
    if not args.only:
        for path in sorted(DEFINITIONS_DIR.glob("*.json")):
            if path.name not in db_files:
                print(f"  {'DELETE' if args.apply else 'STALE '}  {path.name}")
                if args.apply:
                    path.unlink()
                counts["delete"] += 1

    print(
        f"\n{'APPLIED' if args.apply else 'DRY-RUN'}: "
        f"create={counts['create']} update={counts['update']} "
        f"delete={counts['delete']} identical={counts['identical']} "
        f"skip_empty={counts['skip_empty']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
