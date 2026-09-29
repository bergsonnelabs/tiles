#!/usr/bin/env python3
"""
sync_definitions.py — One-way sync: canonical product DB → definitions/.

The product DB (the TileEditor app writes to it) is the single source of
truth for tile definitions. This script mirrors every public tile's JSON
(status production, beta or obsolete) into `definitions/<stem>.json`, where
the stem is derived from the DB columns:

    tile_families.name  +  tiles.name  +  tiles.rev
    "Core"              +  "ST.L4"     +  "b"        -> Core-ST-L4-b.json

It is a *faithful mirror*: it writes/updates every non-empty public tile
AND deletes any `definitions/*.json` that no longer corresponds to a public
DB row, so a tile whose status leaves production/beta/obsolete leaves the
public repo in the same sync. (The previous archived sync only ever created/updated, never deleted —
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
    TILES_DB_PASSWORD=… python3 tools/sync_definitions.py --statuses-out s.json

--statuses-out also writes {definition filename: status} (production, beta,
alpha, dark, ...) for tools/gen_kicad_lib.py. Status stays out of the
definition files themselves: they are public, and a tile's status is not.

Private repos. Tiles that are not public (by DB status) go only to private
repos, which share one overlay layout (definitions/, drivers/,
twins/src/sims/; only definitions/ is written here):

    production, beta, obsolete               -> public tiles only
    alpha                                    -> bergsonnelabs/tiles-alpha
    dark                                     -> bergsonnelabs/tile-<family>-<name>
                                                (Sense.X.1 -> tile-sense-x-1)
    design, concept, prototype, abandoned,
    no status, or any status not listed here -> bergsonnelabs/tiles-internal

    … --private-repos-out repos.txt      # the private repos the DB needs, one per line
    … --private-root DIR --apply         # mirror into DIR/<repo>/definitions/
    … --private-only                     # leave the public definitions/ alone

Each checkout under DIR that is a private repo (tiles-alpha, tiles-internal,
tile-*) is a faithful mirror of its own tiles: a definition that no longer
routes there (status changed, tile deleted) is removed, even when nothing
routes there any more. A dark tile whose repo has no checkout under DIR is
published nowhere and the run exits 3, after mirroring everything else, so
the caller can still push the other repos and then fail.
"""

from __future__ import annotations

import argparse
import json
import os
import re
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


# ---- Private routing -----------------------------------------------------
PUBLIC_STATUSES = {"production", "beta", "obsolete"}
ALPHA_REPO = "tiles-alpha"
INTERNAL_REPO = "tiles-internal"
# What counts as a private overlay checkout under --private-root. `tiles-`
# alone is not enough: tiles-bundles and tiles-sdk are not overlays.
PRIVATE_REPO_RE = re.compile(r"^(tiles-alpha|tiles-internal|tile-[a-z0-9._-]+)$")


def private_repo(family: str, name: str, status: str | None) -> str | None:
    """The private repo a tile belongs in, or None for a public tile.

    Anything that isn't explicitly public or alpha or dark lands in
    tiles-internal, so a status added to the DB later stays private until
    someone decides otherwise."""
    if status in PUBLIC_STATUSES:
        return None
    if status == "alpha":
        return ALPHA_REPO
    if status == "dark":
        return f"tile-{family}-{name.replace('.', '-')}".lower()
    return INTERNAL_REPO


def mirror_private(root: Path, routed: dict[str, dict[str, str | None]], apply: bool,
                   counts: dict[str, int]) -> list[str]:
    """Mirror each routed repo's definitions into root/<repo>/definitions/.

    A None text (unparseable DB json) leaves that file as it is, as the
    public mirror does. Returns the repos that have tiles routed to them but
    no checkout under root; their tiles are written nowhere."""
    checkouts = sorted(p.name for p in root.iterdir()
                       if p.is_dir() and PRIVATE_REPO_RE.match(p.name))
    # Against the checkouts actually mirrored, so a repo name the pattern
    # doesn't cover can't pass as present and publish nothing quietly.
    missing = sorted(r for r in routed if r not in checkouts)
    for repo in checkouts:
        want = routed.get(repo, {})
        ddir = root / repo / "definitions"
        if apply and want:
            ddir.mkdir(exist_ok=True)
        for fname, text in sorted(want.items()):
            path = ddir / fname
            if text is None:
                continue
            if path.exists() and path.read_text(encoding="utf-8") == text:
                counts["identical"] += 1
                continue
            new = not path.exists()
            verb = ("CREATE" if apply else "NEW   ") if new else ("UPDATE" if apply else "DIFF  ")
            print(f"  {verb}  {repo}/definitions/{fname}")
            if apply:
                path.write_text(text, encoding="utf-8")
            counts["create" if new else "update"] += 1
        for path in sorted(ddir.glob("*.json")) if ddir.is_dir() else []:
            if path.name not in want:
                print(f"  {'DELETE' if apply else 'STALE '}  {repo}/definitions/{path.name}")
                if apply:
                    path.unlink()
                counts["delete"] += 1
    for repo in missing:
        for fname in sorted(routed[repo]):
            print(f"  ERROR   {fname}: no checkout of {repo} under {root}; "
                  f"published nowhere", file=sys.stderr)
    return missing


def main() -> int:
    ap = argparse.ArgumentParser(description="Mirror the product DB into definitions/.")
    ap.add_argument("--apply", action="store_true", help="write changes (default: dry-run)")
    ap.add_argument("--only", help="restrict to a single filename, e.g. Core-ST-L4-b.json")
    ap.add_argument("--statuses-out", type=Path,
                    help="also write {definition filename: status} as JSON to this path")
    ap.add_argument("--private-root", type=Path,
                    help="also mirror non-public tiles into DIR/<repo>/definitions/")
    ap.add_argument("--private-repos-out", type=Path,
                    help="write the private repos the DB's tiles route to, one per line")
    ap.add_argument("--private-only", action="store_true",
                    help="skip the public definitions/ mirror")
    args = ap.parse_args()

    if args.only and args.private_root:
        # A faithful per-repo mirror needs every tile; with --only it would
        # delete the rest.
        print("ERROR: --only and --private-root don't combine.", file=sys.stderr)
        return 2
    if args.private_root and not args.private_root.is_dir():
        print(f"ERROR: private root not found: {args.private_root}", file=sys.stderr)
        return 2

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
        SELECT f.name AS family, t.name, t.rev, t.json, s.status
        FROM tiles t
        JOIN tile_families f ON t.tile_family_id = f.id
        LEFT JOIN tile_statuses s ON s.id = t.tile_status_id
        ORDER BY f.name, t.name, t.rev
        """
    )
    rows = cur.fetchall()
    cur.close()
    conn.close()

    counts = {"create": 0, "update": 0, "identical": 0, "delete": 0, "skip_empty": 0}
    db_files: set[str] = set()
    statuses: dict[str, str | None] = {}
    routed: dict[str, dict[str, str | None]] = {}  # private repo -> {filename: text}

    for family, name, rev, raw, status in rows:
        fname = tile_filename(family, name, rev)
        if not raw or not str(raw).strip():
            counts["skip_empty"] += 1
            continue  # placeholder DB row with no JSON — not a real definition
        statuses[fname] = status
        repo = private_repo(family, name, status)
        if repo:
            routed.setdefault(repo, {})[fname] = None
        else:
            db_files.add(fname)  # only public tiles belong in public definitions/
        if args.only and fname != args.only:
            continue

        try:
            db_obj = json.loads(raw)
        except (ValueError, TypeError) as e:
            print(f"  BADJSON {fname}  (unparseable DB json: {e})")
            continue
        text = json.dumps(db_obj, indent=2, ensure_ascii=False) + "\n"
        if repo:
            routed[repo][fname] = text
        if args.private_only or repo:
            continue  # a non-public tile never touches public definitions/
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

    # Faithful mirror: delete any definition file with no public DB row (a
    # deleted tile, or one whose status isn't public).
    if not args.only and not args.private_only:
        for path in sorted(DEFINITIONS_DIR.glob("*.json")):
            if path.name not in db_files:
                print(f"  {'DELETE' if args.apply else 'STALE '}  {path.name}")
                if args.apply:
                    path.unlink()
                counts["delete"] += 1

    if args.statuses_out:
        args.statuses_out.write_text(json.dumps(statuses, indent=2, sort_keys=True) + "\n",
                                     encoding="utf-8")
        print(f"  wrote {len(statuses)} statuses to {args.statuses_out}")

    if args.private_repos_out:
        args.private_repos_out.write_text("".join(f"{r}\n" for r in sorted(routed)),
                                          encoding="utf-8")
        print(f"  wrote {len(routed)} private repos to {args.private_repos_out}")

    missing: list[str] = []
    pcounts = {"create": 0, "update": 0, "identical": 0, "delete": 0}
    if args.private_root:
        missing = mirror_private(args.private_root, routed, args.apply, pcounts)

    print(
        f"\n{'APPLIED' if args.apply else 'DRY-RUN'}: "
        f"create={counts['create']} update={counts['update']} "
        f"delete={counts['delete']} identical={counts['identical']} "
        f"skip_empty={counts['skip_empty']}"
    )
    if args.private_root:
        print(f"{'APPLIED' if args.apply else 'DRY-RUN'} (private): "
              f"create={pcounts['create']} update={pcounts['update']} "
              f"delete={pcounts['delete']} identical={pcounts['identical']} "
              f"repos={len(routed)} missing={len(missing)}")
    if missing:
        print(f"ERROR: no checkout of {', '.join(missing)}; its tiles were published "
              f"nowhere (a dark tile needs its own private repo).", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
