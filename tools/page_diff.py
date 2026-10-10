#!/usr/bin/env python3
"""Count the flash pages that differ between two firmware images.

Studio's SWD flash skips pages whose contents are unchanged, and a Bluetooth
delta update would send only the pages that differ, so this is the number that
says how much a small program edit costs to deliver. The link order puts the
project's own code last in flash for exactly this reason (Makefile, OBJECTS;
the Cores' linker scripts).

Both images are raw .bin files starting at the same flash address. A page that
exists in only one of them (the image grew or shrank) counts as differing; the
shorter image is padded with 0xFF, the erased-flash value.

Usage: page_diff.py [--page-size 8192] [--base 0x08000000] [--frames] old.bin new.bin
Prints a one-line summary, then each differing page with its address and how
many bytes in it changed. Exit 0 = identical, 1 = pages differ, 2 = error.

--frames also prints the Bluetooth delta plan (docs/ble-update-protocol.md
§3.1): 2048-byte chunks equal to the old image's go as S runs that never
cross an 8 KB page boundary, the rest as D frames. An S covering a whole page
(or the image's partial last page) keeps that page: the Core neither stages
nor rewrites it.
"""

import argparse
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--page-size", type=lambda s: int(s, 0), default=8192,
                    help="flash page size in bytes (default 8192: W5, H5)")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x08000000,
                    help="flash address of the images' first byte (default 0x08000000)")
    ap.add_argument("--frames", action="store_true",
                    help="also print the BLE delta plan: S runs vs D frames")
    ap.add_argument("old")
    ap.add_argument("new")
    a = ap.parse_args()
    if a.page_size <= 0:
        print("page_diff: --page-size must be positive", file=sys.stderr)
        return 2
    try:
        with open(a.old, "rb") as f:
            old = f.read()
        with open(a.new, "rb") as f:
            new = f.read()
    except OSError as e:
        print(f"page_diff: {e}", file=sys.stderr)
        return 2

    ps = a.page_size
    old_len, new_len = len(old), len(new)
    n_pages = max((len(old) + ps - 1) // ps, (len(new) + ps - 1) // ps)
    old = old.ljust(n_pages * ps, b"\xff")
    new = new.ljust(n_pages * ps, b"\xff")

    diff = []
    for p in range(n_pages):
        o, n = old[p * ps:(p + 1) * ps], new[p * ps:(p + 1) * ps]
        if o != n:
            diff.append((p, sum(x != y for x, y in zip(o, n))))

    print(f"{len(diff)} of {n_pages} pages differ (page size {ps}; old {old_len} B, new {new_len} B)")
    for p, nbytes in diff:
        print(f"  page {p:4d}  0x{a.base + p * ps:08x}  {nbytes} B changed")
    if a.frames:
        s_frames, d_frames, s_bytes, kept = delta_plan(old[:old_len], new[:new_len], ps)
        print(f"delta plan: {s_frames} S + {d_frames} D frames, {new_len - s_bytes} B sent; "
              f"{kept} of {(new_len + ps - 1) // ps} pages kept, the rest staged and rewritten "
              f"(a full transfer is {(new_len + CHUNK - 1) // CHUNK} D frames)")
    return 1 if diff else 0


CHUNK = 2048          # D payload / S granularity
RUN_MAX = 8192        # longest S run


def delta_plan(old, new, page):
    """The host's plan, as in docs §3.1: a chunk equal to the old image's bytes
    (and inside the old image) joins an S run; a run is sent at a page
    boundary, at 8 KB, at the next differing chunk, or at the end. A run that
    is exactly one page (or the partial last page) keeps it."""
    s_frames = d_frames = s_bytes = kept = 0
    run_off = run = 0

    def flush():
        nonlocal s_frames, kept, run
        if run:
            s_frames += 1
            if run_off % page == 0 and (run == page or run_off + run == len(new)):
                kept += 1
            run = 0

    for off in range(0, len(new), CHUNK):
        n = min(CHUNK, len(new) - off)
        if off + n <= len(old) and new[off:off + n] == old[off:off + n]:
            if not run:
                run_off = off
            run += n
            s_bytes += n
            if run >= RUN_MAX or (off + n) % page == 0:
                flush()
            continue
        flush()
        d_frames += 1
    flush()
    return s_frames, d_frames, s_bytes, kept


if __name__ == "__main__":
    sys.exit(main())
