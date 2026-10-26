#!/usr/bin/env python3
"""
core_caps — the Core's capabilities, from its status file (core_caps.h)

One source of truth: the feature x Core table in sdk/status/ is what the site
shows, what Studio offers and what the compiler allows. A feature in
sdk/status/features.json may carry

  "cap":       "CORE_HAS_DAC"                 the capability macro it backs
  "functions": ["core_dac_*", "core_x_init"]  the core_ functions that need it
                                              (fnmatch patterns over the
                                              functions sdk/core/core_*.h
                                              declare; "!name" leaves one
                                              out)

and core_caps.h, written by coregen into the project's coregen/ for every
build, says for the Core being built (its status file, named by CORE_STATUS
in its core.mk):

  - CORE_HAS_<X> 1 or 0 for every "cap": 1 when the feature's status is
    verified, compile or partial; 0 for wip, none and na.
  - CORE_CAPS_MISSING_<X> for every cap that is 0: a `#pragma GCC error`
    with the reason ("Core.ST.L4 has no DAC ..."), which the feature's core_
    header expands under `#if !CORE_HAS_<X>` (the header stops right there,
    as its hand-written #error did).
  - For every function of a feature that is na, none or wip on this Core, an
    `unavailable` mark: calling it is a compile-time error with the same
    reason, never a link error. GCC's `unavailable` attribute, not `error`:
    most core_ functions are static inline, and a call that -Og inlines never
    reaches the `error` attribute's check. The marks are redeclarations
    (`__typeof__(f) f __attribute__((unavailable(...)))`), so they need the
    real declarations first: a core_ header with such functions ends with
    `#define CORE_CAPS_DONE_<STEM>` and `#include "core_caps.h"` again, and
    the marks part of this file (outside its include guard) marks that
    header's functions once.

Facts that are not status features (CORE_HAS_DWT, CORE_HAS_SMALL_RAM) stay
the port's: core_caps.h includes port_caps.h first.

Usage (host tests; coregen calls generate() itself):
    python3 core_caps.py <core status .json> [-o core_caps.h]
"""

import argparse
import fnmatch
import json
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
FEATURES_JSON = os.path.join(ROOT, "sdk", "status", "features.json")
CORE_HEADERS_DIR = os.path.join(ROOT, "sdk", "core")

# A feature with one of these statuses is there: CORE_HAS_<X> 1, no marks.
AVAILABLE = {"verified", "compile", "partial"}
# na: the Core has no such thing. none / wip: not implemented (yet).
UNAVAILABLE = {"na", "none", "wip"}

CAP_RE = re.compile(r"^CORE_HAS_[A-Z0-9_]+$")


class CapsError(Exception):
    """A status or features.json problem coregen reports as one ERROR line."""


def status_of(value):
    """A core-*.json feature value: "verified" or {"status": ..., "note": ...}."""
    return value.get("status") if isinstance(value, dict) else value


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def feature_list(features):
    """The features.json entries in order, each with its group alongside."""
    return [(g, f) for g in features["groups"] for f in g["features"]]


# ---- The functions sdk/core declares ----

_COMMENT_RE = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)
# A declaration at column 0: optional storage class / qualifiers, a type, the
# name. Statements in bodies are indented, so `return core_x(...)` never
# matches, and #define lines are dropped before matching.
_DECL_RE = re.compile(
    r"^(?:(?:CORE_INLINE|static|inline|extern|const|unsigned|signed|struct|enum)\s+)*"
    r"[A-Za-z_]\w*[\s\*]+\**(core_[a-z0-9_]+)\s*\(", re.M)


def header_stem_macro(header):
    """core_dac.h -> CORE_DAC: the suffix of CORE_CAPS_DONE_ / _MARKED_."""
    return re.sub(r"[^A-Za-z0-9]", "_", os.path.splitext(os.path.basename(header))[0]).upper()


def declared_functions(headers_dir=CORE_HEADERS_DIR):
    """{function: [header file name, ...]} for every core_ function the
    neutral headers declare (prototypes and inline definitions; not macros)."""
    out = {}
    for name in sorted(os.listdir(headers_dir)):
        if not (name.startswith("core_") and name.endswith(".h")):
            continue
        with open(os.path.join(headers_dir, name), encoding="utf-8", errors="replace") as f:
            text = _COMMENT_RE.sub("", f.read())
        # Drop preprocessor lines (with their continuations): macros are not
        # functions, and a mark can't redeclare one.
        text = re.sub(r"^[ \t]*#(?:[^\n]*\\\n)*[^\n]*", "", text, flags=re.M)
        for fn in _DECL_RE.findall(text):
            out.setdefault(fn, [])
            if name not in out[fn]:
                out[fn].append(name)
    return out


def resolve_functions(features, declared=None):
    """{function: (feature id, pattern)}: each declared function a feature's
    "functions" pattern names. A function two features name goes to the more
    specific pattern (an exact name, then the longer prefix); "!name" leaves
    a function out. Raises CapsError for a pattern that matches nothing (a
    typo, or a renamed function)."""
    declared = declared_functions() if declared is None else declared
    owner = {}

    def rank(pat):
        return (0 if any(c in pat for c in "*?[") else 1, len(pat.split("*")[0]))

    for _, feat in feature_list(features):
        pats = feat.get("functions", [])
        drop = {p[1:] for p in pats if p.startswith("!")}
        for pat in drop:
            if pat not in declared:
                raise CapsError(f"features.json {feat['id']}: \"!{pat}\" names no function "
                                f"declared in sdk/core/core_*.h")
        for pat in (p for p in pats if not p.startswith("!")):
            hits = [fn for fn in declared if fnmatch.fnmatchcase(fn, pat)]
            if not hits:
                raise CapsError(f"features.json {feat['id']}: \"{pat}\" matches no function "
                                f"declared in sdk/core/core_*.h")
            for fn in hits:
                if fn in drop:
                    continue
                prev = owner.get(fn)
                if prev is None or rank(pat) > rank(prev[1]):
                    owner[fn] = (feat["id"], pat)
    return owner


# ---- The caps of one Core ----

def caps_for(status, features):
    """[(cap, feature id, status, value 1/0), ...] in features.json order."""
    out, seen = [], {}
    for _, feat in feature_list(features):
        cap = feat.get("cap")
        if not cap:
            continue
        if not CAP_RE.match(cap):
            raise CapsError(f"features.json {feat['id']}: cap \"{cap}\" is not CORE_HAS_<NAME>")
        if cap in seen:
            raise CapsError(f"features.json: {cap} is the cap of both {seen[cap]} and {feat['id']}")
        seen[cap] = feat["id"]
        st = status_of(status["features"].get(feat["id"]))
        if st not in AVAILABLE | UNAVAILABLE:
            raise CapsError(f"{status.get('id', '?')}: {feat['id']} has status {st!r}, "
                            f"so {cap} can't be decided")
        out.append((cap, feat["id"], st, 1 if st in AVAILABLE else 0))
    return out


def cap_values(status, features):
    """{cap: 1/0} for one Core: what the generated core_caps.h defines."""
    return {cap: v for cap, _, _, v in caps_for(status, features)}


def _label(group, feat):
    """What a message calls the feature: the cap's name for a cap ("DAC",
    "Thread"); else the feature's name when it starts with an acronym ("USB
    HID"), or the group's and the feature's ("I2C target / device mode")."""
    if feat.get("cap"):
        name = feat["cap"][len("CORE_HAS_"):]
        return name if len(name) <= 3 else name.capitalize()
    first = feat["name"].split()[0] if feat["name"].split() else ""
    if len(first) > 1 and first.isupper():
        return feat["name"]
    return f"{group['name']} {feat['name']}"


def reason(core_id, status_file, group, feat, st):
    """The one-line message a gate or a mark gives."""
    what = _label(group, feat)
    where = f"{feat['id']} is {st} in {os.path.basename(status_file)}"
    if st == "na":
        text = f"{core_id} has no {what} ({where})"
    else:
        text = f"{what} is not implemented yet on {core_id} ({where})"
    # It lands inside a C string literal (and a _Pragma): no quotes, no escapes.
    return text.replace("\\", "/").replace('"', "'")


def render(status_file, features_file=FEATURES_JSON, headers_dir=CORE_HEADERS_DIR, core_name=None):
    """The text of core_caps.h for the Core whose status file is given.
    core_name: what the messages call the Core (coregen passes the public
    name of the one being built; default the status file's "id")."""
    status = load_json(status_file)
    features = load_json(features_file)
    core_id = core_name or status.get("id") or os.path.basename(status_file)
    caps = caps_for(status, features)
    declared = declared_functions(headers_dir)
    owner = resolve_functions(features, declared)
    by_id = {f["id"]: (g, f) for g, f in feature_list(features)}

    L = []
    w = L.append
    w("/**")
    w(f" * core_caps.h — What {core_id} has: CORE_HAS_* from its status file")
    w(" *")
    w(f" * AUTO-GENERATED by coregen (tools/coregen/core_caps.py) from")
    w(f" * {os.path.basename(status_file)} ({status.get('id', '?')}) and features.json. DO NOT EDIT:")
    w(" * change the status file, or a feature's \"cap\" / \"functions\".")
    w(" *")
    w(" * CORE_HAS_<X> is 1 when the feature is verified, compile or partial, else")
    w(" * 0; test with `#if`. The port's facts that are not status features")
    w(" * (CORE_HAS_DWT, CORE_HAS_SMALL_RAM) come from its port_caps.h. A core_")
    w(" * header for a feature this Core lacks stops at its CORE_CAPS_MISSING_<X>;")
    w(" * a call to a function this Core lacks is a compile-time error (the marks")
    w(" * at the end of this file).")
    w(" */")
    w("")
    w("#ifndef CORE_CAPS_H")
    w("#define CORE_CAPS_H")
    w("")
    w('#include "port_caps.h"')
    w("")
    names = [cap for cap, _, _, _ in caps]
    if names:
        w("/* The port's port_caps.h has only facts that are not status features. */")
        w("#if " + " || \\\n    ".join(f"defined({c})" for c in names))
        w('#error "port_caps.h defines a CORE_HAS_ that the Core\'s status file decides (features.json \\"cap\\")"')
        w("#endif")
        w("")
    width = max([len(c) for c in names] + [0])
    for cap, fid, st, v in caps:
        w(f"#define {cap:<{width}} {v}   /* {fid}: {st} */")
    w("")
    w("/* Expanded by a core_ header under `#if !CORE_HAS_<X>`: the reason, as a")
    w(" * compile-time error. */")
    w("#define CORE_CAPS_PRAGMA_(x) _Pragma(#x)")
    for cap, fid, st, v in caps:
        if not v:
            g, f = by_id[fid]
            w(f"#define CORE_CAPS_MISSING_{cap[len('CORE_HAS_'):]} \\")
            w(f"    CORE_CAPS_PRAGMA_(GCC error \"{reason(core_id, status_file, g, f, st)}\")")
    why = {}
    for fn, (fid, _) in sorted(owner.items()):
        st = status_of(status["features"].get(fid))
        if st in UNAVAILABLE and fid not in why:
            g, f = by_id[fid]
            why[fid] = "CORE_CAPS_WHY_" + re.sub(r"[^A-Za-z0-9]", "_", fid).upper()
    if why:
        w("")
        w("/* Why a marked function (below) is unavailable. */")
        for fid in sorted(why):
            g, f = by_id[fid]
            st = status_of(status["features"].get(fid))
            w(f'#define {why[fid]} "{reason(core_id, status_file, g, f, st)}"')
    w("")
    w("#endif /* CORE_CAPS_H */")
    w("")

    # Marks: per header, the functions of features this Core lacks.
    per_header = {}
    for fn in sorted(owner):
        fid, _ = owner[fn]
        st = status_of(status["features"].get(fid))
        if st not in UNAVAILABLE:
            continue
        for hdr in declared[fn]:
            per_header.setdefault(hdr, []).append((fn, why[fid]))
    w("/* ---- Functions this Core lacks ----")
    w(" * Outside the include guard: each core_ header below includes this file again")
    w(" * at its end, after defining CORE_CAPS_DONE_<STEM>, and its functions are")
    w(" * marked then, once. */")
    if not per_header:
        w("/* (none) */")
    for hdr in sorted(per_header):
        stem = header_stem_macro(hdr)
        w("")
        w(f"#if defined(CORE_CAPS_DONE_{stem}) && !defined(CORE_CAPS_MARKED_{stem})")
        w(f"#define CORE_CAPS_MARKED_{stem}   /* {hdr} */")
        for fn, why_macro in per_header[hdr]:
            w(f"__typeof__({fn}) {fn} __attribute__((unavailable({why_macro})));")
        w("#endif")
    return "\n".join(L) + "\n"


def headers_needing_marks(features_file=FEATURES_JSON, headers_dir=CORE_HEADERS_DIR):
    """The core_ headers that declare a function some feature names: each must
    end with `#define CORE_CAPS_DONE_<STEM>` + `#include "core_caps.h"`."""
    declared = declared_functions(headers_dir)
    owner = resolve_functions(load_json(features_file), declared)
    return sorted({h for fn in owner for h in declared[fn]})


def generate(status_file, out_path, features_file=FEATURES_JSON, core_name=None):
    """Write core_caps.h; returns the path."""
    text = render(status_file, features_file, core_name=core_name)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="Write core_caps.h from a Core's status file.")
    ap.add_argument("status", help="the Core's sdk/status/core-*.json")
    ap.add_argument("-o", "--output", help="output file (default: stdout)")
    ap.add_argument("--features", default=FEATURES_JSON, help="features.json")
    args = ap.parse_args()
    try:
        text = render(args.status, args.features)
    except (CapsError, OSError, ValueError) as e:
        print(f"ERROR: core_caps: {e}", file=sys.stderr)
        sys.exit(1)
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
