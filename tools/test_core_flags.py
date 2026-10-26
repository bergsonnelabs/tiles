#!/usr/bin/env python3
"""coregen's project flags and the port's capability macros stay apart.

CORE_USES_<X> says what a project uses: coregen's templates define it, and SDK
code and the WAMR natives generator test it with #ifdef. CORE_HAS_<X> says
what the Core has: only a port's port_caps.h defines it, always as 1 or 0.

A CORE_USES_ name that SDK code reads but no template defines compiles the
guarded code out silently (core_timer_pwm_set_pad, core_stepper's pad setup
and the WAMR PWM natives were lost this way after the rename in #346, until
core_pads.h.j2 was fixed). Run: python3 tools/test_core_flags.py
"""
import os
import re
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATES = os.path.join(ROOT, "tools", "coregen", "templates")
READERS = [os.path.join(ROOT, "sdk"), os.path.join(ROOT, "tools", "gen_studio_natives.py")]

USES = re.compile(r"\bCORE_USES_[A-Z0-9_]+\b")
DEFINE = re.compile(r"#define\s+(CORE_(?:USES|HAS)_[A-Z0-9_]+)\b")


def _files(path, exts):
    if os.path.isfile(path):
        yield path
        return
    for dirpath, _, names in os.walk(path):
        for n in names:
            if n.endswith(exts):
                yield os.path.join(dirpath, n)


def _read(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read()


class CoreFlags(unittest.TestCase):
    def setUp(self):
        self.template_defines = {}
        for p in _files(TEMPLATES, (".j2",)):
            for name in DEFINE.findall(_read(p)):
                self.template_defines.setdefault(name, []).append(os.path.relpath(p, ROOT))

    def test_every_uses_flag_read_is_defined_by_coregen(self):
        read = {}
        for root in READERS:
            for p in _files(root, (".h", ".c", ".py")):
                for name in USES.findall(_read(p)):
                    read.setdefault(name, set()).add(os.path.relpath(p, ROOT))
        missing = {n: sorted(f) for n, f in read.items() if n not in self.template_defines}
        self.assertEqual(missing, {}, "CORE_USES_ flags read but never defined by a coregen template")

    def test_templates_define_no_capabilities(self):
        has = {n: f for n, f in self.template_defines.items() if n.startswith("CORE_HAS_")}
        self.assertEqual(has, {}, "CORE_HAS_ is a port capability (port_caps.h), not a coregen project flag")


if __name__ == "__main__":
    unittest.main()
