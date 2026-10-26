#!/usr/bin/env python3
"""core_caps.h from the status files (tools/coregen/core_caps.py).

  - Every Core's CORE_HAS_* from its status file (core.mk CORE_STATUS) equals
    the hand table port_caps.h had before the status files took it over.
  - port_caps.h keeps only facts that are not status features.
  - Every capability SDK code tests comes from a feature's "cap" or from
    port_caps.h; every core_ header gate expands the generated reason.
  - Every core_ header with functions a feature names ends with the
    CORE_CAPS_DONE_<STEM> define and the include that marks them.
  - The marks work (GCC): a call to a function of a feature that is na, none
    or wip is a compile-time error with the reason, also for a static inline
    function that -Og inlines; a function the Core has still compiles.

Run: python3 tools/test_core_caps.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "coregen"))
import core_caps  # noqa: E402

# port_caps.h's table before the status files took it over (tiles b693809),
# by the part each core.mk names in MCU_PART.
OLD_PORT_CAPS = {
    #               USB BLE THREAD AUDIO DAC I3C SPI RNG OTP
    "STM32L011xx":  (0,  0,  0,     0,    0,  0,  0,  0,  0),
    "STM32L422xx":  (1,  0,  0,     0,    0,  0,  1,  1,  0),
    "STM32WBA55xx": (0,  1,  1,     1,    0,  0,  1,  1,  1),
    "STM32H523xx":  (1,  0,  0,     0,    1,  1,  1,  1,  0),
}
OLD_NAMES = ("USB", "BLE", "THREAD", "AUDIO", "DAC", "I3C", "SPI", "RNG", "OTP")
PORT_FACTS = {"CORE_HAS_DWT", "CORE_HAS_SMALL_RAM"}

CORES_DIR = os.path.join(ROOT, "cores")
PORT_CAPS = os.path.join(ROOT, "sdk", "port", "stm32", "core", "port_caps.h")


def _read(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read()


def core_mks():
    """{stem: (MCU_PART, status file path)} for every public core.mk."""
    out = {}
    for stem in sorted(os.listdir(CORES_DIR)):
        mk = os.path.join(CORES_DIR, stem, "core.mk")
        if not os.path.exists(mk):
            continue
        text = _read(mk)
        part = re.search(r"^MCU_PART\s*=\s*(\S+)", text, re.M).group(1)
        st = re.search(r"^CORE_STATUS\s*=\s*\$\(SDK_DIR\)(\S+)", text, re.M)
        out[stem] = (part, os.path.join(ROOT, st.group(1)) if st else None)
    return out


class CapsFromStatus(unittest.TestCase):
    def setUp(self):
        self.features = core_caps.load_json(core_caps.FEATURES_JSON)

    def test_every_core_mk_names_a_status_file(self):
        for stem, (_, status) in core_mks().items():
            self.assertIsNotNone(status, f"cores/{stem}/core.mk sets no CORE_STATUS")
            self.assertTrue(os.path.exists(status), f"cores/{stem}/core.mk: {status} missing")

    def test_caps_equal_the_old_port_caps_table(self):
        mks = core_mks()
        self.assertEqual({p for p, _ in mks.values()}, set(OLD_PORT_CAPS))
        for stem, (part, status) in mks.items():
            got = core_caps.cap_values(core_caps.load_json(status), self.features)
            want = {f"CORE_HAS_{n}": v for n, v in zip(OLD_NAMES, OLD_PORT_CAPS[part])}
            self.assertEqual(got, want, f"{stem} ({os.path.basename(status)})")

    def test_port_caps_has_only_port_facts(self):
        defined = set(re.findall(r"#define\s+(CORE_HAS_[A-Z0-9_]+)", _read(PORT_CAPS)))
        self.assertEqual(defined, PORT_FACTS)
        caps = {f["cap"] for _, f in core_caps.feature_list(self.features) if f.get("cap")}
        self.assertFalse(caps & PORT_FACTS)

    def test_every_capability_read_is_defined(self):
        caps = {f["cap"] for _, f in core_caps.feature_list(self.features) if f.get("cap")}
        read = {}
        for dirpath, _, names in os.walk(os.path.join(ROOT, "sdk")):
            for n in names:
                if n.endswith((".h", ".c")) and n != "port_caps.h":
                    for name in re.findall(r"\bCORE_HAS_[A-Z0-9_]+\b", _read(os.path.join(dirpath, n))):
                        read.setdefault(name, set()).add(os.path.relpath(os.path.join(dirpath, n), ROOT))
        unknown = {n: sorted(f) for n, f in read.items() if n not in caps | PORT_FACTS}
        self.assertEqual(unknown, {}, "CORE_HAS_ read but neither a features.json cap nor a port fact")

    def test_header_gates_expand_the_generated_reason(self):
        caps = {f["cap"] for _, f in core_caps.feature_list(self.features) if f.get("cap")}
        hdr_dir = os.path.join(ROOT, "sdk", "core")
        for n in sorted(os.listdir(hdr_dir)):
            if not n.endswith(".h"):
                continue
            text = _read(os.path.join(hdr_dir, n))
            for cap, body in re.findall(r"#if !(CORE_HAS_[A-Z0-9_]+)\n(.*?)#endif", text, re.S):
                self.assertIn(cap, caps, f"{n}: gate on {cap}, which no feature backs")
                self.assertRegex(body, r"^CORE_CAPS_MISSING_" + cap[len("CORE_HAS_"):] + r"\b",
                                 f"{n}: the gate on {cap} should expand CORE_CAPS_MISSING_*")

    def test_marked_headers_end_with_the_marks_include(self):
        for hdr in core_caps.headers_needing_marks():
            text = _read(os.path.join(ROOT, "sdk", "core", hdr))
            stem = core_caps.header_stem_macro(hdr)
            tail = text[text.rindex("#define CORE_CAPS_DONE_"):] if "#define CORE_CAPS_DONE_" in text else ""
            self.assertRegex(tail, r"^#define CORE_CAPS_DONE_" + stem + r"\n#include \"core_caps.h\"\n",
                             f"{hdr} declares functions features.json names: end it with "
                             f"#define CORE_CAPS_DONE_{stem} + #include \"core_caps.h\"")
            self.assertTrue(text.rstrip().endswith(f"#endif /* {stem}_H */"),
                            f"{hdr}: the marks include belongs right before the closing #endif")

    def test_function_patterns_resolve(self):
        owner = core_caps.resolve_functions(self.features)   # raises on a dead pattern
        self.assertIn("core_dac_init", owner)
        self.assertEqual(owner["core_usb_hid_send"][0], "conn.usb_hid")   # the more specific pattern
        self.assertNotIn("core_thread_service_tick", owner)                # "!" leaves it out

    def test_every_status_file_renders(self):
        for _, (_, status) in core_mks().items():
            text = core_caps.render(status)
            self.assertIn("#define CORE_HAS_USB", text)

    def test_messages(self):
        text = core_caps.render(os.path.join(ROOT, "sdk", "status", "core-u.json"), core_name="Core.ST.L4.2")
        self.assertIn('"Core.ST.L4.2 has no DAC (adv.dac is na in core-u.json)"', text)
        self.assertIn("__typeof__(core_dac_init) core_dac_init __attribute__((unavailable(CORE_CAPS_WHY_ADV_DAC)))",
                      text)
        self.assertIn('"OTP is not implemented yet on Core.ST.L4.2 (nvm.otp is none in core-u.json)"', text)


GCC = shutil.which("arm-none-eabi-gcc")


@unittest.skipIf(GCC is None, "arm-none-eabi-gcc not on PATH")
class MarksCompile(unittest.TestCase):
    """The mechanism on a stand-in header: one static inline and one extern
    function, a feature that names them, and a Core with each status."""

    HEADER = """\
#ifndef CORE_WIDGET_H
#define CORE_WIDGET_H
#include "core_caps.h"
#if !CORE_HAS_WIDGET
CORE_CAPS_MISSING_WIDGET
#endif
static inline int core_widget_read(void) { return 1; }
#define CORE_CAPS_DONE_CORE_WIDGET
#include "core_caps.h"
#endif /* CORE_WIDGET_H */
"""
    GADGET = """\
#ifndef CORE_GADGET_H
#define CORE_GADGET_H
static inline int core_gadget_poke(void) { return 3; }
static inline int core_gadget_twice(void) { return 2 * core_gadget_poke(); }
extern void core_gadget_init(int speed);
#define CORE_CAPS_DONE_CORE_GADGET
#include "core_caps.h"
#endif /* CORE_GADGET_H */
"""
    MAIN = """\
#include "core_gadget.h"
#include "core_widget.h"
int main(void) { core_gadget_init(4); return core_widget_read() + core_gadget_twice() + core_gadget_poke(); }
"""

    def build(self, widget, gadget):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d)
        feats = {"groups": [{"id": "w", "name": "Widget", "features": [
            {"id": "w.core", "name": "core", "cap": "CORE_HAS_WIDGET", "functions": ["core_widget_*"]},
            {"id": "w.gadget", "name": "gadget", "functions": ["core_gadget_*"]}]}]}
        status = {"id": "Core.T.X", "features": {"w.core": widget, "w.gadget": gadget}}
        hdrs = os.path.join(d, "core")
        os.makedirs(hdrs)
        for name, text in (("core_widget.h", self.HEADER), ("core_gadget.h", self.GADGET),
                           ("port_caps.h", "#define CORE_HAS_DWT 1\n"), ("main.c", self.MAIN)):
            with open(os.path.join(hdrs, name), "w") as f:
                f.write(text)
        for name, obj in (("features.json", feats), ("core-x.json", status)):
            with open(os.path.join(d, name), "w") as f:
                json.dump(obj, f)
        gen = os.path.join(d, "gen")
        os.makedirs(gen)
        with open(os.path.join(gen, "core_caps.h"), "w") as f:
            f.write(core_caps.render(os.path.join(d, "core-x.json"), os.path.join(d, "features.json"), hdrs))
        r = subprocess.run([GCC, "-mcpu=cortex-m4", "-mthumb", "-std=c17", "-Wall", "-Wextra", "-Og",
                            "-I", gen, "-I", hdrs, "-c", os.path.join(hdrs, "main.c"),
                            "-o", os.path.join(d, "main.o")], capture_output=True, text=True)
        return r.returncode, r.stderr

    def test_available_compiles_clean(self):
        rc, err = self.build("verified", "compile")
        self.assertEqual((rc, err), (0, ""))

    def test_na_stops_at_the_header_gate(self):
        rc, err = self.build("na", "verified")
        self.assertNotEqual(rc, 0)
        self.assertIn("error: Core.T.X has no Widget (w.core is na in core-x.json)", err)

    def test_none_functions_are_compile_errors(self):
        # Static inline (inlined at -Og) and extern alike. core_gadget_twice()
        # calls core_gadget_poke() inside the header, before the marks: only
        # main.c's three calls are errors.
        rc, err = self.build("verified", "none")
        self.assertNotEqual(rc, 0)
        for fn in ("core_gadget_init", "core_gadget_poke", "core_gadget_twice"):
            self.assertIn(f"'{fn}' is unavailable: Widget gadget is not implemented yet on "
                          f"Core.T.X (w.gadget is none in core-x.json)", err)
        self.assertEqual(err.count("error:"), 3, err)

    def test_wip_cap_stops_at_the_gate(self):
        rc, err = self.build("wip", "verified")
        self.assertNotEqual(rc, 0)
        self.assertIn("error: Widget is not implemented yet on Core.T.X (w.core is wip in core-x.json)", err)


if __name__ == "__main__":
    unittest.main()
