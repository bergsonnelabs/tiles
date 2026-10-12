#!/usr/bin/env python3
"""Rebuild the trimmed OpenThread FTD library that BLE + Thread builds link.

    python3 tools/build_openthread_trim.py [--cube DIR] [--out LIB] [-j N]
    python3 tools/build_openthread_trim.py --check     # rebuild in a temp dir, compare

Why: BLE + Thread in one Core.ST.W5 image does not leave enough of the WBA55's
128 KB of SRAM for an application with ST's prebuilt stm32wba_ot_ftd_lib.a
(128 message buffers, a DTLS-sized mbedTLS heap, commissioner / joiner / SRP /
DNS / TCP compiled in). This builds the same OpenThread source ST ships in
STM32CubeWBA V1.6.1 with the SDK's configuration
(sdk/thread/config/stm32wba-openthread-ftd-trim-config.h: ST's FTD config
minus the features the SDK does not use, fewer buffers, a smaller heap).

How: the source list, include paths and defines come from ST's own Keil
project for the FTD library (openthread_lib/MDK-ARM/thread_ftd_keil_config/
thread_ftd_keil_config.uvprojx), so the build follows ST's, with these
differences:
  - arm-none-eabi-gcc instead of armclang, with the SDK's W5 CPU flags and
    ST's -fshort-enums / -mno-unaligned-access, so its objects keep the ABI of
    ST's prebuilt mbedTLS library (which this library still uses) and of ST's
    OpenThread library;
  - OPENTHREAD_CONFIG_FILE / OPENTHREAD_PROJECT_CORE_CONFIG_FILE = the trim
    config;
  - not built: the CLI and NCP (src/cli, src/ncp, examples/apps/cli,
    debug_uart.c: the SDK uses neither) and TCPlp (TCP is off);
  - armclang-only pieces left out: stack/wbx_include (newlib has errno.h,
    strings.h, sys/types.h), WBx_OTLIB_BUILD_IAR_KEIL, _AEABI_PORTABILITY_LEVEL;
    the CMSIS device header comes from the Cube's Drivers/CMSIS (Keil takes it
    from its device pack).
The archive is deterministic (sorted members, `ar D`, no build paths in the
objects): the same toolchain and the same Cube package give the same bytes.
--check rebuilds and compares with the committed library; it needs the Cube
package and the toolchain recorded in the .txt next to the library.

Output: sdk/thread/lib/stm32wba_ot_ftd_trim_lib.a and
        sdk/thread/lib/stm32wba_ot_ftd_trim_lib.txt (how it was built, sha256).
"""

import argparse
import concurrent.futures
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CUBE = os.path.expanduser("~/STM32Cube/Repository/STM32Cube_FW_WBA_V1.6.1")
LIB = os.path.join(ROOT, "sdk", "thread", "lib", "stm32wba_ot_ftd_trim_lib.a")
CONFIG = "stm32wba-openthread-ftd-trim-config.h"
PROJECT = ("Middlewares/ST/STM32_WPAN/thread/openthread/openthread_lib/MDK-ARM/"
           "thread_ftd_keil_config/thread_ftd_keil_config.uvprojx")

# Not built (see the docstring).
SKIP = (
    "stack/src/cli/",
    "stack/src/ncp/",
    "stack/examples/apps/cli/",
    "stack/examples/platforms/utils/debug_uart.c",
    "stack/third_party/tcplp/",
    "Third_Party/mbedtls/",          # ST's separate mbedTLS library, prebuilt
)
DROP_DEFINES = ("_AEABI_PORTABILITY_LEVEL", "WBx_OTLIB_BUILD_IAR_KEIL",
                "OPENTHREAD_CONFIG_FILE", "OPENTHREAD_PROJECT_CORE_CONFIG_FILE")
DROP_INCLUDES = ("stack/wbx_include",)

CPU = ["-mcpu=cortex-m33", "-mthumb", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard"]
COMMON = CPU + ["-Os", "-g0", "-ffunction-sections", "-fdata-sections", "-fno-common",
                "-fshort-enums", "-mno-unaligned-access", "-w"]
CFLAGS = ["-std=gnu11"]
CXXFLAGS = ["-std=gnu++11", "-fno-exceptions", "-fno-rtti", "-fno-threadsafe-statics",
            "-fno-use-cxa-atexit"]


def project(cube):
    """Source files, include dirs and defines of ST's FTD library project."""
    path = os.path.join(cube, PROJECT)
    pdir = os.path.dirname(path)
    root = ET.parse(path).getroot()
    target = next(root.iter("Target"))
    vc = target.find(".//Cads/VariousControls")

    def norm(p):
        return os.path.normpath(os.path.join(pdir, p.replace("\\", "/")))

    incs = [norm(p) for p in (vc.findtext("IncludePath") or "").split(";") if p]
    incs = [p for p in incs if not any(p.endswith(d) for d in DROP_INCLUDES)]
    # Keil finds the CMSIS device header (stm32wbaxx.h, which ST's mbedTLS
    # config includes) through its device pack; here it is the Cube's copy.
    incs.append(os.path.join(cube, "Drivers/CMSIS/Device/ST/STM32WBAxx/Include"))
    defs = [d.strip() for d in (vc.findtext("Define") or "").split(",") if d.strip()]
    defs = [d for d in defs if d.split("=")[0] not in DROP_DEFINES]
    files = []
    for f in target.iter("File"):
        p = (f.findtext("FilePath") or "").replace("\\", "/")
        if not p.endswith((".c", ".cpp")) or any(s in p for s in SKIP):
            continue
        files.append(norm(p))
    return files, incs, defs


def tool(name):
    return "arm-none-eabi-" + name


def toolchain_version():
    out = subprocess.run([tool("gcc"), "--version"], capture_output=True, text=True, check=True).stdout
    return out.splitlines()[0].strip()


def build(cube, out_lib, jobs):
    files, incs, defs = project(cube)
    if not files:
        sys.exit("no sources found in " + os.path.join(cube, PROJECT))
    missing = [f for f in files if not os.path.isfile(f)]
    if missing:
        sys.exit("missing sources (wrong --cube?):\n  " + "\n  ".join(missing[:10]))

    # The SDK's config dir first: the trim config includes ST's FTD config,
    # which the SDK carries unchanged (sdk/thread/config).
    inc_flags = ["-I" + os.path.join(ROOT, "sdk", "thread", "config")] + ["-I" + i for i in incs]
    def_flags = ["-D" + d for d in defs] + [
        "-DOPENTHREAD_CONFIG_FILE=<%s>" % CONFIG,
        "-DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=<%s>" % CONFIG,
        # otGetVersionString() would carry __DATE__ __TIME__: a fixed text
        # keeps the build reproducible.
        '-DOPENTHREAD_BUILD_DATETIME="tiles-trim"',
    ]
    # No machine paths in the objects (__FILE__ in asserts, debug info is off).
    stack = os.path.join(cube, "Middlewares/ST/STM32_WPAN/thread/openthread")
    prefix = ["-ffile-prefix-map=%s=openthread" % stack, "-ffile-prefix-map=%s=cube" % cube,
              "-ffile-prefix-map=%s=tiles" % ROOT]

    work = tempfile.mkdtemp(prefix="ot-trim-")
    objs = {}
    names = set()
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0] + ".o"
        if name in names:
            sys.exit("two sources map to " + name)
        names.add(name)
        objs[f] = os.path.join(work, name)

    def compile_one(f):
        cxx = f.endswith(".cpp")
        cmd = [tool("g++" if cxx else "gcc")] + COMMON + (CXXFLAGS if cxx else CFLAGS) + \
              prefix + def_flags + inc_flags + ["-c", f, "-o", objs[f]]
        r = subprocess.run(cmd, capture_output=True, text=True)
        return f, r.returncode, r.stderr

    failed = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        for f, rc, err in ex.map(compile_one, files):
            if rc != 0:
                failed.append((f, err))
    if failed:
        for f, err in failed[:5]:
            print("FAILED", f, "\n", err[-3000:], file=sys.stderr)
        sys.exit("%d of %d sources failed" % (len(failed), len(files)))

    members = sorted(objs.values(), key=os.path.basename)
    tmp_lib = os.path.join(work, "lib.a")
    subprocess.run([tool("ar"), "rcsD", tmp_lib] + members, check=True)
    os.makedirs(os.path.dirname(out_lib), exist_ok=True)
    shutil.copyfile(tmp_lib, out_lib)
    shutil.rmtree(work)

    sha = hashlib.sha256(open(out_lib, "rb").read()).hexdigest()
    rel = lambda p: os.path.relpath(p, cube)
    with open(os.path.splitext(out_lib)[0] + ".txt", "w") as fp:
        fp.write("stm32wba_ot_ftd_trim_lib.a: OpenThread FTD (trimmed), built by tools/build_openthread_trim.py\n")
        fp.write("source:    STM32CubeWBA %s, Middlewares/ST/STM32_WPAN/thread/openthread/stack (OpenThread 4a0d6b7)\n"
                 % os.path.basename(cube.rstrip("/")).replace("STM32Cube_FW_WBA_", ""))
        fp.write("project:   %s (sources, includes, defines)\n" % PROJECT)
        fp.write("config:    sdk/thread/config/%s\n" % CONFIG)
        fp.write("toolchain: %s\n" % toolchain_version())
        fp.write("flags:     %s\n" % " ".join(COMMON))
        fp.write("           C: %s  C++: %s\n" % (" ".join(CFLAGS), " ".join(CXXFLAGS)))
        fp.write("defines:   %s\n" % " ".join(d for d in def_flags))
        fp.write("sources:   %d (ST's list minus %s)\n" % (len(files), ", ".join(SKIP[:-1])))
        fp.write("sha256:    %s\n" % sha)
    return sha, len(files)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--cube", default=DEFAULT_CUBE, help="STM32CubeWBA V1.6.1 package (default %(default)s)")
    ap.add_argument("--out", default=LIB)
    ap.add_argument("-j", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--check", action="store_true", help="rebuild to a temp file and compare with --out")
    a = ap.parse_args()

    if a.check:
        tmpd = tempfile.mkdtemp(prefix="ot-trim-check-")
        tmp = os.path.join(tmpd, os.path.basename(a.out))
        build(os.path.abspath(a.cube), tmp, a.j)
        same = open(tmp, "rb").read() == open(a.out, "rb").read()
        shutil.rmtree(tmpd)
        print("ok: %s is reproducible" % os.path.relpath(a.out, ROOT) if same else
              "DIFFERS: %s does not match a rebuild (toolchain or config changed?)" % os.path.relpath(a.out, ROOT))
        return 0 if same else 1

    sha, n = build(os.path.abspath(a.cube), os.path.abspath(a.out), a.j)
    print("built %s from %d sources, sha256 %s" % (os.path.relpath(a.out, ROOT), n, sha))
    return 0


if __name__ == "__main__":
    sys.exit(main())
