#!/usr/bin/env python3
"""Check that a Core.ST.W5 BLE image's install copier is self-contained.

The BLE update's copier (sdk/ble/ble_update_boot.c, sdk/serial_update/su_ota.c)
runs while every flash page but page 0 is being replaced, so the code it runs
(sections .ota_boot in page 0 and .ota_ram, loaded into SRAM from page 0) must
never branch to, or load a pointer into, the rest of the image. A compiler that
turns a loop into a memcpy call, a switch into a flash-resident table, or an
inline helper into an out-of-line one would break an install only when power is
cut mid-copy. This catches that at build time by disassembling both sections:

  * every branch target must lie inside .ota_boot or .ota_ram
    (Reset_Handler, the startup's own code in .ota_boot, branches on to
    Reset_Continue, linked with the program, after the hook returns, but its
    first call must be Reset_EarlyHook);
  * no literal-pool word may point into flash outside page 0, except the
    reserved page (0x080F8000..) the copier reads as data;
  * both sections must end inside page 0 (the linker script asserts it too).

Usage: check_ota_boot.py [--objdump arm-none-eabi-objdump] image.elf
Exit 0 = ok, 1 = a violation (printed), 2 = could not run.
"""

import argparse
import re
import subprocess
import sys

PAGE0_END = 0x08002000
RES_PAGE = 0x080F8000
FLASH_END = 0x08100000


def sections(objdump, elf):
    out = subprocess.run([objdump, "-h", elf], capture_output=True, text=True, check=True).stdout
    res = {}
    for line in out.splitlines():
        m = re.match(r"\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)", line)
        if m:
            name, size, vma, lma = m.group(1), int(m.group(2), 16), int(m.group(3), 16), int(m.group(4), 16)
            res[name] = (vma, size, lma)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--objdump", default="arm-none-eabi-objdump")
    ap.add_argument("elf")
    a = ap.parse_args()
    try:
        secs = sections(a.objdump, a.elf)
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"check_ota_boot: cannot run {a.objdump}: {e}", file=sys.stderr)
        return 2

    errors = []
    if ".ota_boot" not in secs or ".ota_ram" not in secs:
        print("check_ota_boot: no .ota_boot / .ota_ram sections (not a BLE W5 image?)", file=sys.stderr)
        return 1
    boot_vma, boot_size, _ = secs[".ota_boot"]
    ram_vma, ram_size, ram_lma = secs[".ota_ram"]
    allowed = [(boot_vma, boot_vma + boot_size), (ram_vma, ram_vma + ram_size)]
    if boot_vma + boot_size > PAGE0_END or ram_lma + ram_size > PAGE0_END:
        errors.append(f".ota_boot / .ota_ram end past page 0 (0x{max(boot_vma + boot_size, ram_lma + ram_size):08x})")

    dis = subprocess.run([a.objdump, "-d", "-j", ".ota_boot", "-j", ".ota_ram", a.elf],
                         capture_output=True, text=True, check=True).stdout
    func = None
    reset_first_call = None
    for line in dis.splitlines():
        m = re.match(r"^([0-9a-f]{8}) <(.+)>:$", line)
        if m:
            func = m.group(2)
            continue
        m = re.match(r"^\s*([0-9a-f]+):\s+(?:[0-9a-f]{4}\s?)+\s*\t(\S+)\s*(.*)$", line)
        if not m:
            continue
        addr, mnem, ops = int(m.group(1), 16), m.group(2), m.group(3)

        if mnem.startswith(".word"):
            if func == "Reset_Handler":
                continue                               # .data / .bss bounds: used after the hook
            vm = re.match(r"0x([0-9a-f]+)", ops)
            if vm:
                v = int(vm.group(1), 16) & ~1
                if 0x08000000 <= v < FLASH_END and PAGE0_END <= v < RES_PAGE:
                    errors.append(f"{func} @0x{addr:08x}: literal 0x{v:08x} points into flash outside page 0")
            continue

        is_branch = re.match(r"^(b|bl|blx|cbz|cbnz)(\.n|\.w)?$|^b(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)(\.n|\.w)?$", mnem)
        if not is_branch:
            continue
        tm = re.search(r"\b([0-9a-f]{6,8})\s+<([^>]+)>", ops)
        if not tm:
            continue                                   # register branch (blx r3): checked via literals
        target, tname = int(tm.group(1), 16), tm.group(2)
        if func == "Reset_Handler":
            if mnem.startswith("bl") and reset_first_call is None:
                reset_first_call = tname
            continue                                   # the startup proper, after the hook
        if not any(lo <= target < hi for lo, hi in allowed):
            errors.append(f"{func} @0x{addr:08x}: {mnem} to 0x{target:08x} <{tname}> leaves .ota_boot/.ota_ram")

    if reset_first_call != "Reset_EarlyHook":
        errors.append(f"Reset_Handler's first call is {reset_first_call!r}, not Reset_EarlyHook")

    if errors:
        print("check_ota_boot: the BLE-update copier is not self-contained:", file=sys.stderr)
        for e in errors:
            print("  " + e, file=sys.stderr)
        return 1
    print(f"  OTA   copier ok ({boot_size} + {ram_size} B in page 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
