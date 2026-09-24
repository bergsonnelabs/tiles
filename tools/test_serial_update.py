#!/usr/bin/env python3
"""Tests for serial update: the real protocol core (su_core.c) on a simulated
L4 flash, driven by the real host client (tools/serial_update.py).

    make -C sdk/serial_update test       # builds su_sim, then runs this

The claim these defend is the one the design rests on: whatever happens, the
Core boots EITHER the old app (flash untouched), OR the ROM bootloader (page 0
erased — the L4's empty check), OR the complete new image. Never a mix.
"""

import argparse
import os
import random
import select
import struct
import subprocess
import sys
import tempfile
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial_update as su  # noqa: E402

FLASH = 128 * 1024
PAGE = 2048
ERASED = b"\xff" * 8


class ProcessLink(su.Link):
    def __init__(self, proc):
        super().__init__()
        self.proc = proc

    def write(self, data):
        try:
            self.proc.stdin.write(data)
            self.proc.stdin.flush()
        except BrokenPipeError:
            pass

    def read_some(self, timeout):
        fd = self.proc.stdout.fileno()
        r, _, _ = select.select([fd], [], [], timeout)
        if not r:
            return b""
        data = os.read(fd, 4096)
        if not data:  # sim exited: nothing more will come
            select.select([], [], [], timeout)
        return data


def image(size, seed, sp=0x20009FF0, entry=0x080001C1):
    rnd = random.Random(seed)
    body = bytes(rnd.getrandbits(8) for _ in range(size - 8))
    return struct.pack("<II", sp, entry) + body


OLD = image(6 * 1024 + 100, seed=1)
NEW = image(9 * 1024 + 300, seed=2)            # 5 pages, the last one partial
OLD_FLASH = OLD + b"\xff" * (FLASH - len(OLD))


class Sim:
    def __init__(self, sim_path, *extra):
        self.dir = tempfile.mkdtemp(prefix="su_sim_")
        self.fin = os.path.join(self.dir, "in.bin")
        self.fout = os.path.join(self.dir, "out.bin")
        with open(self.fin, "wb") as f:
            f.write(OLD_FLASH)
        self.proc = subprocess.Popen([sim_path, self.fin, self.fout, *extra],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.link = ProcessLink(self.proc)

    def finish(self):
        """Close the link (host gone / power cut) and return (exit code, flash)."""
        try:
            self.proc.stdin.close()
        except BrokenPipeError:
            pass
        code = self.proc.wait(timeout=10)
        with open(self.fout, "rb") as f:
            return code, f.read()


def boots(flash):
    """What an L4 would boot from this flash after a power cycle."""
    if flash[:8] == ERASED:
        return "rom"
    if flash == OLD_FLASH:
        return "old"
    if flash[:len(NEW)] == NEW:
        return "new"
    return "CORRUPT"


def quiet(*_a, **_k):
    pass


FAILURES = []


def check(name, cond, detail=""):
    print(("  ok    " if cond else "  FAIL  ") + name + (f" — {detail}" if detail and not cond else ""))
    if not cond:
        FAILURES.append(name)


def frames_for(img, dev_id=0x464):
    fs = [su.frame(su.T_QUERY), su.frame(su.T_BEGIN, len(img), zlib.crc32(img), struct.pack("<I", dev_id))]
    for off in range(0, len(img), PAGE):
        chunk = img[off:off + PAGE]
        fs.append(su.frame(su.T_DATA, off, zlib.crc32(chunk), chunk))
    fs.append(su.frame(su.T_END))
    return fs


def run(sim_path):
    print("serial update — protocol core on a simulated L4")

    s = Sim(sim_path)
    su.update(s.link, NEW, log=quiet)
    code, fl = s.finish()
    check("happy path: new image written, Core rebooted", code == 0 and boots(fl) == "new", f"exit {code}, boots {boots(fl)}")
    check("happy path: pages past the image left alone", fl[len(NEW) + PAGE:] == OLD_FLASH[len(NEW) + PAGE:])

    s = Sim(sim_path)
    try:
        su.update(s.link, NEW, dev_id=0x415, log=quiet)
        refused = False
    except su.UpdateError:
        refused = True
    code, fl = s.finish()
    check("wrong chip: refused, flash untouched", refused and boots(fl) == "old")

    s = Sim(sim_path)
    s.link.write(su.frame(su.T_QUERY))
    su.reply(s.link, 1)
    s.link.write(su.frame(su.T_BEGIN, len(NEW), zlib.crc32(NEW), struct.pack("<I", 0x415)))
    r = su.reply(s.link, 1, want_ready=True)
    code, fl = s.finish()
    check("wrong DEV_ID in BEGIN: SU ERR 2, flash untouched", r[:3] == ["SU", "ERR", "2"] and boots(fl) == "old", str(r))

    for label, bad in (("SP outside SRAM", image(len(NEW), 2, sp=0x30000000)),
                       ("reset vector not Thumb", image(len(NEW), 2, entry=0x080001C0)),
                       ("reset vector past the image", image(len(NEW), 2, entry=0x08010001))):
        s = Sim(sim_path)
        try:
            su.update(s.link, bad, log=quiet)
            refused = False
        except su.UpdateError as e:
            refused = "vector table" in str(e)
        code, fl = s.finish()
        check(f"bad vectors ({label}): refused before any erase", refused and boots(fl) == "old", f"boots {boots(fl)}")

    s = Sim(sim_path)
    fs = frames_for(NEW)
    s.link.write(fs[0]); su.reply(s.link, 1)
    s.link.write(fs[1]); su.reply(s.link, 1)
    corrupt = bytearray(fs[2]); corrupt[-1] ^= 0xFF
    s.link.write(bytes(corrupt))
    r1 = su.reply(s.link, 1)
    s.link.write(fs[2])
    r2 = su.reply(s.link, 1)
    s.link.write(fs[2])                       # the same chunk again: its OK was "lost"
    r3 = su.reply(s.link, 1)
    code, fl = s.finish()
    check("corrupt chunk: SU ERR 7, then the resend is accepted", r1[:3] == ["SU", "ERR", "7"] and r2[1] == "OK", f"{r1} {r2}")
    check("duplicate chunk after a lost OK: acknowledged again", r3 == r2, f"{r3} vs {r2}")

    s = Sim(sim_path)
    lie = frames_for(NEW)
    lie[1] = su.frame(su.T_BEGIN, len(NEW), zlib.crc32(NEW) ^ 1, struct.pack("<I", 0x464))
    rs = []
    for fr in lie:
        s.link.write(fr)
        rs.append(su.reply(s.link, 2))
    code, fl = s.finish()
    check("whole-image CRC mismatch: SU ERR 8, page 0 NOT written", rs[-1][:3] == ["SU", "ERR", "8"] and boots(fl) == "rom", f"{rs[-1]} boots {boots(fl)}")

    s = Sim(sim_path)
    s.link.write(b"hello from the app\nSUxx garbage" + frames_for(NEW)[0])
    r = su.reply(s.link, 1, want_ready=True)
    s.finish()
    check("resyncs past app output and junk", r is not None and r[1] == "READY", str(r))

    s = Sim(sim_path, "--idle-ms", "10000")
    code, fl = s.finish()
    check("idle 10 s with flash untouched: reboots into the old app", code == 0 and boots(fl) == "old", f"exit {code}")

    s = Sim(sim_path, "--idle-ms", "600000")
    fs = frames_for(NEW)
    for fr in fs[:3]:
        s.link.write(fr); su.reply(s.link, 1)
    code, fl = s.finish()
    check("idle after page 0 erased: keeps waiting, never resets", code == 3 and boots(fl) == "rom", f"exit {code}")

    # Power cut after every frame of a full update.
    fs = frames_for(NEW)
    seen = []
    for k in range(len(fs) + 1):
        s = Sim(sim_path)
        for fr in fs[:k]:
            s.link.write(fr)
            su.reply(s.link, 2)
        code, fl = s.finish()
        seen.append(boots(fl))
    check("power cut after every frame: old / rom / new only, in that order",
          "CORRUPT" not in seen and seen == sorted(seen, key=["old", "rom", "new"].index)
          and seen[0] == "old" and seen[-1] == "new", " ".join(seen))

    # A flash write failure at every point.
    outcomes = set()
    for n in range(1, 40):
        s = Sim(sim_path, "--fail-program-at", str(n))
        try:
            su.update(s.link, NEW, log=quiet)
        except su.UpdateError:
            pass
        code, fl = s.finish()
        outcomes.add(boots(fl))
    check("program() failing at each call: never a corrupt boot", "CORRUPT" not in outcomes, str(outcomes))

    print(f"{'FAILED: ' + ', '.join(FAILURES) if FAILURES else 'all passed'}")
    return 1 if FAILURES else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True, help="path to the built su_sim")
    return run(os.path.abspath(ap.parse_args().sim))


if __name__ == "__main__":
    sys.exit(main())
