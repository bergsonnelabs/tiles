#!/usr/bin/env python3
"""Host side of tests/hw-usb-robust (Core.ST.L4.1). Needs python3 + pyserial.

    python3 host_test.py            # robustness checks (~45 s)
    python3 host_test.py sleep      # sleep + reachability (flash-serial while asleep)

Robustness checks, in order:
  early   after a reset, the first line a terminal sees is the EARLY line the
          Core printed right after core_init(), before the port was open
  serial  the USB serial number is the chip UID the firmware reads, not 000001
  zlp     writes of exactly 64 and 128 bytes arrive without a following write
  burst   4096 bytes sent in one go to a slow reader arrive intact (NAK)
  dtr     text written while DTR is low arrives after it is raised, and the
          writes didn't wait
  stall   the host stops reading for 30 s while the Core prints flat out: no
          write blocks longer than the bound (max latency reported)

Sleep mode (`sleep`): the Core loops core_stop_for(60). While it sleeps, a
line sent to it gets "PONG" from its USB interrupt, and `make flash-serial`
reflashes it without a replug or BOOT0; the CDC comes back afterwards.
"""

import os
import re
import subprocess
import sys
import time

import serial
from serial.tools import list_ports

VID = 0x1209
HERE = os.path.dirname(os.path.abspath(__file__))
results = []


def log(msg):
    print(msg, flush=True)


def record(name, ok, detail):
    results.append((name, ok, detail))
    log(f"  {'PASS' if ok else 'FAIL'}  {name:7s} {detail}")


def find_port(timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for p in list_ports.comports():
            if p.vid == VID:
                return p
        time.sleep(0.1)
    raise SystemExit("no Core (VID 0x1209) on USB")


def wait_gone(dev, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if not any(p.device == dev for p in list_ports.comports()):
            return True
        time.sleep(0.05)
    return False


def open_core(timeout=15.0):
    end = time.monotonic() + timeout
    while True:
        p = find_port(max(0.5, end - time.monotonic()))
        try:
            s = serial.Serial(p.device, 115200, timeout=0.05)
            s.dtr = True
            return s, p
        except (serial.SerialException, OSError):
            if time.monotonic() > end:
                raise
            time.sleep(0.2)


def send(s, text):
    s.write((text + "\n").encode())
    s.flush()


def read_lines(s, until=None, timeout=3.0, quiet=None):
    """Lines until one matches `until` (regex), `timeout`, or `quiet` s of silence."""
    lines, buf = [], b""
    end = time.monotonic() + timeout
    last = time.monotonic()
    while time.monotonic() < end:
        d = s.read(4096)
        if d:
            buf += d
            last = time.monotonic()
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                ln = ln.decode(errors="replace").rstrip("\r")
                lines.append(ln)
                if until and re.search(until, ln):
                    return lines, ln
        elif quiet and time.monotonic() - last > quiet:
            break
    return lines, None


def check_early(s, p):
    send(s, "X")                                    # reset the Core
    s.close()
    wait_gone(p.device)
    s, p = open_core()
    lines, _ = read_lines(s, timeout=2.0, quiet=0.8)
    first = next((ln for ln in lines if ln.strip()), "")
    record("early", first.startswith("EARLY"), f"first line: {first!r}")
    return s, p


def check_serial(s, p):
    send(s, "I")
    _, ln = read_lines(s, until=r"^ID ", timeout=2.0)
    m = re.search(r"serial=([0-9A-F]+)", ln or "")
    fw = m.group(1) if m else None
    ok = fw is not None and p.serial_number == fw and fw != "000001" and len(fw) == 24
    record("serial", ok, f"USB serial={p.serial_number} firmware UID={fw} tty={p.device}")


def check_zlp(s):
    for n in (64, 128):
        s.reset_input_buffer()
        send(s, f"Z {n}")
        t0 = time.monotonic()
        got = b""
        while len(got) < n and time.monotonic() - t0 < 1.5:
            got += s.read(n - len(got))
        dt = (time.monotonic() - t0) * 1000
        ok = got == b"z" * (n - 1) + b"\n" and dt < 300
        record("zlp", ok, f"{n}-byte write: {len(got)} bytes in {dt:.0f} ms (no following write)")


def fnv1a(data):
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def check_burst(s, n=4096):
    s.reset_input_buffer()
    send(s, f"B {n}")
    _, ln = read_lines(s, until=r"^BREADY", timeout=2.0)
    if not ln:
        record("burst", False, "no BREADY")
        return
    data = os.urandom(n)
    t0 = time.monotonic()
    s.write(data)
    s.flush()
    wr = (time.monotonic() - t0) * 1000
    _, ln = read_lines(s, until=r"^BRES", timeout=25.0)
    m = re.search(r"n=(\d+) fnv=([0-9a-f]+) ms=(\d+)", ln or "")
    ok = bool(m) and int(m.group(1)) == n and int(m.group(2), 16) == fnv1a(data)
    record("burst", ok, f"sent {n} B (host write {wr:.0f} ms); Core: {ln} host fnv={fnv1a(data):08x}")


def check_dtr(s):
    s.reset_input_buffer()
    send(s, "D 5")
    time.sleep(0.2)
    s.dtr = False
    time.sleep(0.3)
    early = s.read(4096)                            # nothing should come while DTR is low
    time.sleep(0.3)
    s.dtr = True
    lines, ln = read_lines(s, until=r"^DRES", timeout=5.0)
    q = [x for x in lines if x.startswith("Q ")]
    m = re.search(r"dtr_dropped=(\d) max_us=(\d+) short=(\d+) back=(\d)", ln or "")
    ok = (bool(m) and m.group(1) == "1" and m.group(4) == "1" and len(q) == 5
          and not early and int(m.group(2)) < 2000)
    record("dtr", ok, f"{len(q)}/5 lines queued while closed arrived after DTR rose, "
                      f"{len(early)} B leaked while low; Core: {ln}")


def check_stall(s, secs=30):
    s.reset_input_buffer()
    send(s, f"S {secs}")
    log(f"          (not reading for {secs + 1} s while the Core prints)")
    time.sleep(secs + 1)
    drained = 0
    t0 = time.monotonic()
    while True:                                     # drain what the host and Core buffered
        d = s.read(65536)
        drained += len(d)
        if not d and time.monotonic() - t0 > 1.0:
            break
        if d:
            t0 = time.monotonic()
    send(s, "R")
    _, ln = read_lines(s, until=r"^SRES", timeout=3.0)
    m = re.search(r"max_us=(\d+) calls=(\d+) short=(\d+)", ln or "")
    ok = bool(m) and int(m.group(1)) <= 60000
    record("stall", ok, f"Core: {ln}; host drained {drained} B afterwards "
                        f"(bound 50 ms + 1 tick, pass <= 60 ms)")


def robust():
    s, p = open_core()
    log(f"Core on {p.device} (serial {p.serial_number})")
    s, p = check_early(s, p)
    check_serial(s, p)
    check_zlp(s)
    check_burst(s)
    check_dtr(s)
    check_stall(s)
    s.close()


def sleep_mode(secs=60):
    s, p = open_core()
    send(s, f"P {secs}")
    _, ln = read_lines(s, until=r"^SLEEP k=0", timeout=3.0)
    log(f"  Core: {ln}")
    time.sleep(2.0)                                 # well inside core_stop_for()
    send(s, "ping")
    t0 = time.monotonic()
    _, pong = read_lines(s, until=r"^PONG", timeout=2.0)
    record("pong", pong is not None, f"reply while in core_stop_for({secs}): "
                                     f"{(time.monotonic() - t0) * 1000:.0f} ms")
    s.close()
    time.sleep(1.0)
    t0 = time.monotonic()
    r = subprocess.run(["make", "flash-serial"], cwd=HERE, capture_output=True, text=True)
    dt = time.monotonic() - t0
    tail = (r.stdout + r.stderr).strip().splitlines()[-2:]
    record("flash", r.returncode == 0, f"make flash-serial while asleep: exit {r.returncode} "
                                       f"in {dt:.1f} s: {' | '.join(tail)}")
    s, p = open_core()
    lines, ln = read_lines(s, until=r"^EARLY", timeout=5.0)
    record("back", ln is not None, f"CDC after the reflash: {ln!r}")
    send(s, f"P {secs}")                            # leave it sleeping again
    _, ln = read_lines(s, until=r"^SLEEP", timeout=3.0)
    log(f"  left sleeping: {ln}")
    s.close()


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "robust"
    if mode == "sleep":
        sleep_mode(int(sys.argv[2]) if len(sys.argv) > 2 else 60)
    else:
        robust()
    bad = [n for n, ok, _ in results if not ok]
    log(f"\n[hw-usb-robust {mode}] {'PASS' if not bad else 'FAIL'} "
        f"({len(results) - len(bad)}/{len(results)})")
    sys.exit(1 if bad else 0)
