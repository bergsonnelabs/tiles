#!/usr/bin/env python3
"""Read hw-spi-loopback's result over SWD (Core.ST.W5 on a CoreProbe, or any Core).

    python3 read_swd.py                 # read g_spi_lb + the report text
    python3 read_swd.py --flash         # make distclean; make TILE=Core.ST.W5;
                                        # probe-rs download + reset; quiet wait; read
    python3 read_swd.py --rerun         # write 'R' to g_spi_lb_cmd, wait, read

Power the W5 first (see README.md). Needs probe-rs and arm-none-eabi-nm.
SWD stays quiet while the Core boots: an attach that lands on a W5 boot can
crash it (PINNED.md, 2026-09-26).
"""
import argparse, os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ELF = os.path.join(HERE, "build", "hw-spi-loopback.elf")
FIELDS = ["magic", "verdict", "pass", "fail", "expected", "runs", "sck_fast_hz", "sck_slow_hz",
          "poll_Bps", "dma_Bps", "t5_poll_us", "t5_dma_us", "t5_async_us", "t5_gated_us",
          "t5_bound_us", "cs_edges", "first_err"]
LOG_LEN = 1600


def run(cmd, **kw):
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode:
        raise SystemExit(f"FAIL: {' '.join(cmd)}\n{r.stdout}{r.stderr}")
    return r.stdout


def symbol(name):
    m = re.search(r"^([0-9a-f]{8}) \w " + name + "$", run(["arm-none-eabi-nm", ELF]), re.M)
    if not m:
        raise SystemExit(f"FAIL: {name} not in {ELF}")
    return int(m.group(1), 16)


def read(chip):
    out = run(["probe-rs", "read", "--chip", chip, "b32", hex(symbol("g_spi_lb")), str(len(FIELDS))])
    words = [int(w, 16) for w in re.findall(r"\b[0-9a-f]{8}\b", out)][-len(FIELDS):]
    res = dict(zip(FIELDS, words))
    out = run(["probe-rs", "read", "--chip", chip, "b8", hex(symbol("g_spi_lb_log")), str(LOG_LEN)])
    data = bytes(int(b, 16) for b in re.findall(r"\b[0-9a-f]{2}\b", out)[-LOG_LEN:])
    return res, data.split(b"\0", 1)[0].decode(errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", default="STM32WBA55CG")
    ap.add_argument("--flash", action="store_true", help="build for Core.ST.W5, flash, reset")
    ap.add_argument("--rerun", action="store_true", help="write 'R' to g_spi_lb_cmd first")
    ap.add_argument("--quiet", type=float, default=6.0, help="seconds without SWD after reset")
    a = ap.parse_args()
    if a.flash:
        run(["make", "distclean"], cwd=HERE)
        run(["make", "TILE=Core.ST.W5"], cwd=HERE)
        run(["probe-rs", "download", "--chip", a.chip, "--binary-format", "elf", ELF])
        run(["probe-rs", "reset", "--chip", a.chip])
        print(f"flashed; no SWD for {a.quiet:.0f} s while it boots and runs", flush=True)
        time.sleep(a.quiet)
    if a.rerun:
        run(["probe-rs", "write", "--chip", a.chip, "b32", hex(symbol("g_spi_lb_cmd")), hex(ord("R"))])
        time.sleep(3)
    res, text = read(a.chip)
    if res.get("magic") != 0x5B1100B5:
        print(f"no result yet (magic 0x{res.get('magic', 0):08x})")
        return 2
    v = res["verdict"]
    name = "PASS" if v == 0x600D600D else "SKIP" if v == 0x5C1B0000 else "FAIL"
    print(f"[hw-spi-loopback] {name}  (pass=0x{res['pass']:02x} fail=0x{res['fail']:02x} "
          f"run {res['runs']}, first_err 0x{res['first_err']:08x})")
    print(text.replace("\r", ""), end="")
    return 0 if name == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
