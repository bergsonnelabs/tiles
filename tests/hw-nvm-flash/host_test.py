#!/usr/bin/env python3
"""Host side of tests/hw-nvm-flash: the whole run, plus T4 (NVM survives a
reflash). Needs python3 + pyserial (L4) or probe-rs (W5).

    python3 host_test.py               # Core.ST.L4, reflash with flash-serial
    python3 host_test.py --dfu         # ... the T4 reflashes with flash-dfu instead
    python3 host_test.py --tile Core.ST.L4.2
    python3 host_test.py --w5          # Core.ST.W5 on a probe: probe-rs download

Steps:
  1. build and flash this test; start a fresh run ('R'); wait for the report
     (T1-T3, T6 on the L4 / W5; T5 in a W5 BLE build) and its T4 token
  2. flash a different image (the Core's starter template) the same way and
     check it runs
  3. flash this test again: it finds the armed token in NVM and checks the
     token, the T1 pattern, the counter and the 600-byte region are intact
The W5 needs its probe powered first (tools/coreprobe_power.py apply ...).
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SDK = os.path.abspath(os.path.join(HERE, "..", ".."))
VID = 0x1209
TEMPLATE = {"Core.ST.L4": "core-st-l4", "Core.ST.L4.2": "core-st-l4-2", "Core.ST.W5": "core-st-w5"}
W5_CHIP = "STM32WBA55CG"


def log(msg):
    print(msg, flush=True)


def run(cmd, cwd=None):
    log("  $ " + " ".join(cmd))
    p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if p.returncode != 0:
        sys.stdout.write(p.stdout[-3000:])
        raise SystemExit(f"FAIL: command exited {p.returncode}")
    return p.stdout


# ---------------------------------------------------------------- L4 (CDC)

def cdc_port():
    from serial.tools import list_ports
    return next((p.device for p in list_ports.comports() if p.vid == VID), None)


def cdc_collect(seconds, send=None, until=None):
    """Read the Core's CDC for up to `seconds`, reopening across resets."""
    import serial
    end, buf, sent = time.time() + seconds, "", send is None
    while time.time() < end:
        port = cdc_port()
        if not port:
            time.sleep(0.2)
            continue
        try:
            with serial.Serial(port, 115200, timeout=0.3) as s:
                if not sent:
                    time.sleep(0.3)
                    s.write(send.encode())
                    sent = True
                while time.time() < end:
                    buf += s.read(4096).decode(errors="replace")
                    if until and re.search(until, buf):
                        return buf
        except Exception:
            time.sleep(0.3)
    return buf


def last_report(text):
    i = text.rfind("[hw-nvm-flash]")
    return text[i:] if i >= 0 else ""


def parse(report):
    r = {"verdict": None, "tests": {}, "token": None, "state": None}
    m = re.search(r"\[hw-nvm-flash\] (PASS|FAIL)", report)
    if m:
        r["verdict"] = m.group(1)
    for name, res in re.findall(r"^\s+(T\d [a-z ]+?)\s{2,}(PASS|FAIL|not run)", report, re.M):
        r["tests"][name.strip()] = res
    m = re.search(r"token 0x([0-9a-f]{8}) \((armed|checked)", report)
    if m:
        r["token"], r["state"] = m.group(1), m.group(2)
    return r


def l4_flash(project_dir, project, tile, dfu):
    target = "flash-dfu" if dfu else "flash-serial"
    run(["make", "-C", SDK, f"TILE={tile}", f"PROJECT={project}", f"PROJECT_DIR={project_dir}",
         "TILES_ENABLED=0", "BLE_ENABLED=0", "V=0", target])


def l4_main(args):
    tile = args.tile
    log(f"[1] flash hw-nvm-flash ({tile}) and run it fresh")
    l4_flash(HERE, "hw-nvm-flash", tile, False)
    time.sleep(2)
    cdc_collect(3, send="R")
    text = cdc_collect(240, until=r"\(armed: reflash")
    rep = parse(last_report(text))
    log(last_report(text).split("\n\r\n")[0] if rep["verdict"] else text[-2000:])
    if rep["state"] != "armed":
        raise SystemExit("FAIL: no armed report within 240 s")
    token = rep["token"]
    log(f"    run: {rep['verdict']}, T4 token 0x{token}")

    other = os.path.join(HERE, "build", "other")
    shutil.rmtree(other, ignore_errors=True)
    os.makedirs(other)
    tdir = os.path.join(SDK, "templates", TEMPLATE[tile])
    for f in ("main.c", "config.json"):
        shutil.copy(os.path.join(tdir, f), other)
    how = "flash-dfu" if args.dfu else "flash-serial"
    log(f"[2] flash the {TEMPLATE[tile]} starter with {how}")
    l4_flash(other, "other", tile, args.dfu)
    ticks = cdc_collect(8, until=r"tick \d+\r\n.*tick \d+")
    ran = re.search(r"tick \d+", ticks) is not None
    log(f"    starter running: {'yes' if ran else 'NO'}")

    log(f"[3] flash hw-nvm-flash back with {how}")
    l4_flash(HERE, "hw-nvm-flash", tile, args.dfu)
    text = cdc_collect(30, until=r"\(checked after reflash\)")
    rep2 = parse(last_report(text))
    log(last_report(text).split("\n\r\n")[0])
    t4 = rep2["tests"].get("T4 reflash")
    ok = ran and rep2["state"] == "checked" and rep2["token"] == token and t4 == "PASS"
    log(f"    token before 0x{token}, after 0x{rep2['token']}; T4 {t4}")
    verdict = rep["verdict"] == "PASS" and ok
    log(f"\n[hw-nvm-flash host] {'PASS' if verdict else 'FAIL'}  run {rep['verdict']}, "
        f"reflash ({how}) {'PASS' if ok else 'FAIL'}")
    return 0 if verdict else 1


# ---------------------------------------------------------------- W5 (SWD)

RESULT_FIELDS = ["magic", "verdict", "pass", "fail", "expected", "phase", "t2_compactions",
                 "t2_max_ms", "pl_old", "pl_new", "pl_garbage", "pl_ecc", "token",
                 "t5_irq_before", "t5_irq_during", "t5_irq_after", "t5_max_gap_ms",
                 "t5_writes", "t5_max_write_ms", "last_rc", "t5_burst_ms",
                 "t7_store_ms", "t7_clear_ms"]


def symbol(elf, name):
    out = run(["arm-none-eabi-nm", elf])
    m = re.search(r"^([0-9a-f]{8}) \w " + name + "$", out, re.M)
    if not m:
        raise SystemExit(f"FAIL: {name} not in {elf}")
    return int(m.group(1), 16)


def w5_read(elf):
    addr = symbol(elf, "g_nvm_result")
    out = run(["probe-rs", "read", "--chip", W5_CHIP, "b32", hex(addr), str(len(RESULT_FIELDS))])
    words = [int(w, 16) for w in re.findall(r"\b[0-9a-f]{8}\b", out)][-len(RESULT_FIELDS):]
    return dict(zip(RESULT_FIELDS, words))


def w5_flash(elf):
    run(["probe-rs", "download", "--chip", W5_CHIP, "--binary-format", "elf", elf])
    run(["probe-rs", "reset", "--chip", W5_CHIP])


# SWD reads while the test resets itself (about 30 times in its first ~100 s)
# can crash the W5: a probe-rs attach that lands on a boot has left it in a
# HardFault / lockup inside core_clock_init (bench, 2026-09-26). So the run
# gets a quiet period first, and the polling after it is slow.
W5_QUIET_S = 150


def w5_wait(elf, want_phase, seconds, quiet=0):
    if quiet:
        log(f"    no SWD for {quiet} s while the test resets itself")
        time.sleep(quiet)
    end = time.time() + seconds
    while time.time() < end:
        time.sleep(5)
        try:
            r = w5_read(elf)
        except SystemExit:
            continue
        if r["magic"] == 0x4E564D52 and r["phase"] in want_phase:
            return r
    raise SystemExit(f"FAIL: no result with phase {want_phase} in {seconds} s")


def w5_main(args):
    ble = ["BLE=1"] if args.ble else []
    run(["make", "distclean"], cwd=HERE)
    run(["make", "TILE=Core.ST.W5", *ble], cwd=HERE)
    elf = os.path.join(HERE, "build", "hw-nvm-flash.elf")
    log("[1] flash hw-nvm-flash (W5) and run it fresh")
    w5_flash(elf)
    # Whatever it boots into (a fresh run after a power-up, a T4 check, or a
    # run the backup registers say is half done) has settled after the quiet
    # period; only then is it safe to ask for a fresh run over SWD.
    r = w5_wait(elf, (3, 4), 240, quiet=W5_QUIET_S)
    log(f"    settled in phase {r['phase']}; starting a fresh run")
    run(["probe-rs", "write", "--chip", W5_CHIP, "b32", hex(symbol(elf, "g_nvm_cmd")), hex(ord("R"))])
    r = w5_wait(elf, (3,), 240, quiet=W5_QUIET_S)   # PH_ARMED
    log("    " + ", ".join(f"{k}={v:#x}" for k, v in r.items()))
    token = r["token"]

    other = os.path.join(HERE, "build", "other")
    shutil.rmtree(other, ignore_errors=True)
    os.makedirs(other)
    for f in ("main.c", "config.json"):
        shutil.copy(os.path.join(SDK, "templates", "core-st-w5", f), other)
    log("[2] flash the core-st-w5 starter")
    run(["make", "-C", SDK, "TILE=Core.ST.W5", "PROJECT=other", f"PROJECT_DIR={other}",
         "TILES_ENABLED=0", "V=0", "all"])
    w5_flash(os.path.join(other, "build", "other.elf"))
    time.sleep(3)

    log("[3] flash hw-nvm-flash back")
    w5_flash(elf)
    r2 = w5_wait(elf, (4,), 60, quiet=15)       # PH_DONE after the T4 check
    t4 = bool(r2["pass"] & (1 << 3))
    ok = r2["token"] == token and t4
    log(f"    token before {token:#010x}, after {r2['token']:#010x}; T4 {'PASS' if t4 else 'FAIL'}")
    verdict = r["verdict"] == 0x600D600D and ok
    log(f"\n[hw-nvm-flash host] {'PASS' if verdict else 'FAIL'}")
    return 0 if verdict else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tile", default="Core.ST.L4", choices=["Core.ST.L4", "Core.ST.L4.2"])
    ap.add_argument("--dfu", action="store_true", help="L4: reflash with flash-dfu (ROM DFU)")
    ap.add_argument("--w5", action="store_true", help="Core.ST.W5 over probe-rs")
    ap.add_argument("--ble", action="store_true", help="W5: the BLE build (T5)")
    args = ap.parse_args()
    return w5_main(args) if args.w5 else l4_main(args)


if __name__ == "__main__":
    sys.exit(main())
