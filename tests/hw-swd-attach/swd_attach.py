#!/usr/bin/env python3
"""Hot-attach series through the CoreProbe (CMSIS-DAP v1 over HID), raw SWD.

Each attach starts cold and never resets the target: DAP_Connect, clock, line
reset, DPIDR, ABORT, the CDBGPWRUPREQ/ACK handshake (timed), AP1 (the
Cortex-M33 AHB-AP) CSW, CPUID, DBGMCU IDCODE (H5: 0x44024000, the CPU alias
AP1 reaches), the image's g_swd block, halt, PC, resume, power-down request,
DAP_Disconnect. A failure reports the step, the ACK (WAIT / FAULT / NOACK,
+PARITY on a data parity error) and CTRL/STAT's sticky bits after a line reset.

With hw-swd-attach flashed, it also checks the target stays up between
attaches: the heartbeat must grow and the boot counter must not move.

    python3 swd_attach.py                         # 20 attaches at 100 kHz, 1 MHz, 4 MHz
    python3 swd_attach.py --khz 4000 --count 50
    python3 swd_attach.py --poll 35               # one session, read g_swd for 35 s

Power the target first (tools/coreprobe_power.py). The probe senses the target
and sets its level shifter on every DAP_Connect; this script never touches the
supply or the level. Needs hidapi (pip install hidapi) and arm-none-eabi-nm.
Close anything else holding the probe (Studio, probe-rs, OpenOCD) first.
"""
import argparse
import os
import struct
import subprocess
import sys
import time

VID, PID = 0x1209, 0xDA01
ACKS = {1: "OK", 2: "WAIT", 4: "FAULT", 7: "NOACK", 0: "NONE"}
DP_IDR, DP_ABORT, DP_CTRL, DP_SELECT = 0x0, 0x0, 0x4, 0x8
AP_CSW, AP_TAR, AP_DRW = 0x0, 0x4, 0xC
DHCSR, DCRSR, DCRDR, CPUID = 0xE000EDF0, 0xE000EDF4, 0xE000EDF8, 0xE000ED00
PWRUP_REQ = 0x50000000        # CSYSPWRUPREQ | CDBGPWRUPREQ
PWRUP_ACK = 0xA0000000        # CSYSPWRUPACK | CDBGPWRUPACK
HERE = os.path.dirname(os.path.abspath(__file__))


def ackname(a):
    s = ACKS.get(a & 7, f"ack{a & 7}")
    return s + ("+PARITY" if a & 8 else "") + ("+MISMATCH" if a & 16 else "")


class SwdError(Exception):
    def __init__(self, step, ack):
        super().__init__(f"{step}: {ackname(ack)}")
        self.step, self.ack = step, ack


class Dap:
    def __init__(self):
        import hid
        self.d = hid.device()
        self.d.open(VID, PID)
        self.d.set_nonblocking(0)

    def close(self):
        self.d.close()

    def cmd(self, p):
        self.d.write(bytes([0]) + bytes(p) + bytes(64 - len(p)))
        r = bytes(self.d.read(64, 2000))
        if not r or r[0] != p[0]:
            raise IOError(f"no/bad response to DAP command 0x{p[0]:02x}")
        return r

    def connect(self, khz):
        if self.cmd([0x02, 1])[1] != 1:
            raise IOError("DAP_Connect(SWD) refused")
        self.cmd([0x11] + list(struct.pack("<I", khz * 1000)))       # SWJ_Clock
        self.cmd([0x04, 0] + list(struct.pack("<HH", 100, 100)))     # TransferConfigure

    def disconnect(self):
        self.cmd([0x03])

    def line_reset(self):
        for n, data in ((56, b"\xff" * 7), (16, b"\x9e\xe7"), (56, b"\xff" * 7), (8, b"\x00")):
            self.cmd([0x12, n] + list(data))                          # SWJ_Sequence

    def xfer(self, ops, step):
        """ops: [(request, value)]; request bits 0 APnDP, 1 RnW, 2-3 A[3:2]."""
        p = [0x05, 0, len(ops)]
        for req, val in ops:
            p.append(req)
            if not req & 2:
                p += list(struct.pack("<I", val & 0xFFFFFFFF))
        r = self.cmd(p)
        if r[1] != len(ops) or r[2] != 1:
            raise SwdError(step, r[2])
        n = sum(1 for req, _ in ops if req & 2)
        return list(struct.unpack_from(f"<{n}I", r, 3))

    @staticmethod
    def req(addr, ap, rnw):
        return ((addr >> 2) & 3) << 2 | (1 if ap else 0) | (2 if rnw else 0)

    def dp_r(self, a, step):
        return self.xfer([(self.req(a, 0, 1), 0)], step)[0]

    def dp_w(self, a, v, step):
        self.xfer([(self.req(a, 0, 0), v)], step)

    def mem_ap(self, apsel, step):
        self.dp_w(DP_SELECT, apsel << 24, step)
        csw = self.xfer([(self.req(AP_CSW, 1, 1), 0)], step)[0]
        self.xfer([(self.req(AP_CSW, 1, 0), (csw & ~0x3F) | 0x12)], step)   # 32-bit, auto-inc

    def rd(self, addr, n=1, step="read"):
        return self.xfer([(self.req(AP_TAR, 1, 0), addr)] + [(self.req(AP_DRW, 1, 1), 0)] * n, step)

    def wr(self, addr, v, step="write"):
        self.xfer([(self.req(AP_TAR, 1, 0), addr), (self.req(AP_DRW, 1, 0), v)], step)

    def power_up(self):
        self.line_reset()
        idr = self.dp_r(DP_IDR, "dpidr")
        self.dp_w(DP_ABORT, 0x1E, "abort")
        self.dp_w(DP_SELECT, 0, "select")
        self.dp_w(DP_CTRL, PWRUP_REQ, "pwrup_req")
        t = time.perf_counter()
        polls = 0
        while True:
            c = self.dp_r(DP_CTRL, "pwrup_poll")
            polls += 1
            if c & PWRUP_ACK == PWRUP_ACK:
                return idr, (time.perf_counter() - t) * 1000, polls
            if time.perf_counter() - t > 0.5:
                raise SwdError(f"pwrup_timeout (CTRL/STAT 0x{c:08x})", 1)

    def sticky(self):
        """After a failure: line reset, then CTRL/STAT's sticky bits."""
        try:
            self.line_reset()
            self.dp_r(DP_IDR, "diag_dpidr")
            c = self.dp_r(DP_CTRL, "diag_ctrl")
            self.dp_w(DP_ABORT, 0x1E, "diag_abort")
            bits = [n for b, n in ((1, "STICKYORUN"), (4, "STICKYCMP"), (5, "STICKYERR"),
                                   (7, "WDATAERR")) if c >> b & 1]
            return f"CTRL/STAT 0x{c:08x} {' '.join(bits) or 'no sticky bits'}"
        except (SwdError, IOError) as e:
            return f"diagnosis failed: {e}"


def symbols(elf):
    out = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True, check=True).stdout
    return {l.split()[2]: int(l.split()[0], 16) for l in out.splitlines() if len(l.split()) == 3}


def attach(dap, khz, g_swd, idcode_addr):
    r = {}
    t0 = time.perf_counter()
    dap.connect(khz)
    r["dpidr"], r["pwrup_ms"], r["polls"] = dap.power_up()
    dap.mem_ap(1, "ap1")
    r["cpuid"] = dap.rd(CPUID, step="cpuid")[0]
    r["idcode"] = dap.rd(idcode_addr, step="idcode")[0] if idcode_addr else None
    r["hb"], r["ms"], r["boot"] = dap.rd(g_swd, 3, "g_swd") if g_swd else (None,) * 3
    dap.wr(DHCSR, 0xA05F0003, "halt")
    if not any(dap.rd(DHCSR, step="halt_poll")[0] & 1 << 17 for _ in range(50)):
        raise SwdError("halt_timeout", 1)
    dap.wr(DCRSR, 15, "dcrsr")
    if not any(dap.rd(DHCSR, step="regrdy_poll")[0] & 1 << 16 for _ in range(50)):
        raise SwdError("regrdy_timeout", 1)
    r["pc"] = dap.rd(DCRDR, step="pc")[0]
    dap.wr(DHCSR, 0xA05F0001, "resume")
    dap.wr(DHCSR, 0xA05F0000, "debugen_off")
    if dap.rd(DHCSR, step="running")[0] & 1 << 17:
        raise SwdError("still_halted", 1)
    dap.dp_w(DP_SELECT, 0, "select_end")
    dap.dp_w(DP_CTRL, 0, "pwrdn_req")
    dap.disconnect()
    r["total_ms"] = (time.perf_counter() - t0) * 1000
    return r


def series(dap, khz, n, gap, g_swd, idcode_addr):
    ok, prev, resets, fails = 0, None, 0, []
    for i in range(n):
        try:
            r = attach(dap, khz, g_swd, idcode_addr)
        except (SwdError, IOError) as e:
            diag = dap.sticky() if isinstance(e, SwdError) else ""
            fails.append(f"#{i} {e} {diag}")
            print(f"  {i:2d} FAIL {e}  {diag}", flush=True)
            try:
                dap.disconnect()
            except IOError:
                pass
            time.sleep(gap)
            continue
        ok += 1
        note = ""
        if prev and g_swd:
            if r["boot"] != prev["boot"] or r["hb"] < prev["hb"]:
                resets += 1
                note = "  <- target reset since last attach"
        prev = r
        extra = f" hb={r['hb']} boot={r['boot']}" if g_swd else ""
        idc = f" idcode=0x{r['idcode']:08x}" if r["idcode"] is not None else ""
        print(f"  {i:2d} ok pwrup {r['pwrup_ms']:.1f} ms/{r['polls']} poll(s){idc} "
              f"pc=0x{r['pc']:08x}{extra} {r['total_ms']:.0f} ms{note}", flush=True)
        time.sleep(gap)
    print(f"== {khz} kHz: {ok}/{n} attached" + (f", {resets} target reset(s) seen" if g_swd else ""))
    return ok == n and not resets


def poll(dap, khz, secs, g_swd):
    dap.connect(khz)
    dap.power_up()
    dap.mem_ap(1, "ap1")
    t0, reads, errs, last = time.time(), 0, 0, None
    while time.time() - t0 < secs:
        try:
            hb, ms, boot = dap.rd(g_swd, 3, "g_swd")
            dh = dap.rd(DHCSR, step="dhcsr")[0]
            reads += 1
            if dh & 1 << 25 or (last and (boot != last[2] or hb < last[0])):
                print(f"  {time.time() - t0:6.2f} s: target reset (ms={ms} boot={boot})", flush=True)
            last = (hb, ms, boot)
        except SwdError as e:
            errs += 1
            print(f"  {time.time() - t0:6.2f} s: {e}  {dap.sticky()}", flush=True)
            try:
                dap.dp_w(DP_CTRL, PWRUP_REQ, "repower")
                dap.mem_ap(1, "ap1")
            except SwdError as e2:
                print(f"  {time.time() - t0:6.2f} s: re-power failed: {e2}", flush=True)
                time.sleep(0.1)
    dap.dp_w(DP_SELECT, 0, "select_end")
    dap.dp_w(DP_CTRL, 0, "pwrdn_req")
    dap.disconnect()
    print(f"== poll {secs:.0f} s at {khz} kHz: {reads} reads, {errs} errors")
    return errs == 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--khz", default="100,1000,4000", help="comma-separated SWD clocks")
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--gap", type=float, default=0.6, help="seconds between attaches")
    ap.add_argument("--elf", default=os.path.join(HERE, "build", "hw-swd-attach.elf"),
                    help="image on the target, for the g_swd address (skipped if missing)")
    ap.add_argument("--idcode", default="0x44024000",
                    help="DBGMCU IDCODE as AP1 sees it (H5 default); 0 to skip")
    ap.add_argument("--poll", type=float, help="instead: one session, read g_swd for N seconds")
    a = ap.parse_args()

    g_swd = symbols(a.elf).get("g_swd") if os.path.isfile(a.elf) else None
    if not g_swd:
        print(f"note: no g_swd in {a.elf}; attaching without the liveness check")
    idcode = int(a.idcode, 0) or None
    dap = Dap()
    try:
        khz = [int(k) for k in a.khz.split(",")]
        if a.poll:
            if not g_swd:
                sys.exit("--poll needs the hw-swd-attach ELF")
            return 0 if poll(dap, khz[0], a.poll, g_swd) else 1
        good = all([series(dap, k, a.count, a.gap, g_swd, idcode) for k in khz])
        return 0 if good else 1
    finally:
        dap.close()


if __name__ == "__main__":
    sys.exit(main())
