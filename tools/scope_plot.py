#!/usr/bin/env python3
"""Live plot of a Core's Scope stream (docs/scope-protocol.md) over USB.

Works with any firmware that uses core_scope.h: the stream describes its own
channels, so nothing here is specific to one tile. Channels named `base[i]`
share a subplot (accel_g[0..2] -> one "accel_g" panel, three lines).

    python3 tools/scope_plot.py                    # auto-picks /dev/cu.usbmodem*
    python3 tools/scope_plot.py --port /dev/cu.usbmodem1101 --window 5
    python3 tools/scope_plot.py --csv run.csv      # also log every sample

Print text sharing the port is echoed to the terminal.
Needs pyserial and matplotlib.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from collections import OrderedDict, deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scope_decode import ScopeDecoder  # noqa: E402


def find_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No USB CDC port found (/dev/cu.usbmodem*). Is the Core plugged in and running?")
    if len(ports) > 1:
        print(f"Several ports, using {ports[0]}: {', '.join(ports)}", file=sys.stderr)
    return ports[0]


def group_of(name: str) -> str:
    m = re.fullmatch(r"(.+)\[\d+\]", name)
    return m.group(1) if m else name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history shown (default 10)")
    ap.add_argument("--csv", help="also append every sample to this CSV file")
    args = ap.parse_args()

    import serial
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    port = args.port or find_port()
    # Opening the port asserts DTR, which is what tells the Core a host is listening.
    ser = serial.Serial(port, 115200, timeout=0)
    try:
        ser.dtr = True
    except OSError:
        pass                                  # not a real modem line (a pty, a replay)
    print(f"Listening on {port} — waiting for the schema (sent every second)…", file=sys.stderr)

    dec = ScopeDecoder()
    state = {"schema": None, "lines": [], "axes": [], "t": deque(), "cols": [], "csv": None, "last_t": None}
    fig = plt.figure(figsize=(10, 7))
    fig.canvas.manager.set_window_title(f"Scope — {port}")
    fig.text(0.01, 0.005, "waiting for schema…", fontsize=8, color="0.4")

    def rebuild():
        """New channel list (first schema, or the firmware changed): start over."""
        fig.clf()
        groups: OrderedDict[str, list[int]] = OrderedDict()
        for i, n in enumerate(dec.names):
            groups.setdefault(group_of(n), []).append(i)
        state["t"] = deque()
        state["cols"] = [deque() for _ in dec.names]
        state["lines"], state["axes"] = [None] * len(dec.names), []
        for k, (g, idxs) in enumerate(groups.items()):
            ax = fig.add_subplot(len(groups), 1, k + 1)
            ax.set_ylabel(g)
            ax.grid(True, alpha=0.3)
            for i in idxs:
                (state["lines"][i],) = ax.plot([], [], lw=1.2, label=dec.names[i])
            if len(idxs) > 1:
                ax.legend(loc="upper left", fontsize=8, ncol=len(idxs))
            state["axes"].append(ax)
        state["axes"][-1].set_xlabel("device time (s)")
        state["status"] = fig.text(0.01, 0.005, "", fontsize=8, color="0.4")
        fig.tight_layout(rect=(0, 0.02, 1, 1))
        state["schema"] = dec.schema_id
        if args.csv:
            if state["csv"]:
                state["csv"].close()
            state["csv"] = open(args.csv, "a")
            state["csv"].write(",".join(["t"] + dec.names) + "\n")
        print(f"Schema {dec.schema_id:#04x}: {', '.join(dec.names)}", file=sys.stderr)

    def tick(_frame):
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as e:
            print(f"\nPort lost: {e}", file=sys.stderr)
            plt.close(fig)
            return []
        samples = dec.feed(chunk) if chunk else []
        if dec.text:
            sys.stdout.write(dec.text.decode("utf-8", "replace"))
            sys.stdout.flush()
            dec.text.clear()
        if dec.schema_id is not None and dec.schema_id != state["schema"]:
            rebuild()
        if state["schema"] is None:
            return []
        for s in samples:
            if state["last_t"] is not None and s.t < state["last_t"] - 1.0:
                rebuild()                              # device rebooted: clock went backwards
            state["last_t"] = s.t
            state["t"].append(s.t)
            for col, v in zip(state["cols"], s.values):
                col.append(float(v))
            if state["csv"]:
                state["csv"].write(",".join([f"{s.t:.6f}"] + [str(float(v)) for v in s.values]) + "\n")
        if not state["t"]:
            return []
        t_end = state["t"][-1]
        while state["t"] and state["t"][0] < t_end - args.window:
            state["t"].popleft()
            for col in state["cols"]:
                col.popleft()
        t = list(state["t"])
        for line, col in zip(state["lines"], state["cols"]):
            line.set_data(t, list(col))
        for ax in state["axes"]:
            ax.set_xlim(max(t[0], t_end - args.window), t_end if t_end > t[0] else t[0] + 1e-3)
            ax.relim()
            ax.autoscale_view(scalex=False)
        latest = "  ".join(f"{n}={c[-1]:.3f}" for n, c in zip(dec.names, state["cols"]))
        state["status"].set_text(f"{latest}   |  frames {dec.frames}  dropped {dec.dropped}  bad {dec.bad_frames}")
        return []

    anim = FuncAnimation(fig, tick, interval=33, cache_frame_data=False)  # noqa: F841 (keep a reference)
    try:
        plt.show()
    finally:
        ser.close()
        if state["csv"]:
            state["csv"].close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
