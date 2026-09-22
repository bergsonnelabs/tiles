#!/usr/bin/env python3
"""Host test for sdk/core/core_scope.c.

Compiles the module with the host C compiler against a scripted link
(tools/test_core_scope_harness.c), runs each scenario, and checks what the
"device" sent with the reference decoder (tools/scope_decode.py).

Run:  python3 tools/test_core_scope.py
"""
from __future__ import annotations

import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from scope_decode import ScopeDecoder, crc8  # noqa: E402

CC = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
FLAGS = ["-std=c17", "-Wall", "-Wextra", "-Wshadow", "-Wdouble-promotion", "-Werror", "-O1",
         "-I", str(ROOT / "sdk/core")]

_build = tempfile.TemporaryDirectory()
_exe: Path | None = None


def harness() -> Path:
    global _exe
    if _exe is None:
        _exe = Path(_build.name) / "harness"
        subprocess.run([CC, *FLAGS, "-DCORE_SCOPE_HOST_TEST", "-o", str(_exe),
                        str(ROOT / "tools/test_core_scope_harness.c"),
                        str(ROOT / "sdk/core/core_scope.c")], check=True)
    return _exe


class Run:
    """One scenario: the link writes, the text, and everything decoded."""

    def __init__(self, scenario: str):
        out = subprocess.run([str(harness()), scenario], check=True, capture_output=True).stdout
        self.packets: list[bytes] = []
        self.order: list[tuple[str, bytes]] = []
        self.device_dropped = None
        i = 0
        while i < len(out):
            tag = chr(out[i])
            if tag == "S":
                (self.device_dropped,) = struct.unpack_from("<I", out, i + 1); i += 5; continue
            (n,) = struct.unpack_from("<H", out, i + 1)
            body = out[i + 3:i + 3 + n]; i += 3 + n
            self.order.append((tag, body))
            if tag == "P":
                self.packets.append(body)
        self.dec = ScopeDecoder()
        self.samples = []
        for _, body in self.order:          # text and frames share one byte stream
            self.samples += self.dec.feed(body)


def frame(ftype: int, seq: int, payload: bytes) -> bytes:
    body = bytes([ftype, seq, len(payload)]) + payload
    return b"\xA5\x53" + body + bytes([crc8(body)])


@unittest.skipIf(CC is None, "no host C compiler")
class CoreScope(unittest.TestCase):
    def test_legacy_frames_are_byte_identical(self):
        """i32 / f32 / bool produce exactly what Studio's generated scope sent,
        so a Studio that predates core_scope plots this firmware unchanged."""
        r = Run("legacy")
        desc = bytes([3]) + b"".join(bytes([t, len(n)]) + n for t, n in
                                     [(0, b"distance_mm"), (1, b"temp_c"), (2, b"pressed")])
        sid = crc8(desc)
        want = frame(0x01, 0, bytes([sid]) + desc)
        want += frame(0x02, 1, bytes([sid]) + struct.pack("<IifI", 1005, -1234, 21.5, 1))
        self.assertEqual(b"".join(r.packets), want)

    def test_all_types_and_shared_text(self):
        r = Run("basic")
        self.assertEqual(r.dec.names, ["distance_mm", "temp_c", "pressed", "big",
                                       "accel[0]", "accel[1]", "accel[2]", "lvl", "isr_value"])
        self.assertEqual([c.type for c in r.dec.channels],
                         ["i32", "f32", "bool", "u32", "i16", "i16", "i16", "u8", "i16"])
        self.assertEqual(len(r.samples), 50)
        for i, s in enumerate(r.samples):
            self.assertEqual(s.values, [i - 25, i * 0.5, bool(i & 1), 4000000000 + i,
                                        -i, i * 100, -32768, (200 + i) & 0xFF, i * -3])
            self.assertAlmostEqual(s.t, (2 * (i + 1)) / 1000.0)
        self.assertEqual(bytes(r.dec.text), b"hello \xA5 world\n" * 5)
        self.assertEqual((r.dec.dropped, r.dec.bad_frames, r.device_dropped), (0, 0, 0))

    def test_interval_gate(self):
        r = Run("interval")
        self.assertEqual(len(r.samples), 10)
        gaps = {round((b.t - a.t) * 1000) for a, b in zip(r.samples, r.samples[1:])}
        self.assertEqual(gaps, {10})

    def test_declared_rate_rows_and_drop_accounting(self):
        r = Run("rows")
        self.assertGreater(r.device_dropped, 0, "the stall should have overflowed the ring")
        self.assertEqual(r.dec.dropped, r.device_dropped, "host and device must agree on the loss")
        self.assertEqual(len(r.samples) + r.device_dropped, 400)
        for s in r.samples:                  # value == sample number: nothing mislabeled
            self.assertEqual(s.values, [s.n, s.n * 2, -s.n, 7])
            self.assertTrue(math.isclose(s.t, s.n / 1000.0))
        data = b"".join(r.packets)
        # 8-byte rows. Timestamped frames would cost 19 bytes a sample; rows
        # pumped every 5 samples cost 11, and the header amortizes further
        # the more samples a pump finds waiting.
        self.assertLess(len(data) / len(r.samples), 12)

    def test_ble_packets(self):
        r = Run("ble")
        self.assertTrue(all(len(p) <= 180 for p in r.packets))
        self.assertEqual(r.dec.bad_frames, 0)
        self.assertEqual(len(r.dec.names), 5)
        # Only a frame longer than a packet may straddle packets.
        stream, cuts, pos = b"".join(r.packets), set(), 0
        for p in r.packets:
            pos += len(p); cuts.add(pos)
        i = split = 0
        while i < len(stream):
            self.assertEqual(stream[i:i + 2], b"\xA5\x53")
            end = i + 6 + stream[i + 4]
            if any(i < c < end for c in cuts):
                split += 1
                self.assertGreater(end - i, 180, "a frame that fits a packet was split")
                self.assertEqual(stream[i + 2], 0x01)
            i = end
        self.assertGreater(split, 0, "the scenario should exercise the oversized schema")
        # Batching: several 31-byte frames share a notification.
        self.assertGreater(len(r.samples) / len(r.packets), 2)
        # The refusing radio cost samples, and both ends know how many.
        self.assertGreater(r.device_dropped, 0)
        self.assertEqual(r.dec.dropped, r.device_dropped)
        ones = [s.values[0] for s in r.samples]
        self.assertEqual(ones, sorted(ones))

    def test_reconnect_and_pause(self):
        r = Run("reconnect")
        kinds = []
        for tag, body in r.order:
            kinds.append(body.decode() if tag == "T" else "P")
        gone, paused = kinds.index("|gone|"), kinds.index("|paused|")
        first_after = [b for t, b in r.order[gone + 1:] if t == "P"][0]
        self.assertEqual(first_after[:3], b"\xA5\x53\x01", "the channel list leads after a reconnect")
        first_after = [b for t, b in r.order[paused + 1:] if t == "P"][0]
        self.assertEqual(first_after[:3], b"\xA5\x53\x01")
        vals = [s.values[0] for s in r.samples]
        self.assertFalse([v for v in vals if 1000 <= v < 2000 or 3000 <= v < 4000],
                         "nothing sampled while nobody listened, or while paused")
        self.assertTrue(any(v >= 4000 for v in vals))

    def test_partial_writes_keep_frames_whole(self):
        r = Run("trickle")
        self.assertTrue(all(len(p) <= 7 for p in r.packets))
        self.assertEqual(r.dec.bad_frames, 0)
        self.assertEqual(bytes(r.dec.text), b"")
        self.assertGreater(len(r.samples), 5)

    def test_spec_example_frames(self):
        """The example frames printed in docs/scope-protocol.md decode as the
        page says. Other decoders are checked against the same bytes."""
        import re
        doc = (ROOT / "docs/scope-protocol.md").read_text()
        blocks = re.findall(r"```\n((?:[0-9a-f ]+\n)+)```", doc.split("## Example frames")[1])
        self.assertEqual(len(blocks), 2)
        d = ScopeDecoder()
        got = d.feed(bytes.fromhex(blocks[0]))
        self.assertEqual(d.names, ["distance_mm", "temp_c", "pressed"])
        self.assertEqual([(s.t, s.values) for s in got], [(1.005, [-1234, 21.5, True])])
        d = ScopeDecoder()
        desc = bytes([4]) + b"".join(bytes([4, len(n)]) + n for n in
                                     [b"isr_value", b"accel[0]", b"accel[1]", b"accel[2]"])
        self.assertEqual(crc8(desc), 0x20)  # the example shows the rows frame alone
        d.feed(frame(0x01, 0, bytes([0x20]) + desc))
        got = d.feed(bytes.fromhex(blocks[1]))
        self.assertEqual([(s.n, s.t, s.values) for s in got], [(0, 0.0, [0, 0, 0, 7])])

    def test_limits(self):
        self.assertEqual(subprocess.run([str(harness()), "limits"]).returncode, 0)

    def test_compiled_out_is_nothing(self):
        """CORE_SCOPE_ENABLED=0: every call vanishes, a watched variable stays
        'used', and nothing needs core_scope.c at link time."""
        src = Path(_build.name) / "off.c"
        src.write_text('#include "core_scope.h"\n'
                       "static int a; static float b; static short c[3];\n"
                       "int main(void) {\n"
                       "  core_scope_init(); core_scope_watch(a); core_scope_watch_as(\"x\", b);\n"
                       "  core_scope_watch_array(c, 3); core_scope_set_interval_ms(5);\n"
                       "  core_scope_declare_rate_hz(100); core_scope_enable(1);\n"
                       "  core_scope_sample(); core_scope_pump(); core_scope_update();\n"
                       "  return core_scope_active() + (int)core_scope_dropped();\n}\n")
        exe = Path(_build.name) / "off"
        subprocess.run([CC, *FLAGS, "-DCORE_SCOPE_ENABLED=0", "-o", str(exe), str(src),
                        str(ROOT / "sdk/core/core_scope.c")], check=True)
        self.assertEqual(subprocess.run([str(exe)]).returncode, 0)


if __name__ == "__main__":
    unittest.main()
