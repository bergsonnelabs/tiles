#!/usr/bin/env python3
"""Reference decoder for the Scope stream (docs/scope-protocol.md).

The smallest complete host: feed it bytes from any link (a serial port, BLE
notifications, a capture file) and it hands back samples plus whatever was
not a frame (print text sharing the port). Dependency-free on purpose — this
is the implementation other decoders are checked against, and the seed of
the Python client.

    from scope_decode import ScopeDecoder
    dec = ScopeDecoder()
    for chunk in port:                       # bytes, any chunking
        for s in dec.feed(chunk):
            print(s.t, dict(zip(dec.names, s.values)))

CLI:  python3 tools/scope_decode.py capture.bin      # prints CSV
"""
from __future__ import annotations

import struct
import sys
from dataclasses import dataclass, field

MAGIC = b"\xA5\x53"
FRAME_SCHEMA, FRAME_DATA, FRAME_ROWS = 0x01, 0x02, 0x03
KNOWN_FRAMES = (FRAME_SCHEMA, FRAME_DATA, FRAME_ROWS)

# type code -> (name, struct format, wire bytes)
TYPES = {
    0: ("i32", "<i", 4),
    1: ("f32", "<f", 4),
    2: ("bool", "<i", 4),
    3: ("u32", "<I", 4),
    4: ("i16", "<h", 2),
    5: ("u16", "<H", 2),
    6: ("i8", "<b", 1),
    7: ("u8", "<B", 1),
}


def crc8(data: bytes, crc: int = 0) -> int:
    """CRC-8, poly 0x07, init 0."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass
class Channel:
    name: str
    type: str
    fmt: str
    size: int


@dataclass
class Sample:
    t: float            # seconds on the device's clock
    values: list
    n: int | None = None  # sample number (rows frames only)


@dataclass
class ScopeDecoder:
    channels: list[Channel] = field(default_factory=list)
    schema_id: int | None = None
    text: bytearray = field(default_factory=bytearray)  # non-frame bytes, in order
    dropped: int = 0          # samples the device reported losing
    bad_frames: int = 0       # CRC-valid frames that did not parse
    frames: int = 0
    _buf: bytearray = field(default_factory=bytearray)
    _seq: int | None = None
    _next_n: int | None = None

    @property
    def names(self) -> list[str]:
        return [c.name for c in self.channels]

    @property
    def row_bytes(self) -> int:
        return sum(c.size for c in self.channels)

    def feed(self, chunk: bytes) -> list[Sample]:
        self._buf += chunk
        buf, out, i = self._buf, [], 0
        while i < len(buf):
            if buf[i] != MAGIC[0]:
                self.text.append(buf[i]); i += 1; continue
            if i + 1 >= len(buf):
                break                                   # wait for the next byte
            if buf[i + 1] != MAGIC[1]:
                self.text.append(buf[i]); i += 1; continue
            if i + 5 > len(buf):
                break                                   # header incomplete
            ftype, seq, ln = buf[i + 2], buf[i + 3], buf[i + 4]
            if ftype not in KNOWN_FRAMES:
                self.text.append(buf[i]); i += 1; continue
            total = 6 + ln
            if i + total > len(buf):
                break                                   # body incomplete
            if crc8(bytes(buf[i + 2:i + 5 + ln])) != buf[i + 5 + ln]:
                self.text.append(buf[i]); i += 1; continue
            out += self._frame(ftype, seq, bytes(buf[i + 5:i + 5 + ln]))
            i += total
        del buf[:i]
        return out

    def _frame(self, ftype: int, seq: int, p: bytes) -> list[Sample]:
        self.frames += 1
        rows_mode = ftype == FRAME_ROWS
        if self._seq is not None and not rows_mode:
            self.dropped += (seq - self._seq - 1) & 0xFF
        self._seq = seq

        if ftype == FRAME_SCHEMA:
            return self._schema(p)
        if not p or p[0] != self.schema_id or not self.channels:
            return []                                   # channel list not known yet
        if ftype == FRAME_DATA:
            if len(p) != 5 + self.row_bytes:
                self.bad_frames += 1; return []
            (ms,) = struct.unpack_from("<I", p, 1)
            return [Sample(ms / 1000.0, self._row(p, 5))]
        # FRAME_ROWS
        rb = self.row_bytes
        if len(p) < 9 + rb or (len(p) - 9) % rb:
            self.bad_frames += 1; return []
        n0, rate_mhz = struct.unpack_from("<II", p, 1)
        if rate_mhz == 0:
            self.bad_frames += 1; return []
        if self._next_n is not None:
            self.dropped += (n0 - self._next_n) & 0xFFFFFFFF
        count = (len(p) - 9) // rb
        self._next_n = (n0 + count) & 0xFFFFFFFF
        return [Sample((n0 + k) * 1000.0 / rate_mhz, self._row(p, 9 + k * rb), n0 + k)
                for k in range(count)]

    def _row(self, p: bytes, off: int) -> list:
        vals = []
        for c in self.channels:
            (v,) = struct.unpack_from(c.fmt, p, off)
            vals.append(bool(v) if c.type == "bool" else v)
            off += c.size
        return vals

    def _schema(self, p: bytes) -> list[Sample]:
        if len(p) < 2 or crc8(p[1:]) != p[0]:
            self.bad_frames += 1; return []
        chans, off = [], 2
        for _ in range(p[1]):
            if off + 2 > len(p) or p[off] not in TYPES or off + 2 + p[off + 1] > len(p):
                self.bad_frames += 1; return []
            tname, fmt, size = TYPES[p[off]]
            nlen = p[off + 1]
            chans.append(Channel(p[off + 2:off + 2 + nlen].decode("utf-8", "replace"), tname, fmt, size))
            off += 2 + nlen
        if p[0] != self.schema_id:                      # new firmware / new channel list
            self._next_n = None
        self.schema_id, self.channels = p[0], chans
        return []


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__); return 2
    dec, header = ScopeDecoder(), False
    data = sys.stdin.buffer.read() if argv[1] == "-" else open(argv[1], "rb").read()
    for s in dec.feed(data):
        if not header:
            print(",".join(["t"] + dec.names)); header = True
        print(",".join([f"{s.t:.6f}"] + [str(int(v) if isinstance(v, bool) else v) for v in s.values]))
    print(f"# frames={dec.frames} dropped={dec.dropped} bad={dec.bad_frames} text_bytes={len(dec.text)}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
