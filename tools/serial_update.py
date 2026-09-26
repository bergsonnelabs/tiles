#!/usr/bin/env python3
"""Flash a running Core over its USB serial port — no bootloader, no driver.

    python3 tools/serial_update.py build/app.bin              # auto-detect the Core
    python3 tools/serial_update.py --port COM5 build/app.bin
    make flash-serial                                         # same, from a project

Opening the Core's CDC port at 2400 baud hands it to the SRAM flasher
(sdk/serial_update/), which rewrites flash and restarts into the new image.
Core.ST.L4 and Core.ST.H5; the flasher reports its chip, flash and page size.
Works wherever the serial port does — including Windows with no Zadig/WinUSB.
Needs firmware built with an SDK that has serial update; older firmware
doesn't answer, and this exits 2 so you can fall back to `make flash-dfu` once.

Protocol: docs/serial-update-protocol.md. Studio's client
(web apps/studio/src/lib/serialUpdate.ts) mirrors this file.
"""

import argparse
import io
import struct
import sys
import time
import zlib

# Python entry points in tools/ force UTF-8 on their own streams (Windows).
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace", line_buffering=True)

# ---- su_protocol.h ----
TRIGGER_BAUD = 2400
HDR = struct.Struct("<2sBBHHII")      # 'SU', type, flags, len, reserved, arg0, arg1
T_QUERY, T_BEGIN, T_DATA, T_END, T_ABORT = (ord(c) for c in "QBDEX")
ERR_CRC = 7
CORE_USB_VID = 0x1209
DEV_ID_L41X_L42X = 0x464
DEV_ID_H523_H533 = 0x478
NVM_RESERVED = 4096                   # SU_NVM_RESERVED: what a v1 (L4) flasher keeps at the top

# Which chip an image is for, from its initial stack pointer: the top of SRAM
# less the 16 bytes the ROM-DFU linker scripts keep for the DFU magic. Only a
# fallback for when --dev-id isn't given (make flash-serial always gives it).
DEV_ID_BY_SP = {
    0x20009FF0: DEV_ID_L41X_L42X, 0x2000A000: DEV_ID_L41X_L42X,   # STM32L422, 40 KB
    0x20043FF0: DEV_ID_H523_H533, 0x20044000: DEV_ID_H523_H533,   # STM32H523, 272 KB
}

ERR_WORDS = {
    1: "payload too long", 2: "image is for a different chip", 3: "image size",
    4: "image does not start with a valid vector table", 5: "flash erase/program failed",
    6: "chunk out of order", 7: "chunk CRC", 8: "whole-image CRC", 9: "protocol state",
    10: "unknown frame",
}


class NotSupported(Exception):
    """The Core didn't answer: its firmware predates serial update."""


class UpdateError(Exception):
    pass


def frame(ftype: int, arg0: int = 0, arg1: int = 0, payload: bytes = b"") -> bytes:
    return HDR.pack(b"SU", ftype, 0, len(payload), 0, arg0 & 0xFFFFFFFF, arg1 & 0xFFFFFFFF) + payload


class Link:
    """Byte transport: write() bytes, readline() one reply line or None on timeout."""

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read_some(self, timeout: float) -> bytes:
        raise NotImplementedError

    def __init__(self):
        self._buf = b""

    def readline(self, timeout: float):
        deadline = time.monotonic() + timeout
        while b"\n" not in self._buf:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self._buf += self.read_some(left)
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("ascii", "replace").strip()


class SerialLink(Link):
    def __init__(self, ser):
        super().__init__()
        self.ser = ser

    def write(self, data):
        self.ser.write(data)
        self.ser.flush()

    def read_some(self, timeout):
        self.ser.timeout = min(timeout, 0.05)
        return self.ser.read(self.ser.in_waiting or 1)


def reply(link: Link, timeout: float, want_ready: bool = False):
    """Next protocol line. Skipped: anything the app printed before the handoff,
    and — unless asking for it — a READY left over from a repeated query."""
    deadline = time.monotonic() + timeout
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return None
        line = link.readline(left)
        if line is None:
            return None
        if line.startswith("SU ") and (want_ready or not line.startswith("SU READY")):
            return line.split()


def expect(link: Link, want: str, timeout: float):
    r = reply(link, timeout)
    if r is None:
        raise UpdateError(f"no reply (waiting for SU {want})")
    if r[1] == "ERR":
        code = int(r[2]) if len(r) > 2 and r[2].isdigit() else 0
        raise UpdateError(f"Core refused: {ERR_WORDS.get(code, ' '.join(r[2:]))} (SU ERR {code})")
    if r[1] != want:
        raise UpdateError(f"unexpected reply: {' '.join(r)}")
    return r


def query(link: Link, timeout: float = 1.5) -> dict:
    """Ask the flasher who it is. Retries until `timeout`: the handoff takes a moment."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.write(frame(T_QUERY))
        r = reply(link, 0.25, want_ready=True)
        if r and r[1] == "READY" and len(r) >= 7:
            flash = int(r[4]) * 1024
            # v2 flashers report the largest image they take; v1 (the L4's)
            # keep SU_NVM_RESERVED at the top.
            limit = int(r[7]) if len(r) >= 8 and r[7].isdigit() else flash - NVM_RESERVED
            return {"version": int(r[2]), "dev_id": int(r[3], 16), "flash": flash,
                    "page": int(r[5]), "page0_erased": r[6] == "1", "limit": limit}
    raise NotSupported("no answer from the flasher")


def image_dev_id(image: bytes):
    """DEV_ID for an image, guessed from its initial SP (None if unknown)."""
    if len(image) < 8:
        return None
    return DEV_ID_BY_SP.get(struct.unpack_from("<I", image)[0])


def update(link: Link, image: bytes, dev_id: int = DEV_ID_L41X_L42X, log=print) -> None:
    info = query(link)
    log(f"flasher v{info['version']}: device 0x{info['dev_id']:03x}, "
        f"{info['flash'] // 1024} KB flash, {info['page']} B pages"
        + (" (previous update did not finish)" if info["page0_erased"] else ""))
    if info["dev_id"] != dev_id:
        raise UpdateError(f"this Core is device 0x{info['dev_id']:03x}, the image is for 0x{dev_id:03x}")
    # The top pages are core_nvm's store: an image that reached them would
    # erase the saved data (the SDK's linker scripts stop short of them).
    limit = info["limit"]
    if len(image) > limit:
        raise UpdateError(f"image is {len(image)} B; the most that fits below the "
                          f"core_nvm pages is {limit} B")

    page = info["page"]
    link.write(frame(T_BEGIN, len(image), zlib.crc32(image), struct.pack("<I", dev_id)))
    expect(link, "OK", 2.0)

    t0 = time.monotonic()
    for off in range(0, len(image), page):
        chunk = image[off:off + page]
        for attempt in range(4):
            link.write(frame(T_DATA, off, zlib.crc32(chunk), chunk))
            r = reply(link, 2.0)
            if r and r[1] == "OK" and len(r) >= 4 and int(r[3]) == off + len(chunk):
                break
            if r and r[1] == "ERR" and r[2] != str(ERR_CRC):
                code = int(r[2])
                raise UpdateError(f"Core refused chunk @{off}: {ERR_WORDS.get(code, code)} (SU ERR {code})")
            if attempt == 3:
                raise UpdateError(f"chunk @{off}: {' '.join(r) if r else 'no reply'}")
        log(f"  {off + len(chunk):>7} / {len(image)} B", end="\r")
    log("")

    link.write(frame(T_END))
    expect(link, "DONE", 10.0)
    log(f"flashed {len(image)} B in {time.monotonic() - t0:.1f} s — the Core is restarting")


def find_core_port():
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid == CORE_USB_VID]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise UpdateError("no Core found on USB (VID 0x1209) — is it plugged in and running?")
    raise UpdateError("more than one Core: pick one with --port " + " / ".join(p.device for p in ports))


def main() -> int:
    ap = argparse.ArgumentParser(description="Flash a running Core over USB serial (no bootloader).")
    ap.add_argument("image", help="raw binary for 0x08000000 (build/<project>.bin)")
    ap.add_argument("--port", help="serial port (default: the one Core on USB)")
    ap.add_argument("--dev-id", type=lambda s: int(s, 0), default=None,
                    help="expected DEV_ID: 0x464 STM32L41x/L42x (Core.ST.L4), 0x478 "
                         "STM32H523/533 (Core.ST.H5); default: from the image's stack pointer")
    args = ap.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        print("serial_update: needs pyserial — pip install pyserial", file=sys.stderr)
        return 1

    with open(args.image, "rb") as f:
        image = f.read()
    dev_id = args.dev_id if args.dev_id is not None else image_dev_id(image)
    if dev_id is None:
        print("serial_update: can't tell which chip the image is for — pass --dev-id",
              file=sys.stderr)
        return 1
    try:
        port = args.port or find_core_port()
        print(f"{port}: handing the Core to its flasher…")
        with serial.Serial(port, TRIGGER_BAUD, timeout=0.05) as ser:
            update(SerialLink(ser), image, dev_id,
                   log=lambda *a, **k: print(*a, **k))
        return 0
    except NotSupported:
        print("serial_update: the Core didn't answer — its firmware predates serial update.\n"
              "  Flash it once with `make flash-dfu`; after that, `make flash-serial` works.",
              file=sys.stderr)
        return 2
    except (UpdateError, OSError) as e:
        print(f"serial_update: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
