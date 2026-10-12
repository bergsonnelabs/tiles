#!/usr/bin/env python3
"""thread_gw_listen.py: print Thread beacons forwarded over BLE by tests/hw-thread-ble-gw.

Run in Terminal.app (it needs Bluetooth permission):
    python3 tools/thread_gw_listen.py   (pip install bleak)

Finds "W5 Thread GW", subscribes to every notify characteristic it exposes,
and decodes the 16-byte payload:
    gw rx count u32 | sender counter u32 | sender rloc16 u16 |
    gw role u8 | gw neighbors u8 | gw uptime s u32        (little-endian)
"""
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

NAME = "W5 Thread GW"
ROLES = {0: "disabled", 1: "detached", 2: "child", 3: "router", 4: "leader"}


def on_notify(_char, data: bytearray) -> None:
    if len(data) < 16:
        print(f"{time.strftime('%H:%M:%S')}  short packet ({len(data)} B): {data.hex()}")
        return
    gw_rx, counter, rloc, role, nbrs, up = struct.unpack("<IIHBBI", bytes(data[:16]))
    print(f"{time.strftime('%H:%M:%S')}  beacon #{counter:<6} from 0x{rloc:04x}   "
          f"gateway: rx {gw_rx:<6} role {ROLES.get(role, role):<8} "
          f"neighbors {nbrs}  up {up}s")


async def main() -> None:
    print(f"scanning for '{NAME}' ...")
    dev = await BleakScanner.find_device_by_name(NAME, timeout=20.0)
    if dev is None:
        sys.exit(f"'{NAME}' not found (is the gateway powered and advertising?)")
    print(f"found {dev.address}, connecting ...")
    async with BleakClient(dev) as client:
        subscribed = 0
        for svc in client.services:
            for ch in svc.characteristics:
                if "notify" in ch.properties:
                    await client.start_notify(ch, on_notify)
                    print(f"subscribed to {ch.uuid} in service {svc.uuid}")
                    subscribed += 1
        if not subscribed:
            sys.exit("no notify characteristics found")
        print("listening (Ctrl-C to stop) ...")
        while client.is_connected:
            await asyncio.sleep(1)
        print("disconnected")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
