# Project templates

A minimal, buildable starting point for every Core. Each folder is a complete
project — `Makefile`, `config.json`, `main.c` — that blinks the status LED and
nothing else. Copy one, rename it, start writing code.

```bash
cp -r templates/core-st-l4-2 ~/dev/my-firmware
cd ~/dev/my-firmware
make SDK_ROOT=/path/to/tiles
```

The folder name becomes the project name and the binary name, so
`my-firmware/build/my-firmware.bin` is what you get. `SDK_ROOT` only needs
passing once you've copied the template out of this repo — inside it, the
Makefile finds the SDK two levels up. Set it in the Makefile if you'd rather
not repeat it, or keep the project inside the SDK checkout.

> **Windows:** build from Git Bash or MSYS2, never `cmd.exe` or PowerShell, and
> keep the project on a path with no spaces in it (`C:\dev\…`, not
> `C:\Users\First Last\…` — make cannot build from those). `make doctor` checks
> the rest of the toolchain.

## The templates

| Folder | Core | MCU | Flashing | USB |
|---|---|---|---|---|
| [`core-st-l0`](core-st-l0/) | `Core.ST.L0` | STM32L011 | SWD (`make flash`) | — |
| [`core-st-l4`](core-st-l4/) | `Core.ST.L4` | STM32L422 | USB DFU (`make flash-dfu`) | CDC |
| [`core-st-l4-2`](core-st-l4-2/) | `Core.ST.L4.2` | STM32L422 | USB DFU (`make flash-dfu`) | CDC |
| [`core-st-w5`](core-st-w5/) | `Core.ST.W5` | STM32WBA55 | SWD via CubeProgrammer | — |
| [`core-st-h5`](core-st-h5/) | `Core.ST.H5` | STM32H523 | USB DFU (`make flash-dfu`) | CDC |

`core-st-l4` and `core-st-l4-2` are for two different boards: the Core.ST.L4
(called Core.ST.L4.1 until the 2026-09 rename) and the larger Core.ST.L4.2.
They share the STM32L422 but not the pad map, so pick the one printed on your
board. (The templates were `core-st-l0-1`, `core-st-l4-1` and `core-st-h5-1`
before the rename.)

Every template uses the clock level its Core JSON declares as the default
(`"clock": "medium"`); raise it in `config.json` when you need the headroom.

## Why the USB templates carry more config

The three USB-capable Cores ship in the fleet-standard shape, which is three
settings working together:

- **`"bootloader": "rom"`** puts the app at `0x08000000` and leaves recovery to
  the ST ROM bootloader in system memory — the one thing an app flash can never
  erase. This is what `make flash-dfu` and Studio's flasher both target.
- **`"usb": { "enabled": true }`** brings CDC up inside `core_init()`, before
  `main()` runs, so the 1200-baud touch can always reboot the board into DFU
  even if your code hangs immediately afterwards.
- **`"iwdg"`** starts a 5 s watchdog. Stop feeding it and the Core resets;
  reset repeatedly and it parks itself in ROM DFU rather than soft-bricking.
  See [`docs/l4-brick-recovery.md`](../docs/l4-brick-recovery.md).

The cost is one `core_watchdog_feed()` call in the main loop. Drop the `iwdg`
block from `config.json` if you'd rather not think about it — you lose the
automatic un-bricking, not the ability to flash.

The L0 and W5 templates leave all three out: neither Core has USB, so there is
no DFU to escape into and an un-fed watchdog would only reset-loop.

## Next steps

- **Add hardware** — pads, buses, and tiles are declared in `config.json`, not
  in C. [`config-json.md`](../config-json.md) is the authoring reference.
- **Add tiles** — flip `TILES_ENABLED` to `1` in the Makefile once `config.json`
  declares a `"tiles"` array.
- **Something bigger** — [`examples/`](../examples/) has USB HID, audio
  streaming, and WASM projects to crib from; BLE lives in `tests/ble-*`.
