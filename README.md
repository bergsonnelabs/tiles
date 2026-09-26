# tiles

Firmware SDK and drivers for the Bergsonne **Mosaic** tile family: a HAL over
STM32 LL registers (no CubeIDE, no Cube HAL), a code generator that turns a
declarative `config.json` into board init code, and platform-agnostic drivers
for every tile in the catalog.

Full documentation lives at **[bergsonne.io/docs](https://bergsonne.io/docs)**.

## Quick start

You need `arm-none-eabi-gcc`, `make`, Python 3, and `dfu-util`. On macOS:

```sh
brew install arm-none-eabi-gcc dfu-util make python
```

Check the whole toolchain at once — it names anything missing, with the fix:

```sh
make doctor
```

Then copy the starter project for your Core, and build it:

```sh
cp -r templates/core-st-l4-2 ~/dev/my-firmware
cd ~/dev/my-firmware
make SDK_ROOT=/path/to/tiles
make flash-dfu                 # USB-capable Cores; `make flash` for SWD
```

The folder name becomes the project name, so that produces
`my-firmware/build/my-firmware.bin`. Keep the project inside this checkout and
`SDK_ROOT` takes care of itself.

## Supported Cores

| Public name    | MCU            | Core                          | USB DFU |
| -------------- | -------------- | ----------------------------- | ------- |
| `Core.ST.L0.1` | STM32L011E4    | Cortex-M0+, 32 MHz, low power | —       |
| `Core.ST.L4.1` | STM32L422TB    | Cortex-M4F, 80 MHz            | ✓       |
| `Core.ST.L4.2` | STM32L422TB    | Cortex-M4F, 80 MHz, more pads | ✓       |
| `Core.ST.W5`   | STM32WBA55HGF6 | Cortex-M33, 100 MHz, BLE      | —       |
| `Core.ST.H5.1` | STM32H523HE    | Cortex-M33, 250 MHz           | ✓       |

Every Core flashes over SWD. The USB-capable ones also take a runtime DFU
update over a plain USB cable — no probe, no jumpers, no buttons.

## Layout

```
tiles/
├── templates/          # one minimal, buildable starter project per Core
├── examples/           # larger reference projects (USB HID, audio, WASM)
├── sdk/
│   ├── ll/             # register access, per MCU family
│   ├── hal/            # hardware abstraction over LL
│   ├── tal/            # pad → peripheral resolution
│   ├── core/           # the user-facing core_* API
│   └── device/         # linker scripts + startup code
├── drivers/            # tile drivers (tile_*.h/c), _template/ to start one
├── definitions/        # Core + tile JSON — a mirror of the product database
├── manifests/          # generated docs + Studio manifests (do not hand-edit)
└── tools/coregen/      # config.json → board init C
```

## Building on Windows

Build from **Git Bash or MSYS2** — not `cmd.exe` or PowerShell. The Makefile
uses POSIX shell tools throughout, and refuses to run without them rather than
failing halfway through in a confusing way.

```sh
winget install Git.Git                        # provides Git Bash
winget install Python.Python.3.12             # tick "Add python.exe to PATH"
winget install Arm.GnuArmEmbeddedToolchain
```

Then `make doctor` from Git Bash. Three things it can't check for you:

- **Keep paths free of spaces.** Make cannot represent a space in a target
  name, so a project under `C:\Users\First Last\…` cannot be built at all.
  `C:\dev\` is a good home. The build refuses such a path up front.
- **`dfu-util` needs a driver.** Bind **WinUSB** to the DFU device once with
  [Zadig](https://zadig.akeo.ie/), or `dfu-util` reports no device even with a
  Core plugged in and in DFU mode.
- **The 1200-baud DFU touch needs a port name.** Windows COM ports have no
  `/dev` entry to detect, so pass `make flash-dfu SERIAL_PORT=COM5` (with
  `pyserial` installed), or enter DFU by hand with BOOT0.

## Documentation

| Where                                                            | What                                                       |
| ---------------------------------------------------------------- | ---------------------------------------------------------- |
| [bergsonne.io/docs](https://bergsonne.io/docs)                     | Everything, rendered — start here                          |
| [`config-json.md`](config-json.md)                                 | Field-by-field `config.json` authoring reference           |
| [`templates/README.md`](templates/README.md)                       | What each starter project ships with, and why              |
| [`AI.md`](AI.md)                                                   | Architecture and conventions, for humans and coding agents |
| [`docs/`](docs/)                                                   | Driver authoring, `@studio` annotations, brick recovery    |
| `drivers/tile_*.h`                                                 | The authoritative API doc for each tile                    |

The tile pages on the website are generated from the Doxygen comments in those
driver headers, so the headers are the source of truth — when prose and header
disagree, fix the header and regenerate.

## Pull requests and CI

`main` requires three checks: `firmware-ok`, `studio-manifest-ok` and
`twins-ok`. Every PR gets all three. Each workflow first checks which paths
changed and skips its heavy jobs when none of its paths are touched, so a
docs-only PR passes in seconds, while an SDK change builds every Core, the
BLE projects, the manifests and the twins. PRs can auto-merge once the checks
pass (`gh pr merge --auto --merge`); merged branches are deleted.

## Third-party code

The Core.ST.W5 Bluetooth LE support in `sdk/ble/` includes STMicroelectronics
code (the BLE stack and link-layer libraries, application modules and two
utilities) under ST's own license, for use with ST microcontrollers only. See
[THIRD_PARTY.md](THIRD_PARTY.md) for the file list and license texts.
