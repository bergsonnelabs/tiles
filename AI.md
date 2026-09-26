# Cores SDK — AI Assistant Reference

Architecture, APIs, and conventions for the Cores SDK. This file gives any AI coding assistant the context needed to write firmware for Tiletown Core boards.

For operational workflows (driver authoring checklist, commit procedures, CI/CD) see the project's CLAUDE.md.

---

## What This Repo Is

The **Cores SDK** is a firmware development kit for the Tiletown **Core** family of STM32-based tiles. It provides:

- A **HAL** (hardware abstraction layer) over STM32 LL drivers — no CubeIDE, no STM32Cube HAL
- A **code generator** (`coregen`) that turns a declarative `config.json` into initialisation C code
- The **tile driver framework** — platform-agnostic drivers for Tiletown sensor/actuator/power tiles
- A clean **build system** (Make + arm-none-eabi-gcc) targeting four Core MCU families

---

## Supported Cores (MCUs)

| Public name | MCU | Definition stem | Architecture |
|---|---|---|---|
| `Core.ST.L0.1` | STM32L011E4 | `Core-ST-L0-1-a` | Cortex-M0+, 32 MHz max, ultra-low-power |
| `Core.ST.L4.1` | STM32L422TB | `Core-ST-L4-1-b` | Cortex-M4F, 80 MHz (first shipping L4) |
| `Core.ST.L4.2` | STM32L422TB | `Core-ST-L4-2-a` | Cortex-M4F, 80 MHz (more pads) |
| `Core.ST.W5` | STM32WBA55HGF6 | `Core-ST-W5-b` | Cortex-M33, 100 MHz, BLE |
| `Core.ST.H5.1` | STM32H523HE | `Core-ST-H5-1-a` | Cortex-M33, 250 MHz |

Pass either the **public name** or the **definition stem** as `TILE`. The
vendor-segmented public names resolve to their definition stem in the Makefile
(e.g. `Core.ST.L4.1` → `Core-ST-L4-1-b`); the matching alias map lives in
`tools/coregen/coregen.py`.

MCU capabilities (PLL ranges, APB clocks, SPI/I2C peripheral mapping) live in `MCU_DB` inside `tools/coregen/coregen.py`.

---

## Repo Layout

```
cores/
├── Makefile                    # Top-level build orchestrator
├── AI.md                       # ← this file
├── sdk/
│   ├── core/                   # User-facing API (core_ namespace, see Core Layer section)
│   ├── hal/                    # HAL implementations (see HAL Layer section)
│   ├── ll/                     # Low-level register access (wraps CMSIS)
│   ├── tal/                    # Tile abstraction layer (pad→peripheral mapping)
│   │   ├── tal_adc.h           # Pad → ADC channel resolution
│   │   ├── tal_timer.h         # Pad → timer instance/channel resolution
│   │   └── tal_exti.h          # Pad → EXTI with auto-pull
│   ├── cmsis/                  # ARM CMSIS headers
│   ├── device/                 # Linker scripts, startup code (per MCU)
│   └── status/                 # SDK implementation status per Core subfamily
│       ├── features.json       # Canonical feature manifest (groups, IDs, layer, desc)
│       ├── core-l.json         # Core.ST.L0 (STM32L011) feature statuses
│       ├── core-u.json         # Core.ST.L4 (STM32L422) feature statuses
│       ├── core-w.json         # Core.ST.W5 (STM32WBA55) feature statuses
│       └── core-h.json         # Core.ST.H5 (STM32H523) feature statuses
├── tiles.h                     # Tile framework entry point — include this
├── tiles_pal.h                 # Platform abstraction interface
├── definitions/                # Tile JSON definitions (canonical source)
├── drivers/                    # Tile peripheral drivers (tile_*.h/c)
│   └── _template/              # Driver scaffolding (tile_template.{h,c})
├── hal/                        # Tile PAL adapters (Arduino / ESP-IDF / STM32)
├── twins/                      # Tile digital twins (TypeScript), one per tile — see twins/README.md
├── manifests/                  # Generated manifests (per-tile + SDK)
├── tools/
│   └── coregen/
│       ├── coregen.py          # Main generator — entry point
│       ├── templates/          # Jinja2 templates (see below)
│       └── config-schema.json
├── templates/                  # One minimal, buildable starter per Core — copy one
│   ├── core-st-l0-1/           # Makefile + config.json + main.c
│   ├── core-st-l4-1/
│   ├── core-st-l4-2/
│   ├── core-st-w5/
│   └── core-st-h5-1/
├── projects/                   # User projects (each has config.json)
│   └── my-project/
└── examples/                   # Reference projects (no config.json changes needed)
    ├── u2-blink/
    ├── h1-usb-hid/             # USB HID example
    └── ...
```

**Starting a new project:** copy the `templates/` folder for your Core rather
than hand-assembling one. Each is a complete, building project, and the
USB-capable ones (L4.1 / L4.2 / H5.1) come pre-wired in the fleet-standard
shape — `bootloader: "rom"`, USB CDC up before `main()`, and the `iwdg`
watchdog + strike→ROM-DFU brick recovery, fed from the starter loop. See
[`templates/README.md`](templates/README.md).

---

## Build System

### Variables

| Variable | Default | Description |
|---|---|---|
| `TILE` | `Core.ST.L4.2` | Core to build for — public name or definition stem |
| `PROJECT` | `blink` | Project name (looks in `examples/` or `projects/`) |
| `PROJECT_DIR` | `examples/$(PROJECT)` | Override if project lives elsewhere |
| `TILES_ENABLED` | `0` or `1` | Auto-set: 1 if project has tiles configured |
| `BOOTLOADER` | from config.json | `1` = custom DFU bootloader (app at 0x08002000) |
| `ROM_DFU` | from config.json | `1` = ROM DfuSe bootloader (app at 0x08000000) |
| `V` | `0` | Verbosity (1 = show all commands) |
| `PYTHON` | auto-probed | Python 3 for coregen — tries `python3`, `py -3`, `python` |
| `SERIAL_PORT` | auto-detected | CDC port for the 1200-baud touch; required on Windows (`COM5`) |
| `STM32_PROG_CLI` | per-host default | STM32CubeProgrammer CLI path (Core.ST.W5 SWD flashing) |

### Common Commands

```bash
make                                           # Build blink for the default Core (Core.ST.L4.2 / Core-ST-L4-2-a)
make TILE=Core.ST.W5 PROJECT=my-project     # Build specific core + project
make TILE=Core.ST.W5 PROJECT=my-project V=1 # Verbose build
make generate                                  # Run coregen only (no compile)
make flash                                     # Flash via OpenOCD / ST-Link
make flash-dfu                                 # Flash via USB DFU
make clean                                     # Remove build artefacts
make distclean                                 # Remove build + coregen output
make doctor                                    # Check the toolchain, report what's missing
```

### Host Platforms

macOS, Linux, and Windows are all supported; Windows must build from **Git Bash
or MSYS2**, never `cmd.exe` or PowerShell. The Makefile relies on POSIX shell
builtins throughout (`[`, `mkdir -p`, `rm -rf`, `ls`), so it forces
`SHELL := sh.exe` on Windows and hard-errors if no POSIX shell is on PATH —
without that, make silently falls back to `cmd.exe` *per recipe line* and the
build dies in a cascade of `'[' is not recognized`.

Four more host differences the Makefile handles, each with its own trap:

- **Python.** On Windows a bare `python3` is usually the Microsoft Store alias
  stub, which resolves but does nothing. `PYTHON` is probed by actually running
  each candidate (`python3`, `py -3`, `python`), so the stub is rejected rather
  than silently accepted; no working Python 3 is a hard error, not a config-less
  coregen run.
- **Serial port.** The 1200-baud DFU touch needs a device node:
  `/dev/tty.usbmodem*` + `stty -f` on macOS, `/dev/ttyACM*` + `stty -F` on
  Linux. Windows COM ports have no `/dev` entry, so `make flash-dfu` there
  either takes `SERIAL_PORT=COM5` (touch via pyserial) or expects the board to
  be put into DFU by hand — and `dfu-util` needs WinUSB bound with Zadig first.
- **Console encoding.** Windows Python encodes stdout with the legacy ANSI code
  page (cp1252) whenever stdout is not a real console — under make it is a pipe,
  so that is always. One non-ASCII character in a tool's output (coregen's `→`,
  validate's `✓`, the `µ` and `—` all over driver briefs) raises
  `UnicodeEncodeError` and takes the build down. **Every `tools/*.py` entry
  point forces UTF-8 on its own streams** in a short block after its imports —
  copy it into any new tool — and the Makefile exports `PYTHONIOENCODING=utf-8`
  to cover the inline `python -c` calls. Reproduce the failure on any host with
  `PYTHONIOENCODING=cp1252 make …`.
- **Paths with spaces.** Make cannot represent a space in a target or
  prerequisite name, so a project under `C:/Users/First Last/…` cannot be built
  at all. Both `SDK_DIR` and `PROJECT_DIR` are checked up front and rejected
  with the remedy, instead of failing later as `No rule to make target
  'Last/main.c'`.

### Build Flow

1. **coregen** runs → produces `project/coregen/` with generated `.h/.c/.mk` files
2. **compiler** builds project `main.c` + SDK HAL sources + `core_init.c` + tile drivers
3. **linker** uses MCU-specific linker script from `sdk/device/`

### coregen — Code Generator

**Entry point:** `tools/coregen/coregen.py`
**Called by:** `Makefile` during build (or `make generate`)

#### What It Generates (`project/coregen/`)

| File | Always? | Contents |
|---|---|---|
| `core_pads.h` | yes | `PAD_n_PORT` / `PAD_n_PIN` macros for every pad |
| `core_board.h` | yes | Board-level defines (LED pad, power rails) |
| `core_interfaces.h` | yes | AF constants per signal (e.g. `I2C1_CLK_AF`) |
| `core_config.h` | project only | `SYSCLK_MHZ`, `PLL_M/N/R`, assigned pad functions |
| `core_init.h` | project only | Init function declarations, extern bus handles |
| `core_init.c` | project only | `core_clock_init()`, `core_pads_init()`, I2C/SPI/UART/tile init |
| `core.h` | project only | Master include (includes all of the above) |
| `tile_handles.h` | project+tiles | Tile handle variables; preserves edits outside markers |
| `core_drivers.mk` | project+tiles | Makefile fragment listing tile driver source files |

#### Key Internal Functions

The `build_*_config()` functions in `coregen.py` each parse one section of `config.json` and return a config dict consumed by Jinja2 templates: `build_pad_map/config` (GPIO port/pin/AF), `build_clock_config` (PLL solver, HSE enforcement), `build_i2c/spi/uart/i3c/timer_config` (peripheral setup), `build_tiles_config` (driver paths + bus handles), and `validate_project_config` (cross-section validation).

#### Editing coregen

- `MCU_DB` — add new MCUs (define, family, PLL ranges, APB clock enables)
- `SPI_CLK_MAP` / `SPI_PRESCALER_MAP` — SPI clock enable + prescaler mappings
- Templates are Jinja2 in `tools/coregen/templates/`; context comes from the `ctx` dict in `generate()`

---

## config.json Format

Every project in `projects/<name>/` has a `config.json`. It is the single source of truth for hardware configuration. coregen derives all generated code from it. The filename is literal — coregen looks for `config.json` in the project directory; the project's human name comes from the directory name itself.

**→ Full authoring reference: [`config-json.md`](config-json.md).** It documents every key coregen actually reads (verified against `tools/coregen/coregen.py`), the per-MCU constraints, the validation/error behaviour, a cookbook, and — importantly — the keys that are **silently ignored**. Read it before hand-editing a config.

Minimal shape:

```jsonc
{
  "core": "Core.ST.W5",        // Public name (or a definitions/<name>.json stem)
  "clock": "max",               // a clock-level NAME from the tile JSON, not a frequency
  "pads": {                     // pad number (string) → function the tile JSON allows
    "9":  "I2C1.CLK",
    "11": "I2C1.DAT",
    "8":  "SPI1.CS",            // SPI CS pads → GPIO output (software CS)
    "4":  "GPIO.OUT"
  },
  "interfaces": {               // TUNING ONLY — never creates a bus; a pad function does
    "I2C1": { "speed": 400000, "pullups": true },
    "SPI1": { "mode": 0, "prescaler": 8, "cs_polarity": "active-low" }
  },
  "gpio": {                     // per-pin pull / output_type / exti / default
    "5": { "pull": "up", "exti": "falling" }
  },
  "tiles": [
    { "tile": "Sense.I.9", "bus": "I2C1", "instance": 0 }   // SPI tiles also need "cs_pad"
  ],
  "bootloader": "custom",       // "custom" | "rom" | "none" (default)
  "usb": { "enabled": false, "vid": "0x1209", "pid": "0xDA01", "product": "…" }
}
```

**Key things that bite (see config-json.md for the rest):**
- Buses/PWM/ADC are **derived from `pads`**, not declared. `interfaces.X` only tunes a bus some pad already created; a freq for a timer goes under `interfaces.TIM<n>.freq`.
- Most mistakes (bad pad/function, unknown clock level, illegal I2C speed, bad SPI mode/prescaler, unknown tile, missing SPI `cs_pad`, bad bootloader) make coregen **exit** — it's fail-fast.
- coregen reads a **fixed allowlist** of keys; unknown/typo'd keys are silently ignored. In particular `ble`, `debug`, `isp`, `programming`, and any `timers`/`pwm`/`capture`/`iwdg` sections are **NOT read by coregen** and do nothing. (For the WBA radio, get HSE by picking an HSE-sourced `clock` level — there is no `ble` switch.)
- I3C interfaces are accepted but unimplemented — coregen emits a `/* I3C1: TODO */` comment, no build error. See `sdk/hal/hal_i3c.h`.

---

## Core Layer (`sdk/core/`) — User-Facing API

The `core_` layer provides platform-agnostic naming for application code. It is a thin alias over the HAL: each `core_` function delegates to the corresponding `hal_` function, and `core_*_t` handle types are typedefs of `hal_*_t`. This lets user code remain stable even if the underlying HAL implementation changes, and keeps example code readable without exposing STM32-specific details.

**Key headers:**

| Header | Purpose |
|---|---|
| `core_timer.h` | `core_timer_init_freq()` (PWM/tick), `core_timer_init_tick()` (capture), `core_timer_pwm_set()` (0-100%), `core_timer_capture_init/read()`, `core_timer_enable_tick()` |
| `core_gpio.h` | Single include for `core_pad_output/input/read/write/toggle()` + `core_pad_on_change()` (EXTI) |
| `core_adc.h` | `core_adc_init()`, `core_adc_read()`, `core_adc_read_mv()`, DMA, temp sensor |
| `core_i2c.h` | `core_i2c_setup()` with auto-timing, `core_i2c_write/read/probe/scan()` |
| `core_watchdog.h` | `core_watchdog_start(ms)`, `core_watchdog_feed()`, `core_watchdog_caused_reset()`, `core_watchdog_running()`, `core_watchdog_sleep_chunk_ms()`. Timeout recorded in `core_watchdog.c` (compiled per project like `core_led.c`); `core_power.h` sleeps in fed chunks around it and `core_standby_for()` returns `HAL_ERROR` past half the timeout. On ROM-DFU builds `feed()` zeroes the brick-recovery strike count once after max(10 s, 2x timeout) of uptime. |
| `core_nvm.h` | Persistent byte store, same API on every Core: `core_nvm_read/write(offset, buf, len)` (0 or negative `CORE_NVM_ERR_*`), `core_nvm_erase_all()`, `core_nvm_size()`, DSL `core_nvm_read_byte/write_byte()`. L0: 512 B data EEPROM. L4 / W5: 1 KB emulated in two flash pages (L4 0x0801F000, W5 0x080FA000; W5 page 127 above it is the BLE bond store): an append log with CRC'd records, compacted to the other page when full, power-loss safe, survives flash-serial / DFU / SWD reflashing (the linker scripts end the app below the pages, `__core_nvm_start`). A compaction erases a page: ~22 ms CPU stall on the L4. W5 with BLE schedules erases/writes through ST's flash manager (`sdk/ble/ble_flash.c`). H5: none yet (size 0). Backed by `core_nvm.c` (compiled per project; it also owns `NMI_Handler` on L4/W5 to survive ECC errors from torn writes). |
| `core_usb.h` | `core_usb_init()`, `core_usb_write()`, `core_usb_printf`, includes `<stdio.h>` |
| `core_timing.h` | `core_delay_ms/us()`, `core_millis()`, `core_timeout()` |
| `core_scope.h` | Scope: live variable streaming to Studio or any host. `core_scope_watch(var)` / `core_scope_watch_array(arr, n)` register variables (type + name via `_Generic`); `core_scope_update()` in the loop snapshots and sends. ISR path: `core_scope_declare_rate_hz()`, `core_scope_sample()` (ISR-safe, copy only) + `core_scope_pump()` (main loop). Never blocks; drops whole samples and tells the host how many. Link chosen per Core: USB CDC (shares the port with print text, via `core_usb_try_write`), BLE notifications on the Studio Link service (`0x5C00/0x5C01`, registered through `core_ble_add_services()` — first scope call must precede `core_ble_init()`), or any `core_scope_link_t`. `"scope": {"enabled": false}` in config.json compiles every call out. Wire format: `docs/scope-protocol.md`; reference decoder `tools/scope_decode.py`; host test `tools/test_core_scope.py`. Backed by `core_scope.c` (compiled per project; gc'd when unused). |
| `core_stepper.h` | STEP/DIR pulse generator for external stepper drivers: `core_stepper_init(&s, step_pad, dir_pad)` (STEP pad declared as `TIMx.y` in config.json; the pulse comes from that timer channel, one ISR per step), `set_speed/accel/min_speed()`, `move/move_to/run/stop/halt()`, `busy/position()`. Trapezoidal ramps, 1 us resolution. Backed by `core_stepper.c` (compiled per project like `core_led.c`). |

**Timer init has two variants:**
- `core_timer_init_freq(&t, TIM2, 1000)` — overflow at 1 kHz (for PWM, periodic tick)
- `core_timer_init_tick(&t, TIM2, 1000000)` — tick at 1 MHz, free-running (for input capture)

**PWM duty is 0-100 integer percent** at the core_ layer. For 0.1% resolution, use `hal_timer_pwm_set_duty()` with 0-1000.

**`core.h`** (generated by coregen) includes `core_gpio.h` and `core_timing.h` automatically, so delays and GPIO are always available.

---

## HAL Layer (`sdk/hal/`)

Design principles:
- Wraps STM32 LL (low-level) drivers — **never** STM32Cube HAL
- Opaque handle structs (`hal_i2c_t`, `hal_spi_t`, etc.) initialised with a config struct
- Single-threaded polling API; some DMA paths in `hal_spi`
- No STM32-specific types in public `.h` files

### Key HAL APIs

**I2C** (`hal_i2c.h`):
```c
hal_status_t hal_i2c_init(hal_i2c_t *h, uint32_t instance, hal_i2c_config_t *cfg);
hal_status_t hal_i2c_write_reg(hal_i2c_t *h, uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len);
hal_status_t hal_i2c_read_reg(hal_i2c_t *h, uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len);
hal_status_t hal_i2c_is_ready(hal_i2c_t *h, uint8_t addr);
```

**SPI** (`hal_spi.h`):
```c
hal_status_t hal_spi_init(hal_spi_t *h, uint32_t instance, hal_spi_config_t *cfg);
hal_status_t hal_spi_set_cs(hal_spi_t *h, GPIO_TypeDef *port, uint32_t pin);
hal_status_t hal_spi_write(hal_spi_t *h, uint8_t *data, uint16_t len);
hal_status_t hal_spi_read(hal_spi_t *h, uint8_t *data, uint16_t len);
// h->cs_active_low = 0  →  active-high CS
```

**UART** (`hal_uart.h`):
```c
// NOTE: instance is USART_TypeDef* (e.g. USART2), not a bus number.
// pclk_hz = SYSCLK_HZ (all APB dividers are 1 in coregen output).
// hal_uart_init() enables its own peripheral clock internally.
hal_status_t hal_uart_init(hal_uart_t *h, USART_TypeDef *instance,
                           uint32_t pclk_hz, const hal_uart_config_t *cfg);
void         hal_uart_tx(hal_uart_t *h, const uint8_t *data, uint32_t len);
void         hal_uart_tx_str(hal_uart_t *h, const char *str);
int          hal_uart_printf(hal_uart_t *h, const char *fmt, ...);
int          hal_uart_rx_ready(hal_uart_t *h);
uint8_t      hal_uart_rx(hal_uart_t *h);         // blocking
int          hal_uart_rx_try(hal_uart_t *h, uint8_t *byte);  // non-blocking
```

**I3C** (`hal_i3c.h` — stub, not yet implemented):
```c
hal_status_t hal_i3c_init(hal_i3c_t *h, uint32_t instance_id,
                           const hal_i3c_config_t *cfg);  // returns HAL_ERROR
hal_status_t hal_i3c_write(hal_i3c_t *h, uint8_t addr,
                            const uint8_t *data, uint16_t len);
hal_status_t hal_i3c_read(hal_i3c_t *h, uint8_t addr,
                           uint8_t *data, uint16_t len);
```

**ADC** (`hal_adc.h` — Tier 1, all four Core families):
```c
// Initialise: enables clock, calibrates, enables ADC
hal_status_t hal_adc_init(hal_adc_t *adc, ADC_TypeDef *instance,
                          uint32_t sysclk_hz, hal_adc_res_t res);

// Add channels before reading (per-channel sampling time)
hal_status_t hal_adc_add_channel(hal_adc_t *adc, uint8_t channel, hal_adc_samp_t samp);

// Oversampling (applies to all channels; can be called after init)
hal_status_t hal_adc_set_oversample(hal_adc_t *adc, hal_adc_oversample_t ratio);

// Single-shot blocking reads
uint16_t hal_adc_read(hal_adc_t *adc, uint8_t channel);        // raw count
uint32_t hal_adc_read_mv(hal_adc_t *adc, uint8_t channel);     // millivolts (VREFINT-calibrated)

// Internal measurements
int32_t  hal_adc_read_temp_decidegc(hal_adc_t *adc);           // die temp x10 (e.g. 253 = 25.3C)
uint32_t hal_adc_read_vdda_mv(hal_adc_t *adc);                 // actual VDD supply

// Read all configured channels in one call
void hal_adc_read_all(hal_adc_t *adc, uint16_t *buf);

// DMA circular buffer (HT + TC callback)
hal_status_t hal_adc_start_dma(hal_adc_t *adc, uint16_t *buf, uint16_t len,
                                void (*callback)(void));
void hal_adc_stop_dma(hal_adc_t *adc);
```

Enums: `HAL_ADC_RES_6/8/10/12BIT` (all families, plus `14BIT` on H5). Sampling: `HAL_ADC_SAMP_FAST/MED/SLOW/VERY_SLOW`. Oversampling: `HAL_ADC_OVERSAMPLE_1X/4X/16X/64X/256X` (plus `1024X` on H5).

`hal_adc_read_mv()` uses VREFINT factory calibration to derive true VDDA, so millivolt results are correct even when VDD differs from 3.3 V. VDDA is cached in `adc->vdda_mv` after first measurement.

**Per-family internal channel numbers:**

| Family     | VREFINT ch | TEMP ch | CAL voltage |
|------------|-----------|---------|-------------|
| L0 (L011)  | 17        | 16      | 3.0 V       |
| L4 (L422)  | 0         | 17      | 3.0 V       |
| WBA (WBA55)| 13        | 12      | 3.3 V       |
| H5 (H523)  | 19        | 16      | 3.3 V       |

`hal_adc_start_dma` uses DMA1 CH1 (L0/L4) or GPDMA1 CH0 (WBA/H5) in circular mode with HT+TC interrupts for double-buffered processing.

---

## Tile Driver Framework

### Framework Headers

**`tiles.h`** — include in all tile drivers and user code when `TILES_ENABLED=1`:
- Defines `tile_t` handle: `{ tiles_pal_t* hal, uint8_t id, tile_state_t state, ... }`
- States: `TILE_STATE_NONE -> TILE_STATE_FOUND -> TILE_STATE_READY` (or `TILE_STATE_ERROR`)
- `TILES_CHECK_VERSION(major, minor)` — compile-time SDK version assertion

**`tiles_pal.h`** — platform abstraction handle (`tiles_pal_t`):
- Function pointers for I2C, SPI, QSPI, delay, error callback
- Bus type flags: `TILES_BUS_I2C`, `TILES_BUS_SPI`, `TILES_BUS_QSPI`

**`sdk/core/core_tiles.h`** — Cores SDK bridge (wires bus handles to `tiles_pal_t`):
```c
// Single function for both I2C and SPI (C11 _Generic dispatch):
tiles_pal_t *hal = core_tiles_pal(&core_i2c1);   // I2C bus
tiles_pal_t *hal = core_tiles_pal(&core_spi1);   // SPI bus
```

### Tile Driver Conventions

Drivers live in `drivers/tile_<family>_<name>.h/c`:

```c
// Minimum viable tile driver
#include "tiles.h"

TILES_CHECK_VERSION(1, 0);

// Find: scan bus for the device, set tile state
uint8_t tile_sense_i_9_find(tiles_pal_t *hal, uint8_t instance);

// Init: configure registers, mark TILE_STATE_READY
void tile_sense_i_9_init(tiles_pal_t *hal, uint8_t instance, tile_t *tile);

// Data access functions (after init)
void tile_sense_i_9_get_raw_accels(tile_t *tile, int16_t accel[3]);
```

- `instance` selects the I2C address variant (0 = primary address, 1 = alternate, etc.)
- For SPI tiles, `instance` is per-CS-line (each CS = one device = one instance)
- Drivers access hardware only through `tiles_pal_t` function pointers — never directly

### Vibe Settings (`@studio control` / `@studio require`)

Studio's Vibe interface lets someone who has never read a datasheet change a tile's
settings, from the tile inspector and from the plain-language story. **The driver
header is the only place those settings are defined.** Nothing is hand-curated in
Studio, and nothing is a second API: a setting is always an argument of a function
that is already `@studio expose`d. Reference driver: `drivers/tile_sense_i_6p6.h`.

```c
/**
 * @brief  Set accelerometer output data rate.
 * @studio expose category=tile name=set_accel_odr section=runtime
 * @studio control odr label="Accel data rate" tier=basic default=SENSE_I_6P6_ODR_100HZ role=sample_rate
 * @studio require expr="value(odr) >= 12.5" when="set_power_mode.accel == SENSE_I_6P6_MODE_LN" message="Below 12.5 Hz the accelerometer only runs in low-power mode. ..."
 */
```

`@studio control <arg> label="…" tier=basic|advanced …`

| Attribute | Meaning |
|---|---|
| `label` | What a person calls it. Unique per tile. |
| `tier` | `basic` = shown by default and always surfaced in the story; `advanced` = behind the inspector's Advanced toggle. |
| `default` | The value in force when the program never sets it: the driver's init value, else the chip's reset value **from the datasheet**. An enum member name or an integer. **Required** for `scope=config`. |
| `scope` | `config` (default): a pure setting, so Studio may write the call itself. `usage`: an argument of something the program *does* (a wait timeout, a detection threshold) and only a setting once the program makes that call. |
| `type=bool` | A 0/1 argument shown as on/off. |
| `allow=A,B,…` | The enum members THIS argument may take, when an enum type is shared between arguments or gives one register value two names. |
| `role=sample_rate` | This setting is the rate at which the part produces data. Studio never lets a program read or stream faster than it. The rate is the selected enum member's `@studio value=` (in Hz), so every offered member needs one, unless `rate=` is given. |
| `rate="<expr>"` | With `role=sample_rate`: the rate in Hz as an expression, for arguments that are not a rate enum. A period: `rate="1000 / period_ms"`. A divider: `rate="1125 / (1 + divider)"`. Required when the argument is numeric. |
| `scale=<n> unit=<u>` | The argument is stored in sub-units: people read and type `value × scale`, in `unit` (`scale=0.1 unit=dB` for a threshold in tenths of a dB). Invertible, so it drives the input as well as the display; numeric arguments only, and not together with `show=`. The `[min..max]` stays in raw units. |
| `when="<expr>"` | The setting only applies while this holds (a filter on a sensor that is switched off, a range on a powered-down axis). Studio greys it out in the inspector, and leaves it and the part that gates it out of the plain-language story. |
| `show="<expr>" unit=Hz` | What the value *means*, computed for the reader (a filter setting shown as its bandwidth in Hz rather than "ODR/16"). |

Numeric settings take their bounds from `@param <arg> [min..max] unit`, which is
required. Enum settings take their options and labels from the enum's member doc
comments; add `@studio value=<number>` to a member's comment to give it a physical
quantity (`/**< 100 Hz @studio value=100 */`).

`@studio require expr="…" [when="…"] message="…"` states a relation between settings.
`when` makes it conditional. `message` is shown to the user verbatim when a change
would break the rule, so write it as advice: say what to change first.

**Expressions are a strict language, not interpreted strings** (`tools/studio_expr.py`).
The generator parses them into a JSON AST, resolves every name, and fails the build on
anything it cannot resolve. Studio only ever evaluates the AST.

- Operands: numbers, enum member names, `arg` (an argument of this function),
  `function.arg` (another exposed function on this tile).
- A bare reference is the raw register value; `value(ref)` is the selected member's
  `@studio value=` quantity.
- Operators: `+ - * /`, `< <= > >= == !=`, `&& || !`, `a ? b : c`, parentheses,
  `min(...)`, `max(...)`.

The generator refuses to build (exit 1, `error: vibe settings: …`) when: a name or
member does not resolve, a `scope=config` control has no default, a numeric control
has no `[min..max]`, a default is not an offered member, two offered members share a
register value, or two settings share a label. Treat these as compile errors.

Read the datasheet for every default and every rule (`~/Documents/local/tile references/<Stem>/`).
The 6P6 sweep found the header's own filter-bandwidth comments were wrong
(`max(400 Hz, ODR)/N`, not `ODR/N`).

The same annotations feed three places, all generated: `manifests/tile_<driver>.json`
(Studio), `manifests/tile-docs/<driver>.json` (`vibe_settings` + `vibe_rules`, the
"Vibe Settings" table on the docs site), and Studio's assistant briefing.

### Using Tile Drivers in User Code

Generated `core_init.c` initialises clocks, pads, and bus peripherals. User code includes `core_tiles.h` for the bridge and calls `core_tiles_pal()` to get a `tiles_pal_t*`:

```c
#include "core.h"           // Generated: clocks, pads, bus handles
#include "core_tiles.h"     // core_tiles_pal() — I2C and SPI
#include "tile_sense_i_9.h"

int main(void) {
    core_init();

    tile_t imu;
    tiles_pal_t *hal = core_tiles_pal(&core_i2c1);
    tile_sense_i_9_init(hal, 0, &imu);

    while (1) {
        int16_t accel[3];
        tile_sense_i_9_get_raw_accels(&imu, accel);
    }
}
```

---

## Design Rules

1. **No CubeIDE.** Never generate or use STM32CubeMX output. Write clean LL-based init from scratch.
2. **No STM32Cube HAL.** Use only `sdk/ll/` and `sdk/hal/`. The LL layer wraps CMSIS register access.
3. **Platform-agnostic tile drivers.** Drivers must compile on Arduino/ESP-IDF/Zephyr. No STM32 types in `tile_*.h`.
4. **config.json is the source of truth.** Never hand-edit generated files inside `coregen:begin/coregen:end` markers.
5. **CS lines are per-tile, not per-bus.** Each tile on an SPI bus gets its own GPIO CS line.
6. **`definitions/` is canonical.** Tile JSON lives there (synced from GitHub). Do not duplicate.
7. **`sdk/status/` is the source of truth for SDK implementation status.** Update `features.json` when adding a feature row, and the relevant `core-*.json` when implementation status changes.

---

## Common Tasks

### Add a new tile driver

#### Phase 1 — Research

- [ ] Read IC datasheets: register map, I2C/SPI protocol, power-on defaults, device ID register
- [ ] Check `definitions/` for the tile JSON (already exists from the DB?)
- [ ] Check `TILE_DRIVER_MAP` in `tools/coregen/coregen.py` (entry exists?)
- [ ] Choose a reference driver to follow:
  - **Sensor (I2C, simple):** `tile_sense_mic.h/c` — config struct, calibration, data reads
  - **Sensor (I2C, full):** `tile_sense_bp.h/c` — ODR/AVG/FS config, FIFO, one-shot, threshold interrupts, autozero/autorefp, offset calibration
  - **Sensor (I2C+SPI):** `tile_sense_i_6p6.h/c` — dual-bus dispatch, interrupt callbacks
  - **Actuator:** `tile_drive_p.h/c` — modes, FIFO, status monitoring
  - **Audio/multi-IC:** `tile_drive_a_2.h/c` — DAC+amp, safe startup sequence

#### Phase 2 — Driver implementation

Create `drivers/tile_<family>_<name>.h` and `.c`. All code must be platform-agnostic — no STM32 types, no direct register access. All bus I/O goes through `tile->hal` function pointers.

**Header (.h) structure:**

```
File-level Doxygen (brief, specs, quick-start @code example, datasheet links)
Include guard + #include "tiles.h"
Version macros: TILE_<FAMILY>_<NAME>_VERSION_{MAJOR,MINOR,PATCH}
TILES_CHECK_VERSION(1, 0)
Instance mapping table (Doxygen comment showing instance → address)
IC address defines (#define <IC>_I2C_ADDR_DEFAULT ...)
Register map defines (#define <IC>_REG_... grouped by function)
Enums (gain, mode, waveform, etc. — values map to hardware register fields)
Config structs (optional init config, AGC config, etc.)
Public API declarations with full Doxygen (@brief, @param, @return, @note)
```

**Implementation (.c) structure:**
```
id_table[] — maps instance index to I2C address (or CS index for SPI)
resolve_id(instance) — bounds-checked lookup, returns 0 for invalid
Per-instance state struct (static, private — cached config, addresses)
state_for(tile_t *) — lookup helper matching tile->id to state slot
Bus helpers — bus-aware read/write:
  For I2C+SPI: check tile->hal->buses & TILES_BUS_SPI to dispatch
  For amp/secondary ICs: use address from state, not tile->id
Lifecycle: find(), init(), sleep(), wake(), reset()
Data/control functions
```

**Init sequence pattern:**

1. Zero tile struct, set `tile->hal` and `tile->id` (resolve from instance)
2. Probe device (`i2c_is_ready` or SPI test read)
3. Read device ID register, verify expected value
4. Configure device (power up outputs, set defaults)
5. **For tiles with amplifiers/power stages:** settle outputs at safe level (e.g., mid-scale) before enabling the power stage. Put amp in shutdown -> configure -> wake. This prevents AGC gain runaway from DC offset transients.
6. Cache configuration in per-instance state
7. Set `tile->state = TILE_STATE_READY` (or `TILE_STATE_ERROR` + `TILE_ON_ERROR()`)

**Do NOT software-reset the device** unless necessary — resets can wipe NVM-stored analog configuration that the tile hardware depends on.

### Add a new HAL peripheral

1. Create `sdk/hal/hal_<name>.h` and `sdk/hal/hal_<name>.c`
2. Follow the opaque-handle + config-struct pattern from `hal_i2c.h`
3. Add to `SOURCES` in the Makefile if always needed, or conditionally

### Add a new Core (MCU variant)

1. Add entry to `MCU_DB` in `tools/coregen/coregen.py`
2. Add linker script + startup to `sdk/device/`
3. Add `SPI_CLK_MAP` entries for the new MCU's APB assignments
4. Add the tile JSON to `definitions/`
5. Update `TILE` -> MCU mapping in `Makefile`
6. Add a `sdk/status/core-<x>.json` with the new Core's feature statuses

### SDK implementation status

Status files live in `sdk/status/`. `features.json` is the canonical feature manifest; `core-*.json` files hold per-subfamily statuses.

**Status values:**
- `"verified"` — hardware-tested and confirmed working
- `"compile"` — builds cleanly, not yet hardware-validated
- `"partial"` — partially working; a `note` field explains the limitation
- `"wip"` — actively in development
- `"none"` — not yet implemented (omitting a key is equivalent)
- `"na"` — not applicable for this Core (hide from display)

**`features.json` entry format:**
```json
{ "id": "uart.polling", "layer": "LL", "name": "TX / RX (polling)" }
{ "id": "system.tick",  "layer": "LL", "name": "SysTick", "desc": "Tooltip shown in the web table." }
```
Layer values: `"LL"`, `"HAL"`, `"TAL"`, `"Core"` — implies all layers below are also available.

**`core-*.json` entry format:**
```json
{
  "id": "Core.ST.W5",
  "mcu": "STM32WBA55",
  "freq": "100 MHz",
  "arch": "M33",
  "bootloaders": ["uart", "i2c", "spi"],
  "features": {
    "uart.polling": "verified",
    "adc.mv": { "status": "partial", "note": "VREFINT bypassed on WBA55; hardcoded 3300 mV" },
    "conn.usb_cdc": "na"
  }
}
```

### Debug a coregen issue

```bash
make generate TILE=Core.ST.W5 PROJECT=my-project V=1
# Check projects/my-project/coregen/ for generated files
# coregen prints validation errors to stderr
```
