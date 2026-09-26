# `config.json` — Authoring Reference

How to hand-write and edit the per-project `config.json` that `coregen` turns
into initialisation C code. This is the **single source of truth** for a
project's hardware setup; every generated file is derived from it.

> Authoritative against `tools/coregen/coregen.py`. Where this doc and a
> code comment disagree, the code wins — please fix this doc.

---

## 1. The mental model

```
definitions/<Core>.json   config.json (you write this)
        │                        │
        └──────────┬─────────────┘
                   ▼
         tools/coregen/coregen.py
                   ▼
   generated C (core_pads.h, core_init.c, core_config.h,
                core.h, tile_handles.h, …)  + Makefile (first run only)
```

- **The tile/core JSON (`definitions/<Core>.json`) is the menu.** It lists
  which pads exist, what each pad can do (its functions and alternate-function
  numbers), the available clock levels, and the on-board LED. coregen validates
  your `config.json` against it. You cannot assign a function to a pad that the
  core JSON doesn't offer.
- **`config.json` is your order.** You pick the core, a clock level, which pads
  do what, and per-bus settings.
- **coregen does the wiring.** It resolves AF numbers, solves the PLL, computes
  I2C timing, enables the right clocks, and emits handles.

**The single most important idea: most things are *derived*, not *declared*.**
You almost never name a peripheral directly. You assign pad functions, and
coregen *infers* the peripherals from them:

| You write in `pads` …          | coregen infers …                          |
|--------------------------------|-------------------------------------------|
| `"I2C1.CLK"` + `"I2C1.DAT"`    | an I2C1 bus (look in `interfaces.I2C1` for speed) |
| `"SPI1.CLK"` + `"SPI1.MOSI"` … | an SPI1 bus (look in `interfaces.SPI1` for mode)  |
| `"USART2.TX"` + `"USART2.RX"`  | a USART2 UART (look in `interfaces.USART2` for baud) |
| `"TIM2.1"`                     | a PWM/timer channel on TIM2 (freq from `interfaces.TIM2`) |
| `"ADC5"` / `"ADC1"` / `"ADC_IN3"` | an ADC input channel                   |
| `"DAC1.OUT"`                   | the DAC                                   |

The `interfaces` section never *creates* a bus — it only *tunes* a bus that
some pad already brought into existence. If you put `"I2C1": {...}` in
`interfaces` but no pad is set to an `I2C1.*` function, you get a warning and
nothing is generated.

---

## 2. Quickstart

Smallest valid file (LED blink, no peripherals):

```json
{
  "core": "Core.ST.L4.2",
  "clock": "max",
  "pads": {}
}
```

A real project — one I2C tile on a USB-powered L4:

```json
{
  "core": "Core.ST.L4.2",
  "clock": "max",
  "usb": { "enabled": true },
  "pads": {
    "4": "I2C1.CLK",
    "5": "I2C1.DAT"
  },
  "interfaces": {
    "I2C1": { "speed": 400000 }
  },
  "tiles": [
    { "tile": "Sense.I.6P6", "bus": "I2C1", "instance": 0 }
  ]
}
```

After editing, regenerate (from the project dir, or `make generate`):

```
python3 tools/coregen/coregen.py definitions/<Core>.json <project_dir> --config <project_dir>/config.json
```

**Never hand-edit generated files between `coregen:begin`/`coregen:end`
markers** — re-running coregen will clobber them. Edit `config.json` instead.

---

## 3. Top-level keys — at a glance

| Key            | Type   | Required | Default        | Read by coregen? |
|----------------|--------|----------|----------------|------------------|
| `core`         | string | **yes**  | —              | ✅ |
| `clock`        | string | no       | tile's default | ✅ |
| `pads`         | object | no       | `{}`           | ✅ |
| `interfaces`   | object | no       | `{}`           | ✅ (tuning only) |
| `gpio`         | object | no       | `{}`           | ✅ |
| `pullups`      | array  | no       | `[]`           | ✅ (on-tile pull-ups, §7) |
| `tiles`        | array  | no       | `[]`           | ✅ |
| `usb`          | object | no       | disabled       | ✅ |
| `bootloader`   | string | no       | `"none"`       | ✅ |
| `iwdg`         | object | no       | disabled       | ✅ |
| `timer`        | object | no       | none           | ✅ (Studio only) |
| `pins`         | object | no       | —              | ✅ (legacy alias for `pads`) |
| `ble`          | object | no       | —              | ✅ (radio + GATT contract) |
| `scope`        | object | no       | enabled        | ❌ (read by the Makefile, see §10) |
| `debug`        | object | no       | —              | ❌ **ignored** |
| `isp`          | object | no       | —              | ❌ **ignored** |
| `programming`  | object | no       | —              | ❌ **ignored** |
| `probe`        | object | no       | self-powered   | ❌ (read by the flash tooling, see §10.5) |

> Keys not in this table are silently ignored — coregen reads a fixed allowlist
> and never sees anything else. A typo'd key name is **not** an error; it just
> does nothing. Double-check spelling.

---

## 4. `core` — pick the MCU/core *(required)*

```json
"core": "Core.ST.L4.2"
```

Selects `definitions/<Core>.json`, which fixes the MCU part, pad map, clock
options, and on-board peripherals. Accepts either form:

| Public name    | DB stem (also accepted) | MCU            | Max SYSCLK | USB? |
|----------------|-------------------------|----------------|------------|------|
| `Core.ST.L0`   | `Core-ST-L0-a`          | STM32L011      | 32 MHz     | no   |
| `Core.ST.L4`   | `Core-ST-L4-b`          | STM32L422      | 80 MHz     | yes  |
| `Core.ST.L4.2` | `Core-ST-L4-2-a`        | STM32L422      | 80 MHz     | yes  |
| `Core.ST.W5`   | `Core-ST-W5-b`          | STM32WBA55     | 100 MHz    | no   |
| `Core.ST.H5`   | `Core-ST-H5-a`          | STM32H523      | 250 MHz    | yes  |

Prefer the public name. (`Core.ST.L4` resolves to rev **b** — the PA13/PA14
superset of rev a.) If the name doesn't resolve cleanly to the tile JSON you
pass, coregen prints a NOTE but continues.

**`Core.ST.L4` is not `Core.ST.L4.2`.** They are two different boards on the
same STM32L422, with different pad maps. Pick the name printed on the board.

**Renamed 2026-09.** The Cores were renamed to match their silkscreens:
`Core.ST.L4.1` is now `Core.ST.L4`, `Core.ST.L0.1` is now `Core.ST.L0`, and
`Core.ST.H5.1` is now `Core.ST.H5` (stems `Core-ST-L4-1-a/b`, `Core-ST-L0-1-a`,
`Core-ST-H5-1-a` lost their `-1` the same way). The old names still work, in
`"core"` and as `TILE`, and print a one-line NOTE, e.g.
`NOTE: config.json "core": "Core.ST.L4.1" is now "Core.ST.L4" (the old name
still works; update config.json)`. `Core.ST.L4.2` and `Core.ST.W5` did not
change.

---

## 5. `clock` — performance level

```json
"clock": "max"
```

A *name* of a clock level defined in the core JSON (`config.clock`), **not** a
frequency. Typical levels: `"low"`, `"medium"`, `"high"`, `"max"`. The special
value `"default"` resolves to the tile's schema default.

- If omitted, the tile's default level is used.
- If you name a level that doesn't exist, coregen lists the valid options and
  **exits with an error**.
- coregen auto-solves the PLL (M/N/R) when the target frequency differs from the
  source, and sets the voltage range and flash wait states the reference manual
  requires for the level (L0: range 3 for `low`, 2 for `medium`, 1 for
  `high`/`max`; WBA55: range 2 for `low` (HSI16), range 1 above 16 MHz; H5: the
  right VOS for higher speeds).
- Per-Core adjustments, printed as a build NOTE or ERROR:
  - **Core.ST.L4 and Core.ST.L4.2:** `low` runs at 16 MHz, not 8 MHz. USB is always on and
    needs an APB clock of at least 10 MHz (RM0394 §46.4), so `low` and
    `medium` are the same clock there.
  - **Core.ST.L0:** `low` runs in voltage range 3 (no Low-power run: that needs
    <= 131 kHz), or range 2 when the project has an I2C bus, whose HSI16
    kernel clock can't run in range 3.
  - **Core.ST.W5:** BLE with `low` is an error: "BLE needs clock medium or
    higher" (the radio needs voltage range 1).

**Clock has knock-on effects on buses** — see the I2C kernel-clock minimums in
§7. Picking too low a clock can make a requested I2C speed invalid.

**Radio (WBA55 BLE) needs HSE.** There is no `ble` switch that does this for
you (see §10). To run the radio you must pick a `clock` level whose source is
HSE in the core JSON. Check `definitions/Core-ST-W5-b.json` → `config.clock` for
which levels are HSE-sourced.

---

## 6. `pads` — assign functions to pins *(the heart of the file)*

```json
"pads": {
  "4": "I2C1.CLK",
  "5": "I2C1.DAT",
  "8": "GPIO.OUT",
  "11": "ADC5"
}
```

- **Key** = pad number, as a **string** (`"4"`, not `4`).
- **Value** = a function name that the core JSON lists as available on that pad.

coregen validates every entry: the pad must exist, and the function must be one
the core JSON offers for that pad. Otherwise it prints the available options for
that pad and **exits**. (To discover what a pad can do, read its `functions`
list in `definitions/<Core>.json`.)

### Function naming

| Family   | Form                          | Examples                                  |
|----------|-------------------------------|-------------------------------------------|
| GPIO     | `GPIO.OUT`, `GPIO.IN`         | `"GPIO.OUT"`                              |
| I2C      | `I2C<n>.CLK`, `I2C<n>.DAT`    | `"I2C1.CLK"`, `"I2C3.DAT"`               |
| SPI      | `SPI<n>.CLK/MOSI/MISO/CS`     | `"SPI1.CLK"`, `"SPI1.CS"`                |
| USART    | `USART<n>.TX`, `USART<n>.RX`  | `"USART2.TX"`                            |
| Timer/PWM| `TIM<n>.<ch>` (ch = 1–4)      | `"TIM2.1"`, `"TIM15.2"`                  |
| ADC      | `ADC`, `ADC<n>`, `ADC_IN<n>`, `ADCIN<n>`, optional trailing `+` | `"ADC1"`, `"ADC5"`, `"ADC_IN3"`, `"ADC7+"` |
| DAC      | `DAC1.OUT`                    | `"DAC1.OUT"`                             |
| USB      | `USB.DP`, `USB.DM`            | `"USB.DP"`                              |

Notes:
- **SPI `CS` is software-managed.** `SPI*.CS` pads are configured as plain GPIO
  outputs, driven high (deasserted) before the SPI starts, and driven by the SPI
  HAL, not hardware NSS. So `SPI*.CS` is valid on **any** GPIO pad, not only the
  pad with the NSS function. You cannot change this from config. A bus may have
  several CS pads, one per tile (see §9).
- **ADC differential negative inputs are not supported** — only single-ended
  positive inputs are emitted (a trailing `+` is fine; bare `-` variants are
  excluded until the HAL exposes them). Today coregen emits a single
  `core_adc1` handle regardless of which ADC peripheral a pad belongs to.
- **All pads a bus needs must be present.** An `I2C1` bus needs both `I2C1.CLK`
  and `I2C1.DAT` assigned before it's usable.

> `pins` is a legacy alias for `pads` (coregen reads `pads` first, falling back
> to `pins`). Use `pads` in new files.

---

## 7. `interfaces` — tune the buses you brought up

This section **only tunes** buses/timers that a pad function already created. It
never creates one. Keys are peripheral names (`I2C1`, `SPI1`, `USART2`, `TIM2`,
…). If a named interface has no backing pads, coregen warns and skips it.

### I2C

```json
"interfaces": {
  "I2C1": { "speed": 400000, "pullups": true }
}
```

| Field     | Type | Default | Allowed                              |
|-----------|------|---------|--------------------------------------|
| `speed`   | int  | 400000  | `100000`, `400000`, `1000000` (Hz)   |
| `pullups` | bool | `true`  | enable internal pad pull-ups         |

**Kernel-clock minimums (enforced, exits on failure):** 100 kHz needs ≥1 MHz,
400 kHz needs ≥4 MHz, 1 MHz needs ≥16 MHz on the I2C kernel clock. On WBA55 and
L0 the I2C kernel clock is **fixed at HSI16 (16 MHz)** regardless of SYSCLK (on
the L0 that costs ~100 µA while running at the MSI levels); on L4 and H5 it
follows SYSCLK. So on those, a too-low `clock` level can make a fast `speed`
illegal.

**On-tile pull-ups (`pullups`, top level).** Some Cores have pull-up resistors
on I2C pads that a GPIO switches on. The source of truth is the Core's tile
definition, `config.pullups` in `definitions/<Core>.json`: coregen reads the
names and control pins from there. Today that is:

| Core             | `pad4`   | `pad5`   |
|------------------|----------|----------|
| Core.ST.L4 (b) | via PC15 | via PA9  |
| Core.ST.W5 (b)   | via PC15 | via PC14 |

Other Cores have no switchable pull-ups. Name the ones you want:

```json
"pullups": ["pad4", "pad5"]
```

coregen drives the control pins high in `core_pads_init()`. An unknown name, or
a control pin that is also assigned as a pad, is an error. With I2C at 400 kHz
or more on a pad whose on-tile pull-up is available but not enabled, coregen
prints a WARNING (the MCU's internal ~40 kΩ pull-up is too weak for that).

### SPI

```json
"interfaces": {
  "SPI1": { "mode": 0, "prescaler": 8, "cs_polarity": "active-low" }
}
```

| Field         | Type   | Default        | Allowed                                  |
|---------------|--------|----------------|------------------------------------------|
| `mode`        | int    | `0`            | `0`–`3` (CPOL = bit1, CPHA = bit0)       |
| `prescaler`   | int    | `8`            | `2,4,8,16,32,64,128,256`                 |
| `cs_polarity` | string | `"active-low"` | `"active-low"` or `"active-high"`        |

Invalid `mode` or `prescaler` → coregen exits with the valid set listed.

### USART / UART

```json
"interfaces": {
  "USART2": { "baud": 115200, "rx_interrupt": false }
}
```

| Field          | Type | Default  | Notes                                            |
|----------------|------|----------|--------------------------------------------------|
| `baud`         | int  | `115200` | any rate; validated downstream by the HAL        |
| `rx_interrupt` | bool | `false`  | `true` = interrupt-driven RX with a ring buffer  |

### Timer / PWM

```json
"pads": { "7": "TIM15.1", "8": "TIM2.1" },
"interfaces": {
  "TIM2": { "freq": 1000 }
}
```

| Field  | Type | Default | Notes                                                 |
|--------|------|---------|-------------------------------------------------------|
| `freq` | int  | `1000`  | overflow/PWM frequency in Hz; **shared by all channels on that timer** (hardware constraint) |

Channels come from the `TIM<n>.<ch>` pad functions; duty cycle is set at runtime
via the `core_pwm` API, **not** in config. There is no separate `pwm`,
`capture`, `timers`, or `iwdg` config section — anything you saw claiming
otherwise is stale (see §10).

---

## 8. `gpio` — per-pin electrical details

Optional refinements for pads you set to `GPIO.IN`/`GPIO.OUT` (key = pad number
string, matching an entry in `pads`):

```json
"gpio": {
  "8": { "pull": "up", "exti": "falling" },
  "11": { "output_type": "open-drain", "default": "high" }
}
```

| Field         | Type   | Default       | Allowed / meaning                        |
|---------------|--------|---------------|------------------------------------------|
| `pull`        | string | `"none"`      | `"none"`, `"up"`, `"down"`               |
| `output_type` | string | `"push-pull"` | `"push-pull"`, `"open-drain"`            |
| `speed`       | string | `"medium"`    | slew rate (passed through)               |
| `exti`        | string | none          | `"rising"`, `"falling"`, `"both"` (interrupt edge) |
| `default`     | string | none          | `"high"`, `"low"` (initial output state) |

These are passed through into the generated config without enum validation, so
spelling matters.

---

## 9. `tiles` — attach tile drivers to buses

```json
"tiles": [
  { "tile": "Sense.I.6P6", "bus": "I2C1", "instance": 0 },
  { "tile": "Drive.H",     "bus": "SPI1", "instance": 0, "cs_pad": "8" }
]
```

| Field      | Type   | Required        | Meaning                                            |
|------------|--------|-----------------|----------------------------------------------------|
| `tile`     | string | **yes**         | a name in coregen's `TILE_DRIVER_MAP`              |
| `bus`      | string | **yes**         | must match a configured `I2C*`/`SPI*` interface    |
| `instance` | int    | no (default 0)  | I2C: address slot · SPI: the tile's chip-select id (see below) |
| `cs_pad`   | string | **SPI only**    | pad number of this tile's chip-select (any free GPIO pad) |
| `bus2`     | string | dual-bus tiles  | the tile's second bus, the other protocol from `bus` (I2C + SPI), configured by `pads` like `bus` |
| `cs_id`    | int    | no (see below)  | a dual-bus tile's chip-select id on its SPI bus, 0-255 |

Validation (each fails → exit): `tile` must be in `TILE_DRIVER_MAP`; `bus` must
be a real configured interface; an SPI tile's `cs_pad` must resolve to a GPIO
port/pin. coregen emits a per-tile handle like `tile_sense_i_6p6_i2c1_0` and
adds the driver sources to the build.

**Several SPI tiles on one bus.** Give each tile its own `cs_pad` and its own
`instance`, and they share SCK/MOSI/MISO:

```json
"pads":  { "2": "SPI1.MOSI", "3": "SPI1.CLK", "8": "SPI1.MISO" },
"tiles": [
  { "tile": "Store.O.128", "bus": "SPI1", "instance": 0, "cs_pad": "9" },
  { "tile": "Sense.I.6P6", "bus": "SPI1", "instance": 1, "cs_pad": "4" }
]
```

How the chip select is assigned:

- Every `cs_pad` is claimed as `<bus>.CS` in `pads` (listing it there too, as
  Studio does, is fine; a pad set to `GPIO.OUT` becomes the chip select; any
  other function there is an error). Each is set high, then made an output,
  before the SPI peripheral is enabled.
- **One CS pad on the bus** (one tile, or a hand-written `SPIn.CS` and no
  tiles): it is the bus's own chip select, as before. `core_spi_select()` and
  the tile bridge drive it whatever `cs` a driver passes.
- **Several CS pads**: coregen emits a chip-select map, `instance` → pad, and
  attaches it with `hal_spi_set_cs_map()`. A driver passes `tile->id` as the
  `cs` argument of its SPI calls, which for an SPI tile is its `instance`
  (Store.O.128, Sense.I.6P6, Drive.A.2), so the bridge (`core_tiles_pal()`)
  asserts only that tile's pad, for the whole transaction. A `cs` in no map
  entry returns -1 and selects nothing. `core_spi_select()` drives none of the
  map's pads; use `hal_spi_select_id(&core_spiN, instance)` by hand.
- Errors: two tiles on one bus with the same `cs_pad`, or (with several CS
  pads) the same chip-select id; a tile with no `cs_pad` on a bus with several;
  two tiles on a bus with only one CS pad. A CS pad no tile claims is a NOTE.
- Store.O.128 accepts only instance 0 today, so on a shared bus give it
  instance 0 and the other tiles 1, 2, ….

**Dual-bus tiles** (Sense.CAM.P: I2C for configuration, SPI for image data)
use `bus` for one protocol and `bus2` for the other, and a PAL carrying both
(`core_tiles_pal2(&core_i2cN, &core_spiM)`). Their `instance` is the I2C
address slot, so it cannot name the chip select. The driver takes its SPI chip
select from its config instead (`sense_cam_p_cfg_t.spi_cs`), and that value is
the tile's **`cs_id`**:

```json
"pads":  { "2": "SPI3.CLK", "6": "SPI3.MOSI", "7": "SPI3.MISO",
           "10": "I2C1.CLK", "11": "I2C1.DAT" },
"tiles": [
  { "tile": "Sense.I.6P6", "bus": "SPI3", "instance": 0, "cs_pad": "3" },
  { "tile": "Sense.CAM.P", "bus": "I2C1", "bus2": "SPI3", "instance": 0,
    "cs_pad": "4", "cs_id": 1 }
]
```

```c
sense_cam_p_cfg_t cfg = { .spi_cs = 1 /* its cs_id */, .resolution = SENSE_CAM_P_RES_160x120 };
tile_sense_cam_p_init(core_tiles_pal2(&core_i2c1, &core_spi3), 0, &cam, &cfg);
```

- A dual-bus tile's `cs_pad` belongs to its SPI bus (`bus2` here), and it
  joins that bus's chip-select map under its `cs_id`.
- `cs_id` omitted: the smallest id no other tile on that SPI bus holds, where
  single-bus SPI tiles' instances and explicit `cs_id`s are taken first, then
  defaults go out in `tiles[]` order. Alone on its bus that is 0, which is the
  driver's default (`cfg` NULL gives `spi_cs` 0), so a lone dual-bus tile works
  as before. Emit `cs_id` explicitly whenever something else shares the bus:
  the application must pass the same number as `spi_cs`.
- A single-bus SPI tile's chip-select id is its `instance`; a `cs_id` on one
  must equal it. Two tiles with the same id on one bus are an error.
- `bus2` errors: not an I2C/SPI bus, the same protocol as `bus`, or not
  configured by `pads`.
- coregen's generated `core_init.h` recipe shows the `core_tiles_pal2(...)` call
  and the `spi_cs` to pass.

The set of valid `tile` names is the `TILE_DRIVER_MAP` dict in
`tools/coregen/coregen.py` (~line 1148) — check there for the current list
(Sense.*, Drive.*, Power.*, Display.RGBW, Store.O.128, …).

---

## 10. `usb`, `bootloader`, and `iwdg`

### `usb`

```json
"usb": {
  "enabled": true,
  "vid": "0x1209",
  "pid": "0xDA01",
  "product": "CoreProbe CMSIS-DAP",
  "manufacturer": "Bergsonne",
  "serial": "CP-0001"
}
```

| Field          | Type           | Default     | Notes                                  |
|----------------|----------------|-------------|----------------------------------------|
| `enabled`      | bool           | `false`     | `true` adds USB-CDC init to `core_init()` |
| `vid` / `pid`  | hex-string/int | SDK default | `"0x1209"` or `4617`                    |
| `product`      | string         | SDK default | CMSIS-DAP hosts auto-detect by matching `CMSIS-DAP` in this string |
| `manufacturer` | string         | SDK default | descriptor override                    |
| `serial`       | string         | SDK default | descriptor override                    |

Each unset descriptor field falls back to the SDK default. USB requires a
USB-capable MCU (L4 or H5); on a non-USB core it's a no-op/warning.

### `bootloader`

```json
"bootloader": "custom"
```

| Value      | Layout                                              | Cores               | Needs USB |
|------------|-----------------------------------------------------|---------------------|-----------|
| `"none"`   | app at `0x08000000`, SWD programming only (default) | all                 | no        |
| `"custom"` | 8 KB DFU at `0x08000000`, app at `0x08002000`       | L4 / H5 — **parked** | yes      |
| `"rom"`    | ST ROM DfuSe, app at `0x08000000`                   | L4 (full); H5 (needs power cycle) | yes |

Sets the `BOOTLOADER` / `ROM_DFU` Make variables automatically. A non-`none`
value on a non-USB core warns. Any other string → exit.

`"rom"` is the fleet standard — Studio's default, and where the L4
strike→ROM-DFU brick recovery lives. **`"custom"` is parked**: the Makefile
refuses `BOOTLOADER=1` unless you also pass `CUSTOM_BOOTLOADER_ACK=1`, because
flashing a custom-layout image onto a ROM-DFU board writes at `0x08002000` and
corrupts the resident app. Use `"rom"` unless you know the board carries the
custom bootloader.

### `iwdg`

```json
"iwdg": { "enabled": true, "timeout_ms": 5000 }
```

| Field        | Type | Default | Meaning                                  |
|--------------|------|---------|------------------------------------------|
| `enabled`    | bool | `false` | Start the independent watchdog in `core_init()` |
| `timeout_ms` | int  | `5000`  | Reset if not fed within this window       |

Opt-in, so regenerating an existing project never surprises it with resets.
When enabled, `core_init()` starts the IWDG (and freezes it under the debugger)
— **your loop must call `core_watchdog_feed()`**, or the Core resets.

Best paired with `"bootloader": "rom"`, where the strike counter escalates
repeated watchdog resets into ROM DFU instead of a soft-brick; coregen warns if
you enable it without ROM mode, since an un-fed watchdog then just reset-loops
with no recovery escape. The strike→ROM-DFU logic itself is compiled into every
ROM_DFU build and stays dormant until a watchdog is actually running.

The [project templates](templates/) for the USB-capable Cores ship with
this configured and fed.

### `scope`

```json
"scope": { "enabled": false }
```

The production switch for [`core_scope`](sdk/core/core_scope.h), the live
variable stream Studio's Scope panel plots. Absent (or `true`) leaves the
module available; a project that never calls it still pays nothing, because
the linker drops it. `false` compiles every `core_scope_*` call to nothing —
no flash, no RAM, and on a radio Core no Studio Link service in the GATT
table. Read by the Makefile, not coregen (`make SCOPE_ENABLED=0` is the
one-off override).

### `timer` (Studio only)

```json
"timer": { "tick_ms": 10 }
```

`tick_ms` feeds the Studio simulation tick dispatcher only — it does **not**
configure a hardware kernel tick. Rarely used outside Studio projects.

---

## 10.5 `probe` — how the CoreProbe should power this board

Only relevant when flashing over SWD with a CoreProbe, which can supply the
board it is programming. coregen never reads this; `make flash-coreprobe` and
Studio do.

```json
"probe": {
  "target_power": "5v"
}
```

| Field          | Values                     | Meaning |
|----------------|----------------------------|---------|
| `target_power` | `off` `5v`                 | What the probe puts on T.V+. Default `off`. Current boards supply only 5 V (see below). |
| `target_logic` | —                          | **Ignored.** The logic level is sensed, not declared. Older projects may still carry it; the tooling says so and it is safe to delete. |

**Why the supply is declared.** Supplying the wrong voltage is destructive, and
nothing the probe can measure on an unpowered board tells it what that board
wants. So a person says, and the tooling refuses rather than guessing.

**Why the logic level is not.** Once the board is up, the probe reads it
directly: the sense taps the target's own SWDIO pull-up, so it reports the rail
the board actually runs its IO at, not what was fed in. A board fed 5 V that
regulates to 1.8 V senses as 1.8 V. The probe's firmware does this itself and
sets its level shifter from the reading. **The shifter is never set before the
target has been sensed**, which is why declaring a level would be worse than
useless: it could only ever override a measurement.

**Why it lives in the project.** It is a property of *the board*, so it belongs
with the project rather than being remembered per probe or per session: a probe
setting outlives the board it was chosen for, and a stale supply meeting the
wrong board is exactly the destructive case. Here it travels with the project
and shows up in a diff.

**Omitting it means self-powered**, which is what a project that never thought
about this means. The probe then supplies nothing and senses the board's logic
level from its SWDIO pull-up.

**A board that already has its own rail is never fed**, whatever this says. The
tooling senses first and declines to supply a live board.

```sh
make flash-coreprobe    # applies the declaration, then flashes via probe-rs
make probe-power-check  # says what it WOULD do, drives nothing
```

**Current boards supply only 5 V.** Every CoreProbe has been reworked to two
modes, self-powered or 5 V, by removing the 3.3 V and 1.8 V load switches: with
all three fitted, selecting 5 V back-fed the probe's own rails to about 4.4 V
through the disabled switches. `1v8` and `3v3` are refused until rev b refits
them. A 5 V target is expected to regulate its own logic rail.

A probe that has **not** been reworked must not be given 5 V, and nothing in
software can tell the two apart. It is not caught after supplying either: such a
probe senses about 1.45 V, which reads as a valid 1V8 target.

---

## 11. Keys that are SILENTLY IGNORED

coregen reads a fixed allowlist of keys; the raw config dict is never handed to
the templates. These keys are **read by nothing** and have **no effect**, even
though older docs described them. Don't rely on them:

| Ignored key   | What people expect          | Reality                                                    |
|---------------|-----------------------------|------------------------------------------------------------|
| `ble`         | "enable the radio / HSE"    | No-op. Get HSE by choosing an HSE-sourced `clock` level (§5). |
| `debug`       | emit SWD/JTAG defines       | No-op.                                                      |
| `isp`         | `boot0_pad` etc.            | No-op.                                                      |
| `programming` | `methods`, `boot0_pad`      | No-op.                                                      |
| `timers`      | per-timer freq/tick         | No-op. Use `interfaces.TIM<n>.freq` (§7).                   |
| `pwm`         | per-channel duty            | No-op. Duty is a runtime API call.                          |
| `capture`     | input-capture mapping       | No-op. (Capture isn't config-driven.)                      |

Because unknown keys are ignored rather than rejected, putting any of these in
your file is harmless but does nothing. If you need one of these behaviours,
it has to be added to coregen first.

---

## 12. Validation & error behaviour

coregen is mostly **fail-fast** — most mistakes print an error and exit, so a
bad config won't silently generate wrong code. Things that **exit**:

- a pad that doesn't exist, or a function not available on that pad
- on the L4 Cores, anything other than `USB.*` on the USB D+/D- pads (6/7 on
  Core.ST.L4, 16/17 on Core.ST.L4.2): USB is always on there
- an unknown `pullups` name, or a pull-up control pin that is also a pad
- on Core.ST.W5, `ble` enabled with `clock: "low"`
- an `interfaces` key (in cross-checks) that doesn't match the core JSON
- an unknown `clock` level
- I2C `speed` not in the allowed set, or kernel clock too low for it
- SPI `mode`/`prescaler` out of range
- an unknown tile name, a tile on an unconfigured bus, or an SPI tile whose
  `cs_pad` doesn't resolve or is another function's pad
- SPI tiles on one bus sharing a `cs_pad` (or an `instance`, when the bus has
  several CS pads), or several tiles on a bus with one CS pad (§9)
- an invalid `bootloader` value

Things that only **warn** (non-fatal):

- the resolved `core` name not exactly matching the tile JSON passed (NOTE)
- an `interfaces` entry with no backing pads
- a bootloader/USB request on a non-USB-capable core

Things that are **silent** (no message):

- unknown top-level keys (§11) and typo'd key names
- omitted optional fields → defaults from §3

---

## 13. Cookbook

**Blink only:**
```json
{ "core": "Core.ST.L4.2", "clock": "max", "pads": {} }
```

**GPIO in + out, input on a falling-edge interrupt with pull-up:**
```json
{
  "core": "Core.ST.L0", "clock": "max",
  "pads": { "11": "GPIO.OUT", "8": "GPIO.IN" },
  "gpio": { "8": { "pull": "up", "exti": "falling" } }
}
```

**Two I2C buses at different speeds, three tiles:**
```json
{
  "core": "Core.ST.L4", "clock": "max",
  "pads": {
    "4": "I2C1.CLK", "5": "I2C1.DAT",
    "2": "I2C3.CLK", "8": "I2C3.DAT"
  },
  "interfaces": { "I2C1": { "speed": 400000 }, "I2C3": { "speed": 100000 } },
  "tiles": [
    { "tile": "Display.RGBW", "bus": "I2C1", "instance": 0 },
    { "tile": "Sense.I.6P6",  "bus": "I2C3", "instance": 0 },
    { "tile": "Sense.T.C",    "bus": "I2C3", "instance": 0 }
  ]
}
```

**SPI tile with software CS:**
```json
{
  "core": "Core.ST.W5", "clock": "max",
  "pads": {
    "9": "SPI1.CLK", "6": "SPI1.MOSI", "7": "SPI1.MISO", "8": "SPI1.CS"
  },
  "interfaces": { "SPI1": { "mode": 0, "prescaler": 8 } },
  "tiles": [ { "tile": "Drive.H", "bus": "SPI1", "instance": 0, "cs_pad": "8" } ]
}
```

**PWM on two channels:**
```json
{
  "core": "Core.ST.L4.2", "clock": "max",
  "pads": { "7": "TIM15.1", "8": "TIM2.1" },
  "interfaces": { "TIM2": { "freq": 2000 } }
}
```

**ADC + UART logging:**
```json
{
  "core": "Core.ST.L0", "clock": "high",
  "pads": { "6": "ADC1", "7": "USART2.TX", "2": "USART2.RX" },
  "interfaces": { "USART2": { "baud": 115200 } }
}
```

**USB CDC + ROM-DFU bootloader:**
```json
{
  "core": "Core.ST.L4.2", "clock": "max",
  "bootloader": "rom",
  "usb": { "enabled": true },
  "pads": { "6": "USB.DP", "7": "USB.DM" }
}
```

---

## 14. Quick checklist before you regenerate

- [ ] `core` set, pad numbers are **strings**, values are functions the core JSON allows
- [ ] Every bus has *all* its pads (`CLK`+`DAT`, or `CLK`+`MOSI`+`MISO`)
- [ ] `interfaces`/`tiles` only reference buses you actually created in `pads`
- [ ] Requested I2C `speed` is legal for your `clock` level (kernel-clock minimums, §7)
- [ ] SPI tiles have a `cs_pad`; several tiles on one SPI bus each have their own `cs_pad` and `instance`
- [ ] `clock` is a level *name*, not a frequency
- [ ] No reliance on the ignored keys in §11
- [ ] Regenerate and build all targets you care about (don't commit generated files between coregen markers)
