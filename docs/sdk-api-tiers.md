# SDK API tiers — low-level vs default-instance

The Cores SDK ships most peripherals with two tiers of public API. Generated Studio projects use the upper tier; legacy examples and bring-up tests tend to use the lower one. Both are supported. This doc explains the difference, names the convention, and lists which functions live on each tier so you can read SDK docs and Studio-generated C without confusion.

## The two tiers

### Tier 1 — handle-based ("explicit instance")

The classic embedded pattern. The user owns a handle struct, passes a peripheral instance, and calls `init` / `set` / `start` / `stop` against the handle.

```c
core_timer_t pwm;
core_pwm_init(&pwm, TIM2, 1000);   // handle, instance, frequency
core_pwm_set(&pwm, 3, 500);        // handle, channel, duty
core_pwm_start(&pwm);
```

The caller decides which timer to use, when to allocate the handle, and when to start it. Maximum control. Verbose.

### Tier 2 — default-instance ("project-centric")

Used when a `config.json` sits next to the source file. Coregen reads `config.json`, allocates one handle per declared peripheral as a static extern (`core_tim2`, `core_dac`, etc.), and emits a dispatcher that maps a pad to the right handle. The user calls a thin wrapper that takes only the user-facing arguments — no handle, no instance.

```c
core_pwm_duty(7, 500);             // pad, duty
```

The handle is hidden. The instance is resolved automatically. Initialization happens during `core_init()`. This is the API Studio codegen targets.

### Why both exist

Tier 1 is the foundation. It works on bare hardware with no config.json, makes peripheral lifetimes obvious, and is what bring-up tests and HAL-level examples use. Tier 2 is layered on top of Tier 1 as `static inline` wrappers — there's no runtime cost, and reading the Tier 2 source is the fastest way to see what the lower tier actually does.

A user of the SDK should reach for Tier 2 unless they need:
- Multiple instances of the same peripheral with different settings.
- A peripheral the project doesn't declare in `config.json`.
- Fine-grained control over init order, frequency changes mid-flight, or per-channel state the wrapper doesn't expose.

A user of Studio **always** sees Tier 2 in their generated C. If they then "escape to C" to extend a Studio project, they should feel the Tier 2 conventions are continuous with what Studio was emitting.

## Which functions are on which tier?

The authoritative answer is `studio_exposed: true` in `manifests/sdk-docs/<category>.json`. The summary below is a snapshot — when in doubt, run `tools/gen_studio_manifest.py` and read the JSON.

Snapshot generated from `manifests/sdk-docs/*.json` on 2026-04-25. "Exposed" means `studio_exposed: true` and shows up in the Studio palette / generated C; "Internal" is everything else on the docs page (init helpers, lower-level building blocks, raw-data variants). When in doubt, run `tools/gen_studio_manifest.py` and read the JSON directly.

| Category | Exposed (Tier 2) | Internal (Tier 1 / one-shot) |
|---|---|---|
| `pwm` | `core_pwm_duty` | `core_pwm_init`, `core_pwm_set`, `core_pwm_set_freq`, `core_pwm_start`, `core_pwm_stop`, `core_pwm_init_pad`, `core_pwm_set_pad`, plus the `core_every_*` family (periodic-tick helpers) |
| `timer` | — | `core_timer_init_freq`, `core_timer_init_tick`, `core_timer_start`, `core_timer_stop`, `core_timer_set_freq`, `core_timer_pwm_set`, `core_timer_capture_init`, `core_timer_capture_read`, `core_timer_enable_tick`, `core_timer_disable_tick`, `core_tick_init` |
| `adc` | `core_adc_read_pad`, `core_adc_read_mv_pad`, `core_adc_temp_decidegc`, `core_adc_vdd_mv` | `core_adc_init`, `core_adc_add`, `core_adc_read`, `core_adc_read_mv`, `core_adc_temp`, `core_adc_vdd`, `core_adc_start_dma`, `core_adc_stop_dma`, `core_adc_dma_read` |
| `dac` | `core_dac_init`, `core_dac_write`, `core_dac_write_mv`, `core_dac_read` | — (DAC's user-facing surface is small enough that every function is exposed) |
| `pad` | `core_pad_write`, `core_pad_read`, `core_pad_toggle` | `core_pad_output`, `core_pad_output_od`, `core_pad_input`, `core_pad_analog`, `core_pad_speed`, `core_pad_on_change`, `core_pad_on_change_stop` (events are declared via `@studio event`, not as exposed functions) |
| `led` | `core_led_on`, `core_led_off`, `core_led_toggle`, `core_led_blink`, `core_led_heartbeat` | `core_led_init`, `core_led_sos` |
| `usb` | `core_usb_print`, `core_usb_print_int`, `core_usb_print_float`, `core_usb_print_bool` | `core_usb_init`, `core_usb_connected`, `core_usb_write`, `core_usb_on_receive`, `core_usb_available`, `core_usb_getc`, `core_usb_read`, `core_usb_try_read` |
| `nvm` | `core_nvm_size` | `core_nvm_read`, `core_nvm_write` (currently Tier 1 only — Studio-side persistence is the [A4d-3](https://www.notion.so/34994a2cf4c681838739dbbe0adef5a9) item) |
| `rtc` | `core_rtc_init`, `core_rtc_set_time`, `core_rtc_set_date`, `core_rtc_wakeup`, `core_rtc_wakeup_stop`, `core_rtc_set_alarm`, `core_rtc_clear_alarm`, `core_rtc_alarm_fired` | `core_rtc_get_time`, `core_rtc_get_date` |
| `timing` | `core_delay_ms`, `core_delay_us`, `core_millis`, `core_timeout` | — |
| `watchdog` | `core_watchdog_start`, `core_watchdog_feed` | `core_watchdog_caused_reset`, `core_watchdog_clear_flags` |
| `backup` | `core_backup_read`, `core_backup_write` | `_core_backup_ensure_clk` (private helper) |

Fixed-singleton peripherals (LED, USB, watchdog) don't have a meaningful Tier 1 — the hardware is not user-configurable, so coregen just emits the init unconditionally. The "Internal" column for those categories is mostly raw-byte read/write paths that Studio doesn't need but C users do.

A few categories worth a closer look:

- **`pwm` is the cleanest two-tier example.** A single Tier 2 entry point (`core_pwm_duty`) sits on top of a full Tier 1 surface. New default-instance peripherals should follow this shape.
- **`timer` has no Tier 2 yet.** Everything is handle-based; Studio doesn't currently expose direct timer programming. The `core_every_*` helpers in `core_pwm.h` are the closest thing to a Tier 2 timing primitive.
- **`adc` splits the namespace by suffix.** `_pad` variants are Tier 2 (resolve from config.json); the un-suffixed versions are Tier 1 (take a handle). When writing examples, prefer the `_pad` form.
- **`nvm` Tier 2 is one symbol** (`core_nvm_size`) reporting the available region size. Read/write are Tier 1 only because the Studio persistence story (A4d-3) is still in design — once that lands, expect a Tier 2 wrapper to follow.

## Reading Studio-generated C

A typical project's `main.c` looks like:

```c
#include "core_init.h"
#include "core_led.h"
#include "core_pwm.h"
#include "core_pad.h"

void on_start(void) {
    core_led_heartbeat(500);
}

void on_pad_rising(uint8_t pad) {
    core_pwm_duty(7, 500);
}
```

Every call here is Tier 2. There's no `core_timer_t` declared, no `TIM2`, no `core_pwm_init`. The handle for `TIM2` lives in coregen's output (`core_init.c`) as `static hal_timer_t core_tim2`, and `core_pwm_timer_for_pad(7)` returns `&core_tim2` because pad 7 was declared in `config.json` as a `TIM2.<ch>`.

If you escape to C and want to take over the timer manually, drop down to Tier 1 by including `core_timer.h` and treating the same `core_tim2` extern as a regular `core_timer_t`. The two tiers share state — Tier 2 wrappers are just sugar.

## Reading SDK docs

The `/docs/sdk/<category>` pages on the website render every documented function from the matching `manifests/sdk-docs/<category>.json`. Functions marked `studio_exposed: true` are Tier 2; the rest are Tier 1 (or Tier 0, if the header re-exports HAL/LL signatures).

Convention: **examples on each docs page should lead with Tier 2 calls**. The Tier 1 reference sits below as a "lower-level building blocks" section. As of 2026-04-25 most pages still open with Tier 1 examples (`core_pwm_init(&pwm, …)` rather than `core_pwm_duty(pad, …)`); fixing this is the [Z1 SDK-docs-consistency](https://www.notion.so/34994a2cf4c681838739dbbe0adef5a9) sweep and is in progress. If you find a page that's drifted, file it against `cores/docs/sdk-api-tiers.md`.

## Mixing tiers in escape-to-C code

The two tiers are wrappers, not silos. A user who escapes from Studio to C — exporting `config.json`, `main.c`, and the coregen output — can freely mix Tier 2 and Tier 1 calls in the same program. They share state.

The mechanics: every peripheral declared in `config.json` produces one extern handle in coregen's output (`core_init.c`):

```c
/* coregen output (excerpt) — declared in core_init.c */
hal_timer_t core_tim2;
hal_dac_t   core_dac;
hal_adc_t   core_adc1;
```

Tier 2 helpers resolve to those same externs internally. `core_pwm_duty(7, …)` calls `core_pwm_timer_for_pad(7)` which returns `&core_tim2` because pad 7 was declared as a `TIM2.<ch>`. So when you escape to C, you can:

**1. Use Tier 2 for everything in `config.json`, ignore Tier 1.** Most users — same calls Studio generated.

**2. Mix Tier 2 with Tier 1 on the same coregen-managed handle.** When you need a Tier 1 capability the wrapper doesn't expose (input capture, runtime frequency change, channel-by-channel duty control), include the Tier 1 header and reach into the extern:

```c
#include "core.h"           /* core_pwm_duty etc. */
#include "core_timer.h"     /* core_timer_set_freq, core_timer_capture_init */

extern core_timer_t core_tim2;   /* coregen's handle for TIM2 */

void on_start(void) {
    core_pwm_duty(7, 500);                    /* Tier 2 */
    core_timer_capture_init(&core_tim2, 1);   /* Tier 1 — same handle */
}
```

The state is shared. There's no double-init or "which tier owns this timer" rule — the handle is just a struct, and either tier reads/writes the same registers.

**3. Use Tier 1 with your own handle for peripherals not in `config.json`.** Coregen only emits an extern for peripherals you declare. If you want to drive `TIM5` ad-hoc without adding it to `config.json`, declare your own handle and stay in Tier 1 — it won't conflict because coregen never touched `TIM5`:

```c
core_timer_t my_tim5;

core_timer_init_freq(&my_tim5, TIM5, 10000);
core_timer_pwm_set(&my_tim5, 1, 250);
core_timer_start(&my_tim5);
```

**The only thing to avoid**: declaring your own handle for a peripheral that's *also* in `config.json`. Coregen's init runs during `core_init()` and your re-init would clobber it (or vice versa). If a peripheral is in `config.json`, treat coregen's extern as the only handle and reach for it explicitly when you need Tier 1 ops.

## Adding peripherals or tiles after escaping

A common shape: someone starts in Studio, escapes to C, and now wants to add a pin or a tile that wasn't in the original project. The runtime work is small — Tier 2 helpers do most of it for you — but the *coregen* work is what makes them possible. Three steps every time:

1. **Declare the new thing in `config.json`.** Coregen reads this on every build to decide what handles to allocate, what to initialize, and what dispatchers to emit. If it isn't here, no Tier 2 helper can find it at runtime.
2. **Rebuild.** `make` re-runs coregen when `config.json` changes; a fresh `core_init.c` and `core_pads.h` land alongside your existing files. You don't edit either by hand — they're build artifacts.
3. **Call the helper.** `core_init()` is already wired into the `main()` Studio generated, and it runs the freshly-emitted init code on the next reset. Then call the Tier 2 function. No new init line.

Three recipes covering the most common cases:

### Adding a PWM pad

```jsonc
// config.json
{
  "pads": {
    "7": "TIM2.3"        // pad 7 → TIM2 channel 3
  }
}
```

```c
// main.c — anywhere after core_init()
core_pwm_duty(7, 500);   // 50% duty
```

Coregen sees the new pad, allocates `core_tim2`, emits init + start in `core_init()`, and adds pad 7 to the `core_pwm_timer_for_pad()` dispatcher. The Tier 2 call resolves the handle and sets duty.

### Adding a GPIO pad

```jsonc
// config.json
{
  "pads": {
    "3": "GPIO.OUT",     // output
    "5": "GPIO.IN"       // input
  }
}
```

```c
// main.c
core_pad_write(3, 1);                 // drive pad 3 high
bool pressed = core_pad_read(5);      // read pad 5
```

GPIO pads don't need a peripheral handle, but coregen still has to configure the port + pin direction during `core_init()`. The Tier 2 helpers compile to BSRR / IDR register writes — zero overhead.

### Adding a new tile

```jsonc
// config.json (Core.ST.L4: I2C3 on pads 2 and 8)
{
  "core": "Core.ST.L4",
  "pads": { "2": "I2C3.CLK", "8": "I2C3.DAT" },
  "interfaces": { "I2C3": { "speed": 400000 } },
  "tiles": [
    { "tile": "Sense.MIC", "bus": "I2C3", "instance": 0 }
  ]
}
```

```c
// main.c
#include "core.h"
#include "core_tiles.h"
#include "tile_handles.h"     // generated: declares tile_sense_mic_i2c3_0

int main(void) {
    core_init();
    tile_sense_mic_init(core_tiles_pal(&core_i2c3), 0, &tile_sense_mic_i2c3_0, NULL);
    for (;;) {
        uint16_t mv = tile_sense_mic_get_raw_mv(&tile_sense_mic_i2c3_0);
        (void)mv;
    }
}
```

Coregen brings up `I2C3`, adds the driver to the build (`core_drivers.mk`),
declares a `tile_t` per tile in `tile_handles.h` (named `<prefix>_<bus>_<instance>`)
and writes the init call into the recipe comment at the end of
`coregen/core_init.h`. It does **not** call the driver's init: that belongs in
main.c, where you control the order and the error handling.

`instance` picks the tile's I2C address (0 = the default, 1 = the alternate,
per the driver's address table); there is no `i2c_address` key. Two of the same
tile on different buses: list both, each gets its own handle. See
[config-json.md §9](../config-json.md) for SPI tiles (`cs_pad`) and dual-bus
tiles.

### What if the rebuild produces an undefined-symbol error?

Two common shapes when config.json and your code disagree:

- `undefined reference to core_pwm_timer_for_pad` — the dispatcher only ships when at least one pad uses a timer. Add a `TIMx.y` pad to `config.json` (or you wrote `core_pwm_duty` but forgot the pads entry).
- `undefined reference to tile_sense_mic_init` — the tile's source file isn't in the build. Coregen wires this automatically when the tile is in `tiles[]`; if it's missing, the tile entry didn't make it through (typo in `tile`, or you edited a stale `tile_handles.h` instead of `config.json`).

Both errors mean coregen and your code are looking at different views of the project. Re-running `make` after touching `config.json` resolves them.

## Naming conventions to watch for

- **Tier 2 functions don't take a handle.** First argument is a `pad`, `pin`, `channel`, or `permil` — never a `core_*_t *h`.
- **Tier 2 functions match a category.** `core_pwm_duty` lives in `core_pwm.h` under the `pwm` category. `core_led_blink` lives in `core_led.h` under `led`. Reading the generated C, the prefix tells you the docs page.
- **Tier 1 functions are typically named with a verb.** `init`, `set`, `read`, `start`, `stop`. Tier 2 names are nouns or short verbs in user vocabulary: `duty`, `read_mv`, `heartbeat`, `now`.
- **One-shot Tier 1 functions** (the clock, pad and bus inits) are called by coregen during `core_init()`. The user shouldn't need to call them. They're documented for completeness; if you find yourself calling them by hand, check whether you're missing a coregen step. Two exceptions you do call yourself: `core_led_init()` (the templates do, after `core_init()`) and each tile driver's `_init()` (see above).

## When a Tier 2 wrapper is missing

If the generated C wants to call `core_<thing>_<verb>(...)` and the symbol isn't there, two things might be wrong:

1. The Tier 2 wrapper hasn't been written. Add a `static inline` in the relevant `core_*.h`, mark it `@studio expose`, and run `tools/gen_studio_manifest.py`. See [studio-annotations.md](studio-annotations.md).
2. The wrapper exists but coregen isn't emitting the supporting scaffold (the dispatcher, the init call, the start). This is the [Z1 high-priority "Studio C codegen: missing peripheral init/scaffold"](https://www.notion.so/34994a2cf4c681838739dbbe0adef5a9) item. The wrapper ships with whatever extern declarations and helpers it needs from coregen; if coregen doesn't fill them in, the link fails.

The first failure is a missing API. The second is a missing scaffold. Both are real, both are tracked, and both are worth filing.

## Related docs

- [studio-annotations.md](studio-annotations.md) — how to add a Tier 2 wrapper to a header.
- [`AI.md`](../AI.md) — driver authoring guide for AI agents (covers tile drivers, which follow the same tier pattern: `tile_<name>_init` is Tier 1, palette-exposed functions are Tier 2).
