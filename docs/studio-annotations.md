# `@studio` annotation reference

Studio builds its block palette, DSL imports, and SDK reference docs from C headers in this repo. The bridge is a small set of `@studio` directives placed inside Doxygen blocks. The generator that reads them is [`tools/gen_studio_manifest.py`](../tools/gen_studio_manifest.py); this file is the human-readable companion.

If you're authoring a new `core_*.h` module or a tile driver and want it to show up in Studio, this is the doc. If you're adding directives the generator doesn't yet understand, edit `gen_studio_manifest.py` and update this file in the same change.

## How the generator finds tags

The generator scans every `/** … */` Doxygen block in the file. Inside a block, any line beginning with `@studio <verb>` is a directive. Lines beginning with `@brief`, `@param`, `@return` are standard Doxygen and feed both palette metadata and the SDK reference docs.

A function is exposed to Studio **only** if its preceding Doxygen block contains an `@studio expose` directive. Functions without `@studio expose` still appear on the SDK reference page (so users can read about `core_pwm_init` / `core_pwm_set` even though Studio doesn't call them) — they just don't show up in the palette.

## File-level directives

These go in the file's top-of-file Doxygen block (one per header).

### `@studio category <name> [label=<str>] [icon=<glyph>]` — core headers

Declares this header as a Studio category. Required on every `sdk/core/core_*.h` that exposes anything; otherwise the generator skips the file entirely.

```c
/**
 * core_pwm.h — PWM output and periodic timers
 *
 * @studio category pwm label=Core.PWM icon=◲
 */
```

- `<name>` (positional, required): canonical category id used as the manifest filename (`manifests/sdk-docs/<name>.json`) and the first segment of every `qname` in this file.
- `label=`: human-readable name shown in the palette and on the docs page (default: `<name>`).
- `icon=`: single glyph shown in the palette (default: empty).

A header may declare at most one `@studio category`. A second declaration with different attributes is a warning and is ignored.

### `@studio tile [label=<str>] [icon=<glyph>]` — tile drivers

The tile-driver counterpart. Goes at the top of `drivers/tile_*.h`.

```c
/**
 * tile_sense_mic.h — MAX11645 ADC + AMM-2742 MEMS mic
 *
 * @studio tile label=Sense.MIC icon=◉
 */
```

- `label=`: palette name (default: file stem, e.g. `tile_sense_mic`).
- `icon=`: single glyph (default: empty).

Each tile driver must have exactly one `@studio tile`.

### `@studio event name=<id> [payload=<spec>] [description=<str>] [icon=<glyph>] [mask=<expr>] [read=<fn>] [read_type=<struct>]`

Declares an event the user can write a handler for. Lives at file scope (alongside the category/tile tag, not on a function).

```c
/**
 * @studio category pad label=Core.Pad icon=⎓
 * @studio event name=rising payload=pad:int
 * @studio event name=falling payload=pad:int
 */
```

- `name=` (required): event id. Becomes `on Core.Pad.rising(pad: int) { … }` in the DSL.
- `payload=`: comma-separated `name:type` pairs declaring handler arguments. Types are DSL-level (`int`, `bool`, `float`, `string`). Empty for no-arg events.
- `description=`, `icon=`: shown in the palette.
- **Tile events only** (`mask=`, `read=`, `read_type=`): wire the event into the tile's `on_event` ABI. `mask` is a C expression the dispatcher AND's against the tile's `events` bitmask. `read` is a `void read(tile_t *, <read_type> *)` function the dispatcher calls to populate payload values. `read_type` is the struct type whose field names match the payload parameter names.

Example with all three (Sense.I.6P6 tap detection):

```c
/**
 * @studio tile label=Sense.I.6P6 icon=▦
 * @studio event name=tap mask=ICM42686P_INT_STATUS3_TAP_DET payload=count:int,axis:int,direction:int read=tile_sense_i_6p6_get_tap_result read_type=sense_i_6p6_tap_result_t
 */
```

### `@studio usage` (tile drivers)

A note on how to use the tile from Studio: what init leaves running, what to
call, what to avoid. The text may start on the tag line and runs until a blank
line or the next `@` line. Published as `usage` in both the palette manifest
and the docs manifest.

```c
/**
 * @studio tile label=Sense.M.3G icon=…
 * @studio usage
 *   Starts measuring at 25 Hz as soon as init returns, so get_x_nt, …
 *   always return the latest sample; no read() or wake() needed.
 */
```

### `@studio unsupported` (known gaps)

A capability the hardware has and the API doesn't expose. Continuation lines
form the description, until a blank line or the next `@` line. Say whether
the gap is hardware-gated (the tile doesn't route the pin) or driver-deferred.

- **Tile drivers**, in the file-level `/** … */` block (a `//` comment is not read):
  `@studio unsupported severity=<common|advanced|niche> category="<short>"`.
  Feeds `studio_unsupported` in `manifests/tile-docs/<driver>.json`, the "Driver
  gaps" rows of the docs Coverage Table.
- **SDK core headers**, as `//` comments at the end of the header:
  `// @studio unsupported tier=<1|2|3> value=<H|M|L> title="<short>"`. `tier`
  is the API tier the gap sits at (1 = C API, 2 = DSL / Studio, 3 = later),
  `value` how much closing it is worth. Feeds `manifests/sdk-docs/gaps.json`,
  rendered per subsystem on the website. Delete a gap in the same change that
  closes it: a stale one is published as a missing feature.

### `@studio coverage` (SDK core headers)

Names the subsystem a core header belongs to on the SDK coverage pages, as
`key: value` lines under the tag (`id`, `name`, `page`, `blurb`). `id`, `name`
and `page` identify the header's gaps in `gaps.json`. A header with only
`@studio coverage` and no `@studio category` contributes no functions to
`manifests/sdk-docs/` (e.g. `core_usb_hid.h`, `core_tiles.h`).

```c
/**
 * @studio coverage
 *   id:    power
 *   name:  Power — sleep / Stop / Standby
 *   page:  /docs/sdk/system
 *   blurb: Sleep, Stop, and Standby helpers …
 */
```

## Function-level directives

These go in the Doxygen block immediately before a function declaration.

### `@studio expose category=<name> name=<dsl_name> [returns=<dsl_type>] [icon=<glyph>] [availability=<cores>]`

Marks a function as palette-exposed and DSL-callable. The single most important directive — without it a function only appears on the docs page, not in Studio.

```c
/**
 * Set PWM duty cycle on a pad. Timer handle resolved from config.json.
 *
 * @studio expose category=pwm name=duty
 * @param pad [1..64] Tile pad number configured as a TIMx.<ch> in config.json.
 * @param duty_permil [0..1000] 0 = off, 500 = 50%, 1000 = fully on.
 */
static inline void core_pwm_duty(uint8_t pad, uint16_t duty_permil) { ... }
```

- `category=`: must match the file's `@studio category` (or be `tile` for tile drivers). Drives the DSL qname (`Core.<category>.<name>`).
- `name=`: DSL-facing name. Convention: lowercase snake_case, short. The generated qname is `Core.PWM.duty`, not `Core.PWM.core_pwm_duty`.
- `returns=`: DSL-level return type. Required for non-void hosts so Studio can use the call as an expression. Values: `int`, `bool`, `float`, `string`, `int[N]` / `bool[N]` / `float[N]` (fixed-length array). Omit for void hosts (statement-only).
- `icon=`: per-host glyph in the palette (rare; usually inherited from the category).
- `availability=`: comma-separated list of Cores this host runs on (e.g. `availability=Core.ST.H5,Core.ST.W5`). Omit when the host runs on every Core in the SDK.

### `@studio param <cname> enum {KEY=label, ...}`

Attaches friendly enum labels to an int-valued parameter. The DSL renders the labels in the block editor; codegen still passes the C identifier.

```c
/**
 * Set the accelerometer full-scale range.
 *
 * @studio expose category=tile name=set_accel_range
 * @studio param range enum {FSR_2G=±2g, FSR_4G=±4g, FSR_8G=±8g, FSR_16G=±16g}
 * @param range Accelerometer full-scale range.
 */
void tile_sense_i_6p6_set_accel_range(tile_t *tile, uint8_t range);
```

- `<cname>` (positional): C parameter name (matches the function signature).
- Values inside `{ … }`: comma-separated `KEY=label` pairs. `KEY` is the C identifier the codegen emits; `label` is what the user sees. A bare `KEY` (no `=`) uses the C identifier as its own label.

### `@studio param <cname> type=<dsl_type>`

Override the DSL type for a parameter the C signature describes as a pointer. Used for fixed-length array inputs.

```c
/**
 * Apply a 16-bin equalizer profile.
 *
 * @studio expose category=tile name=apply_eq
 * @studio param effects type=int[16]
 */
void tile_audio_apply_eq(tile_t *tile, int16_t *effects);
```

Without the override, `int16_t *` would map to `?int16_t*` (an unknown DSL type). With it, the user passes a 16-element int array and the DSL handles the buffer.

### `@studio out_buffer <cname> type=<ctype> length=<N>`

Promote a pointer-out parameter into a typed array return. The driver writes into the buffer and returns void; Studio presents the call as if it returns `int[N]`.

```c
/**
 * Read raw accelerometer [X, Y, Z]. Convert: g = raw / sensitivity.
 *
 * @studio expose category=tile name=get_raw_accels returns=int[3]
 * @studio out_buffer buffer type=int16_t length=3
 */
void tile_sense_i_6p6_get_raw_accels(tile_t *tile, int16_t *buffer);
```

- `<cname>` (positional, required): the C parameter the driver writes into (here `buffer`).
- `type=` (required): C element type. Must be one of `int8_t`, `int16_t`, `int32_t`, `uint8_t`, `uint16_t`, `uint32_t`, `float`.
- `length=` (required): integer, the fixed buffer length.

The annotated parameter is stripped from the DSL-visible parameter list. Pair with `returns=int[N]` (or `float[N]`) on the `@studio expose` so the manifest carries both the DSL return type and the underlying C-buffer shape.

Only one `@studio out_buffer` per function. Multiple are dropped with a warning.

The variable-length form, `@studio out_buffer <cname> type=<ctype> cap_param=<name>`,
keeps the function's scalar return (the count) and exposes the buffer as an
array the DSL caller passes in; the `cap_param` C parameter (the buffer's
capacity) is filled in by codegen and stripped from the DSL signature.

### `@studio out_scalar <cname> type=<ctype>`

A scalar pointer parameter (`int32_t *out`) the function writes into. The DSL
caller passes a local variable and gets the value back when the call returns;
the parameter stays in the DSL signature, flagged `out_scalar`, and codegen
passes its address. Several per function are allowed.

```c
/**
 * @studio expose category=tile name=read_tilt_centi_degrees returns=bool section=runtime
 * @studio out_scalar out_centi_deg type=int32_t
 */
uint8_t tile_sense_i_6p6_read_tilt_centi_degrees_flat(tile_t *tile, uint8_t axis,
                                                      int32_t *out_centi_deg);
```

### `@studio in_buffer <cname> type=<ctype> [length=<N>] [length_param=<name>]`

A caller-supplied array. The buffer parameter (and its length parameter, if
any) collapse into one DSL array parameter in the buffer's position:

- `length=N length_param=<name>`: the DSL sees `int[N]`; codegen passes `N` as
  the C count.
- `length_param=<name>` only: the DSL sees `int[]`; codegen passes the
  caller's array length.
- `length=N` only: fixed length, and the C function has no count argument.

```c
/**
 * @studio expose category=tile name=play_sequence section=runtime
 * @studio in_buffer effects type=uint8_t length=8 length_param=count
 */
void tile_drive_h_play_sequence(tile_t *tile, const uint8_t *effects, uint8_t count);
```

### `@studio control <arg> label="…" tier=basic|advanced …` and `@studio require expr="…" message="…"`

Vibe Settings: `control` makes an argument of an exposed tile function a
setting a person can change in Studio's inspector; `require` states a rule
between settings. Both are tile-driver only and validated strictly (the
generator fails the build on a name it cannot resolve). The attribute table,
the expression language and the failure rules are in
[AI.md, "Vibe Settings"](../AI.md); reference driver `drivers/tile_sense_i_6p6.h`.

## Standard Doxygen the generator consumes

These are not `@studio` directives but the generator reads them and they directly affect what the user sees.

### `@brief <one-liner>`

First sentence shown on the SDK docs page and as the palette tooltip. If absent, the generator falls back to the first non-tag line of the Doxygen block. A block with only `@deprecated <text>` gets the brief "Deprecated. <text>".

Every tag's text runs until a blank line or the next `@` line, as Doxygen reads it, so a wrapped `@param`, `@return`, `@note` or `@warning` keeps its continuation lines. Indent them; don't start them with `@`.

### `@param <cname> [{<dsl_type>}] [[lo..hi]] [<unit>] <description>`

```c
/**
 * @param period_ms {int} [0..60000] ms Time between LED toggles.
 */
```

- `<cname>`: C parameter name.
- `{<dsl_type>}`: optional, override the DSL-mapped type (similar to `@studio param … type=`). Prefer the `@studio param` form for new code; this brace form is older syntax.
- `[lo..hi]`: numeric range, surfaced as min/max in the palette + docs.
- `<unit>`: one of the recognised unit tokens (`ms`, `us`, `s`, `hz`, `mv`, `v`, `pct`, `°c`, …; full list in `gen_studio_manifest.py:UNIT_VOCAB`). Shown next to the parameter.
- `<description>`: free text.

The DSL parameter name **is the C identifier**. There is no rename mechanism — if the C param is `duty_permil`, the DSL block label says `duty_permil`. Pick C names that read well in the palette.

### `@return <description>`

Shown on the SDK docs page (`return_desc` in `manifests/sdk-docs/`, `return` in a tile's docs manifest). Not consumed by Studio codegen — return semantics come from `returns=` on the expose tag.

### `@note` / `@warning <text>`

Tile drivers: published as `notes` in the docs manifest; a `@warning` becomes a note starting "Warning:".

### Types, constants and macros (SDK core headers)

The SDK reference also lists, per category, the core header's documented
typedefs (`types`: callbacks, structs with their `/**< */` field docs, enums
with their member docs), its `#define NAME value  /**< text */` constants
(`constants`), and its documented function-like macros (`macros`, e.g.
`core_scope_watch`). Give a typedef or macro a `/** … */` block, and a struct
field or constant a trailing `/**< … */`, for it to appear.

## Putting it all together — a complete header

Here is an annotated `core_*.h` showing every directive at once:

```c
/**
 * core_widget.h — example module
 *
 * @studio category widget label=Core.Widget icon=◯
 * @studio event name=ready
 * @studio event name=overflow payload=count:int
 */

/**
 * Initialize the widget. Not Studio-exposed — coregen calls this from core_init().
 */
hal_status_t core_widget_init(void);

/**
 * Set the widget intensity.
 *
 * @studio expose category=widget name=set_intensity
 * @studio param mode enum {SOFT=soft, HARD=hard}
 * @param mode Intensity ramp mode.
 * @param level [0..100] pct Intensity percentage.
 */
void core_widget_set(uint8_t mode, uint8_t level);

/**
 * Read the current widget reading.
 *
 * @studio expose category=widget name=read returns=int
 */
int core_widget_read(void);

/**
 * Capture a 16-sample trace.
 *
 * @studio expose category=widget name=capture returns=int[16]
 * @studio out_buffer out type=int16_t length=16
 */
void core_widget_capture(int16_t *out);
```

This header generates:
- A `Core.Widget` palette entry with three blocks (`set_intensity`, `read`, `capture`) and two events (`ready`, `overflow`).
- A `manifests/sdk-docs/widget.json` carrying all four functions (init included, marked `studio_exposed: false`).
- An `int[16]` return type for `capture` with a transparently-managed stack buffer.

## Validating your driver

Once the header has annotations, run the generator:

```sh
tools/gen_studio_manifest.py
```

It writes:
- `manifests/core.json` — palette manifest used by Studio's frontend.
- `manifests/sdk-docs/<category>.json` — per-category SDK reference doc.
- `manifests/tile_<name>.json` — per-tile manifest (for tile drivers).

To verify the manifests on disk match the headers without overwriting (CI-friendly):

```sh
tools/gen_studio_manifest.py --check
```

The Python test suite `tools/test_gen_studio_manifest.py` covers each directive's parsing rules. If you add a directive, add a test there.

A "minimum viable Studio project" template that imports a single host and verifies it on hardware lives at TODO — link once the template lands.

## Common warnings and what they mean

The generator prints warnings to stderr without failing the build. Watch for:

- `@param count N != C arg count M + out_buffer count K` — your `@param` lines don't cover every C argument. Either add a missing `@param` or check that `@studio out_buffer` matches the parameter you intended.
- `@studio out_buffer <name> doesn't match any C parameter` — typo'd the parameter name.
- `category '<name>' redeclared in <file>` — two headers claim the same category. The first wins; rename or merge.
- `N @studio event(s) but K @studio category declarations — events dropped` — events live on a category, so the file must have exactly one `@studio category`.
- `unknown brace key '<key>' — only 'enum' is recognised` — typo on `@studio param … enum {…}`.

## Driver authoring checklist

When adding `@studio` annotations to an existing driver:

1. File-level: add `@studio category <name>` (SDK module) or `@studio tile label=<Name>` (tile driver). Pick a category name that's unique across the repo.
2. For each function the user should be able to call from Studio, add `@studio expose category=<name> name=<dsl_name>` plus `returns=` if non-void.
3. Add `@param` lines with ranges and units for every C parameter.
4. For pointer-output parameters: add `@studio out_buffer <param> type=<ctype> length=<N>` and `returns=int[N]` (or `float[N]`) on the expose tag.
5. For enum-coded parameters: add `@studio param <param> enum {…}`.
6. List what the chip can do and the driver doesn't with `@studio unsupported`, and add a `@studio usage` note if init leaves something running.
7. Run `tools/gen_studio_manifest.py` and check stderr for warnings.
8. Commit the regenerated `manifests/` files alongside the header changes — the manifests are checked in.

Reference patterns:
- Default-instance helper (most common): [`core_pwm_duty`](../sdk/core/core_pwm.h) in `sdk/core/core_pwm.h`.
- Tile driver with events: [`tile_sense_i_6p6.h`](../drivers/tile_sense_i_6p6.h) — five events, two with payload, two with read functions.
- Array-returning host: `tile_sense_i_6p6_get_raw_accels` in the same file.
