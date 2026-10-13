# BLE contract — design note (Phase 0)

Status: **implemented** (coregen `build_ble_contract()` and
`tools/coregen/templates/ble_contract.{c,h}.j2`, first in a9d6895, 2026-08-25;
example `tests/ble-contract`; config reference in [config-json.md](../config-json.md)
§10). This page is the original design note, written 2026-08-25 against the
Ring_Av2 project (not in this repo) as the acceptance test; where it and the
code differ, the code wins.

## Why this exists

`config.json` can now turn the radio on (`"ble": { "enabled": true }`, tiles#209)
and Studio can tick that box (web#241). That makes BLE *reachable*. It does not
make it *usable*: there are no `Core.BLE.*` DSL hosts, no twin, and no help with
the part that actually costs time.

## Principles

Two, and they constrain everything below.

**Approachable but complete.** Studio should be comfortable for a novice and
still give an expert everything they need — in the *same* object, with advanced
fields hidden rather than absent. A project that succeeds must never require a
format conversion at the moment it gains deployed clients.

**The file is the source of truth; Studio is one editor of it.** Plenty of
projects are hand-authored or agent-authored outside Studio — `Ring_Av2` is the
only shipped BLE product we have and it is hand-written, `tests/` is entirely
hand-authored, and agent-authoring is growing. Anything that exists only as a
Studio UI affordance is unavailable to all of them. Concretely: every binding is
a JSON field before it is a control, and every diagnostic runs in coregen before
it is a Studio marker.

This is the same branches model Studio already uses for escape-to-C. BLE should
not invent a different one.

## What the Ring tells us

Measured, not guessed:

| Artifact | Size | Maintained how |
|---|---|---|
| `firmware/ring_ble_contract.h` | 368 lines | source of truth, hand-written |
| `Ring_AV_MacOS/.../RingBLEContract.swift` | **699 lines** | hand-mirrored from the header |
| `BLE_CONTRACT_REQUESTS.md` | 601 lines, 18 requests | `REQUESTED → DONE(fw) → MIRRORED` protocol between two humans |

**The GATT builder is not the problem.** `build_services()` is ~60 readable
lines, and the SDK audit rates the API well: *"correct SIG-vs-custom service
builders, clean config table."* Generating it saves little on its own.

**The cross-language mirror is the problem.** Over a thousand lines of
duplicated interface definition kept in sync by discipline.

Three constraints the Ring imposes on any design — each one rules out an
"obvious" simplification:

1. **IDs are pinned and family-organised**, never generated:
   `FExx` System/Power · `FCxx` Sense/Motion · `FDxx` Control/Haptics ·
   `FBxx` Audio. The contract's own words: *"pick an unused id; existing ones
   never shift, so deployed clients keep working."* `core_ble_add_service()`'s
   auto-sequential `0xB000, 0xB100…` is the **anti-pattern** here — inserting a
   service renumbers everything after it and breaks every deployed app. (This is
   already flagged in `core_ble.h` as the "Custom UUIDs" gap.)
2. **Characteristics are packed structs, not scalars**, each with a leading
   **format-version byte** so layouts can grow without a new characteristic.
   `RING_CH_MOTION` carries *four* layouts (v1/v2 single, v3/v4 batched) behind
   one UUID.
3. **Both SIG-adopted and custom** services coexist — DIS and Battery must use
   their standard 16-bit UUIDs so the host OS recognises them.

## Prior art — what to copy, what to skip

| System | Shape | Verdict |
|---|---|---|
| **Silicon Labs GATT Configurator** | GUI in Simplicity Studio → `gatt.xml` → generated `gatt_db.c/.h` | **The closest precedent.** Copy the model: declared structure, generated device code. Firmware-side only. |
| **Zephyr `BT_GATT_SERVICE_DEFINE`** | Declarative C macros, no external file | Nothing to generate *from*; no client side. Good ergonomics reference only. |
| **Renesas e2Studio, Cypress PSoC** | Vendor GUI → firmware profile code | Same shape as Silicon Labs, same limit: device only. |
| **Matter ZAP + `.matter` IDL** | GUI → `.zap` → `.matter` IDL → codegen for **device firmware and clients** | The strongest full precedent — proves the multi-target model works. Matter-specific and heavyweight; take the architecture, not the tool. |
| **Kaitai Struct** (`.ksy`) | Declarative packed-binary spec → parsers in ~12 languages, incl. a Swift runtime | Right idea for Tier B. **Not usable as-is**: plain **C is not a target** (C++ only) and it is parse-oriented, while firmware must *encode*. |

The notable gap in the survey: **no tool generates both the peripheral firmware
and the client from one GATT spec.** Every vendor configurator stops at the
device. That is exactly where the Ring's 699 hand-written Swift lines come from,
and it is the part worth building.

Conclusion: **copy the established model, don't take a dependency.** The
conceptual model (declared services → generated device code) is well-trodden;
our serialization of it should live in the project doc alongside `pads` and
`tiles`, because Studio already owns that file.

## Proposed split

**Tier A — structure.** Services, characteristics, pinned IDs, access modes,
value sizes. Generates the builder, the contract header, and the client's
UUID/access tables.

**Tier B — payload layouts.** Packed fields, version-selected variants,
bitfields, batched arrays. Generates encode/decode on both sides. This is a real
IDL and a much larger lift.

Tier A alone is a complete, useful increment: it removes every UUID and access
constant from both sides of the mirror, which is the bulk of the boilerplate,
while leaving hand-written layout code untouched.

## Tier A schema — validated against Ring

```json
{
  "device": { "name_prefix": "Ring-", "name_suffix": "uid16" },
  "services": [
    { "name": "Battery", "sig": "0x180F",
      "characteristics": [
        { "name": "Battery Level", "sig": "0x2A19",
          "access": ["read","notify"], "type": "uint8" } ] },
    { "name": "Ring System", "id": "0xFE40",
      "characteristics": [
        { "name": "Power Status", "id": "0xFE41",
          "access": ["read","notify"], "type": "bytes", "len": 7 },
        { "name": "Param", "id": "0xFE43",
          "access": ["read","write","notify"], "type": "bytes", "len": 5,
          "on_write": "on_param_write" } ] }
  ]
}
```

`sig` vs `id` distinguishes SIG-adopted from custom. `on_write` names a C symbol
the app implements — structure is declared, behaviour stays escape-to-C.

**Validated:** a prototype generator emitted a `build_services()` that matches
Ring's hand-written one — same 7 services, same order, same 16 characteristics,
same pinned IDs. Ring's contract round-trips losslessly.

## What Tier A deliberately does NOT cover

Being explicit so nobody assumes otherwise:

- Packed field layouts and the version-byte variants (Tier B).
- The key-value **param** pattern (`write [id]` = query, `write [id, u32]` = set)
  with per-param ranges, defaults, units and ACTION guards. This is a
  *convention* the Ring invented on top of one characteristic. It may deserve
  first-class support — see open questions.
- Batched/variable-length payloads capped by the negotiated MTU.
- Runtime-derived advertised names (`"Ring-" + uid16`).

## Binding — how characteristics reach user code

The unsolved half. A contract that declares `Count` is useless until something
publishes it, and users arrive from three directions: existing code then enable
BLE, enable BLE then write code, or — typically — both, iterating.

**Studio already solves this shape for tile events.** `studio-shim.ts` emits a
user handler's declaration when one exists and a no-op stub when it does not, so
the dispatcher links either way. That is rendezvous-by-name, late-bound, and
proven in production. Generalise it rather than inventing something.

**Binding is a field on the characteristic**, edited in Studio's contract modal
as a column beside the characteristic — so wiring is visible where the thing is
defined, not a separate step to forget — and written directly by hand/agent
authors.

Outbound (`read` / `notify`) sources:

| `source` | Generates | For |
|---|---|---|
| `{ "var": "count" }` | publish on change, rate-limited | "expose this value" — the novice case |
| `{ "tile": "sense_t_c.read_temp" }` | poll at a declared rate, publish | "put my sensor on my phone" |
| `"code"` | a setter symbol, `ble_count_set(v)` | escape-to-C; what the Ring uses |

Inbound (`write`) is the existing event pattern exactly: a weak handler symbol,
surfaced in the DSL as `on Core.BLE.write("param")` or implemented in C as
`on_ble_param_write()`.

**The blend-and-mix case then falls out symmetrically.** Coming from existing
code, the source picker is populated from variables and tile readings already in
the project, so wiring is two clicks rather than a refactor. Coming from an empty
project, the characteristic sits unbound and Studio offers to scaffold a variable
for it. Neither direction is ever silently broken, because diagnostics close the
loop — **in coregen, so they fire for hand-authored projects too**:

- characteristic with no source → *"nothing publishes 'Count' — it will always read zero"*
- handler with no characteristic → *"'on_ble_foo_write' matches no characteristic"*

Same cross-check shape that already catches an orphaned `Core.Watchdog.feed()`.

**Generated output stays readable and diffable** — something close to the
`build_services()` a human would have written, not a macro thicket. Hand authors
read it to debug; agents read it to infer intent.

## Open design questions

1. ~~**Novice vs product author.**~~ **DECIDED 2026-08-25 — one contract.**
   Studio is "approachable but complete": comfortable for a novice, with
   everything an expert needs in the same object. Two surfaces are rejected —
   they recreate the mirror problem internally and would force a rewrite at
   exactly the wrong moment, when a hobby project gains deployed clients.

   **Rule: auto-assign once, then freeze.** An id is allocated when the
   characteristic is created and written into the contract immediately —
   persisted, never recomputed. The novice never types or sees it (Studio
   surfaces it read-only behind an advanced disclosure); it is nonetheless a
   pinned id from the first save. So the beginner path and the product path are
   the same file, and `core_ble_add_service()`'s renumbering scheme never
   enters the picture — the generator always emits `add_service_id()`.

   **Ids must stay DEFROSTABLE.** Not first-tier, not built now, but the design
   must not preclude it: a project may need to re-jigger ids to match a client
   it does not control. Two implications carried forward:
     - Ids are **stored data, never derived values.** Nothing may recompute an
       id from position, name, or hash — that is what would make unlocking a
       rewrite instead of an edit.
     - The contract records how each id was set (`auto` vs `explicit`) so a
       future bulk-renumber can tell assigned ids from deliberately chosen
       ones. Note this is a UX affordance, not a safety one: once a client is
       deployed, both kinds are equally load-bearing, so any unlock UI warns
       about deployed clients rather than about provenance.
2. ~~**Does the param pattern become first-class?**~~ **DECIDED 2026-08-25 — yes,
   but declare intent, not wire format.**

   Params are declared as a typed set (name, type, range, default, units, enum
   labels) and the GENERATOR chooses the encoding. **Actions** (Ring's `SHIP`,
   which is "do a thing behind a magic-value guard", not "set a value") are
   declared separately from params even when they share a characteristic — the
   Ring already flags the distinction in prose; we encode it.

   **Default: one real characteristic per knob. The packed key-value blob is an
   expert opt-in.** This inverts the first proposal, and the reason is worth
   keeping. Scanners cannot name what the device does not publish, and
   `ble_svc.c` was discarding every `name` (`(void)name;`), so a custom
   characteristic showed as a bare UUID — observed live in LightBlue against a
   Core.ST.W5. Fixed in tiles#211 by publishing a Characteristic User
   Description (0x2901); "Count" now appears where a UUID used to be.

   That fix *widens* the discoverability gap rather than closing it uniformly: N
   real characteristics now announce themselves by name in any generic app,
   while a param blob remains one opaque 5-byte characteristic whose twelve
   knobs are invisible. The novice — squinting at a UUID wondering where their
   battery went — is exactly who pays that cost, so the novice default must be
   real characteristics.

   The expert path stays justified: attribute space and discovery time are
   finite (the SDK reserves 8 characteristics per service), so a project with
   dozens of tunables and a client it controls should still be able to compact
   them. Because we declare intent rather than wire format, that is a lowering
   choice, not a schema change.

   **Hard limit to design around:** GATT has no service-name mechanism at all.
   0x2901 attaches to a characteristic; there is no service equivalent. A custom
   service will read as a raw UUID in every scanner, forever. Only SIG-assigned
   service UUIDs get resolved, and then only from the scanner's own table.
3. ~~**Where does the contract live?**~~ **DECIDED 2026-08-25 — inline is the
   wire form, an include is the authoring convenience.**

   The cloud path has no choice: `services/build/src/build.ts` materialises
   exactly `main.c`, `config.json`, `Makefile` (plus a `tile_handles.h` stub),
   taking `main_c` and `config_json` as named fields. There is no general file
   map, so a separate contract file would mean a build-service API change,
   re-vendor and Fly deploy before anything worked. Inline under `ble` works on
   the deployed service today.

   But a 368-line contract inlined into `config.json` makes that file ~90% BLE,
   and for an agent it means rewriting the whole blob to touch one
   characteristic — bad for surgical edits, bad for diffs. So:

   ```json
   "ble": { "enabled": true, "contract": "ring.ble.json" }
   ```

   is resolved by **coregen**, which is the right home because it runs in both
   the local and the cloud path. Studio always emits inline (it has no files to
   reference); hand and agent authors get a file they can edit surgically.
   Flattening an include for a cloud build is a one-command chore, not an
   architecture change.
4. ~~**Which client targets?**~~ **DECIDED 2026-08-25 — the targets need
   different tiers, which settles the ordering.**

   **Tier A → C + TypeScript.** C is target zero. TypeScript needs only Tier A
   and is worth more than "nice for the twin": Studio is a browser app already
   driving hardware over WebHID and WebSerial, so Web Bluetooth is the same class
   of capability, and a generated TS client powers an **in-Studio BLE
   inspector** — connect to your own board, see characteristics, poke values,
   watch notifications, without writing an app. Critically it shows *semantics*
   no scanner can: that `motion_hz` is 1-100, defaults to 50, is in Hz, that
   `IMU_MODE` reads `raw`/`quaternion`. Three consumers for one artifact: twin,
   inspector, user-built Web-Bluetooth clients.

   **Tier B → Swift.** The Ring's 699 Swift lines are mostly packed-layout
   decoders and version dispatch, not UUID tables — Tier A would remove maybe a
   third and leave the part that actually drifts. Biggest payoff, but it lands
   with Tier B. Risk to design around: the Ring app is real shipping SwiftUI
   with opinions about types and concurrency; generated code that doesn't feel
   native gets wrapped in adapters, adding a layer instead of removing one.
   Design the Swift output against that app's actual usage.

   Kotlin/Android: no evidence of need; add on request.

5. ~~**Tier B: adopt or invent?**~~ **PINNED 2026-08-25** — deferred with Tier B
   in favour of Tier A + the inspector. See `source/PINNED.md`.

6. ~~**Publish semantics.**~~ **DECIDED 2026-08-25 — on change, rate-limited,
   gated on connected AND subscribed.**

   The dominant issue is gating, not timing: the Ring audit's top battery
   finding is that everything free-runs "even when disconnected or unsubscribed",
   and a generated publisher that did the same would bake that bug into every
   novice project.

   - *on change* — a rarely-moving value generates no idle traffic
   - *rate-limited* — a minimum interval bounds anything fast; default 10 Hz,
     declared per characteristic
   - *gated* — nothing is published to nobody

   For a genuine 50 Hz stream "on change" is always true, so the rule collapses
   to fixed-rate at the limit — one rule covers both cases without the author
   picking a mode. Escape: `"publish": "on_change" | "always" | { "hz": N }`.

   Not solved now, both pinned: **batching** (Ring R11 — decoupling sample rate
   from packet rate against Apple's ~31 Hz single-peripheral ceiling) and
   **runtime-adjustable rates**, though the latter composes for free once params
   exist, since a rate can itself be a param.
5. **Tier B: adopt or invent?** Kaitai's model is right but its C/encode gaps are
   disqualifying. A narrow layout IDL of our own is probably smaller than
   fighting that — but this is the decision to make deliberately.

## Sequencing

| Phase | Work | Acceptance |
|---|---|---|
| 0 | This note + schema draft | Ring round-trips (**done**) |
| 1a | coregen emits builder + contract header; every characteristic `source: "code"` | Ring builds from generated code, byte-identical GATT (**done**) |
| 1b | binding (`var` / `tile`) + coregen diagnostics | a bound value reaches a phone with no hand-written publish (**done for `var`**, see below) |
| 2 | Swift client generation | Hand-written mirror shrinks measurably |
| 3 | DSL hosts + events, name-addressed | `on Core.BLE.write("param")`, via the Core.Pad event pipeline (**done**) |
| 4 | Twin model | DSL testable without hardware |

Phase 3 landed ahead of 1b, so for a while a contract could be read and written
from the DSL while still needing a hand-written publish.

## 1b as built

`source` and `publish` are fields on the characteristic, parsed by coregen, and
`tests/ble-contract` exercises both against every characteristic shape.

```json
{ "name": "Battery Level", "sig": "0x2A19", "access": ["read", "notify"],
  "type": "uint8", "source": { "var": "battery_pct" }, "publish": { "hz": 1 } }
```

- `"source": "code"` or no `source` emits a setter and nothing else, so every
  contract written before this keeps behaving exactly as it did.
- `{ "var": "name" }` generates `ble_contract_publish()`, which `core_ble_process()`
  calls through a weak symbol. Nothing is added to the application loop.
- `{ "tile": "..." }` is **not implemented**. coregen rejects it with a message
  pointing at the workaround, which is to read the tile into a variable and bind
  that. It is sugar over `{var}`, not a missing capability.

The bound variable is read from the generated translation unit, so it needs
external linkage: a file-scope `int count;` links and a `static int count;` does
not. Studio must emit bound variables without `static`; its DSL codegen makes
every global `static` today, which is the remaining work on the Studio side.

**Publishing is gated, per the decision above.** `core_ble_subscribed()` was
added to the SDK for it: `ble_svc.c` now tracks each characteristic's CCCD from
the attribute-modified event and clears it on disconnect. Before this there was
no way to ask whether anyone was listening.

Both diagnostics run in coregen, so hand-authored projects get them:

- a readable characteristic with no `source` at all warns that nothing publishes
  it. An explicit `"code"` does not warn: the author said they would publish it.
- a `ble_*_on_write` in main.c matching no characteristic warns that it will
  never be called. A text scan of main.c, not a parse.

Coregen warnings now go to stderr. They went to stdout, which a normal build
sends to `/dev/null`, so no warning it emitted had ever been visible.

Phase 1's acceptance test is Ring_Av2. If the schema cannot express Ring's
contract exactly, it is not ready.

## Separately: the SDK's own BLE gaps

From `Ring_Av2/SDK_FIRMWARE_AUDIT.md` — a different programme, worth not
conflating with the Studio-facing work:

- `hal_fault_set_callback()` never registered — the key to the flagless reset.
- `core_ble_set_adv_interval()` never called; wearable standby wastes power.
- `core_ble_on_connect/on_disconnect` unused; link state is polled instead.
- **Strategic:** a BLE-aware Stop2 low-power manager. The radio holds an ~11 mA
  floor, and `core_power`'s Stop is documented broken on Core.ST.W5.
