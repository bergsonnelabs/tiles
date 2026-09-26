# Core.ST.L4 soft-brick recovery — watchdog + strike-counter → ROM DFU

**Status:** proposed (design) · **Scope:** Core.ST.L4 first, Core.ST.H5 fast-follow
**Owner:** (firmware) · **Depends on:** existing `core_watchdog`, `hal_dfu`, `core_init` codegen

## Problem

New users flash bad app code to the L4 that hangs *immediately on boot* — a
blocking `i2c` init with no timeout (cf. the STC31-C bus lockup), an infinite
loop before USB comes up, a clock misconfig. The board then:

- doesn't enumerate USB (the app hung before/at CDC init), and
- doesn't respond to the 1200-baud touch (there's no live CDC to touch).

It's soft-bricked: the only recovery is BOOT0-pin gymnastics or an SWD probe,
which a hackathon student doesn't have. We want the board to **rescue itself**.

## Principle: the recovery agent must live where a bad app can never erase it

A watchdog *alone does not un-brick* — a 5 s IWDG reset just drops you back into
the same immediate hang, and USB still never stays up. Recovery requires
something that runs **before the app on every boot** and can decide "don't
launch it again."

The custom bootloader (`sdk/bootloader/`, `1209:0002`, app @ `0x08002000`) is one
such place, but it has a fatal footgun: it lives in the first 8 KB at
`0x08000000`, so anyone who reasonably flashes via the ST toolchain
(CubeProgrammer / ST-Link at `0x08000000`) **wipes the recovery agent itself**.

So the recovery agent is the **ST ROM bootloader** in system memory
(`0x1FFF0000`): unerasable by any app flash, entered by a software jump we
already have (`hal_dfu_jump_to_rom()`, `sdk/hal/hal_dfu.h:86`). The app sits at
`0x08000000` (standard), so ST-toolchain flashing "just works" and can never
destroy the recovery path. Bonus: a rescued board lands in `0483:df11`, which is
exactly the DFU device Studio's bring-up wizard already handles.

**This means the L4 default project config becomes `"bootloader": "rom"`**
(`ROM_DFU=1`, app @ `0x08000000`), not `custom`.

## Part A — watchdog on by default

- Wire the **currently no-op** `iwdg` config section (`config-json.md:290`) into
  real codegen. Emit `core_watchdog_start(5000)` in `core_init()` after
  `core_clock_init()`/`core_pads_init()` (`core_init.c.j2:509-510`).
- Default **on for L4/H5**, **off for L0** — an always-on IWDG runs through
  Stop/Standby and can't be paused (`ll_pwr.h:10`), which fights the
  low-power core's entire purpose.
- Auto-start (chosen): `core_init()` starts the dog whenever `iwdg` is enabled,
  and the **starter `main.c` template feeds it** in the loop. Even minimal user
  code is protected; un-fed code falls into Part B's recovery. Teaches the
  feed-the-dog pattern.
- **Default polarity: OFF / opt-in** (chosen). Absent `iwdg` config → off, so
  regenerating an existing project never surprises it with resets. "On by
  default" is delivered by the **Studio starter scaffolding** writing
  `iwdg.enabled=true` and emitting the feed — so it's on for *new* projects
  without retroactively changing old ones.
- **Starter must leave PA13/PA14 (SWDIO/SWCLK) alone.** Rev b breaks out SWD on
  those pins; an app that reconfigures them as GPIO loses SWD access at runtime
  (only recovered by power-cycle or the DFU rescue). The starter template should
  not touch them, and it's worth a one-line comment saying so.
- Add **DBGMCU freeze-on-halt for the IWDG — firmware-side**, in `core_init`
  when the watchdog is enabled (`DBGMCU->APB1FZR1 |= DBG_IWDG_STOP`). This is
  **mandatory now that rev b exposes SWD**: people will attach probes to the L4,
  and a default-on 5 s IWDG resets the chip out from under any halt >5 s.
  Firmware-side (not just the OpenOCD `.cfg`) so it holds for *any* probe /
  toolchain — CubeIDE, a raw CMSIS-DAP, the CoreProbe — not only our config.
  (Today `sdk/debug/stm32l4.cfg` has no freeze; H5/WBA have debugger-side only.)
- Timeout stays configurable via the `iwdg` knob; 5 s is the beginner-generous
  default (range ~100 ms–28 s, `core_watchdog.h:52`).

## Part B — strike counter → SOS → ROM DFU (in `core_init`)

Extend the existing `#ifdef ROM_DFU` early hook in `core_init.c.j2:493-502`
(runs pre-clock-init, before any user code) to, in order:

1. **Read the reset cause.** `IWDGRSTF` (RCC_CSR bit 29) via
   `ll_iwdg_caused_reset()` (`ll_iwdg.h:145`). Then **clear the flags (RMVF)**
   (`ll_rcc_clear_reset_flags`, `ll_iwdg.h:162`) — else the bit latches and every
   later reset miscounts as a watchdog reset.
2. **Update the strike counter** in reserved SRAM (see below):
   - watchdog reset → `strikes++`
   - any other cause (power-on, pin, software/DFU-download reset) → `strikes = 0`
3. **Honor the existing DFU magic** (`hal_dfu_reboot()` path) → `hal_dfu_jump_to_rom()`.
4. **If `strikes >= N` (N ≈ 3):** don't launch the app → blink **SOS** on the
   status LED → `hal_dfu_jump_to_rom()`. ~15 s from first bad flash to recovery.

App side (in `core_watchdog_feed` wrapper or a `core_init` late hook): once the
app has fed the dog for ~2× the timeout of *healthy* operation, **clear the
strike counter**. So a transient hang that recovers never accrues toward a
brick — **only immediate-boot-hang code accumulates strikes**.

### Boot flow

```
reset ─▶ core_init (pre-clock hook)
          read IWDGRSTF; clear RMVF
          IWDG reset?  ── yes ─▶ strikes++
                       ── no  ─▶ strikes = 0
          DFU magic set?      ─▶ jump_to_rom (0483:df11)
          strikes >= N ?      ─▶ SOS blink ─▶ jump_to_rom (0483:df11)
          else ─▶ clock/pads init ─▶ core_watchdog_start(5000) ─▶ user main
                                                                     │
                        healthy uptime ≥ 2× timeout ─▶ strikes = 0 ◀─┘
```

### Persistent strike counter — reserved SRAM (no linker change)

The DFU magic already carves out 16 bytes at top-of-RAM
(`0x20009FF0` on L422, `hal_dfu.h:33`); the magic uses word 0. Use:

| addr | field |
|---|---|
| `0x20009FF0` | DFU jump magic (existing, `0xDEADBEEF`) |
| `0x20009FF4` | strike counter |
| `0x20009FF8` | validity tag (e.g. `0xB0BB1E`) |

Startup `.data`/`.bss` init doesn't touch the reserved region, so words 1–2
survive a warm/IWDG reset. On **power-on** SRAM is random → the validity tag
won't match → strikes read as 0. Net: **power-cycle = fresh start** — an
intuitive "unplug to recover" story for students. (RTC backup regs also work and
survive power-cycle with a coin cell, `core_backup.h` — but reserved SRAM is the
friendlier behavior here and needs no RTC clock enable in the pre-clock hook.)

### SOS blink

Before clock init the MCU runs on default MSI (~4 MHz). Enable the status-LED
port clock (RCC_AHB2ENR), drive the pad, busy-loop delays (no SysTick yet):
`· · ·  — — —  · · ·`, then jump. Status-LED pad TBD from the Core.ST.L4
definition/pads.

## Studio / pipeline implications

- **DFU identity unifies on `0483:df11`.** A rescued or 1200-touched L4 in `rom`
  mode enumerates as the ST ROM DfuSe device — the same VID/PID Studio's
  `requestDfuDevice` / `findPairedDfuDevice` already filter to, and the same one
  the bring-up wizard's blank-Core branch drives. No web changes required.
- A successful DFU download resets the MCU (non-IWDG cause) → strikes clear on
  the next boot. Fresh good code feeds the dog → stays healthy → strikes stay 0.
- Existing boards flashed in `custom` mode still work (Studio handles both
  `1209:0002` and `0483:df11`); this changes only the L4 *default* going forward.

## Implementation status (2026-07-13)

Built on branch `studio/l4-brick-recovery` (SDK + codegen only):
`hal_dfu.h` strike storage, new `core_recovery.h`, `core_watchdog.h`
(stash-preserving `caused_reset` + `debug_freeze`), `core_init.c.j2` (recovery
hook + SOS + watchdog auto-start), `coregen.py` (`iwdg` config, default off).

- **Verified (compile + on hardware):** compiles clean for L4 (`-Wall -Wextra`),
  full link OK. **Bench-validated end-to-end on the CoreProbe (Core.ST.L4 rev b, not the L4.2)**
  via `tests/hw-watchdog-recovery/`: healthy feed → hang → strike cascade
  (1→2→3, counter survives resets) → one rapid SOS on PA8 → ROM DFU (`0483:df11`)
  → reflash. `caused_reset()` reports WATCHDOG correctly after the early RMVF clear.
- **Pre-existing blocker (not this change):** `sdk/hal/hal_sai.c` uses H5-only
  GPDMA registers and is compiled unconditionally, so a *full link* fails for
  every L4 project on `main`. Independent of this work — worth a separate fix.
- **Not done — needed before it's real:** (1) the **starter feed** + healthy-uptime
  `core_recovery_clear()` (likely a Studio/web change); (2) making `rom` the L4
  default (3-place bootloader default + Studio scaffolding + custom→rom migration);
  (3) **on-hardware verification** of the hang→reset→strike→SOS→DFU loop and the
  pre-clock SOS timing/LED pad.

## Scope / phasing

1. **L4 only.** Flip the L4 default project config to `bootloader: "rom"`; wire
   Part A codegen + starter feed + DBGMCU freeze; add Part B to the `ROM_DFU`
   hook. Hardware-verify the full loop: flash a deliberately-hanging app → 3×
   watchdog reset → SOS → lands in `0483:df11` → Studio reflashes clean.
2. **H5 fast-follow.** Same pattern; H5 already has DBGMCU freeze and a ROM DFU
   addr (`0x0BF97000`).
3. L0 excluded (low-power); W5 excluded (no ROM-DFU jump path verified here).

## Open questions / to confirm

- **Where is the per-core default bootloader mode set?** (coregen per-core
  defaults vs Studio build-config) — that's the one line that makes `rom` the L4
  default.
- **Status-LED pad** for the pre-clock SOS blink (from the L4 definition).
- **N and the healthy-uptime threshold** — start N=3, healthy=2× timeout; tune on
  the bench.
- Should Part A also expose the watchdog in the Tiletown configurator's existing
  `snippetWatchdog` (priority 20/90) for consistency? (out of scope for cut 1)
- Fix the blocking calls that have no timeout (the `ll_i2c_probe` STOPF-wait HAL
  bug) in parallel — the watchdog is a safety net, not a substitute.

## SWD interplay (rev b) — mostly synergy

Rev b exposes SWD (PA13/PA14, plus **NRST as an optional pad**), which reinforces
this design rather than fighting it:

- **Second, unerasable recovery path.** A probe on SWD can always halt/erase/flash
  regardless of app state — belt (watchdog→DFU, for the no-probe student) and
  suspenders (SWD, for whoever has a CoreProbe).
- **Strike-counter helps probe attach.** Parking in ROM DFU after N strikes
  quiesces the hang→reset loop, so a probe attaches to an idle core instead of a
  moving target. NRST (optional) enables connect-under-reset if needed before it
  parks.
- **ROM DFU preserves SWD.** The recovery jumps to ROM, which leaves PA13/PA14
  alone — so the rescue restores SWD access even if a bad app had remapped them.
- **DBGMCU freeze becomes mandatory** (see Part A) — the one hard requirement SWD
  adds.

**Separate but related workstream — Studio "SWD for all Core.ST.x":** SWD should
be selectable for every Core.ST core (L0 = *only* way in, Core.ST.L4.2 always,
Core.ST.L4 rev b new (the board called Core.ST.L4.1 before the 2026-09 rename,
not the L4.2), H5 already, W5 done).

*Transport: done.* Studio speaks two SWD probes — ST-Link (`lib/stlink/probe.ts`)
and the in-house **CoreProbe** over WebHID CMSIS-DAP v1 (`lib/cmsisdap/coreprobe.ts`,
plus vendor commands 0x80+ for target power / `V_shift`). `TransportKind` covers
`usb-dfu | st-link | cmsis-dap`, and the FlashPanel exposes a USB-DFU/SWD toggle.
Neither is gated to `Core-ST-W5-b` any more.

*Flash drivers: still one.* `lib/stlink/flash-wba.ts` (shared by both probes) is
the only family driver, so SWD flashing still reaches **WBA only**. Un-gating the
rest needs **per-family flash drivers keyed on DBGMCU DEV_ID** — per Studio's own
table in `lib/coreId.ts`: L4/L422 `0x464` (algorithm already in
`sdk/bootloader/main.c`), H5/H523 `0x478` (quad-word), L0/L011 `0x457`. With no L0
driver and no L0 USB-DFU, **Studio can't flash an L0 at all today.** Tracked
separately from this firmware doc.
