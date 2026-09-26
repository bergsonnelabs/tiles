# hw-pin-wake

Bench test for pin-change wake from Stop (`core_stop_until_on_change`,
`sdk/core/core_power.h`) on **Core.ST.L4**.

| Test | What passes |
|------|-------------|
| T1 pad loopback     | pad 4 (PB6) reads high after `core_init()` (coregen drives PC15 for `"pullups": ["pad4"]`), then follows PC15 high/low, in run mode |
| T2 no-GPIO fast-fail | `core_stop_until_on_change(1, EDGE_FALLING)` (pad 1 = GND, no GPIO) returns in under 100 ms instead of sleeping |
| T3 wake from Stop   | a human shorts pad 4 to pad 1 (GND); the Core actually wakes from Stop, having slept more than one watchdog chunk (2.5 s), with no watchdog reset. Unattended, the 60 s timeout ends it and T3 reports FAIL ("no touch") |
| T4 already at level | PC15 low holds pad 4 low; a falling-edge wait returns 1 in under 100 ms without entering Stop, with `core_stop_until_on_change_timeout()` and then the untimed `core_stop_until_on_change()`. Skipped (FAIL) when T1 failed, since without the loopback the pad isn't low |
| T5 timed wait       | pad 4 held high; `core_stop_until_on_change_timeout(4, EDGE_FALLING, 3000)` returns 0 after 2.5-4 s of Stop, no watchdog reset |

## Unattended run

`make EXTRA_CFLAGS=-DPIN_WAKE_NO_T3 && make EXTRA_CFLAGS=-DPIN_WAKE_NO_T3 flash-serial`
runs only the automated tests (T1, T2, T4, T5) and reports T3 as skipped.
Every wait is bounded: T4's untimed call runs only after the timed one
returned at once, and T5 is a 3 s timed wait.

## Hardware

- **Needs a production Core.ST.L4 (rev b) with the pull-ups fitted.** Early
  prototype L4 boards have no 2.2 kOhm pull-ups on pads 4/5; there T1 fails
  (the pads follow only the internal pulls), T4 is skipped, and that is the
  board, not the SDK (seen 2026-09-25).
- Pad 4 (PB6) has a 2.2 kOhm on-tile pull-up switched by chip pin **PC15**
  (`definitions/Core-ST-L4-b.json`, `config.pullups`: pad 4 via PC15, pad 5
  via PA9). Driving PC15 high/low pulls pad 4 high/low. `config.json` turns
  it on through coregen (`"pullups": ["pad4"]`). Until 2026-09-25 the test
  drove PA9, which is pad 5's.
- Pad 1 is GND, pad 10 is V+. The onboard LED is PA8, active-high.
- Nothing that runs in Stop can drive PC15, so a real wake-from-Stop edge needs
  a human: briefly touch pad 4 to pad 1 (GND) with tweezers while PC15 holds
  the pull-up (~1.5 mA through the 2.2k — safe).

## About T2 and T4 — the "already at the wake level" case

EXTI hardware only latches a transition that happens *while the trigger is
enabled*, and every `core_stop_until_on_change` call arms the EXTI afresh. Until
2026-09-25 that meant a pad already sitting at its wake level (a sensor INT held
low until serviced) never woke the Core, and with no timeout the call slept in
fed 2.5 s chunks forever. Since then the call reads the pad right after arming
and returns at once if it is already low (`EDGE_FALLING`) or high
(`EDGE_RISING`); `EDGE_BOTH` always waits for a change. T4 covers that, and
`core_stop_until_on_change_timeout(pad, edge, timeout_ms)` (0 = no timeout;
returns 1 woken, 0 timed out, -1 no EXTI on that pad) bounds a wait, which T5
covers.

T2 checks the other promise in the same doc comment: "returns right away if
the pad can't take an edge interrupt." Pad 1 (GND) has no GPIO, so
`core_pad_on_change` fails immediately and the call returns without touching
Stop, checked against a 100 ms bound.

## T3 — the real wake-from-Stop proof

The "within 30 s" prompt is a *get-ready* cue: the Core drives PC15 high
(pull-up on), prints the prompt, blinks the LED fast for ~30 s while feeding
the watchdog, then calls `core_stop_until_on_change_timeout(4, EDGE_FALLING,
60000)`. It waits up to 60 s for the edge; with nobody at the bench it times
out and T3 is reported as FAIL ("no touch"), so an unattended run finishes.

**What to do, and when:** wait for the fast LED blink and the "touch pad 4 to
GND ... within 30 s" line on the USB CDC console, then, any time after the
blinking stops and the "sleeping now" line prints, briefly touch pad 4 to pad
1 (GND) with tweezers. Waiting a couple of seconds after "sleeping now"
before touching gives the clearest pass (see below); touching immediately
still wakes the Core, just too fast to prove the chunked-sleep path, so that
attempt is retried automatically.

Pass bar: slept longer than one watchdog chunk (2.5 s, at the 5 s IWDG
timeout this test configures) with no watchdog reset. A touch inside the
first chunk is scored "too fast" and retried, up to 5 attempts, so a human
who is consistently faster than 2.5 s (or who never touches the pad at all)
can't spin the test forever — it gives up and reports FAIL after the 5th.

Like `hw-sleep-cycle` (whose USB-across-Stop workaround predates the
2026-09-26 USB fixes), progress is kept in backup registers
and the Core does its own software reset after every Stop attempt to get a
clean re-enumeration before printing again. `core_watchdog_caused_reset()` is
checked on every boot; a real watchdog reset (as opposed to the deliberate
software reset) marks T3 FAIL and is called out explicitly.

## Build and flash

```sh
make distclean && make      # Core.ST.L4 only — config.json pins the core
make flash-dfu               # or: make flash-serial
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

## Reading the result

USB CDC, 115200, any terminal. T1, T2 and T4 print immediately (after a 1.5 s
settle so the CDC enumeration window doesn't eat the first lines). T5 then
sleeps 3 s and resets; T3 prints its prompt, then goes quiet while asleep; the
full report appears after the post-Stop reset and reprints every 3 s:

```
[hw-pin-wake] PASS  (pass=0x1f fail=0x00 expected=0x1f)
  T1 pad loopback        PASS
  T2 no-GPIO fast-fail   PASS
  T3 wake from Stop      PASS
  T4 already at level    PASS
  T5 timed wait          PASS
  T3: slept 4123 ms before wake (need >2500 ms, one watchdog chunk), watchdog reset seen: no, attempts used: 2
  T5: returned 0 after 3004 ms (want 0 after 2500-4000 ms)
```

**LED:** fast blink during the T3 countdown. Once the final report is up: slow
blink (1 s on/off) for PASS, or N quick blinks + a 1.5 s pause for FAIL (N =
first failing test, 1-5).

**SWD:** `g_pin_wake` mirrors the report in RAM once `magic` reads
`0x51EEC7C2`:

```sh
arm-none-eabi-nm build/hw-pin-wake.elf | grep g_pin_wake
```

Layout: `magic, verdict, pass, fail, expected, phase, attempt, wd_seen,
slept_ms, chunk_ms`. `verdict` is `0x900DBEEF` for PASS, or `0xBAD000nn` for
FAIL (nn = index of the first failing test). Backup registers 0-6 also carry
the same story and survive resets/power cycles for later inspection
(`arm-none-eabi-nm`/`probe-rs` per the `hw-sleep-cycle` README for the
TAMP_BKPxR address on this Core).

## What a failure points at

- T1: the PC15 -> 2.2k -> pad 4 pull-up, or the pad-4 GPIO input config. T4
  depends on it and is skipped when it fails. Bench, 2026-09-25 (board with
  USB serial 005D002A343050102039324B): T1 FAIL. PC15 and PA9 both toggle
  (IDR read back), but pads 4 and 5 follow only the MCU's internal ~40k pull,
  up or down, whatever PC15/PA9 drive: no 2.2k reaches PB6/PB7 on that board
  (resistors not fitted, or the board doesn't match the definition). The
  earlier "pad 4 low with PA9 high and low" was the same symptom.
- T4: the level check after arming in `core_stop_until_on_change_timeout`.
- T5: the timeout path (RTC-measured, chunked with the watchdog).
- T2: `core_pad_on_change`/`hal_pad_lookup` no longer failing fast for a
  no-GPIO pad — check `hal_pad_lookup(1)` still returns a NULL port.
- T3 "reset while waiting" at boot: something reset the Core mid-wait. A real
  watchdog reset means the chunked feed-and-resleep loop in
  `core_stop_until_on_change` broke; a non-watchdog reset usually just means
  the board was power-cycled or reflashed mid-test.
- T3 "giving up after N attempts": the edge is consistently caught inside the
  first 2.5 s chunk. Either the human is touching too fast (wait a beat after
  "sleeping now"), or the RTC-armed wakeup chunking is shorter than it should
  be (compare `chunk_ms` in the report against half the configured IWDG
  timeout).
