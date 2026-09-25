# hw-pin-wake

Bench test for pin-change wake from Stop (`core_stop_until_on_change`,
`sdk/core/core_power.h`) on **Core.ST.L4.1**.

| Test | What passes |
|------|-------------|
| T1 pad loopback     | drive PA9 high/low, pad 4 (PB6) reads back high/low, in run mode |
| T2 no-GPIO fast-fail | `core_stop_until_on_change(1, EDGE_FALLING)` (pad 1 = GND, no GPIO) returns in under 100 ms instead of sleeping |
| T3 wake from Stop   | a human shorts pad 4 to pad 1 (GND); the Core actually wakes from Stop, having slept more than one watchdog chunk (2.5 s), with no watchdog reset |

## Hardware

- Pad 4 (PB6) is wired through a 2.2 kOhm on-tile resistor to chip pin PA9.
  Driving PA9 high/low pulls pad 4 high/low.
- Pad 1 is GND, pad 10 is V+. The onboard LED is PA8, active-high.
- Nothing that runs in Stop can drive PA9, so a real wake-from-Stop edge needs
  a human: briefly touch pad 4 to pad 1 (GND) with tweezers while PA9 holds
  the pull-up (~1.5 mA through the 2.2k — safe).

## About T2 — an API limitation, not just a test

The original plan for T2 was "arm the wake, create the edge just before
calling `core_stop_until_on_change`, check it returns promptly instead of
sleeping or missing the edge" — i.e. an edge that's already pending when the
call is made. Reading `core_power.h` shows that isn't a case the API catches:

```c
_core_stop_pad_edge = 0;
if (core_pad_on_change(pad, edge, _core_stop_pad_cb, (void *)0) != HAL_OK)
    return;
```

Every call to `core_stop_until_on_change` unconditionally clears the pending
flag and freshly re-arms the EXTI trigger (SYSCFG mux + FTSR/RTSR + IMR, in
`hal_exti.c` / `ll_exti`). Real EXTI hardware only latches a transition that
happens *while the trigger is enabled* — it can't retroactively notice that a
pin reached its target level a few instructions earlier. So driving PA9 low
and immediately calling `core_stop_until_on_change(4, EDGE_FALLING)` does
*not* return promptly: pad 4 is already settled low by the time the
falling-edge trigger arms, no new transition ever occurs, and the call sleeps
in fed 2.5 s chunks forever — there's no timeout parameter, and nothing else
on this single-core chip can toggle PA9 while it's asleep in Stop to give it
back. Proving that live would hang the test binary permanently (the only way
out is a human touching the pad — which is what T3 already covers), so it
isn't attempted here.

T2 instead checks the part of the same doc comment that *is* automatable and
bounded: "returns right away if the pad can't take an edge interrupt." Pad 1
(GND) has no GPIO, so `core_pad_on_change` fails immediately and
`core_stop_until_on_change` returns without ever touching Stop — checked
against a 100 ms bound. The pending-edge finding above is logged to the
console by T2 as a note, and is not scored.

## T3 — the real wake-from-Stop proof

`core_stop_until_on_change` has no timeout of its own (see above), so the
"within 30 s" prompt is a *get-ready* cue, not a firmware deadline: the Core
drives PA9 high (pull-up on), prints the prompt, blinks the LED fast for
~30 s while feeding the watchdog, then commits to the blocking call. Once
inside it, the Core waits for a real edge no matter how long that takes.

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

USB CDC doesn't reliably survive Stop on the L4 (known issue — see
`hw-sleep-cycle`), so, like that test, progress is kept in backup registers
and the Core does its own software reset after every Stop attempt to get a
clean re-enumeration before printing again. `core_watchdog_caused_reset()` is
checked on every boot; a real watchdog reset (as opposed to the deliberate
software reset) marks T3 FAIL and is called out explicitly.

## Build and flash

```sh
make distclean && make      # Core.ST.L4.1 only — config.json pins the core
make flash-dfu               # or: make flash-serial
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

## Reading the result

USB CDC, 115200, any terminal. T1 and T2 print immediately (after a 1.5 s
settle so the CDC enumeration window doesn't eat the first lines). T3 prints
its prompt, then goes quiet while asleep; the full report appears after the
post-Stop reset and reprints every 3 s from then on:

```
[hw-pin-wake] PASS  (pass=0x07 fail=0x00 expected=0x07)
  T1 pad loopback        PASS
  T2 no-GPIO fast-fail   PASS
  T3 wake from Stop      PASS
  T3: slept 4123 ms before wake (need >2500 ms, one watchdog chunk), watchdog reset seen: no, attempts used: 2
```

**LED:** fast blink during the T3 countdown. Once the final report is up: slow
blink (1 s on/off) for PASS, or N quick blinks + a 1.5 s pause for FAIL (N =
first failing test, 1-3).

**SWD:** `g_pin_wake` mirrors the report in RAM once `magic` reads
`0x51EEC7C2`:

```sh
arm-none-eabi-nm build/hw-pin-wake.elf | grep g_pin_wake
```

Layout: `magic, verdict, pass, fail, expected, phase, attempt, wd_seen,
slept_ms, chunk_ms`. `verdict` is `0x900DBEEF` for PASS, or `0xBAD000nn` for
FAIL (nn = index of the first failing test). Backup registers 0-5 also carry
the same story and survive resets/power cycles for later inspection
(`arm-none-eabi-nm`/`probe-rs` per the `hw-sleep-cycle` README for the
TAMP_BKPxR address on this Core).

## What a failure points at

- T1: the PA9<->pad 4 loopback wiring, or the pad-4 GPIO input config.
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
