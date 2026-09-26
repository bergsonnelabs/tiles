# hw-sleep-cycle

Bench test for the 2026-09-25 sleep / RTC / backup-register / watchdog fixes
on **Core.ST.L0.1**, **Core.ST.L4.1 / L4.2** and **Core.ST.W5**. It runs
unattended in about 30 s, with the 5 s watchdog armed by `core_init()` exactly
as in a Studio project.

| Test | What passes |
|------|-------------|
| T1 backup   | two backup registers read back what was written |
| T2 alarm    | `RTC_ALRMAR` reads back `0x80808003`, and Alarm A fires 1.5-4.5 s after it is set for xx:xx:03 |
| T3 stop #1  | `core_stop_for(10)` returns with the watchdog running, no watchdog reset, 9-12 s by the RTC |
| T4 stop #2  | the same again |
| T5 guard    | `core_standby_for(10)` returns `HAL_ERROR` (a 10 s Standby would outlast the 5 s watchdog) |
| T6 standby  | L4 only: `core_standby_for(2)` resets into a boot where `core_woke_from_standby()` is 1 |
| T7 clear    | L4 only: after `core_clear_standby_flag()` it reads 0 |
| T8 watchdog | `core_init()` armed the watchdog (5 s, so sleeps run in 2.5 s chunks) |

Progress is kept in backup registers 0-4 (the L0 has only five), so a
watchdog reset in the middle of a sleep shows up as that test failing rather
than as a silent restart. Resetting the Core after the result is shown runs
the test again.

## Build and flash

`make distclean` when switching Cores: `make clean` keeps `coregen/`, which
would carry the previous Core's generated init.

```sh
make distclean && make                                   # Core.ST.L4.1
make flash-dfu            # or: make flash-serial

make distclean && make TILE=Core.ST.L0.1
make TILE=Core.ST.L0.1 flash-coreprobe                   # SWD

make distclean && make TILE=Core.ST.W5
make TILE=Core.ST.W5 flash-coreprobe                     # or: flash (CubeProgrammer)
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

Stop and Standby drop an attached SWD session. Let the test run with the probe
idle, and attach only to read the result once the LED shows it.

## Reading the result

**L4:** a report over USB CDC every 3 s (115200, any terminal). It is printed
on a fresh boot after the run. That once worked around USB not surviving
Stop; since 2026-09-26 the SDK keeps USB up (it waits in Sleep while a host is
awake), and the fresh-boot report is kept because Standby resets anyway:

```
[hw-sleep-cycle] PASS  (pass=0xff fail=0x00 expected=0xff)
  T1 backup regs     PASS
  ...
  stop #1 10 s, stop #2 10 s (RTC); alarm after 3002 ms; watchdog resets mid-test: 0
```

**LED (all Cores):** on during each 10 s sleep. Then:

- PASS: slow blink, 1 s on / 1 s off.
- FAIL: N quick blinks, a 1.5 s pause, repeat. N is the first failing test (1 = T1 ... 8 = T8).

**SWD (L0 and W5; works on the L4 too):** read the five backup registers.
They survive resets, so they can be read at any time after the run.

| Core | BKP0 address | probe-rs |
|------|--------------|----------|
| Core.ST.L0.1 | `0x40002850` (RTC_BKP0R, RM0377 §22.7.20) | `probe-rs read --chip STM32L011K4 b32 0x40002850 5` |
| Core.ST.L4.x | `0x40003500` (TAMP_BKP0R, RM0394 §36.6.8) | `probe-rs read --chip STM32L422KB b32 0x40003500 5` |
| Core.ST.W5   | `0x46007D00` (TAMP_BKP0R, RM0493 §37.6.18) | `probe-rs read --chip STM32WBA55CG b32 0x46007D00 5` |

| Register | Meaning | PASS looks like |
|----------|---------|-----------------|
| BKP0 | `0x5C << 24` \| phase (6 = done) | `0x5C000006` |
| BKP1 | fail mask << 16 \| pass mask | `0x000000FF` on the L4, `0x0000009F` on the L0 / W5 |
| BKP2 | watchdog resets mid-test << 16 \| stop #2 s << 8 \| stop #1 s | `0x00000A0A` (each byte 9-12) |
| BKP3 | verdict | `0x600D600D`; `0xBAD000nn` = FAIL, nn = first failing test |
| BKP4 | T2 alarm latency, ms | about 3000 (`0x0BB8`) |

A RAM copy with a few more details (millis across each sleep, the raw
`RTC_ALRMAR`) is at the `g_sleep_cycle` symbol; `magic` reads `0x51EEC7C1`
once the report is up:

```sh
arm-none-eabi-nm build/hw-sleep-cycle.elf | grep g_sleep_cycle
probe-rs read --chip <chip> b32 <address> 14
```

Layout: magic, verdict, pass, fail, expected, phase, wd_resets,
stop_s[2], stop_ms[2], alarm_ms, alrmar.

## What a failure points at

- T1: the backup-register map (L4: TAMP at 0x40003400, W5: TAMP bus clock).
  With T1 failing nothing can be carried across a reset, so the run stops after T2.
- T2: `RTC_ALRMAR` wrong = alarm register offset; right but no fire = RTC not
  ticking (LSI / RTCSEL / RTC bus clock).
- T3/T4 with BKP2 bits 16+ non-zero: the watchdog fired inside `core_stop_for`
  (chunking). Elapsed out of 9-12 s: wakeup timer clock or flag handling.
- T5: the Standby watchdog guard.
- T6/T7: PWR_SCR (L4, 0x18) and the Standby flag.
