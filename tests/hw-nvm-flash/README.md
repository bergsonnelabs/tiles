# hw-nvm-flash

Bench test for `core_nvm`'s flash emulation (`sdk/core/core_nvm.c`) on
**Core.ST.L4 / L4.2**, **Core.ST.W5** and **Core.ST.H5** (where the store is
the flash high-cycle data area). It runs unattended in about a minute, with
the 5 s watchdog armed by `core_init()` as in a Studio project.
`host_test.py` adds the reflash test (T4).

| Test | What passes |
|------|-------------|
| T1 basic      | `core_nvm_erase_all()`, every byte reads 0xFF, out-of-range calls return `CORE_NVM_ERR_RANGE`, a 64-byte pattern and a single byte read back |
| T2 compact    | three pages' worth of 8-byte writes to one offset: at least two compactions (seen as slow writes), the last value and the T1 pattern read back |
| T3 reset      | a boot counter in NVM reads back after a software reset |
| T4 reflash    | (`host_test.py`) a token, the pattern, the counter and a 600-byte region survive flashing another image and then this one again |
| T5 radio      | W5 BLE build only: ~1000 writes (two compactions) while advertising; the radio interrupts keep coming (no gap over 450 ms, at least half the idle rate) |
| T6 power loss | not in the BLE build: 24 window-watchdog resets land inside 600-byte writes, 10 inside appends and 14 inside compactions (the erase included). After each, the region reads all-old or all-new and nothing else has changed |
| T7 bonds      | W5 BLE build only: a record added to the BLE bond store (`NVM_Add`, as the stack does) reaches page 127 through the flash manager, and `NVM_Discard` erases the page again. Clears any real bonds on the board |

T6 uses the window watchdog (WWDG): it counts on PCLK1, so it fires even while
the CPU is stalled on a flash erase, and a reset turns it off again. It isn't
an IWDG strike, so the L4's brick recovery doesn't count it. Progress is kept
in backup registers 0-23.

`../host-nvm-sim` runs the same `core_nvm.c` on a simulated flash with a power
cut at every erase and program step (about 30,000 cuts on the L4 and W5, 76,000
on the H5, one per 16-bit word); this test checks the real chip at a couple of
dozen points.

## Build and flash

`make distclean` when switching Cores or BLE.

```sh
make && make flash-serial              # Core.ST.L4, then read the CDC
python3 host_test.py                   # the whole run + T4 with flash-serial (~2 min)
python3 host_test.py --dfu             # ... T4 with make flash-dfu (ROM DFU)

make distclean && make TILE=Core.ST.L4.2
make distclean && make TILE=Core.ST.H5 && make TILE=Core.ST.H5 flash-serial   # config-h5.json
python3 host_test.py --tile Core.ST.H5            # the whole run + T4 (add --dfu for flash-dfu)
make distclean && make TILE=Core.ST.W5            # result in g_nvm_result (SWD)
make distclean && make TILE=Core.ST.W5 BLE=1      # adds T5 and T7
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

## Reading the result

**L4, H5:** a report over USB CDC every 3 s (115200). Send `R` for a fresh run.
The H5 report ends with the data area's option register and the first-boot
record (`_core_nvm_h5_boot`, see `sdk/core/core_nvm.h`), and the H5 build
resets itself if USB has no address 15 s after boot (as `hw-h5-bringup` does).

H5 bench, 2026-09-26 ("medium", 64 MHz): PASS with the T4 reflash by
flash-serial and by flash-dfu. A 600-byte append takes 12.6 ms and a
compaction ~34 ms; T6: 24 resets, 20 old, 4 new, 0 garbage.

```
[hw-nvm-flash] PASS  (pass=0x27 fail=0x00 expected=0x27)
  T1 basic         PASS
  ...
  T6: 24 resets: 21 old, 3 new, 0 garbage; ECC errors recovered: 0
    #0  append     reset at   300 us: inside the write   -> old
  ...
  boot counter 2; token 0x2d62012f (armed: reflash another image, then this one)
```

After a run the token is *armed*: the next boot that finds it in NVM checks it
(T4) and says `checked after reflash`. `host_test.py` does the reflash in
between and compares the tokens.

**W5 (and any Core):** `g_nvm_result` in RAM (`arm-none-eabi-nm
build/hw-nvm-flash.elf | grep g_nvm_result`): `magic` 0x4E564D52 once ready,
`verdict` 0x600D600D for PASS or 0xBAD0000n (n = first failing test), the
masks, the T2/T5/T6 figures and the token. Write 0x52 (`R`) to `g_nvm_cmd`
for a fresh run. LED: 1 s on / 1 s off for PASS, n quick blinks for FAIL.

## Core.ST.W5 on a CoreProbe

The W5 is powered from the CoreProbe's 5 V out (the board regulates to 1.8 V).
Power it first, then flash with plain probe-rs:

```sh
python3 ../../tools/coreprobe_power.py apply --config <copy of config.json with "probe": {"target_power": "5v"}>
python3 host_test.py --w5              # no BLE: T1-T4, T6
python3 host_test.py --w5 --ble        # BLE: T1-T5, T7 (advertises as "nvm-test")
```

`host_test.py --w5` builds, runs `probe-rs download` + `reset`, writes `R` to
`g_nvm_cmd`, polls `g_nvm_result` with `probe-rs read`, flashes the
`core-st-w5` starter and then this test again, and compares the token. For
T5, a phone scanner showing "nvm-test" throughout the run is a second check.
