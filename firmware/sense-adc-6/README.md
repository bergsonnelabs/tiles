# Sense.ADC.6 firmware

The firmware that makes a Core.ST.L0.1 into a **Sense.ADC.6** tile. Flash
this onto an L0 and it stops being a Core and becomes a six-channel analog
input peripheral: hosts see an I2C device, not an MCU.

Paired with the `tile_sense_adc_6` driver in `drivers/`, which is what
host firmware and Studio talk to. This README documents the wire-level
register map behind that driver; day to day, use the driver.

Six-channel ADC sensor hub on a Core.ST.L0.1, answering as an I2C device
at address **0x28**. The Core samples six analog inputs into a DMA ring at
a TIM2-paced scan rate, averages each channel over a configurable window,
and publishes calibrated millivolts into a register file the host reads.
100 Hz output by default; rate and averaging window are settable over I2C.

## Pad map

Six is the ceiling, not a round number: these are every ADC-capable pad
the tile has once I2C1 claims pads 4 and 5.

| Pad | Function  | Role            |
|-----|-----------|-----------------|
| 1   | GND       | ground          |
| 2   | ADC3      | **CH0**         |
| 3   | ADC0      | **CH1** (fast channel) |
| 4   | I2C1.CLK  | host bus        |
| 5   | I2C1.DAT  | host bus        |
| 6   | ADC1      | **CH2**         |
| 7   | ADC2      | **CH3**         |
| 8   | ADC5      | **CH4** (fast channel) |
| 9   | ADC8      | **CH5**         |
| 10  | V+        | supply          |
| 11  | BOOT0     | reserved        |
| 12  | NRST      | reserved        |
| 13  | SWCLK     | debug           |
| 14  | SWDIO     | debug           |

Channels are numbered in ascending pad order, so CH0..CH5 read left to
right on the board. That is deliberately NOT the ADC channel-number
order (3, 0, 1, 2, 5, 8), which is the order the hardware actually
converts in.

## Register map

Multi-byte values are **little-endian**, matching the Core's native
order, so a host on a little-endian machine can memcpy the block
straight into a `uint16_t` array.

| Offset | Name     | Notes                                          |
|--------|----------|------------------------------------------------|
| 0x00   | WHO_AM_I | `0x6D`, constant                               |
| 0x01   | VERSION  | `0x12` = v1.2                                  |
| 0x02   | STATUS   | bit0 valid, bit1 DMA running, bit2 VDDA calibrated |
| 0x03   | SEQ      | increments per published set, wraps at 255     |
| 0x04   | CH0 mV   | pad 2                                          |
| 0x06   | CH1 mV   | pad 3                                          |
| 0x08   | CH2 mV   | pad 6                                          |
| 0x0A   | CH3 mV   | pad 7                                          |
| 0x0C   | CH4 mV   | pad 8                                          |
| 0x0E   | CH5 mV   | pad 9                                          |
| 0x10   | VDDA mV  | measured supply, for host sanity checks        |
| 0x12   | ADDR_CUR | address this hub is answering on right now     |
| 0x13   | ADDR_SET | **writable** — staged address                  |
| 0x14   | COMMIT   | **writable** — save to EEPROM                  |
| 0x15   | RATE_HZ  | **writable** — output rate, 1–250 Hz           |
| 0x16   | OVERSAMP | **writable** — scans averaged per period, 1–32 |
| 0x17   | WINDOW   | **writable** — periods averaged, 1 or 2        |
| 0x18   | APPLY    | **writable** — apply 0x15–0x17 as a set        |

Only 0x13–0x18 accept host writes. Everything else is read-only: writes
to it set the register pointer and are otherwise absorbed. The data
block 0x02–0x0F is unchanged since v1.0.

### Reading it

Write the pointer `0x02`, then burst-read 14 bytes. That returns STATUS,
SEQ and all six channels from **one coherent sample set**: the driver
latches the published snapshot into the register file at the start of a
read, and nothing rewrites it mid-transfer.

A SEQ that has not moved between two reads means the host is polling
faster than RATE_HZ, not that the hub has stopped. Each hub runs on its
own HSI16 clock (±1% at 25 °C, a few % over temperature), so poll 2–3×
faster than RATE_HZ and deduplicate on SEQ.

## Acquisition control

TIM2 triggers one scan of all six channels at RATE_HZ × OVERSAMP scans
per second. Every 1/RATE_HZ seconds the hub publishes the average of the
most recent WINDOW × OVERSAMP scans.

- **WINDOW 1**: a boxcar over one output period. Nulls at RATE_HZ and
  every multiple.
- **WINDOW 2**: a moving boxcar over two periods, still published every
  period. Nulls at RATE_HZ / 2 and every multiple. This is the
  mains-rejection setting.

| Goal | RATE_HZ | OVERSAMP | WINDOW | Window |
|------|---------|----------|--------|--------|
| Default | 100 | 8 | 1 | 10 ms |
| Reject 50 Hz, 100 Hz output | 100 | 8 | 2 | 20 ms |
| Reject 60 Hz, 120 Hz output | 120 | 8 | 2 | 16.7 ms |
| Reject both, 10 Hz output | 10 | 8 | 1 | 100 ms |

Note the default (100/8/1) passes 50 Hz at only −3.9 dB and 60 Hz at
−5.9 dB. If the sensor leads can pick up mains, use one of the rejection
settings.

The three settings only make sense together, so they are staged, then
applied as a set:

1. write RATE_HZ, OVERSAMP, WINDOW (one burst starting at `0x15` is fine)
2. write `0x5A` to `APPLY`
3. read `APPLY` back:

| Value | Meaning |
|-------|---------|
| 0x00 | idle |
| 0x01 | applied |
| 0xE1 | rejected; 0x15–0x17 are restored to the running values |

Limits: OVERSAMP × WINDOW ≤ 32 scans (the ring costs 12 bytes per scan
on a 2 KB part), and RATE_HZ × OVERSAMP ≤ 8000 scans/s (a scan takes
~65 µs and must finish before the next trigger).

A change takes effect within one output period. Until the ring holds only
samples taken under the new settings, STATUS.valid is clear and SEQ does
not advance, so a host never averages across a change.

## Saving: address and settings

Several hubs share a bus and there is no spare pad for an address strap,
so the address lives in the L011's 512 bytes of true EEPROM. COMMIT
saves the staged address **and** the running acquisition settings:

1. write the wanted address (0x08–0x77) to `ADDR_SET`
2. write `0xA5` to `COMMIT`
3. read `COMMIT` back:

| Value | Meaning |
|-------|---------|
| 0x00 | idle |
| 0x01 | stored |
| 0xE1 | address out of range, nothing stored |
| 0xE2 | EEPROM write failed or did not read back |

4. reset the hub if the address changed

Acquisition settings are live the moment they're applied; COMMIT only
makes them survive a reset. **The address applies at the next reset**,
because a board that moves address underneath a running host is far
worse to debug than one that needs a reset. The magic byte is there so a
stray write can't knock a deployed board off the bus.

**Commit on a bench or jig, not on a live array.** The L011 is
single-bank, so per RM0377 any NVM read during a write stalls the CPU,
and with it the I2C ISR, for ~3.2 ms per byte. Only changed bytes are
written, so a commit costs 0 to ~29 ms of clock stretching. The host
needs a timeout longer than that. Other hubs are unaffected.

A blank or corrupt EEPROM falls back to the defaults (address 0x28,
100/8/1), so an unprogrammed board still answers. A board committed by
v1.1 keeps its address and picks up the default settings. The default
address can be overridden per build:

```bash
make EXTRA_CFLAGS=-DHUB_I2C_ADDR_DEFAULT=0x2A
```

## How many per bus

At 100 Hz each hub costs about 4% of a 400 kHz bus (a 14-byte poll is
~400 µs), so keeping utilization under 40% gives roughly **10 hubs (60
channels) per bus**. At 100 kHz that drops to **2–3 hubs**. With long
harnesses the 400 pF fast-mode limit is a real risk, so bus speed is an
architectural decision here, not a default.

## How it works

- TIM2 emits TRGO on each update; the ADC converts one scan of all six
  channels per trigger, and DMA drops each scan into the ring with no CPU
  involved. Spreading scans across the window is the point: a
  free-running ADC fills the ring in ~60 µs, which averages noise inside
  the burst but does nothing about aliasing.
- The main loop publishes each time the DMA crosses an output-period
  boundary in the ring. Publishing is locked to TIM2, not to a
  millisecond timer, so RATE_HZ can be any value (60 and 120 Hz
  included) with no drift between two clocks.
- Publishing averages the ring per channel, converts to millivolts, and
  swaps the set in under `ll_irq_save()`.
- The I2C target ISR copies that set into the register file at
  `READ_START`, while SCL is still stretched and before the first byte
  goes out. That is what makes a burst read coherent.
- VDDA is measured once at startup, **before** DMA starts. A VREFINT
  conversion under a running DMA rewrites the channel selection, which is
  how the first silicon run failed.

## Build and flash

```bash
make
```

Flash over SWD with the CoreProbe (pads 13/14). The project Makefile's
`make flash` assumes OpenOCD and an ST-Link; with the CoreProbe use
probe-rs:

```bash
probe-rs download --probe 1209:da01 --chip STM32L011K4 --protocol swd build/l0-adc-mux.elf
```

```bash
probe-rs reset --probe 1209:da01 --chip STM32L011K4 --protocol swd
```

The L011 has no USB, so there is no DFU path.

## Status

v1.2 builds clean at 6440 bytes flash and 820 bytes of static RAM (`size`
also counts the linker's 768-byte heap + stack reservation in bss).
About 1.2 KB is left for stack; nothing uses the heap.

**On silicon (2026-09-22):** trigger, ADC, DMA, averaging, mV conversion
and publish all verified. Rate and window changes verified by writing the
staging registers and pending flag over SWD: scan rates exact (800, 960,
80 Hz), invalid settings rejected with registers restored.

**Over a real I2C host (2026-09-22): 20/20.** A Core.ST.L4.1 running
`tests/hw-adc-mux-host` read the hub at 100 kHz and 400 kHz: 99.8 Hz
publish rate, zero bus errors and zero missed samples across ~4,000
transactions, and rate/window changes written over I2C applied live
(100.00 Hz and 120.00 Hz measured) with an invalid request rejected.

**Not yet verified:** absolute accuracy (inputs were floating), COMMIT
persisting settings across a reset, and behavior on a long harness.

Measuring the publish rate over SWD undercounts: each `probe-rs` attach
stalls the CPU for tens of milliseconds while TIM2 and DMA keep cycling
the ring, so those periods are never published. The measured rate rose
from 87 to 97 Hz as reads went from 0.3 s to 2.2 s apart. A real I2C host
doesn't do this.

The IWDG is **disabled** in `config.json` for bring-up, because an armed
watchdog resets the chip ~0.3 s after an SWD halt and corrupts a flash
sequence in progress. Turn it on before this goes anywhere real.
