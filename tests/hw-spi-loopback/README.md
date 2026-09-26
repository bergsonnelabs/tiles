# hw-spi-loopback

Bench test for `core_spi` (portal task #285) on **Core.ST.L4.1 / L4.2** and
**Core.ST.W5**: bounded polled transfers, DMA, all four modes, timeouts, and
the `core_tiles_pal()` SPI bridge. It needs one jumper from MOSI to MISO and
runs unattended in under a second, with the 5 s watchdog armed as in a Studio
project. The bus and its CS pad come from `config.json`, as coregen sets them
up for any project.

| Core | Config | Jumper | SCK / CS (leave open) | Clock |
|------|--------|--------|-----------------------|-------|
| Core.ST.L4.1 | `config.json` | pad 2 (PA7, SPI1 MOSI) → pad 8 (PB4, SPI1 MISO) | pad 3 / pad 9 | max, 80 MHz |
| Core.ST.L4.2 | `config-l42.json` | pad 2 (PA7) → pad 18 (PB4) | pad 3 / pad 19 | max, 80 MHz |
| Core.ST.W5 | `config-w5.json` | pad 6 (PB8, SPI3 MOSI) → pad 7 (PB9, SPI3 MISO) | pad 2 / pad 3 | high, 64 MHz |

Never use the L4's USB pads (6/7 on the L4.1, 16/17 on the L4.2). The W5 runs
at `high` so its fastest SCK (32 MHz) is legal at the 1.8 V a CoreProbe
supplies (33 MHz below 2.7 V, DS14127 Table 94).

| Test | What passes |
|------|-------------|
| T0 jumper  | with the SPI parked, MOSI driven as a GPIO: MISO follows it high (against a pull-down) and low (against a pull-up). Otherwise the result is **SKIP, "loopback jumper missing"**, and T1-T6 don't run |
| T1 bytes   | single bytes 0x00 0xFF 0xA5 0x5A and 0..255, and a 256-byte buffer polled and by DMA, at /256 and at /2 |
| T2 buffers | 1 2 3 63 64 65 255 256 1024 4096 bytes, polled and DMA, rx == tx; also in place (tx == rx), NULL rx, and NULL tx (reads back the 0xFF fill) |
| T3 modes   | modes 0-3 and LSB-first, polled and DMA, and SCK idles at CPOL (read on the SCK pad: `0011`) |
| T4 speed   | 4096 B x 8 at the fastest SCK (/2), polled and DMA, rx == tx; reports bytes/s |
| T5 timeout | a transfer that cannot complete returns `HAL_TIMEOUT` within its stall budget, polled, blocking DMA, and async DMA + `core_spi_dma_wait()`; on the L4 also with the SPI clock gated. The next transfer works with no help from the test |
| T6 pal     | `core_tiles_pal()` `spi_transfer`: a Store.O.128-style READ (0x03 + 3-byte address + 64 data bytes) returns 0 under **one** CS assertion (EXTI on the CS pad counts 2 edges); the same command captured full duplex comes back as sent; the register read/write adapters take one CS assertion each |

**How T5 forces the stall:** the SPI is switched to an unselected slave
(`MSTR` / `MASTER` cleared, software SS still high), so nothing drives SCK and
no frame ever arrives. That is defined behavior on both SPI generations and
safe to leave: the HAL's timeout path resets the peripheral (RCC reset) and
reapplies the handle's configuration. On the L4 a second case gates the SPI's
bus clock (`RCC_APB2ENR.SPI1EN = 0`), where its registers read 0. The bound
checked is 2 x the stall budget + 1 ms; the report shows the measured time
next to the budget.

## Build and flash

`make distclean` when switching Cores: `make clean` keeps `coregen/`.

```sh
make && make flash-serial               # Core.ST.L4.1 (exit 2 = old firmware: make flash-dfu once)
make distclean && make TILE=Core.ST.L4.2
make distclean && make TILE=Core.ST.W5  # result over SWD, see below
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

## Reading the result

**L4:** a report over USB CDC every 3 s (115200). Send `R` to run it again.

```
[hw-spi-loopback] PASS  (pass=0x7f fail=0x00 expected=0x7f run 1, first_err 0x00000000)
  T0 jumper  PASS  MISO (pad 8) follows MOSI (pad 2)
  T1 bytes   PASS  single 00 FF A5 5A + 0..255, 256-byte buffer; SCK 312500 Hz and 40000000 Hz
  ...
  T4 speed   PASS  SCK 40000000 Hz (wire max 5000000 B/s): polled 1769222 B/s, DMA 4961625 B/s (4096 B x 8, rx == tx)
  T5 timeout PASS  forced stall (unselected slave): polled 2018 us, DMA 2022 us, async+wait 2024 us, clock gated 2008 us (stall budget 2012 us); next transfer OK
  T6 pal     PASS  spi_transfer(03 12 34 56 + 64 B read): rc 0, 2 CS edges (want 2), data 0xFF; ...
```

`first_err` on a FAIL is `test << 24 | case << 12 | size` for the first check
that failed (see `err_once()` calls in `main.c`).

**Any Core over SWD:** `g_spi_lb` (verdict `0x600D600D` PASS, `0x5C1B0000`
SKIP, `0xBAD0000n` FAIL with n the first failing test, plus the T4/T5/T6
figures) and `g_spi_lb_log` (the same report text). Write `0x52` (`R`) to
`g_spi_lb_cmd` to run again. `read_swd.py` does all of it.

**LED:** PASS 1 s on / 1 s off; SKIP two short blinks and a 2 s pause; FAIL
n+1 quick blinks (n = first failing test, T0 = 1 blink) and a 1.5 s pause.

## Core.ST.W5 on a CoreProbe

The W5 is powered from the CoreProbe's 5 V out (the board regulates to 1.8 V).
Fit the pad 6 → pad 7 jumper, leave pads 2 and 3 open, then:

```sh
python3 ../../tools/coreprobe_power.py apply --config <copy of a W5 config with "probe": {"target_power": "5v"}>
python3 read_swd.py --flash      # distclean, build for the W5, probe-rs download + reset,
                                 # 6 s without SWD, then read g_spi_lb + the report
python3 read_swd.py --rerun      # run again without a reset
```

Keep SWD quiet while the W5 boots: an attach that lands on a W5 boot can crash
it (PINNED.md, 2026-09-26). `read_swd.py` waits after the reset and attaches
once; the test itself never resets.
