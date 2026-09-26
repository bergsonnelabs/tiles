# hw-clock-levels

Bench test for the generated clock levels (`"clock": "low" | "medium" | "high"
| "max"`) on **Core.ST.L0.1**, **Core.ST.L4.1 / L4.2** and **Core.ST.W5**,
written for the 2026-09-25 clock fixes. One build per level; each build checks
the hardware against the reference manual and against an independent clock.

| Test | What passes |
|------|-------------|
| T1 flash    | `FLASH_ACR.LATENCY` is the RM's value for SYSCLK in the active voltage range (RM0377 Table 14, RM0394 Table 9, RM0493 Table 41) |
| T2 range    | the voltage range allows SYSCLK and is the one coregen chose (`CLOCK_VOS_RANGE`); L4: range 1 (USB) |
| T3 tree     | SYSCLK source; PLL within limits (L0 VCO <= 96 MHz; L4 input 4-16 MHz, VCO 64-344; W5 input 4-16 MHz with the matching PLL1RGE, VCO 128-544, the PLL1R rule, prescaler released); W5 hclk5 <= 32 MHz in range 1, HDIV5 + SRAM 1 WS + PLL off in range 2; L4 SYSCLK >= 10 MHz (USB); SysTick reload = SYSCLK_HZ / 1000 |
| T4 clock    | SYSCLK / SysTick against a precise reference (below) |
| T5 rtc      | `core_millis()` per RTC second is inside the LSI datasheet band |

The RM tables are written out in `main.c` rather than taken from `ll_rcc.h`, so
a wrong table in the SDK can't pass itself.

### The references (T4)

| Core | Reference | Limit |
|------|-----------|-------|
| L4 | USB start-of-frame count (`USB_FNR`), 1 per ms from the host (±500 ppm) over 1.5 s | ±2 % (MSI / HSI16 at room temperature) |
| W5 | TIM16 input capture of HSE32 / 32 (`TISEL` = 3, RM0493 Table 295), /8, over 200 ms: measures SYSCLK itself | ±0.1 % on HSE levels; ±1.2 % at `low` (HSI16, DS Table 65) |
| L0 | LPTIM1 counting HSI16 / 128 over 50 SysTick ms | ±2.5 % at MSI levels (MSI and HSI16 are both factory-trimmed RCs); ±0.5 % at HSI16 levels (a ratio check) |

On the L0 a range-3 build (`low`) steps up to range 2 for the LPTIM
measurement, because HSI16 can't run in range 3 (RM0377 Table 43), and back
after. T1/T2/T3 read the registers before that.

### LSI bands (T5)

The RTC runs on LSI and the SDK assumes its nominal frequency (`LL_LSI_HZ`),
so an RTC second is `1000 x nominal / actual` ms. Coarse: it only catches gross
errors (a wrong multiplier, a SysTick reload off by a factor).

| Core | LSI (datasheet) | nominal | ms per RTC second |
|------|-----------------|---------|-------------------|
| L0 | 26-56 kHz (DS Table 39) | 37 kHz | 660-1424 |
| L4 | 29.5-34 kHz (DS Table 50) | 32 kHz | 941-1085 |
| W5 | LSI1 30.4-33.6 kHz (DS Table 66) | 32 kHz | 952-1053 |

## What each level is, after the 2026-09-25 fixes

| Core | low | medium | high | max |
|------|-----|--------|------|-----|
| L0.1 | MSI 1.048576 MHz, range 3, 0 WS (range 2 if the project has I2C) | MSI 2.097152 MHz, range 2, 0 WS | HSI16, range 1, 0 WS | HSI16 x4 /2 = 32 MHz (VCO 64), range 1, 1 WS |
| L4.x | **MSI 16 MHz** (USB needs APB >= 10 MHz), range 1, 0 WS | MSI 16 MHz, 0 WS | MSI 48 MHz, 2 WS | HSI16 /1 x10 /2 = 80 MHz (VCO 160), 4 WS |
| W5 | HSI16, range 2, 1 WS, hclk5 8 MHz | HSE32, range 1, 0 WS, hclk5 32 MHz | HSE /2 x8 /2 = 64 MHz (VCO 128), 1 WS, hclk5 32 | HSE /4 x25 /2 = 100 MHz (VCO 200), 3 WS, hclk5 25 |

## Build and flash

Each level has its own `config-<level>.json`. The Makefile starts from a clean
`coregen/` and `build/` whenever `CLOCK` or `TILE` changes, so a build never
carries another level's clock setup.

```sh
make CLOCK=high                              # Core.ST.L4.1 (default TILE)
make CLOCK=high flash-serial                 # or: flash-dfu

make CLOCK=max TILE=Core.ST.L0.1
make CLOCK=max TILE=Core.ST.L0.1 flash-coreprobe

make CLOCK=low TILE=Core.ST.W5
make CLOCK=low TILE=Core.ST.W5 flash-coreprobe
```

`"bootloader": "rom"` is the ROM-DFU layout (app at 0x08000000). Never build
this with `BOOTLOADER := 1`.

## Reading the result

**L4:** a report over USB CDC every 3 s (115200, any terminal):

```
[hw-clock-levels] PASS  level=max SYSCLK=80000000 Hz  (pass=0x1f fail=0x00)
  T1 flash latency     PASS
  ...
  latency 4 (RM 4), range 1, SWS 3, VCO 160 MHz, PLL in 16000 kHz
  SysTick 1500 ms over 1500 USB frames: 0 ppm (limit 20000); RTC second = 995 ms (941-1085)
```

**LED (all Cores):** PASS = slow blink, 1 s on / 1 s off. FAIL = N quick
blinks, a 1.5 s pause, repeat; N is the first failing test (1 = T1 ... 5 = T5).

**SWD (L0 and W5; works on the L4 too):** the `g_clock_levels` struct.

```sh
arm-none-eabi-nm build/hw-clock-levels.elf | grep g_clock_levels
probe-rs read --chip <chip> b32 <address> 27
```

Layout (32-bit words): magic (`0xC10CC10C` when final), verdict (`0x600D600D`
= PASS, `0xBAD000nn` = FAIL at test nn), pass mask, fail mask, level name (2
words, ASCII), SYSCLK_HZ, latency, RM latency, range, wanted range, SWS, VCO
MHz, PLL input kHz, hclk5 kHz (W5), reference kind (1 USB SOF, 2 HSE/32,
3 HSI16), clock error ppm (signed), its limit, measured SYSCLK Hz (W5),
reference ms, SysTick ms, ms per RTC second, and its low / high bound.

## What a failure points at

- T1: the flash latency table or its ordering against the clock switch.
- T2: the voltage range (L0 `PWR_CR.VOS`, W5 `PWR_VOSR`); on the L4 something
  left range 1.
- T3: PLL factors (coregen's solver / `ll_rcc_pll_config`), PLL1RGE, HPRE5 /
  HDIV5, or a SYSCLK source other than the level's.
- T4: SYSCLK_HZ disagrees with the real clock (the L0 MSI levels used to be 5 %
  off: `SYSCLK_HZ` said 1 / 2 MHz for 1.048576 / 2.097152 MHz), or a wrong
  multiplier. L4 with `USB frames` near 0: no host was sending SOFs.
- T5: only gross errors (see the bands above).
