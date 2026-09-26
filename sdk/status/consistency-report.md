# Cores SDK Horizontal Consistency Report

> **Superseded (kept for history).** This April 2026 report is mostly stale and partly wrong
> (it claims SAI on the L4 and H5, and AES on the H5). The current findings are the portal page
> "Core.ST SDK audit and priority list, September 2026" (SDK project), and the live per-Core
> status is `core-*.json` in this folder.

> 2026-04-06 — API patterns compared across subsystems within each layer
> Goal: Ensure users can activate new functionality without learning new syntax

---

## Executive Summary

The SDK has strong internal consistency in some areas (handle pattern, typedef naming, family guards) but meaningful divergence in others. The issues below are ranked by their impact on developer experience.

### The 4 Most Impactful Inconsistencies

1. **Callbacks: context pointer missing on ADC DMA, BLE, USB, Fault** — Forces global state, prevents composable code
2. **Clock parameters leak through Core API** — `core_pwm_init` and `core_serial_init` expose `pclk_hz` while `core_timer_init_freq` hides it
3. **Three different error conventions** — `hal_status_t`, `int 0/-1`, and `void` (no error) coexist
4. **PWM duty unit conflict** — permil (0-1000) in `core_pwm_set`, percent (0-100) in `core_timer_pwm_set`

---

## LL Layer Patterns

### What's Consistent (Good)
- TypeDef struct pointers for multi-instance peripherals (GPIO, UART, I2C, SPI, TIM, ADC)
- REG32() macros for single-instance peripherals (IWDG, RNG, RTC, PWR)
- `#if defined(STM32xxxx)` family guards with the same 4 defines everywhere
- `LL_XXX_H` include guard naming
- All include `ll_common.h` (except one)

### What Diverges

| Pattern | Standard | Deviations |
|---------|----------|-----------|
| Init naming | `ll_xxx_init()` | `_config()` for TIM/DMA/EXTI, `_power_on()` for USB, `_enable()` for RNG/CRS |
| Clock enable | Caller responsible | RTC and EXTI enable clocks inside init |
| Constant prefix | `LL_XXX_` | USB, Flash, PWR, CRS, DAC use raw `USB_*`, `FLASH_*`, `DAC_*` |
| RCC functions | All in `ll_rcc.h` | 3 `ll_rcc_*` functions in `ll_dma.h`, `ll_pwr.h`, `ll_exti.h` |
| Includes | `ll_common.h` | `ll_systick.h` includes only `<stdint.h>` (misses `REG32` definition) |

### LL Fixes Needed
- **`ll_systick.h`**: Add `#include "ll_common.h"` (compilation depends on include order otherwise)
- **Consolidate RCC clock enables**: Move `ll_rcc_dma1_clk_enable`, `ll_rcc_pwr_clk_enable`, `ll_rcc_syscfg_clk_enable` into `ll_rcc.h`
- **DAC channel parameter**: `ll_dac_init()` should take a channel argument (even if only channel 2 is currently used)

---

## HAL Layer Patterns

### What's Consistent (Good)
- `hal_xxx_t` handle struct with instance pointer for all communication peripherals
- `memset(h, 0, sizeof(*h))` in init for UART, I2C, SPI, Timer, ADC
- `hal_status_t` return from init for communication peripherals
- Static `_handles[]` array for ISR dispatch (UART, Timer)
- `_xxx_clk_enable()` private function pattern (UART, I2C, SPI, ADC)

### What Diverges

| Pattern | Standard | Deviations |
|---------|----------|-----------|
| Handle struct | `hal_xxx_t` for all | USB CDC: global `_cdc`, no handle. DAC: handle exists but unused in read/write |
| Init return | `hal_status_t` | DAC, USB, Debug: return `void` |
| DMA callback | `hal_callback_t (void (*)(void *ctx))` | ADC DMA: `void (*)(void)` — no context pointer |
| Deinit | `hal_xxx_deinit()` | Timer, ADC, DAC, USB CDC: no deinit function |
| Clock flush | `(void)REG32(RCC_BASE)` after enable | Timer `_tim_clk_enable()` is missing the pipeline flush |
| Bool type | `uint8_t` | ADC uses `bool` from `<stdbool.h>` (only module) |

### HAL Fixes Needed
- **Timer clock flush**: Add `(void)REG32(RCC_BASE)` to `_tim_clk_enable()` to match UART/I2C/SPI/ADC
- **ADC DMA callback**: Change signature to `hal_callback_t` (add `void *ctx` parameter)
- **Add deinit for Timer and ADC**: Follow UART/I2C/SPI pattern (disable peripheral, clear handle, disable NVIC)
- **DAC and USB init should return `hal_status_t`**

---

## Core API Layer Patterns

### What's Consistent (Good)
- `core_xxx_` function prefix
- Pad-centric API where appropriate (ADC, EXTI, PWM-pad)
- Handle-based API for multi-instance peripherals (I2C, SPI, Serial, Timer)
- Singleton API for chip-global peripherals (USB, BLE, Watchdog)

### What Diverges — with Recommendations

#### A. Clock Parameter Leakage

| Function | Exposes clock? | Recommendation |
|----------|---------------|----------------|
| `core_timer_init_freq(h, TIMx, freq)` | No (auto) | **Model to follow** |
| `core_i2c_setup(h, I2Cx, speed)` | No (auto) | Good |
| `core_pwm_init(h, TIMx, pclk, freq)` | **Yes** | Auto-resolve from `SYSCLK_HZ` |
| `core_serial_init(h, USARTx, pclk, cfg)` | **Yes** | Auto-resolve from `PCLK1_HZ`/`PCLK2_HZ` |
| `core_every_us(h, TIMx, pclk, period, cb, ctx)` | **Yes** | Auto-resolve from `SYSCLK_HZ` |

**Rule**: The Core layer should NEVER expose clock parameters. That's what coregen's `core_config.h` defines are for.

#### B. Error Return Conventions

| Convention | Used by | Recommendation |
|-----------|---------|----------------|
| `hal_status_t` | ADC, I2C, SPI, Serial, Timer, EXTI | **Standard — use everywhere** |
| `int` (0/-1) | NVM, BLE | Migrate to `hal_status_t` |
| `void` (no error) | DAC, USB, Debug, RTC, Watchdog | Add `hal_status_t` return where hardware can fail |

#### C. Callback Registration

| Pattern | Used by | Context? | Recommendation |
|---------|---------|----------|----------------|
| `core_xxx_on_event(cb, ctx)` | EXTI | **Yes** | **Model to follow** |
| `core_timer_enable_tick(h, cb, ctx)` | Timer | **Yes** | Good |
| `core_xxx_on_event(cb)` | USB, BLE | **No** | Add `void *ctx` |
| `core_adc_start_dma(adc, buf, len, cb)` | ADC | **No** | Add `void *ctx` |

**Rule**: Every callback must accept `void *ctx`. This is non-negotiable for composable embedded code.

#### D. PWM Duty Units

| Function | Unit | Range |
|----------|------|-------|
| `core_pwm_set(h, ch, duty_permil)` | Permil | 0-1000 |
| `core_timer_pwm_set(h, ch, duty_percent)` | Percent | 0-100 |

**Rule**: Use permil (0-1000) everywhere. It's the HAL native unit and gives 0.1% resolution. Deprecate the percent version.

#### E. Text Output Interfaces

| Function | Serial | USB | Debug |
|----------|--------|-----|-------|
| `_write(data, len)` | Yes | Yes | **Missing** |
| `_print(str)` | Yes | **Missing** | Yes |
| `_printf(fmt, ...)` | Yes | Yes | Yes |
| `_putc(byte)` | Yes | **Missing** | **Missing** |

**Rule**: All three text interfaces should offer the same 4 functions. The missing ones are trivial wrappers.

#### F. Naming Consistency

| Issue | Current | Recommendation |
|-------|---------|----------------|
| I2C init | `core_i2c_setup()` (friendly) + `core_i2c_init()` (advanced) | Rename: `core_i2c_init()` = friendly, `core_i2c_init_cfg()` = advanced |
| ADC VDD | `core_adc_vdd()` returns mV, no suffix | `core_adc_vdd_mv()` |
| ADC temp | `core_adc_temp()` returns deci-degrees C | Document unit prominently |
| BLE write | `core_ble_set_value()` | Consider `core_ble_write()` for consistency |

#### G. Availability Guards

| Current behavior | Headers | Recommendation |
|-----------------|---------|----------------|
| Silent empty | DAC, USB, USB HID | `#error` with clear message naming required Core variant |
| Link-time error | BLE | Add `#if defined(STM32WBA55xx)` with `#error` in else |
| Runtime -1 | NVM | Keep runtime return, add `#warning` at compile time |
| No guard | All others | N/A (compile on all cores) |

#### H. Documentation Template

Every `core_xxx.h` should have:
```c
/**
 * @file core_xxx.h
 * @brief One-sentence description.
 *
 * Quick start:
 * @code
 *   core_xxx_t h;
 *   core_xxx_init(&h, ...);
 *   core_xxx_write(&h, data, len);  // (blocking)
 * @endcode
 */
```
Currently only Watchdog, NVM, Timer, and BLE have usage examples. Serial, SPI, Debug, and RTC have minimal documentation.

---

## Cross-Layer Pattern Summary

| Aspect | LL Convention | HAL Convention | Core Convention | Consistent? |
|--------|-------------|---------------|----------------|-------------|
| Multi-instance | TypeDef pointer param | `hal_xxx_t` handle | Handle or pad number | Yes |
| Singleton | Global REG32 macros | Global struct (USB) or handle (DAC) | No handle needed | Mostly |
| Init | `ll_xxx_init()` | `hal_xxx_init(h, ...)` | `core_xxx_init(...)` | Yes (naming varies at LL) |
| Clock enable | Caller does it | HAL init does it | Hidden from user | Yes (good layering) |
| Family guards | `#if defined()` | `#if defined()` | `#if defined()` or nothing | HAL/Core need improvement |
| Error return | void (LL is fire-and-forget) | `hal_status_t` | Mixed | **Needs harmonizing** |
| Callback | N/A | `hal_callback_t` + ctx | Mixed | **Needs harmonizing** |

---

## Implementation Priority

### Before Public Release (API-affecting)
1. Add `void *ctx` to ADC DMA, BLE, USB, and Fault callbacks
2. Remove clock parameters from `core_pwm_init`, `core_serial_init`, `core_every_us`
3. Standardize on `hal_status_t` for all failable functions
4. Fix PWM duty unit conflict (permil everywhere)
5. Add `#error` availability guards to DAC, USB, BLE headers
6. Rename `core_i2c_setup` → `core_i2c_init`

### Before V1.0 (Quality)
7. Add timer pipeline flush
8. Add deinit for Timer and ADC
9. Harmonize text output interfaces (write/print/printf/putc)
10. Document blocking behavior on I2C, SPI
11. Add usage examples to Serial, SPI, Debug, RTC headers

### Ongoing
12. Consolidate scattered `ll_rcc_*` functions
13. Normalize LL constant prefixes (add `LL_` to USB, Flash, DAC, etc.)
14. Standardize LL init naming (`_init` vs `_config` vs `_enable`)
