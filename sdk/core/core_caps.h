/**
 * core_caps.h — Core capability macros (CORE_HAS_<FEATURE>)
 *
 * What the Core being built for has: CORE_HAS_USB, CORE_HAS_BLE,
 * CORE_HAS_THREAD, CORE_HAS_AUDIO, CORE_HAS_DAC, CORE_HAS_I3C, CORE_HAS_SPI,
 * CORE_HAS_RNG, CORE_HAS_OTP, CORE_HAS_DWT, CORE_HAS_SMALL_RAM. Each is
 * always defined, as 1 or 0: test with `#if CORE_HAS_X`.
 *
 * The values come from the port: this header only includes port_caps.h,
 * which each port ships in its own include directory (the STM32 one is
 * sdk/port/stm32/core/port_caps.h, which also has the feature x MCU table).
 * Every core_ header that gates on a capability includes this file, so the
 * neutral core_ headers never name a vendor's part.
 *
 * Not the same as coregen's CORE_USES_* project flags (core_pads.h), which
 * say what a project uses and are defined only when it does.
 */

#ifndef CORE_CAPS_H
#define CORE_CAPS_H

#include "port_caps.h"

#endif /* CORE_CAPS_H */
