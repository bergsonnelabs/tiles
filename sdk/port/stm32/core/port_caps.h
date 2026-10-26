/**
 * port_caps.h — The STM32 port's facts that are not status features
 *
 * Included first by coregen's core_caps.h (the project's coregen/), which
 * defines every status-backed CORE_HAS_<FEATURE> (USB, BLE, THREAD, AUDIO,
 * DAC, I3C, SPI, RNG, OTP) from the Core's status file: a feature's "cap" in
 * sdk/status/features.json. This file has only what no status file says,
 * for the part named by -D$(MCU_PART), always defined as 1 or 0 (test with
 * `#if`). Include core_caps.h, not this file. Another port (e.g. nRF) ships
 * its own port_caps.h with the same macro names, found through its own
 * include directory; it must not define a status-backed CORE_HAS_
 * (core_caps.h stops with an #error if it does).
 *
 *   fact         L0 (L011)  L4 (L422)  W5 (WBA55)  H5 (H523)
 *   DWT             0          1          1           1     DWT cycle counter (M4/M33)
 *   SMALL_RAM       1          0          0           0     2 KB-class SRAM: small buffer defaults
 */

#ifndef PORT_CAPS_H
#define PORT_CAPS_H

#if defined(STM32L011xx)
#define CORE_HAS_DWT        0
#define CORE_HAS_SMALL_RAM  1

#elif defined(STM32L422xx) || defined(STM32WBA55xx) || defined(STM32H523xx)
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0

#else
/* No STM32 part defined: host builds of SDK sources (tools/test_core_scope.py
 * and the like). Same answers the old part-name guards gave with no part
 * ("is not the L011"). */
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0
#endif

#endif /* PORT_CAPS_H */
