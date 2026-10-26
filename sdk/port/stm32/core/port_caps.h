/**
 * port_caps.h — What this Core has: the STM32 port's capability macros
 *
 * One CORE_HAS_<FEATURE> per feature the neutral core_ headers gate on,
 * always defined, as 1 or 0, for the part named by -D$(MCU_PART). Test them
 * with `#if CORE_HAS_X`, never `#ifdef`. Include core_caps.h, not this file.
 *
 * These are the STM32 port's capabilities until coregen generates them from
 * the Core's status file (later in SDK phase 1). Another port (e.g. nRF)
 * ships its own port_caps.h with the same macro names, found through its own
 * include directory.
 *
 * Not to be confused with coregen's CORE_USES_* project flags (core_pads.h):
 * those say what a project uses and are defined only when it does.
 *
 *   feature      L0 (L011)  L4 (L422)  W5 (WBA55)  H5 (H523)
 *   USB             0          1          0           1     USB device (CDC, HID)
 *   BLE             0          0          1           0     BLE radio
 *   THREAD          0          0          1           0     802.15.4 radio (Thread)
 *   AUDIO           0          0          1           0     SAI (PDM capture)
 *   DAC             0          0          0           1
 *   I3C             0          0          0           1
 *   SPI             0          1          1           1
 *   RNG             0          1          1           1
 *   OTP             0          0          1           0     OTP area exposed by core_otp
 *   DWT             0          1          1           1     DWT cycle counter (M4/M33)
 *   SMALL_RAM       1          0          0           0     2 KB-class SRAM: small buffer defaults
 */

#ifndef PORT_CAPS_H
#define PORT_CAPS_H

#if defined(STM32L011xx)
#define CORE_HAS_USB        0
#define CORE_HAS_BLE        0
#define CORE_HAS_THREAD     0
#define CORE_HAS_AUDIO      0
#define CORE_HAS_DAC        0
#define CORE_HAS_I3C        0
#define CORE_HAS_SPI        0
#define CORE_HAS_RNG        0
#define CORE_HAS_OTP        0
#define CORE_HAS_DWT        0
#define CORE_HAS_SMALL_RAM  1

#elif defined(STM32L422xx)
#define CORE_HAS_USB        1
#define CORE_HAS_BLE        0
#define CORE_HAS_THREAD     0
#define CORE_HAS_AUDIO      0
#define CORE_HAS_DAC        0
#define CORE_HAS_I3C        0
#define CORE_HAS_SPI        1
#define CORE_HAS_RNG        1
#define CORE_HAS_OTP        0
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0

#elif defined(STM32WBA55xx)
#define CORE_HAS_USB        0
#define CORE_HAS_BLE        1
#define CORE_HAS_THREAD     1
#define CORE_HAS_AUDIO      1
#define CORE_HAS_DAC        0
#define CORE_HAS_I3C        0
#define CORE_HAS_SPI        1
#define CORE_HAS_RNG        1
#define CORE_HAS_OTP        1
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0

#elif defined(STM32H523xx)
#define CORE_HAS_USB        1
#define CORE_HAS_BLE        0
#define CORE_HAS_THREAD     0
#define CORE_HAS_AUDIO      0
#define CORE_HAS_DAC        1
#define CORE_HAS_I3C        1
#define CORE_HAS_SPI        1
#define CORE_HAS_RNG        1
#define CORE_HAS_OTP        0
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0

#else
/* No STM32 part defined: host builds of SDK sources (tools/test_core_scope.py
 * and the like). Same answers the old part-name guards gave with no part:
 * features gated on "is one of these parts" are absent, those gated on "is
 * not the L011" are present. */
#define CORE_HAS_USB        0
#define CORE_HAS_BLE        0
#define CORE_HAS_THREAD     0
#define CORE_HAS_AUDIO      0
#define CORE_HAS_DAC        0
#define CORE_HAS_I3C        0
#define CORE_HAS_SPI        1
#define CORE_HAS_RNG        0
#define CORE_HAS_OTP        0
#define CORE_HAS_DWT        1
#define CORE_HAS_SMALL_RAM  0
#endif

#endif /* PORT_CAPS_H */
