/**
 * core_watchdog.h — Independent Watchdog (IWDG)
 *
 * The IWDG runs on its own LSI clock (32 kHz nominal, 37 kHz on the L0),
 * independent of SYSCLK. Once started, it CANNOT be stopped — only a full
 * MCU reset disables it. If your code doesn't call core_watchdog_feed()
 * before the timeout, the MCU resets.
 *
 * It also keeps counting in Stop and Standby (see ll_iwdg.h). core_power.h
 * reads the timeout recorded here and sleeps in fed chunks; Standby, which
 * can't be chunked, refuses to outlast half the timeout.
 *
 * On ROM-DFU builds (L4/H5), feeding also retires the brick-recovery strike
 * counter once the app has run healthy for a while — see core_recovery.h.
 *
 * Typical usage:
 *
 *   #include "core_watchdog.h"
 *
 *   if (core_watchdog_caused_reset()) {
 *       // handle recovery...
 *   }
 *
 *   core_watchdog_start(2000);   // 2-second timeout
 *
 *   while (1) {
 *       do_work();
 *       core_watchdog_feed();
 *   }
 *
 * @studio category watchdog label=Core.Watchdog icon=🐕
 *
 * @studio coverage
 *   id:    watchdog
 *   name:  Watchdog — independent watchdog (IWDG)
 *   blurb: Auto-prescaler IWDG with millisecond timeouts (~100 ms to
 *          ~28 s). Tier 2 covers start + feed; the cause-of-reset and
 *          flag-clear helpers are Tier 1 (escape-to-C, typically used
 *          once at boot). Once started, the watchdog can't be stopped
 *          — the only way out is a full MCU reset.
 */

#ifndef CORE_WATCHDOG_H
#define CORE_WATCHDOG_H

#include "core_types.h"   /* CORE_INLINE, uint32_t */
#include "core_reset.h"  /* core_watchdog_caused_reset() is core_reset_reason() */

/* Timeout (ms) of the IWDG this firmware started, or 0 if it hasn't started
 * one. Set by core_watchdog_start(); defined in core_watchdog.c. The IWDG has
 * no "running" status bit on the L0/L4, so software has to remember. */
extern volatile uint32_t _core_watchdog_timeout_ms;

/**
 * Start the independent watchdog with a timeout in milliseconds.
 * Selects the best prescaler/reload combination automatically.
 *
 * Common values: 1000, 2000, 5000, 10000 (max ~28000).
 *
 * WARNING: Once started, the IWDG cannot be stopped, and it keeps running in
 * Stop and Standby. core_stop_for() wakes to feed it; core_standby_for()
 * refuses sleeps longer than half the timeout.
 *
 * @studio expose category=watchdog name=start
 * @studio twin full
 * @param timeout_ms [100..28000] Timeout in milliseconds before reset.
 */
CORE_INLINE void core_watchdog_start(uint32_t timeout_ms);

/** Returns 1 if this firmware has started the watchdog (core_watchdog_start). */
CORE_INLINE int core_watchdog_running(void);

/**
 * The longest a sleep may run between feeds while the watchdog runs: half
 * its timeout, in ms. 0 when the watchdog isn't running. The RTC that times
 * the sleep and the IWDG share the LSI, so the margin holds even when the
 * LSI is far from nominal (the L0's is 26-56 kHz).
 */
CORE_INLINE uint32_t core_watchdog_sleep_chunk_ms(void);

#if defined(THREAD_ENABLED)
void core_thread_service_tick(void);   /* sdk/core/core_thread.c */
#endif

/**
 * Feed the watchdog. Must be called before the timeout expires.
 *
 * On ROM-DFU builds this also retires the brick-recovery strike counter, once,
 * after the app has run for max(10 s, 2 x the timeout) — no call needed.
 *
 * Thread builds (Core.ST.W5): also services Thread (at most one scheduler
 * pass every 5 ms), so Thread callbacks can run inside this call.
 *
 * @studio expose category=watchdog name=feed
 * @studio twin full
 */
CORE_INLINE void core_watchdog_feed(void);

/** Check if the last reset was caused by a watchdog (IWDG or WWDG):
 *  core_reset_reason() == CORE_RESET_WATCHDOG. core_init() reads the RCC
 *  flags once, keeps the cause and clears them, so this gives the same answer
 *  for the whole run (core_reset.h). */
CORE_INLINE int core_watchdog_caused_reset(void);

/** Clear the RCC reset flags. core_init() already does this at boot, and the
 *  cause it kept stays, so this is only needed by code that skips core_init. */
CORE_INLINE void core_watchdog_clear_flags(void);

/** Freeze the IWDG while the core is halted under a debugger, so a breakpoint
 *  doesn't let the watchdog reset the chip out from under an SWD session.
 *  Firmware-side so it holds for any probe/toolchain (rev b exposes SWD on L4). */
CORE_INLINE void core_watchdog_debug_freeze(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="No DSL access to caused_reset / clear_flags"
//   Both stay C-only. The DSL reads the same answer from
//   Core.Power.reset_reason() (2 = watchdog), which core_init() keeps for
//   the whole run, so there is no "before studio_start" phase to model.
//
// @studio unsupported tier=1 value=L title="No window watchdog (WWDG)"
//   STM32 also has a windowed watchdog (must feed within a window,
//   not just before the deadline). Useful for catching feed-too-fast
//   bugs. Not wrapped here.

/* The port's bodies of the functions above. */
#include "port_core_watchdog.h"

#endif /* CORE_WATCHDOG_H */
