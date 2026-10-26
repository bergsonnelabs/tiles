/**
 * core_power.h — Sleep and low-power modes
 *
 * Simple API:
 *   core_sleep()              — lightest, wakes on any interrupt
 *   core_stop_for(seconds)   — Stop + RTC wakeup, auto clock recovery
 *   core_standby_for(sec)  — Standby + RTC wakeup, full reset on wake
 *
 * Advanced:
 *   core_deep_sleep()         — enter Stop (caller manages wakeup + clock recovery)
 *   core_shutdown()           — enter Standby (caller manages wakeup)
 *
 * Watchdog: the IWDG keeps counting in Stop and Standby (always on the L0; on
 * the L4 and WBA with the option bytes as shipped). When core_watchdog_start()
 * has armed it, core_stop_for() and core_stop_until_on_change() sleep in
 * chunks of half the timeout and feed it in between, and core_standby_for()
 * refuses (returns CORE_ERROR) a Standby longer than half the timeout, because
 * nothing can feed the watchdog until main() runs again. On Core.ST.H5,
 * core_standby_for() does not check the watchdog yet.
 *
 * USB (Core.ST.L4): Stop turns the USB clock off, so while a USB host is awake
 * on the bus, core_stop_for() and core_stop_until_on_change() wait in Sleep
 * mode instead (CPU halted, USB answering: Studio, flash-serial and prints
 * keep working, and the timing is the same). They use real Stop when the bus
 * is suspended or there is no host (on battery); a host that resumes the bus
 * wakes the Core, it serves USB while the bus stays awake, then goes back to
 * Stop for the rest of the sleep. On USB power from a live host the sleep
 * current is therefore Sleep-mode current, not Stop current.
 *
 * USB (Core.ST.H5): the suspended-bus wake is not set up there, so once USB
 * has started these sleeps always wait in Sleep mode, never Stop. core_init()
 * starts USB on every H5 project, so in practice an H5 sleep is Sleep-mode
 * current with USB answering; timing and watchdog feeding are the same.
 *
 * @studio category power label=Core.Power icon=☾
 *
 * @studio coverage
 *   id:    power
 *   name:  Power — sleep / Stop / Standby
 *   page:  /docs/sdk/system
 *   blurb: Sleep, Stop, and Standby helpers with auto-clock-recovery on
 *          wake (Stop) or full reset (Standby), plus GPIO-edge /
 *          RTC-timer wakeup variants. Stop sleeps in fed chunks while the
 *          watchdog runs. Tier 2 exposes the recoverable variants —
 *          sleep, stop_for, woke_from_standby — to the DSL.
 *          The Standby family stays Tier 1 because it's `noreturn` on
 *          real hardware (resets on wake), which would deadlock the
 *          simulator. Backup-domain wakeup status lives here too; the
 *          watchdog re-export is for backward compatibility — canonical
 *          surface is in core_watchdog.h.
 */

#ifndef CORE_POWER_H
#define CORE_POWER_H

/* The port's LL headers, ahead of the rest as they always were. */
#include "port_core_power_types.h"
#include "core_pad.h"   /* EDGE_FALLING / EDGE_RISING / EDGE_BOTH */
#include "core_types.h"

/* The watchdog: the sleeps below feed it and size their chunks from it. The
 * canonical watchdog API lives in core_watchdog.h (millisecond precision,
 * auto-prescaler); including it here also keeps the old re-export working. */
#include "core_watchdog.h"

/* core_reset() and core_reset_reason() share this category (Core.Power). */
#include "core_reset.h"

/**
 * Bring up the clock tree for the project's `clock` level (oscillator, PLL,
 * flash wait states, bus prescalers) and start SysTick. Generated per project
 * by coregen into core_init.c and called by core_init(); call it again only
 * to restore the clocks by hand (core_stop_for already does after a wake).
 */
extern void core_clock_init(void);

/* ---- Simple API ---- */

/**
 * Sleep until any interrupt (CPU stopped, peripherals running).
 *
 * @studio expose category=power name=sleep
 * @studio twin full
 */
CORE_INLINE void core_sleep(void);

/**
 * Enter Stop mode for a number of seconds, then wake and restore clocks.
 * Uses the RTC wakeup timer (LSI). Returns after wake with PLL + SysTick
 * restored; core_millis() advances by the time slept, as the RTC measured it.
 * If the watchdog is running, sleeps in chunks of half its timeout and feeds
 * it in between. Core.ST.L4 with a USB host awake on the bus: waits in Sleep
 * mode so the host can still reach it (see the USB note at the top).
 *
 * @studio expose category=power name=stop_for
 * @studio twin full
 * @param seconds [1..86400] Seconds to spend in Stop mode.
 */
CORE_INLINE void core_stop_for(uint32_t seconds);

/**
 * Enter Stop mode until a GPIO edge on the given pad, or until `timeout_ms`
 * has passed (0 = no timeout). Configures the pad as input, arms its EXTI,
 * enters Stop, and restores clocks on wake. If the watchdog is running, wakes
 * every half timeout to feed it and goes back to sleep. Core.ST.L4 with a USB
 * host awake on the bus: waits in Sleep mode (see the USB note at the top).
 *
 * Already at the wake level: EXTI latches only edges that happen after it is
 * armed, so a line that is already at its wake level — a sensor INT held low
 * until serviced, for EDGE_FALLING — would never wake the Core. After arming,
 * this reads the pad and returns 1 at once if it is already low (EDGE_FALLING)
 * or high (EDGE_RISING). EDGE_BOTH always waits for a change.
 *
 * @param pad         Tile pad number
 * @param edge        EDGE_FALLING, EDGE_RISING, or EDGE_BOTH
 * @param timeout_ms  Longest time to stay asleep, measured on the RTC; 0 waits
 *                    for the edge however long it takes.
 * @return 1 = woken by the pad (or it was already at the wake level),
 *         0 = timed out, -1 = the pad can't take an edge interrupt.
 */
CORE_INLINE int core_stop_until_on_change_timeout(uint8_t pad, uint32_t edge,
                                                  uint32_t timeout_ms);

/**
 * Enter Stop mode until a GPIO edge occurs on the given pad (no timeout).
 * Returns right away if the pad can't take an edge interrupt, or if it is
 * already at the level the edge wakes on (see
 * core_stop_until_on_change_timeout()). If the watchdog is running, wakes
 * every half timeout to feed it and goes back to sleep.
 *
 * @param pad   Tile pad number
 * @param edge  EDGE_FALLING, EDGE_RISING, or EDGE_BOTH
 */
CORE_INLINE void core_stop_until_on_change(uint8_t pad, uint32_t edge);

/**
 * Enter Standby mode with RTC wakeup after the given seconds.
 * On success it does not return — the MCU resets on wake. Check
 * core_woke_from_standby() at the top of main() to detect a Standby wake.
 *
 * Watchdog: the IWDG keeps counting through Standby and nothing feeds it until
 * main() runs again, and Standby can't be split into fed chunks. So while the
 * watchdog is running this refuses — returns CORE_ERROR without sleeping —
 * unless seconds is at most half the watchdog timeout (2 s with the default
 * 5 s watchdog). Use core_stop_for() for longer sleeps. (Core.ST.H5: no check.)
 *
 * @param seconds [1..65536] Seconds in Standby.
 * @return CORE_ERROR if refused because of the watchdog; does not return otherwise.
 */
CORE_INLINE core_status_t core_standby_for(uint32_t seconds);

/**
 * Enter Standby mode until a GPIO edge on the given pad (via WKUP pin).
 * Does not return — MCU resets on wake. Not all pads support WKUP;
 * returns without entering Standby if the pad has no WKUP capability, or if
 * the watchdog is running (it would reset the Core long before most edges).
 *
 * On Core.ST.L4.2: pad 8 (PA0 = WKUP1) and pad 7 (PA2 = WKUP4).
 *
 * @param pad   Tile pad number
 * @param edge  EDGE_FALLING or EDGE_RISING
 */
CORE_INLINE void core_standby_until_on_change(uint8_t pad, uint32_t edge);

/* ---- Advanced API ---- */

/** Enter Stop mode (caller manages wakeup source + clock recovery). A running
 *  watchdog keeps counting and is not fed: wake within its timeout. On the
 *  WBA the caller also owns the Stop 1 wait-state/HDIV5 rules (RM0493 §11.7.7). */
CORE_INLINE void core_stop(void);

/** Enter Standby (caller manages wakeup source). Does not return. A running
 *  watchdog keeps counting through Standby and resets the Core at its timeout. */
CORE_INLINE void core_standby(void) __attribute__((noreturn));

/* Backward compatibility */
#define core_deep_sleep  core_stop
#define core_shutdown    core_standby

/**
 * Returns 1 if the MCU woke from Standby. The flag survives later resets
 * until core_clear_standby_flag() — call that once you've handled the wake.
 *
 * @studio expose category=power name=woke_from_standby returns=bool
 * @studio twin full
 */
CORE_INLINE int core_woke_from_standby(void);

/** Clear the Standby wake flag. Call after core_woke_from_standby() returns 1. */
CORE_INLINE void core_clear_standby_flag(void);

/**
 * Start the independent watchdog with a timeout in seconds (convenience).
 * For finer control use core_watchdog_start() from core_watchdog.h which
 * accepts milliseconds.
 */
CORE_INLINE void core_watchdog_start_seconds(uint32_t seconds);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="Standby family stays Tier 1 (noreturn)"
//   core_standby_for / core_standby reset on wake — they never return
//   on success and can't be safely exposed to the DSL,
//   since the simulator would deadlock on a call that never returns.
//   Recoverable variants (sleep, stop_for) are Tier 2; reaching Standby
//   from a DSL program needs an "escape-to-C" fallthrough or a separate
//   "would have entered Standby" event the simulator can model.
//
// @studio unsupported tier=2 value=M title="No wake-on-pad Tier 2"
//   core_stop_until_on_change blocks until a configured pad edge wakes
//   the chip. The DSL surface is straightforward in spirit but needs
//   the EXTI handler bridge to express "block until" cleanly across
//   the host-call boundary. Tracked alongside the broader DSL event-
//   model story.
//
// @studio unsupported tier=1 value=H title="Standby not bench-verified on Core.ST.L0 or Core.ST.W5; Core.ST.H5 sleep compile-only"
//   The register maps behind stop_for / standby_for / RTC / backup were
//   corrected against RM0377 and RM0493 on 2026-09-25. Stop with the
//   watchdog running passed tests/hw-sleep-cycle on Core.ST.L4 and
//   Core.ST.W5 (2026-09-25) and Core.ST.L0 (2026-09-26). Standby on the L0
//   and the W5 is compile-only. Core.ST.H5 moved to the same fed, chunked
//   Stop code on 2026-10-13 and has not run it on a board yet; its
//   core_standby_for() still has no watchdog check.
//
// @studio unsupported tier=1 value=M title="Standby can't outlast the watchdog"
//   The IWDG keeps counting in Standby (always on the L0, by option byte on
//   the L4/WBA) and nothing feeds it until main() runs again, so with the
//   watchdog on (the Studio default, 5 s) core_standby_for refuses anything
//   longer than half the timeout and returns CORE_ERROR, and
//   core_standby_until_on_change returns without sleeping. Long sleeps use
//   core_stop_for, which wakes to feed. Changing the IWDG_STDBY option byte
//   would lift this on the L4/WBA; the SDK doesn't touch option bytes.
//
// @studio unsupported tier=1 value=M title="WKUP-pin Standby is L4-only"
//   core_standby_until_on_change is implemented for STM32L422 only —
//   Core.ST.L4.2's PA0/PA2 WKUP pins. Other Cores hit the (void) fallback
//   and return without entering Standby.
//
// @studio unsupported tier=1 value=M title="No LPTIM (low-power timer) wakeup"
//   The SDK roadmap flags LPTIM as Tier 1 high-impact: it runs in Stop
//   without the RTC and gives finer wakeup granularity than seconds.
//   Not wrapped here yet.
//
// @studio unsupported tier=1 value=L title="No autonomous-mode peripherals (Core.ST.W5)"
//   The WBA's Stop2 autonomous mode (ADC / SPI / I2C / UART operating
//   while the CPU sleeps) is on the roadmap as Tier 3 — none of it is
//   surfaced through core_power yet.

/* The port's bodies of the functions above. */
#include "port_core_power.h"

#endif /* CORE_POWER_H */
