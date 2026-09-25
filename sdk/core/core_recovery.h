/**
 * core_recovery.h — watchdog-strike brick recovery (ROM-DFU builds)
 *
 * A soft-bricked Core (bad app that hangs before USB comes up) resets every
 * IWDG timeout but never enumerates, so it can't be reflashed. This counts
 * consecutive watchdog resets that never reached healthy operation; after
 * CORE_RECOVERY_STRIKE_LIMIT of them, core_init() stops launching the app and
 * drops to the ST ROM bootloader (0483:DF11) so it can always be reflashed.
 *
 * The strike state lives in the reserved-SRAM words next to the DFU magic
 * (hal_dfu.h): it survives a warm/IWDG reset and clears on a cold power-on, so
 * a power-cycle is always a fresh start. Only meaningful on ROM_DFU builds of a
 * USB-capable core (where hal_dfu.h defines the reserved addresses).
 *
 * Flow:
 *   note_boot()  → (core_init, pre-clock) read HW reset cause, stash it,
 *                  inc/reset strikes, clear RMVF
 *   over_limit() → (core_init, pre-clock) decide whether to bail to DFU
 *   clear        → automatic: core_watchdog_feed() zeroes the counter, once,
 *                  after max(10 s, 2 x the watchdog timeout) of uptime, so
 *                  transient resets on a healthy app never add up to a false
 *                  park. Works for any project that feeds the watchdog (Studio
 *                  output included) with no template changes. A bad app that
 *                  hangs before that point never feeds, so it still escalates.
 *                  core_recovery_clear() remains for apps that want it sooner.
 *
 * Before 2026-09-25 the clear was documented as the starter template's job,
 * but nothing called it (only tests/hw-watchdog-recovery): three watchdog
 * resets anywhere in a powered session parked a working Core in ROM DFU.
 */

#ifndef CORE_RECOVERY_H
#define CORE_RECOVERY_H

#include "core_watchdog.h"
#include "hal_dfu.h"

/* Consecutive watchdog resets (with no healthy boot in between) before we stop
 * launching the app and park in ROM DFU. ~N × the IWDG timeout to recover. */
#ifndef CORE_RECOVERY_STRIKE_LIMIT
#define CORE_RECOVERY_STRIKE_LIMIT 3u
#endif

#ifdef DFU_STRIKE_TAG_ADDR

/**
 * Account for this boot and return the running strike count.
 * Call once, very early in core_init(), before clocks/user code.
 */
static inline uint32_t core_recovery_note_boot(void)
{
    uint32_t strikes = hal_recovery_strikes();
    int wd = ll_iwdg_caused_reset();     /* read the raw HW cause (IWDGRSTF) */
    hal_recovery_stash_cause(wd);        /* preserve for core_watchdog_caused_reset() */
    strikes = wd ? (strikes + 1u) : 0u;  /* any non-watchdog reset = fresh start */
    hal_recovery_set_strikes(strikes);
    ll_rcc_clear_reset_flags();          /* RMVF, else the flag latches and miscounts */
    return strikes;
}

/** True once we've had enough consecutive watchdog resets to give up on the app. */
static inline int core_recovery_over_limit(uint32_t strikes)
{
    return strikes >= CORE_RECOVERY_STRIKE_LIMIT;
}

/**
 * Clear the strike counter now. core_watchdog_feed() already does this once
 * the app has run healthy for max(10 s, 2x the timeout); call this only to
 * declare health earlier than that.
 */
static inline void core_recovery_clear(void)
{
    hal_recovery_set_strikes(0);
}

#endif /* DFU_STRIKE_TAG_ADDR */

#endif /* CORE_RECOVERY_H */
