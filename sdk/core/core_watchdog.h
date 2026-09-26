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
 *       core_watchdog_clear_flags();
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

#include "ll_iwdg.h"
#include "ll_systick.h" /* _systick_ticks: uptime for the strike-counter auto-clear */
#include "hal_dfu.h"  /* recovery stash: lets caused_reset() survive core_init's RMVF clear */

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
static inline void core_watchdog_start(uint32_t timeout_ms)
{
    /* LSI = ll_lsi_hz(): 32 kHz nominal; on the L0 measured once at the
       first call (it is untrimmed, 26-56 kHz), so a 5 s timeout is 5 s rather
       than anything from 3.3 to 7.1 s. Pick the smallest prescaler whose
       12-bit reload (1..4096 ticks) fits.
       tick_ms = psc * 1000 / LSI  →  reload = timeout_ms * LSI / (psc * 1000).
       In Hz, not kHz: a measured 36.99 kHz as "36" would be 2.7 % short. */
    static const uint32_t psc_vals[] = { 4, 8, 16, 32, 64, 128, 256 };
    static const uint32_t psc_regs[] = {
        LL_IWDG_PSC_4, LL_IWDG_PSC_8, LL_IWDG_PSC_16, LL_IWDG_PSC_32,
        LL_IWDG_PSC_64, LL_IWDG_PSC_128, LL_IWDG_PSC_256
    };
    const uint32_t lsi_hz = ll_lsi_hz();

    /* The longest the hardware can do is 4096 x 256 ticks (~40 s at the L0's
     * slowest LSI), and 60 s x 70 kHz still fits in 32 bits. */
    if (timeout_ms > 60000UL) timeout_ms = 60000UL;

    for (int i = 0; i < 7; i++) {
        /* (The earlier form divided by an extra 1000, making every timeout
         *  ~1000x too short.) */
        uint32_t reload = (timeout_ms * lsi_hz) / (psc_vals[i] * 1000UL);
        if (reload == 0) reload = 1;
        if (reload <= 4096) {
            ll_iwdg_init(psc_regs[i], reload - 1);
            _core_watchdog_timeout_ms = (reload * psc_vals[i] * 1000UL) / lsi_hz;
            if (_core_watchdog_timeout_ms == 0) _core_watchdog_timeout_ms = 1;
            return;
        }
    }
    /* Fallback: max timeout (4096 x 256 LSI ticks: ~32.8 s at 32 kHz,
     * ~28.3 s at the L0's nominal 37 kHz) */
    ll_iwdg_init(LL_IWDG_PSC_256, 4095);
    _core_watchdog_timeout_ms = (4096UL * 256UL * 1000UL) / lsi_hz;
}

/** Returns 1 if this firmware has started the watchdog (core_watchdog_start). */
static inline int core_watchdog_running(void)
{
    return _core_watchdog_timeout_ms != 0;
}

/**
 * The longest a sleep may run between feeds while the watchdog runs: half
 * its timeout, in ms. 0 when the watchdog isn't running. The RTC that times
 * the sleep and the IWDG share the LSI, so the margin holds even when the
 * LSI is far from nominal (the L0's is 26-56 kHz).
 */
static inline uint32_t core_watchdog_sleep_chunk_ms(void)
{
    uint32_t t = _core_watchdog_timeout_ms;
    if (t == 0) return 0;
    return (t >= 2UL) ? t / 2UL : 1UL;
}

/**
 * Feed the watchdog. Must be called before the timeout expires.
 *
 * On ROM-DFU builds this also retires the brick-recovery strike counter, once,
 * after the app has run for max(10 s, 2 x the timeout) — no call needed.
 *
 * @studio expose category=watchdog name=feed
 * @studio twin full
 */
static inline void core_watchdog_feed(void)
{
    ll_iwdg_refresh();
#if defined(DFU_STRIKE_TAG_ADDR) && defined(ROM_DFU)
    /* (ROM_DFU only: that is where core_init() keeps the count, and where the
     * linker reserves these SRAM words; elsewhere they are the stack top.)
     * Brick recovery (core_recovery.h): core_init() counts consecutive
     * watchdog resets and parks the Core in the ROM bootloader at the limit.
     * Nothing used to zero the count on a healthy run, so three unrelated
     * watchdog resets without a power cycle parked a working board. An app
     * that is still feeding after max(10 s, 2 x timeout) of uptime is healthy:
     * zero it here, once. While the count is 0 (every boot not preceded by a
     * watchdog reset, and every feed after the clear) this costs one SRAM load
     * and a compare. Uptime includes time in core_stop_for(). */
    if (DFU_STRIKE_ADDR != 0UL && hal_recovery_valid()) {
        uint32_t healthy_ms = 2UL * _core_watchdog_timeout_ms;
        if (healthy_ms < 10000UL) healthy_ms = 10000UL;
        if (_systick_ticks >= healthy_ms) {
            hal_recovery_set_strikes(0);
        }
    }
#endif
}

/** Check if the last reset was caused by the watchdog.
 *  On ROM_DFU builds, core_init() reads and clears the hardware flag early (for
 *  the strike counter), stashing the cause in reserved SRAM — so prefer that
 *  when it's valid; otherwise fall back to the raw RCC_CSR flag. */
static inline int core_watchdog_caused_reset(void)
{
#ifdef DFU_STRIKE_TAG_ADDR
    if (hal_recovery_valid()) return (int)hal_recovery_stashed_cause();
#endif
    return ll_iwdg_caused_reset();
}

/** Clear all reset flags (call after checking cause). */
static inline void core_watchdog_clear_flags(void)
{
    ll_rcc_clear_reset_flags();
}

/** Freeze the IWDG while the core is halted under a debugger, so a breakpoint
 *  doesn't let the watchdog reset the chip out from under an SWD session.
 *  Firmware-side so it holds for any probe/toolchain (rev b exposes SWD on L4). */
static inline void core_watchdog_debug_freeze(void)
{
    /* DBG_IWDG_STOP is bit 12 everywhere; only the register moves. Note the two
     * Cortex-M33 parts do NOT put DBGMCU at 0xE0042000 — that is the CoreSight
     * CTI block on both (confirmed on a WBA55 by walking its ROM table). */
#if defined(STM32L011xx)
    /* RM0377 27.9.4: DBG_APB1_FZ is mapped at 0x40015808. */
    SET_BITS(REG32(0x40015808UL), (1UL << 12));
#elif defined(STM32L422xx)
    /* RM0394: DBGMCU_APB1FZR1 at 0xE0042008. */
    SET_BITS(REG32(0xE0042008UL), (1UL << 12));
#elif defined(STM32WBA55xx)
    /* RM0493 43.12.7: DBGMCU @ 0xE0044000, DBGMCU_APB1LFZR at offset 0x08.
     * The RM documents only the debugger base, so software access was verified
     * on hardware: firmware writes bit 12 and reads it back, and a 6 s halt
     * under the CoreProbe with a 2 s IWDG no longer resets the part. */
    SET_BITS(REG32(0xE0044008UL), (1UL << 12));
#elif defined(STM32H523xx)
    /* RM0481 59.12.4: DBGMCU is at 0xE00E4000 for the DEBUGGER and 0x44024000
     * for SOFTWARE; DBGMCU_APB1LFZR is at offset 0x08. Firmware must use the
     * software base. (This previously wrote 0xE004203C — the CTI block — so the
     * freeze never took effect on the H5.) */
    SET_BITS(REG32(0x44024008UL), (1UL << 12));
#endif
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="No DSL access to caused_reset / clear_flags"
//   These return / clear hardware flags that only matter on the very
//   first boot iteration. Exposing them needs a story for "before
//   studio_start runs" — Studio doesn't currently model that phase.
//
// @studio unsupported tier=1 value=L title="No window watchdog (WWDG)"
//   STM32 also has a windowed watchdog (must feed within a window,
//   not just before the deadline). Useful for catching feed-too-fast
//   bugs. Not wrapped here.

#endif /* CORE_WATCHDOG_H */
