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
 * refuses (returns HAL_ERROR) a Standby longer than half the timeout, because
 * nothing can feed the watchdog until main() runs again. Core.ST.H5 keeps its
 * previous behavior (no chunking) until its own low-power pass.
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

#include "ll_pwr.h"
#include "ll_rtc.h"
#include "ll_rcc.h"
#include "ll_iwdg.h"
#include "ll_systick.h"
#include "core_pad.h"

/* EXTI addresses come from ll_exti.h */
#include "ll_exti.h"

/* The watchdog: the sleeps below feed it and size their chunks from it. The
 * canonical watchdog API lives in core_watchdog.h (millisecond precision,
 * auto-prescaler); including it here also keeps the old re-export working. */
#include "core_watchdog.h"

/* Generated per project in core_init.c (PLL + SysTick). */
extern void core_clock_init(void);

#if !defined(STM32H523xx)

/* ============================================================
 * Internal: Stop entry/exit per family (L0, L4, WBA)
 * ============================================================ */

#if defined(STM32WBA55xx)
/* WBA55 Stop 1 (RM0493 §11.7.7): exit is on HSI16 in voltage range 2 (VOS is
 * cleared by hardware) with RCC_CFGR4.HDIV5 set (hclk5 = SYSCLK / 2). So:
 *  - before entry, FLASH_ACR.LATENCY >= 1 (§7.3.3) and RAMCFG_M1CR/M2CR
 *    WSC >= 1 (§6.3.4), which range 2 at 16 MHz needs;
 *  - after core_clock_init() is back in range 1, HDIV5 (and any 0-WS SRAM
 *    setting) is restored. The 2.4 GHz radio needs HDIV5 clear (§12.4.6), and
 *    the BLE stack clears RCC_CFGR4 at start-up.
 * core_clock_init() reprograms the flash latency for the configured clock. */
#define CORE_W5_RCC_CFGR4     REG32(RCC_BASE + 0x200UL)   /* RM0493 §12.8.51 */
#define CORE_W5_HDIV5         (1UL << 4)
#define CORE_W5_RAMCFG_M1CR   REG32(0x40026000UL)          /* RM0493 §6.6.1 */
#define CORE_W5_RAMCFG_M2CR   REG32(0x40026040UL)          /* RM0493 §6.6.4 */
#define CORE_W5_RAMCFG_WSC    (0x7UL << 16)
#define CORE_W5_PWR_VOSR      REG32(PWR_BASE + 0x0CUL)     /* RM0493 §11.10.4 */

typedef struct { uint32_t hdiv5, m1_wsc, m2_wsc; } core_stop1_ctx_t;

static inline void _core_stop1_enter_prep(core_stop1_ctx_t *c)
{
    c->hdiv5  = CORE_W5_RCC_CFGR4 & CORE_W5_HDIV5;
    c->m1_wsc = CORE_W5_RAMCFG_M1CR & CORE_W5_RAMCFG_WSC;
    c->m2_wsc = CORE_W5_RAMCFG_M2CR & CORE_W5_RAMCFG_WSC;
    if ((FLASH_ACR & 0xFUL) == 0) ll_flash_set_latency(1);
    if (!c->m1_wsc) MOD_BITS(CORE_W5_RAMCFG_M1CR, CORE_W5_RAMCFG_WSC, 1UL << 16);
    if (!c->m2_wsc) MOD_BITS(CORE_W5_RAMCFG_M2CR, CORE_W5_RAMCFG_WSC, 1UL << 16);
}

/* Call after core_clock_init(). Undoes the Stop 1 settings only once the core
 * is back in range 1 (VOS = 1, VOSRDY = 1); range 2 needs them as they are. */
static inline void _core_stop1_exit_restore(const core_stop1_ctx_t *c)
{
    uint32_t vosr = CORE_W5_PWR_VOSR;
    if (!(vosr & (1UL << 16)) || !(vosr & (1UL << 15))) return;
    if (!c->hdiv5) {
        CLR_BITS(CORE_W5_RCC_CFGR4, CORE_W5_HDIV5);
        (void)CORE_W5_RCC_CFGR4;
    }
    if (!c->m1_wsc) CLR_BITS(CORE_W5_RAMCFG_M1CR, CORE_W5_RAMCFG_WSC);
    if (!c->m2_wsc) CLR_BITS(CORE_W5_RAMCFG_M2CR, CORE_W5_RAMCFG_WSC);
}
#else
typedef struct { uint8_t unused; } core_stop1_ctx_t;
static inline void _core_stop1_enter_prep(core_stop1_ctx_t *c) { (void)c; }
static inline void _core_stop1_exit_restore(const core_stop1_ctx_t *c) { (void)c; }
#endif

/* Longest wake the RTCCLK/16 wakeup clock can time (16-bit reload), in ms:
 * ~32.8 s at 32 kHz, ~28.3 s on the 37 kHz L0. */
static inline uint32_t _core_rtc_wake_fine_max_ms(uint32_t rtc_hz)
{
    return (65536UL * 16UL * 1000UL) / rtc_hz;
}

/* Arm the RTC wakeup timer to fire `ms` from now. Up to ~28 s it counts
 * RTCCLK/16 (~0.5 ms steps) — no dependency on the calendar prescaler; above
 * that the 1 Hz ck_spre (callers only pass whole seconds there). Leaves WUTF
 * and the EXTI line clear, as Stop and Standby entry require. */
static inline void _core_rtc_wake_arm(uint32_t ms)
{
    uint32_t hz = ll_rtc_clk_hz();
    if (ms <= _core_rtc_wake_fine_max_ms(hz)) {
        uint32_t ticks = (ms * (hz / 16UL)) / 1000UL;
        if (ticks == 0) ticks = 1;
        if (ticks > 65536UL) ticks = 65536UL;
        ll_rtc_wakeup_config_raw(LL_RTC_WUCKSEL_DIV16, ticks - 1UL);
    } else {
        ll_rtc_wakeup_config(ms / 1000UL);
    }
#if defined(LL_RTC_WKUP_EXTI)
    /* L0/L4: the wakeup timer reaches the core on EXTI line 20, rising edge,
     * interrupt mode (RM0377 / RM0394 EXTI line tables). */
    SET_BITS(REG32(EXTI_BASE + 0x00UL), (1UL << LL_RTC_WKUP_EXTI));   /* IMR */
    SET_BITS(REG32(EXTI_BASE + 0x08UL), (1UL << LL_RTC_WKUP_EXTI));   /* RTSR */
    REG32(EXTI_BASE + 0x14UL) = (1UL << LL_RTC_WKUP_EXTI);            /* PR: clear */
#endif
}

/* Stop the wakeup timer and clear everything it left pending. */
static inline void _core_rtc_wake_disarm(void)
{
    ll_rtc_wakeup_disable();
    ll_rtc_wakeup_clear_flag();
#if defined(LL_RTC_WKUP_EXTI)
    REG32(EXTI_BASE + 0x14UL) = (1UL << LL_RTC_WKUP_EXTI);
#endif
}

/*
 * Sleep in Stop until the armed RTC wakeup timer fires (rtc != 0) and/or an
 * interrupt handler sets *edge. Returns 1 when *edge ended it.
 *
 * WFI only leaves Stop for an interrupt that is enabled in the NVIC (RM0377
 * Table 39, RM0493 Table 90), so the RTC wakeup vector is enabled — but with
 * PRIMASK set, so its handler never runs: there isn't one, and the weak
 * default is an infinite loop. A pending interrupt still ends WFI under
 * PRIMASK. If something other than the RTC ends Stop early, its handler runs
 * in a short window with the RTC vector masked and Stop is re-entered for the
 * rest of the timer.
 *
 * The same rule means WFI does nothing while the RTC vector is already
 * pending, so unless the application owns that vector, its pending bit is
 * cleared before every Stop entry. Nothing services it while it is masked,
 * and on the WBA it is shared with Alarm A: the pend an alarm left behind
 * (and one re-pended as the previous chunk ended) kept every WFI from
 * sleeping. Bench, 2026-09-25: core_stop_for(10) ran 315 M cycles at 32 MHz,
 * i.e. never left Run.
 */
static inline int _core_stop_wait(int rtc, volatile uint8_t *edge)
{
    int by_edge = 0;
    int rtc_irq_was_on = ll_nvic_irq_enabled(LL_RTC_WKUP_IRQn);
    uint32_t pm = ll_irq_save();

    for (;;) {
        if (rtc) {
            if (!rtc_irq_was_on) ll_nvic_clear_pending(LL_RTC_WKUP_IRQn);
            ll_nvic_enable_irq(LL_RTC_WKUP_IRQn);
        }
        if (rtc && ll_rtc_wakeup_flag()) break;
        if (edge && *edge) { by_edge = 1; break; }
        ll_pwr_stop();
        if (rtc && ll_rtc_wakeup_flag()) break;
        if (rtc) ll_nvic_disable_irq(LL_RTC_WKUP_IRQn);
        ll_irq_restore(pm);                 /* let the other handler run */
        __asm volatile ("isb" ::: "memory");
        pm = ll_irq_save();
    }
    if (rtc) {
        /* Mask the vector, then clear the source before the NVIC pending bit
         * (the EXTI/RTC request is level: clearing NVIC first would re-pend).
         * The clear is a posted APB write, so wait until it has landed — on
         * the WBA the request re-pended ~15 cycles after an immediate NVIC
         * clear (bench, 2026-09-25). */
        ll_nvic_disable_irq(LL_RTC_WKUP_IRQn);
        ll_rtc_wakeup_clear_flag();
#if defined(LL_RTC_WKUP_EXTI)
        REG32(EXTI_BASE + 0x14UL) = (1UL << LL_RTC_WKUP_EXTI);
        (void)REG32(EXTI_BASE + 0x14UL);
#endif
        for (uint32_t t = 1000UL; t && ll_rtc_wakeup_flag(); t--)
            ;
        __asm volatile ("dsb" ::: "memory");
        ll_nvic_clear_pending(LL_RTC_WKUP_IRQn);
        if (rtc_irq_was_on) ll_nvic_enable_irq(LL_RTC_WKUP_IRQn);
    }
    ll_irq_restore(pm);
    return by_edge;
}

/* Set by the pad callback core_stop_until_on_change() registers. Per
 * translation unit, like the static inline callback that sets it. */
static volatile uint8_t _core_stop_pad_edge __attribute__((unused));
static inline void _core_stop_pad_cb(void *ctx)
{
    (void)ctx;
    _core_stop_pad_edge = 1;
}

#endif /* !STM32H523xx */

/* ---- Simple API ---- */

/**
 * Sleep until any interrupt (CPU stopped, peripherals running).
 *
 * @studio expose category=power name=sleep
 * @studio twin full
 */
static inline void core_sleep(void)
{
    ll_pwr_sleep_wfi();
}

/**
 * Enter Stop mode for a number of seconds, then wake and restore clocks.
 * Uses the RTC wakeup timer (LSI). Returns after wake with PLL + SysTick
 * restored; core_millis() advances by the time slept, as the RTC measured it.
 * If the watchdog is running, sleeps in chunks of half its timeout and feeds
 * it in between.
 *
 * @studio expose category=power name=stop_for
 * @studio twin full
 * @param seconds [1..86400] Seconds to spend in Stop mode.
 */
static inline void core_stop_for(uint32_t seconds)
{
#if defined(STM32H523xx)
    /* Enable PWR + backup domain */
    ll_rcc_pwr_clk_enable();
    ll_pwr_enable_backup_access();

    /* Init RTC on LSI and set wakeup alarm */
    ll_rtc_init(0);
    ll_rtc_wakeup_config(seconds);

    /* Unmask RTC wakeup for Stop mode wake.
     * H5: RTC has direct NVIC interrupt (RTC_IRQn=2). Enable NVIC +
     *      EXTI line 19 (wakeup timer) for wakeup from Stop.
     *      Note: line 17 = RTC (non-secure, all events), line 19 = TAMP.
     *      RTC is a "direct" EXTI event — only IMR needed, no RTSR. */
    /* H5: EXTI line 17 = RTC non-secure (direct event, IMR only) */
    ll_nvic_set_priority(2, 0x30);  /* RTC_IRQn = 2 */
    ll_nvic_enable_irq(2);
    SET_BITS(REG32(EXTI_BASE + 0x80UL), (1UL << 17));  /* IMR1: line 17 */

    /* Enter Stop mode */
    ll_pwr_stop();

    /* --- Woke up --- */

    /* Clear RTC wakeup flag */
    ll_rtc_wakeup_clear_flag();
    ll_nvic_disable_irq(2);

    /* Restore PLL + SysTick */
    core_clock_init();
#else
    if (seconds == 0) return;
    if (seconds > 2000000UL) seconds = 2000000UL;     /* 2 x ms + a chunk fit 32 bits */

    uint32_t t0 = _systick_ticks;
    uint32_t want = seconds * 1000UL;
    uint32_t slept = 0, armed = 0;
    uint32_t chunk_max = core_watchdog_sleep_chunk_ms();   /* 0: no watchdog */
    core_stop1_ctx_t s1;

    ll_rtc_ensure();                     /* running RTC, calendar untouched */
    /* Time is measured on the RTC calendar, not assumed from what was armed:
     * a chunk that ends early is made up by the next one, and core_millis()
     * gets what was actually slept. One calendar tick (1 / (PREDIV_S + 1) s)
     * short counts as done. If the calendar stops advancing, the timer's own
     * total ends the sleep at twice the request. */
    uint32_t tick_ms = 1000UL / ((RTC_PRER & 0x7FFFUL) + 1UL) + 1UL;
    ll_rtc_resync();
    uint32_t mark = ll_rtc_ms_of_day();
    _core_stop1_enter_prep(&s1);

    while (slept + tick_ms <= want && armed < 2UL * want) {
        uint32_t ms = want - slept;
        if (chunk_max && ms > chunk_max) ms = chunk_max;
        if (ms > 65536000UL) ms = 65536000UL;       /* 16-bit reload at 1 Hz */
        core_watchdog_feed();
        _core_rtc_wake_arm(ms);
        (void)_core_stop_wait(1, (volatile uint8_t *)0);
        armed += ms;
        ll_rtc_resync();                 /* calendar shadows are stale after Stop */
        uint32_t now = ll_rtc_ms_of_day();
        slept += (now + 86400000UL - mark) % 86400000UL;
        mark = now;
    }
    _core_rtc_wake_disarm();
    core_watchdog_feed();
    if (slept + tick_ms <= want) slept = armed;     /* the calendar never moved */

    core_clock_init();                   /* PLL + SysTick (SysTick restarts at 0) */
    _core_stop1_exit_restore(&s1);
    ll_rtc_resync();
    _systick_ticks += t0 + slept;
#endif
}

/**
 * Enter Stop mode until a GPIO edge occurs on the given pad.
 * Configures the pad as input, enables EXTI, enters Stop, and restores
 * clocks on wake. Returns after the edge is detected (right away if the pad
 * can't take an edge interrupt). If the watchdog is running, wakes every half
 * timeout to feed it and goes back to sleep.
 *
 * @param pad   Tile pad number
 * @param edge  EDGE_FALLING, EDGE_RISING, or EDGE_BOTH
 */
static inline void core_stop_until_on_change(uint8_t pad, uint32_t edge)
{
#if defined(STM32H523xx)
    core_pad_on_change(pad, edge, (hal_callback_t)0, (void *)0);

    ll_pwr_stop();

    core_clock_init();
#else
    uint32_t t0 = _systick_ticks, slept = 0;
    uint32_t chunk = core_watchdog_sleep_chunk_ms();
    core_stop1_ctx_t s1;

    _core_stop_pad_edge = 0;
    if (core_pad_on_change(pad, edge, _core_stop_pad_cb, (void *)0) != HAL_OK)
        return;

    _core_stop1_enter_prep(&s1);
    if (chunk) {
        ll_rtc_ensure();
        for (;;) {
            core_watchdog_feed();
            _core_rtc_wake_arm(chunk);
            if (_core_stop_wait(1, &_core_stop_pad_edge)) break;
            slept += chunk;
        }
        _core_rtc_wake_disarm();
        core_watchdog_feed();
    } else {
        (void)_core_stop_wait(0, &_core_stop_pad_edge);
    }

    core_clock_init();
    _core_stop1_exit_restore(&s1);
    if (chunk) ll_rtc_resync();          /* the RTC only runs if we started it */
    _systick_ticks += t0 + slept;        /* the unfinished last chunk isn't counted */
#endif
}

/**
 * Enter Standby mode with RTC wakeup after the given seconds.
 * On success it does not return — the MCU resets on wake. Check
 * core_woke_from_standby() at the top of main() to detect a Standby wake.
 *
 * Watchdog: the IWDG keeps counting through Standby and nothing feeds it until
 * main() runs again, and Standby can't be split into fed chunks. So while the
 * watchdog is running this refuses — returns HAL_ERROR without sleeping —
 * unless seconds is at most half the watchdog timeout (2 s with the default
 * 5 s watchdog). Use core_stop_for() for longer sleeps. (Core.ST.H5: no check.)
 *
 * @param seconds [1..65536] Seconds in Standby.
 * @return HAL_ERROR if refused because of the watchdog; does not return otherwise.
 */
static inline hal_status_t core_standby_for(uint32_t seconds)
{
#if defined(STM32H523xx)
    ll_rcc_pwr_clk_enable();
    ll_pwr_enable_backup_access();

    ll_rtc_init(0);
    ll_rtc_wakeup_config(seconds);

    SET_BITS(REG32(EXTI_BASE + 0x80UL), (1UL << 17));  /* IMR1: line 17 (RTC) */

    ll_pwr_standby();
    __builtin_unreachable();
#else
    if (seconds == 0) seconds = 1;
    if (seconds > 65536UL) seconds = 65536UL;

    uint32_t chunk = core_watchdog_sleep_chunk_ms();
    if (chunk && seconds * 1000UL > chunk)
        return HAL_ERROR;                /* the watchdog would reset us first */

    ll_rtc_ensure();
    core_watchdog_feed();
    _core_rtc_wake_arm(seconds * 1000UL);   /* also clears WUTF + EXTI line */

    /* Standby is skipped if an interrupt is pending at WFI (RM0377 Table 40,
     * RM0493 Table 92) and ll_pwr_standby() then spins until the watchdog
     * fires. The 1 ms SysTick is the likely one: stop it and drop a pending
     * tick. (SysTick is restarted by core_clock_init() after the reset.) */
    SYSTICK_CSR = 0;
    REG32(0xE000ED04UL) = (1UL << 25);    /* SCB_ICSR: PENDSTCLR */

    ll_pwr_standby();                    /* clears the PWR wake flags; no return */
    __builtin_unreachable();
#endif
}

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
static inline void core_standby_until_on_change(uint8_t pad, uint32_t edge)
{
#if defined(STM32L422xx)
    if (core_watchdog_running()) return;

    /* Resolve pad to GPIO */
    hal_pad_gpio_t g = hal_pad_lookup(pad);
    if (!g.port) return;

    /* Match GPIO to WKUP pin number (STM32L422 fixed mapping) */
    int wkup = -1;
    if (g.port == GPIOA && g.pin == 0) wkup = 0;       /* WKUP1 */
    else if (g.port == GPIOC && g.pin == 13) wkup = 1;  /* WKUP2 */
    else if (g.port == GPIOA && g.pin == 2) wkup = 3;   /* WKUP4 */
    if (wkup < 0) return;  /* pad not a WKUP pin */

    ll_rcc_pwr_clk_enable();

    /* PWR_CR3: enable WKUPx (bits 0-4) */
    SET_BITS(REG32(PWR_BASE + 0x08UL), (1UL << wkup));

    /* PWR_CR4: set polarity. Bit=0 → rising edge wake, bit=1 → falling edge */
    if (edge == EDGE_FALLING)
        SET_BITS(REG32(PWR_BASE + 0x0CUL), (1UL << wkup));
    else
        CLR_BITS(REG32(PWR_BASE + 0x0CUL), (1UL << wkup));

    /* PWR_SCR (0x18, RM0394 §5.4.7): clear wakeup flag. (Was 0x14 = PWR_SR2.) */
    REG32(PWR_BASE + 0x18UL) = (1UL << wkup);

    ll_pwr_standby();
    __builtin_unreachable();
#else
    /* WKUP-pin standby not yet implemented on this core family. */
    (void)pad;
    (void)edge;
#endif
}

/* ---- Advanced API ---- */

/** Enter Stop mode (caller manages wakeup source + clock recovery). A running
 *  watchdog keeps counting and is not fed: wake within its timeout. On the
 *  WBA the caller also owns the Stop 1 wait-state/HDIV5 rules (RM0493 §11.7.7). */
static inline void core_stop(void)
{
    ll_pwr_stop();
}

/** Enter Standby (caller manages wakeup source). Does not return. A running
 *  watchdog keeps counting through Standby and resets the Core at its timeout. */
static inline void core_standby(void) __attribute__((noreturn));
static inline void core_standby(void)
{
    ll_pwr_standby();
    __builtin_unreachable();
}

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
static inline int core_woke_from_standby(void)
{
    return ll_pwr_woke_from_standby();
}

/** Clear the Standby wake flag. Call after core_woke_from_standby() returns 1. */
static inline void core_clear_standby_flag(void)
{
    ll_pwr_clear_standby_flag();
}

/**
 * Start the independent watchdog with a timeout in seconds (convenience).
 * For finer control use core_watchdog_start() from core_watchdog.h which
 * accepts milliseconds.
 */
static inline void core_watchdog_start_seconds(uint32_t seconds)
{
    core_watchdog_start(seconds * 1000);
}

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
// @studio unsupported tier=1 value=H title="Stop / Standby not bench-verified on Core.ST.L0 / Core.ST.W5"
//   The register maps behind stop_for / standby_for / RTC / backup were
//   corrected against RM0377 and RM0493 on 2026-09-25 (the L0 enabled the
//   LSE instead of the LSI and hung; the WBA wrote CCIPR instead of BDCR1,
//   had no RTC bus clock and a reserved Standby LPMS). Compile-only until
//   tests/hw-sleep-cycle passes on each board; Core.ST.L4 and Core.ST.H5
//   have been verified end-to-end before.
//
// @studio unsupported tier=1 value=M title="Standby can't outlast the watchdog"
//   The IWDG keeps counting in Standby (always on the L0, by option byte on
//   the L4/WBA) and nothing feeds it until main() runs again, so with the
//   watchdog on (the Studio default, 5 s) core_standby_for refuses anything
//   longer than half the timeout and returns HAL_ERROR, and
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

#endif /* CORE_POWER_H */
