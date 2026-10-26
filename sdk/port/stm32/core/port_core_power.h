/**
 * port_core_power.h — STM32 port: the bodies of core_power.h
 *
 * Included at the end of core_power.h, after its prototypes; not for direct
 * use. Sleep over ll_pwr (WFI, Stop, Standby), the RTC wakeup timer over
 * ll_rtc and the raw EXTI lines it reaches the core on, the per-MCU Stop entry
 * and exit (WBA Stop 1 RAMCFG / HDIV5, H5 flash latency), Alarm A masking,
 * the USB-aware Stop/Sleep choice (L4, H5), the L4 WKUP-pin Standby and the
 * PWR Standby flag.
 */

#ifndef PORT_CORE_POWER_H
#define PORT_CORE_POWER_H

#include "ll_pwr.h"
#include "ll_rtc.h"
#include "ll_rcc.h"
#include "ll_iwdg.h"
#include "ll_systick.h"
#include "core_pad.h"      /* core_pad_on_change(), core_pad_read() */
#include "core_watchdog.h" /* core_watchdog_feed(), core_watchdog_sleep_chunk_ms() */

/* EXTI addresses come from ll_exti.h */
#include "ll_exti.h"

/* ============================================================
 * Internal: Stop entry/exit per family (L0, L4, WBA, H5)
 * ============================================================ */

#if defined(STM32WBA55xx)
/* WBA55 Stop 1 (RM0493 §11.7.7): exit is on HSI16 in voltage range 2 (VOS is
 * cleared by hardware) with RCC_CFGR4.HDIV5 set (hclk5 = SYSCLK / 2). So:
 *  - before entry, FLASH_ACR.LATENCY >= 1 (§7.3.3) and RAMCFG_M1CR/M2CR
 *    WSC >= 1 (§6.3.4), which range 2 at 16 MHz needs;
 *  - after core_clock_init() is back in range 1, HDIV5 (and any 0-WS SRAM
 *    setting) is restored. The 2.4 GHz radio needs HDIV5 clear (§12.4.6);
 *    core_clock_init() owns RCC_CFGR4 (HPRE5 keeps hclk5 <= 32 MHz on the
 *    PLL levels), and the BLE stack no longer touches it.
 * core_clock_init() reprograms the flash latency for the configured clock. */
#define CORE_W5_RCC_CFGR4     REG32(RCC_BASE + 0x200UL)   /* RM0493 §12.8.51 */
#define CORE_W5_HDIV5         (1UL << 4)
#define CORE_W5_RAMCFG_M1CR   REG32(0x40026000UL)          /* RM0493 §6.6.1 */
#define CORE_W5_RAMCFG_M2CR   REG32(0x40026040UL)          /* RM0493 §6.6.4 */
#define CORE_W5_RAMCFG_WSC    (0x7UL << 16)
#define CORE_W5_RCC_AHB1ENR   REG32(RCC_BASE + 0x088UL)    /* RM0493 §12.8.20 */
#define CORE_W5_RAMCFGEN      (1UL << 17)

/* RAMCFG reads as 0 and drops writes while its bus clock is off. */
static inline void _core_w5_ramcfg_clk_on(void)
{
    SET_BITS(CORE_W5_RCC_AHB1ENR, CORE_W5_RAMCFGEN);
    (void)CORE_W5_RCC_AHB1ENR;
}
#define CORE_W5_PWR_VOSR      REG32(PWR_BASE + 0x0CUL)     /* RM0493 §11.10.4 */

typedef struct { uint32_t hdiv5, m1_wsc, m2_wsc; } core_stop1_ctx_t;

static inline void _core_stop1_enter_prep(core_stop1_ctx_t *c)
{
    _core_w5_ramcfg_clk_on();
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
#elif defined(STM32H523xx)
/* H523 Stop 0 (RM0481 §10.8.5): exit is on the HSI (RCC_CFGR1.STOPWUCK = 0,
 * its reset value) in VOS3. The sleeps below run between chunks on that
 * clock, so the flash latency has to cover it: RM0481 Table 45 asks 3 wait
 * states for the HSI's 64 MHz in VOS3, and the HSI 64 MHz level runs at 2 in
 * VOS2. core_clock_init() sets the level's own wait states on the way out. */
typedef struct { uint8_t unused; } core_stop1_ctx_t;
static inline void _core_stop1_enter_prep(core_stop1_ctx_t *c)
{
    (void)c;
    if (ll_flash_get_latency() < 3UL) ll_flash_set_latency(3UL);
}
static inline void _core_stop1_exit_restore(const core_stop1_ctx_t *c) { (void)c; }
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
#elif defined(STM32H523xx)
    /* H5: ll_rtc_wakeup_config_raw() doesn't clear a stale WUTF there, so
     * clear it now (RTC_SCR.CWUTF, RM0481 §46.6.23); the timer just restarted,
     * so it can't set again before `ms`. The RTC is a direct event on EXTI
     * line 17 (RM0481 Table 150), which wakes the CPU from Stop once
     * EXTI_IMR1 (offset 0x080, §18.6.20) unmasks it. */
    ll_rtc_wakeup_clear_flag();
    SET_BITS(REG32(EXTI_BASE + 0x80UL), (1UL << 17));
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
 * Alarm A shares the RTC wakeup vector on the WBA (RTC_IRQn 2 carries both)
 * and on the L0 (RTC_IRQn 2; the alarm reaches it through EXTI line 17, the
 * wakeup timer through line 20). core_rtc_set_alarm() always sets ALRAIE, so
 * once an alarm has matched, its request (level, held by ALRAF) pends that
 * vector again the moment the loop below clears it: every WFI returned at
 * once and the rest of a core_stop_for() ran as a busy-wait in Run mode.
 * While the vector is ours (the application hasn't enabled it), the alarm's
 * path into it is masked for the sleep: ALRAIE on the WBA (RTC_CR bit 12,
 * RM0493 §36.6.7), EXTI IMR line 17 on the L0 (RM0377 §12.5.1). ALRAF still
 * sets, so core_rtc_alarm_fired() reports the match afterwards. The L4 has a
 * separate RTC_Alarm vector (41) and needs nothing. The H5 is like the WBA:
 * one RTC vector (2) for all RTC events, ALRAIE at RTC_CR bit 12 (RM0481
 * §46.6.7).
 */
static inline uint32_t _core_stop_alarm_mask(void)
{
#if defined(STM32WBA55xx) || defined(STM32H523xx)
    if (!(RTC_CR & LL_RTC_CR_ALRAIE)) return 0;
    ll_rtc_unlock();
    CLR_BITS(RTC_CR, LL_RTC_CR_ALRAIE);
    ll_rtc_lock();
    return 1;
#elif defined(STM32L011xx)
    if (!(REG32(EXTI_BASE + 0x00UL) & (1UL << 17))) return 0;
    CLR_BITS(REG32(EXTI_BASE + 0x00UL), (1UL << 17));        /* IMR: line 17 */
    return 1;
#else
    return 0;
#endif
}

static inline void _core_stop_alarm_unmask(uint32_t masked)
{
    if (!masked) return;
#if defined(STM32WBA55xx) || defined(STM32H523xx)
    ll_rtc_unlock();
    SET_BITS(RTC_CR, LL_RTC_CR_ALRAIE);
    ll_rtc_lock();
#elif defined(STM32L011xx)
    SET_BITS(REG32(EXTI_BASE + 0x00UL), (1UL << 17));
#endif
}

/*
 * USB on the L4 (hal_usb_cdc.c). Stop turns HSI48 off (RCC_CRRCR.HSI48ON,
 * RM0394 §6.4.30), so a Core in Stop stops answering a host that has the bus
 * awake: the host's next transfer fails and it may reset or drop the port.
 * So while a host is awake on the bus, the sleeps below wait in Sleep mode
 * instead (CPU halted; clocks, SysTick and the USB interrupt running; timing,
 * wake sources and watchdog chunks unchanged), and use Stop only while the bus
 * is suspended (host asleep or gone, or no host at all). The host's resume or
 * reset then ends Stop through the USB wakeup (EXTI line 17), the clocks come
 * back before the USB interrupt runs, the USB is serviced for as long as the
 * bus stays awake, and the rest of the sleep resumes in Stop once it is
 * suspended again. Weak: a build without the USB driver links without it.
 *
 * The H5's hal_usb_cdc_stop_allowed() allows Stop only before USB has
 * started: the suspended-bus wake chain isn't set up there, so a running USB
 * always waits in Sleep.
 */
#if defined(STM32L422xx) || defined(STM32H523xx)
extern int hal_usb_cdc_started(void) __attribute__((weak));
extern int hal_usb_cdc_stop_allowed(void) __attribute__((weak));
#endif

/* 1: enter Stop. 0: a USB host is awake on the bus, wait in Sleep. */
static inline int _core_stop_usb_ok(void)
{
#if defined(STM32L422xx) || defined(STM32H523xx)
    return !hal_usb_cdc_stop_allowed || hal_usb_cdc_stop_allowed();
#else
    return 1;
#endif
}

/* After Stop: the USB interrupt (possibly what ended it) can only answer at
 * full clock with HSI48 back; the CRS resumes with HSI48 (RM0394 §7.5).
 * core_clock_init() restores both, before the handler gets to run. */
static inline void _core_stop_usb_exit(void)
{
#if defined(STM32L422xx)
    if (hal_usb_cdc_started && hal_usb_cdc_started()) core_clock_init();
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
    uint32_t alarm_masked = (rtc && !rtc_irq_was_on) ? _core_stop_alarm_mask() : 0;
    uint32_t pm = ll_irq_save();

    for (;;) {
        if (rtc) {
            if (!rtc_irq_was_on) ll_nvic_clear_pending(LL_RTC_WKUP_IRQn);
            ll_nvic_enable_irq(LL_RTC_WKUP_IRQn);
        }
        if (rtc && ll_rtc_wakeup_flag()) break;
        if (edge && *edge) { by_edge = 1; break; }
        if (_core_stop_usb_ok()) {
            ll_pwr_stop();
            _core_stop_usb_exit();
        } else {
            ll_pwr_sleep_wfi();             /* USB host awake: see above */
        }
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
    /* Back on; a pend it raises now stays pending with the vector masked,
     * and the next Stop masks it again. */
    _core_stop_alarm_unmask(alarm_masked);
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

/* After arming the wakeup timer: where the sleep's calendar time is measured
 * from. On the H5, ll_rtc_wakeup_config_raw() sets WUTE through RTC init mode,
 * and leaving init mode restarts the current calendar second, so the mark is
 * taken again after each arm (the under 1 ms spent arming is not counted). Elsewhere arming leaves the calendar alone and the mark
 * carries over from the previous chunk. */
static inline void _core_stop_mark_after_arm(uint32_t *mark)
{
#if defined(STM32H523xx)
    ll_rtc_resync();
    *mark = ll_rtc_ms_of_day();
#else
    (void)mark;
#endif
}

static inline void core_sleep(void)
{
    ll_pwr_sleep_wfi();
}

static inline void core_stop_for(uint32_t seconds)
{
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
        _core_stop_mark_after_arm(&mark);
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
}

/* True when `pad` already sits at the level `edge` wakes on: low for
 * EDGE_FALLING, high for EDGE_RISING. EDGE_BOTH has no wake level. */
static inline int _core_stop_pad_at_wake_level(uint8_t pad, uint32_t edge)
{
    if (edge == EDGE_FALLING) return core_pad_read(pad) == 0;
    if (edge == EDGE_RISING)  return core_pad_read(pad) != 0;
    return 0;
}

static inline int core_stop_until_on_change_timeout(uint8_t pad, uint32_t edge,
                                                    uint32_t timeout_ms)
{
    uint32_t t0 = _systick_ticks, slept = 0, mark = 0, tick_ms = 0;
    uint32_t chunk = core_watchdog_sleep_chunk_ms();
    int use_rtc = (chunk != 0) || (timeout_ms != 0);
    int woke = 0;
    core_stop1_ctx_t s1;

    _core_stop_pad_edge = 0;
    if (core_pad_on_change(pad, edge, _core_stop_pad_cb, (void *)0) != HAL_OK)
        return -1;
    /* Armed: an edge from here on is latched. Now the level. */
    if (_core_stop_pad_at_wake_level(pad, edge))
        return 1;

    _core_stop1_enter_prep(&s1);
    if (use_rtc) {
        ll_rtc_ensure();
        /* Time on the RTC calendar, as core_stop_for() measures it. One
         * calendar tick short of the timeout counts as done. */
        tick_ms = 1000UL / ((RTC_PRER & 0x7FFFUL) + 1UL) + 1UL;
        ll_rtc_resync();
        mark = ll_rtc_ms_of_day();
        for (;;) {
            uint32_t ms = chunk ? chunk : 65536000UL;       /* 16-bit reload at 1 Hz */
            if (timeout_ms) {
                if (slept + tick_ms > timeout_ms) break;    /* timed out */
                if (timeout_ms - slept < ms) ms = timeout_ms - slept;
            }
            core_watchdog_feed();
            _core_rtc_wake_arm(ms);
            _core_stop_mark_after_arm(&mark);
            woke = _core_stop_wait(1, &_core_stop_pad_edge);
            ll_rtc_resync();                 /* calendar shadows are stale after Stop */
            uint32_t now = ll_rtc_ms_of_day();
            slept += (now + 86400000UL - mark) % 86400000UL;
            mark = now;
            if (woke) break;
        }
        _core_rtc_wake_disarm();
        core_watchdog_feed();
    } else {
        (void)_core_stop_wait(0, &_core_stop_pad_edge);
        woke = 1;
    }

    core_clock_init();
    _core_stop1_exit_restore(&s1);
    if (use_rtc) ll_rtc_resync();        /* the RTC only runs if we started it */
    _systick_ticks += t0 + slept;        /* without an RTC nothing measured the sleep */
    return woke;
}

static inline void core_stop_until_on_change(uint8_t pad, uint32_t edge)
{
    (void)core_stop_until_on_change_timeout(pad, edge, 0);
}

static inline core_status_t core_standby_for(uint32_t seconds)
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

static inline void core_stop(void)
{
    ll_pwr_stop();
}

static inline void core_standby(void)
{
    ll_pwr_standby();
    __builtin_unreachable();
}

static inline int core_woke_from_standby(void)
{
    return ll_pwr_woke_from_standby();
}

static inline void core_clear_standby_flag(void)
{
    ll_pwr_clear_standby_flag();
}

static inline void core_watchdog_start_seconds(uint32_t seconds)
{
    core_watchdog_start(seconds * 1000);
}

#endif /* PORT_CORE_POWER_H */
