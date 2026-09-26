/**
 * hw-lsi-cal — bench test for the Core.ST.L0's LSI calibration
 * (ll_lsi_hz(), sdk/hal/hal_lsi.c), one build per clock level.
 *
 * The L0's LSI is untrimmed (26 / 38 / 56 kHz, DS Table 39). ll_lsi_hz()
 * measures it once against SYSCLK with TIM21 input capture; the RTC
 * prescaler, the RTC wakeup timer and the IWDG reload are computed from it.
 * This test times that measurement, then checks it against 10 RTC seconds
 * timed in SYSCLK cycles (SysTick's counter, not core_millis(): at `low`
 * the 1 ms reload is truncated, 1048 for 1048.576 cycles).
 *
 *   T1 measure   ll_lsi_hz() is in the DS band, took under 10 ms, left TIM21
 *                off and in reset state, and a second call is the cached value
 *   T2 agree     the measurement agrees with the LSI implied by 10 RTC seconds
 *                to within 0.5 % (both against SYSCLK)
 *   T3 rtc       with the calibrated PREDIV_S an RTC second is within 0.5 % of
 *                SYSCLK time
 *   T4 iwdg      core_watchdog_start(5000) programs a timeout that is 5 s
 *                within 0.5 % at the implied LSI
 *   T5 fire      and it really fires then: boot 1 arms it and stops feeding;
 *                boot 2 finds a watchdog reset 5 s +-1.5 % later by the
 *                calibrated RTC. The reset clears LSION (RM0377 §7.3.20), so
 *                the RTC stands still from the reset until boot 2 turns the
 *                LSI back on: what it measures is the watchdog period itself.
 *
 * The limits: one 3.5 ms reading scatters about +-0.1 %, the LSI wanders about
 * as much between 10 s windows, and a PREDIV_S step is 0.35 %. Uncalibrated,
 * the same numbers can be off by -30 % to +50 %.
 *
 * The first reading waits 200 ms after boot: probe-rs keeps the SWD busy for
 * ~20 ms after a reset, which holds the CPU off the capture flags.
 *
 * "Before" is recorded too: the same 10 s with the nominal PREDIV_S (288,
 * from 37 kHz), which is what the SDK programmed until now. Result about
 * 30 s after the reset (5 s starved, then 25 s) at `g_lsi_cal` (arm-none-eabi-nm build/hw-lsi-cal.elf).
 */

#include "core.h"
#include "core_rtc.h"
#include "core_watchdog.h"
#include "core_backup.h"

#if !defined(STM32L011xx)
#error "hw-lsi-cal is written for Core.ST.L0"
#endif

#ifndef REG32
#define REG32(a)       (*(volatile uint32_t *)(a))
#endif
#define SYST_LOAD      REG32(0xE000E014UL)
#define SYST_VAL       REG32(0xE000E018UL)
#define IWDG_PR_REG    REG32(0x40003004UL)
#define IWDG_RLR_REG   REG32(0x40003008UL)
#define APB2ENR        REG32(0x40021034UL)
#define TIM21_OR_REG   REG32(0x40010850UL)

#define T_MEASURE  (1u << 0)
#define T_AGREE    (1u << 1)
#define T_RTC      (1u << 2)
#define T_IWDG     (1u << 3)
#define T_FIRE     (1u << 4)
#define EXPECTED   0x1Fu

#define BKP_ARMED  0x57A2F00Du   /* BKP0: boot 1 left the watchdog starving */

#define N_SECONDS  10u

typedef struct {
    uint32_t magic;            /* 0x15C0CA1D once final */
    uint32_t verdict;          /* 0x600D600D, or 0xBAD000nn */
    uint32_t pass, fail;
    uint32_t level[2];
    uint32_t sysclk_hz;
    uint32_t lsi_hz;           /* ll_lsi_hz() (measured) */
    uint32_t measure_us;       /* its first call */
    uint32_t lsi_implied_hz;   /* from 10 RTC seconds with the calibrated PREDIV_S */
    int32_t  measure_err_ppm;  /* lsi_hz vs implied */
    uint32_t prediv_s_nominal; /* 288 */
    uint32_t prediv_s_cal;
    int32_t  rtc_err_ppm_before;   /* RTC second vs SYSCLK, nominal PREDIV_S */
    int32_t  rtc_err_ppm_after;    /* calibrated */
    uint32_t iwdg_pr, iwdg_rlr;
    uint32_t wd_timeout_ms;        /* what the SDK believes */
    int32_t  wd_err_ppm_before;    /* 5 s at the nominal reload (2890 x 64), at the implied LSI */
    int32_t  wd_err_ppm_after;     /* at the programmed reload */
    uint32_t tim21_after;          /* APB2ENR.TIM21EN | TIM21_OR after (both 0) */
    uint32_t wd_fire_ms;           /* T5: RTC ms from the last feed to the next boot */
    uint32_t wd_caused_reset;      /* T5: core_watchdog_caused_reset() on boot 2 */
} lsi_cal_result_t;

__attribute__((used)) volatile lsi_cal_result_t g_lsi_cal;

static uint32_t s_pass, s_fail;
static int s_wd_on;

static void mark(uint32_t t, int ok)
{
    if (ok) s_pass |= t; else s_fail |= t;
}

static void feed(void)
{
    if (s_wd_on) core_watchdog_feed();
}

/* SYSCLK cycles since boot, modulo 2^32 (differences are exact under ~2 min
 * at 32 MHz). */
static uint32_t now_cycles(void)
{
    uint32_t ms, val;
    do {
        ms = _systick_ticks;
        val = SYST_VAL;
    } while (ms != _systick_ticks);
    uint32_t load = SYST_LOAD;
    return ms * (load + 1u) + (load - val);
}

static uint32_t rtc_s(void)
{
    uint8_t h, m, s;
    core_rtc_get_time(&h, &m, &s);
    return (uint32_t)h * 3600u + (uint32_t)m * 60u + s;
}

/* RTC time of day in ms, subseconds included: SSR, then TR, then DR (reading
 * SSR locks TR / DR until DR is read, RM0377 §22.4.8). Waits for a fresh
 * shadow copy first: after a reset RSF is clear until the next one. */
static uint32_t rtc_ms(void)
{
    while (!(RTC_ISR & LL_RTC_ISR_RSF)) ;
    uint32_t ps = RTC_PRER & 0x7FFFu;
    uint32_t ss = RTC_SSR & 0xFFFFu, tr = RTC_TR;
    (void)RTC_DR;
    uint32_t h = ((tr >> 20) & 3u) * 10u + ((tr >> 16) & 0xFu);
    uint32_t m = ((tr >> 12) & 7u) * 10u + ((tr >> 8) & 0xFu);
    uint32_t sec = ((tr >> 4) & 7u) * 10u + (tr & 0xFu);
    return ((h * 3600u + m * 60u + sec) * 1000u) + ((ps - ss) * 1000u) / (ps + 1u);
}

/* SYSCLK cycles across N_SECONDS RTC seconds, edge to edge. */
static uint32_t rtc_cycles(void)
{
    uint32_t s0 = rtc_s();
    while (rtc_s() == s0) feed();
    uint32_t c0 = now_cycles(), a = rtc_s();
    while (rtc_s() - a < N_SECONDS) feed();
    return now_cycles() - c0;
}

static int32_t ppm(uint64_t got, uint64_t want)
{
    return (int32_t)((((int64_t)got - (int64_t)want) * 1000000LL) / (int64_t)want);
}

static void set_prediv_s(uint32_t s)
{
    ll_rtc_unlock();
    ll_rtc_enter_init();
    RTC_PRER = s;                          /* two writes, synchronous first */
    RTC_PRER = (127UL << 16) | s;
    ll_rtc_exit_init();
    ll_rtc_lock();
}

static uint32_t verdict_of(void)
{
    uint32_t f = s_fail | (EXPECTED & ~s_pass);
    if (!f) return 0x600D600Du;
    uint32_t n = 1;
    while (!(f & 1u)) { f >>= 1; n++; }
    return 0xBAD00000u | n;
}

int main(void)
{
    core_init();                /* config.json: watchdog off, so the first
                                   ll_lsi_hz() call is this test's */
    core_led_init();
    const uint64_t f = SYSCLK_HZ;
    g_lsi_cal.sysclk_hz = SYSCLK_HZ;
    const char *lv = CLOCK_LEVEL_NAME;
    uint32_t w[2] = { 0, 0 };
    for (uint32_t i = 0; i < 8 && lv[i]; i++) w[i / 4] |= (uint32_t)(uint8_t)lv[i] << (8 * (i % 4));
    g_lsi_cal.level[0] = w[0];
    g_lsi_cal.level[1] = w[1];

    core_delay_ms(200);        /* SWD quiet after the reset */

    if (core_backup_read(0) != BKP_ARMED) {
        /* Boot 1: arm the watchdog on the calibrated LSI, feed it once, note
         * the RTC time, and stop feeding. */
        (void)ll_lsi_hz();
        core_rtc_init();
        core_watchdog_clear_flags();
        core_watchdog_start(5000);
        core_watchdog_feed();
        core_backup_write(1, rtc_ms());
        core_backup_write(0, BKP_ARMED);
        while (1) ;
    }
    /* Boot 2: T5, before anything touches the RTC */
    {
        uint32_t t1 = rtc_ms(), t0 = core_backup_read(1);
        g_lsi_cal.wd_fire_ms = (t1 + 86400000u - t0) % 86400000u;
        g_lsi_cal.wd_caused_reset = (uint32_t)core_watchdog_caused_reset();
        core_watchdog_clear_flags();
        core_backup_write(0, 0);
        mark(T_FIRE, g_lsi_cal.wd_caused_reset &&
                     g_lsi_cal.wd_fire_ms >= 4925u && g_lsi_cal.wd_fire_ms <= 5075u);
    }

    /* T1 */
    uint32_t c0 = now_cycles();
    uint32_t lsi = ll_lsi_hz();
    uint32_t dc = now_cycles() - c0;
    g_lsi_cal.lsi_hz = lsi;
    g_lsi_cal.measure_us = (uint32_t)(((uint64_t)dc * 1000000u) / f);
    g_lsi_cal.tim21_after = (APB2ENR & (1UL << 2)) | TIM21_OR_REG;
    mark(T_MEASURE, lsi >= 26000u && lsi <= 56000u && lsi != LL_LSI_HZ &&
                    g_lsi_cal.measure_us < 10000u && g_lsi_cal.tim21_after == 0u &&
                    ll_lsi_hz() == lsi);

    /* Watchdog: what core_init() would have started, now from the measured LSI */
    core_watchdog_start(5000);
    s_wd_on = 1;
    g_lsi_cal.iwdg_pr = IWDG_PR_REG;
    g_lsi_cal.iwdg_rlr = IWDG_RLR_REG;
    g_lsi_cal.wd_timeout_ms = _core_watchdog_timeout_ms;

    /* RTC: calibrated (ll_rtc_init), then the old nominal PREDIV_S */
    core_rtc_init();
    g_lsi_cal.prediv_s_cal = RTC_PRER & 0x7FFFu;
    g_lsi_cal.prediv_s_nominal = (LL_LSI_HZ / 128u) - 1u;
    uint32_t cyc_after = rtc_cycles();
    set_prediv_s(g_lsi_cal.prediv_s_nominal);
    uint32_t cyc_before = rtc_cycles();
    set_prediv_s(g_lsi_cal.prediv_s_cal);

    const uint64_t want = f * N_SECONDS;
    g_lsi_cal.rtc_err_ppm_after = ppm(cyc_after, want);
    g_lsi_cal.rtc_err_ppm_before = ppm(cyc_before, want);

    /* LSI implied by the calibrated run: 128 x (PREDIV_S + 1) LSI cycles per
     * RTC second. */
    uint64_t implied = (128ULL * (g_lsi_cal.prediv_s_cal + 1u) * N_SECONDS * f + cyc_after / 2u)
                       / cyc_after;
    g_lsi_cal.lsi_implied_hz = (uint32_t)implied;
    g_lsi_cal.measure_err_ppm = ppm(lsi, implied);
    int32_t e = g_lsi_cal.measure_err_ppm;
    mark(T_AGREE, (e < 0 ? -e : e) <= 5000);
    e = g_lsi_cal.rtc_err_ppm_after;
    mark(T_RTC, (e < 0 ? -e : e) <= 5000);

    /* T4: the programmed IWDG period at the implied LSI, vs 5 s; and what the
     * nominal reload (5000 x 37 / 64 = 2890) would have given. */
    static const uint32_t psc_div[] = { 4, 8, 16, 32, 64, 128, 256 };
    uint64_t ticks = (uint64_t)(g_lsi_cal.iwdg_rlr + 1u) * psc_div[g_lsi_cal.iwdg_pr & 7u];
    g_lsi_cal.wd_err_ppm_after = ppm((ticks * 1000000ULL) / implied, 5000000ULL);
    g_lsi_cal.wd_err_ppm_before = ppm((2890ULL * 64ULL * 1000000ULL) / implied, 5000000ULL);
    e = g_lsi_cal.wd_err_ppm_after;
    mark(T_IWDG, (e < 0 ? -e : e) <= 5000);

    g_lsi_cal.pass = s_pass;
    g_lsi_cal.fail = s_fail;
    g_lsi_cal.verdict = verdict_of();
    g_lsi_cal.magic = 0x15C0CA1Du;

    while (1) {
        LED_TOGGLE();
        for (uint32_t i = 0; i < (g_lsi_cal.verdict == 0x600D600Du ? 10u : 2u); i++) {
            core_watchdog_feed();
            core_delay_ms(100);
        }
    }
}
