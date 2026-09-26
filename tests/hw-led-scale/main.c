/**
 * hw-led-scale — bench test for core_led_scaled_on_ms() (sdk/core/core_led.c)
 * on Core.ST.L0, one build per clock level. The LED code stretches a short
 * on-time by VDD, which it reads through VREFINT with a throwaway ADC; on the
 * L0 that never worked until VREFINT was connected (2026-09-22) and the ADC
 * got a clock at the MSI levels (tiles#283).
 *
 *   T1 vdd      the VDD the LED path reads is calibrated (VREFINT, not the
 *               3.3 V fallback) and within 2.9-3.6 V (bench board: 3.3 V)
 *   T2 scale    core_led_scaled_on_ms(2) is 1-3 ms at 3.3 V (about 1x), 50
 *               comes back unchanged, and a second call returns the same
 *   T3 time     the first call (the ADC read) takes under 5 ms, and the
 *               cached second call under 0.5 ms (at 1 MHz that is mostly
 *               the timing itself: see us_overhead)
 *   T4 cache    after 61 s the next call measures again (it takes about as
 *               long as the first); final result about 65 s after boot
 *
 * SWD: `g_led_scale` (arm-none-eabi-nm build/hw-led-scale.elf), laid out as
 * led_scale_result_t below. PASS: magic 0x1ED5CA1F, verdict 0x600D600D,
 * pass 0xF (at ~3 s magic 0x1ED5CA1E: T1-T3 in, T4 still waiting).
 */

#include "core.h"
#include "core_watchdog.h"
#include "hal_adc.h"

#if !defined(STM32L011xx)
#error "hw-led-scale is written for Core.ST.L0"
#endif

#ifndef REG32
#define REG32(a)       (*(volatile uint32_t *)(a))
#endif
#define SYST_LOAD      REG32(0xE000E014UL)
#define SYST_VAL       REG32(0xE000E018UL)

#define T_VDD    (1u << 0)
#define T_SCALE  (1u << 1)
#define T_TIME   (1u << 2)
#define T_CACHE  (1u << 3)
#define EXPECTED 0xFu

typedef struct {
    uint32_t magic;          /* 0x1ED5CA1E once T1-T3 are in; 0x1ED5CA1F after T4 */
    uint32_t verdict;        /* 0x600D600D, or 0xBAD000nn (nn = first failing test) */
    uint32_t pass, fail;
    uint32_t level[2];       /* ASCII */
    uint32_t sysclk_hz;
    uint32_t on2_first;      /* core_led_scaled_on_ms(2), first call */
    uint32_t us_first;       /* its duration */
    uint32_t on2_cached, us_cached;
    uint32_t on20, on50;
    uint32_t vdd_mv;         /* the same read the LED code does, repeated here */
    uint32_t vdd_calibrated;
    uint32_t vref_cal;       /* factory VREFINT_CAL at 0x1FF80078 */
    uint32_t cfgr3_after;    /* SYSCFG_CFGR3 after the LED read */
    uint32_t adc_ccr_after;  /* ADC_CCR after it */
    uint32_t adc_cr_after;   /* ADC_CR after it (0 = off) */
    uint32_t us_rescan;      /* T4: the call after 61 s */
    uint32_t us_overhead;    /* now_us() - now_us() back to back */
} led_scale_result_t;

__attribute__((used)) volatile led_scale_result_t g_led_scale;

static uint32_t s_pass, s_fail;

static void mark(uint32_t t, int ok)
{
    if (ok) s_pass |= t; else s_fail |= t;
}

/* Microseconds from SysTick: ms counter plus the down-counter's progress. */
static uint32_t now_us(void)
{
    uint32_t ms, val;
    do {
        ms = _systick_ticks;
        val = SYST_VAL;
    } while (ms != _systick_ticks);
    uint32_t load = SYST_LOAD + 1u;
    return ms * 1000u + ((load - 1u - val) * 1000u) / load;
}

static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 100u ? 100u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
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
    core_init();
    core_led_init();
    wait_ms(100);

    const char *lv = CLOCK_LEVEL_NAME;
    uint32_t w[2] = { 0, 0 };
    for (uint32_t i = 0; i < 8 && lv[i]; i++) w[i / 4] |= (uint32_t)(uint8_t)lv[i] << (8 * (i % 4));
    g_led_scale.level[0] = w[0];
    g_led_scale.level[1] = w[1];
    g_led_scale.sysclk_hz = SYSCLK_HZ;

    uint32_t t = now_us();
    g_led_scale.us_overhead = now_us() - t;
    t = now_us();
    g_led_scale.on2_first = core_led_scaled_on_ms(2);
    g_led_scale.us_first = now_us() - t;
    g_led_scale.cfgr3_after = REG32(0x40010020UL);
    g_led_scale.adc_ccr_after = REG32(0x40012708UL);
    g_led_scale.adc_cr_after = REG32(0x40012408UL);

    t = now_us();
    g_led_scale.on2_cached = core_led_scaled_on_ms(2);
    g_led_scale.us_cached = now_us() - t;
    g_led_scale.on20 = core_led_scaled_on_ms(20);
    g_led_scale.on50 = core_led_scaled_on_ms(50);

    /* The read the LED code does when the project has no ADC of its own. */
    hal_adc_t adc;
    if (hal_adc_init(&adc, ADC1, SYSCLK_HZ, HAL_ADC_RES_12BIT) == HAL_OK) {
        g_led_scale.vdd_mv = hal_adc_read_vdda_mv(&adc);
        g_led_scale.vdd_calibrated = adc.vdda_calibrated ? 1u : 0u;
        hal_adc_deinit(&adc);
    }
    g_led_scale.vref_cal = *(volatile uint16_t *)0x1FF80078UL;

    mark(T_VDD, g_led_scale.vdd_calibrated && g_led_scale.vdd_mv >= 2900u &&
                g_led_scale.vdd_mv <= 3600u);
    mark(T_SCALE, g_led_scale.on2_first >= 1u && g_led_scale.on2_first <= 3u &&
                  g_led_scale.on2_cached == g_led_scale.on2_first &&
                  g_led_scale.on50 == 50u);
    mark(T_TIME, g_led_scale.us_first < 5000u && g_led_scale.us_cached < 500u);

    g_led_scale.pass = s_pass;
    g_led_scale.fail = s_fail;
    g_led_scale.verdict = verdict_of();
    g_led_scale.magic = 0x1ED5CA1Eu;

    /* T4: the scale is re-read at most once a minute. */
    wait_ms(61000u);
    t = now_us();
    (void)core_led_scaled_on_ms(2);
    g_led_scale.us_rescan = now_us() - t;
    mark(T_CACHE, g_led_scale.us_rescan * 2u > g_led_scale.us_first &&
                  g_led_scale.us_rescan > 4u * g_led_scale.us_cached);
    g_led_scale.pass = s_pass;
    g_led_scale.fail = s_fail;
    g_led_scale.verdict = verdict_of();
    g_led_scale.magic = 0x1ED5CA1Fu;

    while (1) {
        LED_TOGGLE();
        wait_ms(g_led_scale.verdict == 0x600D600Du ? 1000u : 150u);
    }
}
