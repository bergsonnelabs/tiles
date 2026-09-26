/**
 * hw-clock-levels — bench test for the generated clock levels on
 * Core.ST.L0.1, Core.ST.L4.1 / L4.2 and Core.ST.W5. One build per level
 * (make CLOCK=low|medium|high|max). See README.md.
 *
 * Each check reads the hardware back and compares it with the reference
 * manual's tables, which are written out here on purpose rather than taken
 * from ll_rcc.h: a wrong table in the SDK must not be able to pass itself.
 *
 *   T1 flash   FLASH_ACR.LATENCY is the RM's value for SYSCLK in this range
 *              (RM0377 Table 14, RM0394 Table 9, RM0493 Table 41)
 *   T2 range   the core voltage range allows SYSCLK (L0: RM0377 Table 33,
 *              W5: RM0493 Table 99) and is the one coregen chose; L4: range 1
 *   T3 tree    SYSCLK source, PLL limits (L0 VCO <= 96 MHz; L4 VCO 64-344,
 *              input 4-16; W5 VCO 128-544, input 4-16 with the matching
 *              PLL1RGE, the PLL1R rule), W5 hclk5 (<= 32 MHz range 1, HDIV5
 *              in range 2, SRAM wait states), L4 APB >= 10 MHz for USB
 *   T4 clock   SYSCLK / SysTick against a precise, independent reference:
 *                L4: USB start-of-frame count (1 kHz from the host)
 *                W5: TIM16 input capture of HSE32 / 32 (crystal)
 *                L0: LPTIM1 counting HSI16 (factory-trimmed RC)
 *              plus the SysTick reload is SYSCLK_HZ / 1000
 *   T5 rtc     core_millis() across RTC seconds (RTC on LSI) is inside the
 *              LSI datasheet tolerance: coarse, catches gross errors only
 *
 * Result: USB CDC report every 3 s on the L4; the g_clock_levels struct and
 * the LED everywhere (slow blink = PASS, N quick blinks = first failing test).
 */

#include "core.h"
#include "core_rtc.h"
#include "core_watchdog.h"
#if defined(STM32L422xx)
#include "core_usb.h"
#endif

#define T_FLASH  (1u << 0)
#define T_RANGE  (1u << 1)
#define T_TREE   (1u << 2)
#define T_CLOCK  (1u << 3)
#define T_RTC    (1u << 4)
#define EXPECTED (T_FLASH | T_RANGE | T_TREE | T_CLOCK | T_RTC)

#define VERDICT_PASS  0x600D600Du
#define VERDICT_FAIL  0xBAD00000u   /* | index (1-5) of the first failing test */

/* SWD-readable result: arm-none-eabi-nm build/hw-clock-levels.elf | grep g_clock_levels */
typedef struct {
    uint32_t magic;          /* 0xC10CC10C once the result is final */
    uint32_t verdict;
    uint32_t pass, fail;
    char     level[8];       /* "low" .. "max" */
    uint32_t sysclk_hz;      /* what the build says (SYSCLK_HZ) */
    uint32_t latency, latency_want;
    uint32_t range, range_want;        /* L4: VOS field value 1/2 */
    uint32_t sws;                      /* RCC SWS field */
    uint32_t vco_mhz, pll_in_khz;      /* 0 when no PLL */
    uint32_t hclk5_khz;                /* W5 only */
    uint32_t ref_kind;                 /* 1 USB SOF, 2 HSE/32 TIM16, 3 HSI16 LPTIM1 */
    int32_t  clock_err_ppm;            /* measured SYSCLK (or SysTick) error */
    uint32_t clock_tol_ppm;
    uint32_t measured_hz;              /* W5: SYSCLK from TIM16; L0/L4: 0 */
    uint32_t ref_ms, tick_ms;          /* reference window vs SysTick over it */
    uint32_t rtc_ms_per_s;             /* core_millis per RTC second */
    uint32_t rtc_lo, rtc_hi;           /* its allowed band */
} clock_levels_t;

__attribute__((used)) volatile clock_levels_t g_clock_levels;

static uint32_t s_pass, s_fail;
static void mark(uint32_t t, int ok) { if (ok) s_pass |= t; else s_fail |= t; }

/* 4-byte FLASH_ACR on every Core; LATENCY is bit 0 on the L0, [3:0] elsewhere. */
#define FLASH_ACR_REG   REG32(0x40022000UL)
#define SYST_RVR        REG32(0xE000E014UL)

static void wait_ms(uint32_t ms)
{
    while (ms) {
        uint32_t d = ms > 100u ? 100u : ms;
        core_watchdog_feed();
        core_delay_ms(d);
        ms -= d;
    }
}

/* ======================================================================
 * Per-family register views and RM tables
 * ====================================================================== */

#if defined(STM32L011xx)
/* RM0377: RCC 0x40021000, PWR 0x40007000, LPTIM1 0x40007C00 */
#define R_RCC_CR       REG32(0x40021000UL)
#define R_RCC_CFGR     REG32(0x4002100CUL)
#define R_RCC_CCIPR    REG32(0x4002104CUL)
#define R_RCC_APB1ENR  REG32(0x40021038UL)
#define R_PWR_CR       REG32(0x40007000UL)
#define R_PWR_CSR      REG32(0x40007004UL)
#define LPTIM1_CFGR    REG32(0x40007C0CUL)
#define LPTIM1_CR      REG32(0x40007C10UL)
#define LPTIM1_ARR     REG32(0x40007C18UL)
#define LPTIM1_CNT     REG32(0x40007C1CUL)

static uint32_t latency_now(void) { return FLASH_ACR_REG & 0x1UL; }
static uint32_t range_now(void)   { return (R_PWR_CR >> 11) & 0x3UL; }   /* 1/2/3 */

/* RM0377 Table 14 / Table 33 */
static uint32_t latency_rm(uint32_t hz, uint32_t range)
{
    if (range == 1) return hz > 16000000u ? 1u : 0u;
    if (range == 2) return hz > 8000000u ? 1u : 0u;
    return 0u;
}
static uint32_t range_max_hz(uint32_t range)
{
    return range == 1 ? 32000000u : range == 2 ? 16000000u : 4200000u;
}

#elif defined(STM32L422xx)
/* RM0394: RCC 0x40021000, PWR 0x40007000, USB 0x40006800 */
#define R_RCC_CR       REG32(0x40021000UL)
#define R_RCC_CFGR     REG32(0x40021008UL)
#define R_RCC_PLLCFGR  REG32(0x4002100CUL)
#define R_PWR_CR1      REG32(0x40007000UL)
#define R_USB_FNR      (*(volatile uint16_t *)(0x40006800UL + 0x48UL))

static uint32_t latency_now(void) { return FLASH_ACR_REG & 0x7UL; }
static uint32_t range_now(void)   { return (R_PWR_CR1 >> 9) & 0x3UL; }   /* 1 = range 1, 2 = range 2 */

/* RM0394 Table 9 */
static uint32_t latency_rm(uint32_t hz, uint32_t range)
{
    uint32_t mhz = (hz + 999999u) / 1000000u;
    if (range == 2) return mhz <= 6 ? 0 : mhz <= 12 ? 1 : mhz <= 18 ? 2 : 3;
    return mhz <= 16 ? 0 : mhz <= 32 ? 1 : mhz <= 48 ? 2 : mhz <= 64 ? 3 : 4;
}
static uint32_t range_max_hz(uint32_t range) { return range == 1 ? 80000000u : 26000000u; }

#elif defined(STM32WBA55xx)
/* RM0493: RCC 0x46020C00, PWR 0x46020800, RAMCFG 0x40026000, TIM16 0x40014400 */
#define R_RCC_CR        REG32(0x46020C00UL)
#define R_RCC_CFGR1     REG32(0x46020C1CUL)
#define R_RCC_CFGR2     REG32(0x46020C20UL)
#define R_RCC_PLL1CFGR  REG32(0x46020C28UL)
#define R_RCC_PLL1DIVR  REG32(0x46020C34UL)
#define R_RCC_AHB1ENR   REG32(0x46020C88UL)
#define R_RCC_APB2ENR   REG32(0x46020CA4UL)
#define R_RCC_CFGR4     REG32(0x46020E00UL)
#define R_PWR_VOSR      REG32(0x4602080CUL)
#define R_RAMCFG_M1CR   REG32(0x40026000UL)
#define R_RAMCFG_M2CR   REG32(0x40026040UL)
#define TIM16_CR1       REG32(0x40014400UL)
#define TIM16_SR        REG32(0x40014410UL)
#define TIM16_EGR       REG32(0x40014414UL)
#define TIM16_CCMR1     REG32(0x40014418UL)
#define TIM16_CCER      REG32(0x40014420UL)
#define TIM16_PSC       REG32(0x40014428UL)
#define TIM16_ARR       REG32(0x4001442CUL)
#define TIM16_CCR1      REG32(0x40014434UL)
#define TIM16_TISEL     REG32(0x4001445CUL)

static uint32_t latency_now(void) { return FLASH_ACR_REG & 0xFUL; }
static uint32_t range_now(void)   { return (R_PWR_VOSR & (1UL << 16)) ? 1u : 2u; }

/* RM0493 Table 41 (LPM = 0) */
static uint32_t latency_rm(uint32_t hz, uint32_t range)
{
    uint32_t mhz = (hz + 999999u) / 1000000u;
    if (range == 2) return mhz <= 8 ? 0 : 1;
    return mhz <= 32 ? 0 : mhz <= 64 ? 1 : mhz <= 96 ? 2 : 3;
}
/* RM0493 Table 99 */
static uint32_t range_max_hz(uint32_t range) { return range == 1 ? 100000000u : 16000000u; }
#endif

/* ======================================================================
 * T1-T3: registers against the RM
 * ====================================================================== */

static void check_registers(void)
{
    uint32_t hz = SYSCLK_HZ;
    uint32_t range = range_now();

    g_clock_levels.latency = latency_now();
    g_clock_levels.range   = range;
#if defined(STM32L422xx)
    uint32_t rm_range = (range == 1u) ? 1u : 2u;
#else
    uint32_t rm_range = range;
#endif
    g_clock_levels.latency_want = latency_rm(hz, rm_range);
    mark(T_FLASH, g_clock_levels.latency == g_clock_levels.latency_want);

    /* T2: the range allows SYSCLK, and is the one the build meant */
    int range_ok = (range >= 1u && range <= 3u) && hz <= range_max_hz(rm_range);
#if defined(CLOCK_VOS_RANGE)
    g_clock_levels.range_want = CLOCK_VOS_RANGE;
    range_ok = range_ok && (range == CLOCK_VOS_RANGE);
#elif defined(STM32L422xx)
    /* USB needs range 1: range 2 caps every peripheral clock at 26 MHz
     * (STM32L422 datasheet, dynamic voltage scaling) and USB runs at 48. */
    g_clock_levels.range_want = 1u;
    range_ok = range_ok && (range == 1u);
#endif
#if defined(STM32WBA55xx)
    range_ok = range_ok && (R_PWR_VOSR & (1UL << 15));  /* VOSRDY */
#endif
    mark(T_RANGE, range_ok);

    /* T3: clock tree */
    int tree_ok = 1;
#if defined(STM32L011xx)
    uint32_t sws = (R_RCC_CFGR >> 2) & 0x3UL;          /* 0 MSI, 1 HSI16, 2 HSE, 3 PLL */
    g_clock_levels.sws = sws;
#if defined(CLOCK_SOURCE_MSI)
    tree_ok = tree_ok && sws == 0u;
    /* MSIRANGE (ICSCR[15:13]) 4 = 1.048 MHz, 5 = 2.097 MHz */
    uint32_t msir = (REG32(0x40021004UL) >> 13) & 0x7UL;
    tree_ok = tree_ok && ((hz == 1048576u && msir == 4u) || (hz == 2097152u && msir == 5u));
#elif defined(CLOCK_SOURCE_HSI16) && !CLOCK_USE_PLL
    tree_ok = tree_ok && sws == 1u;
#else
    static const uint8_t mul[9] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
    uint32_t cfgr = R_RCC_CFGR;
    uint32_t m = (cfgr >> 18) & 0xFUL, d = ((cfgr >> 22) & 0x3UL) + 1u;
    uint32_t vco = (m < 9u) ? 16u * mul[m] : 0u;       /* PLLSRC = HSI16 */
    g_clock_levels.vco_mhz = vco;
    g_clock_levels.pll_in_khz = 16000u;
    /* RM0377 §7.2.4: VCO <= 96 MHz in range 1, output <= 32 MHz */
    tree_ok = tree_ok && sws == 3u && !(cfgr & (1UL << 16)) && vco && vco <= 96u
              && d >= 2u && vco / d == hz / 1000000u;
#endif
#elif defined(STM32L422xx)
    uint32_t sws = (R_RCC_CFGR >> 2) & 0x3UL;          /* 0 MSI, 1 HSI16, 2 HSE, 3 PLL */
    g_clock_levels.sws = sws;
    tree_ok = tree_ok && hz >= 10000000u;               /* USB: APB >= 10 MHz, RM0394 §46.4 */
#if CLOCK_USE_PLL
    uint32_t pc = R_RCC_PLLCFGR;
    uint32_t pm = ((pc >> 4) & 0x7UL) + 1u, pn = (pc >> 8) & 0x7FUL, pr = (((pc >> 25) & 0x3UL) + 1u) * 2u;
    uint32_t in_khz = 16000u / pm, vco = 16u * pn / pm;
    g_clock_levels.vco_mhz = vco;
    g_clock_levels.pll_in_khz = in_khz;
    /* RM0394 §6.2.5: input 4-16 MHz, VCO 64-344 MHz, PLLR output <= 80 MHz */
    tree_ok = tree_ok && sws == 3u && (pc & 0x3UL) == 2u && in_khz >= 4000u && in_khz <= 16000u
              && vco >= 64u && vco <= 344u && vco / pr == hz / 1000000u && (pc & (1UL << 24));
#else
    tree_ok = tree_ok && sws == 0u;
#endif
#elif defined(STM32WBA55xx)
    uint32_t sws = (R_RCC_CFGR1 >> 2) & 0x3UL;         /* 0 HSI16, 2 HSE32, 3 PLL1 */
    g_clock_levels.sws = sws;
    uint32_t cfgr4 = R_RCC_CFGR4;
    uint32_t hclk5;
    if (sws == 3u) {
        static const uint8_t hpre5_div[8] = { 1, 1, 1, 1, 2, 3, 4, 6 };
        hclk5 = hz / hpre5_div[cfgr4 & 0x7UL];
    } else {
        hclk5 = (cfgr4 & (1UL << 4)) ? hz / 2u : hz;
    }
    g_clock_levels.hclk5_khz = hclk5 / 1000u;
    /* RM0493 Table 99 / §12.4.6: hclk5 <= 32 MHz in range 1; in range 2 HDIV5
     * set (hclk5 = 8 MHz at 16 MHz), SRAM1/2 1 WS (Table 36), no PLL. */
    if (range == 1u) {
        tree_ok = tree_ok && hclk5 <= 32000000u;
    } else {
        /* RAMCFG reads as 0 while its bus clock is off (RCC_AHB1ENR.RAMCFGEN,
         * bit 17, off after reset), which read as "SRAM 0 WS" here. */
        uint32_t ahb1enr = R_RCC_AHB1ENR;
        R_RCC_AHB1ENR = ahb1enr | (1UL << 17);
        (void)R_RCC_AHB1ENR;
        uint32_t m1 = (R_RAMCFG_M1CR >> 16) & 0x7UL, m2 = (R_RAMCFG_M2CR >> 16) & 0x7UL;
        R_RCC_AHB1ENR = ahb1enr;
        tree_ok = tree_ok && (cfgr4 & (1UL << 4)) && hclk5 <= 8000000u && sws == 0u
                  && m1 >= 1u && m2 >= 1u && !(R_RCC_CR & (1UL << 24));
    }
    /* AHB/APB undivided (HPRE / PPRE1 / PPRE2 in CFGR2, RM0493 §12.8.5) */
    tree_ok = tree_ok && (R_RCC_CFGR2 & 0x444UL) == 0u;
#if CLOCK_USE_PLL
    uint32_t pc = R_RCC_PLL1CFGR, pd = R_RCC_PLL1DIVR;
    uint32_t pm = ((pc >> 8) & 0x7UL) + 1u, rge = (pc >> 2) & 0x3UL;
    uint32_t pn = (pd & 0x1FFUL) + 1u, pr = ((pd >> 24) & 0x7FUL) + 1u;
    uint32_t in_khz = 32000u / pm, vco = 32u * pn / pm;
    g_clock_levels.vco_mhz = vco;
    g_clock_levels.pll_in_khz = in_khz;
    /* RM0493 §12.4.3 / §12.8.7: ref 4-16 MHz with PLL1RGE 11 above 8 MHz,
     * VCO 128-544, pll1rclk <= 100 MHz and VCO / (2 x (R / 2)) <= 100 MHz */
    uint32_t half = 2u * (pr / 2u);
    tree_ok = tree_ok && sws == 3u && (pc & 0x3UL) == 3u
              && in_khz >= 4000u && in_khz <= 16000u
              && (in_khz > 8000u ? rge == 3u : rge != 3u)
              && vco >= 128u && vco <= 544u
              && half && vco / half <= 100u && vco / pr == hz / 1000000u
              && (pc & (1UL << 18)) && !(pc & (1UL << 20));       /* PLL1REN, prescaler released */
#elif defined(CLOCK_SOURCE_HSE)
    tree_ok = tree_ok && sws == 2u && !(REG32(0x46020C00UL) & (1UL << 20));  /* HSEPRE clear */
#else
    tree_ok = tree_ok && sws == 0u;
#endif
#endif
    /* SysTick reload = SYSCLK_HZ / 1000 */
    tree_ok = tree_ok && (SYST_RVR + 1u) == SYSCLK_HZ / 1000u;
    mark(T_TREE, tree_ok);
}

/* ======================================================================
 * T4: against a precise reference
 * ====================================================================== */

static int32_t ppm(int64_t got, int64_t want)
{
    return (int32_t)(((got - want) * 1000000LL) / want);
}

#if defined(STM32L422xx)
/* USB SOF: the host sends one per ms (±500 ppm). FNR.FN counts them. */
static void measure_reference(void)
{
    g_clock_levels.ref_kind = 1u;
    g_clock_levels.clock_tol_ppm = 20000u;   /* MSI / HSI16 at room temp: ±2 % */
    uint16_t f0 = R_USB_FNR & 0x7FFu;
    uint32_t t = core_millis();
    while (((R_USB_FNR & 0x7FFu) == f0) && (core_millis() - t) < 20u)
        ;
    f0 = R_USB_FNR & 0x7FFu;                  /* aligned to a frame edge */
    uint32_t m0 = core_millis();
    wait_ms(1500);
    uint32_t m1 = core_millis();
    uint16_t f1 = R_USB_FNR & 0x7FFu;
    uint32_t frames = (uint32_t)((f1 - f0) & 0x7FFu);
    g_clock_levels.ref_ms  = frames;
    g_clock_levels.tick_ms = m1 - m0;
    if (frames < 100u) {                      /* no host sending SOFs: can't judge */
        g_clock_levels.clock_err_ppm = 0;
        mark(T_CLOCK, 0);
        return;
    }
    /* SysTick ms per real ms */
    g_clock_levels.clock_err_ppm = ppm(g_clock_levels.tick_ms, frames);
    int32_t e = g_clock_levels.clock_err_ppm;
    mark(T_CLOCK, (e < 0 ? -e : e) <= (int32_t)g_clock_levels.clock_tol_ppm);
}

#elif defined(STM32WBA55xx)
/* TIM16 on PCLK2 (= SYSCLK here) captures HSE32 / 32 (TISEL TI1SEL = 3,
 * tim_ti1_in3, RM0493 Table 295), prescaled /8: one capture per 8 us of
 * crystal time. The /32 divider has its own enable, TIM16_OR1.HSE32EN
 * (§31.6.19, CubeWBA LL_TIM_EnableHSE32): without it tim_ti1_in3 is idle and
 * nothing is ever captured (the first W5 runs read "measured SYSCLK = 0").
 *
 * A capture can be missed when an interrupt delays the poll by more than
 * 8 us (128 cycles at "low"); CC1OF then says the interval read spans more
 * than one period. Only those intervals are split by the nominal period, so
 * a wrong SYSCLK still shows up in the clean ones. Waits are bounded in
 * SysTick time and feed the watchdog, so a dead reference fails T4 instead
 * of hanging the test. */
#define TIM16_OR1       REG32(0x40014468UL)

static int tim16_wait_capture(uint32_t ms)
{
    uint32_t t0 = core_millis();
    while (!(TIM16_SR & 2u)) {
        if ((uint32_t)(core_millis() - t0) > ms) return 0;
        core_watchdog_feed();
    }
    return 1;
}

static void measure_reference(void)
{
    g_clock_levels.ref_kind = 2u;
#if defined(CLOCK_SOURCE_HSE)
    g_clock_levels.clock_tol_ppm = 1000u;     /* crystal-derived: resolution only */
#else
    g_clock_levels.clock_tol_ppm = 12000u;    /* HSI16: 15.84-16.16 MHz (DS Table 65) */
#endif
    R_RCC_CR |= (1UL << 16);                  /* HSEON, if the level left it off */
    for (uint32_t t0 = core_millis(); !(R_RCC_CR & (1UL << 17))
                                      && (uint32_t)(core_millis() - t0) < 50u; )
        ;
    R_RCC_APB2ENR |= (1UL << 17);             /* TIM16EN */
    (void)R_RCC_APB2ENR;
    TIM16_CR1 = 0;
    TIM16_PSC = 0;
    TIM16_ARR = 0xFFFFu;
    TIM16_OR1 |= (1u << 1);                   /* HSE32EN: the HSE / 32 divider */
    TIM16_TISEL = 3u;                         /* TI1 = tim_ti1_in3 = HSE32 / 32 */
    TIM16_CCMR1 = (1u << 0) | (3u << 2);      /* CC1S = TI1, IC1PSC = /8 */
    TIM16_CCER = 1u;                          /* CC1E */
    TIM16_EGR = 1u;
    TIM16_SR = 0;
    TIM16_CR1 = 1u;

    const uint32_t N = 25000u;                /* 25000 x 8 us = 200 ms */
    const uint32_t per = SYSCLK_HZ / 125000u; /* nominal counts per 8 us */
    uint32_t prev = 0, total = 0, n = 0, ok = 0, m0 = 0, m1 = 0;
    if (tim16_wait_capture(20u)) {
        prev = TIM16_CCR1;                    /* reading CCR1 clears CC1IF */
        TIM16_SR = ~(1u << 9);                /* CC1OF (rc_w0) */
        m0 = core_millis();
        ok = 1;
        while (n < N) {
            if (!tim16_wait_capture(5u)) { ok = 0; break; }
            uint32_t c = TIM16_CCR1;
            uint32_t d = (c - prev) & 0xFFFFu, k = 1;
            prev = c;
            if (TIM16_SR & (1u << 9)) {       /* missed one or more captures */
                TIM16_SR = ~(1u << 9);
                k = (d + per / 2u) / per;
                if (k < 2u) k = 2u;
            }
            total += d;
            n += k;
        }
        m1 = core_millis();
    }
    TIM16_CR1 = 0;

    g_clock_levels.ref_ms  = (n * 8u) / 1000u;
    g_clock_levels.tick_ms = m1 - m0;
    if (!ok) {
        g_clock_levels.measured_hz = 0;
        mark(T_CLOCK, 0);
        return;
    }
    /* SYSCLK = counts per second = total / (n x 8 us) */
    g_clock_levels.measured_hz = (uint32_t)(((uint64_t)total * 125000u) / n);
    g_clock_levels.clock_err_ppm = ppm(g_clock_levels.measured_hz, SYSCLK_HZ);
    int32_t e = g_clock_levels.clock_err_ppm;
    int32_t dt = (int32_t)g_clock_levels.tick_ms - (int32_t)g_clock_levels.ref_ms;
    mark(T_CLOCK, (e < 0 ? -e : e) <= (int32_t)g_clock_levels.clock_tol_ppm
                  && dt >= -2 && dt <= 2);
}

#elif defined(STM32L011xx)
/* LPTIM1 counting HSI16 / 128 = 125 kHz, 8 us a count (RCC_CCIPR.LPTIM1SEL =
 * 10, RM0377 §7.3.19; PRESC Table 87). Not /16: LPTIM_CNT is asynchronous and
 * only a pair of equal back-to-back reads is valid (RM0377 §19.7.8), and at
 * the `low` level (MSI 1.05 MHz) a 1 MHz counter moves between the two reads,
 * so the pair almost never matched. Bench, 2026-09-26: the read spun ~85 ms
 * and the window hit the 16-bit wrap (135 SysTick ms read back as 3.9 ms).
 * At 125 kHz the counter wraps after 524 ms. HSI16 can't run in range 3, so a range-3 build steps up to range 2
 * for the measurement (MSI doesn't change with the range) and back after. */
static uint32_t lptim_cnt(void)
{
    uint32_t a, b;
    do { a = LPTIM1_CNT; b = LPTIM1_CNT; } while (a != b);   /* async counter */
    return a;
}

static void measure_reference(void)
{
    g_clock_levels.ref_kind = 3u;
#if defined(CLOCK_SOURCE_MSI)
    g_clock_levels.clock_tol_ppm = 25000u;    /* MSI vs HSI16, both factory-trimmed, room temp */
#else
    g_clock_levels.clock_tol_ppm = 5000u;     /* SYSCLK derived from HSI16 itself: ratio only */
#endif
    uint32_t r0 = range_now();
    R_RCC_APB1ENR |= (1UL << 28);             /* PWREN */
    if (r0 == 3u) {
        while (R_PWR_CSR & (1UL << 4)) ;
        R_PWR_CR = (R_PWR_CR & ~(3UL << 11)) | (2UL << 11);
        while (R_PWR_CSR & (1UL << 4)) ;
    }
    uint32_t hsi_was_on = R_RCC_CR & 1UL;
    R_RCC_CR |= 1UL;                          /* HSI16ON */
    for (uint32_t t = 100000u; t && !(R_RCC_CR & (1UL << 2)); t--)
        ;
    R_RCC_CCIPR = (R_RCC_CCIPR & ~(3UL << 18)) | (2UL << 18);  /* LPTIM1SEL = HSI16 */
    R_RCC_APB1ENR |= (1UL << 31);             /* LPTIM1EN */
    (void)R_RCC_APB1ENR;
    LPTIM1_CR = 0;
    LPTIM1_CFGR = (7UL << 9);                 /* PRESC = /128 */
    LPTIM1_CR = 1u;                           /* ENABLE */
    LPTIM1_ARR = 0xFFFFu;
    LPTIM1_CR = 1u | (1u << 2);               /* CNTSTRT: continuous */
    core_delay_ms(2);

    /* 50 SysTick ms = 6250 counts if SysTick is right (16-bit counter) */
    uint32_t t0 = core_millis();
    while (core_millis() == t0) ;
    uint32_t c0 = lptim_cnt(), m0 = core_millis();
    while (core_millis() - m0 < 50u) ;
    uint32_t c1 = lptim_cnt(), m1 = core_millis();
    LPTIM1_CR = 0;
    R_RCC_APB1ENR &= ~(1UL << 31);
    if (!hsi_was_on) R_RCC_CR &= ~1UL;
    if (r0 == 3u) {
        while (R_PWR_CSR & (1UL << 4)) ;
        R_PWR_CR = (R_PWR_CR & ~(3UL << 11)) | (3UL << 11);
        while (R_PWR_CSR & (1UL << 4)) ;
    }
    uint32_t us = ((c1 - c0) & 0xFFFFu) * 8u;
    g_clock_levels.ref_ms  = us / 1000u;
    g_clock_levels.tick_ms = m1 - m0;
    /* SysTick ms vs HSI16 time */
    g_clock_levels.clock_err_ppm = ppm((int64_t)g_clock_levels.tick_ms * 1000, us);
    int32_t e = g_clock_levels.clock_err_ppm;
    mark(T_CLOCK, us > 1000u && (e < 0 ? -e : e) <= (int32_t)g_clock_levels.clock_tol_ppm);
}
#endif

/* ======================================================================
 * T5: RTC on LSI (coarse)
 * ====================================================================== */

static uint32_t rtc_s(void)
{
    uint8_t h, m, s;
    core_rtc_get_time(&h, &m, &s);
    return (uint32_t)h * 3600u + (uint32_t)m * 60u + s;
}

static void measure_rtc(void)
{
    /* LSI datasheet band, against the SDK's nominal LL_LSI_HZ: ms per RTC
     * second = 1000 x nominal / actual. */
#if defined(STM32L011xx)
    const uint32_t lo_khz10 = 260, hi_khz10 = 560;    /* 26-56 kHz, DS Table 39 */
#elif defined(STM32L422xx)
    const uint32_t lo_khz10 = 295, hi_khz10 = 340;    /* 29.5-34 kHz, DS Table 50 */
#else
    const uint32_t lo_khz10 = 304, hi_khz10 = 336;    /* LSI1 30.4-33.6 kHz, DS Table 66 */
#endif
    uint32_t nominal10 = LL_LSI_HZ / 100u;
    g_clock_levels.rtc_lo = (1000u * nominal10) / hi_khz10;
    g_clock_levels.rtc_hi = (1000u * nominal10) / lo_khz10 + 1u;

    core_rtc_init();
    uint32_t s0 = rtc_s(), t = core_millis();
    while (rtc_s() == s0 && core_millis() - t < 3000u) core_watchdog_feed();
    uint32_t a = rtc_s(), m0 = core_millis();
    while (rtc_s() - a < 3u && core_millis() - m0 < 8000u) core_watchdog_feed();
    uint32_t n = rtc_s() - a, m1 = core_millis();
    g_clock_levels.rtc_ms_per_s = n ? (m1 - m0) / n : 0u;
    mark(T_RTC, n == 3u && g_clock_levels.rtc_ms_per_s >= g_clock_levels.rtc_lo
                && g_clock_levels.rtc_ms_per_s <= g_clock_levels.rtc_hi);
}

/* ====================================================================== */

static uint32_t first_failure(uint32_t missing)
{
    for (uint32_t i = 0; i < 5; i++)
        if (missing & (1u << i)) return i + 1u;
    return 0;
}

#if defined(STM32L422xx)
static void print_report(uint32_t verdict)
{
    static const char *const names[5] = {
        "T1 flash latency", "T2 voltage range", "T3 clock tree", "T4 SysTick/USB SOF", "T5 RTC (LSI)"
    };
    core_usb_printf("\r\n[hw-clock-levels] %s  level=%s SYSCLK=%lu Hz  (pass=0x%02lx fail=0x%02lx)\r\n",
                    verdict == VERDICT_PASS ? "PASS" : "FAIL", CLOCK_LEVEL_NAME,
                    (unsigned long)SYSCLK_HZ, (unsigned long)s_pass, (unsigned long)s_fail);
    for (uint32_t i = 0; i < 5; i++) {
        uint32_t b = 1u << i;
        core_usb_printf("  %-20s %s\r\n", names[i],
                        (s_pass & b) ? "PASS" : (s_fail & b) ? "FAIL" : "not run");
    }
    core_usb_printf("  latency %lu (RM %lu), range %lu, SWS %lu, VCO %lu MHz, PLL in %lu kHz\r\n",
                    (unsigned long)g_clock_levels.latency, (unsigned long)g_clock_levels.latency_want,
                    (unsigned long)g_clock_levels.range, (unsigned long)g_clock_levels.sws,
                    (unsigned long)g_clock_levels.vco_mhz, (unsigned long)g_clock_levels.pll_in_khz);
    core_usb_printf("  SysTick %lu ms over %lu USB frames: %ld ppm (limit %lu); "
                    "RTC second = %lu ms (%lu-%lu)\r\n",
                    (unsigned long)g_clock_levels.tick_ms, (unsigned long)g_clock_levels.ref_ms,
                    (long)g_clock_levels.clock_err_ppm, (unsigned long)g_clock_levels.clock_tol_ppm,
                    (unsigned long)g_clock_levels.rtc_ms_per_s,
                    (unsigned long)g_clock_levels.rtc_lo, (unsigned long)g_clock_levels.rtc_hi);
}
#endif

int main(void)
{
    core_init();                  /* the level's clock setup + 5 s watchdog */
    core_led_init();

    {
        const char *n = CLOCK_LEVEL_NAME;
        for (uint32_t i = 0; i < sizeof(g_clock_levels.level) - 1u && n[i]; i++)
            g_clock_levels.level[i] = n[i];
    }
    g_clock_levels.sysclk_hz = SYSCLK_HZ;

    check_registers();            /* before anything else touches the clocks */
#if defined(STM32L422xx)
    wait_ms(1500);                /* USB enumerates; SOFs start */
#else
    /* A probe-rs reset halts the core again shortly after it starts (~13 ms
     * on the W5 bench, 2026-09-25). SysTick stops with the core but TIM16 /
     * LPTIM1 and their reference do not, so a halt inside the T4 window
     * skews it (W5 read +-20..40 ppm, exact after a power cycle). Start
     * measuring once the debugger is done. */
    wait_ms(500);
#endif
    measure_reference();
    measure_rtc();

    uint32_t missing = (EXPECTED & ~s_pass) | s_fail;
    uint32_t verdict = missing ? (VERDICT_FAIL | first_failure(missing)) : VERDICT_PASS;
    g_clock_levels.pass = s_pass;
    g_clock_levels.fail = s_fail;
    g_clock_levels.verdict = verdict;
    g_clock_levels.magic = 0xC10CC10Cu;

    uint32_t last_print = 0;
    while (1) {
#if defined(STM32L422xx)
        if ((uint32_t)(core_millis() - last_print) >= 3000u) {
            last_print = core_millis();
            print_report(verdict);
        }
#else
        (void)last_print;
#endif
        if (verdict == VERDICT_PASS) {
            LED_ON();  wait_ms(1000);
            LED_OFF(); wait_ms(1000);
        } else {
            for (uint32_t i = 0; i < (verdict & 0xFFu); i++) {
                LED_ON();  wait_ms(150);
                LED_OFF(); wait_ms(250);
            }
            wait_ms(1500);
        }
    }
}
