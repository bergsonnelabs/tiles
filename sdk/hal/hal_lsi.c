/**
 * hal_lsi.c — the LSI frequency the RTC and IWDG are programmed from.
 *
 * Core.ST.L0 only. The STM32L011's LSI is untrimmed: 26 / 38 / 56 kHz min /
 * typ / max (DS DocID027973 Table 39). With the nominal 37 kHz assumed, a
 * "5 s" watchdog is anything from 3.3 to 7.1 s and an RTC second runs 660 to
 * 1420 ms. So the first call to ll_lsi_hz() measures it:
 *
 *   TIM21 counts its kernel clock (SYSCLK through the AHB / APB2 prescalers,
 *   x2 when APB2 is divided: RM0377 §7.2) and captures every 8th LSI rising
 *   edge on CH1 (TIM21_OR TI1_RMP = 101: LSI, §17.4.14; CCMR1 CC1S = 01,
 *   IC1PSC = 11, §17.4.7). 16 captures span 128 LSI periods, ~3.5 ms.
 *
 * The reference is SYSCLK: HSI16 at the high / max levels (+-1 % at 25 C, DS
 * Table 38) and MSI at low / medium (+-0.5 % typ at 25 C, Table 40). Either is
 * ~40x better than the untrimmed LSI, and neither needs another oscillator
 * started (HSI16 can't even run in voltage range 3, RM0377 §6.1.4).
 *
 * Bench (2026-09-26, one board, 3.3 V, room temperature): single readings
 * scatter about +-0.1 %, their average matched the LSI implied by 10 RTC
 * seconds to 118 ppm, and the LSI itself wandered ~0.09 % between consecutive
 * 10 s windows. With PREDIV_S steps of 0.35 %, an RTC second lands within
 * about +-0.3 % of SYSCLK time, against 660-1420 ms uncalibrated.
 *
 * The result is cached for the life of the firmware: every consumer (RTC
 * prescalers, RTC wakeup ticks, IWDG reload) must agree, or a sleep chunked
 * to half the watchdog timeout could outlast it. TIM21 is used only if it is
 * off (RCC_APB2ENR.TIM21EN = 0), and is reset and switched off afterwards; if
 * it is busy, or the reading is out of band, the nominal LL_LSI_HZ is kept.
 */

#include <stdint.h>

#if defined(STM32L011xx)

#include "ll_common.h"
#include "core_config.h"     /* SYSCLK_HZ (generated) */
#include "hal_lsi.h"

#define RCC_CR_REG        REG32(0x40021000UL)
#define RCC_CFGR_REG      REG32(0x4002100CUL)
#define RCC_APB2RSTR_REG  REG32(0x40021024UL)   /* §7.3.9 */
#define RCC_APB2ENR_REG   REG32(0x40021034UL)   /* §7.3.13 */
#define RCC_CSR_REG       REG32(0x40021050UL)   /* §7.3.20: LSION 0, LSIRDY 1 */
#define APB2_TIM21        (1UL << 2)            /* TIM21EN / TIM21RST */

#define TIM21_BASE_ADDR   0x40010800UL          /* §2.2.2 memory map */
#define TIM21_CR1         REG32(TIM21_BASE_ADDR + 0x00UL)
#define TIM21_SR          REG32(TIM21_BASE_ADDR + 0x10UL)
#define TIM21_EGR         REG32(TIM21_BASE_ADDR + 0x14UL)
#define TIM21_CCMR1       REG32(TIM21_BASE_ADDR + 0x18UL)
#define TIM21_CCER        REG32(TIM21_BASE_ADDR + 0x20UL)
#define TIM21_PSC         REG32(TIM21_BASE_ADDR + 0x28UL)
#define TIM21_ARR         REG32(TIM21_BASE_ADDR + 0x2CUL)
#define TIM21_CCR1        REG32(TIM21_BASE_ADDR + 0x34UL)
#define TIM21_OR          REG32(TIM21_BASE_ADDR + 0x50UL)
#define SR_UIF            (1UL << 0)
#define SR_CC1IF          (1UL << 1)
#define SR_CC1OF          (1UL << 9)

#define LSI_EDGES_PER_CAPTURE  8u
#define LSI_CAPTURES           16u
#define LSI_MIN_HZ             20000UL      /* DS band is 26-56 kHz; outside */
#define LSI_MAX_HZ             70000UL      /* this the reading is wrong     */

static uint32_t s_lsi_hz;                   /* 0 = not measured yet */

/* TIM21's kernel clock (RM0377 §7.2: x2 when the APB2 prescaler isn't 1). */
static uint32_t tim21_clk_hz(void)
{
    uint32_t cfgr = RCC_CFGR_REG;
    uint32_t hclk = SYSCLK_HZ;
    uint32_t hpre = (cfgr >> 4) & 0xFUL;               /* §7.3.3 HPRE */
    if (hpre & 0x8UL) {
        uint32_t sh = (hpre & 0x7UL) + 1UL;            /* /2 /4 /8 /16, then */
        if (sh >= 5UL) sh++;                           /* /64 .. /512 (no /32) */
        hclk >>= sh;
    }
    uint32_t ppre2 = (cfgr >> 11) & 0x7UL;             /* PPRE2 */
    return (ppre2 & 0x4UL) ? (hclk >> (ppre2 - 4UL)) : hclk;   /* 2 x HCLK / div */
}

/* Timer count at the next capture, or 0 if it can't be trusted: none within
 * a counter wrap (8 LSI periods are at most ~310 us, a wrap at least ~2 ms),
 * or one was missed (CC1OF, §17.4.5): the CPU was held off for more than 8
 * LSI periods, 227 cycles at the 1 MHz level, by an interrupt, an EEPROM
 * write, or a debugger. Bench, 2026-09-26: SWD traffic right after a reset
 * cost one reading in six up to 33 %. */
static uint32_t wait_capture(void)
{
    uint32_t wraps = 0;
    TIM21_SR = ~SR_UIF;                               /* rc_w0: clear UIF only */
    for (;;) {
        uint32_t sr = TIM21_SR;
        if (sr & SR_CC1OF) return 0u;
        if (sr & SR_CC1IF) break;
        if (sr & SR_UIF) {
            TIM21_SR = ~SR_UIF;
            if (++wraps > 1u) return 0u;
        }
    }
    uint32_t c = TIM21_CCR1;                          /* reading CCR1 clears CC1IF */
    if (TIM21_SR & SR_CC1OF) return 0u;               /* another landed before it */
    return c | 0x10000UL;
}

/* Timer counts across LSI_CAPTURES x 8 LSI periods, or 0. */
static uint32_t capture_run(void)
{
    TIM21_SR = 0;
    uint32_t sum = 0;
    uint32_t prev = wait_capture();
    for (uint32_t i = 0; prev && i < LSI_CAPTURES; i++) {
        uint32_t c = wait_capture();
        if (!c) return 0u;
        sum += (c - prev) & 0xFFFFUL;
        prev = c;
    }
    return prev ? sum : 0u;
}

uint32_t hal_lsi_measure_hz(void)
{
    if (RCC_APB2ENR_REG & APB2_TIM21) return 0u;      /* the application's */

    SET_BITS(RCC_CSR_REG, 1UL << 0);                  /* LSION */
    for (uint32_t t = 200000UL; !(RCC_CSR_REG & (1UL << 1)); )
        if (--t == 0u) return 0u;                     /* LSIRDY */

    SET_BITS(RCC_APB2ENR_REG, APB2_TIM21);
    (void)RCC_APB2ENR_REG;
    TIM21_CR1 = 0;
    TIM21_PSC = 0;
    TIM21_ARR = 0xFFFFUL;
    TIM21_OR = 5UL << 2;                              /* TI1_RMP = LSI */
    TIM21_CCMR1 = (3UL << 2) | 1UL;                   /* IC1PSC /8, CC1S = TI1 */
    TIM21_CCER = 1UL;                                 /* CC1E, rising edge */
    TIM21_EGR = 1UL;                                  /* UG: load PSC */
    TIM21_CR1 = 1UL;                                  /* CEN */

    uint32_t sum = 0;
    for (uint32_t attempt = 0; attempt < 3u && sum == 0u; attempt++)
        sum = capture_run();

    TIM21_CR1 = 0;
    SET_BITS(RCC_APB2RSTR_REG, APB2_TIM21);           /* back to reset state */
    CLR_BITS(RCC_APB2RSTR_REG, APB2_TIM21);
    CLR_BITS(RCC_APB2ENR_REG, APB2_TIM21);
    if (sum == 0u) return 0u;

    /* f_lsi = f_tim x edges / counts, without 64-bit division:
     * f_tim <= 32 MHz and sum < 2^18, so neither product overflows. */
    const uint32_t edges = LSI_EDGES_PER_CAPTURE * LSI_CAPTURES;   /* 128 */
    uint32_t f = tim21_clk_hz();
    return (f / sum) * edges + ((f % sum) * edges + sum / 2u) / sum;
}

uint32_t ll_lsi_hz(void)
{
    if (s_lsi_hz == 0u) {
        uint32_t hz = hal_lsi_measure_hz();
        s_lsi_hz = (hz >= LSI_MIN_HZ && hz <= LSI_MAX_HZ) ? hz : LL_LSI_HZ;
    }
    return s_lsi_hz;
}

#endif /* STM32L011xx */
