/**
 * ll_rcc.h — Low-level RCC (Reset and Clock Control)
 *
 * Peripheral clock enable/disable, clock source selection, PLL
 * configuration, and flash latency. RCC register layouts differ
 * significantly across STM32 families, so this header provides a
 * unified API with per-family implementations.
 */

#ifndef LL_RCC_H
#define LL_RCC_H

#include "ll_common.h"

/* ---- RCC base address ---- */

#if defined(STM32L011xx)
  #define RCC_BASE          (AHB_BASE + 0x1000UL)
#elif defined(STM32L422xx)
  #define RCC_BASE          (AHB1_BASE + 0x1000UL)
#elif defined(STM32WBA55xx)
  #define RCC_BASE          (AHB4_BASE + 0x0C00UL)
#elif defined(STM32H523xx)
  #define RCC_BASE          (AHB1_BASE + 0x0C00UL)
#endif

/* ---- Flash base address (for wait state config) ---- */

#if defined(STM32L011xx)
  #define FLASH_BASE        (PERIPH_BASE + 0x00022000UL)
#elif defined(STM32L422xx)
  #define FLASH_BASE        (PERIPH_BASE + 0x00022000UL)
#elif defined(STM32WBA55xx)
  #define FLASH_BASE        (PERIPH_BASE + 0x00022000UL)
#elif defined(STM32H523xx)
  #define FLASH_BASE        (PERIPH_BASE + 0x00022000UL)
#endif

#define FLASH_ACR           REG32(FLASH_BASE + 0x00UL)

/* ============================================================
 * Clock source enable/ready
 * ============================================================ */

/* ---- HSI16 (16MHz internal, available on L0/L4/WBA) ---- */

#if defined(STM32L011xx)

/** Enable the high-speed internal (HSI16) oscillator. */
static inline void ll_rcc_hsi16_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 0));  /* CR: HSION */
}
static inline int ll_rcc_hsi16_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 2)) != 0;  /* CR: HSIRDY */
}

#elif defined(STM32L422xx)

static inline void ll_rcc_hsi16_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 8));  /* CR: HSION */
}
static inline int ll_rcc_hsi16_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 10)) != 0;  /* CR: HSIRDY */
}

#elif defined(STM32WBA55xx)

static inline void ll_rcc_hsi16_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 8));  /* CR: HSION */
}
static inline int ll_rcc_hsi16_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 10)) != 0;  /* CR: HSIRDY */
}

#elif defined(STM32H523xx)

/* H5 "HSI" is 64 MHz (called HSI16 here for coregen compatibility) */
static inline void ll_rcc_hsi16_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 0));  /* CR: HSION */
}
static inline int ll_rcc_hsi16_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 1)) != 0;  /* CR: HSIRDY */
}

#endif /* HSI16 */

/* ---- CSI (4 MHz low-power internal, H5 only) ---- */

#if defined(STM32H523xx)

static inline void ll_rcc_csi_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 8));  /* CR: CSION */
}
static inline int ll_rcc_csi_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 9)) != 0;  /* CR: CSIRDY */
}

#endif /* CSI */

/* ---- MSI (Multi-Speed Internal, L0/L4) ---- */

#if defined(STM32L422xx)

/* MSIRANGE values for STM32L4 (4-bit field in RCC_CR[7:4]).
 * Set MSIRGSEL (CR[3]=1) to use this field instead of the CSR range. */
#define LL_RCC_MSI_RANGE_1MHZ    0x4UL
#define LL_RCC_MSI_RANGE_2MHZ    0x5UL
#define LL_RCC_MSI_RANGE_4MHZ    0x6UL   /* reset default */
#define LL_RCC_MSI_RANGE_8MHZ    0x7UL
#define LL_RCC_MSI_RANGE_16MHZ   0x8UL
#define LL_RCC_MSI_RANGE_24MHZ   0x9UL
#define LL_RCC_MSI_RANGE_32MHZ   0xAUL
#define LL_RCC_MSI_RANGE_48MHZ   0xBUL

/** Configure the MSI range and select it via CR (not CSR). */
static inline void ll_rcc_msi_set_range(uint32_t range)
{
    uint32_t cr = REG32(RCC_BASE + 0x00UL);
    cr &= ~(0xFUL << 4);   /* clear MSIRANGE[7:4] */
    cr |= (range << 4);    /* set new range */
    cr |= (1UL << 3);      /* MSIRGSEL=1: use CR MSIRANGE (not CSR) */
    REG32(RCC_BASE + 0x00UL) = cr;
}

/** Poll until MSI oscillator is stable. */
static inline int ll_rcc_msi_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 1)) != 0;  /* CR: MSIRDY */
}

#elif defined(STM32L011xx)

/* MSIRANGE for STM32L0 (3-bit field in RCC_ICSCR[15:13]).
 * Range values: 0=65kHz, 1=131kHz, 2=262kHz, 3=524kHz,
 *               4=1.048MHz, 5=2.097MHz, 6=4.194MHz */
#define LL_RCC_MSI_RANGE_1MHZ    0x4UL   /* 1.048576 MHz */
#define LL_RCC_MSI_RANGE_2MHZ    0x5UL   /* 2.097152 MHz (reset default) */
#define LL_RCC_MSI_RANGE_4MHZ    0x6UL   /* 4.194304 MHz */

/** Configure MSI range on STM32L0 (RCC_ICSCR[15:13]).
 *  Only modifies MSIRANGE — preserves the MSI/HSI16 calibration and trim
 *  fields around it (MSITRIM is ICSCR[31:24], MSICAL [23:16], RM0377 §7.3.2). */
static inline void ll_rcc_msi_set_range(uint32_t range)
{
    MOD_BITS(REG32(RCC_BASE + 0x04UL), 0x7UL << 13, range << 13);
}

/** Poll until MSI is stable. On the L0 MSIRDY is RCC_CR bit 9 (MSION is bit
 *  8, RM0377 §7.3.1); bit 1 is HSI16KERON, which reads 0, so polling it hung
 *  every MSI range change. */
static inline int ll_rcc_msi_ready(void)
{
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 9)) != 0;
}

#endif /* MSI */

/* ---- HSE (external oscillator) ---- */

/** Enable the high-speed external (HSE) oscillator / crystal. */
static inline void ll_rcc_hse_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 16));  /* CR: HSEON */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 16));  /* CR: HSEON */
#elif defined(STM32WBA55xx)
    /* WBA55 has HSEPRE (bit 20): divides HSE by 2 for SYSCLK if set.
     * Explicitly clear HSEPRE (HSE/1) and set HSEON together, matching
     * the HAL __HAL_RCC_HSE_CONFIG(RCC_HSE_ON | RCC_HSE_DIV1) pattern.
     * HSEPRE may be set by option bytes from prior BLE firmware. */
    MOD_BITS(REG32(RCC_BASE + 0x00UL),
             (1UL << 16) | (1UL << 20),   /* mask: HSEON | HSEPRE */
             (1UL << 16));                 /* value: HSEON=1, HSEPRE=0 */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 16));  /* CR: HSEON */
#endif
}

static inline int ll_rcc_hse_ready(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 17)) != 0;
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 17)) != 0;
#elif defined(STM32WBA55xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 17)) != 0;
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 17)) != 0;
#else
    return 0;
#endif
}

/* ---- HSI48 (48MHz internal, L4/H5) ---- */

#if defined(STM32L422xx) || defined(STM32H523xx)

static inline void ll_rcc_hsi48_enable(void)
{
#if defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x98UL), (1UL << 0));  /* CRRCR: HSI48ON */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 12));  /* CR: HSI48ON */
#endif
}

static inline int ll_rcc_hsi48_ready(void)
{
#if defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x98UL) & (1UL << 1)) != 0;  /* CRRCR: HSI48RDY */
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 13)) != 0;  /* CR: HSI48RDY */
#else
    return 0;
#endif
}

#endif /* HSI48 */

/* ============================================================
 * Flash wait states
 * ============================================================ */

/* LATENCY field of FLASH_ACR. The L0 has one bit (bit 0; bits 1-6 are
 * PRFTEN, SLEEP_PD, RUN_PD, DISAB_BUF, PRE_READ — RM0377 §3.7.1), so a 4-bit
 * mask there would clear the prefetch and power-down settings. */
#if defined(STM32L011xx)
  #define LL_FLASH_LATENCY_MASK  0x1UL
#else
  #define LL_FLASH_LATENCY_MASK  0xFUL
#endif

/**
 * Set flash read latency (wait states).
 * Must be set BEFORE increasing SYSCLK, or AFTER decreasing it.
 * See ll_flash_latency_for_range() for the tables.
 */
static inline void ll_flash_set_latency(uint32_t wait_states)
{
    MOD_BITS(FLASH_ACR, LL_FLASH_LATENCY_MASK, wait_states);
    /* Wait until the new latency is effective */
    uint32_t timeout = 100000;
    while ((FLASH_ACR & LL_FLASH_LATENCY_MASK) != wait_states && --timeout)
        ;
}

/** Current flash read latency (wait states). */
static inline uint32_t ll_flash_get_latency(void)
{
    return FLASH_ACR & LL_FLASH_LATENCY_MASK;
}

/**
 * Flash wait states for an HCLK of `mhz` (rounded up) in voltage range
 * `range` (1 = the highest-performance range).
 *
 *   L0  (RM0377 Table 14): range 1: 0 WS <= 16, 1 WS <= 32 MHz
 *                          range 2: 0 WS <= 8,  1 WS <= 16 MHz
 *                          range 3: 0 WS up to its 4.2 MHz limit
 *   L4  (RM0394 Table 9):  range 1: 0/1/2/3/4 WS <= 16/32/48/64/80 MHz
 *                          range 2: 0/1/2/3 WS <= 6/12/18/26 MHz
 *   WBA (RM0493 Table 41, LPM = 0):
 *                          range 1: 0/1/2/3 WS <= 32/64/96/100 MHz
 *                          range 2: 0 WS <= 8, 1 WS <= 16 MHz
 *   H5: range argument ignored (ll_flash_latency_for_mhz table).
 */
static inline uint32_t ll_flash_latency_for_range(uint32_t mhz, uint32_t range)
{
#if defined(STM32L011xx)
    if (range == 1) return (mhz > 16) ? 1 : 0;
    if (range == 2) return (mhz > 8) ? 1 : 0;
    return 0;
#elif defined(STM32L422xx)
    if (range == 2) {
        if (mhz <= 6)  return 0;
        if (mhz <= 12) return 1;
        if (mhz <= 18) return 2;
        return 3;
    }
    if (mhz <= 16) return 0;
    if (mhz <= 32) return 1;
    if (mhz <= 48) return 2;
    if (mhz <= 64) return 3;
    return 4;  /* up to 80 MHz */
#elif defined(STM32WBA55xx)
    if (range == 2) return (mhz > 8) ? 1 : 0;
    if (mhz <= 32) return 0;
    if (mhz <= 64) return 1;
    if (mhz <= 96) return 2;
    return 3;  /* up to 100 MHz */
#elif defined(STM32H523xx)
    (void)range;
    if (mhz <= 32) return 0;
    if (mhz <= 64) return 1;
    if (mhz <= 96) return 2;
    if (mhz <= 128) return 3;
    if (mhz <= 160) return 4;
    return 5;  /* up to 250 MHz */
#else
    (void)mhz; (void)range;
    return 0;
#endif
}

/**
 * Flash wait states for SYSCLK `mhz` in the highest voltage range (range 1 on
 * L0/L4/WBA). The WBA and L0 generated clock setup uses
 * ll_flash_latency_for_range(), which also covers their lower ranges.
 */
static inline uint32_t ll_flash_latency_for_mhz(uint32_t mhz)
{
    return ll_flash_latency_for_range(mhz, 1);
}

/* ============================================================
 * PLL configuration
 * ============================================================ */

/* PLL source selection values */
#if defined(STM32L011xx)
  #define LL_RCC_PLLSRC_HSI16  0x0UL
  #define LL_RCC_PLLSRC_HSE    0x1UL
#elif defined(STM32L422xx)
  #define LL_RCC_PLLSRC_NONE   0x0UL
  #define LL_RCC_PLLSRC_MSI    0x1UL
  #define LL_RCC_PLLSRC_HSI16  0x2UL
  #define LL_RCC_PLLSRC_HSE    0x3UL
#elif defined(STM32WBA55xx)
  #define LL_RCC_PLLSRC_NONE   0x0UL
  #define LL_RCC_PLLSRC_HSI16  0x2UL
  #define LL_RCC_PLLSRC_HSE    0x3UL
#elif defined(STM32H523xx)
  #define LL_RCC_PLLSRC_NONE   0x0UL
  #define LL_RCC_PLLSRC_HSI48  0x1UL  /* HSI 64 MHz as PLL source */
  #define LL_RCC_PLLSRC_HSI16  0x1UL  /* alias for coregen compatibility */
  #define LL_RCC_PLLSRC_CSI    0x2UL
  #define LL_RCC_PLLSRC_HSE    0x3UL
#endif

/**
 * Configure and enable the main PLL.
 *   src:  PLL source (LL_RCC_PLLSRC_*)
 *   m:    input divider (1-based)
 *   n:    VCO multiplier
 *   r:    output divider for SYSCLK
 *
 * PLL must be disabled before calling this function.
 */
static inline void ll_rcc_pll_config(uint32_t src, uint32_t m, uint32_t n, uint32_t r)
{
#if defined(STM32L011xx)
    /* L0: RCC_CFGR bits [21:18]=PLLMUL, [23:22]=PLLDIV, [16]=PLLSRC.
       m is ignored (no input divider). n = multiplier (3,4,6,8,12,16,24,32,48).
       r = output divider (2,3,4). PLLDIV encoding: 01=÷2, 10=÷3, 11=÷4. */
    {
        (void)m;
        uint32_t mul_enc;
        switch (n) {
            case  3: mul_enc = 0x0UL; break;
            case  4: mul_enc = 0x1UL; break;
            case  6: mul_enc = 0x2UL; break;
            case  8: mul_enc = 0x3UL; break;
            case 12: mul_enc = 0x4UL; break;
            case 16: mul_enc = 0x5UL; break;
            case 24: mul_enc = 0x6UL; break;
            case 32: mul_enc = 0x7UL; break;
            default: mul_enc = 0x8UL; break; /* 48× */
        }
        uint32_t div_enc = (uint32_t)(r - 1);  /* ÷2→1, ÷3→2, ÷4→3 */
        uint32_t cfgr = REG32(RCC_BASE + 0x0CUL);
        cfgr &= ~((0xFUL << 18) | (0x3UL << 22) | (0x1UL << 16));
        cfgr |= (mul_enc << 18) | (div_enc << 22) | (src << 16);
        REG32(RCC_BASE + 0x0CUL) = cfgr;
    }

#elif defined(STM32L422xx)
    /* L4: RCC_PLLCFGR at offset 0x0C
       [1:0] PLLSRC, [6:4] PLLM-1, [14:8] PLLN, [26:25] PLLR/2-1, [24] PLLREN */
    uint32_t val = src
                 | ((m - 1) << 4)
                 | (n << 8)
                 | (((r / 2) - 1) << 25)
                 | (1UL << 24);  /* PLLREN: enable R output */
    REG32(RCC_BASE + 0x0CUL) = val;

#elif defined(STM32WBA55xx)
    /* WBA: RCC_PLL1CFGR at offset 0x28, RCC_PLL1DIVR at offset 0x34
       CFGR: [1:0] PLL1SRC, [3:2] PLL1RGE, [10:8] PLL1M-1, [18] PLL1REN
       DIVR: [8:0] PLL1N-1, [30:24] PLL1R-1
       (PLL1M_Pos=8, PLL1REN_Pos=18, PLL1RGE_Pos=2 per stm32wba55xx.h CMSIS header) */
    {
        /* PLL1RGE[3:2], RM0493 §12.8.7: 00/01/10 = ref_ck 4-8 MHz, 11 = 8-16 MHz.
         * (This wrote 01 for 8-16 MHz and 10 for 16-32 MHz, which isn't a legal
         * PLL input at all — ref_ck is 4-16 MHz, §12.4.3.) Matches the CubeWBA
         * HAL: 11 above 8 MHz, else 00. */
        uint32_t vco_mhz = (src == LL_RCC_PLLSRC_HSE) ? (32 / m) : (16 / m);
        uint32_t rge = (vco_mhz > 8) ? 0x3UL : 0x0UL;
        uint32_t cfgr = src
                      | (rge << 2)         /* PLL1RGE */
                      | ((m - 1) << 8)     /* PLL1M */
                      | (1UL << 18);       /* PLL1REN: enable R output */
        uint32_t divr = ((n - 1) << 0)
                      | ((r - 1) << 24);
        REG32(RCC_BASE + 0x28UL) = cfgr;
        REG32(RCC_BASE + 0x34UL) = divr;
    }

#elif defined(STM32H523xx)
    /* H5: RCC_PLL1CFGR at offset 0x28, RCC_PLL1DIVR at offset 0x34
       CFGR: [1:0] PLL1SRC, [3:2] PLL1RGE, [12:7] PLL1M-1, [16] PLL1PEN
       DIVR: [8:0] PLL1N-1, [15:9] PLL1P-1 (P=SYSCLK output, odd values only)
       Note: SYSCLK uses PLL1P output (not PLL1R like WBA55) */
    /* H5: Compute RGE from M and write PLL1CFGR in a single atomic write
     * to avoid read-modify-write losing bit 7 (M LSB). */
    {
        uint32_t vco_in_mhz = 64 / m;  /* HSI = 64 MHz (after HSIDIV clear) */
        uint32_t rge = (vco_in_mhz > 8) ? 0x3UL : (vco_in_mhz > 4) ? 0x2UL :
                       (vco_in_mhz > 2) ? 0x1UL : 0x0UL;
        uint32_t cfgr = src
                      | (rge << 2)            /* PLL1RGE at [3:2] */
                      | (m << 8)              /* PLL1M at [13:8] (direct, not M-1) */
                      | (1UL << 16);          /* PLL1PEN */
        REG32(RCC_BASE + 0x28UL) = cfgr;
    }
    {
        uint32_t divr = ((n - 1) << 0)    /* PLL1N at [8:0] */
                      | ((r - 1) << 9);   /* PLL1P at [15:9] */
        REG32(RCC_BASE + 0x34UL) = divr;
    }
#endif
}

/**
 * Set PLL VCO input frequency range.
 * Must be called after ll_rcc_pll_config() and before ll_rcc_pll_enable().
 *   vco_input_mhz: VCO input = source_mhz / M
 *
 * WBA55 PLL1CFGR[3:2]   PLL1RGE: 00/01/10 = 4–8 MHz, 11 = 8–16 MHz
 * H5    PLL1CFGR[4:3]   PLL1RGE: 00 = 1–2 MHz, 01 = 2–4 MHz, 10 = 4–8 MHz, 11 = 8–16 MHz
 */
static inline void ll_rcc_pll_set_input_range(uint32_t vco_input_mhz)
{
#if defined(STM32WBA55xx)
    /* PLL1CFGR[3:2] = PLL1RGE: 00/01/10 = 4-8 MHz, 11 = 8-16 MHz (RM0493 §12.8.7) */
    uint32_t rge = (vco_input_mhz > 8) ? 0x3UL : 0x0UL;
    MOD_BITS(REG32(RCC_BASE + 0x28UL), 0x3UL << 2, rge << 2);
#elif defined(STM32H523xx)
    uint32_t rge = (vco_input_mhz > 8) ? 0x3UL : (vco_input_mhz > 4) ? 0x2UL :
                   (vco_input_mhz > 2) ? 0x1UL : 0x0UL;
    MOD_BITS(REG32(RCC_BASE + 0x28UL), 0x3UL << 2, rge << 2);  /* PLL1RGE[3:2] */
#else
    (void)vco_input_mhz;
#endif
}

/** Enable the main PLL */
static inline void ll_rcc_pll_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 24));  /* CR: PLLON */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 24));  /* CR: PLLON */
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 24));  /* CR: PLL1ON */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 24));  /* CR: PLL1ON */
#endif
}

/** Check if PLL is locked and ready */
static inline int ll_rcc_pll_ready(void)
{
#if defined(STM32L011xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 25)) != 0;  /* CR: PLLRDY */
#elif defined(STM32L422xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 25)) != 0;
#elif defined(STM32WBA55xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 25)) != 0;
#elif defined(STM32H523xx)
    return (REG32(RCC_BASE + 0x00UL) & (1UL << 25)) != 0;
#else
    return 0;
#endif
}

/**
 * Enable PLL and wait for lock with timeout.
 * Returns 1 on success, 0 on timeout (PLL failed to lock).
 */
static inline int ll_rcc_pll_enable_timeout(uint32_t retries)
{
    ll_rcc_pll_enable();
    while (!ll_rcc_pll_ready()) {
        if (--retries == 0) return 0;
    }
    return 1;
}

/** Disable the main PLL and wait until it has stopped (PLLRDY = 0). It must
 *  not be the SYSCLK source. PLL settings can only be written while it is off. */
static inline void ll_rcc_pll_disable(void)
{
    CLR_BITS(REG32(RCC_BASE + 0x00UL), (1UL << 24));  /* CR: PLLON / PLL1ON */
    uint32_t timeout = 100000;
    while (ll_rcc_pll_ready() && --timeout)
        ;
}

/**
 * Wait for HSI16 ready with timeout.
 * Returns 1 on success, 0 on timeout.
 */
static inline int ll_rcc_hsi16_enable_timeout(uint32_t retries)
{
#if !defined(STM32H523xx)
    ll_rcc_hsi16_enable();
    while (!ll_rcc_hsi16_ready()) {
        if (--retries == 0) return 0;
    }
    return 1;
#else
    (void)retries;
    return 0;
#endif
}

/* ============================================================
 * SYSCLK source selection
 * ============================================================ */

/* SYSCLK source values (for RCC_CFGR SW bits) */
#if defined(STM32L011xx)
  #define LL_RCC_SYSCLK_MSI    0x0UL
  #define LL_RCC_SYSCLK_HSI16  0x1UL
  #define LL_RCC_SYSCLK_HSE    0x2UL
  #define LL_RCC_SYSCLK_PLL    0x3UL
  #define RCC_CFGR_OFFSET      0x0CUL
  #define RCC_CFGR_SW_MASK     0x3UL
  #define RCC_CFGR_SWS_SHIFT   2
#elif defined(STM32L422xx)
  #define LL_RCC_SYSCLK_MSI    0x0UL
  #define LL_RCC_SYSCLK_HSI16  0x1UL
  #define LL_RCC_SYSCLK_HSE    0x2UL
  #define LL_RCC_SYSCLK_PLL    0x3UL
  #define RCC_CFGR_OFFSET      0x08UL
  #define RCC_CFGR_SW_MASK     0x3UL
  #define RCC_CFGR_SWS_SHIFT   2
#elif defined(STM32WBA55xx)
  /* WBA55 CFGR1 SW[1:0]: 00=HSI16, 10=HSE, 11=PLL1 (01=reserved)
   * Per HAL: RCC_SYSCLKSOURCE_HSE = RCC_CFGR1_SW_1 = 0x2,
   *          RCC_SYSCLKSOURCE_PLLCLK = SW_1|SW_0 = 0x3              */
  #define LL_RCC_SYSCLK_HSI16  0x0UL
  #define LL_RCC_SYSCLK_HSE    0x2UL
  #define LL_RCC_SYSCLK_PLL    0x3UL
  #define RCC_CFGR_OFFSET      0x1CUL  /* RCC_CFGR1 */
  #define RCC_CFGR_SW_MASK     0x3UL
  #define RCC_CFGR_SWS_SHIFT   2      /* RCC_CFGR1_SWS_Pos = 2 */
#elif defined(STM32H523xx)
  #define LL_RCC_SYSCLK_HSI16  0x0UL  /* HSI on H5 is 64 MHz; SW=0 selects it */
  #define LL_RCC_SYSCLK_HSI48  0x0UL  /* alias — same oscillator */
  #define LL_RCC_SYSCLK_CSI    0x1UL
  #define LL_RCC_SYSCLK_HSE    0x2UL
  #define LL_RCC_SYSCLK_PLL    0x3UL
  #define RCC_CFGR_OFFSET      0x1CUL  /* RCC_CFGR1 */
  #define RCC_CFGR_SW_MASK     0x7UL
  #define RCC_CFGR_SWS_SHIFT   3
#endif

/** Set the SYSCLK source */
static inline void ll_rcc_set_sysclk(uint32_t source)
{
    MOD_BITS(REG32(RCC_BASE + RCC_CFGR_OFFSET), RCC_CFGR_SW_MASK, source);
}

/** Wait until the SYSCLK source has switched */
static inline void ll_rcc_wait_sysclk(uint32_t source)
{
    uint32_t sws_mask = RCC_CFGR_SW_MASK << RCC_CFGR_SWS_SHIFT;
    uint32_t sws_val = source << RCC_CFGR_SWS_SHIFT;
    uint32_t timeout = 100000;
    while ((REG32(RCC_BASE + RCC_CFGR_OFFSET) & sws_mask) != sws_val && --timeout)
        ;
}

/* ============================================================
 * Bus prescalers (AHB, APB1, APB2)
 * ============================================================ */

/**
 * Set AHB prescaler.
 *   div: 1, 2, 4, 8, 16, 64, 128, 256, 512
 */
static inline void ll_rcc_set_ahb_div(uint32_t div)
{
    uint32_t val;
    switch (div) {
        case 1:   val = 0x0; break;
        case 2:   val = 0x8; break;
        case 4:   val = 0x9; break;
        case 8:   val = 0xA; break;
        case 16:  val = 0xB; break;
        case 64:  val = 0xC; break;
        case 128: val = 0xD; break;
        case 256: val = 0xE; break;
        default:  val = 0xF; break;  /* 512 */
    }
#if defined(STM32L011xx)
    MOD_BITS(REG32(RCC_BASE + 0x0CUL), 0xFUL << 4, val << 4);    /* CFGR HPRE */
#elif defined(STM32L422xx)
    MOD_BITS(REG32(RCC_BASE + 0x08UL), 0xFUL << 4, val << 4);    /* CFGR HPRE */
#elif defined(STM32WBA55xx)
    /* RM0493 §12.8.5: HPRE is RCC_CFGR2[2:0] (3 bits: 0xx = /1, 100 = /2,
     * 101 = /4, 110 = /8, 111 = /16). CFGR1[23:4] is reserved; this used to
     * write CFGR1[11:8]. Every generated project divides by 1, so the old
     * write was a harmless zero, but a divider would have gone nowhere. */
    {
        uint32_t v3 = (div >= 16) ? 0x7UL : (div >= 8) ? 0x6UL : (div >= 4) ? 0x5UL
                    : (div >= 2) ? 0x4UL : 0x0UL;
        (void)val;
        MOD_BITS(REG32(RCC_BASE + 0x20UL), 0x7UL << 0, v3 << 0);  /* CFGR2 HPRE */
    }
#elif defined(STM32H523xx)
    MOD_BITS(REG32(RCC_BASE + 0x1CUL), 0xFUL << 8, val << 8);    /* CFGR1 HPRE */
#endif
}

/**
 * Set APB1 prescaler.
 *   div: 1, 2, 4, 8, 16
 */
static inline void ll_rcc_set_apb1_div(uint32_t div)
{
    uint32_t val;
    switch (div) {
        case 1:  val = 0x0; break;
        case 2:  val = 0x4; break;
        case 4:  val = 0x5; break;
        case 8:  val = 0x6; break;
        default: val = 0x7; break;  /* 16 */
    }
#if defined(STM32L011xx)
    MOD_BITS(REG32(RCC_BASE + 0x0CUL), 0x7UL << 8, val << 8);    /* CFGR PPRE1 */
#elif defined(STM32L422xx)
    MOD_BITS(REG32(RCC_BASE + 0x08UL), 0x7UL << 8, val << 8);    /* CFGR PPRE1 */
#elif defined(STM32WBA55xx)
    MOD_BITS(REG32(RCC_BASE + 0x20UL), 0x7UL << 4, val << 4);    /* CFGR2 PPRE1 (RM0493 §12.8.5) */
#elif defined(STM32H523xx)
    MOD_BITS(REG32(RCC_BASE + 0x1CUL), 0x7UL << 12, val << 12);  /* CFGR1 PPRE1 */
#endif
}

/**
 * Set APB2 prescaler.
 *   div: 1, 2, 4, 8, 16
 */
static inline void ll_rcc_set_apb2_div(uint32_t div)
{
    uint32_t val;
    switch (div) {
        case 1:  val = 0x0; break;
        case 2:  val = 0x4; break;
        case 4:  val = 0x5; break;
        case 8:  val = 0x6; break;
        default: val = 0x7; break;  /* 16 */
    }
#if defined(STM32L011xx)
    MOD_BITS(REG32(RCC_BASE + 0x0CUL), 0x7UL << 11, val << 11);  /* CFGR PPRE2 */
#elif defined(STM32L422xx)
    MOD_BITS(REG32(RCC_BASE + 0x08UL), 0x7UL << 11, val << 11);  /* CFGR PPRE2 */
#elif defined(STM32WBA55xx)
    MOD_BITS(REG32(RCC_BASE + 0x20UL), 0x7UL << 8, val << 8);    /* CFGR2 PPRE2 (RM0493 §12.8.5; was PPRE1's [6:4]) */
#elif defined(STM32H523xx)
    MOD_BITS(REG32(RCC_BASE + 0x20UL), 0x7UL << 4, val << 4);    /* CFGR2 PPRE2 */
#endif
}

/* ============================================================
 * GPIO clock enable/disable (unchanged from before)
 * ============================================================ */

/** Enable the clock for a GPIO port. */
static inline void ll_rcc_gpio_clk_enable(GPIO_TypeDef *port)
{
    uint32_t index = ((uint32_t)port - (uint32_t)GPIOA) / 0x0400UL;
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x2CUL), (1UL << index));
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x4CUL), (1UL << index));
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << index));
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << index));
#endif
    (void)REG32(RCC_BASE);
    (void)REG32(RCC_BASE);
}

/** Disable the clock for a GPIO port. */
static inline void ll_rcc_gpio_clk_disable(GPIO_TypeDef *port)
{
    uint32_t index = ((uint32_t)port - (uint32_t)GPIOA) / 0x0400UL;
#if defined(STM32L011xx)
    CLR_BITS(REG32(RCC_BASE + 0x2CUL), (1UL << index));
#elif defined(STM32L422xx)
    CLR_BITS(REG32(RCC_BASE + 0x4CUL), (1UL << index));
#elif defined(STM32WBA55xx)
    CLR_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << index));
#elif defined(STM32H523xx)
    CLR_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << index));
#endif
}

/* ============================================================
 * Peripheral clock enable (by bus)
 * ============================================================ */

static inline void ll_rcc_apb1_clk_enable(uint32_t mask)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x38UL), mask);
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x58UL), mask);
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0x9CUL), mask);
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x9CUL), mask);
#endif
    (void)REG32(RCC_BASE);
}

static inline void ll_rcc_apb2_clk_enable(uint32_t mask)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x34UL), mask);
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x60UL), mask);
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0xA4UL), mask);
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0xA4UL), mask);
#endif
    (void)REG32(RCC_BASE);
}

#if defined(STM32WBA55xx)

/** Enable APB7 peripheral clocks (WBA only: I2C3, LPTIM1, SPI3, LPUART1). */
static inline void ll_rcc_apb7_clk_enable(uint32_t mask)
{
    SET_BITS(REG32(RCC_BASE + 0xA8UL), mask);  /* APB7ENR */
    (void)REG32(RCC_BASE);
}

/* AHB4/AHB5 bit masks (defined here so functions below can use them) */
#define LL_AHB4_PWR       (1UL << 2)   /* PWREN */
#define LL_AHB4_ADC4      (1UL << 5)   /* ADC4EN */
#define LL_AHB5_RADIO     (1UL << 0)

/** Enable AHB4 peripheral clocks (WBA only: PWR, ADC4). */
static inline void ll_rcc_ahb4_clk_enable(uint32_t mask)
{
    SET_BITS(REG32(RCC_BASE + 0x94UL), mask);  /* AHB4ENR */
    /* Read back AHB4ENR (not just RCC_CR) to ensure clock enable has
     * propagated before accessing the peripheral — matches ST LL pattern. */
    (void)REG32(RCC_BASE + 0x94UL);
}

/* ---- AHB5 peripheral clock (RADIO) ---- */

static inline void ll_rcc_ahb5_clk_enable(uint32_t mask)
{
    /* RCC_AHB5ENR at offset 0x098 */
    SET_BITS(REG32(RCC_BASE + 0x98UL), mask);
    (void)REG32(RCC_BASE + 0x98UL);
}

static inline void ll_rcc_ahb5_clk_sleep_enable(void)
{
    /* RCC_AHB5SMENR at offset 0x0C0 — keep RADIO clock in sleep */
    SET_BITS(REG32(RCC_BASE + 0x0C0UL), LL_AHB5_RADIO);
}

static inline void ll_rcc_ahb5_clk_sleep_disable(void)
{
    CLR_BITS(REG32(RCC_BASE + 0x0C0UL), LL_AHB5_RADIO);
}

/* ---- PWR: backup domain access ---- */

/* PWR_BASE has one definition, ll_pwr.h's per-MCU table. This header used to
 * carry a second (AHB5_BASE + 0x0800UL, the same address) behind an #ifndef —
 * whichever header a translation unit reached first won, and the other warned
 * "PWR_BASE redefined" on every WBA build. Identical values made that harmless
 * but include-order-dependent; a change to one would have diverged silently.
 * (ll_pwr.h pulls in only ll_common.h, so there is no cycle here.) */
#include "ll_pwr.h"

/** Get the 2.4 GHz radio operating mode: PWR_RADIOSCR.MODE[1:0] (offset
 *  0x100, RM0493 §11.10.24): 00 deep sleep, 01 sleep, 1x active. This read
 *  PWR_SR1 bits [2:1], which are SBF:STOPF, so it never reported "active";
 *  CubeWBA's LL_PWR_GetRadioMode() reads RADIOSCR. */
static inline uint32_t ll_pwr_get_radio_mode(void)
{
    return REG32(PWR_BASE + 0x100UL) & 0x3UL;
}

#define LL_PWR_RADIO_ACTIVE_MODE    0x2UL
#define LL_PWR_RADIO_DEEPSLEEP      0x0UL

/* ---- LSI1 oscillator ---- */

/** Enable LSI1 (32kHz internal RC). Requires backup domain access. */
static inline void ll_rcc_lsi1_enable(void)
{
    /* RCC_BDCR1 at offset 0x0F0, LSI1ON = bit 26 */
    SET_BITS(REG32(RCC_BASE + 0xF0UL), (1UL << 26));
}

/** Check if LSI1 is ready. */
static inline int ll_rcc_lsi1_ready(void)
{
    /* RCC_BDCR1, LSI1RDY = bit 27 */
    return (REG32(RCC_BASE + 0xF0UL) & (1UL << 27)) != 0;
}

/** Wait for LSI1 ready with timeout. Returns 1 on success, 0 on timeout. */
static inline int ll_rcc_lsi1_enable_wait(void)
{
    ll_rcc_lsi1_enable();
    for (volatile uint32_t i = 0; i < 100000; i++)
        if (ll_rcc_lsi1_ready()) return 1;
    return 0;
}

/* ---- Radio sleep timer clock source ---- */

/* Radio sleep timer clock source — values shifted by 18 into BDCR1.
 * WARNING: ST LL uses pre-shifted bit masks, we use 2-bit field values.
 *   Value 0 = NONE, 1 = LSE, 2 = LSI, 3 = HSE/1000 */
#define LL_RCC_RADIOSLEEPSOURCE_NONE     0x0UL
#define LL_RCC_RADIOSLEEPSOURCE_LSE      0x1UL
#define LL_RCC_RADIOSLEEPSOURCE_LSI      0x2UL
#define LL_RCC_RADIOSLEEPSOURCE_HSE_DIV  0x3UL  /* HSE / 1000 */

/** Set the radio sleep timer clock source. Requires backup domain access. */
static inline void ll_rcc_set_radio_sleep_clk(uint32_t src)
{
    /* RCC_BDCR1, RADIOSTSEL bits [19:18] */
    MOD_BITS(REG32(RCC_BASE + 0xF0UL), 0x3UL << 18, (src & 0x3UL) << 18);
}

/** Get the current radio sleep timer clock source. */
static inline uint32_t ll_rcc_get_radio_sleep_clk(void)
{
    return (REG32(RCC_BASE + 0xF0UL) >> 18) & 0x3UL;
}

/* ---- Radio baseband clock (active clock for 2.4GHz radio) ---- */
/* RCC_RADIOENR at offset 0x208:
 *   Bit 1:  BBCLKEN       — baseband clock enable
 *   Bit 16: STRADIOCLKON  — sleep timer radio clock on (enables HSE+bus on wakeup)
 *   Bit 17: RADIOCLKRDY   — radio clock ready (read-only status)
 * Note: there is NO bit 0 (RADIOENEN) — that was a documentation error. */

static inline void ll_rcc_radio_bb_clk_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x208UL), (1UL << 1));   /* BBCLKEN */
}

static inline void ll_rcc_radio_bb_clk_disable(void)
{
    CLR_BITS(REG32(RCC_BASE + 0x208UL), (1UL << 1));   /* BBCLKEN */
}

static inline void ll_rcc_radio_wakeup_clk_enable(void)
{
    SET_BITS(REG32(RCC_BASE + 0x208UL), (1UL << 16));  /* STRADIOCLKON */
}

static inline int ll_rcc_radio_clk_ready(void)
{
    return (REG32(RCC_BASE + 0x208UL) & (1UL << 17)) != 0;  /* RADIOCLKRDY */
}

#endif /* STM32WBA55xx */

#if defined(STM32H523xx)

/** Enable APB3 peripheral clocks (H5 only: I2C3, SPI3, LPUART1). */
static inline void ll_rcc_apb3_clk_enable(uint32_t mask)
{
    SET_BITS(REG32(RCC_BASE + 0xA8UL), mask);  /* APB3ENR */
    (void)REG32(RCC_BASE);
}

#endif /* STM32H523xx */

/** Enable AHB2 peripheral clocks. */
static inline void ll_rcc_ahb2_clk_enable(uint32_t mask)
{
#if defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x4CUL), mask);
#elif defined(STM32WBA55xx)
    SET_BITS(REG32(RCC_BASE + 0x8CUL), mask);  /* AHB2ENR */
#elif defined(STM32H523xx)
    SET_BITS(REG32(RCC_BASE + 0x8CUL), mask);
#endif
    /* No L011 branch: that part has no AHB2 bus. Nothing reaches here on it
     * either — it has no RNG/AES/DAC, and its ADC clock is enabled from
     * APB2ENR directly (_adc_clk_enable in hal_adc.c). */
    (void)mask;
    (void)REG32(RCC_BASE);
}

/* ============================================================
 * USB 48MHz clock source selection (L4 only)
 * ============================================================ */

#if defined(STM32L422xx)

/**
 * Select the 48MHz clock source for USB.
 *   0 = HSI48 (default, crystal-less with CRS)
 *   1 = PLL48M1CLK (PLLSAI1 Q output)
 *   2 = PLL48M2CLK (PLL Q output)
 *   3 = MSI
 */
static inline void ll_rcc_set_usb_clk_source(uint32_t src)
{
    /* RCC_CCIPR at offset 0x88, CLK48SEL bits [27:26] */
    MOD_BITS(REG32(RCC_BASE + 0x88UL), 3UL << 26, (src & 3UL) << 26);
}

#define LL_RCC_USB48_HSI48      0x0UL
#define LL_RCC_USB48_PLLSAI1Q   0x1UL
#define LL_RCC_USB48_PLLQ       0x2UL
#define LL_RCC_USB48_MSI        0x3UL

#endif /* STM32L422xx */

#if defined(STM32H523xx)

/**
 * Select the USB kernel clock source (H5).
 * RCC_CCIPR4 at offset 0xE4, USBSEL bits [5:4].
 *   0 = Disable
 *   1 = PLL1Q
 *   2 = PLL3Q (not available on H523)
 *   3 = HSI48
 */
static inline void ll_rcc_set_usb_clk_source(uint32_t src)
{
    MOD_BITS(REG32(RCC_BASE + 0xE4UL), 3UL << 4, (src & 3UL) << 4);
}

#define LL_RCC_USB_DISABLE      0x0UL
#define LL_RCC_USB_PLL1Q        0x1UL
#define LL_RCC_USB_HSI48        0x3UL

#endif /* STM32H523xx */

/* ============================================================
 * Peripheral clock enable bit masks (per family)
 * ============================================================ */

#if defined(STM32L011xx)
  /* APB1 */
  #define LL_APB1_TIM2      (1UL << 0)
  #define LL_APB1_USART2    (1UL << 17)
  #define LL_APB1_I2C1      (1UL << 21)
  #define LL_APB1_LPUART1   (1UL << 18)
  #define LL_APB1_LPTIM1    (1UL << 31)
  /* APB2 */
  #define LL_APB2_TIM21     (1UL << 2)
  #define LL_APB2_SPI1      (1UL << 12)

#elif defined(STM32L422xx)
  /* APB1 */
  #define LL_APB1_TIM2      (1UL << 0)
  #define LL_APB1_USART2    (1UL << 17)
  #define LL_APB1_I2C1      (1UL << 21)
  #define LL_APB1_I2C3      (1UL << 23)
  #define LL_APB1_USB       (1UL << 26)
  /* APB2 */
  #define LL_APB2_TIM1      (1UL << 11)
  #define LL_APB2_SPI1      (1UL << 12)
  #define LL_APB2_USART1    (1UL << 14)
  #define LL_APB2_TIM15     (1UL << 16)
  #define LL_APB2_TIM16     (1UL << 17)
  /* AHB2 */
  #define LL_AHB2_ADC       (1UL << 13)

#elif defined(STM32WBA55xx)
  /* APB1 */
  #define LL_APB1_TIM2      (1UL << 0)
  #define LL_APB1_TIM3      (1UL << 1)
  #define LL_APB1_USART2    (1UL << 17)
  #define LL_APB1_I2C1      (1UL << 21)
  /* APB2 */
  #define LL_APB2_TIM1      (1UL << 11)
  #define LL_APB2_SPI1      (1UL << 12)
  #define LL_APB2_USART1    (1UL << 14)
  #define LL_APB2_TIM16     (1UL << 17)
  #define LL_APB2_TIM17     (1UL << 18)
  /* APB7 (WBA-specific, register offset 0xA8) */
  #define LL_APB7_SYSCFG    (1UL << 1)   /* SYSCFGEN */
  #define LL_APB7_SPI3      (1UL << 5)   /* SPI3EN */
  #define LL_APB7_LPUART1   (1UL << 6)   /* LPUART1EN */
  #define LL_APB7_I2C3      (1UL << 7)   /* I2C3EN */
  #define LL_APB7_LPTIM1    (1UL << 11)  /* LPTIM1EN */
  /* AHB4/AHB5 defined above with WBA55 functions */

#elif defined(STM32H523xx)
  /* APB1 (RCC_APB1LENR, offset 0x9C) */
  #define LL_APB1_TIM2      (1UL << 0)
  #define LL_APB1_TIM3      (1UL << 1)
  #define LL_APB1_TIM6      (1UL << 4)
  #define LL_APB1_TIM7      (1UL << 5)
  #define LL_APB1_USART2    (1UL << 17)
  #define LL_APB1_USART3    (1UL << 18)
  #define LL_APB1_I2C1      (1UL << 21)
  #define LL_APB1_I2C2      (1UL << 22)
  #define LL_APB1_CRS       (1UL << 24)
  /* APB2 (RCC_APB2ENR, offset 0xA4) */
  #define LL_APB2_TIM1      (1UL << 11)
  #define LL_APB2_SPI1      (1UL << 12)
  #define LL_APB2_USART1    (1UL << 14)
  #define LL_APB2_USB       (1UL << 24)
  /* APB3 */
  #define LL_APB3_LPUART1   (1UL << 6)
  #define LL_APB3_I2C3      (1UL << 23)
  #define LL_APB3_SPI3      (1UL << 5)
  /* AHB2 */
  #define LL_AHB2_ADC       (1UL << 10)
#endif

/* ============================================================
 * Voltage Output Scaling (VOS) — WBA55 only
 * ============================================================ */

#if defined(STM32WBA55xx)

/* PWR_BASE defined above with radio LL functions */

/**
 * Set voltage scaling range.  Must be called BEFORE increasing SYSCLK
 * above Range 2 limits (16MHz).
 *
 *   range: 1 = Range 1 / high performance (up to 100MHz)
 *          2 = Range 2 / low power (up to 16MHz, default after reset)
 *
 * PWR clock must be enabled (ll_rcc_ahb4_clk_enable(LL_AHB4_PWR)) before
 * calling this function.
 *
 * PWR_VOSR (offset 0x0C) for WBA55:
 *   bit  [16]   VOS:    0 = Range 2 (up to 16MHz),  1 = Range 1 (up to 100MHz)
 *   bit  [15]   VOSRDY: 1 when voltage scaling output is ready
 * (PWR_VOSR_VOS_Pos=16, PWR_VOSR_VOSRDY_Pos=15 per stm32wba55xx.h CMSIS header)
 */
static inline void ll_pwr_set_vos(uint32_t range)
{
    if (range == 1) {
        SET_BITS(REG32(PWR_BASE + 0x0CUL), (1UL << 16));   /* VOS = 1 → Range 1 */
    } else {
        CLR_BITS(REG32(PWR_BASE + 0x0CUL), (1UL << 16));   /* VOS = 0 → Range 2 */
    }
    /* Wait for VOSRDY (bit 15) */
    uint32_t timeout = 100000;
    while (!(REG32(PWR_BASE + 0x0CUL) & (1UL << 15)) && --timeout)
        ;
}

/** Current voltage range: 1 or 2 (PWR_VOSR.VOS). Needs the PWR clock. */
static inline uint32_t ll_pwr_get_vos(void)
{
    return (REG32(PWR_BASE + 0x0CUL) & (1UL << 16)) ? 1UL : 2UL;
}

/* ---- AHB5 (radio bus) clock, RM0493 §12.4.6 / §12.8.51 ----
 * hclk5 must never exceed 32 MHz (range 1) and must be <= 8 MHz with HDIV5
 * set before entering range 2. HPRE5 divides SYSCLK when it comes from the
 * PLL and must be written BEFORE the switch to the PLL (it takes effect with
 * the switch; writes are ignored, and must not be made, while SWS = PLL).
 * HDIV5 divides by 2 when SYSCLK is HSI16 or HSE32. The 2.4 GHz radio needs
 * range 1, hclk5 >= 16 MHz and HDIV5 clear. */
#define LL_RCC_CFGR4         REG32(RCC_BASE + 0x200UL)
#define LL_RCC_CFGR4_HDIV5   (1UL << 4)

/** Set HPRE5 for a PLL SYSCLK: div = 1, 2, 3, 4 or 6. */
static inline void ll_rcc_set_hpre5(uint32_t div)
{
    uint32_t v = (div >= 6) ? 0x7UL : (div == 4) ? 0x6UL : (div == 3) ? 0x5UL
               : (div == 2) ? 0x4UL : 0x0UL;
    MOD_BITS(LL_RCC_CFGR4, 0x7UL, v);
    (void)LL_RCC_CFGR4;              /* read back before any range change */
}

/** HDIV5: 1 = hclk5 is SYSCLK / 2 (HSI16/HSE32 SYSCLK), 0 = not divided. */
static inline void ll_rcc_set_hdiv5(int on)
{
    if (on) SET_BITS(LL_RCC_CFGR4, LL_RCC_CFGR4_HDIV5);
    else    CLR_BITS(LL_RCC_CFGR4, LL_RCC_CFGR4_HDIV5);
    (void)LL_RCC_CFGR4;
}

/* ---- pll1rclk SYSCLK prescaler (PLL1CFGR bits 22:20, RM0493 §12.8.7) ----
 * A SYSCLK increase may be at most 43 MHz in one step (RM0493 Table 99). The
 * switch to the PLL goes through the prescaler: select it (divided) before
 * setting SW = PLL, then release it; hardware steps up to the undivided clock
 * and sets PLL1RCLKPRERDY. This is the CubeWBA HAL_RCC_ClockConfig sequence
 * (2-step division, whatever the PLL frequency). */
#define LL_RCC_PLL1CFGR              REG32(RCC_BASE + 0x28UL)
#define LL_RCC_PLL1CFGR_RCLKPRE      (1UL << 20)
#define LL_RCC_PLL1CFGR_RCLKPRESTEP  (1UL << 21)
#define LL_RCC_PLL1CFGR_RCLKPRERDY   (1UL << 22)

/** Divide pll1rclk on its way to SYSCLK (2-step division). */
static inline void ll_rcc_pll1_rclkpre_divide(void)
{
    MOD_BITS(LL_RCC_PLL1CFGR, LL_RCC_PLL1CFGR_RCLKPRESTEP | LL_RCC_PLL1CFGR_RCLKPRE,
             LL_RCC_PLL1CFGR_RCLKPRE);
}

/** Release the pll1rclk prescaler and wait for the undivided clock. */
static inline void ll_rcc_pll1_rclkpre_release(void)
{
    CLR_BITS(LL_RCC_PLL1CFGR, LL_RCC_PLL1CFGR_RCLKPRE);
    uint32_t timeout = 100000;
    while (!(LL_RCC_PLL1CFGR & LL_RCC_PLL1CFGR_RCLKPRERDY) && --timeout)
        ;
}

#elif defined(STM32L011xx)

#include "ll_pwr.h"

/**
 * Set the L0 core voltage range (PWR_CR.VOS[12:11], RM0377 §6.1.5):
 *   1 = 1.8 V (up to 32 MHz, needs VDD >= 1.71 V), 2 = 1.5 V (up to 16 MHz,
 *   the reset value), 3 = 1.2 V (up to 4.2 MHz; no flash/EEPROM program or
 *   erase, no HSI16 as system clock, PLL VCO <= 24 MHz).
 * Raise the range before raising the clock; lower the clock before lowering
 * the range. The system clock stops until VOSF clears. Needs the PWR clock.
 */
static inline void ll_pwr_set_vos(uint32_t range)
{
    uint32_t timeout = 100000;
    while ((REG32(PWR_BASE + 0x04UL) & (1UL << 4)) && --timeout)   /* CSR.VOSF */
        ;
    MOD_BITS(REG32(PWR_BASE + 0x00UL), 0x3UL << 11, (range & 0x3UL) << 11);
    timeout = 100000;
    while ((REG32(PWR_BASE + 0x04UL) & (1UL << 4)) && --timeout)
        ;
}

/** Current L0 voltage range (1, 2 or 3). */
static inline uint32_t ll_pwr_get_vos(void)
{
    return (REG32(PWR_BASE + 0x00UL) >> 11) & 0x3UL;
}

#elif defined(STM32H523xx)

#ifndef PWR_BASE
  #define PWR_BASE  0x44020800UL
#endif

/**
 * Set voltage scaling range for H5.
 *   0 = Scale 0 (up to 250MHz, boost)
 *   1 = Scale 1 (up to 150MHz)
 *   2 = Scale 2 (up to 100MHz)
 *   3 = Scale 3 (up to 32MHz, reset default)
 *
 * PWR_VOSR  at offset 0x0C: BOOSTEN(18), BOOSTRDY(14), VOSRDY(3)
 * PWR_VOSCR at offset 0x10: VOS[5:4]
 *
 * The EPOD booster must be enabled before changing VOS above Scale 3.
 */
static inline void ll_pwr_set_vos(uint32_t scale)
{
    /* PWR_VOSCR at offset 0x10: VOS[5:4]
     * PWR_VOSSR at offset 0x14: VOSRDY(3), ACTVOSRDY(13), ACTVOS[15:14] */
    MOD_BITS(REG32(PWR_BASE + 0x10UL), 0x3UL << 4, scale << 4);

    /* Wait for VOSRDY (bit 3 of PWR_VOSSR at offset 0x14) */
    uint32_t timeout = 100000;
    while (!(REG32(PWR_BASE + 0x14UL) & (1UL << 3)) && --timeout)
        ;
}

#endif /* VOS */

/* ============================================================
 * I2C kernel clock source selection — WBA55 only
 * ============================================================ */

#if defined(STM32WBA55xx)

/* I2C kernel clock source values:
 *   00 = PCLK  (APB clock, varies with SYSCLK)
 *   01 = SYSCLK
 *   10 = HSI16 (16MHz, independent of SYSCLK — preferred for stable timing)
 *   11 = reserved
 */
#define LL_RCC_I2C_CLK_PCLK     0x0UL
#define LL_RCC_I2C_CLK_SYSCLK   0x1UL
#define LL_RCC_I2C_CLK_HSI16    0x2UL

/**
 * Set the kernel clock source for an I2C peripheral.
 *
 *   i2c_num: 1 or 3
 *   source:  LL_RCC_I2C_CLK_PCLK / LL_RCC_I2C_CLK_SYSCLK / LL_RCC_I2C_CLK_HSI16
 *
 * I2C1: RCC_CCIPR1 (offset 0xE0) bits [11:10]
 * I2C3: RCC_CCIPR3 (offset 0xE8) bits [7:6]
 */
static inline void ll_rcc_set_i2c_clk_source(uint32_t i2c_num, uint32_t source)
{
    if (i2c_num == 1) {
        MOD_BITS(REG32(RCC_BASE + 0xE0UL), 0x3UL << 10, (source & 0x3UL) << 10);
    } else if (i2c_num == 3) {
        MOD_BITS(REG32(RCC_BASE + 0xE8UL), 0x3UL << 6, (source & 0x3UL) << 6);
    }
}

/**
 * SAI1 kernel clock source (RCC_CCIPR2, offset 0xE4, bits [7:5] = SAI1SEL).
 * Encoding per RM0493: 000 pll1pclk · 001 pll1qclk · 010 SYSCLK ·
 * 011 AUDIOCLK input pin · 100 HSI16.
 */
#define LL_RCC_SAI_CLK_PLL1P     0x0UL
#define LL_RCC_SAI_CLK_PLL1Q     0x1UL
#define LL_RCC_SAI_CLK_SYSCLK    0x2UL
#define LL_RCC_SAI_CLK_AUDIOCLK  0x3UL
#define LL_RCC_SAI_CLK_HSI16     0x4UL

/**
 * Set the SAI1 kernel clock source. HSI16 (16 MHz) is the simplest robust
 * choice for PDM capture — always available and independent of SYSCLK.
 *
 *   source: LL_RCC_SAI_CLK_HSI16 / _SYSCLK / _PLL1P / _PLL1Q / _AUDIOCLK
 */
static inline void ll_rcc_set_sai_clk_source(uint32_t source)
{
    MOD_BITS(REG32(RCC_BASE + 0xE4UL), 0x7UL << 5, (source & 0x7UL) << 5);
}

#endif /* STM32WBA55xx I2C clock source */

#if defined(STM32L011xx)

/* I2C1 kernel clock (RCC_CCIPR 0x4C, I2C1SEL[13:12], RM0377 §7.3.19). */
#define LL_RCC_I2C_CLK_PCLK     0x0UL
#define LL_RCC_I2C_CLK_SYSCLK   0x1UL
#define LL_RCC_I2C_CLK_HSI16    0x2UL

/** Set the I2C1 kernel clock source (i2c_num must be 1; the L011 has only I2C1). */
static inline void ll_rcc_set_i2c_clk_source(uint32_t i2c_num, uint32_t source)
{
    if (i2c_num == 1)
        MOD_BITS(REG32(RCC_BASE + 0x4CUL), 0x3UL << 12, (source & 0x3UL) << 12);
}

#endif /* STM32L011xx I2C clock source */

#endif /* LL_RCC_H */
