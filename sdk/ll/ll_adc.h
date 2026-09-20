/**
 * ll_adc.h — Low-level ADC operations
 *
 * Single-shot and continuous ADC reads. The ADC IP differs
 * across families, so this header has per-family implementations.
 *
 * Families:
 *   L0:  ADC "v3" — simple, minimal configuration
 *   L4:  ADC "v5" — with calibration, oversampling
 *   WBA: ADC4 — similar to L4 but different base/registers
 *   H5:  ADC "v5+" — similar to L4
 */

#ifndef LL_ADC_H
#define LL_ADC_H

#include "ll_common.h"

/* ---- ADC register structure ---- */
/* L0 and L4/WBA/H5 share a mostly-compatible register layout */

typedef struct {
    volatile uint32_t ISR;      /* 0x00: Interrupt and status */
    volatile uint32_t IER;      /* 0x04: Interrupt enable */
    volatile uint32_t CR;       /* 0x08: Control register */
    volatile uint32_t CFGR1;    /* 0x0C: Configuration register 1 */
    volatile uint32_t CFGR2;    /* 0x10: Configuration register 2 */
    volatile uint32_t SMPR;     /* 0x14: Sampling time (L0) / SMPR1 (L4) */
    volatile uint32_t SMPR2;    /* 0x18: Sampling time 2 (L4, not on L0) */
    volatile uint32_t _RESERVED0; /* 0x1C */
    volatile uint32_t TR;       /* 0x20: Watchdog threshold (L0) / TR1 (L4) */
    volatile uint32_t TR2;      /* 0x24: (L4 only) */
    volatile uint32_t TR3;      /* 0x28: (L4 only) */
    volatile uint32_t _RESERVED1; /* 0x2C */
    volatile uint32_t SQR1;     /* 0x30: Sequence register 1 (L4) / CHSELR (L0) */
    volatile uint32_t SQR2;     /* 0x34: (L4 only) */
    volatile uint32_t SQR3;     /* 0x38: (L4 only) */
    volatile uint32_t SQR4;     /* 0x3C: (L4 only) */
    volatile uint32_t DR;       /* 0x40: Data register */
} ADC_TypeDef;

/* ---- Instance base addresses ---- */

#if defined(STM32L011xx)
  #define ADC1      ((ADC_TypeDef *)0x40012400UL)

#elif defined(STM32L422xx)
  #define ADC1      ((ADC_TypeDef *)0x50040000UL)
  #define ADC_CCR   REG32(0x50040308UL)  /* Common control register */

#elif defined(STM32WBA55xx)
  #define ADC4      ((ADC_TypeDef *)0x46021000UL)
  #define ADC_CCR   REG32(0x46021308UL)

#elif defined(STM32H523xx)
  #define ADC1      ((ADC_TypeDef *)0x42028000UL)
  #define ADC_CCR   REG32(0x42028308UL)
#endif

/* ---- ISR bit definitions ---- */

#define LL_ADC_ISR_ADRDY        (1UL << 0)    /* ADC ready */
#define LL_ADC_ISR_EOSMP        (1UL << 1)    /* End of sampling */
#define LL_ADC_ISR_EOC          (1UL << 2)    /* End of conversion */
#define LL_ADC_ISR_EOS          (1UL << 3)    /* End of sequence */
#define LL_ADC_ISR_OVR          (1UL << 4)    /* Overrun */

/* ---- CR bit definitions ---- */

#define LL_ADC_CR_ADEN          (1UL << 0)    /* ADC enable */
#define LL_ADC_CR_ADDIS         (1UL << 1)    /* ADC disable */
#define LL_ADC_CR_ADSTART       (1UL << 2)    /* Start conversion */
#define LL_ADC_CR_ADSTP         (1UL << 4)    /* Stop conversion */
#define LL_ADC_CR_ADCAL         (1UL << 31)   /* Calibration */

/* ---- CFGR1 bit definitions ---- */

#define LL_ADC_CFGR1_CONT       (1UL << 13)   /* Continuous mode */
#define LL_ADC_CFGR1_OVRMOD     (1UL << 12)   /* Overrun management */
#define LL_ADC_CFGR1_DISCEN     (1UL << 16)   /* Discontinuous mode */

/* ---- External trigger (CFGR1) ----
 * EXTSEL selects one of the timer/EXTI trigger sources; the mapping is
 * per family, so callers pass the encoded value. EXTSEL is 3 bits wide
 * on the v3 ADC (L0, WBA ADC4) and 4 bits on L4/H5, where bit 9 is part
 * of EXTSEL rather than ALIGN. Masking the wrong width on L0 would
 * clobber ALIGN, hence the split. */
#define LL_ADC_CFGR1_EXTSEL_SHIFT  6
#if defined(STM32L011xx) || defined(STM32WBA55xx)
  #define LL_ADC_CFGR1_EXTSEL_MASK 0x7UL
#else
  #define LL_ADC_CFGR1_EXTSEL_MASK 0xFUL
#endif
#define LL_ADC_CFGR1_EXTEN_SHIFT   10
#define LL_ADC_CFGR1_EXTEN_MASK    0x3UL

/* L0 (RM0377 Table 58) trigger sources, for EXTSEL. */
#define LL_ADC_L0_TRG_TIM6_TRGO    0x0UL
#define LL_ADC_L0_TRG_TIM21_CH2    0x1UL
#define LL_ADC_L0_TRG_TIM2_TRGO    0x2UL
#define LL_ADC_L0_TRG_TIM2_CH4     0x3UL
#define LL_ADC_L0_TRG_TIM21_TRGO   0x4UL
#define LL_ADC_L0_TRG_TIM2_CH3     0x5UL
#define LL_ADC_L0_TRG_TIM3_TRGO    0x6UL
#define LL_ADC_L0_TRG_EXTI11       0x7UL

/* ---- Resolution values ---- */

#define LL_ADC_RES_12BIT        0x0UL
#define LL_ADC_RES_10BIT        0x1UL
#define LL_ADC_RES_8BIT         0x2UL
#define LL_ADC_RES_6BIT         0x3UL

/* ---- Sampling time values ---- */
/* Values vary by family but the index encoding is similar */

#define LL_ADC_SMPR_1_5         0x0UL   /* Fastest */
#define LL_ADC_SMPR_3_5         0x1UL
#define LL_ADC_SMPR_7_5         0x2UL
#define LL_ADC_SMPR_12_5        0x3UL
#define LL_ADC_SMPR_19_5        0x4UL
#define LL_ADC_SMPR_39_5        0x5UL
#define LL_ADC_SMPR_79_5        0x6UL
#define LL_ADC_SMPR_160_5       0x7UL   /* Slowest, most accurate */

/* ============================================================
 * Calibration
 * ============================================================ */

/**
 * Run ADC self-calibration. Must be called before enabling ADC.
 * ADC must be disabled (ADEN=0).
 */
static inline void ll_adc_calibrate(ADC_TypeDef *adc)
{
    /* Ensure ADC is disabled */
    CLR_BITS(adc->CR, LL_ADC_CR_ADEN);

    /* Start calibration */
    SET_BITS(adc->CR, LL_ADC_CR_ADCAL);

    /* Wait for calibration to complete */
    { uint32_t timeout = 100000;
      while ((adc->CR & LL_ADC_CR_ADCAL) && --timeout) ; }
}

/* ============================================================
 * Configuration
 * ============================================================ */

/**
 * Initialize ADC for single-shot conversion, 12-bit resolution.
 *   adc:     ADC instance
 *   smpr:    sampling time (LL_ADC_SMPR_* — applies to all channels)
 *
 * Prerequisites:
 *   - ADC peripheral clock enabled
 *   - ADC pins configured as analog via ll_gpio_config_analog()
 */
static inline void ll_adc_init(ADC_TypeDef *adc, uint32_t smpr)
{
#if defined(STM32L011xx)
    /* L0: simple ADC — CFGR2 has clock config */
    adc->CR = 0;
    adc->CFGR1 = 0;                        /* 12-bit, single conversion */
    adc->CFGR2 = 0;                        /* PCLK synchronous */
    adc->SMPR = smpr;                       /* Single SMPR register for all channels */

    /* Calibrate */
    ll_adc_calibrate(adc);

    /* Enable */
    adc->ISR = LL_ADC_ISR_ADRDY;           /* Clear ready flag */
    SET_BITS(adc->CR, LL_ADC_CR_ADEN);
    { uint32_t timeout = 100000;
      while (!(adc->ISR & LL_ADC_ISR_ADRDY) && --timeout) ; }

#elif defined(STM32L422xx) || defined(STM32H523xx)
    /* L4/H5: ADC with per-channel sampling time in SMPR1/SMPR2 */

    /* Exit deep power-down and enable the internal voltage regulator.
       The ADC starts in deep power-down after reset — DEEPPWD must be
       cleared first, then ADVREGEN set. Regulator needs ~20µs to start. */
    adc->CR = 0;                                    /* Clear DEEPPWD */
    SET_BITS(adc->CR, (1UL << 28));                 /* ADVREGEN = 1 */
    for (volatile int i = 0; i < 1000; i++) {}      /* Wait ~20µs for regulator */

    /* Set ADC clock: ADCCLK = PCLK/4 via CCR PRESC */
    MOD_BITS(ADC_CCR, 0xFUL << 18, 0x2UL << 18);  /* PRESC = 0010 → /4 */

    /* Calibrate (ADC must be disabled, regulator must be on) */
    ll_adc_calibrate(adc);

    /* Configuration: 12-bit, single conversion, overwrite on overrun */
    adc->CFGR1 = LL_ADC_CFGR1_OVRMOD;

    /* Default all channels to the chosen sampling time */
    uint32_t smpr_all = 0;
    for (int i = 0; i < 10; i++)
        smpr_all |= smpr << (i * 3);
    adc->SMPR = smpr_all;                  /* SMPR1: channels 0-9 */
    adc->SMPR2 = smpr_all;                 /* SMPR2: channels 10-18 */

    /* Enable */
    adc->ISR = LL_ADC_ISR_ADRDY;
    SET_BITS(adc->CR, LL_ADC_CR_ADEN);
    { uint32_t timeout = 100000;
      while (!(adc->ISR & LL_ADC_ISR_ADRDY) && --timeout) ; }

#elif defined(STM32WBA55xx)
    /* WBA: ADC4 uses channel-select register like L0 */
    adc->CR = 0;
    MOD_BITS(ADC_CCR, 0xFUL << 18, 0x2UL << 18);

    ll_adc_calibrate(adc);

    adc->CFGR1 = LL_ADC_CFGR1_OVRMOD;
    adc->SMPR = smpr;

    adc->ISR = LL_ADC_ISR_ADRDY;
    SET_BITS(adc->CR, LL_ADC_CR_ADEN);
    { uint32_t timeout = 100000;
      while (!(adc->ISR & LL_ADC_ISR_ADRDY) && --timeout) ; }
#endif
}

/* ============================================================
 * Single-shot conversion
 * ============================================================ */

/**
 * Read a single ADC channel (blocking).
 *   adc:     ADC instance
 *   channel: ADC channel number (0-18, depends on pin mapping)
 *
 * Returns: 12-bit ADC reading (0-4095)
 */
static inline uint16_t ll_adc_read(ADC_TypeDef *adc, uint32_t channel)
{
#if defined(STM32L011xx) || defined(STM32WBA55xx)
    /* L0/WBA: Channel select register (CHSELR at SQR1 offset) */
    adc->SQR1 = (1UL << channel);

#elif defined(STM32L422xx) || defined(STM32H523xx)
    /* L4/H5: Sequence register — single conversion in SQR1
       SQR1[3:0] = L (sequence length - 1 = 0 for single)
       SQR1[10:6] = SQ1 (first channel in sequence) */
    adc->SQR1 = (channel << 6);  /* L=0, SQ1=channel */
#endif

    /* Start conversion */
    SET_BITS(adc->CR, LL_ADC_CR_ADSTART);

    /* Wait for end of conversion */
    while (!(adc->ISR & LL_ADC_ISR_EOC))
        ;

    /* Read and return result (also clears EOC) */
    return (uint16_t)adc->DR;
}

/**
 * Read a channel and convert to millivolts.
 *   vref_mv: reference voltage in millivolts (typically 3300 for 3.3V)
 */
static inline uint32_t ll_adc_read_mv(ADC_TypeDef *adc, uint32_t channel, uint32_t vref_mv)
{
    uint16_t raw = ll_adc_read(adc, channel);
    return ((uint32_t)raw * vref_mv) / 4095;
}

/* ============================================================
 * Enable / Disable
 * ============================================================ */

static inline void ll_adc_enable(ADC_TypeDef *adc)
{
    adc->ISR = LL_ADC_ISR_ADRDY;
    SET_BITS(adc->CR, LL_ADC_CR_ADEN);
    { uint32_t timeout = 100000;
      while (!(adc->ISR & LL_ADC_ISR_ADRDY) && --timeout) ; }
}

static inline void ll_adc_disable(ADC_TypeDef *adc)
{
    SET_BITS(adc->CR, LL_ADC_CR_ADDIS);
    while (adc->CR & LL_ADC_CR_ADDIS)
        ;
}

#endif /* LL_ADC_H */
