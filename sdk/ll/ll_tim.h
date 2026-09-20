/**
 * ll_tim.h — Low-level timer operations
 *
 * Basic counting, PWM output, and input capture.
 * The TIM register layout is largely shared across all STM32
 * families. Advanced timers (TIM1) have additional registers
 * for complementary outputs and break/dead-time, but the core
 * counting and capture/compare registers are the same.
 */

#ifndef LL_TIM_H
#define LL_TIM_H

#include "ll_common.h"

/* ---- Timer register structure ---- */

typedef struct {
    volatile uint32_t CR1;      /* 0x00: Control register 1 */
    volatile uint32_t CR2;      /* 0x04: Control register 2 */
    volatile uint32_t SMCR;     /* 0x08: Slave mode control */
    volatile uint32_t DIER;     /* 0x0C: DMA/interrupt enable */
    volatile uint32_t SR;       /* 0x10: Status register */
    volatile uint32_t EGR;      /* 0x14: Event generation */
    volatile uint32_t CCMR1;    /* 0x18: Capture/compare mode 1 */
    volatile uint32_t CCMR2;    /* 0x1C: Capture/compare mode 2 */
    volatile uint32_t CCER;     /* 0x20: Capture/compare enable */
    volatile uint32_t CNT;      /* 0x24: Counter */
    volatile uint32_t PSC;      /* 0x28: Prescaler */
    volatile uint32_t ARR;      /* 0x2C: Auto-reload */
    volatile uint32_t RCR;      /* 0x30: Repetition counter (advanced) */
    volatile uint32_t CCR1;     /* 0x34: Capture/compare 1 */
    volatile uint32_t CCR2;     /* 0x38: Capture/compare 2 */
    volatile uint32_t CCR3;     /* 0x3C: Capture/compare 3 */
    volatile uint32_t CCR4;     /* 0x40: Capture/compare 4 */
    volatile uint32_t BDTR;     /* 0x44: Break and dead-time (advanced) */
    volatile uint32_t DCR;      /* 0x48: DMA control */
    volatile uint32_t DMAR;     /* 0x4C: DMA address for burst */
} TIM_TypeDef;

/* ---- Instance base addresses ---- */

#if defined(STM32L011xx)
  #define TIM2      ((TIM_TypeDef *)0x40000000UL)
  #define TIM21     ((TIM_TypeDef *)0x40010800UL)

#elif defined(STM32L422xx)
  #define TIM1      ((TIM_TypeDef *)0x40012C00UL)  /* Advanced */
  #define TIM2      ((TIM_TypeDef *)0x40000000UL)  /* General purpose, 32-bit */
  #define TIM15     ((TIM_TypeDef *)0x40014000UL)  /* 2 channels */
  #define TIM16     ((TIM_TypeDef *)0x40014400UL)  /* 1 channel */

#elif defined(STM32WBA55xx)
  #define TIM1      ((TIM_TypeDef *)0x40012C00UL)
  #define TIM2      ((TIM_TypeDef *)0x40000000UL)
  #define TIM3      ((TIM_TypeDef *)0x40000400UL)
  #define TIM16     ((TIM_TypeDef *)0x40014400UL)
  #define TIM17     ((TIM_TypeDef *)0x40014800UL)

#elif defined(STM32H523xx)
  #define TIM1      ((TIM_TypeDef *)0x40012C00UL)
  #define TIM2      ((TIM_TypeDef *)0x40000000UL)
  #define TIM3      ((TIM_TypeDef *)0x40000400UL)
  #define TIM6      ((TIM_TypeDef *)0x40001000UL)
  #define TIM7      ((TIM_TypeDef *)0x40001400UL)
#endif

/* ---- CR1 bit definitions ---- */

#define LL_TIM_CR1_CEN          (1UL << 0)    /* Counter enable */
#define LL_TIM_CR1_UDIS         (1UL << 1)    /* Update disable */
#define LL_TIM_CR1_URS          (1UL << 2)    /* Update request source */
#define LL_TIM_CR1_OPM          (1UL << 3)    /* One-pulse mode */
#define LL_TIM_CR1_DIR          (1UL << 4)    /* Direction: 0=up, 1=down */
#define LL_TIM_CR1_ARPE         (1UL << 7)    /* Auto-reload preload enable */

/* ---- DIER bit definitions ---- */

#define LL_TIM_DIER_UIE         (1UL << 0)    /* Update interrupt enable */
#define LL_TIM_DIER_CC1IE       (1UL << 1)    /* CC1 interrupt enable */
#define LL_TIM_DIER_CC2IE       (1UL << 2)
#define LL_TIM_DIER_CC3IE       (1UL << 3)
#define LL_TIM_DIER_CC4IE       (1UL << 4)

/* ---- SR bit definitions ---- */

#define LL_TIM_SR_UIF           (1UL << 0)    /* Update interrupt flag */
#define LL_TIM_SR_CC1IF         (1UL << 1)    /* CC1 interrupt flag */
#define LL_TIM_SR_CC2IF         (1UL << 2)
#define LL_TIM_SR_CC3IF         (1UL << 3)
#define LL_TIM_SR_CC4IF         (1UL << 4)
#define LL_TIM_SR_CC1OF         (1UL << 9)    /* CC1 overcapture flag */

/* ---- EGR bit definitions ---- */

#define LL_TIM_EGR_UG           (1UL << 0)    /* Update generation */

/* ---- CCMR output mode bits ---- */
/* These repeat per channel pair (CCMR1: CH1+CH2, CCMR2: CH3+CH4)
   CH1/CH3 are in bits [6:4], CH2/CH4 are in bits [14:12] */

#define LL_TIM_OCMODE_FROZEN    0x0UL
#define LL_TIM_OCMODE_ACTIVE    0x1UL
#define LL_TIM_OCMODE_INACTIVE  0x2UL
#define LL_TIM_OCMODE_TOGGLE    0x3UL
#define LL_TIM_OCMODE_PWM1      0x6UL   /* Active while CNT < CCR */
#define LL_TIM_OCMODE_PWM2      0x7UL   /* Active while CNT > CCR */

/* ---- CCER bit definitions ---- */

#define LL_TIM_CCER_CC1E        (1UL << 0)    /* CC1 output enable */
#define LL_TIM_CCER_CC1P        (1UL << 1)    /* CC1 polarity: 1=active low */
#define LL_TIM_CCER_CC1NE       (1UL << 2)    /* CC1N output enable (advanced) */
#define LL_TIM_CCER_CC2E        (1UL << 4)
#define LL_TIM_CCER_CC2P        (1UL << 5)
#define LL_TIM_CCER_CC3E        (1UL << 8)
#define LL_TIM_CCER_CC3P        (1UL << 9)
#define LL_TIM_CCER_CC4E        (1UL << 12)
#define LL_TIM_CCER_CC4P        (1UL << 13)

/* ---- BDTR (advanced timers: TIM1) ---- */

#define LL_TIM_BDTR_MOE         (1UL << 15)   /* Main output enable */

/* ============================================================
 * Basic counting
 * ============================================================ */

/**
 * Configure basic timebase.
 *   tim:       timer instance
 *   prescaler: clock divider (0 = no division, N = divide by N+1)
 *   period:    auto-reload value (counter counts 0 → period)
 *
 * Timer frequency = pclk / (prescaler + 1)
 * Overflow rate   = pclk / (prescaler + 1) / (period + 1)
 *
 * Example: 80MHz pclk, 1kHz overflow:
 *   prescaler = 79 (→ 1MHz tick), period = 999 (→ 1kHz)
 */
/* ---- Master mode (CR2 MMS): what the timer emits on TRGO ---- */

#define LL_TIM_MMS_SHIFT     4
#define LL_TIM_MMS_RESET     0x0UL   /* EGR.UG */
#define LL_TIM_MMS_ENABLE    0x1UL   /* counter enable */
#define LL_TIM_MMS_UPDATE    0x2UL   /* update event — the usual ADC pacer */
#define LL_TIM_MMS_CMP_PULSE 0x3UL
#define LL_TIM_MMS_OC1REF    0x4UL
#define LL_TIM_MMS_OC2REF    0x5UL
#define LL_TIM_MMS_OC3REF    0x6UL
#define LL_TIM_MMS_OC4REF    0x7UL

/**
 * Route an internal timer event to TRGO, where another peripheral can
 * consume it. LL_TIM_MMS_UPDATE turns the timer into a periodic pacer:
 * every overflow emits a trigger, with no CPU involvement and no jitter
 * from interrupt latency.
 *
 * No pin is involved and no channel is consumed, which is what makes
 * this affordable on a part with every pad already committed.
 */
static inline void ll_tim_set_trgo(TIM_TypeDef *tim, uint32_t mms)
{
    MOD_BITS(tim->CR2, 0x7UL << LL_TIM_MMS_SHIFT, mms << LL_TIM_MMS_SHIFT);
}

static inline void ll_tim_config(TIM_TypeDef *tim, uint32_t prescaler, uint32_t period)
{
    tim->CR1 = 0;
    tim->PSC = prescaler;
    tim->ARR = period;
    tim->EGR = LL_TIM_EGR_UG;    /* Force update to load PSC and ARR */
    tim->SR  = 0;                 /* Clear update flag generated by UG */
}

/** Start the timer counter */
static inline void ll_tim_start(TIM_TypeDef *tim)
{
    SET_BITS(tim->CR1, LL_TIM_CR1_CEN);
}

/** Stop the timer counter */
static inline void ll_tim_stop(TIM_TypeDef *tim)
{
    CLR_BITS(tim->CR1, LL_TIM_CR1_CEN);
}

/** Read the current counter value */
static inline uint32_t ll_tim_get_counter(TIM_TypeDef *tim)
{
    return tim->CNT;
}

/** Set the counter value */
static inline void ll_tim_set_counter(TIM_TypeDef *tim, uint32_t value)
{
    tim->CNT = value;
}

/** Check and clear the update (overflow) flag */
static inline int ll_tim_update_flag(TIM_TypeDef *tim)
{
    if (tim->SR & LL_TIM_SR_UIF) {
        tim->SR = ~LL_TIM_SR_UIF;  /* Write 0 to clear (rc_w0) */
        return 1;
    }
    return 0;
}

/* ============================================================
 * PWM output
 * ============================================================ */

/**
 * Configure a channel for PWM output (mode 1: active while CNT < CCR).
 *   tim:     timer instance
 *   channel: 1, 2, 3, or 4
 *   duty:    compare value (0 = always off, ARR = always on)
 *
 * For advanced timers (TIM1), you must also call ll_tim_enable_moe().
 */
static inline void ll_tim_pwm_config(TIM_TypeDef *tim, uint32_t channel, uint32_t duty)
{
    /* Enable preload on ARR */
    SET_BITS(tim->CR1, LL_TIM_CR1_ARPE);

    switch (channel) {
    case 1:
        MOD_BITS(tim->CCMR1, 0x7UL << 4, LL_TIM_OCMODE_PWM1 << 4);
        SET_BITS(tim->CCMR1, 1UL << 3);    /* OC1PE: preload enable */
        tim->CCR1 = duty;
        SET_BITS(tim->CCER, LL_TIM_CCER_CC1E);
        break;
    case 2:
        MOD_BITS(tim->CCMR1, 0x7UL << 12, LL_TIM_OCMODE_PWM1 << 12);
        SET_BITS(tim->CCMR1, 1UL << 11);   /* OC2PE */
        tim->CCR2 = duty;
        SET_BITS(tim->CCER, LL_TIM_CCER_CC2E);
        break;
    case 3:
        MOD_BITS(tim->CCMR2, 0x7UL << 4, LL_TIM_OCMODE_PWM1 << 4);
        SET_BITS(tim->CCMR2, 1UL << 3);    /* OC3PE */
        tim->CCR3 = duty;
        SET_BITS(tim->CCER, LL_TIM_CCER_CC3E);
        break;
    case 4:
        MOD_BITS(tim->CCMR2, 0x7UL << 12, LL_TIM_OCMODE_PWM1 << 12);
        SET_BITS(tim->CCMR2, 1UL << 11);   /* OC4PE */
        tim->CCR4 = duty;
        SET_BITS(tim->CCER, LL_TIM_CCER_CC4E);
        break;
    }
}

/** Set the duty cycle (compare value) for a channel */
static inline void ll_tim_set_duty(TIM_TypeDef *tim, uint32_t channel, uint32_t duty)
{
    switch (channel) {
    case 1: tim->CCR1 = duty; break;
    case 2: tim->CCR2 = duty; break;
    case 3: tim->CCR3 = duty; break;
    case 4: tim->CCR4 = duty; break;
    }
}

/** Set PWM duty as a percentage (0-100) of the current ARR period */
static inline void ll_tim_set_duty_pct(TIM_TypeDef *tim, uint32_t channel, uint32_t pct)
{
    uint32_t duty = (tim->ARR * pct) / 100;
    ll_tim_set_duty(tim, channel, duty);
}

/**
 * Enable the main output enable (MOE) bit on advanced timers (TIM1).
 * Required for TIM1 PWM outputs to actually drive the pins.
 */
static inline void ll_tim_enable_moe(TIM_TypeDef *tim)
{
    SET_BITS(tim->BDTR, LL_TIM_BDTR_MOE);
}

/* ============================================================
 * Input capture (basic)
 * ============================================================ */

/* CCMR input capture mode: CC1S/CC2S field values */
#define LL_TIM_IC_DIRECT        0x1UL   /* IC mapped to TI1/TI2 directly */
#define LL_TIM_IC_INDIRECT      0x2UL   /* IC mapped to TI2/TI1 (crossover) */

/* Input capture prescaler (ICPSC) */
#define LL_TIM_IC_PSC_NONE      0x0UL
#define LL_TIM_IC_PSC_2         0x1UL
#define LL_TIM_IC_PSC_4         0x2UL
#define LL_TIM_IC_PSC_8         0x3UL

/**
 * Configure a channel for input capture on rising edge.
 *   tim:     timer instance
 *   channel: 1, 2, 3, or 4
 */
static inline void ll_tim_ic_config(TIM_TypeDef *tim, uint32_t channel)
{
    switch (channel) {
    case 1:
        MOD_BITS(tim->CCMR1, 0xFFUL, LL_TIM_IC_DIRECT);  /* CC1S = direct */
        CLR_BITS(tim->CCER, LL_TIM_CCER_CC1P);            /* Rising edge */
        SET_BITS(tim->CCER, LL_TIM_CCER_CC1E);            /* Enable capture */
        break;
    case 2:
        MOD_BITS(tim->CCMR1, 0xFFUL << 8, LL_TIM_IC_DIRECT << 8);
        CLR_BITS(tim->CCER, LL_TIM_CCER_CC2P);
        SET_BITS(tim->CCER, LL_TIM_CCER_CC2E);
        break;
    case 3:
        MOD_BITS(tim->CCMR2, 0xFFUL, LL_TIM_IC_DIRECT);
        CLR_BITS(tim->CCER, LL_TIM_CCER_CC3P);
        SET_BITS(tim->CCER, LL_TIM_CCER_CC3E);
        break;
    case 4:
        MOD_BITS(tim->CCMR2, 0xFFUL << 8, LL_TIM_IC_DIRECT << 8);
        CLR_BITS(tim->CCER, LL_TIM_CCER_CC4P);
        SET_BITS(tim->CCER, LL_TIM_CCER_CC4E);
        break;
    }
}

/** Read the captured value for a channel */
static inline uint32_t ll_tim_ic_read(TIM_TypeDef *tim, uint32_t channel)
{
    switch (channel) {
    case 1: return tim->CCR1;
    case 2: return tim->CCR2;
    case 3: return tim->CCR3;
    case 4: return tim->CCR4;
    default: return 0;
    }
}

/** Check and clear the capture/compare flag for a channel */
static inline int ll_tim_cc_flag(TIM_TypeDef *tim, uint32_t channel)
{
    uint32_t flag = (1UL << channel);  /* CC1IF=bit1, CC2IF=bit2, ... */
    if (tim->SR & flag) {
        tim->SR = ~flag;
        return 1;
    }
    return 0;
}

#endif /* LL_TIM_H */
