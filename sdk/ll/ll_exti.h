/**
 * ll_exti.h — Low-level EXTI (External Interrupt) operations
 *
 * Configure GPIO pins as interrupt sources with rising/falling
 * edge detection. Works with NVIC for interrupt-driven GPIO.
 *
 * EXTI lines 0-15 correspond to GPIO pins 0-15. The SYSCFG
 * EXTICR registers select which port (A, B, C, ...) drives
 * each EXTI line (only one port per line at a time).
 */

#ifndef LL_EXTI_H
#define LL_EXTI_H

#include "ll_common.h"

/* ---- EXTI base address ---- */

#if defined(STM32L011xx)
  #define EXTI_BASE         0x40010400UL
#elif defined(STM32L422xx)
  #define EXTI_BASE         0x40010400UL
#elif defined(STM32WBA55xx)
  #define EXTI_BASE         0x46022000UL
#elif defined(STM32H523xx)
  #define EXTI_BASE         0x44022000UL
#endif

/* ---- SYSCFG base address (for EXTICR) ---- */

#if defined(STM32L011xx)
  #define SYSCFG_BASE       0x40010000UL
#elif defined(STM32L422xx)
  #define SYSCFG_BASE       0x40010000UL
#elif defined(STM32WBA55xx)
  #define SYSCFG_BASE       0x46000400UL  /* Actually in EXTI itself on WBA */
#elif defined(STM32H523xx)
  #define SYSCFG_BASE       0x44000400UL
#endif

/* ---- Edge detection options ---- */

#define LL_EXTI_RISING      0x01
#define LL_EXTI_FALLING     0x02
#define LL_EXTI_BOTH        0x03

/* ============================================================
 * SYSCFG clock enable
 * ============================================================ */

static inline void ll_rcc_syscfg_clk_enable(void)
{
#if defined(STM32L011xx)
    SET_BITS(REG32(RCC_BASE + 0x34UL), (1UL << 0));   /* APB2ENR: SYSCFGEN */
#elif defined(STM32L422xx)
    SET_BITS(REG32(RCC_BASE + 0x60UL), (1UL << 0));   /* APB2ENR: SYSCFGEN */
#elif defined(STM32WBA55xx)
    /* WBA: EXTICR is inside EXTI peripheral, no separate SYSCFG clock needed */
#elif defined(STM32H523xx)
    /* H5: the port select is in EXTI itself (EXTI_EXTICRx), which needs no
     * clock enable. This used to set RCC_APB1LENR bit 1, which is TIM3EN.
     * SBSEN (RCC_APB3ENR bit 1, RM0481 §11.8.32) is set for SBS users. */
    SET_BITS(REG32(RCC_BASE + 0xA8UL), (1UL << 1));
#endif
    (void)REG32(RCC_BASE);
}

/* ============================================================
 * EXTI source selection
 * ============================================================ */

/**
 * Select which GPIO port drives a given EXTI line.
 *   line:       EXTI line number (0-15, matches pin number)
 *   port_index: 0=GPIOA, 1=GPIOB, 2=GPIOC, ...
 *
 * Example: ll_exti_set_source(6, 1) → EXTI6 from GPIOB pin 6
 */
static inline void ll_exti_set_source(uint32_t line, uint32_t port_index)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    /* SYSCFG_EXTICR1..4 at SYSCFG offset 0x08, 0x0C, 0x10, 0x14
       Each register has 4 lines × 4 bits */
    uint32_t reg_offset = 0x08UL + (line / 4) * 4;
    uint32_t shift = (line % 4) * 4;
    MOD_BITS(REG32(SYSCFG_BASE + reg_offset), 0xFUL << shift, port_index << shift);

#elif defined(STM32WBA55xx)
    /* WBA: EXTI_EXTICR1..4 at EXTI offset 0x60, 0x64, 0x68, 0x6C
       Each register has 4 lines × 8 bits */
    uint32_t reg_offset = 0x60UL + (line / 4) * 4;
    uint32_t shift = (line % 4) * 8;
    MOD_BITS(REG32(EXTI_BASE + reg_offset), 0xFFUL << shift, port_index << shift);

#elif defined(STM32H523xx)
    /* H5: EXTI_EXTICR1..4 at EXTI offset 0x060-0x06C, 8 bits per line
     * (RM0481 §18.6.15-18). This used to write SBS + 0x58.., which on the
     * H523 is not a port-select register at all, so every H5 pad interrupt
     * listened to port A. */
    uint32_t reg_offset = 0x60UL + (line / 4) * 4;
    uint32_t shift = (line % 4) * 8;
    MOD_BITS(REG32(EXTI_BASE + reg_offset), 0xFFUL << shift, port_index << shift);
#endif
}

/**
 * Helper: get port index from GPIO_TypeDef pointer.
 */
static inline uint32_t ll_exti_port_index(GPIO_TypeDef *port)
{
    return ((uint32_t)port - (uint32_t)GPIOA) / 0x0400UL;
}

/* ============================================================
 * EXTI line configuration
 * ============================================================ */

/**
 * Configure an EXTI line for interrupt generation.
 *   line:  EXTI line (0-15)
 *   edge:  LL_EXTI_RISING, LL_EXTI_FALLING, or LL_EXTI_BOTH
 *
 * Prerequisites:
 *   - SYSCFG clock enabled
 *   - EXTI source selected via ll_exti_set_source()
 *   - GPIO pin configured as input
 *   - NVIC IRQ enabled after this call
 */
static inline void ll_exti_config(uint32_t line, uint32_t edge)
{
    uint32_t mask = (1UL << line);

#if defined(STM32L011xx) || defined(STM32L422xx)
    /* L0/L4: Simple register layout */
    /* IMR: interrupt mask register at offset 0x00 */
    SET_BITS(REG32(EXTI_BASE + 0x00UL), mask);

    /* RTSR: rising trigger at offset 0x08 */
    if (edge & LL_EXTI_RISING)
        SET_BITS(REG32(EXTI_BASE + 0x08UL), mask);
    else
        CLR_BITS(REG32(EXTI_BASE + 0x08UL), mask);

    /* FTSR: falling trigger at offset 0x0C */
    if (edge & LL_EXTI_FALLING)
        SET_BITS(REG32(EXTI_BASE + 0x0CUL), mask);
    else
        CLR_BITS(REG32(EXTI_BASE + 0x0CUL), mask);

#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    /* WBA/H5: Extended register layout with IMR1/RTSR1/FTSR1 */
    /* IMR1 at offset 0x80 (WBA) or 0x80 (H5) */
    SET_BITS(REG32(EXTI_BASE + 0x80UL), mask);

    /* RTSR1 at offset 0x00 */
    if (edge & LL_EXTI_RISING)
        SET_BITS(REG32(EXTI_BASE + 0x00UL), mask);
    else
        CLR_BITS(REG32(EXTI_BASE + 0x00UL), mask);

    /* FTSR1 at offset 0x04 */
    if (edge & LL_EXTI_FALLING)
        SET_BITS(REG32(EXTI_BASE + 0x04UL), mask);
    else
        CLR_BITS(REG32(EXTI_BASE + 0x04UL), mask);
#endif
}

/** Disable an EXTI line interrupt */
static inline void ll_exti_disable(uint32_t line)
{
    uint32_t mask = (1UL << line);
#if defined(STM32L011xx) || defined(STM32L422xx)
    CLR_BITS(REG32(EXTI_BASE + 0x00UL), mask);
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    CLR_BITS(REG32(EXTI_BASE + 0x80UL), mask);
#endif
}

/* ============================================================
 * Pending flag management
 * ============================================================ */

/** Check if an EXTI line has a pending interrupt */
static inline int ll_exti_pending(uint32_t line)
{
    uint32_t mask = (1UL << line);
#if defined(STM32L011xx) || defined(STM32L422xx)
    /* PR: pending register at offset 0x14 */
    return (REG32(EXTI_BASE + 0x14UL) & mask) != 0;
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    /* RPR1: rising pending at offset 0x0C, FPR1: falling at 0x10 */
    return ((REG32(EXTI_BASE + 0x0CUL) | REG32(EXTI_BASE + 0x10UL)) & mask) != 0;
#endif
}

/** Clear the pending flag for an EXTI line (write 1 to clear) */
static inline void ll_exti_clear_pending(uint32_t line)
{
    uint32_t mask = (1UL << line);
#if defined(STM32L011xx) || defined(STM32L422xx)
    REG32(EXTI_BASE + 0x14UL) = mask;
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    REG32(EXTI_BASE + 0x0CUL) = mask;  /* RPR1 */
    REG32(EXTI_BASE + 0x10UL) = mask;  /* FPR1 */
#endif
}

/* ============================================================
 * Software trigger
 * ============================================================ */

/** Generate a software interrupt on an EXTI line */
static inline void ll_exti_sw_trigger(uint32_t line)
{
#if defined(STM32L011xx) || defined(STM32L422xx)
    SET_BITS(REG32(EXTI_BASE + 0x10UL), (1UL << line));  /* SWIER */
#elif defined(STM32WBA55xx) || defined(STM32H523xx)
    SET_BITS(REG32(EXTI_BASE + 0x08UL), (1UL << line));  /* SWIER1 */
#endif
}

/* NVIC helpers are in ll_common.h (included above) */

/* ---- EXTI IRQ numbers ---- */

#if defined(STM32L011xx)
  #define EXTI0_1_IRQn      5
  #define EXTI2_3_IRQn      6
  #define EXTI4_15_IRQn     7
#elif defined(STM32L422xx)
  #define EXTI0_IRQn        6
  #define EXTI1_IRQn        7
  #define EXTI2_IRQn        8
  #define EXTI3_IRQn        9
  #define EXTI4_IRQn        10
  #define EXTI9_5_IRQn      23
  #define EXTI15_10_IRQn    40
#elif defined(STM32WBA55xx)
  #define EXTI0_IRQn        11
  #define EXTI1_IRQn        12
  #define EXTI2_IRQn        13
  #define EXTI3_IRQn        14
  #define EXTI4_IRQn        15
  #define EXTI5_IRQn        16
  #define EXTI6_IRQn        17
  #define EXTI7_IRQn        18
  #define EXTI8_IRQn        19
  #define EXTI9_IRQn        20
  #define EXTI10_IRQn       21
  #define EXTI11_IRQn       22
  #define EXTI12_IRQn       23
  #define EXTI13_IRQn       24
  #define EXTI14_IRQn       25
  #define EXTI15_IRQn       26
#elif defined(STM32H523xx)
  #define EXTI0_IRQn        11
  #define EXTI1_IRQn        12
  #define EXTI2_IRQn        13
  #define EXTI3_IRQn        14
  #define EXTI4_IRQn        15
  #define EXTI5_IRQn        16
  #define EXTI6_IRQn        17
  #define EXTI7_IRQn        18
  #define EXTI8_IRQn        19
  #define EXTI9_IRQn        20
  #define EXTI10_IRQn       21
  #define EXTI11_IRQn       22
  #define EXTI12_IRQn       23
  #define EXTI13_IRQn       24
  #define EXTI14_IRQn       25
  #define EXTI15_IRQn       26
#endif

/* ============================================================
 * Convenience: configure GPIO pin as EXTI source in one call
 * ============================================================ */

/**
 * Set up a GPIO pin as an interrupt source.
 *   port: GPIO port (GPIOA, GPIOB, ...)
 *   pin:  pin number (0-15)
 *   edge: LL_EXTI_RISING, LL_EXTI_FALLING, or LL_EXTI_BOTH
 *
 * After calling this, enable the appropriate NVIC IRQ:
 *   ll_nvic_enable_irq(EXTI0_IRQn);
 *
 * Then implement the ISR handler (e.g. EXTI0_IRQHandler).
 */
static inline void ll_exti_gpio_config(GPIO_TypeDef *port, uint32_t pin, uint32_t edge)
{
    ll_rcc_syscfg_clk_enable();
    ll_exti_set_source(pin, ll_exti_port_index(port));
    ll_exti_config(pin, edge);
}

#endif /* LL_EXTI_H */
