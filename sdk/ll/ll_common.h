/**
 * ll_common.h — Low-level common definitions
 *
 * Register access helpers, bit manipulation, and GPIO/peripheral
 * type definitions shared across all STM32 families.
 */

#ifndef LL_COMMON_H
#define LL_COMMON_H

#include <stdint.h>

/* ---- Register access ---- */

#define REG32(addr)         (*(volatile uint32_t *)(addr))
#define SET_BITS(reg, mask) ((reg) |= (mask))
#define CLR_BITS(reg, mask) ((reg) &= ~(mask))
#define MOD_BITS(reg, mask, val) ((reg) = ((reg) & ~(mask)) | (val))

/* ---- Peripheral base addresses ---- */

#define PERIPH_BASE         0x40000000UL

#if defined(STM32L011xx)
  #define AHB_BASE          (PERIPH_BASE + 0x00020000UL)
  #define IOPORT_BASE       (PERIPH_BASE + 0x10000000UL)
#elif defined(STM32L422xx)
  #define AHB1_BASE         (PERIPH_BASE + 0x00020000UL)
  #define AHB2_BASE         (PERIPH_BASE + 0x08000000UL)
  #define IOPORT_BASE       AHB2_BASE
#elif defined(STM32WBA55xx)
  #define AHB1_BASE         (PERIPH_BASE + 0x00020000UL)
  #define AHB2_BASE         (PERIPH_BASE + 0x02020000UL)
  #define AHB4_BASE         (PERIPH_BASE + 0x06020000UL)
  #define AHB5_BASE         (PERIPH_BASE + 0x06020000UL)  /* Same as AHB4 on WBA55 */
  #define IOPORT_BASE       (PERIPH_BASE + 0x02020000UL)
#elif defined(STM32H523xx)
  #define AHB1_BASE         (PERIPH_BASE + 0x04020000UL)
  #define AHB2_BASE         (PERIPH_BASE + 0x08020000UL)
  #define IOPORT_BASE       (PERIPH_BASE + 0x02020000UL)
#else
  #error "Unknown MCU — define one of: STM32L011xx, STM32L422xx, STM32WBA55xx, STM32H523xx"
#endif

/* ---- LSI nominal frequency (Hz) ----
 * The LSI clocks the IWDG, and the RTC whenever the RTC runs from LSI, so the
 * watchdog prescaler and the RTC 1 Hz prescalers are both derived from this.
 * It is an RC oscillator: these are typical values, not calibrated ones.
 *   L0 (STM32L011): 37 kHz (RM0377 LSI section; DS table "LSI oscillator
 *                   characteristics": 26 / 38 / 56 kHz min / typ / max).
 *                   The old 32 kHz assumption made a 5 s watchdog fire at
 *                   ~4.3 s (as early as ~2.9 s on a fast part).
 *   L4 (STM32L422): 32 kHz (DS: 29.5-34 kHz over temperature).
 *   WBA55 (LSI1):   32 kHz (DS: 30.4-33.6 kHz, LSI1PREDIV = 0).
 *   H5:             32 kHz (unchanged).
 * No calibration against HSI16 is done; the RTC and IWDG share the LSI, so
 * durations measured in RTC ticks against the watchdog stay consistent even
 * when the absolute frequency is off. */
#if defined(STM32L011xx)
  #define LL_LSI_HZ         37000UL
#else
  #define LL_LSI_HZ         32000UL
#endif

/* ---- GPIO register structure ---- */
/* Common across all STM32 families */

typedef struct {
    volatile uint32_t MODER;       /* 0x00: Mode register */
    volatile uint32_t OTYPER;      /* 0x04: Output type register */
    volatile uint32_t OSPEEDR;     /* 0x08: Output speed register */
    volatile uint32_t PUPDR;       /* 0x0C: Pull-up/pull-down register */
    volatile uint32_t IDR;         /* 0x10: Input data register */
    volatile uint32_t ODR;         /* 0x14: Output data register */
    volatile uint32_t BSRR;        /* 0x18: Bit set/reset register */
    volatile uint32_t LCKR;        /* 0x1C: Lock register */
    volatile uint32_t AFR[2];      /* 0x20-0x24: Alternate function low/high */
    volatile uint32_t BRR;         /* 0x28: Bit reset register */
} GPIO_TypeDef;

/* GPIO port instances */
#if defined(STM32L011xx)
  #define GPIOA  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0000UL))
  #define GPIOB  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0400UL))
  #define GPIOC  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0800UL))
#elif defined(STM32L422xx)
  #define GPIOA  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0000UL))
  #define GPIOB  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0400UL))
  #define GPIOC  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0800UL))
  #define GPIOH  ((GPIO_TypeDef *)(IOPORT_BASE + 0x1C00UL))
#elif defined(STM32WBA55xx)
  #define GPIOA  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0000UL))
  #define GPIOB  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0400UL))
  #define GPIOC  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0800UL))
  #define GPIOH  ((GPIO_TypeDef *)(IOPORT_BASE + 0x1C00UL))
#elif defined(STM32H523xx)
  #define GPIOA  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0000UL))
  #define GPIOB  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0400UL))
  #define GPIOC  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0800UL))
  #define GPIOD  ((GPIO_TypeDef *)(IOPORT_BASE + 0x0C00UL))
  #define GPIOH  ((GPIO_TypeDef *)(IOPORT_BASE + 0x1C00UL))
#endif

/* GPIO mode values (2 bits per pin in MODER) */
#define LL_GPIO_MODE_INPUT      0x0UL
#define LL_GPIO_MODE_OUTPUT     0x1UL
#define LL_GPIO_MODE_AF         0x2UL
#define LL_GPIO_MODE_ANALOG     0x3UL

/* GPIO output type (1 bit per pin in OTYPER) */
#define LL_GPIO_OTYPE_PP        0x0UL   /* Push-pull */
#define LL_GPIO_OTYPE_OD        0x1UL   /* Open-drain */

/* GPIO speed (2 bits per pin in OSPEEDR) */
#define LL_GPIO_SPEED_LOW       0x0UL
#define LL_GPIO_SPEED_MED       0x1UL
#define LL_GPIO_SPEED_HIGH      0x2UL
#define LL_GPIO_SPEED_VHIGH     0x3UL

/* GPIO pull (2 bits per pin in PUPDR) */
#define LL_GPIO_PULL_NONE       0x0UL
#define LL_GPIO_PULL_UP         0x1UL
#define LL_GPIO_PULL_DOWN       0x2UL

/* ============================================================
 * NVIC helpers (ARM Cortex-M core, not peripheral-specific)
 * ============================================================ */

#define LL_NVIC_ISER_BASE   0xE000E100UL   /* Interrupt Set Enable */
#define LL_NVIC_ICER_BASE   0xE000E180UL   /* Interrupt Clear Enable */
#define LL_NVIC_ISPR_BASE   0xE000E200UL   /* Interrupt Set Pending */
#define LL_NVIC_ICPR_BASE   0xE000E280UL   /* Interrupt Clear Pending */
#define LL_NVIC_IPR_BASE    0xE000E400UL   /* Interrupt Priority */

/* ============================================================
 * Critical sections
 * ============================================================ */

/**
 * Disable interrupts and return the previous PRIMASK.
 *
 * Save/restore rather than a bare disable/enable, so nesting one
 * critical section inside another does not re-enable interrupts early.
 *
 *   uint32_t s = ll_irq_save();
 *   ... update shared state an ISR also touches ...
 *   ll_irq_restore(s);
 *
 * Keep the body short: this blocks every interrupt, not just the one
 * you are racing with.
 */
static inline uint32_t ll_irq_save(void)
{
    uint32_t primask;
    __asm volatile ("mrs %0, primask" : "=r" (primask));
    __asm volatile ("cpsid i" ::: "memory");
    return primask;
}

/** Restore the interrupt state captured by ll_irq_save(). */
static inline void ll_irq_restore(uint32_t primask)
{
    if (!(primask & 1UL)) {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

static inline void ll_nvic_enable_irq(uint32_t irq)
{
    REG32(LL_NVIC_ISER_BASE + (irq / 32) * 4) = (1UL << (irq % 32));
}

static inline void ll_nvic_disable_irq(uint32_t irq)
{
    REG32(LL_NVIC_ICER_BASE + (irq / 32) * 4) = (1UL << (irq % 32));
}

/** Set IRQ priority (0 = highest). Upper 4 bits are implemented on Cortex-M. */
static inline void ll_nvic_set_priority(uint32_t irq, uint8_t priority)
{
    volatile uint8_t *ipr = (volatile uint8_t *)(LL_NVIC_IPR_BASE + irq);
    *ipr = priority << 4;
}

static inline void ll_nvic_clear_pending(uint32_t irq)
{
    REG32(LL_NVIC_ICPR_BASE + (irq / 32) * 4) = (1UL << (irq % 32));
}

static inline void ll_nvic_set_pending(uint32_t irq)
{
    REG32(LL_NVIC_ISPR_BASE + (irq / 32) * 4) = (1UL << (irq % 32));
}

/** Returns 1 if the IRQ is enabled in the NVIC (ISER reads back the enables). */
static inline int ll_nvic_irq_enabled(uint32_t irq)
{
    return (REG32(LL_NVIC_ISER_BASE + (irq / 32) * 4) & (1UL << (irq % 32))) != 0;
}

#endif /* LL_COMMON_H */
