/**
 * hal_common.h — HAL shared types and utilities
 *
 * Status codes, callback types, ring buffer, IRQ number definitions.
 * Used by all HAL drivers.
 */

#ifndef HAL_COMMON_H
#define HAL_COMMON_H

#include <stdint.h>
#include <stddef.h>

/* ============================================================
 * Status codes
 * ============================================================ */

typedef enum {
    HAL_OK       =  0,
    HAL_ERROR    = -1,
    HAL_BUSY     = -2,
    HAL_TIMEOUT  = -3,
    HAL_NACK     = -4,
} hal_status_t;

/* ============================================================
 * Callback type
 * ============================================================ */

/** Generic completion callback */
typedef void (*hal_callback_t)(void *ctx);

/* ============================================================
 * Ring buffer (lock-free, single producer / single consumer)
 *
 * Used by UART interrupt RX. Size must be a power of 2.
 * ============================================================ */

typedef struct {
    uint8_t  *buf;
    uint16_t  size;             /* Must be power of 2 */
    volatile uint16_t head;     /* Written by producer (ISR) */
    volatile uint16_t tail;     /* Read by consumer (main) */
} hal_ringbuf_t;

static inline void hal_ringbuf_init(hal_ringbuf_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buf  = buf;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

static inline int hal_ringbuf_empty(const hal_ringbuf_t *rb)
{
    return rb->head == rb->tail;
}

static inline int hal_ringbuf_full(const hal_ringbuf_t *rb)
{
    return ((rb->head + 1) & (rb->size - 1)) == rb->tail;
}

static inline uint16_t hal_ringbuf_count(const hal_ringbuf_t *rb)
{
    return (rb->head - rb->tail) & (rb->size - 1);
}

/** Put a byte into the ring buffer. Returns 1 on success, 0 if full. */
static inline int hal_ringbuf_put(hal_ringbuf_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) & (rb->size - 1);
    if (next == rb->tail)
        return 0;  /* Full */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 1;
}

/** Get a byte from the ring buffer. Returns 1 on success, 0 if empty. */
static inline int hal_ringbuf_get(hal_ringbuf_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail)
        return 0;  /* Empty */
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) & (rb->size - 1);
    return 1;
}

/** Drain up to max_len bytes into buf. Returns number of bytes read. */
static inline uint16_t hal_ringbuf_read(hal_ringbuf_t *rb, uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && !hal_ringbuf_empty(rb)) {
        hal_ringbuf_get(rb, &buf[count++]);
    }
    return count;
}

/* ============================================================
 * NVIC helpers (ARM Cortex-M)
 * ============================================================ */

static inline void hal_nvic_enable_irq(uint32_t irqn)
{
    ((volatile uint32_t *)0xE000E100UL)[irqn >> 5] = 1UL << (irqn & 0x1F);
}

static inline void hal_nvic_disable_irq(uint32_t irqn)
{
    ((volatile uint32_t *)0xE000E180UL)[irqn >> 5] = 1UL << (irqn & 0x1F);
}

static inline void hal_nvic_set_priority(uint32_t irqn, uint8_t priority)
{
#if defined(__ARM_ARCH_6M__)
    /* ARMv6-M (Cortex-M0+, i.e. Core.ST.L0): NVIC_IPR is word-accessible
     * only, so the byte write used below is architecturally undefined and
     * can be dropped silently. Read-modify-write the containing word,
     * which is what CMSIS does for __CORTEX_M < 3. */
    volatile uint32_t *ipr = &((volatile uint32_t *)0xE000E400UL)[irqn >> 2];
    uint32_t shift = (irqn & 3UL) * 8UL;
    uint32_t val   = ((uint32_t)priority << 4) & 0xFFUL;
    *ipr = (*ipr & ~(0xFFUL << shift)) | (val << shift);
#else
    ((volatile uint8_t *)0xE000E400UL)[irqn] = (uint8_t)(priority << 4);
#endif
}

static inline void hal_nvic_clear_pending(uint32_t irqn)
{
    ((volatile uint32_t *)0xE000E280UL)[irqn >> 5] = 1UL << (irqn & 0x1F);
}

/* ============================================================
 * Per-family IRQ numbers for peripherals used by HAL
 * ============================================================ */

#if defined(STM32L011xx)
  /* L0: USART2 only, no USART1 */
  #define HAL_IRQ_USART2        28
  #define HAL_IRQ_LPUART1       29
  #define HAL_IRQ_I2C1_EV       23
  #define HAL_IRQ_SPI1          25
  #define HAL_IRQ_TIM2          15
  #define HAL_IRQ_TIM21         20
  #define HAL_IRQ_ADC1          12
  #define HAL_IRQ_DMA1_CH1      9
  #define HAL_IRQ_DMA1_CH2_3    10
  #define HAL_IRQ_DMA1_CH4_7    11

#elif defined(STM32L422xx)
  #define HAL_IRQ_USART1        37
  #define HAL_IRQ_USART2        38
  #define HAL_IRQ_LPUART1       70
  #define HAL_IRQ_I2C1_EV       31
  #define HAL_IRQ_I2C1_ER       32
  #define HAL_IRQ_I2C3_EV       72
  #define HAL_IRQ_I2C3_ER       73
  #define HAL_IRQ_SPI1          35
  #define HAL_IRQ_TIM1_UP_16    25
  #define HAL_IRQ_TIM1_CC       27
  #define HAL_IRQ_TIM2          28
  #define HAL_IRQ_TIM15         24
  #define HAL_IRQ_ADC1          18
  #define HAL_IRQ_DMA1_CH1      11
  #define HAL_IRQ_DMA1_CH2      12
  #define HAL_IRQ_DMA1_CH3      13
  #define HAL_IRQ_DMA1_CH4      14
  #define HAL_IRQ_DMA1_CH5      15
  #define HAL_IRQ_DMA1_CH6      16
  #define HAL_IRQ_DMA1_CH7      17
  #define HAL_IRQ_USB            67

#elif defined(STM32WBA55xx)
  #define HAL_IRQ_USART1        46
  #define HAL_IRQ_USART2        47
  #define HAL_IRQ_LPUART1       48
  #define HAL_IRQ_I2C1_EV       43
  #define HAL_IRQ_I2C1_ER       44
  #define HAL_IRQ_I2C3_EV       54
  #define HAL_IRQ_I2C3_ER       55
  #define HAL_IRQ_SPI1          45
  #define HAL_IRQ_SPI3          63
  #define HAL_IRQ_TIM1_BRK      37
  #define HAL_IRQ_TIM1_UP       38
  #define HAL_IRQ_TIM1_TRG_COM  39
  #define HAL_IRQ_TIM1_CC       40
  #define HAL_IRQ_TIM2          41
  #define HAL_IRQ_TIM3          42
  #define HAL_IRQ_TIM16         51
  #define HAL_IRQ_TIM17         52
  #define HAL_IRQ_ADC4          65
  #define HAL_IRQ_GPDMA1_CH0    29
  #define HAL_IRQ_GPDMA1_CH1    30
  #define HAL_IRQ_GPDMA1_CH2    31
  #define HAL_IRQ_GPDMA1_CH3    32
  #define HAL_IRQ_GPDMA1_CH4    33
  #define HAL_IRQ_GPDMA1_CH5    34
  #define HAL_IRQ_GPDMA1_CH6    35
  #define HAL_IRQ_GPDMA1_CH7    36
  #define HAL_IRQ_LPTIM1        49
  #define HAL_IRQ_LPTIM2        50
  #define HAL_IRQ_RADIO         66
  #define HAL_IRQ_HASH          61

#elif defined(STM32H523xx)
  #define HAL_IRQ_USART1        58
  #define HAL_IRQ_USART2        59
  #define HAL_IRQ_USART3        60
  #define HAL_IRQ_LPUART1       63
  #define HAL_IRQ_I2C1_EV       55
  #define HAL_IRQ_I2C1_ER       56
  #define HAL_IRQ_I2C2_EV       57
  #define HAL_IRQ_I2C2_ER       58
  #define HAL_IRQ_SPI1          51
  #define HAL_IRQ_SPI2          52
  #define HAL_IRQ_SPI3          53
  #define HAL_IRQ_TIM1_UP       41
  #define HAL_IRQ_TIM1_CC       44
  #define HAL_IRQ_TIM2          45
  #define HAL_IRQ_TIM3          46
  /* Note: TIM6 and TIM7 have NO dedicated NVIC interrupt on the STM32H523.
     Position 54 is reserved and 55 is I2C1_EV.  These basic timers can still
     be used for DAC triggering and timebase, but cannot generate tick callbacks. */
  #define HAL_IRQ_ADC1          37
  #define HAL_IRQ_GPDMA1_CH0    29
  #define HAL_IRQ_GPDMA1_CH1    30
  #define HAL_IRQ_GPDMA1_CH2    31
  #define HAL_IRQ_GPDMA1_CH3    32
  #define HAL_IRQ_GPDMA1_CH4    33
  #define HAL_IRQ_GPDMA1_CH5    34
  #define HAL_IRQ_GPDMA1_CH6    35
  #define HAL_IRQ_GPDMA1_CH7    36
  #define HAL_IRQ_USB            74    /* USB_DRD_FS — RM0492 Table, vector 0x0168 */
#endif

/* ============================================================
 * Utility: simple timeout helper using SysTick
 * ============================================================ */

extern volatile uint32_t _systick_ticks;

/** Milliseconds since boot, from the SysTick counter (requires ll_systick_init). */
static inline uint32_t hal_tick(void)
{
    return _systick_ticks;
}

/** Wrap-safe check that timeout_ms has elapsed since the tick value start. */
static inline int hal_timeout_expired(uint32_t start, uint32_t timeout_ms)
{
    return (hal_tick() - start) >= timeout_ms;
}

#endif /* HAL_COMMON_H */
