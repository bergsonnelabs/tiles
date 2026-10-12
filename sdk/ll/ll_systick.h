/**
 * ll_systick.h — Low-level SysTick timer for delays
 *
 * Provides blocking millisecond and microsecond delays using the
 * ARM Cortex-M SysTick timer. SysTick is identical across all
 * Cortex-M cores (M0+, M4, M33).
 *
 * Call ll_systick_init() once at startup with the SYSCLK frequency.
 * The SYSCLK_HZ define from tile_config.h works perfectly here.
 */

#ifndef LL_SYSTICK_H
#define LL_SYSTICK_H

#include <stdint.h>
#include "ll_common.h"   /* REG32 — this header is unusable without it */

/* ---- SysTick registers (ARM Cortex-M, same on all cores) ---- */

#define SYSTICK_BASE        0xE000E010UL
#define SYSTICK_CSR         REG32(SYSTICK_BASE + 0x00UL)  /* Control and Status */
#define SYSTICK_RVR         REG32(SYSTICK_BASE + 0x04UL)  /* Reload Value */
#define SYSTICK_CVR         REG32(SYSTICK_BASE + 0x08UL)  /* Current Value */

#define SYSTICK_CSR_ENABLE  (1UL << 0)
#define SYSTICK_CSR_TICKINT (1UL << 1)
#define SYSTICK_CSR_CLKSRC  (1UL << 2)  /* 1 = processor clock */
#define SYSTICK_CSR_COUNTFLAG (1UL << 16)

/* ---- State ---- */

/* Ticks per microsecond, set by ll_systick_init().
 * Defined in hal_systick.c (extern to avoid multiple-definition errors). */
extern uint32_t _systick_us_ticks;

/* Millisecond tick counter, incremented by SysTick_Handler.
 * Defined in hal_systick.c. */
extern volatile uint32_t _systick_ticks;

/**
 * Initialize SysTick for 1ms interrupts + delay functions.
 *   sysclk_hz: System clock frequency in Hz (e.g., 80000000 for 80MHz)
 *
 * After calling this, SysTick fires every 1ms and increments
 * _systick_ticks. ll_delay_ms() uses this counter.
 * ll_delay_us() still uses polling mode for sub-ms precision.
 */
static inline void ll_systick_init(uint32_t sysclk_hz)
{
    _systick_us_ticks = sysclk_hz / 1000000UL;
    _systick_ticks = 0;

    /* Configure SysTick for 1ms interrupt */
    SYSTICK_RVR = (sysclk_hz / 1000UL) - 1;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = SYSTICK_CSR_ENABLE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_CLKSRC;
}

/**
 * SysTick interrupt handler — increments the ms tick counter.
 * Placed here as a weak-overridable default.
 */
void SysTick_Handler(void);

/**
 * Blocking delay in microseconds.
 * Uses a counting loop based on the known ticks-per-us.
 * Less precise than the old SysTick-polling approach, but
 * compatible with the interrupt-driven SysTick.
 */
static inline void ll_delay_us(uint32_t us)
{
    uint32_t count = us * _systick_us_ticks / 4;  /* ~4 cycles per loop iteration */
    while (count--) {
        __asm volatile ("nop");
    }
}

#if defined(THREAD_ENABLED)
/* Thread builds (Core.ST.W5): the wait runs Thread's scheduler, so a program
 * that only blinks with delays stays on the network (sdk/core/core_thread.c;
 * a no-op before core_thread_start(), in interrupts, and inside OpenThread /
 * BLE callbacks). Each pass sleeps (WFI) until the next interrupt, so the
 * delay can end up to ~1 ms late. */
void core_thread_process(void);

static inline void ll_delay_ms(uint32_t ms)
{
    uint32_t start = _systick_ticks;
    while ((_systick_ticks - start) < ms)
        core_thread_process();
}
#else
/**
 * Blocking delay in milliseconds.
 * Uses the SysTick interrupt counter for accurate timing.
 */
static inline void ll_delay_ms(uint32_t ms)
{
    uint32_t start = _systick_ticks;
    while ((_systick_ticks - start) < ms)
        ;
}
#endif

#endif /* LL_SYSTICK_H */
