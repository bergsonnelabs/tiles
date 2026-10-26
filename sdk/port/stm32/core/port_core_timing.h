/**
 * port_core_timing.h — STM32 port: the bodies of core_timing.h
 *
 * Included at the end of core_timing.h, after its prototypes; not for direct
 * use. Delays and the tick over ll_systick / hal_common; the cycle delays use
 * the DWT cycle counter where the Core has one (CORE_HAS_DWT), else a
 * calibrated busy loop.
 */

#ifndef PORT_CORE_TIMING_H
#define PORT_CORE_TIMING_H

#include "ll_systick.h"
#include "hal_common.h"
#include "core_config.h"  /* SYSCLK_MHZ — for core_delay_ns() */
#include "core_caps.h"

static inline void core_delay_ms(uint32_t ms)
{
    ll_delay_ms(ms);
}

static inline void core_delay_us(uint32_t us)
{
    ll_delay_us(us);
}

static inline uint32_t core_millis(void)
{
    return hal_tick();
}

static inline int core_timeout(uint32_t start, uint32_t ms)
{
    return hal_timeout_expired(start, ms);
}

#if CORE_HAS_DWT  /* Cortex-M4/M33 have the DWT cycle counter */

#define CORE_DWT_DEMCR   (*(volatile uint32_t *)0xE000EDFCUL)  /* bit24 TRCENA   */
#define CORE_DWT_CTRL    (*(volatile uint32_t *)0xE0001000UL)  /* bit0  CYCCNTENA */
#define CORE_DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004UL)

static inline void core_cycle_init(void)
{
    CORE_DWT_DEMCR |= (1UL << 24);
    CORE_DWT_CTRL  |= 1UL;
}

static inline void core_delay_cycles(uint32_t cycles)
{
    uint32_t start = CORE_DWT_CYCCNT;
    while ((CORE_DWT_CYCCNT - start) < cycles) { }
}

#else  /* Cortex-M0+ : no DWT — calibrated busy loop (~3 cycles/iteration) */

static inline void core_cycle_init(void) { }

static inline void core_delay_cycles(uint32_t cycles)
{
    volatile uint32_t n = cycles / 3u;
    while (n--) { __asm volatile("nop"); }
}

#endif

static inline void core_delay_ns(uint32_t ns)
{
    /* cycles = ns * (SYSCLK_HZ / 1e9) = ns * SYSCLK_MHZ / 1000 (32-bit-safe
     * for ns up to ~17 ms at 250 MHz). */
    core_delay_cycles((ns * (uint32_t)SYSCLK_MHZ) / 1000u);
}

#endif /* PORT_CORE_TIMING_H */
