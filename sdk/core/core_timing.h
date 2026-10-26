/**
 * core_timing.h — Delays and time tracking
 *
 * Blocking delays (ms/us) and a millisecond tick counter for
 * timeouts and general timekeeping.
 *
 * @studio category timing label=Core.Timing icon=⏱
 *
 * @studio coverage
 *   id:    timing
 *   name:  Timing — delays and millis
 *   blurb: Blocking delay_ms / delay_us, a free-running millis counter
 *          (wraps at ~49 days), and a timeout helper that compares
 *          against a stored start. delay_ms gets full Twin coverage —
 *          the simulator scales the busy-spin by the speed knob. The
 *          others are exposed but unobserved (no virtual clock to
 *          advance).
 */

#ifndef CORE_TIMING_H
#define CORE_TIMING_H

#include "ll_systick.h"
#include "hal_common.h"
#include "core_config.h"  /* SYSCLK_MHZ — for core_delay_ns() */
#include "core_caps.h"

/**
 * Blocking delay in milliseconds.
 *
 * Thread builds (Core.ST.W5): the wait runs Thread's scheduler, so Thread
 * callbacks (UDP receive, state changes) can run inside it, and it can end
 * up to ~1 ms late.
 *
 * @studio expose category=timing name=delay_ms
 * @studio twin full
 * @param ms [1..60000] Delay duration in milliseconds.
 */
static inline void core_delay_ms(uint32_t ms)
{
    ll_delay_ms(ms);
}

/**
 * Blocking delay in microseconds. For delays > 1 ms prefer
 * `delay_ms` — it won't starve the rest of the system as long.
 *
 * @studio expose category=timing name=delay_us
 * @studio twin full
 * @param us [1..10000] Delay duration in microseconds.
 */
static inline void core_delay_us(uint32_t us)
{
    ll_delay_us(us);
}

/**
 * Milliseconds since boot (wraps at ~49 days).
 *
 * @studio expose category=timing name=millis returns=int
 * @studio twin full
 */
static inline uint32_t core_millis(void)
{
    return hal_tick();
}

/**
 * Check if a timeout has elapsed.
 *   start: value returned by core_millis() at the beginning
 *   ms:    timeout duration in milliseconds
 * Returns 1 if expired, 0 otherwise.
 *
 * @studio expose category=timing name=timeout returns=bool
 * @studio twin full
 * @param start Value previously returned from `millis()`.
 * @param ms [1..3600000] Timeout duration in milliseconds.
 */
static inline int core_timeout(uint32_t start, uint32_t ms)
{
    return hal_timeout_expired(start, ms);
}

/* ---- Cycle-accurate delay (sub-µs) ----
 *
 * core_delay_us/ms bottom out at ~1 µs. Tight bitbang timing (e.g. SWCLK
 * rate control in a software CMSIS-DAP probe) needs finer steps. These use
 * the DWT cycle counter on Cortex-M4/M33 (Core.ST.L4/H5/W5); the Cortex-M0+
 * Core.ST.L0 has no DWT, so it falls back to a calibrated busy loop (coarser,
 * but the bitbang use case targets the M4/M33 cores). Call core_cycle_init()
 * once at startup before the first core_delay_cycles/ns. C-only — no DSL surface.
 */

#if CORE_HAS_DWT  /* Cortex-M4/M33 have the DWT cycle counter */

#define CORE_DWT_DEMCR   (*(volatile uint32_t *)0xE000EDFCUL)  /* bit24 TRCENA   */
#define CORE_DWT_CTRL    (*(volatile uint32_t *)0xE0001000UL)  /* bit0  CYCCNTENA */
#define CORE_DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004UL)

/** Enable the DWT cycle counter. Idempotent; call once at startup. */
static inline void core_cycle_init(void)
{
    CORE_DWT_DEMCR |= (1UL << 24);
    CORE_DWT_CTRL  |= 1UL;
}

/** Busy-wait `cycles` CPU cycles (≈ ±a few cycles of loop overhead). */
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

/** Busy-wait `ns` nanoseconds (rounds to whole CPU cycles). */
static inline void core_delay_ns(uint32_t ns)
{
    /* cycles = ns * (SYSCLK_HZ / 1e9) = ns * SYSCLK_MHZ / 1000 (32-bit-safe
     * for ns up to ~17 ms at 250 MHz). */
    core_delay_cycles((ns * (uint32_t)SYSCLK_MHZ) / 1000u);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=L title="No monotonic 64-bit clock"
//   millis wraps at ~49.7 days. Long-running systems (industrial /
//   datalogger) need either a 64-bit upcounter or a wrap-aware
//   helper for diff-since-start.

#endif /* CORE_TIMING_H */
