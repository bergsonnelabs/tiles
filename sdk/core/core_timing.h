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

#include "core_types.h"   /* CORE_INLINE, uint32_t */

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
CORE_INLINE void core_delay_ms(uint32_t ms);

/**
 * Blocking delay in microseconds. For delays > 1 ms prefer
 * `delay_ms` — it won't starve the rest of the system as long.
 *
 * @studio expose category=timing name=delay_us
 * @studio twin full
 * @param us [1..10000] Delay duration in microseconds.
 */
CORE_INLINE void core_delay_us(uint32_t us);

/**
 * Milliseconds since boot (wraps at ~49 days).
 *
 * @studio expose category=timing name=millis returns=int
 * @studio twin full
 */
CORE_INLINE uint32_t core_millis(void);

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
CORE_INLINE int core_timeout(uint32_t start, uint32_t ms);

/* ---- Cycle-accurate delay (sub-µs) ----
 *
 * core_delay_us/ms bottom out at ~1 µs. Tight bitbang timing (e.g. SWCLK
 * rate control in a software CMSIS-DAP probe) needs finer steps. These use
 * the DWT cycle counter on Cortex-M4/M33 (Core.ST.L4/H5/W5); the Cortex-M0+
 * Core.ST.L0 has no DWT, so it falls back to a calibrated busy loop (coarser,
 * but the bitbang use case targets the M4/M33 cores). Call core_cycle_init()
 * once at startup before the first core_delay_cycles/ns. C-only — no DSL surface.
 */

/** Enable the DWT cycle counter. Idempotent; call once at startup. */
CORE_INLINE void core_cycle_init(void);

/** Busy-wait `cycles` CPU cycles (≈ ±a few cycles of loop overhead). */
CORE_INLINE void core_delay_cycles(uint32_t cycles);

/** Busy-wait `ns` nanoseconds (rounds to whole CPU cycles). */
CORE_INLINE void core_delay_ns(uint32_t ns);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=L title="No monotonic 64-bit clock"
//   millis wraps at ~49.7 days. Long-running systems (industrial /
//   datalogger) need either a 64-bit upcounter or a wrap-aware
//   helper for diff-since-start.

/* The port's bodies of the functions above. */
#include "port_core_timing.h"

#endif /* CORE_TIMING_H */
