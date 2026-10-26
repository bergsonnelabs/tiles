/**
 * core_rng.h — Hardware Random Number Generator
 *
 * Generates true random numbers using analog noise sources.
 * Available on Core.ST.L4 (STM32L422), Core.ST.W5 (STM32WBA55), and Core.ST.H5 (STM32H523).
 * Not available on Core.ST.L0 (STM32L011).
 *
 * Usage:
 *   core_rng_init();
 *   uint32_t val = core_rng_read();     // single 32-bit random value
 *   uint32_t buf[4];
 *   core_rng_fill(buf, 4);              // fill buffer with random values
 *   core_rng_deinit();                  // power down (optional)
 *
 * @studio category rng label=Core.RNG icon=🎲
 *
 * @studio coverage
 *   id:    rng
 *   name:  RNG — hardware random numbers
 *   blurb: Wraps the analog-noise RNG peripheral on Core.ST.L4 / Core.ST.W5 /
 *          Core.ST.H5 (Core.ST.L0 has no RNG). Tier 2 exposes a single 32-bit
 *          read (`rng.read32 -> int`); the Twin backs it with a seeded
 *          xorshift PRNG so DSL programs replay identically across runs.
 *          Bulk fill, seed-error check, and deinit stay Tier 1.
 */

#ifndef CORE_RNG_H
#define CORE_RNG_H

#include "core_caps.h"

#if !CORE_HAS_RNG
CORE_CAPS_MISSING_RNG   /* the reason, from the status file (coregen core_caps.h) */
#endif

#include "core_types.h"   /* CORE_INLINE, uint32_t */

/**
 * Initialize the hardware RNG.
 * Enables the peripheral clock, configures the RNG, and waits
 * for the first random number to be ready.
 *
 * On Core.ST.L4: requires HSI48 to be running (auto-enabled if USB is used,
 * otherwise call ll_rcc_hsi48_enable() first).
 */
CORE_INLINE void core_rng_init(void);

/**
 * Read a single 32-bit random value (blocking).
 * Returns 0 on timeout — check core_rng_error() if this happens.
 *
 * @studio expose category=rng name=read32 returns=int
 * @studio twin full
 */
CORE_INLINE uint32_t core_rng_read(void);

/**
 * Fill a buffer with random 32-bit values (blocking).
 * @param buf    Destination buffer
 * @param count  Number of uint32_t values to generate
 * @return       Number of values successfully generated
 */
CORE_INLINE uint32_t core_rng_fill(uint32_t *buf, uint32_t count);

/** Check if the RNG has a seed error (entropy source failure). */
CORE_INLINE int core_rng_error(void);

/** Power down the RNG peripheral. */
CORE_INLINE void core_rng_deinit(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=L title="No bias correction / health tests"
//   The hardware exposes a CED (clock error) and SEIS (seed error)
//   bit which core_rng_error checks, but there's no NIST SP 800-90B
//   continuous-health-test wrapper. Crypto-grade users should consume
//   the raw words through a vetted DRBG (mbedTLS, etc.).

/* The port's bodies of the functions above. */
#include "port_core_rng.h"

/* coregen's core_caps.h marks the functions above that this Core lacks
 * (its status file): calling one is a compile-time error. */
#define CORE_CAPS_DONE_CORE_RNG
#include "core_caps.h"

#endif /* CORE_RNG_H */
