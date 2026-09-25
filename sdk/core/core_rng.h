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

#if !defined(STM32L422xx) && !defined(STM32WBA55xx) && !defined(STM32H523xx)
#error "core_rng.h: Hardware RNG is not available on this Core tile. Only Core.ST.L4, Core.ST.W5, and Core.ST.H5 have an RNG peripheral."
#endif

#include "ll_rng.h"
#include "ll_rcc.h"

/**
 * Initialize the hardware RNG.
 * Enables the peripheral clock, configures the RNG, and waits
 * for the first random number to be ready.
 *
 * On Core.ST.L4: requires HSI48 to be running (auto-enabled if USB is used,
 * otherwise call ll_rcc_hsi48_enable() first).
 */
static inline void core_rng_init(void)
{
#if defined(STM32L422xx)
    /* L422 RNG is clocked from HSI48 — ensure it's running */
    ll_rcc_hsi48_enable();
    while (!ll_rcc_hsi48_ready())
        ;
    /* RNG clock = the 48 MHz clock, CLK48SEL = RCC_CCIPR[27:26]; 00 = HSI48
     * (RM0394 §6.4.27), the same selection USB uses. This used to write
     * [29:28], ADCSEL, which the L41x/L42x don't have. */
    MOD_BITS(REG32(RCC_BASE + 0x88UL), 0x3UL << 26, 0x0UL << 26);
#elif defined(STM32H523xx)
    /* H5 RNG is clocked from HSI48. Enable it if not already running
     * (USB init may have already enabled it). */
    ll_rcc_hsi48_enable();
    while (!ll_rcc_hsi48_ready())
        ;
    /* Select HSI48 as RNG clock: RCC_CCIPR5 bits [3:2] = 0b11 (HSI48) */
    MOD_BITS(REG32(RCC_BASE + 0xD4UL), 0x3UL << 2, 0x3UL << 2);
#endif
    /* WBA55: RNG clock source is set inside ll_rng_enable() (HSI16 via CCIPR2) */

    ll_rcc_ahb2_clk_enable(LL_AHB2_RNG);
    ll_rng_enable();
}

/**
 * Read a single 32-bit random value (blocking).
 * Returns 0 on timeout — check core_rng_error() if this happens.
 *
 * @studio expose category=rng name=read32 returns=int
 * @studio twin full
 */
static inline uint32_t core_rng_read(void)
{
    return ll_rng_read();
}

/**
 * Fill a buffer with random 32-bit values (blocking).
 * @param buf    Destination buffer
 * @param count  Number of uint32_t values to generate
 * @return       Number of values successfully generated
 */
static inline uint32_t core_rng_fill(uint32_t *buf, uint32_t count)
{
    return ll_rng_fill(buf, count);
}

/** Check if the RNG has a seed error (entropy source failure). */
static inline int core_rng_error(void)
{
    return ll_rng_seed_error() || ll_rng_clock_error();
}

/** Power down the RNG peripheral. */
static inline void core_rng_deinit(void)
{
    ll_rng_disable();
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=L title="No bias correction / health tests"
//   The hardware exposes a CED (clock error) and SEIS (seed error)
//   bit which core_rng_error checks, but there's no NIST SP 800-90B
//   continuous-health-test wrapper. Crypto-grade users should consume
//   the raw words through a vetted DRBG (mbedTLS, etc.).

#endif /* CORE_RNG_H */
