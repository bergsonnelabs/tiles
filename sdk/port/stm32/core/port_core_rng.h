/**
 * port_core_rng.h — STM32 port: the bodies of core_rng.h
 *
 * Included at the end of core_rng.h, after its prototypes; not for direct
 * use. The RNG peripheral over ll_rng, with each MCU's RNG clock selection.
 */

#ifndef PORT_CORE_RNG_H
#define PORT_CORE_RNG_H

#include "ll_rng.h"
#include "ll_rcc.h"

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

static inline uint32_t core_rng_read(void)
{
    return ll_rng_read();
}

static inline uint32_t core_rng_fill(uint32_t *buf, uint32_t count)
{
    return ll_rng_fill(buf, count);
}

static inline int core_rng_error(void)
{
    return ll_rng_seed_error() || ll_rng_clock_error();
}

static inline void core_rng_deinit(void)
{
    ll_rng_disable();
}

#endif /* PORT_CORE_RNG_H */
