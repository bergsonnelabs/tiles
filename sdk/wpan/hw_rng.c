/**
 * hw_rng.c — Hardware RNG using STM32WBA55 RNG peripheral
 *
 * Direct register access — no ST HAL dependency.
 */

#include <stdint.h>
#include "hw.h"
#include "ll_common.h"
#include "ll_rcc.h"

/* RNG peripheral base address (AHB2 non-secure domain) */
#define RNG_BASE    (PERIPH_BASE + 0x020C0800UL)  /* 0x420C0800 */

#define RNG_CR      REG32(RNG_BASE + 0x00UL)
#define RNG_SR      REG32(RNG_BASE + 0x04UL)
#define RNG_DR      REG32(RNG_BASE + 0x08UL)

/* RNG_CR bits */
#define RNG_CR_RNGEN    (1UL << 2)
#define RNG_CR_CONDRST  (1UL << 6)

/* RNG_SR bits */
#define RNG_SR_DRDY     (1UL << 0)
#define RNG_SR_SECS     (1UL << 2)
#define RNG_SR_CEIS     (1UL << 5)
#define RNG_SR_SEIS     (1UL << 6)

/* AHB2ENR bit for RNG */
#define LL_AHB2_RNG     (1UL << 18)

void HW_RNG_Get(uint8_t n, uint32_t *val)
{
    /* Enable RNG clock */
    ll_rcc_ahb2_clk_enable(LL_AHB2_RNG);

    /* Enable RNG */
    RNG_CR = RNG_CR_RNGEN;

    for (uint8_t i = 0; i < n; i++) {
        /* Wait for data ready */
        uint32_t timeout = 100000;
        while (!(RNG_SR & RNG_SR_DRDY) && --timeout)
            ;

        if (timeout == 0) {
            val[i] = 0xDEADBEEF;  /* fallback on timeout */
            continue;
        }

        val[i] = RNG_DR;
    }
}
