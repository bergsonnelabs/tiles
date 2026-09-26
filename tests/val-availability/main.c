/* NEGATIVE TEST */
/**
 * val-availability -- peripheral availability guards
 *
 * Core.ST.L0, clock=default
 *
 * This test MUST FAIL to compile. It verifies that core_dac.h correctly
 * produces a #error when included on a Core that lacks DAC hardware.
 *
 * Core.ST.L0 is STM32L011 -- no DAC peripheral. core_dac.h contains:
 *   #if !defined(STM32H523xx)
 *   #error "core_dac.h: DAC is not available on this Core tile."
 *   #endif
 *
 * Expected result: compilation fails with the #error message above.
 * If this file compiles successfully, the availability guard is broken.
 */

#include "core.h"

/* This include MUST trigger a compile-time #error on Core.ST.L0 */
#include "core_dac.h"

int main(void)
{
    core_init();

    /* This code should never be reached -- the #error above must fire */
    core_dac_init();
    core_dac_write(2048);

    while (1) {
        core_delay_ms(1000);
    }
}
