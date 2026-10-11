/* Host stand-in for coregen's core_config.h (tests/host-timer-sim): the W5
 * clock knob's levels, picked with -DSIM_MHZ (APB1 / AHB undivided, as
 * coregen sets them). */
#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H
#define SYSCLK_HZ  (SIM_MHZ * 1000000UL)
#define HCLK_HZ    SYSCLK_HZ
#define PCLK1_HZ   HCLK_HZ
#endif
