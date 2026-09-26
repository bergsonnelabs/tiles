/**
 * hal_lsi.h — measure the Core.ST.L0's LSI (sdk/hal/hal_lsi.c).
 *
 * Most code wants ll_lsi_hz() (ll_common.h): the value the RTC and IWDG were
 * programmed from, measured once and then fixed. This is the measurement
 * itself, for tests and for code that wants a fresh reading (the LSI moves
 * with temperature and supply). It does not change what ll_lsi_hz() returns.
 */
#ifndef HAL_LSI_H
#define HAL_LSI_H

#include <stdint.h>

#if defined(STM32L011xx)
/** LSI frequency in Hz measured now against the TIM21 clock (SYSCLK), over
 *  128 LSI periods (~3.5 ms). Turns the LSI on if it is off. A run the CPU
 *  was held off from (a missed capture) is retried, up to three runs. 0 if
 *  TIM21 is in use (RCC_APB2ENR.TIM21EN set), no LSI edge arrived, or all
 *  three runs were disturbed. One reading scatters about +-0.1 % (the LSI's
 *  own short-term jitter; bench 2026-09-26), and the LSI wanders by about as
 *  much over tens of seconds. */
uint32_t hal_lsi_measure_hz(void);
#endif

#endif /* HAL_LSI_H */
