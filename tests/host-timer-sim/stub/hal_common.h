/* Host stand-in for sdk/hal/hal_common.h (tests/host-timer-sim): the NVIC
 * calls go to timer_sim.c. */
#ifndef HAL_COMMON_H
#define HAL_COMMON_H
#include <stdint.h>
#define HAL_IRQ_LPTIM2  50
extern int sim_nvic_enabled;
static inline void hal_nvic_enable_irq(uint32_t irqn) { (void)irqn; sim_nvic_enabled = 1; }
static inline void hal_nvic_disable_irq(uint32_t irqn) { (void)irqn; sim_nvic_enabled = 0; }
static inline void hal_nvic_set_priority(uint32_t irqn, uint8_t p) { (void)irqn; (void)p; }
#endif
