/**
 * port_core_timer.h — STM32 port: timer types for core_timer.h (and core_pwm.h,
 * core_stepper.h, which build on it)
 *
 * Included by core_timer.h; not for direct use.
 */

#ifndef PORT_CORE_TIMER_H
#define PORT_CORE_TIMER_H

#include "ll_tim.h"

/** A timer peripheral instance: TIM1, TIM2, ... */
typedef TIM_TypeDef core_timer_instance_t;

#endif /* PORT_CORE_TIMER_H */
