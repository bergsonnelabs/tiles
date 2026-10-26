/**
 * port_core_timer_types.h — STM32 port: timer types for core_timer.h (and
 * core_pwm.h, core_stepper.h, which build on it)
 *
 * Included by core_timer.h before its prototypes; not for direct use. The
 * bodies of its functions are in port_core_timer.h.
 */

#ifndef PORT_CORE_TIMER_TYPES_H
#define PORT_CORE_TIMER_TYPES_H

#include "ll_tim.h"

/** A timer peripheral instance: TIM1, TIM2, ... */
typedef TIM_TypeDef core_timer_instance_t;

#endif /* PORT_CORE_TIMER_TYPES_H */
