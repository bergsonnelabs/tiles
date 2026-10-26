/**
 * port_core_pad.h — STM32 port: pad types and value checks for core_pad.h
 *
 * Included by core_pad.h after its neutral constants; not for direct use.
 * core_pad_port_t is the GPIO register block a resolved pad points at
 * (core_pad_fast_t, core_pad_bulk_write). The checks hold the neutral
 * CORE_PULL_* / CORE_OTYPE_* / CORE_SPEED_* / CORE_EDGE_* values to the LL
 * values they are passed straight through as.
 */

#ifndef PORT_CORE_PAD_H
#define PORT_CORE_PAD_H

#include "ll_common.h"   /* GPIO_TypeDef, LL_GPIO_PULL_* / _OTYPE_* / _SPEED_* */
#include "ll_exti.h"     /* LL_EXTI_RISING / _FALLING / _BOTH */

/** A GPIO port: what core_pad_fast_t.port points at and core_pad_bulk_write takes. */
typedef GPIO_TypeDef core_pad_port_t;

#ifndef __cplusplus
_Static_assert(CORE_PULL_NONE == LL_GPIO_PULL_NONE, "CORE_PULL_NONE");
_Static_assert(CORE_PULL_UP   == LL_GPIO_PULL_UP,   "CORE_PULL_UP");
_Static_assert(CORE_PULL_DOWN == LL_GPIO_PULL_DOWN, "CORE_PULL_DOWN");
_Static_assert(CORE_OTYPE_PP  == LL_GPIO_OTYPE_PP,  "CORE_OTYPE_PP");
_Static_assert(CORE_OTYPE_OD  == LL_GPIO_OTYPE_OD,  "CORE_OTYPE_OD");
_Static_assert(CORE_SPEED_LOW   == LL_GPIO_SPEED_LOW,   "CORE_SPEED_LOW");
_Static_assert(CORE_SPEED_MED   == LL_GPIO_SPEED_MED,   "CORE_SPEED_MED");
_Static_assert(CORE_SPEED_HIGH  == LL_GPIO_SPEED_HIGH,  "CORE_SPEED_HIGH");
_Static_assert(CORE_SPEED_VHIGH == LL_GPIO_SPEED_VHIGH, "CORE_SPEED_VHIGH");
_Static_assert(CORE_EDGE_RISING  == LL_EXTI_RISING,  "CORE_EDGE_RISING");
_Static_assert(CORE_EDGE_FALLING == LL_EXTI_FALLING, "CORE_EDGE_FALLING");
_Static_assert(CORE_EDGE_BOTH    == LL_EXTI_BOTH,    "CORE_EDGE_BOTH");
#endif

#endif /* PORT_CORE_PAD_H */
