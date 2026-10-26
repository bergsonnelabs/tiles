/**
 * port_core_pad_types.h — STM32 port: pad types for core_pad.h
 *
 * Included by core_pad.h before its prototypes; not for direct use.
 * core_pad_port_t is the GPIO register block a resolved pad points at
 * (core_pad_fast_t, core_pad_bulk_write). The value checks and the bodies of
 * its functions are in port_core_pad.h.
 */

#ifndef PORT_CORE_PAD_TYPES_H
#define PORT_CORE_PAD_TYPES_H

#include "ll_common.h"   /* GPIO_TypeDef */

/** A GPIO port: what core_pad_fast_t.port points at and core_pad_bulk_write takes. */
typedef GPIO_TypeDef core_pad_port_t;

#endif /* PORT_CORE_PAD_TYPES_H */
