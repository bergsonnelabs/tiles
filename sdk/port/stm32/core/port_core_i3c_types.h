/**
 * port_core_i3c_types.h — STM32 port: I3C types for core_i3c.h
 *
 * Included by core_i3c.h before its prototypes; not for direct use. The
 * bodies of its functions are in port_core_i3c.h.
 */

#ifndef PORT_CORE_I3C_TYPES_H
#define PORT_CORE_I3C_TYPES_H

#include "hal_i3c.h"

/** An I3C peripheral instance: I3C1, I3C2. */
typedef I3C_TypeDef core_i3c_instance_t;

#endif /* PORT_CORE_I3C_TYPES_H */
