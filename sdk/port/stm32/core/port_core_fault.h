/**
 * port_core_fault.h — STM32 port: the bodies of core_fault.h
 *
 * Included at the end of core_fault.h, after its prototypes; not for direct
 * use. The types are in port_core_fault_types.h.
 */

#ifndef PORT_CORE_FAULT_H
#define PORT_CORE_FAULT_H

#include "hal_fault.h"
#include "port_core_fault_types.h"

static inline void core_fault_set_callback(core_fault_cb_t cb)
{
    hal_fault_set_callback(cb);
}

#endif /* PORT_CORE_FAULT_H */
