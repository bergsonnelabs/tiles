/**
 * port_core_fault.h — STM32 port: fault types for core_fault.h
 *
 * Included by core_fault.h; not for direct use.
 */

#ifndef PORT_CORE_FAULT_H
#define PORT_CORE_FAULT_H

#include "hal_fault.h"

/** Which fault: HAL_FAULT_HARD, _MEMMANAGE, _BUS, _USAGE (Cortex-M). */
typedef hal_fault_type_t core_fault_type_t;

/** Registers stacked on exception entry: r0-r3, r12, lr, pc, psr. */
typedef hal_fault_frame_t core_fault_frame_t;

#endif /* PORT_CORE_FAULT_H */
