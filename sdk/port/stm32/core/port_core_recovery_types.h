/**
 * port_core_recovery_types.h — STM32 port: what gates core_recovery.h
 *
 * Included near the top of core_recovery.h; not for direct use. hal_dfu.h
 * defines DFU_STRIKE_TAG_ADDR (the reserved-SRAM strike words next to the DFU
 * magic) on the USB Cores, and core_recovery.h declares its API only where
 * it is defined.
 */

#ifndef PORT_CORE_RECOVERY_TYPES_H
#define PORT_CORE_RECOVERY_TYPES_H

#include "hal_dfu.h"

#endif /* PORT_CORE_RECOVERY_TYPES_H */
