/**
 * port_core_ble_hw.h — STM32 port: the chip access behind core_ble.c
 *
 * Included by core_ble.c only; not for direct use. The radio stack (sdk/ble,
 * sdk/wpan) has its own; core_ble.c needs only the watchdog refresh, which
 * _core_reset_ble() runs between sequencer passes while it waits for a
 * disconnect.
 */

#ifndef PORT_CORE_BLE_HW_H
#define PORT_CORE_BLE_HW_H

#include "ll_rcc.h"     /* RCC_BASE, which ll_iwdg.h uses */
#include "ll_iwdg.h"

/* Refresh the IWDG; harmless if it isn't running. */
static inline void port_core_ble_feed_watchdog(void)
{
    ll_iwdg_refresh();
}

#endif /* PORT_CORE_BLE_HW_H */
