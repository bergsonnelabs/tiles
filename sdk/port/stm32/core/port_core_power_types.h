/**
 * port_core_power_types.h — STM32 port: what core_power.h includes first
 *
 * Included near the top of core_power.h, before core_pad.h and
 * core_watchdog.h; not for direct use. No types yet: the LL headers the
 * bodies in port_core_power.h use, in the order core_power.h always included
 * them. GCC emits the out-of-line static ll_ functions in the order they are
 * defined, so including these later (from port_core_power.h, after
 * core_watchdog.h has brought in ll_iwdg.h) moves functions around in the
 * image without changing them (ll_iwdg_init ahead of ll_rtc_resync on
 * Core.ST.L0, for one).
 */

#ifndef PORT_CORE_POWER_TYPES_H
#define PORT_CORE_POWER_TYPES_H

#include "ll_pwr.h"
#include "ll_rtc.h"
#include "ll_rcc.h"
#include "ll_iwdg.h"
#include "ll_systick.h"

#endif /* PORT_CORE_POWER_TYPES_H */
