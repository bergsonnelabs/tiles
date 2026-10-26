/**
 * port_core_debug.h — STM32 port: the bodies of core_debug.h
 *
 * Included at the end of core_debug.h, after its prototypes; not for direct
 * use. SWO / ITM output through hal_debug (a no-op on the Cortex-M0+ L0).
 */

#ifndef PORT_CORE_DEBUG_H
#define PORT_CORE_DEBUG_H

#include "hal_debug.h"
#include "core_config.h"  /* SYSCLK_HZ */

static inline void core_debug_init(void)
{
    hal_debug_init(SYSCLK_HZ);
}

static inline void core_debug_print(const char *str)
{
    hal_debug_puts(str);
}

/** Printf via SWO/ITM — same format as standard printf. */
#define core_debug_printf  hal_debug_printf

#endif /* PORT_CORE_DEBUG_H */
