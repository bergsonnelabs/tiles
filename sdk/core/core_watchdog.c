/**
 * core_watchdog.c — watchdog state shared across translation units.
 *
 * The rest of core_watchdog is header-only. The IWDG has no "running" bit on
 * the L0/L4, so core_watchdog_start() records the timeout it programmed here,
 * and core_power.h reads it to sleep in fed chunks (the IWDG keeps counting in
 * Stop and Standby). Compiled into every build, like core_led.c.
 */

#include <stdint.h>

/* Timeout (ms) of the IWDG this firmware started; 0 = not started. */
volatile uint32_t _core_watchdog_timeout_ms;
