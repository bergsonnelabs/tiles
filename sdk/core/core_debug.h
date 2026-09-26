/**
 * core_debug.h — Debug output via SWO/ITM trace
 *
 * Core.ST.L4 / W5 / H5 only: the Cortex-M0+ on Core.ST.L0 has no ITM, so
 * there every call is a no-op. core_debug_printf is a macro for
 * hal_debug_printf.
 *
 * @studio category debug label=Core.Debug icon=⌁
 *
 * @studio coverage
 *   id:    debug
 *   name:  Debug — SWO / ITM trace output
 *   page:  /docs/sdk/system
 *   blurb: Tier 1 only. SWO-pin trace output (printf + raw-string) for
 *          on-chip debugging via an ST-Link / J-Link probe. Useful
 *          alongside core_usb_print when you want a debug stream that
 *          doesn't depend on USB enumeration. No DSL surface — debug
 *          output is the realm of escape-to-C.
 */

#ifndef CORE_DEBUG_H
#define CORE_DEBUG_H

#include "hal_debug.h"
#include "core_config.h"  /* SYSCLK_HZ */

/* ============================================================
 * Init
 * ============================================================ */

/** Initialize SWO debug output using the project's SYSCLK_HZ (no-op on Core.ST.L0). */
static inline void core_debug_init(void)
{
    hal_debug_init(SYSCLK_HZ);
}

/* ============================================================
 * Output
 * ============================================================ */

/**
 * Print a string via SWO (no formatting).
 *
 * @param str NUL-terminated string to send on ITM stimulus port 0.
 */
static inline void core_debug_print(const char *str)
{
    hal_debug_puts(str);
}

/** Printf via SWO/ITM — same format as standard printf. */
#define core_debug_printf  hal_debug_printf

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="No DSL surface"
//   Debug output is intentionally Tier 1 — the DSL doesn't have a
//   notion of "debug print to a hardware probe." DSL programs use
//   Core.USB.print for visible output; SWO is reserved for users who
//   are already in C and have a probe attached.
//
// @studio unsupported tier=1 value=L title="No ITM channel selection / timestamps"
//   All output goes to ITM stimulus port 0 with no timestamp packets.
//   Multi-channel routing (e.g., separate streams for log vs. data)
//   and ETM/CYCCNT correlation aren't wrapped.

#endif /* CORE_DEBUG_H */
