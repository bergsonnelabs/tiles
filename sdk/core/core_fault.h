/**
 * core_fault.h — Fault handler configuration
 *
 * The SDK automatically catches HardFault, MemManage, BusFault, and
 * UsageFault exceptions. No setup needed. On fault the LED blinks SOS; on
 * Core.ST.L4 the registers are also dumped over USB CDC (if it enumerated);
 * on Core.ST.L4 / H5 ROM-DFU builds one SOS is followed by a reboot into the
 * ROM bootloader. See hal_fault.h for the per-Core detail.
 *
 * Optionally register a callback to run custom logic first (every Core).
 *
 * @studio category fault label=Core.Fault icon=⚠
 *
 * @studio coverage
 *   id:    fault
 *   name:  Fault — exception handlers
 *   page:  /docs/sdk/system
 *   blurb: Tier 1 only. The SDK installs default HardFault / MemManage /
 *          BusFault / UsageFault handlers that SOS-blink the LED (and, on
 *          the L4 only, dump registers over USB CDC). The single API here lets escape-
 *          to-C users hook in custom pre-dump logic (e.g., log to NVM
 *          before the reset). DSL programs aren't expected to handle
 *          their own faults.
 */

#ifndef CORE_FAULT_H
#define CORE_FAULT_H

#include "hal_fault.h"

/**
 * Register a fault callback, run in fault context before the SOS blink (and
 * before the L4's USB register dump). Keep it minimal: no heap, no interrupts.
 *
 * @param cb Callback receiving the fault type and stacked frame; NULL removes it.
 */
static inline void core_fault_set_callback(hal_fault_callback_t cb)
{
    hal_fault_set_callback(cb);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=1 value=M title="No structured fault report capture"
//   The default handler dumps to USB CDC at runtime on the L4 only (the
//   W5, H5 and L0 lose the cause), and there's no crash log that survives
//   reset for post-mortem analysis.
//   A small ring buffer in backup or NVM (with the captured PC / LR
//   / xPSR / fault status registers) would make field debugging
//   tractable.

#endif /* CORE_FAULT_H */
