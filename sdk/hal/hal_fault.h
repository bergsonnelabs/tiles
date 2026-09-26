/**
 * hal_fault.h — Fault handler with register dump
 *
 * Overrides the default HardFault, MemManage, BusFault, and UsageFault
 * handlers. On fault, per Core:
 *   - every Core: calls the optional callback with the stacked frame;
 *   - Core.ST.L4 only: dumps the frame and CFSR over USB CDC (polled, no
 *     interrupts needed; skipped if USB never enumerated);
 *   - Core.ST.L4 / H5 built with a DFU path ("bootloader": "rom"): blinks
 *     SOS once, then reboots into the ROM bootloader so the board can be
 *     reflashed;
 *   - otherwise (Core.ST.L0 / W5, or no DFU path): blinks SOS forever.
 *
 * Usage:
 *   Just link hal_fault.c into your project — the handlers are
 *   defined with the correct symbol names and override the weak
 *   aliases in the startup assembly.
 *
 *   Optional: call hal_fault_set_callback() to add your own handler
 *   (e.g., log to flash, trigger a reset after delay).
 */

#ifndef HAL_FAULT_H
#define HAL_FAULT_H

#include <stdint.h>

/** Stacked register frame pushed by Cortex-M hardware on exception entry. */
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;    /* Link Register (return address before fault) */
    uint32_t pc;    /* Program Counter (instruction that faulted) */
    uint32_t psr;   /* Program Status Register */
} hal_fault_frame_t;

/** Fault type passed to the callback. */
typedef enum {
    HAL_FAULT_HARD = 0,
    HAL_FAULT_MEMMANAGE,
    HAL_FAULT_BUS,
    HAL_FAULT_USAGE,
} hal_fault_type_t;

/**
 * Optional user callback, called from fault context before SOS.
 * Keep it minimal — no heap, no interrupts, no blocking.
 */
typedef void (*hal_fault_callback_t)(hal_fault_type_t type, const hal_fault_frame_t *frame);

/**
 * Register a fault callback (optional).
 * Called on every Core before the SOS blink (and before the L4's USB dump).
 *
 * @param cb Callback run in fault context; NULL removes it.
 */
void hal_fault_set_callback(hal_fault_callback_t cb);

#endif /* HAL_FAULT_H */
