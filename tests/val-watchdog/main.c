/**
 * val-watchdog — the Core watchdog API compiles and links on EVERY Core.
 *
 * Regression pin for a build break that hit every Core.ST.W5 project Studio
 * generated (2026-08-24). The chain:
 *
 *   main.c → core_watchdog.h → hal_dfu.h
 *
 * `core_watchdog.h` includes `hal_dfu.h` unconditionally (it reads the
 * reserved-SRAM recovery stash so `caused_reset()` survives core_init's RMVF
 * clear). But `hal_dfu.h` only defines DFU_MAGIC_ADDR / DFU_ROM_ADDR for the
 * USB-capable parts (STM32L422xx, STM32H523xx), and its two DFU entry points
 * — hal_dfu_reboot() / hal_dfu_jump_to_rom() — were NOT guarded. On the WBA55,
 * which has no USB and therefore no DFU path, merely INCLUDING the header was
 * a hard compile error, whether or not anything called those functions.
 *
 * The watchdog itself is fine on every Core (it's plain IWDG via ll_iwdg) —
 * only the DFU escape hatch is USB-only. So the API must compile everywhere.
 *
 * Build all four Cores (all must succeed):
 *   make && make clean
 *   make TILE=Core.ST.L4 && make clean
 *   make TILE=Core.ST.H5 && make clean
 *   make TILE=Core.ST.L0 && make clean
 *
 * Compile/link validation only — nothing here is meant to run on hardware.
 * core_watchdog_start() is deliberately NOT called: once started the IWDG
 * cannot be stopped, and an un-fed dog on a Core with no ROM-DFU escape just
 * reset-loops. Referencing it is enough to prove it compiles.
 */

#include "core.h"
#include "core_watchdog.h"

/* Reference core_watchdog_start() without arming it: taking its address forces
 * the compiler through the body (the point of the test) while leaving the
 * watchdog off. */
static void (*const start_fn)(uint32_t) = core_watchdog_start;

int main(void)
{
    core_init();

    (void)start_fn;
    (void)core_watchdog_caused_reset();
    core_watchdog_clear_flags();
    core_watchdog_debug_freeze();

    while (1) {
        core_watchdog_feed(); /* no-op: nothing was started */
        core_delay_ms(250);
    }
}
