/**
 * core_reset.c — core_reset() and the boot-time reset reason (core_reset.h)
 *
 * Compiled into every build, like core_watchdog.c. The BLE link drop lives in
 * core_ble.c (BLE builds only) behind a weak hook; the chip's part (reset
 * flags, watchdog refresh, serial flush, the L4 empty-flash fix-up) in the
 * port's port_core_reset_hw.h. The system reset itself is the Cortex-M
 * architecture's (SCB AIRCR), the same on every Core, and stays here.
 */

#include "core_reset.h"
/* The port's hardware steps: port_core_reset_read_flags(), _clear_flags() and
 * _classify() for the reset reason; _feed_watchdog(), _flush() and _prepare()
 * for core_reset(). */
#include "port_core_reset_hw.h"

/* ============================================================
 * Reset reason
 * ============================================================ */

static uint32_t            s_flags;
static core_reset_reason_t s_reason = CORE_RESET_OTHER;
static uint8_t             s_captured;

void core_reset_capture(void)
{
    if (s_captured) return;
    s_flags    = port_core_reset_read_flags();
    s_reason   = port_core_reset_classify(s_flags);
    s_captured = 1;
    port_core_reset_clear_flags();
}

core_reset_reason_t core_reset_reason(void)
{
    core_reset_capture();
    return s_reason;
}

uint32_t core_st_reset_flags(void)
{
    core_reset_capture();
    return s_flags;
}

/* ============================================================
 * core_reset()
 * ============================================================ */

/* Drops a connected BLE link and waits for it to go (core_ble.c). Absent
 * from builds without BLE. */
extern void _core_reset_ble(void) __attribute__((weak));

/* SCB AIRCR: ARMv7-M / ARMv8-M, at this address on every Cortex-M. */
#define CORE_RESET_SCB_AIRCR  (*(volatile uint32_t *)0xE000ED0CUL)

void core_reset(void)
{
    port_core_reset_feed_watchdog();    /* harmless if the watchdog isn't running */

    /* 1. BLE: a clean drop, so the central sees it now rather than at its
     *    supervision timeout. Bounded; skipped from interrupt context. */
    if (_core_reset_ble) _core_reset_ble();

    /* 2. Serial output: let it reach the terminal (USB CDC on Core.ST.L4 /
     *    H5). Bounded. */
    port_core_reset_flush();

    __asm volatile ("cpsid i" ::: "memory");

    port_core_reset_prepare();

    /* 3. System reset (ARMv7-M / ARMv8-M AIRCR; PRIGROUP kept, as CMSIS
     *    NVIC_SystemReset does). Sets SFTRSTF. */
    __asm volatile ("dsb" ::: "memory");
    CORE_RESET_SCB_AIRCR = (0x05FAUL << 16) | (CORE_RESET_SCB_AIRCR & (7UL << 8)) | (1UL << 2);
    __asm volatile ("dsb" ::: "memory");
    for (;;)
        ;
}
