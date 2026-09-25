/**
 * ble_seq_idle.c — Sequencer idle hooks for BLE
 */

#include <stdint.h>
#include "stm32_seq.h"
#include "ll_rcc.h"

extern int ll_sys_dp_slp_exit(void);
extern void LINKLAYER_PLAT_NotifyWFIEnter(void);
extern void LINKLAYER_PLAT_NotifyWFIExit(void);
extern void LINKLAYER_PLAT_WaitHclkRdy(void);

/**
 * Called by UTIL_SEQ_Run when no tasks are pending, with PRIMASK set (the
 * sequencer's idle critical section), so no ISR runs until this returns.
 *
 * WFI is Sleep mode. With the radio not active and RADIOSMEN / STRADIOCLKON
 * clear, Sleep gates the AHB5 radio bus clock, and after the wake the radio
 * registers hold the values frozen at WFI entry until the bus resynchronizes:
 * the sleep timer (0x48020014) read back up to ~1.6 ms stale (bench,
 * 2026-09-25). The link layer's RCO compensation (low ISR) reads that register
 * directly and asserts "now > reference"; a stale read tripped it and hung the
 * CPU in HASH_IRQHandler with interrupts masked, after minutes of advertising.
 *
 * So wrap the WFI the way CubeWBA's PWR_EnterSleepMode / PWR_ExitSleepMode do
 * (stm32_lpm_if.c): NotifyWFIEnter decides whether AHB5 will be cut,
 * NotifyWFIExit samples the frozen timer value, and WaitHclkRdy (which the LL
 * calls via ll_sys_radio_wait_for_busclkrdy) spins until the timer moves,
 * i.e. the bus is back. That wait runs here, still inside the critical
 * section, so every ISR the wake releases sees a live radio bus. It costs at
 * most one sleep-timer tick (~30 us), only when AHB5 was actually cut.
 */
void UTIL_SEQ_Idle(void)
{
    LINKLAYER_PLAT_NotifyWFIEnter();
    __asm volatile ("wfi");
    LINKLAYER_PLAT_NotifyWFIExit();
    LINKLAYER_PLAT_WaitHclkRdy();
}

/**
 * Called before UTIL_SEQ_Idle.
 */
void UTIL_SEQ_PreIdle(void)
{
}

/**
 * Called after UTIL_SEQ_Idle (CPU woke up).
 * Must re-enable radio AHB5 clock and notify link layer.
 */
void UTIL_SEQ_PostIdle(void)
{
    /* Re-enable AHB5 radio clock (may have been gated during sleep) */
    ll_rcc_ahb5_clk_enable(LL_AHB5_RADIO);

    /* Notify link layer that deep sleep is over */
    ll_sys_dp_slp_exit();
}

/**
 * Called by UTIL_SEQ_WaitEvt while waiting for an event.
 */
void UTIL_SEQ_EvtIdle(uint32_t TaskId_bm, uint32_t EvtWaited_bm)
{
    (void)EvtWaited_bm;
    UTIL_SEQ_Run(~TaskId_bm);
}
