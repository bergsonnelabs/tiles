/**
 * wpan_sys.c -- link layer system interface (ll_sys_*) and small runtime stubs
 *
 * Shared by every ST link-layer protocol (BLE, Thread). Formerly
 * sdk/ble/ble_glue.c; the functions and their order are unchanged, so BLE
 * images stay byte-identical (the Makefile keeps this file at ble_glue's
 * place in the BLE link order).
 *
 * Real implementations elsewhere:
 *   linklayer_plat.c — LINKLAYER_PLAT_* functions (wrapped below)
 *   ll_sys_if.c     — ll_sys_bg_process_init, ll_sys_schedule_bg_process, etc.
 *   stm32_seq.c     — UTIL_SEQ_* sequencer
 *   sdk/ble/host_stack_if.c — HostStack/BleStackCB (BLE builds)
 *
 * This file provides:
 *   - ll_sys_* functions (link layer system interface)
 *   - deep-sleep stubs (not used)
 *   - _sbrk (heap allocator for nano.specs)
 *   - APP_DBG_MSG (debug printf stub)
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "app_conf.h"

/* ============================================================
 * ll_sys_* — Link Layer system interface stubs
 *
 * NOTE: ll_sys_bg_process_init, ll_sys_schedule_bg_process,
 * ll_sys_schedule_bg_process_isr, ll_sys_config_params, and
 * ll_sys_reset are now in ll_sys_if.c
 * ============================================================ */

void ll_sys_init(void)
{
    extern void LINKLAYER_PLAT_ClockInit(void);
    LINKLAYER_PLAT_ClockInit();
}

void ll_sys_delay_us(uint32_t delay)
{
    volatile uint32_t count = delay * 25;
    while (count--)
        ;
}

void ll_sys_assert(uint8_t condition)
{
    if (!condition)
    {
        while (1)
            ;
    }
}

void ll_sys_get_rng(uint8_t *ptr_rnd, uint32_t len)
{
    extern void LINKLAYER_PLAT_GetRNG(uint8_t *p_rng, uint32_t len);
    LINKLAYER_PLAT_GetRNG(ptr_rnd, len);
}

void ll_sys_radio_ack_ctrl(uint8_t enable)
{
    extern void LINKLAYER_PLAT_AclkCtrl(uint8_t enable);
    LINKLAYER_PLAT_AclkCtrl(enable);
}

void ll_sys_radio_wait_for_busclkrdy(void)
{
    extern void LINKLAYER_PLAT_WaitHclkRdy(void);
    LINKLAYER_PLAT_WaitHclkRdy();
}

void ll_sys_setup_radio_intr(void (*intr_cb)())
{
    extern void LINKLAYER_PLAT_SetupRadioIT(void (*intr_cb)());
    LINKLAYER_PLAT_SetupRadioIT(intr_cb);
}

void ll_sys_setup_radio_sw_low_intr(void (*intr_cb)())
{
    extern void LINKLAYER_PLAT_SetupSwLowIT(void (*intr_cb)());
    LINKLAYER_PLAT_SetupSwLowIT(intr_cb);
}

void ll_sys_radio_sw_low_intr_trigger(uint8_t priority)
{
    extern void LINKLAYER_PLAT_TriggerSwLowIT(uint8_t priority);
    LINKLAYER_PLAT_TriggerSwLowIT(priority);
}

void ll_sys_radio_evt_not(uint8_t start)
{
    extern void LINKLAYER_PLAT_StartRadioEvt(void);
    extern void LINKLAYER_PLAT_StopRadioEvt(void);
    if (start) {
        LINKLAYER_PLAT_StartRadioEvt();
    } else {
        LINKLAYER_PLAT_StopRadioEvt();
    }
}

void ll_sys_rco_clbr_not(uint8_t start)
{
    extern void LINKLAYER_PLAT_RCOStartClbr(void);
    extern void LINKLAYER_PLAT_RCOStopClbr(void);
    if (start) {
        LINKLAYER_PLAT_RCOStartClbr();
    } else {
        LINKLAYER_PLAT_RCOStopClbr();
    }
}

/* STM32CubeWBA 1.7.0's link layers (BLE_Thread included) also call
 * ll_sys_phy_start_clbr() / ll_sys_phy_stop_clbr() (ll_sys_cs.c, around a PHY
 * calibration: LINKLAYER_PLAT_PhyStartClbr / PhyStopClbr). The 1.6.1 libraries
 * the SDK links, LinkLayer_BLE_Thread_lib.a too, do not reference them, so
 * they are not here; an upgrade to 1.7.0 must add them. */

void ll_sys_request_temperature(void)
{
}

void ll_sys_schldr_timing_update_not(void *p_evnt_timing)
{
    (void)p_evnt_timing;
}

void ll_sys_bg_process(void)
{
    extern void emngr_handle_all_events(void);
#if defined(BLE)
    extern void HostStack_Process(void);
#endif
    extern void ll_sys_schedule_bg_process(void);
    extern uint8_t emngr_can_mcu_sleep(void);

    /* Always process events — don't gate on emngr_can_mcu_sleep().
     * Skipping processing causes connection supervision timeouts (0x08)
     * because the link layer events don't get handled in time. */
    emngr_handle_all_events();
#if defined(BLE)
    HostStack_Process();          /* BLE host; Thread runs from its own tasks */
#endif

    if (emngr_can_mcu_sleep() == 0)
    {
        ll_sys_schedule_bg_process();
    }
}

void ll_sys_enable_irq(void)
{
    extern void LINKLAYER_PLAT_EnableIRQ(void);
    LINKLAYER_PLAT_EnableIRQ();
}

void ll_sys_disable_irq(void)
{
    extern void LINKLAYER_PLAT_DisableIRQ(void);
    LINKLAYER_PLAT_DisableIRQ();
}

void ll_sys_enable_specific_irq(uint8_t isr_type)
{
    extern void LINKLAYER_PLAT_EnableSpecificIRQ(uint8_t isr_type);
    LINKLAYER_PLAT_EnableSpecificIRQ(isr_type);
}

void ll_sys_disable_specific_irq(uint8_t isr_type)
{
    extern void LINKLAYER_PLAT_DisableSpecificIRQ(uint8_t isr_type);
    LINKLAYER_PLAT_DisableSpecificIRQ(isr_type);
}

void ll_sys_enable_os_context_switch(void)
{
}

void ll_sys_disable_os_context_switch(void)
{
}

/* Deep sleep — not used */
int ll_sys_dp_slp_init(void)
{
    return 0;
}

int ll_sys_dp_slp_enter(uint32_t dp_slp_duration)
{
    (void)dp_slp_duration;
    return 0;
}

int ll_sys_dp_slp_exit(void)
{
    return 0;
}

int ll_sys_dp_slp_get_state(void)
{
    return 0; /* LL_SYS_DP_SLP_DISABLED */
}

void ll_sys_dp_slp_wakeup_evt_clbk(void const *ptr_arg)
{
    (void)ptr_arg;
}

uint8_t ll_sys_get_concurrent_state_machines_num(void)
{
    return 1;
}

/* LINKLAYER_DEBUG_SIGNAL_* now provided by RTDebug.c in sdk/stm/ */

/* Debug printf stub */
void APP_DBG_MSG(const char *fmt, ...)
{
    (void)fmt;
}

/* Weak _sbrk for malloc (nano.specs) */
void *_sbrk(int incr)
{
    extern char end;
    static char *heap_end = 0;
    if (heap_end == 0)
        heap_end = &end;
    char *prev = heap_end;
    heap_end += incr;
    return prev;
}

/* missed_hci_event_flag — referenced by host_stack_if.c and ll_sys_startup.c (BLE) */
uint8_t missed_hci_event_flag = 0;
