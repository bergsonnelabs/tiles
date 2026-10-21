/**
 * linklayer_plat.h — Link Layer platform interface header
 *
 * Copied from ST's reference with adaptation:
 * - Removed dependency on bsp.h (we include our own types)
 * - Uses Evnt_timing_t from the link layer library
 *
 * Original: Copyright (c) 2022 STMicroelectronics. Licensed under ST terms.
 */

#ifndef LINKLAYER_PLAT_H
#define LINKLAYER_PLAT_H

#include <stdint.h>
#include "ll_intf.h"  /* For Evnt_timing_t */

extern void LINKLAYER_PLAT_ClockInit(void);
extern void LINKLAYER_PLAT_DelayUs(uint32_t delay);
extern void LINKLAYER_PLAT_Assert(uint8_t condition);
extern void LINKLAYER_PLAT_AclkCtrl(uint8_t enable);
extern void LINKLAYER_PLAT_NotifyWFIEnter(void);
extern void LINKLAYER_PLAT_NotifyWFIExit(void);
extern void LINKLAYER_PLAT_WaitHclkRdy(void);
extern void LINKLAYER_PLAT_GetRNG(uint8_t *ptr_rnd, uint32_t len);
extern void LINKLAYER_PLAT_SetupRadioIT(void (*intr_cb)());
extern void LINKLAYER_PLAT_SetupSwLowIT(void (*intr_cb)());
extern void LINKLAYER_PLAT_TriggerSwLowIT(uint8_t priority);
extern void LINKLAYER_PLAT_EnableIRQ(void);
extern void LINKLAYER_PLAT_DisableIRQ(void);
extern void LINKLAYER_PLAT_EnableSpecificIRQ(uint8_t isr_type);
extern void LINKLAYER_PLAT_DisableSpecificIRQ(uint8_t isr_type);
extern void LINKLAYER_PLAT_EnableRadioIT(void);
extern void LINKLAYER_PLAT_DisableRadioIT(void);
extern void LINKLAYER_PLAT_StartRadioEvt(void);
extern void LINKLAYER_PLAT_StopRadioEvt(void);
extern void LINKLAYER_PLAT_RCOStartClbr(void);
extern void LINKLAYER_PLAT_RCOStopClbr(void);
extern void LINKLAYER_PLAT_RequestTemperature(void);
extern void LINKLAYER_PLAT_EnableOSContextSwitch(void);
extern void LINKLAYER_PLAT_DisableOSContextSwitch(void);
extern void LINKLAYER_PLAT_SCHLDR_TIMING_UPDATE_NOT(Evnt_timing_t *p_evnt_timing);
extern uint32_t LINKLAYER_PLAT_GetSTCompanyID(void);
extern uint32_t LINKLAYER_PLAT_GetUDN(void);


/* Statistics for the radio-bus guards (linklayer_plat.c): WFI resync and the
 * low-ISR guard against the link layer's RCO-compensation assert. */
typedef struct {
    uint32_t magic;              /* 0x42554C4B "BULK" */
    uint32_t wfi_enter;          /* WFIs entered through the sequencer idle hook */
    uint32_t wfi_predicted_cut;  /* of which CubeWBA's rule predicted an AHB5 cut */
    uint32_t wait_calls;         /* LINKLAYER_PLAT_WaitHclkRdy waits */
    uint32_t wait_loops_max;     /* longest wait, in poll loops */
    uint32_t wait_timeouts;      /* waits that gave up (bus never ticked) */
    uint32_t low_isr;            /* low-priority radio ISRs entered */
    uint32_t low_isr_loops_max;  /* longest bus-live wait there, in poll loops */
    uint32_t low_isr_timeouts;
    uint32_t low_isr_same_tick;  /* low ISRs that had to wait out the previous exit's tick */
    uint32_t low_isr_behind_ref; /* low ISRs that found the timer at or behind the LL's reference */
    uint32_t low_isr_ref_loops_max;
    uint32_t low_isr_ref_timeouts;
} wpan_busclk_stats_t;
extern volatile wpan_busclk_stats_t g_wpan_busclk_stats;
void LINKLAYER_PLAT_LowIsrEnter(void);
void LINKLAYER_PLAT_LowIsrExit(void);

#endif /* LINKLAYER_PLAT_H */
