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

#endif /* LINKLAYER_PLAT_H */
