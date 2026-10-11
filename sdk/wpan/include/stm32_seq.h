/**
 * stm32_seq.h — ST Sequencer interface
 *
 * Copied from ST Utilities/sequencer/stm32_seq.h
 * Copyright (c) 2019 STMicroelectronics. Licensed under ST terms.
 */

#ifndef STM32_SEQ_H
#define STM32_SEQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Bit mapping type for tasks/events */
typedef uint32_t UTIL_SEQ_bm_t;

/* Warning IDs */
typedef enum {
    UTIL_SEQ_WARNING_INVALIDTASKID,
} UTIL_SEQ_WARNING;

/* Reserved/unused flags parameter */
#define UTIL_SEQ_RFU     0

/* Default mask — all tasks */
#define UTIL_SEQ_DEFAULT (~0U)

/* ---- Public API ---- */

void     UTIL_SEQ_Init(void);
void     UTIL_SEQ_DeInit(void);
void     UTIL_SEQ_Run(UTIL_SEQ_bm_t Mask_bm);
void     UTIL_SEQ_RegTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Flags, void (*Task)(void));
uint32_t UTIL_SEQ_IsRegisteredTask(UTIL_SEQ_bm_t TaskId_bm);
void     UTIL_SEQ_SetTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Task_Prio);
uint32_t UTIL_SEQ_IsSchedulableTask(UTIL_SEQ_bm_t TaskId_bm);
void     UTIL_SEQ_PauseTask(UTIL_SEQ_bm_t TaskId_bm);
uint32_t UTIL_SEQ_IsPauseTask(UTIL_SEQ_bm_t TaskId_bm);
void     UTIL_SEQ_ResumeTask(UTIL_SEQ_bm_t TaskId_bm);
void     UTIL_SEQ_SetEvt(UTIL_SEQ_bm_t EvtId_bm);
void     UTIL_SEQ_ClrEvt(UTIL_SEQ_bm_t EvtId_bm);
void     UTIL_SEQ_WaitEvt(UTIL_SEQ_bm_t EvtId_bm);
UTIL_SEQ_bm_t UTIL_SEQ_IsEvtPend(void);

/* ---- Weak callbacks (override in application) ---- */

void     UTIL_SEQ_Idle(void);
void     UTIL_SEQ_PreIdle(void);
void     UTIL_SEQ_PostIdle(void);
void     UTIL_SEQ_EvtIdle(UTIL_SEQ_bm_t TaskId_bm, UTIL_SEQ_bm_t EvtWaited_bm);
void     UTIL_SEQ_PreTask(uint32_t TaskId);
void     UTIL_SEQ_PostTask(uint32_t TaskId);
void     UTIL_SEQ_CatchWarning(UTIL_SEQ_WARNING WarningId);

#ifdef __cplusplus
}
#endif

#endif /* STM32_SEQ_H */
