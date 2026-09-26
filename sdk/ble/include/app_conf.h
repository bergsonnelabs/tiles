/**
 * app_conf.h — Minimal BLE application configuration
 *
 * Defines task IDs, priorities, IRQ numbers, and feature flags
 * needed by the BLE stack glue code (ll_sys_if.c, linklayer_plat.c,
 * host_stack_if.c, stm32_seq.c).
 */

#ifndef APP_CONF_H
#define APP_CONF_H

#ifdef USE_HAL_DRIVER
#include "stm32wbaxx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Feature flags — disable everything we don't use
 * ============================================================ */

#define USE_TEMPERATURE_BASED_RADIO_CALIBRATION  0
#define CFG_LPM_LEVEL                            0
#define CFG_LPM_STDBY_SUPPORTED                  0
#define CFG_LPM_STDBY_WAKEUP_TIME               1500
#define CFG_SCM_SUPPORTED                        0
#define CFG_DEBUGGER_LEVEL                       1
#define CFG_RT_DEBUG                             0
#define CFG_LOG_SUPPORTED                        0

/* ============================================================
 * Radio IRQ numbers (STM32WBA55)
 * ============================================================ */

#define RADIO_INTR_NUM            66   /* 2.4GHz Radio interrupt */
#define RADIO_SW_LOW_INTR_NUM     61   /* HASH IRQ repurposed as SW low ISR */

/* ============================================================
 * Radio IRQ priorities
 * ============================================================ */

#define RADIO_INTR_PRIO_HIGH      0    /* Highest priority for radio ISR */
#define RADIO_INTR_PRIO_LOW       5    /* Lowered priority when radio inactive */
#define RADIO_SW_LOW_INTR_PRIO    15   /* SW low ISR — MUST be lowest priority */

/* ============================================================
 * ISR type masks (used by LINKLAYER_PLAT_Enable/DisableSpecificIRQ)
 * ============================================================ */

#define LL_HIGH_ISR_ONLY          0x01
#define LL_LOW_ISR_ONLY           0x02
#define SYS_LOW_ISR               0x04

/* ============================================================
 * Sequencer task IDs
 * ============================================================ */

enum {
    CFG_TASK_HW_RNG = 0,
    CFG_TASK_LINK_LAYER,
    CFG_TASK_HCI_ASYNCH_EVT_ID,
    CFG_TASK_TEMP_MEAS,
    CFG_TASK_BLE_HOST,
    CFG_TASK_AMM,
    CFG_TASK_BPKA,
    CFG_TASK_BLE_TIMER_BCKGND,
    CFG_TASK_FLASH_MANAGER,
    /* Application tasks */
    CFG_TASK_APP_START,
    CFG_TASK_NBR  /* Must be last */
};

/* ============================================================
 * Sequencer priority levels
 * ============================================================ */

enum {
    CFG_SEQ_PRIO_0 = 0,
    CFG_SEQ_PRIO_1,
    CFG_SEQ_PRIO_NBR  /* Must be last */
};

#define UTIL_SEQ_CONF_PRIO_NBR    CFG_SEQ_PRIO_NBR

/* Sequencer idle events — gap_cmd_resp_wait/release, ble_flash.c */
enum {
    CFG_IDLEEVT_PROC_GAP_COMPLETE = 0,
    CFG_IDLEEVT_FLASH_OP,           /* ble_flash.c: a flash manager operation finished */
    CFG_IDLEEVT_NBR
};
#define UTIL_SEQ_CONF_EVTNBR    CFG_IDLEEVT_NBR

/* ============================================================
 * Radio TX power table
 * ============================================================ */

#define CFG_RF_TX_POWER_TABLE_ID  0

/* ============================================================
 * Radio sleep clock SCA
 * ============================================================ */

#define CFG_RADIO_LSE_SLEEP_TIMER_CUSTOM_SCA_RANGE  0

/* ============================================================
 * Log / trace buffer sizes (stubs, not used)
 * ============================================================ */

#define CFG_LOG_TRACE_BUF_SIZE    256
#define CFG_LOG_TRACE_FIFO_SIZE   512

/* ============================================================
 * Advanced Memory Manager (AMM) configuration
 *
 * Pool size in 32-bit words, virtual memory IDs and sizes
 * matching the working ST project.
 * ============================================================ */

/* Forward-declare the element size macro (depends on AMM header) */
#ifndef AMM_VIRTUAL_INFO_ELEMENT_SIZE
/* Approximate -- will be overridden by advanced_memory_manager.h */
#define AMM_VIRTUAL_INFO_ELEMENT_SIZE  4
#endif

#define CFG_MM_POOL_SIZE                          (4000U)  /* bytes */
#define CFG_AMM_VIRTUAL_MEMORY_NUMBER             (2U)
#define CFG_AMM_VIRTUAL_STACK_BLE                 (1U)
#define CFG_AMM_VIRTUAL_STACK_BLE_BUFFER_SIZE     (400U)  /* words (32-bit) */
#define CFG_AMM_VIRTUAL_APP_BLE                   (2U)
#define CFG_AMM_VIRTUAL_APP_BLE_BUFFER_SIZE       (200U)  /* words (32-bit) */
#define CFG_AMM_POOL_SIZE                         (((CFG_MM_POOL_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)) \
                                                   + (AMM_VIRTUAL_INFO_ELEMENT_SIZE * CFG_AMM_VIRTUAL_MEMORY_NUMBER))

/* ============================================================
 * BLEPLAT NVM config
 * ============================================================ */

#define CFG_BLEPLAT_NVM_MAX_SIZE  ((2048/8)-4)

/* ============================================================
 * SNVMA (Simple NVM Arbiter) configuration
 * ============================================================ */

#define CFG_SNVMA_START_SECTOR_ID     ((0x100000 / 0x2000) - 2u)  /* 1MB flash, 8KB pages */
#define CFG_SNVMA_START_ADDRESS       (0x08000000UL + (0x2000UL * CFG_SNVMA_START_SECTOR_ID))

/* ============================================================
 * Low power manager IDs
 * ============================================================ */

enum {
    CFG_LPM_APP = 0,
    CFG_LPM_LOG,
    CFG_LPM_LL_DEEPSLEEP,
    CFG_LPM_LL_HW_RCO_CLBR,
};

/* ============================================================
 * HW_RNG configuration
 * ============================================================ */

#define CFG_HW_RNG_POOL_SIZE   32

/* ============================================================
 * RT Debug configuration
 * ============================================================ */

#define CFG_RT_DEBUG_DTB             0
#define CFG_RT_DEBUG_GPIO_MODULE     0

/* ============================================================
 * RCC interrupt priority (for SCM)
 * ============================================================ */

#define RCC_INTR_PRIO   1

/* ============================================================
 * Error handler — provided as a function, not a macro,
 * to avoid conflicts with main.h declarations.
 * ============================================================ */

/* ============================================================
 * WPAN success code
 * ============================================================ */

#ifndef WPAN_SUCCESS
#define WPAN_SUCCESS  0
#endif

/* ============================================================
 * Default OTP index
 * ============================================================ */

#ifndef DEFAULT_OTP_IDX
#define DEFAULT_OTP_IDX  0
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_CONF_H */
