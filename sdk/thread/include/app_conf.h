/**
 * app_conf.h — Thread build configuration for the shared radio plumbing
 *
 * The sdk/wpan sources (sequencer, link-layer platform, flash manager) read
 * their task ids, IRQ numbers and priorities from "app_conf.h". BLE builds
 * get sdk/ble/include/app_conf.h; Thread builds get this one (the Makefile
 * puts exactly one of the two on the include path); BLE + Thread builds get
 * the BLE one, which adds the OpenThread tasks below. The radio values match
 * the BLE file: the same IRQs, the same priorities.
 */

#ifndef APP_CONF_H
#define APP_CONF_H

/* BLE + Thread builds take sdk/ble/include/app_conf.h (the BLE tasks, AMM and
 * BLE settings plus the OpenThread tasks); reaching this file there means the
 * include order broke. */
#if defined(BLE_ENABLED)
#error "BLE + Thread builds use sdk/ble/include/app_conf.h: put sdk/ble/include ahead of sdk/thread/include"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Feature flags — everything off that the SDK does not use
 * ============================================================ */

#define USE_TEMPERATURE_BASED_RADIO_CALIBRATION  0
#define CFG_LPM_LEVEL                            0
#define CFG_LPM_STDBY_SUPPORTED                  0
#define CFG_SCM_SUPPORTED                        0
#define CFG_DEBUGGER_LEVEL                       1
#define CFG_RT_DEBUG                             0
#define CFG_LOG_SUPPORTED                        0

/* ============================================================
 * Radio IRQs (STM32WBA55) — as in sdk/ble/include/app_conf.h
 * ============================================================ */

#define RADIO_INTR_NUM            66   /* 2.4 GHz radio interrupt */
#define RADIO_SW_LOW_INTR_NUM     61   /* HASH IRQ repurposed as the SW low ISR */

#define RADIO_INTR_PRIO_HIGH      0    /* radio active */
#define RADIO_INTR_PRIO_LOW       5    /* radio not active (sleep timer only) */
#define RADIO_SW_LOW_INTR_PRIO    15   /* SW low ISR — lowest priority */

/* ISR type masks (LINKLAYER_PLAT_Enable/DisableSpecificIRQ) */
#define LL_HIGH_ISR_ONLY          0x01
#define LL_LOW_ISR_ONLY           0x02
#define SYS_LOW_ISR               0x04

/* ============================================================
 * Sequencer tasks (sdk/thread/thread_app.c registers the OT ones)
 * ============================================================ */

enum {
    CFG_TASK_HW_RNG = 0,
    CFG_TASK_LINK_LAYER,
    CFG_TASK_FLASH_MANAGER,
    CFG_TASK_OT_ALARM,           /* otPlatAlarmMilliFired */
    CFG_TASK_OT_US_ALARM,        /* otPlatAlarmMicroFired */
    CFG_TASK_OT_TASKLETS,        /* otTaskletsProcess */
    CFG_TASK_OT_SETTINGS,        /* write the settings buffer to flash */
    CFG_TASK_NBR                 /* must be last */
};

enum {
    CFG_SEQ_PRIO_0 = 0,
    CFG_SEQ_PRIO_1,
    CFG_SEQ_PRIO_NBR             /* must be last */
};

#define UTIL_SEQ_CONF_PRIO_NBR    CFG_SEQ_PRIO_NBR

/* Priorities, as ST's Thread applications (stm32_rtos.h) */
#define TASK_PRIO_LINK_LAYER      CFG_SEQ_PRIO_0
#define TASK_PRIO_TASKLETS        CFG_SEQ_PRIO_0
#define TASK_PRIO_ALARM           CFG_SEQ_PRIO_1
#define TASK_PRIO_US_ALARM        CFG_SEQ_PRIO_1
#define TASK_PRIO_SETTINGS        CFG_SEQ_PRIO_1

/* Sequencer idle events */
enum {
    CFG_IDLEEVT_FLASH_OP = 0,    /* wpan_flash.c: a flash manager operation finished */
    CFG_IDLEEVT_NBR
};
#define UTIL_SEQ_CONF_EVTNBR      CFG_IDLEEVT_NBR

/* ============================================================
 * Radio
 * ============================================================ */

#define CFG_RF_TX_POWER_TABLE_ID  0
#define CFG_RADIO_LSE_SLEEP_TIMER_CUSTOM_SCA_RANGE  0

/* ============================================================
 * Low power manager ids (not used: CFG_LPM_LEVEL 0)
 * ============================================================ */

enum {
    CFG_LPM_APP = 0,
    CFG_LPM_LOG,
    CFG_LPM_LL_DEEPSLEEP,
    CFG_LPM_LL_HW_RCO_CLBR,
};

#define RCC_INTR_PRIO   1

#ifndef WPAN_SUCCESS
#define WPAN_SUCCESS  0
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_CONF_H */
