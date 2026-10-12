/**
 * thread_app.h — internal interface of the SDK's OpenThread integration
 *
 * Used by sdk/core/core_thread.c and the sdk/thread/ glue files. Applications
 * use core_thread.h instead.
 */

#ifndef THREAD_APP_H
#define THREAD_APP_H

#include <stdint.h>
#include <openthread/instance.h>

/* ---- thread_app.c ---- */

/** Bring up the radio, the sequencer and the OpenThread instance (once). */
otInstance *thread_app_init(void);

/** The OpenThread instance, or NULL before thread_app_init(). */
otInstance *thread_app_instance(void);

/** Run the sequencer once: pending radio / OpenThread tasks, then WFI. */
void thread_app_process(void);

/** Nonzero inside a sequencer pass (thread_app_process, or core_ble's in BLE + Thread builds). */
extern volatile uint8_t thread_app_seq_depth;

/** Schedule the alarm tasks (called from the link layer's timer callbacks). */
void thread_app_schedule_alarm(void);
void thread_app_schedule_us_alarm(void);

/* Reasons for thread_app_fatal(). */
enum {
    THREAD_FATAL_OT_ASSERT = 1,   /* otPlatAssertFail: arg = line */
    THREAD_FATAL_LIB_ASSERT,      /* __aeabi_assert in a prebuilt library: arg = line */
    THREAD_FATAL_TIMER,           /* os_timer_start refused: arg = which */
    THREAD_FATAL_CRYPTO,          /* AES self-test failed at init */
};

/**
 * Unrecoverable platform error: recorded in thread_fatal_info (RAM, survives
 * a warm reset in .noinit), then the CPU halts in a loop so a debugger (or the
 * watchdog, if running) finds it.
 */
void thread_app_fatal(uint32_t reason, uint32_t arg, const char *file);

/** Last fatal error, kept across a warm reset: magic, reason, arg. */
typedef struct {
    uint32_t magic;     /* THREAD_FATAL_MAGIC when valid */
    uint32_t reason;
    uint32_t arg;
} thread_fatal_info_t;
#define THREAD_FATAL_MAGIC  0x46415441UL   /* "FATA" */
extern thread_fatal_info_t thread_fatal_info;

/* ---- thread_alarm.c ---- */
void thread_alarm_init(void);
void thread_alarm_process(otInstance *aInstance);
void thread_alarm_us_process(otInstance *aInstance);

/* ---- thread_settings.c ---- */

/** Load the newest settings snapshot from flash into RAM (before otInstanceInit). */
void thread_settings_load(void);

/** Sequencer task: write the settings to flash if they changed. */
void thread_settings_process(void);

/** Diagnostics: snapshots written since boot, and the last flash error (0 = none). */
uint32_t thread_settings_save_count(void);
int thread_settings_last_error(void);

/* ---- thread_crypto_hal.c ---- */

/** FIPS-197 AES-128 encrypt + decrypt through the HAL shim: 0 pass, -1 fail. */
int thread_crypto_selftest(void);

#endif /* THREAD_APP_H */
