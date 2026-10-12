/**
 * thread_app.c — OpenThread bring-up on Core.ST.W5 (bare metal, sequencer)
 *
 * The Thread counterpart of sdk/ble/ble_app.c: the radio clocks, the
 * sequencer, the flash manager and the OpenThread instance, in the order ST's
 * Thread applications use (STM32CubeWBA V1.6.1 Projects/NUCLEO-WBA55CG/
 * Applications/Thread/Thread_Udp: app_entry.c MX_APPE_Init, app_thread.c
 * Thread_Init). Everything OpenThread does runs from sequencer tasks:
 *
 *   CFG_TASK_LINK_LAYER   the link layer's background process (ll_sys_if.c)
 *   CFG_TASK_OT_TASKLETS  otTaskletsProcess (OpenThread's own work queue)
 *   CFG_TASK_OT_ALARM     otPlatAlarmMilliFired (thread_alarm.c timers)
 *   CFG_TASK_OT_US_ALARM  otPlatAlarmMicroFired
 *   CFG_TASK_FLASH_MANAGER / CFG_TASK_OT_SETTINGS  settings to flash
 *
 * core_thread_process() runs them: UTIL_SEQ_Run, then WFI until the next
 * interrupt (SysTick wakes it at least every millisecond).
 *
 * The radio clock setup duplicates ble_app_init()'s steps 0-2 on purpose:
 * moving them into a shared function would change the BLE binaries, which
 * stay byte-identical.
 *
 * BLE + Thread builds (BLE_ENABLED and THREAD_ENABLED) bring the radio up in
 * the order of ST's BLE_HeartRate_Thread (STM32CubeWBA V1.6.1, app_entry.c
 * MX_APPE_Init): APP_BLE_Init, the flash manager's RFTS bypass off, then
 * APP_THREAD_Init. Here: core_ble_init() (ble_app_init: radio clocks, RNG,
 * sequencer, timer server, flash manager, the link layer through
 * ll_sys_ble_cntrl_init, the BLE stack, wpan_flash_ll_started), then
 * core_thread_start(), which skips everything ble_app_init() already did
 * (UTIL_SEQ_Init / UTIL_TIMER_Init again would drop BLE's tasks and timers)
 * and starts the 802.15.4 side (otSysInit: radio_init) and OpenThread on the
 * same sequencer. ll_sys_thread_init() is then a no-op (ll_sys_startup.c
 * initializes the link layer's resources once). Thread's settings writes go
 * through the flash manager's radio-free windows, like BLE's.
 */

#include <stdint.h>
#include <stddef.h>
#include "core_config.h"

/* The 2.4 GHz radio needs voltage range 1 (RM0493 §12.4.6); "low" runs in
 * range 2. coregen refuses the combination; this catches a forced build. */
#if defined(SYSCLK_HZ) && (SYSCLK_HZ < 32000000UL)
#error "Thread needs clock medium or higher (the radio needs voltage range 1; clock low is HSI16 in range 2)"
#endif

#include "ll_common.h"
#include "ll_rcc.h"
#include "ll_pwr.h"
#include "ll_rtc.h"
#include "app_conf.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "wpan_flash.h"

#include <openthread/instance.h>
#include <openthread/tasklet.h>
#include "thread_app.h"

extern void ll_sys_thread_init(void);                       /* ll_sys_startup.c */
extern void otSysInit(int argc, char *argv[]);              /* thread_platform.c */
extern void otDispatch_tbl_init(otInstance *sInstance);     /* thread_platform.c */

static otInstance *s_instance;

/* Survives a warm reset (.noinit): what stopped the last run, if anything. */
thread_fatal_info_t thread_fatal_info __attribute__((section(".noinit")));

/* ============================================================
 * Fatal errors
 * ============================================================ */

void thread_app_fatal(uint32_t reason, uint32_t arg, const char *file)
{
    (void)file;
    __asm volatile ("cpsid i" ::: "memory");
    thread_fatal_info.reason = reason;
    thread_fatal_info.arg = arg;
    thread_fatal_info.magic = THREAD_FATAL_MAGIC;
    for (;;)
        ;
}

#if !defined(BLE_ENABLED)   /* BLE + Thread: ble_app_init() does this */
/* ============================================================
 * HSE trim from OTP — as ble_app.c (ST's OTP record, CubeWBA otp.h)
 * ============================================================ */

#define OTP_AREA_BASE     0x0BF90000UL       /* 512 B (FLASH_OTP_BASE) */
#define OTP_AREA_SIZE     512u
#define RCC_ECSCR1        REG32(RCC_BASE + 0x210UL)   /* HSETRIM [21:16], RM0493 §12.8.53 */
#define HSE_TRIM_DEFAULT  0x0Cu

static void config_hse_tuning(void)
{
    uint8_t hsetune = HSE_TRIM_DEFAULT;
    /* The last 16-byte slot whose index byte is 0 holds the current trim. */
    for (uint32_t off = OTP_AREA_SIZE; off >= 16u; off -= 16u) {
        const volatile uint8_t *slot = (const volatile uint8_t *)(OTP_AREA_BASE + off - 16u);
        if (slot[15] == 0x00u && slot[14] <= 0x3Fu) {
            hsetune = slot[14];
            break;
        }
    }
    MOD_BITS(RCC_ECSCR1, 0x3FUL << 16, ((uint32_t)(hsetune & 0x3Fu)) << 16);
}
#endif /* !BLE_ENABLED */

/* ============================================================
 * Sequencer tasks
 * ============================================================ */

static void task_tasklets(void)
{
    otTaskletsProcess(s_instance);
    if (otTaskletsArePending(s_instance))
        UTIL_SEQ_SetTask(1U << CFG_TASK_OT_TASKLETS, TASK_PRIO_TASKLETS);
}

static void task_alarm(void)
{
    thread_alarm_process(s_instance);
}

static void task_us_alarm(void)
{
    thread_alarm_us_process(s_instance);
}

/* OpenThread: the tasklet queue went from empty to non-empty. */
void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
    UTIL_SEQ_SetTask(1U << CFG_TASK_OT_TASKLETS, TASK_PRIO_TASKLETS);
}

void thread_app_schedule_alarm(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_OT_ALARM, TASK_PRIO_ALARM);
}

void thread_app_schedule_us_alarm(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_OT_US_ALARM, TASK_PRIO_US_ALARM);
}

/* ============================================================
 * Init
 * ============================================================ */

#if !defined(BLE_ENABLED)   /* BLE + Thread: ble_app_init() steps 0-2 */
static void radio_clocks_init(void)
{
    /* Range 1 with hclk5 undivided by HDIV5 (RM0493 §12.4.6). core_clock_init
     * already set it for every level the radio builds at; only HDIV5 here. */
    if (ll_pwr_get_vos() == 1u)
        ll_rcc_set_hdiv5(0);

    /* Instruction cache, 1-way (as ST's applications and ble_app.c). */
    REG32(0x40030400UL) = (1UL << 2);                  /* ICACHE_CR: WAYSEL = 1-way */
    REG32(0x40030400UL) = (1UL << 2) | (1UL << 0);     /* + EN */

    config_hse_tuning();

    /* RNG kernel clock = HSI (CCIPR2 RNGSEL[13:12] = 0b10); HASH clock (its
     * IRQ is the link layer's SW low ISR) and RNG clock (AHB2ENR 17, 18). */
    MOD_BITS(REG32(RCC_BASE + 0xE4UL), 0x3UL << 12, 0x2UL << 12);
    SET_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << 17) | (1UL << 18));
    {
        volatile uint32_t *rng_cr = (volatile uint32_t *)(PERIPH_BASE + 0x020C0800UL);
        *rng_cr = 0;
        *rng_cr = (1UL << 2) | (1UL << 6);    /* RNGEN + CONDRST */
        *rng_cr = (1UL << 2);                 /* conditioning set */
    }

    /* The RTC is the SDK's (core_rtc); keep it, then the radio sleep timer on
     * LSI: Core.ST.W5-b has no 32 kHz crystal. ll_rtc_ensure() first, since a
     * backup-domain reset in it would clear RADIOSTSEL and LSI1. */
    ll_rtc_ensure();
    ll_pwr_enable_backup_access();
    ll_rcc_lsi1_enable_wait();
    ll_rcc_set_radio_sleep_clk(LL_RCC_RADIOSLEEPSOURCE_LSI);
}
#endif /* !BLE_ENABLED */

#if defined(BLE_ENABLED)
extern int ble_app_ready(void);                             /* sdk/ble/ble_app.c */
#endif

otInstance *thread_app_init(void)
{
    if (s_instance != NULL)
        return s_instance;

#if defined(BLE_ENABLED)
    /* BLE + Thread: BLE first, as ST does (see the top of this file). */
    if (!ble_app_ready())
        return NULL;
#else
    radio_clocks_init();
#endif

    /* The AES path Thread's MAC and MLE security use (thread_crypto_hal.c):
     * refuse to start on a wrong result rather than send garbage. */
    if (thread_crypto_selftest() != 0)
        thread_app_fatal(THREAD_FATAL_CRYPTO, 0, NULL);

#if !defined(BLE_ENABLED)
    UTIL_SEQ_Init();
    UTIL_TIMER_Init();
    wpan_flash_init();    /* Thread: the RFTS bypass stays on (see wpan_flash.h) */
#endif
    UTIL_SEQ_RegTask(1U << CFG_TASK_OT_SETTINGS, UTIL_SEQ_RFU, thread_settings_process);

    /* Settings into RAM before OpenThread reads them (otInstanceInit). */
    thread_settings_load();

    otSysInit(0, NULL);
    s_instance = otInstanceInitSingle();
    if (s_instance == NULL)
        return NULL;
    otDispatch_tbl_init(s_instance);

    UTIL_SEQ_RegTask(1U << CFG_TASK_OT_ALARM, UTIL_SEQ_RFU, task_alarm);
    UTIL_SEQ_RegTask(1U << CFG_TASK_OT_US_ALARM, UTIL_SEQ_RFU, task_us_alarm);
    UTIL_SEQ_RegTask(1U << CFG_TASK_OT_TASKLETS, UTIL_SEQ_RFU, task_tasklets);
    UTIL_SEQ_SetTask(1U << CFG_TASK_OT_ALARM, TASK_PRIO_ALARM);   /* first run */

    /* Deep-sleep stubs, the link layer's background task, its parameters. */
    ll_sys_thread_init();

    return s_instance;
}

otInstance *thread_app_instance(void)
{
    return s_instance;
}

/* Nonzero while a sequencer pass started by the SDK is running (this one, or
 * core_ble's in BLE + Thread builds): the tasks it runs (OpenThread, BLE, and
 * the application callbacks they call) must not be re-entered from a delay or
 * a watchdog feed inside one of them (core_thread_process()). */
volatile uint8_t thread_app_seq_depth;

void thread_app_process(void)
{
    thread_app_seq_depth++;
    UTIL_SEQ_Run(~0UL);
    thread_app_seq_depth--;
}
