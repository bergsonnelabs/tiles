/**
 * thread_alarm.c — OpenThread millisecond / microsecond alarms
 *
 * On the link layer's own timer service (os_timer_*, LinkLayer_Thread_lib.a),
 * which runs on the radio sleep timer, and with otPlatRadioGetNow() as the
 * clock, exactly as ST's Thread applications do. The alternative, the SDK's
 * timer server (stm32_timer on LPTIM2 + SysTick), was not used: it is a second
 * time base next to the one the radio and OpenThread already share, with only
 * millisecond resolution, and it is not what ST validated these libraries with.
 *
 * A timer expiry only schedules a sequencer task; OpenThread's handlers run
 * from the task (thread_app.c), never from interrupt context.
 *
 * Adapted from STM32CubeWBA V1.6.1
 * Middlewares/ST/STM32_WPAN/thread/openthread/platform/alarm.c
 * Copyright (c) 2023 STMicroelectronics. All rights reserved.
 * Licensed under the terms in sdk/thread/LICENSE-ST.txt.
 * tiles: single instance only, no weak stubs, our task hooks.
 */

#include <stdint.h>
#include <openthread/instance.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/radio.h>
#include "os_wrapper.h"        /* LL: os_timer_* */
#include "linklayer_plat.h"    /* LINKLAYER_PLAT_Disable/EnableIRQ */
#include "thread_app.h"

extern void ral_set_ot_base_slp_time_value(uint32_t time);   /* LL (ral.h) */
extern uint32_t get_current_time(void);                      /* LL */

#define US_TO_MS(us)          ((us) / 1000u)
#define MS_TO_SLP_STEP(ms)    ((ms) * 32u)   /* sleep timer step: 31.25 us */
#define MILLI_START_THRESHOLD 0u             /* ms: fire now at or below this */
#define MICRO_START_THRESHOLD 100u           /* us */

typedef struct {
    uint32_t expires;
    uint8_t  running;          /* 1 armed, 2 parked (> 37 h away), 0 off */
} alarm_t;

static alarm_t s_milli, s_micro;
static os_timer_id s_milli_timer, s_micro_timer;

/* otPlatRadioGetNow() ignores its instance argument but the LL asserts it is
 * non-NULL (ST passes the address of a dummy). */
static uint32_t s_not_null;
#define ANY_INSTANCE ((otInstance *)(void *)&s_not_null)

static void milli_timer_cb(const void *arg)
{
    (void)arg;
    thread_app_schedule_alarm();
}

static void micro_timer_cb(const void *arg)
{
    (void)arg;
    thread_app_schedule_us_alarm();
}

void thread_alarm_init(void)
{
    if (s_milli_timer == NULL)
        s_milli_timer = os_timer_create(milli_timer_cb, os_timer_once, NULL);
    if (s_micro_timer == NULL)
        s_micro_timer = os_timer_create(micro_timer_cb, os_timer_once, NULL);
    s_milli.running = 0;
    s_micro.running = 0;
    ral_set_ot_base_slp_time_value(get_current_time());
}

uint64_t otPlatTimeGet(void)
{
    return otPlatRadioGetNow(ANY_INSTANCE);
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return (uint32_t)US_TO_MS(otPlatRadioGetNow(ANY_INSTANCE));
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;
    uint32_t remaining;

    LINKLAYER_PLAT_DisableIRQ();

    s_milli.expires = aT0 + aDt;
    s_milli.running = 1;
    os_timer_stop(s_milli_timer);

    uint32_t now = otPlatAlarmMilliGetNow();
    remaining = (now > aT0 + aDt) ? 0u : (s_milli.expires - now);
    if (aT0 > (UINT32_MAX - aDt) && now >= aT0)   /* aT0 + aDt wrapped, now has not */
        remaining = aDt - (now - aT0);

    if (remaining <= MILLI_START_THRESHOLD) {
        thread_app_schedule_alarm();
    } else {
        uint64_t steps = MS_TO_SLP_STEP((uint64_t)remaining);
        if (steps > UINT32_MAX) {
            /* > 37 h: park it; OpenThread re-arms long before that. */
            s_milli.expires = UINT32_MAX;
            s_milli.running = 2;
        } else if (os_timer_start(s_milli_timer, (uint32_t)steps) != 0) {
            thread_app_fatal(THREAD_FATAL_TIMER, 1, NULL);
        }
    }

    LINKLAYER_PLAT_EnableIRQ();
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    (void)aInstance;
    s_milli.running = 0;
    os_timer_stop(s_milli_timer);
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return (uint32_t)otPlatRadioGetNow(ANY_INSTANCE);
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;
    uint32_t remaining;

    s_micro.running = 1;
    s_micro.expires = aT0 + aDt;

    LINKLAYER_PLAT_DisableIRQ();

    uint32_t now = otPlatAlarmMicroGetNow();
    os_timer_stop(s_micro_timer);
    remaining = (now > aT0 + aDt) ? 0u : (s_micro.expires - now);
    if (aT0 > (UINT32_MAX - aDt) && now >= aT0)
        remaining = aDt - (now - aT0);

    if (remaining <= MICRO_START_THRESHOLD) {
        thread_app_schedule_us_alarm();
    } else if (os_timer_start_in_us(s_micro_timer, remaining) != 0) {
        thread_app_fatal(THREAD_FATAL_TIMER, 2, NULL);
    }

    LINKLAYER_PLAT_EnableIRQ();
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
    (void)aInstance;
    s_micro.running = 0;
    os_timer_stop(s_micro_timer);
}

void thread_alarm_process(otInstance *aInstance)
{
    if (s_milli.running == 1) {
        s_milli.running = 0;
        otPlatAlarmMilliFired(aInstance);
    }
}

void thread_alarm_us_process(otInstance *aInstance)
{
    if (s_micro.running == 1) {
        s_micro.running = 0;
        otPlatAlarmMicroFired(aInstance);
    }
}
