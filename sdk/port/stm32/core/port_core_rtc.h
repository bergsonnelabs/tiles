/**
 * port_core_rtc.h — STM32 port: the bodies of core_rtc.h
 *
 * Included at the end of core_rtc.h, after its prototypes; not for direct
 * use. Thin wrappers over ll_rtc (LSI by default; ll_rtc_init(1) for LSE).
 */

#ifndef PORT_CORE_RTC_H
#define PORT_CORE_RTC_H

#include "ll_rtc.h"

static inline void core_rtc_init(void)
{
    ll_rtc_init(0);
}

static inline void core_rtc_set_time(uint8_t h, uint8_t m, uint8_t s)
{
    ll_rtc_set_time(h, m, s);
}

static inline void core_rtc_get_time(uint8_t *h, uint8_t *m, uint8_t *s)
{
    ll_rtc_get_time(h, m, s);
}

static inline void core_rtc_set_date(uint8_t y, uint8_t mo, uint8_t d,
                                      uint8_t wd)
{
    ll_rtc_set_date(y, mo, d, wd);
}

static inline void core_rtc_get_date(uint8_t *y, uint8_t *mo, uint8_t *d,
                                      uint8_t *wd)
{
    ll_rtc_get_date(y, mo, d, wd);
}

static inline void core_rtc_wakeup(uint32_t seconds)
{
    ll_rtc_wakeup_config(seconds);
}

static inline void core_rtc_wakeup_stop(void)
{
    ll_rtc_wakeup_disable();
}

static inline void core_rtc_set_alarm(uint8_t hours, uint8_t minutes,
                                       uint8_t seconds)
{
    ll_rtc_alarm_a_set(hours, minutes, seconds);
}

static inline void core_rtc_clear_alarm(void)
{
    ll_rtc_alarm_a_disable();
}

static inline int core_rtc_alarm_fired(void)
{
    if (ll_rtc_alarm_a_flag()) {
        ll_rtc_alarm_a_clear_flag();
        return 1;
    }
    return 0;
}

#endif /* PORT_CORE_RTC_H */
