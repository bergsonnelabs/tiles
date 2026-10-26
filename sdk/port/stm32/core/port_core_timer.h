/**
 * port_core_timer.h — STM32 port: the bodies of core_timer.h
 *
 * Included at the end of core_timer.h, after its prototypes; not for direct
 * use. The types are in port_core_timer_types.h.
 */

#ifndef PORT_CORE_TIMER_H
#define PORT_CORE_TIMER_H

#include "hal_timer.h"
#include "ll_tim.h"
#include "port_core_timer_types.h"

#if __has_include("core_config.h")
#include "core_config.h"  /* SYSCLK_HZ */
#else
#ifndef SYSCLK_HZ
#error "SYSCLK_HZ not defined — include core_config.h or define it manually"
#endif
#endif

static inline core_status_t core_timer_init_freq(core_timer_t *h,
                                                  core_timer_instance_t *instance,
                                                  uint32_t freq_hz)
{
    return hal_timer_pwm_init(h, instance, SYSCLK_HZ, freq_hz);
}

static inline void core_timer_set_trgo(core_timer_t *h, uint32_t mms)
{
    ll_tim_set_trgo(h->instance, mms);
}

static inline core_status_t core_timer_init_tick(core_timer_t *h,
                                                  core_timer_instance_t *instance,
                                                  uint32_t tick_hz)
{
    hal_status_t rc = hal_timer_pwm_init(h, instance, SYSCLK_HZ, tick_hz);
    uint32_t psc = (SYSCLK_HZ / tick_hz) - 1;
    h->instance->PSC = psc;
    h->instance->ARR = 0xFFFFFFFF;  /* 16-bit timers see 0xFFFF, 32-bit see full range */
    h->instance->EGR = 1;           /* force reload */
    return rc;
}

static inline void core_timer_start(core_timer_t *h)
{
    hal_timer_pwm_start(h);
}

static inline void core_timer_stop(core_timer_t *h)
{
    hal_timer_pwm_stop(h);
}

static inline void core_timer_set_freq(core_timer_t *h, uint32_t freq_hz)
{
    hal_timer_pwm_set_freq(h, freq_hz);
}

static inline void core_timer_pwm_set(core_timer_t *h, uint8_t channel,
                                       uint16_t duty_permil)
{
    hal_timer_pwm_set_duty(h, channel, duty_permil);
}

#if __has_include("core_pads.h")
#ifdef CORE_USES_TIMER_PADS
static inline void core_timer_pwm_set_pad(core_timer_t *h, uint8_t pad,
                                           uint16_t duty_permil)
{
    core_timer_instance_t *inst;
    uint8_t ch;
    if (core_pad_timer_info(pad, &inst, &ch) == 0) {
        hal_timer_pwm_set_duty(h, ch, duty_permil);
    }
}
#endif
#endif

static inline void core_timer_capture_init(core_timer_t *h, uint8_t channel)
{
    ll_tim_ic_config(h->instance, channel);
}

static inline uint32_t core_timer_capture_read(core_timer_t *h, uint8_t channel)
{
    return ll_tim_ic_read(h->instance, channel);
}

static inline core_status_t core_timer_enable_tick(core_timer_t *h,
                                                    core_callback_t cb, void *ctx)
{
    return hal_timer_tick_enable(h, cb, ctx);
}

static inline void core_timer_disable_tick(core_timer_t *h)
{
    hal_timer_tick_disable(h);
}

static inline core_status_t core_tick_init(core_timer_t *h,
                                            core_timer_instance_t *instance,
                                            uint32_t period_us,
                                            core_callback_t cb, void *ctx)
{
    hal_status_t rc = hal_timer_tick_init(h, instance, SYSCLK_HZ, period_us, cb, ctx);
    return rc;
}

#endif /* PORT_CORE_TIMER_H */
