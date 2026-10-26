/**
 * port_core_pwm.h — STM32 port: the bodies of core_pwm.h
 *
 * Included at the end of core_pwm.h, after its prototypes; not for direct
 * use. The timer types and SYSCLK_HZ come from core_timer.h and its port
 * header (port_core_timer.h).
 */

#ifndef PORT_CORE_PWM_H
#define PORT_CORE_PWM_H

#include "port_core_timer.h"

static inline core_status_t core_pwm_init(core_timer_t *h, core_timer_instance_t *instance,
                                           uint32_t freq_hz)
{
    return hal_timer_pwm_init(h, instance, SYSCLK_HZ, freq_hz);
}

static inline core_status_t core_pwm_init_clk(core_timer_t *h, core_timer_instance_t *instance,
                                               uint32_t pclk_hz, uint32_t freq_hz)
{
    return hal_timer_pwm_init(h, instance, pclk_hz, freq_hz);
}

static inline void core_pwm_set(core_timer_t *h, uint8_t channel,
                                 uint16_t duty_permil)
{
    hal_timer_pwm_set_duty(h, channel, duty_permil);
}

static inline void core_pwm_set_freq(core_timer_t *h, uint32_t freq_hz)
{
    hal_timer_pwm_set_freq(h, freq_hz);
}

static inline void core_pwm_start(core_timer_t *h)
{
    hal_timer_pwm_start(h);
}

static inline void core_pwm_stop(core_timer_t *h)
{
    hal_timer_pwm_stop(h);
}

#if __has_include("core_pads.h") && __has_include("core_config.h")
#include "tal_timer.h"

static inline core_status_t core_pwm_init_pad(core_timer_t *h, uint8_t pad,
                                               uint32_t freq_hz)
{
    return tal_pwm_init(h, pad, freq_hz);
}

static inline void core_pwm_set_pad(core_timer_t *h, uint8_t pad,
                                     uint16_t duty_permil)
{
    tal_pwm_set(h, pad, duty_permil);
}
#endif

static inline void core_pwm_duty(uint8_t pad, uint16_t duty_permil)
{
    hal_timer_t *h = core_pwm_timer_for_pad(pad);
    if (h) tal_pwm_set(h, pad, duty_permil);
}

static inline core_status_t core_every_us(core_timer_t *h, core_timer_instance_t *instance,
                                           uint32_t period_us,
                                           core_callback_t cb, void *ctx)
{
    return hal_timer_tick_init(h, instance, SYSCLK_HZ, period_us, cb, ctx);
}

static inline core_status_t core_every_us_clk(core_timer_t *h, core_timer_instance_t *instance,
                                               uint32_t pclk_hz, uint32_t period_us,
                                               core_callback_t cb, void *ctx)
{
    return hal_timer_tick_init(h, instance, pclk_hz, period_us, cb, ctx);
}

static inline void core_every_start(core_timer_t *h)
{
    hal_timer_tick_start(h);
}

static inline void core_every_stop(core_timer_t *h)
{
    hal_timer_tick_stop(h);
}

#endif /* PORT_CORE_PWM_H */
