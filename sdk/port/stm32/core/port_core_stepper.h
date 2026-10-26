/**
 * port_core_stepper.h — STM32 port: the timer channel behind core_stepper.c
 *
 * Included by core_stepper.c only, where its channel helpers always sat (GCC
 * emits functions in definition order); not for direct use. core_stepper.c
 * keeps the profile, the planner and the ISR logic; this file has the timer
 * register work they need, nothing else:
 *
 *   port_core_stepper_output(h, mode)       STEP output: PORT_CORE_STEPPER_OUT_INACTIVE
 *                                           (forced off, takes effect at once) or
 *                                           PORT_CORE_STEPPER_OUT_PULSE (active while
 *                                           counter >= compare)
 *   port_core_stepper_compare_write/_read   the STEP channel's compare value
 *   port_core_stepper_channel_setup(h)      channel config: compare preload, polarity
 *                                           (h->step_active_low), output on
 *   port_core_stepper_period_write(t, n)    period of n ticks (preloaded)
 *   port_core_stepper_counter_clear/_read   the counter
 *   port_core_stepper_irq_off(t)            update interrupt off
 *   port_core_stepper_latch(t)              load the preloaded values now, clear the
 *                                           pending update, update interrupt on
 *   port_core_stepper_tick_1us(t)           count at 1 MHz
 *   port_core_stepper_is_32bit(t)           1 for a 32-bit counter (TIM2), 0 for 16
 *   port_core_stepper_lock/_unlock()        interrupts off / back as they were
 *
 * On STM32 the timer is a general-purpose or advanced TIMx and the STEP output
 * one of its channels 1..4 in PWM mode 2; ARR and CCRx are preloaded (ARPE,
 * OCxPE), so values written take effect at the next update event.
 */

#ifndef PORT_CORE_STEPPER_H
#define PORT_CORE_STEPPER_H

#include "core_stepper.h"
#include "core_config.h"   /* SYSCLK_HZ */
#include "ll_common.h"
#include "ll_tim.h"

/* Output-compare modes for port_core_stepper_output(). "Force inactive"
 * (OCxM = 100b) is not preloaded, so a halt can kill a pulse in progress. */
#define PORT_CORE_STEPPER_OUT_INACTIVE  0x4u
#define PORT_CORE_STEPPER_OUT_PULSE     LL_TIM_OCMODE_PWM2

static void port_core_stepper_output(core_stepper_t *h, uint32_t mode)
{
    TIM_TypeDef *t = h->timer.instance;
    switch (h->channel) {
    case 1: MOD_BITS(t->CCMR1, 0x7u << 4,  mode << 4);  break;
    case 2: MOD_BITS(t->CCMR1, 0x7u << 12, mode << 12); break;
    case 3: MOD_BITS(t->CCMR2, 0x7u << 4,  mode << 4);  break;
    case 4: MOD_BITS(t->CCMR2, 0x7u << 12, mode << 12); break;
    default: break;
    }
}

static inline void port_core_stepper_compare_write(core_stepper_t *h, uint32_t v)
{
    TIM_TypeDef *t = h->timer.instance;
    switch (h->channel) {
    case 1: t->CCR1 = v; break;
    case 2: t->CCR2 = v; break;
    case 3: t->CCR3 = v; break;
    case 4: t->CCR4 = v; break;
    default: break;
    }
}

static inline uint32_t port_core_stepper_compare_read(const core_stepper_t *h)
{
    TIM_TypeDef *t = h->timer.instance;
    switch (h->channel) {
    case 1: return t->CCR1;
    case 2: return t->CCR2;
    case 3: return t->CCR3;
    default: return t->CCR4;
    }
}

/* Configure the channel: PWM2, CCR preload, polarity, output enabled. */
static void port_core_stepper_channel_setup(core_stepper_t *h)
{
    TIM_TypeDef *t = h->timer.instance;
    uint32_t sh = (h->channel - 1) * 4;           /* CCER bits per channel */
    CLR_BITS(t->CCER, 0xFu << sh);
    switch (h->channel) {
    case 1: SET_BITS(t->CCMR1, 1u << 3);  break;  /* OC1PE */
    case 2: SET_BITS(t->CCMR1, 1u << 11); break;  /* OC2PE */
    case 3: SET_BITS(t->CCMR2, 1u << 3);  break;  /* OC3PE */
    case 4: SET_BITS(t->CCMR2, 1u << 11); break;  /* OC4PE */
    default: break;
    }
    port_core_stepper_output(h, PORT_CORE_STEPPER_OUT_INACTIVE);
    if (h->step_active_low) SET_BITS(t->CCER, 0x2u << sh);   /* CCxP */
    SET_BITS(t->CCER, 0x1u << sh);                           /* CCxE */
    SET_BITS(t->CR1, LL_TIM_CR1_ARPE);
    ll_tim_enable_moe(t);                                    /* TIM1/15/16 need it */
}

/* ARR: a period of `ticks` counter ticks. */
static inline void port_core_stepper_period_write(core_timer_instance_t *t, uint32_t ticks)
{
    t->ARR = ticks - 1;
}

static inline void port_core_stepper_counter_clear(core_timer_instance_t *t)
{
    t->CNT = 0;
}

/* A macro, not a function: core_stepper_halt() compares it with the compare
 * value, and only a plain read keeps that comparison the code it always was. */
#define port_core_stepper_counter_read(t)  ((t)->CNT)

static inline void port_core_stepper_irq_off(core_timer_instance_t *t)
{
    CLR_BITS(t->DIER, LL_TIM_DIER_UIE);
}

/* UG copies the preload registers (PSC, ARR, CCRx) into the active ones; the
 * UIF it sets is cleared before the update interrupt is turned back on. */
static inline void port_core_stepper_latch(core_timer_instance_t *t)
{
    t->EGR = LL_TIM_EGR_UG;
    t->SR  = 0;
    SET_BITS(t->DIER, LL_TIM_DIER_UIE);
}

static inline void port_core_stepper_tick_1us(core_timer_instance_t *t)
{
    t->PSC = (SYSCLK_HZ / 1000000UL) - 1;
}

/* Leaves ARR at its maximum; the caller sets the period next. */
static inline uint8_t port_core_stepper_is_32bit(core_timer_instance_t *t)
{
    t->ARR = 0xFFFFFFFFu;
    return (t->ARR == 0xFFFFFFFFu) ? 1 : 0;   /* 16-bit timers read back 0xFFFF */
}

static inline uint32_t port_core_stepper_lock(void)
{
    return ll_irq_save();
}

static inline void port_core_stepper_unlock(uint32_t s)
{
    ll_irq_restore(s);
}

#endif /* PORT_CORE_STEPPER_H */
