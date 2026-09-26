/**
 * core_pwm.h — PWM output and periodic timers
 *
 * Three usage modes:
 *
 *   0. Project-centric (simplest — coregen owns handle + init):
 *        core_pwm_duty(7, 500);   // 50% duty on pad 7; timer resolved from config.json
 *
 *   1. Pad-centric (manual init):
 *        core_pwm_init_pad(&pwm, 7, 1000);
 *        core_pwm_set_pad(&pwm, 7, 500);
 *
 *   2. Explicit instance (no coregen needed):
 *        core_pwm_init(&pwm, TIM2, SYSCLK_HZ, 1000);
 *        core_pwm_set(&pwm, 3, 500);
 *
 * @studio category pwm label=Core.PWM icon=◲
 *
 * @studio coverage
 *   id:    pwm
 *   name:  PWM — pulse-width modulation
 *   page:  /docs/sdk/timers
 *   blurb: Three layered APIs: an explicit-instance core_pwm_init / set
 *          for hand-rolled C, a pad-centric core_pwm_init_pad that
 *          resolves the timer from coregen tables, and a project-centric
 *          core_pwm_duty (Tier 2 default-instance) that's the DSL's
 *          knob. The simulator renders pad output level live from
 *          duty_permil. Periodic-tick (`every_us`) is also wrapped here
 *          for ISR-driven callbacks.
 */

#ifndef CORE_PWM_H
#define CORE_PWM_H

#include "core_timer.h"

/* ============================================================
 * PWM output
 * ============================================================ */

/** Initialize a timer for PWM output at the given frequency.
 *  Clock is auto-resolved from SYSCLK_HZ (core_config.h).
 */
static inline hal_status_t core_pwm_init(core_timer_t *h, TIM_TypeDef *instance,
                                          uint32_t freq_hz)
{
    return hal_timer_pwm_init(h, instance, SYSCLK_HZ, freq_hz);
}

/** @deprecated Use core_pwm_init(h, instance, freq_hz) — clock auto-resolved. */
static inline hal_status_t core_pwm_init_clk(core_timer_t *h, TIM_TypeDef *instance,
                                              uint32_t pclk_hz, uint32_t freq_hz)
{
    return hal_timer_pwm_init(h, instance, pclk_hz, freq_hz);
}

/**
 * Set PWM duty cycle for a channel.
 *   channel:      1–4
 *   duty_permil:  0–1000 (0 = off, 500 = 50%, 1000 = 100%)
 */
static inline void core_pwm_set(core_timer_t *h, uint8_t channel,
                                 uint16_t duty_permil)
{
    hal_timer_pwm_set_duty(h, channel, duty_permil);
}

/** Change PWM frequency (recalculates PSC/ARR, resets all channel duties). */
static inline void core_pwm_set_freq(core_timer_t *h, uint32_t freq_hz)
{
    hal_timer_pwm_set_freq(h, freq_hz);
}

/** Start the PWM timer. */
static inline void core_pwm_start(core_timer_t *h)
{
    hal_timer_pwm_start(h);
}

/** Stop the PWM timer. */
static inline void core_pwm_stop(core_timer_t *h)
{
    hal_timer_pwm_stop(h);
}

/* ============================================================
 * PWM output — pad-centric (requires coregen)
 *
 * These wrap tal_timer.h and auto-resolve the timer instance and
 * channel from the pad number using coregen-generated mappings.
 * Only available when core_pads.h provides core_pad_timer_info().
 * ============================================================ */

#if __has_include("core_pads.h") && __has_include("core_config.h")
#include "tal_timer.h"

/** Initialize PWM on a pad. Timer instance resolved from config.json. */
static inline hal_status_t core_pwm_init_pad(core_timer_t *h, uint8_t pad,
                                              uint32_t freq_hz)
{
    return tal_pwm_init(h, pad, freq_hz);
}

/** Set PWM duty on a pad (0–1000 permil). */
static inline void core_pwm_set_pad(core_timer_t *h, uint8_t pad,
                                     uint16_t duty_permil)
{
    tal_pwm_set(h, pad, duty_permil);
}
#endif

/* ============================================================
 * Project-centric PWM (default-instance convention)
 * ============================================================
 *
 * Coregen emits one `core_tim<n>` handle per timer declared in
 * config.json and a `core_pwm_timer_for_pad(pad)` dispatch function.
 * These wrappers use the dispatch to set duty cycles without the
 * caller having to pick a timer. Timers are initialized and started
 * automatically during `core_init()`.
 *
 * If the pad is not bound to a timer in config.json, the wrapper
 * silently does nothing — DSL code running on the wrong project
 * won't fault, it just won't produce output.
 */

/**
 * The timer driving a pad. Emitted per project by coregen into core_init.c
 * when any TIM<n> pad is configured.
 *
 * @param pad Tile pad number.
 * @return The timer handle, or NULL for pads not bound to a timer in config.json.
 */
hal_timer_t *core_pwm_timer_for_pad(uint8_t pad);

/**
 * Set PWM duty cycle on a pad. Timer handle resolved from config.json.
 *
 * @studio expose category=pwm name=duty
 * @studio twin full
 * @param pad [1..64] Tile pad number configured as a TIMx.<ch> in config.json.
 * @param duty_permil [0..1000] 0 = off, 500 = 50%, 1000 = fully on.
 */
static inline void core_pwm_duty(uint8_t pad, uint16_t duty_permil)
{
    hal_timer_t *h = core_pwm_timer_for_pad(pad);
    if (h) tal_pwm_set(h, pad, duty_permil);
}

/* ============================================================
 * Periodic tick ("every")
 * ============================================================ */

/**
 * Configure a periodic callback at a fixed interval.
 *   period_us: interval in microseconds (1 – 1000000)
 *   cb:        callback function (called from ISR context!)
 *   ctx:       user context passed to callback
 *
 * Clock is auto-resolved from SYSCLK_HZ (core_config.h).
 */
static inline hal_status_t core_every_us(core_timer_t *h, TIM_TypeDef *instance,
                                          uint32_t period_us,
                                          hal_callback_t cb, void *ctx)
{
    return hal_timer_tick_init(h, instance, SYSCLK_HZ, period_us, cb, ctx);
}

/** @deprecated Use core_every_us(h, instance, period_us, cb, ctx) — clock auto-resolved. */
static inline hal_status_t core_every_us_clk(core_timer_t *h, TIM_TypeDef *instance,
                                              uint32_t pclk_hz, uint32_t period_us,
                                              hal_callback_t cb, void *ctx)
{
    return hal_timer_tick_init(h, instance, pclk_hz, period_us, cb, ctx);
}

/** Start the periodic timer. */
static inline void core_every_start(core_timer_t *h)
{
    hal_timer_tick_start(h);
}

/** Stop the periodic timer. */
static inline void core_every_stop(core_timer_t *h)
{
    hal_timer_tick_stop(h);
}

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No DSL access to PWM frequency"
//   Tier 2 only exposes duty. Frequency is fixed at coregen time from
//   config.json's timer.freq_hz. DSL programs that want to sweep
//   frequency (e.g., a buzzer playing notes) need set_freq exposed
//   per-pad — the underlying tal_pwm has the path, but the wrapper +
//   default-instance dispatch don't.
//
// @studio unsupported tier=2 value=M title="No DSL access to periodic 'every' callbacks"
//   core_every_us is Tier 1 only — it takes a C function pointer for
//   the ISR callback. The DSL surface is `Core.Timer.tick` (in
//   core_timer.h) which fires at config.json's timer.tick_ms; finer
//   custom periods aren't reachable.
//
// @studio unsupported tier=1 value=M title="No complementary PWM / dead-time (TIM1 advanced)"
//   SDK roadmap Tier 2 item: TIM1's complementary outputs + dead-time
//   insertion + break input are needed for half-bridge / motor /
//   power-converter tiles. The PWM wrapper is single-channel only.
//
// @studio unsupported tier=1 value=L title="No phase / center-aligned modes"
//   Edge-aligned PWM only. STM32 timers can do center-aligned (Mode 1
//   / 2 / 3) and per-channel phase offsets — useful for low-EMI
//   switching. Not surfaced.

#endif /* CORE_PWM_H */
