/**
 * core_timer.h — Timer subsystem: timebase, PWM, input capture, periodic tick
 *
 * The timer is the fundamental object. It has a frequency (shared
 * timebase), and its channels can independently do PWM output,
 * input capture, or nothing. The update interrupt (tick) is
 * orthogonal — it piggybacks on the same timebase.
 *
 * The DSL-facing `Core.Timer.tick` event fires at a fixed period set
 * by config.json's `timer.tick_ms`. Coregen bakes the period into a
 * main-loop dispatcher that polls `hal_tick()` — no hardware timer
 * consumed, no ISR context to manage. When no `tick_ms` is set the
 * dispatcher is a no-op stub.
 *
 * @studio category timer label=Core.Timer icon=◷
 * @studio event name=tick
 *
 * @studio coverage
 *   id:    timer
 *   name:  Timer — timebase, capture, periodic tick
 *   page:  /docs/sdk/timers
 *   blurb: Timer-instance API: init at frequency or tick rate, start /
 *          stop, set frequency, configure input capture, read captured
 *          values, and enable a per-overflow ISR callback. The DSL-
 *          facing `Core.Timer.tick` event is wired separately through
 *          a coregen-baked main-loop dispatcher (`hal_tick()`-driven,
 *          no hardware timer consumed). Tier 1 only — the explicit-
 *          handle surface lives below the DSL line.
 *
 * Typical usage:
 *
 *   #include "core_timer.h"
 *
 *   core_timer_t tim1;
 *   core_timer_init_freq(&tim1, TIM1, 1000);   // 1 kHz overflow
 *   core_timer_pwm_set(&tim1, 1, 500);        // CH1 = 50% PWM (permil)
 *   core_timer_capture_init(&tim1, 3);         // CH3 = capture
 *   core_timer_start(&tim1);
 *
 *   while (1) {
 *       core_timer_pwm_set(&tim1, 1, new_duty);
 *       uint32_t cap = core_timer_capture_read(&tim1, 3);
 *   }
 */

#ifndef CORE_TIMER_H
#define CORE_TIMER_H

#include "hal_timer.h"
#include "core_types.h"
#include "port_core_timer_types.h"   /* core_timer_instance_t */

/** Core-level timer handle. Alias for hal_timer_t — use this in application code. */
typedef hal_timer_t core_timer_t;

/* Defined in core_types.h; repeated (an identical typedef) so the timer
 * reference documents it. */
/** Core-level callback type: `void cb(void *ctx)`. */
typedef void (*core_callback_t)(void *ctx);

/* ============================================================
 * Timer timebase
 * ============================================================ */

/**
 * Initialize a timer at a given overflow frequency.
 * Use for PWM output and periodic tick — the frequency is how
 * often the counter wraps (= the PWM frequency).
 *
 *   core_timer_init_freq(&t, TIM2, 1000);   // overflows at 1 kHz
 */
CORE_INLINE core_status_t core_timer_init_freq(core_timer_t *h,
                                                core_timer_instance_t *instance,
                                                uint32_t freq_hz);

/**
 * Route an internal timer event to TRGO so another peripheral can be
 * paced by it, with no pin, no channel and no interrupt involved.
 *
 * LL_TIM_MMS_UPDATE is the periodic-pacer case: every counter overflow
 * emits a trigger. Pair it with core_adc_set_trigger() to run the ADC at
 * an exact rate instead of free-running.
 *
 *   core_timer_init_freq(&t, TIM2, 800);
 *   core_timer_set_trgo(&t, LL_TIM_MMS_UPDATE);
 *   core_timer_start(&t);
 */
CORE_INLINE void core_timer_set_trgo(core_timer_t *h, uint32_t mms);

/**
 * Initialize a timer at a given tick rate, free-running to max count.
 * Use for input capture — the tick rate sets the measurement
 * resolution, and the counter runs as long as possible before
 * wrapping (0xFFFF for 16-bit, 0xFFFFFFFF for 32-bit TIM2).
 *
 *   core_timer_init_tick(&t, TIM2, 1000000);  // 1 us per tick
 *   core_timer_capture_init(&t, 1);
 *   core_timer_start(&t);
 */
CORE_INLINE core_status_t core_timer_init_tick(core_timer_t *h,
                                                core_timer_instance_t *instance,
                                                uint32_t tick_hz);

/* Backward compat */
#define core_timer_init          core_timer_init_freq
#define core_timer_init_freerun  core_timer_init_tick

/** Start the timer counter. */
CORE_INLINE void core_timer_start(core_timer_t *h);

/** Stop the timer counter. */
CORE_INLINE void core_timer_stop(core_timer_t *h);

/** Change the timer frequency (recalculates PSC/ARR). */
CORE_INLINE void core_timer_set_freq(core_timer_t *h, uint32_t freq_hz);

/* ============================================================
 * PWM output (per-channel)
 * ============================================================ */

/**
 * Set PWM duty cycle for a channel.
 *   channel:      1–4
 *   duty_permil:  0–1000 (0 = off, 500 = 50%, 1000 = always on)
 */
CORE_INLINE void core_timer_pwm_set(core_timer_t *h, uint8_t channel,
                                     uint16_t duty_permil);

#if __has_include("core_pads.h")
#include "core_pads.h"
#ifdef CORE_USES_TIMER_PADS
/**
 * Set PWM duty cycle by pad number (requires coregen: only defined when the
 * project's config.json binds a timer to a pad). Resolves the channel from
 * the pad's timer assignment; pads without a timer are ignored.
 *
 * @param h           Timer handle the pad belongs to.
 * @param pad         Tile pad number.
 * @param duty_permil Duty cycle, 0 to 1000 (0 = off, 500 = 50%, 1000 = always on).
 */
CORE_INLINE void core_timer_pwm_set_pad(core_timer_t *h, uint8_t pad,
                                         uint16_t duty_permil);
#endif
#endif

/* ============================================================
 * Input capture (per-channel)
 * ============================================================ */

/**
 * Configure a channel for input capture (rising edge).
 * The timer must already be initialized with core_timer_init().
 */
CORE_INLINE void core_timer_capture_init(core_timer_t *h, uint8_t channel);

/** Read the last captured value from a channel. */
CORE_INLINE uint32_t core_timer_capture_read(core_timer_t *h, uint8_t channel);

/* ============================================================
 * Periodic tick (ISR callback at the timer frequency)
 * ============================================================ */

/**
 * Enable periodic tick on an already-initialized timer.
 * Does NOT touch PSC/ARR — the timer keeps its existing timebase.
 * The callback fires on each counter overflow at the timer's frequency.
 *
 * Typical pattern:
 *   core_timer_init(&t, TIM2, 1000);         // 1 kHz timebase
 *   core_timer_pwm_set(&t, 1, 500);          // CH1 = 50% PWM (permil)
 *   core_timer_enable_tick(&t, on_tick, NULL); // also fire ISR at 1 kHz
 *   core_timer_start(&t);
 */
CORE_INLINE core_status_t core_timer_enable_tick(core_timer_t *h,
                                                  core_callback_t cb, void *ctx);

/** Disable the tick callback (clears UIE, keeps timer running). */
CORE_INLINE void core_timer_disable_tick(core_timer_t *h);

/**
 * Convenience: initialize a timer as a tick-only source.
 * Sets up the timebase AND enables the update interrupt.
 * Use this when the timer's only job is a periodic callback.
 *
 *   core_tick_init(&t, TIM3, 500000, on_tick, NULL);  // 2 Hz
 *   core_timer_start(&t);
 */
CORE_INLINE core_status_t core_tick_init(core_timer_t *h,
                                          core_timer_instance_t *instance,
                                          uint32_t period_us,
                                          core_callback_t cb, void *ctx);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="Tick event period is set in config.json, not from DSL"
//   The DSL's `Core.Timer.tick` event fires at the period baked into
//   coregen from config.json's timer.tick_ms. There's no
//   `Core.Timer.set_tick_ms()` host call to retune at runtime — the
//   period is fixed for the life of the program.
//
// @studio unsupported tier=2 value=M title="No DSL access to per-channel capture / PWM"
//   core_timer_capture_init / capture_read / pwm_set are Tier 1 — they
//   take a core_timer_t* the DSL can't construct. PWM has its own
//   default-instance helper (Core.PWM.duty); capture / measurement has
//   no DSL surface.
//
// @studio unsupported tier=1 value=M title="No encoder / quadrature mode"
//   The hardware supports incremental encoder mode (CH1+CH2 quadrature
//   counting). The wrapper exposes neither encoder init nor read.
//   Motion / rotary-encoder tiles can't use it without dropping into
//   ll_tim.
//
// @studio unsupported tier=1 value=L title="No one-pulse mode (OPM)"
//   STM32 timers can fire a single pulse of programmable width and
//   then auto-disable — useful for camera trigger / strobe / ultrasound
//   ranging. Not wrapped.

/* The port's bodies of the functions above. */
#include "port_core_timer.h"

#endif /* CORE_TIMER_H */
