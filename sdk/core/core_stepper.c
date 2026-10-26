/**
 * core_stepper.c — STEP/DIR pulse generator
 *
 * One timer per axis, ticking at 1 us, with the STEP pad on one of its
 * output channels in PWM mode 2 ("active while CNT >= CCR"):
 *
 *      |<------------- interval (ARR+1) ------------->|
 *      |                                    |<-pulse->|
 *   ___|____________________________________|         |____
 *      ^ update: count the step that        ^ CCR     ^ update
 *        just ended, latch the next
 *        interval, plan the one after
 *
 * ARR and CCR are preloaded: what the ISR writes takes effect at the NEXT
 * update, so the interval now running was decided one step ago and the
 * hardware never waits on the ISR. The ISR only has to finish within one
 * interval (>= 2 * pulse_us). A pulse-less interval is CCR = 0xFFFFFFFF
 * (never reached), which is how the last step's tail and a ramp-to-zero end.
 *
 * Because DIR is written at the start of an interval and the pulse sits in
 * its tail, DIR setup is always >= interval - pulse_us >= pulse_us.
 *
 * Rates are carried as steps/s x 256 (q8). Per step the rate changes by
 * accel / rate (constant acceleration integrated over one step interval);
 * doing that in q8 keeps the increment from rounding to zero at high rates.
 *
 * Compiled per project (it needs the generated pad tables), like core_led.c.
 */

#include "core.h"
#include "core_stepper.h"
#include <string.h>

/* ---- Defaults ---- */
#define DEF_MAX_RATE    1000u
#define DEF_MIN_RATE    100u
#define DEF_ACCEL       4000u
#define DEF_PULSE_US    8u

#define MAX_ARR_16      65000u          /* longest interval on a 16-bit counter */
#define NO_PULSE        0xFFFFFFFFu     /* CCR beyond any ARR: output never active */

/* ---- Pad helpers ---- */

static inline void _dir_write(core_stepper_t *h, int8_t dir)
{
    int level = (dir > 0) ? 1 : 0;
    if (h->dir_invert) level = !level;
    core_pad_fast_write(h->dir, level);
}

/* ---- Channel register helpers ---- */

/* The timer's registers are the port's: port_core_stepper_output(),
 * _compare_write() / _read(), _channel_setup(), _period_write(), the counter,
 * the update interrupt, _latch(), _tick_1us(), _is_32bit(), _lock() /
 * _unlock() (port_core_stepper.h lists them). Included here, where these
 * helpers always sat, so GCC keeps the function order. */
#include "port_core_stepper.h"

/* Write the (preloaded) interval and pulse for a planned step. */
static inline void _program(core_stepper_t *h, uint32_t interval_us, int pulse)
{
    port_core_stepper_period_write(h->timer.instance, interval_us);
    port_core_stepper_compare_write(h, pulse ? interval_us - h->pulse_us : NO_PULSE);
}

/* ---- Profile math ---- */

static inline uint32_t _q8(uint32_t rate) { return rate << 8; }

static inline uint32_t _min_rate_q8(const core_stepper_t *h)
{
    uint32_t r = h->min_rate;
    uint32_t floor_r = h->wide ? 1u : (1000000u / MAX_ARR_16) + 1u;
    if (r < floor_r) r = floor_r;
    return _q8(r);
}

/* Steps to ramp from `rate_q8` down to min_rate at `accel`. */
static uint32_t _stop_steps(const core_stepper_t *h, uint32_t rate_q8)
{
    if (h->accel == 0) return 0;
    uint64_t r  = rate_q8 >> 8;
    uint64_t r0 = h->min_rate;
    if (r <= r0) return 0;
    return (uint32_t)((r * r - r0 * r0) / (2u * (uint64_t)h->accel));
}

/* Advance the rate one step towards `goal_q8` (above or below). */
static uint32_t _ramp(const core_stepper_t *h, uint32_t rate_q8, uint32_t goal_q8)
{
    if (h->accel == 0 || rate_q8 == 0) return goal_q8;
    uint32_t delta = (uint32_t)(((uint64_t)h->accel << 16) / rate_q8);
    if (delta == 0) delta = 1;
    if (goal_q8 > rate_q8) {
        uint32_t next = rate_q8 + delta;
        return (next > goal_q8 || next < rate_q8) ? goal_q8 : next;
    }
    if (goal_q8 < rate_q8) {
        return (rate_q8 - goal_q8 <= delta) ? goal_q8 : rate_q8 - delta;
    }
    return rate_q8;
}

/* Interval for a rate, honouring the pulse and the minimum low time. */
static uint32_t _interval_us(const core_stepper_t *h, uint32_t rate_q8)
{
    uint32_t us = (uint32_t)(256000000ull / rate_q8);   /* 1e6 * 256 / q8 */
    if (us < 2 * h->pulse_us) us = 2 * h->pulse_us;
    if (!h->wide && us > MAX_ARR_16) us = MAX_ARR_16;
    return us;
}

/* ---- Planner: decides the step AFTER the ones already committed ---- */

/* Fills nxt_* and programs the preload registers. `cur_*` describe the
 * interval now running (its pulse, if any, is still to come). */
static void _plan(core_stepper_t *h)
{
    uint32_t min_q8 = _min_rate_q8(h);
    uint32_t max_q8 = _q8(h->max_rate);
    if (max_q8 < min_q8) max_q8 = min_q8;
    uint32_t rate   = h->cur_rate_q8;
    int8_t   dir    = h->cur_dir;
    int32_t  vpos   = h->position + (h->cur_pulse ? h->cur_dir : 0);
    int8_t   want;
    uint32_t goal_q8;
    int      pulse = 1;

    if (h->state == CORE_STEPPER_MOVING) {
        int32_t remaining = h->target - vpos;
        if (remaining == 0) {
            pulse = 0; want = dir; goal_q8 = min_q8;
        } else {
            want = (remaining > 0) ? 1 : -1;
            uint32_t dist = (uint32_t)(remaining > 0 ? remaining : -remaining);
            if (want != dir)                          goal_q8 = min_q8;   /* ramp down, then reverse */
            else if (dist <= _stop_steps(h, rate))    goal_q8 = min_q8;   /* decelerate into target */
            else                                      goal_q8 = max_q8;
        }
    } else {                                                             /* RUNNING */
        int32_t g = h->goal_rate;
        if (g == 0) {
            want = dir; goal_q8 = min_q8;
            if (rate <= min_q8 || !h->cur_pulse) pulse = 0;
        } else {
            want = (g > 0) ? 1 : -1;
            uint32_t mag = (uint32_t)(g > 0 ? g : -g);
            if (mag > h->max_rate) mag = h->max_rate;
            goal_q8 = (want != dir) ? min_q8 : _q8(mag);
        }
    }

    if (goal_q8 < min_q8) goal_q8 = min_q8;
    rate = _ramp(h, rate, goal_q8);
    if (rate < min_q8) rate = min_q8;
    if (want != dir && rate <= min_q8) dir = want;   /* reverse once at the start rate */

    if (!h->cur_pulse) {
        /* The running interval is already the pulse-less tail: nothing to
         * follow. Keep the plan empty; the ISR will stop the timer. */
        pulse = 0;
    }

    h->nxt_pulse   = (uint8_t)pulse;
    h->nxt_dir     = dir;
    h->nxt_rate_q8 = rate;
    _program(h, _interval_us(h, pulse ? rate : min_q8), pulse);
}

static void _finish(core_stepper_t *h)
{
    core_timer_stop(&h->timer);
    port_core_stepper_output(h, PORT_CORE_STEPPER_OUT_INACTIVE);
    port_core_stepper_counter_clear(h->timer.instance);
    h->cur_pulse = h->nxt_pulse = 0;
    h->cur_rate_q8 = h->nxt_rate_q8 = 0;
    h->state = CORE_STEPPER_IDLE;
}

/* Kick the timer from idle. The first interval is committed directly (the
 * timer is stopped, so the preload is latched with a forced update), the
 * second is planned into the preload registers, then the counter runs. */
static void _start(core_stepper_t *h, int8_t dir)
{
    core_timer_instance_t *t = h->timer.instance;
    uint32_t rate = (h->accel == 0) ? _q8(h->max_rate) : _min_rate_q8(h);
    if (rate < _min_rate_q8(h)) rate = _min_rate_q8(h);

    h->cur_dir = dir;
    h->cur_pulse = 1;
    h->cur_rate_q8 = rate;
    _dir_write(h, dir);

    port_core_stepper_irq_off(t);
    port_core_stepper_counter_clear(t);
    _program(h, _interval_us(h, rate), 1);
    port_core_stepper_latch(t);      /* latch ARR/CCR into the active registers */
    port_core_stepper_output(h, PORT_CORE_STEPPER_OUT_PULSE);

    _plan(h);                        /* second interval → preload */
    core_timer_start(&h->timer);
}

/* Anything left to do after an interval ends? Used when a command lands
 * during the pulse-less tail: restart instead of idling. */
static int _work_pending(const core_stepper_t *h, int8_t *dir)
{
    if (h->state == CORE_STEPPER_MOVING && h->target != h->position) {
        *dir = (h->target > h->position) ? 1 : -1;
        return 1;
    }
    if (h->state == CORE_STEPPER_RUNNING && h->goal_rate != 0) {
        *dir = (h->goal_rate > 0) ? 1 : -1;
        return 1;
    }
    return 0;
}

/* ---- Timer ISR: one per interval ---- */

static void _isr(void *ctx)
{
    core_stepper_t *h = (core_stepper_t *)ctx;

    if (h->cur_pulse) h->position += h->cur_dir;   /* the pulse that just ended */

    /* The interval that just started is the one we planned last time. */
    h->cur_pulse   = h->nxt_pulse;
    h->cur_dir     = h->nxt_dir;
    h->cur_rate_q8 = h->nxt_rate_q8;

    if (!h->cur_pulse) {
        /* Pulse-less interval running: we are done, unless a command
         * arrived meanwhile. */
        int8_t dir;
        _finish(h);
        if (_work_pending(h, &dir)) _start(h, dir);
        return;
    }

    _dir_write(h, h->cur_dir);      /* >= interval - pulse_us before its edge */
    _plan(h);
}

/* ============================================================
 * Lifecycle
 * ============================================================ */

static core_status_t _init_common(core_stepper_t *h, core_timer_instance_t *instance,
                                  uint8_t channel, uint8_t step_pad, uint8_t dir_pad)
{
    memset(h, 0, sizeof(*h));
    if (channel < 1 || channel > 4) return CORE_ERROR;
    h->channel  = channel;
    h->step_pad = step_pad;
    h->dir_pad  = dir_pad;
    h->dir = core_pad_resolve(dir_pad);
    if (!h->dir.port) return CORE_ERROR;

    h->max_rate = DEF_MAX_RATE;
    h->min_rate = DEF_MIN_RATE;
    h->accel    = DEF_ACCEL;
    h->pulse_us = DEF_PULSE_US;
    h->cur_dir  = 1;
    h->nxt_dir  = 1;
    h->state    = CORE_STEPPER_IDLE;

    core_pad_output(dir_pad);
    _dir_write(h, 1);

    /* Claim the timer + its update interrupt, then retime it to 1 us ticks.
     * The period core_tick_init programs is irrelevant. */
    core_status_t rc = core_tick_init(&h->timer, instance, 1000, _isr, h);
    if (rc != CORE_OK) return rc;
    core_timer_stop(&h->timer);
    port_core_stepper_irq_off(instance);
    port_core_stepper_tick_1us(instance);
    h->wide = port_core_stepper_is_32bit(instance);
    port_core_stepper_period_write(instance, 2 * h->pulse_us);
    port_core_stepper_latch(instance);                   /* latch PSC */
    port_core_stepper_channel_setup(h);
    port_core_stepper_compare_write(h, NO_PULSE);
    return CORE_OK;
}

core_status_t core_stepper_init(core_stepper_t *h, uint8_t step_pad, uint8_t dir_pad)
{
#ifdef CORE_USES_TIMER_PADS
    core_timer_instance_t *instance;
    uint8_t channel;
    if (core_pad_timer_info(step_pad, &instance, &channel) != 0) {
        memset(h, 0, sizeof(*h));
        return CORE_ERROR;         /* STEP pad is not declared as TIMx.y in config.json */
    }
    return _init_common(h, instance, channel, step_pad, dir_pad);
#else
    (void)step_pad; (void)dir_pad;
    memset(h, 0, sizeof(*h));
    return CORE_ERROR;             /* no timer pads in this project: use core_stepper_init_ch */
#endif
}

core_status_t core_stepper_init_ch(core_stepper_t *h, core_timer_instance_t *instance,
                                   uint8_t channel, uint8_t step_pad, uint8_t dir_pad)
{
    return _init_common(h, instance, channel, step_pad, dir_pad);
}

void core_stepper_set_enable_pad(core_stepper_t *h, uint8_t en_pad, bool active_low)
{
    h->en_pad = en_pad;
    h->en_active_low = active_low ? 1 : 0;
    if (en_pad == 0) {
        h->en.port = 0;
        h->en.mask = 0;
        return;
    }
    h->en = core_pad_resolve(en_pad);
    if (!h->en.port) return;
    core_pad_output(en_pad);
    core_stepper_enable(h, false);
}

void core_stepper_enable(core_stepper_t *h, bool on)
{
    if (!h->en.port) return;
    int level = on ? 1 : 0;
    if (h->en_active_low) level = !level;
    core_pad_fast_write(h->en, level);
}

void core_stepper_set_polarity(core_stepper_t *h, bool step_active_low, bool dir_invert)
{
    h->step_active_low = step_active_low ? 1 : 0;
    h->dir_invert = dir_invert ? 1 : 0;
    if (h->state == CORE_STEPPER_IDLE) {
        port_core_stepper_channel_setup(h);
        _dir_write(h, h->cur_dir);
    }
}

void core_stepper_set_pulse_us(core_stepper_t *h, uint32_t pulse_us)
{
    if (pulse_us < 1) pulse_us = 1;
    if (pulse_us > 1000) pulse_us = 1000;
    h->pulse_us = pulse_us;
}

/* ============================================================
 * Profile
 * ============================================================ */

void core_stepper_set_speed(core_stepper_t *h, uint32_t steps_per_s)
{
    if (steps_per_s < 1) steps_per_s = 1;
    if (steps_per_s > 500000) steps_per_s = 500000;
    h->max_rate = steps_per_s;
    if (h->min_rate > h->max_rate) h->min_rate = h->max_rate;
}

void core_stepper_set_accel(core_stepper_t *h, uint32_t steps_per_s2)
{
    if (steps_per_s2 > 1000000) steps_per_s2 = 1000000;
    h->accel = steps_per_s2;
}

void core_stepper_set_min_speed(core_stepper_t *h, uint32_t steps_per_s)
{
    if (steps_per_s < 1) steps_per_s = 1;
    if (steps_per_s > h->max_rate) steps_per_s = h->max_rate;
    h->min_rate = steps_per_s;
}

/* ============================================================
 * Motion
 * ============================================================ */

void core_stepper_move_to(core_stepper_t *h, int32_t position)
{
    uint32_t s = port_core_stepper_lock();
    h->target = position;
    if (h->state == CORE_STEPPER_IDLE) {
        int32_t remaining = position - h->position;
        if (remaining != 0) {
            h->state = CORE_STEPPER_MOVING;
            _start(h, remaining > 0 ? 1 : -1);
        }
    } else {
        /* A run() becomes a move to the new target; a move just retargets.
         * The planner picks it up at the next interval. */
        h->state = CORE_STEPPER_MOVING;
    }
    port_core_stepper_unlock(s);
}

void core_stepper_move(core_stepper_t *h, int32_t steps)
{
    uint32_t s = port_core_stepper_lock();
    int32_t base = (h->state == CORE_STEPPER_MOVING) ? h->target : h->position;
    port_core_stepper_unlock(s);
    core_stepper_move_to(h, base + steps);
}

void core_stepper_run(core_stepper_t *h, int32_t steps_per_s)
{
    uint32_t s = port_core_stepper_lock();
    h->goal_rate = steps_per_s;
    if (h->state == CORE_STEPPER_IDLE) {
        if (steps_per_s != 0) {
            h->state = CORE_STEPPER_RUNNING;
            _start(h, steps_per_s > 0 ? 1 : -1);
        }
    } else {
        h->state = CORE_STEPPER_RUNNING;
    }
    port_core_stepper_unlock(s);
}

void core_stepper_stop(core_stepper_t *h)
{
    core_stepper_run(h, 0);
}

void core_stepper_halt(core_stepper_t *h)
{
    uint32_t s = port_core_stepper_lock();
    core_timer_instance_t *t = h->timer.instance;
    if (h->state != CORE_STEPPER_IDLE) {
        core_timer_stop(&h->timer);
        /* Rising edge already out? Then the driver took the step. */
        if (h->cur_pulse && port_core_stepper_counter_read(t) >= port_core_stepper_compare_read(h))
            h->position += h->cur_dir;
    }
    _finish(h);
    h->goal_rate = 0;
    h->target = h->position;
    port_core_stepper_unlock(s);
}

/* ============================================================
 * Status
 * ============================================================ */

bool core_stepper_busy(const core_stepper_t *h)
{
    return h->state != CORE_STEPPER_IDLE;
}

core_stepper_state_t core_stepper_state(const core_stepper_t *h)
{
    return (core_stepper_state_t)h->state;
}

int32_t core_stepper_position(const core_stepper_t *h)
{
    return h->position;
}

void core_stepper_set_position(core_stepper_t *h, int32_t position)
{
    uint32_t s = port_core_stepper_lock();
    if (h->state == CORE_STEPPER_IDLE) {
        h->position = position;
        h->target = position;
    }
    port_core_stepper_unlock(s);
}

int32_t core_stepper_target(const core_stepper_t *h)
{
    return h->target;
}

int32_t core_stepper_speed(const core_stepper_t *h)
{
    if (h->state == CORE_STEPPER_IDLE || !h->cur_pulse) return 0;
    int32_t r = (int32_t)(h->cur_rate_q8 >> 8);
    return h->cur_dir < 0 ? -r : r;
}

uint32_t core_stepper_stop_distance(const core_stepper_t *h, uint32_t steps_per_s)
{
    return _stop_steps(h, _q8(steps_per_s));
}
