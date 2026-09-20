/**
 * core_stepper.h — STEP/DIR pulse generator for external stepper drivers
 *
 * Drives any STEP/DIR stepper driver (StepperOnline DM320T/DM542T, TI DRV8428,
 * Trinamic, A4988 ...) from a timer output channel (STEP), a GPIO pad (DIR)
 * and the timer's update interrupt. The hardware makes every STEP pulse: the
 * timer period is the step interval and a compare channel carves the pulse
 * out of its tail, so pulse width and step timing are jitter-free and do not
 * depend on interrupt latency. One interrupt per step ramps the rate up and
 * down with a linear acceleration profile and keeps a signed microstep count.
 * Nothing blocks — queue a move and poll core_stepper_busy(), or run
 * continuously at a signed velocity.
 *
 * Signal contract (defaults suit the DM320T, the most conservative common
 * driver — loosen with core_stepper_set_pulse_us for faster inputs):
 *   - STEP idles low and pulses high for `pulse_us` (default 8 us; the DM320T
 *     needs >= 7.5 us). The low time between pulses is also >= `pulse_us`.
 *   - DIR only changes at the start of a step interval, so it is always at
 *     least (interval - pulse_us) >= pulse_us ahead of the next rising edge
 *     (the DM320T needs >= 5 us of setup).
 *   - Optional ENABLE pad, either polarity.
 * Both signals are push-pull. Tie a driver's opto-isolated PUL+/DIR+ (or the
 * shared OPTO terminal) to the logic rail and let the pads sink PUL-/DIR-; a
 * DM320T steps fine from a 3.3 V Core this way. If the driver counts falling
 * edges instead, pick that with core_stepper_set_polarity().
 *
 * STEP must be on a pad with a timer output channel (a TIMx.y function in the
 * tile definition, e.g. Core.ST.L4.1 pad 3 = TIM2.2). Declare it as that
 * function in config.json so coregen routes the pin; core_stepper_init() then
 * finds the timer and channel from the pad. Without coregen, use
 * core_stepper_init_ch() and set the pad's alternate function yourself. DIR
 * (and ENABLE) are ordinary GPIO pads.
 *
 * Rates are in microsteps per second at whatever microstep setting the driver
 * is in — the module does not know or care about the driver's step angle.
 * Position is a signed 32-bit microstep count that you can zero at will.
 *
 * Timing resolution is 1 us. The highest rate is bounded by
 * 1 / (2 * pulse_us) — 62.5 kHz at the default 8 us. The lowest is 1 step/s
 * on a 32-bit timer (TIM2 on the ST Cores) and ~16 steps/s on a 16-bit one.
 *
 * @studio category stepper label=Core.Stepper icon=⟳
 *
 * @studio coverage
 *   id:    stepper
 *   name:  Stepper — STEP/DIR pulse generation
 *   page:  /docs/sdk/stepper
 *   blurb: Tier 1 handle-based STEP/DIR generator: STEP on a timer output
 *          channel (hardware pulse, one interrupt per step), DIR on a pad.
 *          Trapezoidal ramps (max rate, acceleration, start rate), relative
 *          and absolute position moves, continuous signed velocity, soft
 *          stop and hard halt, signed microstep position. Defaults honour
 *          the slowest common driver inputs. No Tier 2 / DSL surface yet.
 *
 * Typical usage (config.json: "pads": { "3": "TIM2.2", "9": "GPIO.OUT" }):
 *
 *   #include "core_stepper.h"
 *
 *   core_stepper_t axis;
 *   core_stepper_init(&axis, 3, 9);              // STEP pad 3, DIR pad 9
 *   core_stepper_set_speed(&axis, 1600);         // steps/s
 *   core_stepper_set_accel(&axis, 8000);         // steps/s^2 (0 = no ramp)
 *
 *   core_stepper_move(&axis, 160);               // 160 microsteps, non-blocking
 *   while (core_stepper_busy(&axis)) {
 *       core_watchdog_feed();
 *   }
 */

#ifndef CORE_STEPPER_H
#define CORE_STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "core_timer.h"
#include "core_pad.h"

/* ============================================================
 * Handle
 * ============================================================ */

/** Motion state. Read with core_stepper_state(); never write it. */
typedef enum {
    CORE_STEPPER_IDLE = 0,   /**< Timer stopped, no pulses. */
    CORE_STEPPER_MOVING,     /**< Position move in progress (move / move_to). */
    CORE_STEPPER_RUNNING,    /**< Velocity mode (run), including a ramp to zero. */
} core_stepper_state_t;

/**
 * Stepper axis handle. Allocate one per motor; treat the fields as private.
 * Fields tagged ISR are written from the timer interrupt.
 */
typedef struct {
    core_timer_t     timer;          /**< Step-clock timer (owned). */
    uint8_t          channel;        /**< Timer output channel driving STEP (1..4). */
    uint8_t          step_pad;       /**< STEP pad number. */
    uint8_t          dir_pad;        /**< DIR pad number. */
    uint8_t          en_pad;         /**< ENABLE pad number, 0 = none. */
    core_pad_fast_t  dir;            /**< Resolved DIR pad. */
    core_pad_fast_t  en;             /**< Resolved ENABLE pad (port NULL if none). */
    uint8_t          step_active_low;/**< 1: STEP idles high, pulses low. */
    uint8_t          dir_invert;     /**< 1: DIR high means negative. */
    uint8_t          en_active_low;  /**< 1: ENABLE pad low enables the driver. */
    uint8_t          wide;           /**< 1: 32-bit counter (long intervals OK). */

    uint32_t         max_rate;       /**< Cruise rate, steps/s. */
    uint32_t         min_rate;       /**< Start/stop rate, steps/s (>= 1). */
    uint32_t         accel;          /**< Ramp, steps/s^2. 0 = jump to max_rate. */
    uint32_t         pulse_us;       /**< STEP pulse width, us (>= 1). */

    volatile int32_t  position;      /**< ISR. Signed microstep count. */
    volatile int32_t  target;        /**< Goal position in MOVING. */
    volatile int32_t  goal_rate;     /**< Signed goal velocity in RUNNING, steps/s. */
    volatile uint8_t  state;         /**< ISR. core_stepper_state_t. */
    /* The interval now running (cur) and the one already latched to follow
     * it (nxt): whether it carries a pulse, in which direction, at what rate. */
    volatile uint8_t  cur_pulse, nxt_pulse;
    volatile int8_t   cur_dir,   nxt_dir;
    volatile uint32_t cur_rate_q8, nxt_rate_q8;   /**< |rate| in steps/s x 256. */
} core_stepper_t;

/* ============================================================
 * Lifecycle
 * ============================================================ */

/**
 * Bind a stepper axis to a STEP pad (timer output) and a DIR pad. The timer
 * and channel come from the pad's TIMx.y assignment in config.json, which is
 * also what routes the pin to the timer. Loads conservative defaults:
 * 1000 steps/s cruise, 100 steps/s start, 4000 steps/s^2 ramp, 8 us pulses.
 * The timer is left stopped until a move.
 *
 * Do not share the timer with a PWM output — the module rewrites its period
 * on every step.
 *
 * @param h         Axis handle to initialise.
 * @param step_pad  [1..64] Pad wired to the driver's STEP / PUL input; must carry a TIMx.y function.
 * @param dir_pad   [1..64] Pad wired to the driver's DIR input.
 * @return HAL_OK; HAL_ERROR if the STEP pad is not a timer pad in config.json, DIR has no GPIO, or the timer cannot interrupt.
 */
hal_status_t core_stepper_init(core_stepper_t *h, uint8_t step_pad, uint8_t dir_pad);

/**
 * Bind an axis with an explicit timer instance and channel (no coregen
 * lookup). The caller must have configured the STEP pad's alternate
 * function for that channel. Everything else as core_stepper_init().
 *
 * @param h         Axis handle to initialise.
 * @param instance  Timer instance, e.g. TIM2.
 * @param channel   [1..4] Output channel wired to the STEP pad.
 * @param step_pad  [1..64] STEP pad (recorded only; its AF is the caller's job).
 * @param dir_pad   [1..64] Pad wired to the driver's DIR input.
 * @return HAL_OK, or HAL_ERROR if DIR has no GPIO or the timer cannot interrupt.
 */
hal_status_t core_stepper_init_ch(core_stepper_t *h, TIM_TypeDef *instance, uint8_t channel,
                                  uint8_t step_pad, uint8_t dir_pad);

/**
 * Add an ENABLE output. The pad is driven to its disabled level immediately;
 * call core_stepper_enable() to power the driver. Pass pad 0 to remove it.
 *
 * @param h           Axis handle.
 * @param en_pad      [0..64] Pad wired to the driver's ENA input (0 = none).
 * @param active_low  true if the driver enables on a LOW level.
 */
void core_stepper_set_enable_pad(core_stepper_t *h, uint8_t en_pad, bool active_low);

/**
 * Drive the ENABLE pad. No effect without core_stepper_set_enable_pad().
 * Motion is not interlocked: halt before disabling if you care where the
 * rotor ends up.
 *
 * @param h   Axis handle.
 * @param on  true to enable the driver.
 */
void core_stepper_enable(core_stepper_t *h, bool on);

/**
 * Set signal polarities. Call while idle.
 *
 * @param h                Axis handle.
 * @param step_active_low  true if the driver steps on the FALLING edge (STEP idles high).
 * @param dir_invert       true to swap which DIR level counts as positive.
 */
void core_stepper_set_polarity(core_stepper_t *h, bool step_active_low, bool dir_invert);

/**
 * Set the STEP pulse width; the low time between pulses is kept at least as
 * long, which also bounds DIR setup. 8 us (default) satisfies the DM320T's
 * 7.5 us; a DRV8428 is happy with 1 us. The maximum step rate is
 * 1 / (2 * pulse_us).
 *
 * @param h         Axis handle.
 * @param pulse_us  [1..1000] us Pulse width.
 */
void core_stepper_set_pulse_us(core_stepper_t *h, uint32_t pulse_us);

/* ============================================================
 * Motion profile
 * ============================================================ */

/**
 * Set the cruise rate for position moves and the cap for run().
 * Takes effect at the next step; a move in flight ramps to the new value.
 *
 * @param h          Axis handle.
 * @param steps_per_s  [1..500000] hz Microsteps per second.
 */
void core_stepper_set_speed(core_stepper_t *h, uint32_t steps_per_s);

/**
 * Set the acceleration used for every ramp, or 0 to switch instantly
 * between the start rate and full speed (fine for small, unloaded motors
 * at low rates; expect stalls otherwise).
 *
 * @param h            Axis handle.
 * @param steps_per_s2 [0..1000000] Microsteps per second per second.
 */
void core_stepper_set_accel(core_stepper_t *h, uint32_t steps_per_s2);

/**
 * Set the rate a ramp starts from and stops at. Steppers can start
 * instantly at a few hundred full steps per second; starting from a
 * crawl only wastes time. Clamped to at least 1 (16 on a 16-bit timer).
 *
 * @param h            Axis handle.
 * @param steps_per_s  [1..500000] hz Start / stop rate.
 */
void core_stepper_set_min_speed(core_stepper_t *h, uint32_t steps_per_s);

/* ============================================================
 * Motion commands (all non-blocking)
 * ============================================================ */

/**
 * Move a signed number of microsteps relative to the current target
 * (so two quick calls of +160 travel 320). Retargets a move in flight;
 * a reversal decelerates to the start rate before flipping DIR.
 *
 * @param h      Axis handle.
 * @param steps  Microsteps, negative for the reverse direction.
 */
void core_stepper_move(core_stepper_t *h, int32_t steps);

/**
 * Move to an absolute microstep position.
 *
 * @param h         Axis handle.
 * @param position  Target position.
 */
void core_stepper_move_to(core_stepper_t *h, int32_t position);

/**
 * Run continuously at a signed rate, ramping from the current speed.
 * Rate 0 decelerates to a stop (same as core_stepper_stop()). The
 * magnitude is capped at the cruise rate from core_stepper_set_speed().
 *
 * @param h            Axis handle.
 * @param steps_per_s  Signed microsteps per second.
 */
void core_stepper_run(core_stepper_t *h, int32_t steps_per_s);

/**
 * Decelerate to a stop using the configured acceleration. Position
 * continues to count during the ramp. Idempotent.
 *
 * @param h  Axis handle.
 */
void core_stepper_stop(core_stepper_t *h);

/**
 * Stop instantly: timer off, STEP forced to idle. A pulse whose rising
 * edge the driver has already seen is counted. A loaded motor at speed
 * will lose steps; the position counter reflects the pulses sent.
 *
 * @param h  Axis handle.
 */
void core_stepper_halt(core_stepper_t *h);

/* ============================================================
 * Status
 * ============================================================ */

/**
 * true while pulses are being generated (including a ramp to zero).
 *
 * @param h  Axis handle.
 * @return true if not idle.
 */
bool core_stepper_busy(const core_stepper_t *h);

/**
 * Current motion state.
 *
 * @param h  Axis handle.
 * @return CORE_STEPPER_IDLE / MOVING / RUNNING.
 */
core_stepper_state_t core_stepper_state(const core_stepper_t *h);

/**
 * Current signed microstep position (pulses actually issued).
 *
 * @param h  Axis handle.
 * @return Position.
 */
int32_t core_stepper_position(const core_stepper_t *h);

/**
 * Redefine the current position without moving (homing). Only while
 * idle; the target follows so the next move() is relative to it.
 *
 * @param h         Axis handle.
 * @param position  New value for the current position.
 */
void core_stepper_set_position(core_stepper_t *h, int32_t position);

/**
 * Target position of the current or last move.
 *
 * @param h  Axis handle.
 * @return Target position.
 */
int32_t core_stepper_target(const core_stepper_t *h);

/**
 * Current signed step rate.
 *
 * @param h  Axis handle.
 * @return Microsteps per second, negative in the reverse direction, 0 when idle.
 */
int32_t core_stepper_speed(const core_stepper_t *h);

/**
 * Microsteps needed to decelerate from `steps_per_s` to the start rate at
 * the configured acceleration. Useful for planning where a run() will
 * stop, e.g. to land on a pocket boundary.
 *
 * @param h            Axis handle.
 * @param steps_per_s  Rate to stop from.
 * @return Stopping distance in microsteps (0 when accel is 0).
 */
uint32_t core_stepper_stop_distance(const core_stepper_t *h, uint32_t steps_per_s);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=M title="No default-instance / DSL surface"
//   Everything takes a core_stepper_t*. Coregen does not yet read a
//   `stepper` block from config.json, so there is no `core_stepper_move(pad,
//   steps)`-style wrapper and nothing in the Studio palette. The natural
//   Tier 2 shape is one axis per config.json entry with STEP/DIR pads,
//   initialised in core_init().
//
// @studio unsupported tier=1 value=L title="STEP must be a timer output pad"
//   The pulse is made by a compare channel, so STEP is restricted to pads
//   carrying a TIMx.y function. There is no GPIO-bitbang fallback for
//   arbitrary pads; DIR and ENABLE can be any pad.
//
// @studio unsupported tier=1 value=L title="Linear ramps only"
//   The profile is trapezoidal (constant acceleration). No S-curve, no
//   per-move acceleration override, no synchronised multi-axis moves.
//
// @studio unsupported tier=1 value=L title="No stall, fault or limit-switch inputs"
//   Drivers' nFAULT / ALM outputs and end-stop switches are left to the
//   application (core_pad_on_change + core_stepper_halt).

#endif /* CORE_STEPPER_H */
