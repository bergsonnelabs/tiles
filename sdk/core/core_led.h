/**
 * core_led.h — Onboard LED pattern helpers
 *
 * Every Core tile has an onboard LED connected to a dedicated GPIO pin.
 * This module provides a lightweight API for common patterns like blinking,
 * heartbeat, and SOS error signaling. The on/off/toggle/blink helpers are
 * header-only static inlines; the free-running heartbeat lives in core_led.c
 * because it owns persistent state serviced from the SysTick interrupt. The
 * LED pin is auto-detected from the tile definition and configured by
 * core_led_init() — it's not part of the configurable pad map.
 *
 * Uses LED_ON/OFF/TOGGLE from core_board.h and ll_delay_ms from ll_systick.h.
 *
 * @studio category led label=Core.LED icon=☀
 *
 * @studio coverage
 *   id:    led
 *   name:  LED — onboard status LED
 *   page:  /docs/sdk/led
 *   blurb: Header-only helpers for the dedicated onboard LED (auto-
 *          detected from the tile definition — not part of the pad
 *          map). Tier 2 covers on / off / toggle / blink / heartbeat;
 *          the simulator renders the LED state in real time, so DSL
 *          programs can iterate on blink patterns without flashing
 *          hardware. SOS pattern is Tier 1 (escape-to-C only — it's
 *          a `noreturn` blocker).
 */

#ifndef CORE_LED_H
#define CORE_LED_H

#include "core_board.h"
#include "ll_systick.h"
#include "ll_rcc.h"
#include "ll_gpio.h"

/**
 * Enable the GPIO clock and configure the LED pin as a push-pull output.
 * Call once after core_init(). The LED starts in the off state.
 */
static inline void core_led_init(void)
{
    ll_rcc_gpio_clk_enable(LED_PORT);
    ll_gpio_config_output(LED_PORT, LED_PIN);
    LED_OFF();
}

/**
 * Turn the onboard LED on. Simplest possible light control — useful
 * alongside blink/heartbeat when the user wants explicit on/off state
 * rather than a timed pattern.
 *
 * @studio expose category=led name=on
 * @studio twin full
 */
static inline void core_led_on(void)
{
    LED_ON();
}

/**
 * Turn the onboard LED off.
 *
 * @studio expose category=led name=off
 * @studio twin full
 */
static inline void core_led_off(void)
{
    LED_OFF();
}

/**
 * Flip the onboard LED's current state. Non-blocking — no delay.
 * Good for driving the LED from an event handler that fires at a
 * rate you've already chosen (e.g., a pad-edge ISR on a button).
 *
 * @studio expose category=led name=toggle
 * @studio twin full
 */
static inline void core_led_toggle(void)
{
    LED_TOGGLE();
}

/*
 * A short LED on-time, stretched for the supply voltage so a blip looks
 * equally bright from 1.8 V to 3.6 V. on_ms is the on-time at 3.3 V; below
 * ~50 ms the eye integrates the flash, so lower VDD (less LED current) gets
 * a proportionally longer blip, up to 16x and capped at 50 ms. On-times of
 * 50 ms and longer come back unchanged. Reads VDD through VREFINT at most
 * once a minute (L0 / L4; other Cores return on_ms as is). Defined in
 * core_led.c. Plain comment on purpose: internal, kept out of the manifest.
 */
uint32_t core_led_scaled_on_ms(uint32_t on_ms);

/**
 * Blink the LED n times. Each cycle turns the LED on for on_ms
 * milliseconds and off for off_ms milliseconds. Blocking — returns
 * after the last off period.
 *
 * A short on_ms (under 50 ms) is the on-time at 3.3 V: at lower supply
 * voltages it is lengthened so the blink looks as bright (the off-time
 * shrinks to keep the rhythm). See core_led_heartbeat().
 *
 * @studio expose category=led name=blink
 * @studio twin full
 * @param n [1..100] Number of times to blink.
 * @param on_ms [1..5000] ms LED-on duration per blink.
 * @param off_ms [1..5000] ms LED-off duration per blink.
 */
static inline void core_led_blink(int n, int on_ms, int off_ms)
{
    if (on_ms < 0) on_ms = 0;
    if (off_ms < 0) off_ms = 0;
    /* Stretch a short on-time for the supply (see core_led_scaled_on_ms)
     * and take the difference out of the off-time, so the rhythm holds. */
    int on = (int)core_led_scaled_on_ms((uint32_t)on_ms);
    int off = off_ms - (on - on_ms);
    if (off < 0) off = 0;
    for (int i = 0; i < n; i++) {
        LED_ON();  ll_delay_ms(on);
        LED_OFF(); ll_delay_ms(off);
    }
}

/**
 * Blink the SOS pattern (3 short, 3 long, 3 short) in an infinite loop.
 * Use for unrecoverable errors. This function never returns.
 */
static inline void core_led_sos(void) __attribute__((noreturn));
static inline void core_led_sos(void)
{
    while (1) {
        core_led_blink(3, 100, 100);
        core_led_blink(3, 400, 400);
        core_led_blink(3, 100, 100);
        ll_delay_ms(2000);
    }
}

/**
 * Start a free-running, asymmetric heartbeat on the onboard LED. Set it
 * up once (e.g. at startup) and it runs on its own — the LED turns on for
 * on_ms, off for the rest of period_ms, repeating forever. Serviced from
 * the 1 ms SysTick interrupt, so it does NOT block your main loop the way
 * a toggle-and-delay would: the loop stays free for tile reads and logic.
 *
 * Call again at any time to change the rhythm; pass period_ms = 0 to stop
 * the heartbeat (the LED is left off). on_ms is clamped to period_ms.
 *
 * Brightness holds across the supply: a short on_ms (under 50 ms, e.g. a
 * 2 ms blip) is the on-time at 3.3 V, and is lengthened as VDD drops
 * (about 32 ms at 1.8 V), because the LED carries far less current there
 * and the eye adds up the light of a short flash. VDD is read through
 * VREFINT when the heartbeat is set (at most once a minute) on the L0 and
 * L4; other Cores use the on-time as given.
 *
 * Examples:
 *   heartbeat(1000, 2)    — a 1 Hz 2 ms blip, the same brightness at any VDD.
 *   heartbeat(1000, 100)  — a 1 Hz "blip": 100 ms on, 900 ms off.
 *   heartbeat(500, 250)   — a steady 1 Hz, 50%-duty pulse.
 *
 * Defined in core_led.c (not header-only) because it owns persistent
 * state shared with the SysTick handler.
 *
 * @studio expose category=led name=heartbeat
 * @studio twin full
 * @param period_ms [0..60000] ms Full cycle length. 0 stops the heartbeat.
 * @param on_ms [0..60000] ms LED-on portion of each cycle (clamped to period_ms).
 */
void core_led_heartbeat(int period_ms, int on_ms);

/*
 * Service the heartbeat state machine. Called once per millisecond from
 * SysTick_Handler (via a weak hook) — application code never calls this
 * directly. Intentionally a plain comment, not a Doxygen block, so the
 * manifest scraper skips it and it stays out of the SDK API reference.
 * A no-op until core_led_heartbeat() has been started.
 */
void core_led_systick_tick(void);

/* ---- Coverage gaps (consumed by the SDK Coverage Table) ---- */

// @studio unsupported tier=2 value=L title="No DSL access to SOS"
//   core_led_sos is `__attribute__((noreturn))` so it can't be
//   exposed safely — DSL programs that "call SOS" would deadlock
//   the worker. Reserved for fault-handler escape-to-C use.
//
// @studio unsupported tier=1 value=L title="No PWM brightness / color"
//   The onboard LED is hard-wired as a digital push-pull output. No
//   PWM dimming wrapper, no RGB/RGBW support (tiles with addressable
//   LEDs use Disp.RGBW or similar, not core_led).

#endif /* CORE_LED_H */
