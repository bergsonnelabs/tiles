/**
 * core_led.c — Free-running heartbeat state machine
 *
 * The on/off/toggle/blink helpers are header-only static inlines in
 * core_led.h. The heartbeat is the one LED pattern that needs persistent
 * state and a periodic tick, so it lives here:
 *
 *   - core_led_heartbeat(period_ms, on_ms) stores the rhythm and arms it.
 *   - core_led_systick_tick() advances the state machine; SysTick_Handler
 *     calls it once per millisecond through a weak hook (see hal_systick.c).
 *
 * Keeping the toggle out of the main loop means a starter program can do
 *   on start { led.heartbeat(1000, 100) }
 * and leave the loop free for real work — the LED keeps blinking on its
 * own off the SysTick interrupt.
 */

/* Pull in the project's generated umbrella so the MCU register context
 * (REG32 / SYSTICK_BASE from ll_common.h) and board macros (LED_PORT/PIN
 * from core_board.h) are established before core_led.h's includes — the
 * same ordering every main.c relies on. core_led.o is compiled per project
 * (it depends on the generated headers), so core.h is always available. */
#include "core.h"
#include "hal_adc.h"

/* ---- Brightness that holds across VDD ----
 * Every Core drives a red LED from a GPIO through ~40 ohm, so its current is
 * (VDD - Vf) / (R + pin): about 10x higher at 3.3 V than at 1.8 V. Series and
 * pin resistance cancel in the ratio, so the current relative to 3.3 V is
 * (VDD - Vf) / (3300 - Vf). For flashes shorter than ~50 ms the eye adds up
 * the light (Bloch's law), so stretching a blip's on-time by the inverse
 * ratio keeps it looking the same. on_ms therefore means "the on-time at
 * 3.3 V": at 1.8 V a 2 ms blip runs ~32 ms. Longer flashes are judged by
 * intensity, not energy, and are left alone. */
#ifndef CORE_LED_VF_MV
#define CORE_LED_VF_MV    1700u   /* red LED at the ~1 mA it carries near 1.8 V */
#endif
#define CORE_LED_REF_MV   3300u   /* on_ms is calibrated here */
#define CORE_LED_BLIP_MS  50u     /* Bloch window: scale only blips shorter than this */
#define CORE_LED_MAX_Q8   (16u << 8)
#define CORE_LED_RESCAN_MS 60000u /* re-read VDD at most once a minute */

/* The project's ADC handle, when it declares ADC pads (coregen). Weak, so a
 * project without one links, and we can tell. */
extern hal_adc_t core_adc1 __attribute__((weak));

static uint32_t s_scale_q8 = 0;      /* 0 = not measured yet */
static uint32_t s_scaled_at_ms = 0;

/* VDD in mV through VREFINT: L0 and L4 (ADC1), W5 (ADC4). The H5's
 * VREFINT read is not working yet and returns a nominal 3300 mV, which
 * gives a scale of 1 (no change). */
#if defined(STM32WBA55xx)
#define LED_VDD_ADC  ADC4
#else
#define LED_VDD_ADC  ADC1
#endif
static uint32_t led_vdd_mv(void)
{
#if defined(STM32L011xx) || defined(STM32L422xx) || defined(STM32WBA55xx)
    if (&core_adc1 != 0 && core_adc1.instance != 0) {
        /* The project owns ADC1: never re-sequence it under running DMA. */
        if (core_adc1.dma_active) {
            return core_adc1.vdda_mv ? core_adc1.vdda_mv : CORE_LED_REF_MV;
        }
        return hal_adc_read_vdda_mv(&core_adc1);
    }
    /* No project ADC: bring one up, read VREFINT, and power it back down. */
    hal_adc_t adc;
    if (hal_adc_init(&adc, LED_VDD_ADC, SYSCLK_HZ, HAL_ADC_RES_12BIT) != HAL_OK) {
        return CORE_LED_REF_MV;
    }
    uint32_t mv = hal_adc_read_vdda_mv(&adc);
    hal_adc_deinit(&adc);
    return mv;
#else
    return CORE_LED_REF_MV;
#endif
}

/* On-time multiplier relative to 3.3 V, Q8 (256 = 1x), clamped to 16x. */
static uint32_t led_scale_q8(void)
{
    uint32_t now = _systick_ticks;
    if (s_scale_q8 != 0u && (now - s_scaled_at_ms) < CORE_LED_RESCAN_MS) {
        return s_scale_q8;
    }
    uint32_t vdd = led_vdd_mv();
    uint32_t headroom_ref = CORE_LED_REF_MV - CORE_LED_VF_MV;
    uint32_t q8 = CORE_LED_MAX_Q8;
    if (vdd > CORE_LED_VF_MV) {
        q8 = (headroom_ref << 8) / (vdd - CORE_LED_VF_MV);
        if (q8 > CORE_LED_MAX_Q8) q8 = CORE_LED_MAX_Q8;
    }
    s_scale_q8 = q8;
    s_scaled_at_ms = now;
    return q8;
}

uint32_t core_led_scaled_on_ms(uint32_t on_ms)
{
    if (on_ms == 0u || on_ms >= CORE_LED_BLIP_MS) {
        return on_ms;
    }
    uint32_t ms = (on_ms * led_scale_q8() + 128u) >> 8;
    if (ms == 0u) ms = 1u;
    if (ms > CORE_LED_BLIP_MS) ms = CORE_LED_BLIP_MS;
    return ms;
}

/* Heartbeat configuration + phase. Touched by both the caller (arming the
 * heartbeat) and the SysTick ISR (advancing it), so everything the ISR
 * reads is volatile. period_ms == 0 means "disabled". */
static volatile uint32_t s_period_ms = 0;
static volatile uint32_t s_on_ms = 0;
static volatile uint32_t s_phase_ms = 0;

void core_led_heartbeat(int period_ms, int on_ms)
{
    /* Negative inputs are nonsense (the DSL bounds-checks to [0..60000],
     * but the C entry point is public) — treat them as 0. */
    uint32_t period = period_ms > 0 ? (uint32_t)period_ms : 0u;
    uint32_t on = on_ms > 0 ? (uint32_t)on_ms : 0u;
    on = core_led_scaled_on_ms(on);  /* same brightness at any VDD */
    if (on > period) {
        on = period;  /* on-time can't exceed the cycle */
    }

    /* Disabled: stop the heartbeat and leave the LED off. */
    if (period == 0u) {
        s_period_ms = 0u;
        LED_OFF();
        return;
    }

    /* Arm it. Reset the phase and drive the first edge immediately so the
     * caller sees the LED react now rather than up to one cycle later.
     * Writing s_period_ms last means the ISR never observes a half-updated
     * config (it bails while s_period_ms is still 0). */
    s_phase_ms = 0u;
    s_on_ms = on;
    if (on > 0u) {
        LED_ON();
    } else {
        LED_OFF();
    }
    s_period_ms = period;
}

void core_led_systick_tick(void)
{
    uint32_t period = s_period_ms;
    if (period == 0u) {
        return;  /* heartbeat not running */
    }

    /* Advance one millisecond, wrapping at the cycle boundary. The on-edge
     * fires at phase 0 (cycle start), the off-edge at phase == on_ms. With
     * on_ms == 0 the LED never lights; with on_ms == period it never goes
     * dark (a solid-on "heartbeat"). */
    uint32_t phase = s_phase_ms + 1u;
    if (phase >= period) {
        phase = 0u;
        if (s_on_ms > 0u) {
            LED_ON();
        }
    } else if (phase == s_on_ms) {
        LED_OFF();
    }
    s_phase_ms = phase;
}
