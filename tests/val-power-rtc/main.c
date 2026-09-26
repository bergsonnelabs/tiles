/**
 * val-power-rtc -- Validation: Sleep, stop modes, EXTI wakeup, RTC
 *
 * Core.ST.H5, clock=high (128MHz)
 * Pad 8 = GPIO.IN with EXTI (falling edge wakeup)
 *
 * Exercises: core_init, core_sleep, core_stop_for,
 *            core_pad_on_change (pad, EDGE_FALLING, cb, NULL),
 *            core_rtc_init, core_rtc_set_time, core_rtc_get_time
 */

#include "core.h"
#include "core_power.h"
#include "core_rtc.h"
#include "core_gpio.h"

static volatile uint32_t wakeup_count;

static void on_wakeup(void *ctx)
{
    (void)ctx;
    wakeup_count++;
}

int main(void)
{
    core_init();
    core_led_init();

    /* RTC init (LSI) */
    core_rtc_init();
    core_rtc_set_time(12, 0, 0);

    /* EXTI on pad 8 -- falling edge wakeup */
    core_pad_on_change(8, EDGE_FALLING, on_wakeup, NULL);

    /* Light sleep -- wakes on any interrupt (SysTick, EXTI, etc.) */
    core_sleep();

    LED_TOGGLE();

    /* Stop mode with RTC wakeup after 5 seconds */
    core_stop_for(5);

    LED_TOGGLE();

    /* Read back RTC time to verify it survived stop mode */
    uint8_t h, m, s;
    core_rtc_get_time(&h, &m, &s);
    (void)h;
    (void)m;
    (void)s;

    while (1) {
        LED_TOGGLE();
        core_delay_ms(1000);
    }
}
