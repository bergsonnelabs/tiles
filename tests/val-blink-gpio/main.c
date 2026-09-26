/**
 * val-blink-gpio -- Validation: GPIO output, input with EXTI, LED, watchdog
 *
 * Core.ST.L0, clock=max
 * Pad 11 = GPIO.OUT (B9), Pad 8 = GPIO.IN with EXTI
 *
 * Exercises: core_init, core_led_init, LED_TOGGLE, core_delay_ms,
 *            core_pad_output, core_pad_write, core_pad_on_change,
 *            core_watchdog_start, core_watchdog_feed
 */

#include "core.h"
#include "core_gpio.h"
#include "core_watchdog.h"

static volatile uint32_t edge_count;

static void on_edge(void *ctx)
{
    (void)ctx;
    edge_count++;
}

int main(void)
{
    core_init();
    core_led_init();

    /* GPIO output on pad 11 */
    core_pad_output(11);
    core_pad_write(11, ON);

    /* EXTI on pad 8 -- falling edge */
    core_pad_on_change(8, EDGE_FALLING, on_edge, NULL);

    /* Independent watchdog -- 2 second timeout */
    core_watchdog_start(2000);

    while (1) {
        LED_TOGGLE();
        core_pad_write(11, OFF);
        core_delay_ms(250);

        core_pad_write(11, ON);
        core_delay_ms(250);

        core_watchdog_feed();
    }
}
