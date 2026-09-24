/**
 * val-led — compile check for the onboard-LED helpers (core_led.h).
 *
 * Calls every public LED function, including the VDD-scaled on-time that
 * core_led_blink() and core_led_heartbeat() apply to short blips. Build-only:
 * no hardware assertions. core_led_sos() is noreturn and deliberately left out.
 */
#include "core.h"

int main(void)
{
    core_init();
    core_led_init();

    core_led_on();
    core_led_off();
    core_led_toggle();
    core_led_blink(3, 2, 200);            /* a short blip: VDD-scaled */
    core_led_blink(1, 100, 100);          /* a long flash: left as is */
    (void)core_led_scaled_on_ms(2);
    core_led_heartbeat(1000, 2);          /* 1 Hz, 2 ms at 3.3 V */

    while (1) {
        ll_delay_ms(1000);
    }
}
