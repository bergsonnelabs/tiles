/**
 * port_core_led.h — STM32 port: the bodies of core_led.h
 *
 * Included at the end of core_led.h, after its prototypes; not for direct
 * use. The LED pin and the LED_ON / LED_OFF / LED_TOGGLE macros come from the
 * project's generated core_board.h, the clock and pin setup from ll_rcc /
 * ll_gpio, and the blocking delays from ll_systick.
 */

#ifndef PORT_CORE_LED_H
#define PORT_CORE_LED_H

#include "core_board.h"
#include "ll_systick.h"
#include "ll_rcc.h"
#include "ll_gpio.h"

static inline void core_led_init(void)
{
    ll_rcc_gpio_clk_enable(LED_PORT);
    ll_gpio_config_output(LED_PORT, LED_PIN);
    LED_OFF();
}

static inline void core_led_on(void)
{
    LED_ON();
}

static inline void core_led_off(void)
{
    LED_OFF();
}

static inline void core_led_toggle(void)
{
    LED_TOGGLE();
}

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

#endif /* PORT_CORE_LED_H */
