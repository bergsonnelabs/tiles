/**
 * val-thread-auto -- Thread joins on boot, with no Thread code in main.c
 *
 * templates/core-st-w5's blink plus `"thread": {"enabled": true}` in
 * config.json, and nothing else: the shape of a Studio DSL program (main()
 * calls core_init(), core_led_init(), then a loop of core_watchdog_feed() and
 * core_delay_ms()). core_init() starts Thread (coregen, thread_autostart) and
 * the delay and the feed service it, so the node joins the development
 * network and stays on it. The build fails the point of the test if OpenThread
 * is not linked: CI checks the image for otInstanceInitSingle (Makefile
 * target check-linked, run by the firmware workflow).
 */

#include "core.h"
#include "core_watchdog.h"

int main(void)
{
    core_init();      /* clock + watchdog + Thread, all from config.json */
    core_led_init();

    while (1) {
        core_watchdog_feed();

        LED_TOGGLE();
        core_delay_ms(250);
    }
}
