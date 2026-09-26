/**
 * Core.ST.H5 starter — blink the status LED, print over USB CDC.
 *
 * The H5 is the big one: STM32H523, Cortex-M33, up to 248 MHz. It shares the
 * L4's USB-DFU story, so config.json is in the same fleet-standard shape:
 *
 *   "bootloader": "rom"  app at 0x08000000, recovery via the ST ROM bootloader
 *                        (which no app flash can erase). This is what makes
 *                        `make flash-dfu` and Studio's flasher work.
 *   "usb"                core_init() brings USB CDC up before main() runs, so
 *                        the 1200-baud touch can always reboot into DFU — that
 *                        is the escape hatch, don't remove it lightly.
 *   "iwdg"               a 5 s watchdog. If your loop stops feeding it, the
 *                        Core resets; repeated resets park it in ROM DFU
 *                        instead of soft-bricking.
 *
 * The cost of the watchdog is the one line below: keep core_watchdog_feed() in
 * any loop that can run longer than the timeout, or drop "iwdg" from
 * config.json if you'd rather not think about it.
 *
 * "clock": "medium" is 64 MHz HSI. Raise it to "high" (128 MHz) or "max"
 * (248 MHz) in config.json when you need the headroom.
 */

#include "core.h"
#include "core_watchdog.h"

int main(void)
{
    core_init();      /* clock + USB CDC + watchdog, all from config.json */
    core_led_init();

    uint32_t ticks = 0;

    while (1) {
        core_watchdog_feed();

        LED_TOGGLE();

        if (core_usb_connected()) {
            core_usb_printf("tick %lu\r\n", (unsigned long)ticks++);
        }

        core_delay_ms(250);
    }
}
