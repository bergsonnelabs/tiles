/**
 * val-bootloader -- Validation: BOOTLOADER=1 on Core.ST.H5
 *
 * Core.ST.H5, clock=default, USB enabled, BOOTLOADER=1
 *
 * The key test is that BOOTLOADER=1 in the Makefile causes APP_OFFSET
 * and DFU bootloader logic to compile correctly. The linker script
 * should place the app at the offset, and the DFU bootloader entry
 * point should be resolved.
 *
 * Exercises: core_init, core_usb_init, core_usb_printf, basic main loop
 */

#include "core.h"
#include "core_usb.h"

int main(void)
{
    core_init();
    core_usb_init();

    core_usb_printf("boot test\r\n");

    core_led_init();

    while (1) {
        LED_TOGGLE();

        if (core_usb_connected()) {
            core_usb_printf("alive\r\n");
        }

        core_delay_ms(500);
    }
}
