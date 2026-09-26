/**
 * val-rom-dfu-h -- Validation: ROM DFU bootloader on Core.ST.H5
 *
 * Core.ST.H5, clock=default, USB enabled, ROM_DFU=1
 *
 * Tests the ROM DFU boot path on STM32H523 (Cortex-M33):
 *   1. App starts at 0x08000000 (no custom bootloader partition)
 *   2. 1200-baud touch on CDC → hal_dfu_reboot() → magic + reset
 *   3. On reboot, core_init() sees magic → hal_dfu_jump_to_rom()
 *   4. ROM bootloader enumerates as 0483:DF11 (DfuSe)
 *   5. dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
 *
 * Visual feedback: LED blinks every 500ms, prints "rom-dfu-h alive" on CDC.
 */

#include "core.h"
#include "core_usb.h"

int main(void)
{
    core_init();
    core_usb_init();

    core_usb_printf("rom-dfu-h test v1\r\n");

    core_led_init();

    uint32_t count = 0;

    while (1) {
        LED_TOGGLE();

        if (core_usb_connected()) {
            core_usb_printf("rom-dfu-h alive %lu\r\n", count++);
        }

        core_delay_ms(500);
    }
}
