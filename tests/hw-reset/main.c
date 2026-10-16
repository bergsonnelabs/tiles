/**
 * hw-reset — bench test for core_reset() and core_reset_reason() on a
 * Core.ST.L4 (ROM-DFU layout; the 5 s watchdog from config.json, fed below).
 *
 * After a 1.5 s start delay (the CDC drops early prints), it prints the
 * reason for the last reset once a second, with a boot counter kept in the
 * RTC backup registers so a reset is visible as a new boot:
 *
 *   boot 3  reason=software (3)  flags=0x14000000  up 7 s
 *
 * Commands over USB CDC:
 *   r   core_reset() from the app            -> next boot: software
 *   w   stop feeding the watchdog (~5 s)     -> next boot: watchdog
 *   z   zero the boot counter
 *
 * From the host, without typing: `make reset-serial` (or
 * `python3 tools/serial_update.py --reset`) sends the remote reset frame
 * (docs/serial-update-protocol.md §10); it should print "SU OK R", the port
 * drops and returns, and the next boot reads software. A power cycle reads
 * power-on, the NRST pin (if wired) pin.
 *
 * Expected flags (bits 31..24, RM0394 §6.4.29): power-on 0x0C000000
 * (BOR + PIN), software 0x14000000 (SFT + PIN), watchdog 0x24000000
 * (IWDG + PIN). An option-byte reload (the first core_reset() after a DFU
 * flash with no power cycle, if PEMPTY can't be cleared) shows OBL, 0x06000000.
 *
 * LED: a 2 ms flash once a second (alive), fast while waiting for the watchdog.
 */

#include "core.h"
#include "core_watchdog.h"
#include "core_reset.h"
#include "core_backup.h"

static const char *reason_name(core_reset_reason_t r)
{
    switch (r) {
    case CORE_RESET_POWER_ON: return "power-on";
    case CORE_RESET_PIN:      return "pin";
    case CORE_RESET_WATCHDOG: return "watchdog";
    case CORE_RESET_SOFTWARE: return "software";
    default:                  return "other";
    }
}

int main(void)
{
    core_init();
    core_usb_init();
    core_led_init();

    /* Boot counter in backup register 0 (survives every reset but a power-on
     * that loses VBAT, so a power cycle usually zeroes it). */
    uint32_t boots = core_backup_read(0) + 1u;
    core_backup_write(0, boots);

    /* 1.5 s for the host to open the port: early CDC output is dropped. */
    for (int i = 0; i < 15; i++) {
        core_watchdog_feed();
        core_delay_ms(100);
    }

    core_reset_reason_t why = core_reset_reason();
    uint32_t flags = core_reset_flags();
    core_usb_printf("\r\n[hw-reset] boot %lu: reason=%s (%d) flags=0x%08lx\r\n",
                    (unsigned long)boots, reason_name(why), (int)why, (unsigned long)flags);
    core_usb_printf("[hw-reset] 'r' = core_reset(), 'w' = watchdog reset, 'z' = zero the counter\r\n");

    uint32_t last = core_millis();
    while (1) {
        core_watchdog_feed();

        if (core_millis() - last >= 1000u) {
            last += 1000u;
            LED_ON();
            core_delay_ms(2);
            LED_OFF();
            core_usb_printf("boot %lu  reason=%s (%d)  flags=0x%08lx  up %lu s\r\n",
                            (unsigned long)boots, reason_name(why), (int)why,
                            (unsigned long)flags, (unsigned long)(core_millis() / 1000u));
        }

        if (core_usb_available()) {
            int ch = core_usb_getc();
            if (ch == 'r') {
                core_usb_printf("[hw-reset] core_reset()\r\n");
                core_reset();                     /* flushes the line above first */
            } else if (ch == 'w') {
                core_usb_printf("[hw-reset] not feeding the watchdog: reset in ~5 s\r\n");
                for (;;) {
                    LED_TOGGLE();
                    core_delay_ms(100);
                }
            } else if (ch == 'z') {
                core_backup_write(0, 0);
                boots = 0;
                core_usb_printf("[hw-reset] boot counter zeroed\r\n");
            }
        }
        core_delay_ms(5);
    }
}
