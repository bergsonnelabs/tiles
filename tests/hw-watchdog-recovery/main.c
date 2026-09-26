/**
 * hw-watchdog-recovery — bench guinea pig for watchdog-default + strike→ROM-DFU
 * brick recovery on Core.ST.L4 rev b (the CoreProbe).
 *
 * core_init() (with iwdg.enabled in config.json) auto-starts the 5 s IWDG and, on
 * each boot, maintains the reserved-SRAM strike counter — after 3 consecutive
 * watchdog resets that never reach healthy operation, it blinks SOS and jumps to
 * the ST ROM bootloader (0483:DF11) so the board can always be reflashed.
 *
 * This app drives that loop, using the reset cause itself as the cascade engine:
 *
 *   - Fresh boot (power-on / flash): HEALTHY — feeds the dog, blinks the LED, and
 *     after ~10 s (2× the timeout) clears the strike counter.
 *   - Type 'h' on the USB CDC console → stop feeding → watchdog resets in ~5 s.
 *   - Every following watchdog-reset boot keeps hanging, so strikes climb
 *     1 → 2 → 3. On the 3rd, core_init() SOS-blinks and drops to ROM DFU.
 *     Reflash from Studio to recover.
 *   - 'c' clears strikes; 'd' forces DFU now.
 *
 * Each boot prints the reset cause + strike count on CDC so you can watch the
 * counter climb to the escape. USB-CDC PASS/FAIL isn't used here — this is an
 * interactive bench test, not an automated val.
 */

#include "core.h"
#include "core_watchdog.h"
#include "core_recovery.h"

#define HEALTHY_CLEAR_MS  10000u   /* ~2× the 5 s watchdog timeout */

/* Stop feeding the dog forever → the watchdog resets us in ~5 s. Fast LED blink
 * signals "stuck". */
static void hang_forever(void)
{
    core_usb_printf("[guinea] hanging — not feeding. Watchdog reset in ~5 s...\r\n");
    while (1) {
        LED_TOGGLE();
        core_delay_ms(120);
    }
}

int main(void)
{
    core_init();          /* auto-starts the 5 s IWDG; may have already SOS→DFU'd */
    core_usb_init();
    core_led_init();
    core_delay_ms(200);   /* let CDC settle before the first print */

    int wd = core_watchdog_caused_reset();
    uint32_t strikes = hal_recovery_strikes();
    core_usb_printf("\r\n[guinea] boot — cause=%s, strikes=%lu/%u\r\n",
                    wd ? "WATCHDOG" : "power/flash",
                    (unsigned long)strikes, (unsigned)CORE_RECOVERY_STRIKE_LIMIT);

    if (wd) {
        /* Mid-cascade: a prior hang reset us. Keep hanging so the counter climbs
         * toward the ROM-DFU escape (core_init handles the escalation next boot). */
        core_usb_printf("[guinea] cascading toward the DFU escape...\r\n");
        hang_forever();
    }

    core_usb_printf("[guinea] HEALTHY. 'h'=hang, 'c'=clear strikes, 'd'=force DFU.\r\n");

    uint32_t t0 = core_millis();
    int cleared = 0;

    while (1) {
        core_watchdog_feed();
        LED_TOGGLE();

        if (core_usb_available()) {
            int ch = core_usb_getc();
            if (ch == 'h') {
                hang_forever();
            } else if (ch == 'c') {
                core_recovery_clear();
                core_usb_printf("[guinea] strikes cleared\r\n");
            } else if (ch == 'd') {
                hal_dfu_reboot();  /* magic + reset → core_init → ROM DFU */
            }
        }

        if (!cleared && (uint32_t)(core_millis() - t0) >= HEALTHY_CLEAR_MS) {
            core_recovery_clear();
            cleared = 1;
            core_usb_printf("[guinea] 10 s healthy — strikes cleared\r\n");
        }

        core_delay_ms(200);   /* well under the 5 s timeout */
    }
}
