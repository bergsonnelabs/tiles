/**
 * Core.ST.L4 factory image: what every new Core.ST.L4 runs until its owner
 * flashes their own program.
 *
 *   LED     a 1 ms blip once a second: alive, and close to no current
 *   USB     CDC serial. On connect it prints one line, then stays quiet:
 *             Core.ST.L4 factory 1.0.0 sdk=<commit> uid=<24 hex digits>
 *           "?" or Enter prints it again.
 *   update  USB serial update (make flash-serial, Studio) and remote reset
 *           come with the SDK's CDC driver; this program needs no code for them.
 *   power   sleeps (WFI) between SysTick ticks instead of spinning.
 *
 * config.json: ROM-bootloader layout (app at 0x08000000, so ROM DFU always
 * recovers it), USB on, 5 s watchdog.
 */
#include "core.h"
#include "core_power.h"
#include "core_uid.h"
#include "core_usb.h"
#include "core_watchdog.h"
#include "build_info.h"

#define FACTORY_VERSION "1.0.0"
#define BLIP_MS         1
#define PERIOD_MS       1000
#define BANNER_DELAY_MS 100   /* let the host finish opening the port */

static void banner(void)
{
    char uid[25];
    if (core_uid_hex(uid, sizeof uid) < 0) uid[0] = '\0';
    core_usb_printf("Core.ST.L4 factory " FACTORY_VERSION " sdk=" FACTORY_SDK " uid=%s\r\n", uid);
}

int main(void)
{
    core_init();
    core_led_init();

    uint32_t next_blip = core_millis();
    uint32_t connected_at = 0;   /* when the host opened the port, 0 = not open */
    uint8_t  greeted = 0;

    for (;;) {
        core_watchdog_feed();
        uint32_t now = core_millis();

        if ((int32_t)(now - next_blip) >= 0) {
            core_led_on();
            core_delay_ms(BLIP_MS);
            core_led_off();
            next_blip += PERIOD_MS;
        }

        if (core_usb_connected()) {
            if (!connected_at) connected_at = now | 1;
            if (!greeted && now - connected_at >= BANNER_DELAY_MS) { banner(); greeted = 1; }
            uint8_t c;
            while (core_usb_try_read(&c))
                if (c == '?' || c == '\r' || c == '\n') banner();
        } else {
            connected_at = 0;
            greeted = 0;
        }

        core_sleep();   /* until the next interrupt: SysTick (1 ms) or USB */
    }
}
