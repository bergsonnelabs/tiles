/**
 * Core.ST.W5 starter — blink the status LED.
 *
 * The W5 is the radio one: STM32WBA55, Cortex-M33 with Bluetooth LE. What
 * differs from the L4/H5 templates:
 *
 *   - No USB. The WBA55 has no USB peripheral, so there is no CDC, no
 *     1200-baud touch, and no `make flash-dfu`. Flash over SWD with
 *     `make flash` (STM32CubeProgrammer + an ST-Link; OpenOCD has no WBA55
 *     support), or from Studio / probe-rs over the CoreProbe.
 *   - The watchdog is on (5 s), same as every other Core — the IWDG is plain
 *     hardware here and works fine. What the W5 does NOT get is the
 *     strike-counter escape into ROM DFU, which needs USB. Recovery on this
 *     Core is SWD, which a probe can always reach.
 *
 * For BLE, add to config.json:
 *
 *     "ble": { "enabled": true }
 *
 * The SDK Makefile reads that key and links the WBA BLE stack in (~200 KB of
 * flash, ~50 KB of RAM), so this works from Studio and the cloud build service
 * too — both only ever write config.json. `make BLE_ENABLED=1` still works as a
 * local override. Drive it from C with the core_ble_* API (include core_ble.h;
 * core.h does not), or declare a BLE contract in config.json for DSL blocks;
 * see the ble-* projects under tests/.
 *
 * "clock": "medium" is 32 MHz from the HSE. Raise it to "high" (64 MHz) or
 * "max" (100 MHz) in config.json when you need the headroom.
 */

#include "core.h"
#include "core_watchdog.h"

int main(void)
{
    core_init();      /* clock + watchdog, both from config.json */
    core_led_init();

    while (1) {
        core_watchdog_feed();

        LED_TOGGLE();
        core_delay_ms(250);
    }
}
