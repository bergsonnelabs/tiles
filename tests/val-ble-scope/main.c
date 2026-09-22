/**
 * val-ble-scope — the Scope's Bluetooth link compiles and links on Core.ST.W5.
 *
 * The W5 has a radio and no USB, so core_scope streams over BLE there: it
 * registers the Studio Link service (0x5C00) with one notify characteristic,
 * Studio Scope (0x5C01), through core_ble_add_services() — beside whatever
 * services the application declares, not instead of them.
 *
 * Order matters and this file shows it: the first core_scope call comes
 * BEFORE core_ble_init(), because a GATT service cannot be added once the
 * stack is running.
 *
 * Compile/link validation. If you do flash it: connect from Studio's Scope
 * panel ("Connect over Bluetooth") and a sawtooth and a counter should plot.
 */

#include "core.h"
#include "core_ble.h"
#include "core_scope.h"
#include "core_timing.h"

static int32_t counter;
static int16_t sawtooth;

int main(void)
{
    core_init();

    (void)core_scope_watch(counter);      /* registers Studio Link: before core_ble_init() */
    (void)core_scope_watch(sawtooth);
    core_scope_set_interval_ms(10);

    core_ble_set_conn_params(15, 30, 0, 4000);   /* the tightest window Apple hosts accept */
    core_ble_init();
    core_ble_process();
    core_ble_advertise("val-scope");

    while (1) {
        core_ble_process();
        counter++;
        sawtooth = (int16_t)(core_millis() % 1000u);
        core_scope_update();
    }
}
