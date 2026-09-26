/**
 * val-scope — the Scope API (core_scope.h) compiles and links on EVERY Core.
 *
 * The module picks its link at compile time — USB on the L4 / H5, Bluetooth
 * on a W5 built with the radio on, none on the L0 — so the same source has to
 * build on all four, and again with the module compiled out:
 *
 *   make && make clean
 *   make TILE=Core.ST.H5 && make clean
 *   make TILE=Core.ST.W5   && make clean
 *   make TILE=Core.ST.L0 && make clean
 *   make SCOPE_ENABLED=0   && make clean
 *
 * The Bluetooth link is covered by tests/val-ble-scope (W5, BLE on). Behavior
 * is covered on the host by tools/test_core_scope.py. Compile/link validation
 * only; on a USB Core it does also stream if you flash it.
 */

#include "core.h"
#include "core_scope.h"

/* ---- What real use looks like: three lines. ---- */

static int32_t counter;
static float   wave;
static int16_t accel[3];

/* ---- Everything else in the API, referenced once so it has to compile and
 * link on every Core. NOT an example: nobody calls all of these together
 * (update() already samples and pumps; interval and rate are alternatives).
 * Behind a flag that is never set, so it links but never runs. ---- */

static uint32_t         uptime_ms;
static bool             led_on;
static uint16_t         adc_raw;
static int8_t           trim;
static uint8_t          level;
static volatile int16_t isr_value;
static volatile int     run_api_coverage;

static void api_coverage(void)
{
    core_scope_init();
    (void)core_scope_watch(uptime_ms);
    (void)core_scope_watch(led_on);
    (void)core_scope_watch(adc_raw);
    (void)core_scope_watch(trim);
    (void)core_scope_watch_as("lvl", level);
    (void)core_scope_watch(isr_value);
    (void)core_scope_add("counter2", &counter, CORE_SCOPE_I32);
    (void)core_scope_add_array("accel2", accel, CORE_SCOPE_I16, 3);
    core_scope_set_interval_ms(10);
    core_scope_declare_rate_hz(1000);
    core_scope_declare_rate_millihertz(416667);
    core_scope_sample();
    core_scope_pump();
    core_scope_enable(core_scope_active() || core_scope_dropped() > 0u);
}

int main(void)
{
    core_init();

    core_scope_watch(counter);
    core_scope_watch(wave);
    core_scope_watch_array(accel, 3);

    while (1) {
        counter++;
        wave = (float)(counter % 100) * 0.01f;
        accel[0] = (int16_t)counter;

        core_scope_update();

        if (run_api_coverage) api_coverage();
    }
}
