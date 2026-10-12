/**
 * val-thread-provision -- compile/link validation of Thread provisioning without BLE
 *
 * Core.ST.W5, Thread only, with config.json `"thread": {"provisioning":
 * true}`: a network stored at run time wins over config.json's at boot, and
 * the API to store one is live (sdk/core/core_thread.c, THREAD_CFG_PROVISIONING).
 * Calls each provisioning function so a signature change or a missing symbol
 * fails the build. On a board it joins config.json's network, or the one a
 * previous image stored; the calls that would change the network sit behind
 * a flag that is never set. tests/hw-ble-thread is the bench test (over BLE).
 */

#include "core.h"
#include "core_thread.h"

static volatile uint8_t s_never;          /* never set: links the calls, never runs them */
static volatile int s_provisioned;
static volatile uint16_t s_panid;

int main(void)
{
    core_init();
    (void)core_thread_start();

    uint32_t last = core_millis();
    while (1) {
        core_thread_process();

        if (core_millis() - last >= 5000u) {
            last = core_millis();
            core_thread_dataset_t net;
            if (core_thread_get_network(&net) == 0)
                s_panid = net.panid;
            s_provisioned = core_thread_is_provisioned();
            if (s_never) {
                net.channel = 20;
                (void)core_thread_set_dataset(&net);
                (void)core_thread_factory_reset();
                (void)core_thread_set_enabled(0);
                (void)core_thread_set_enabled(1);
            }
        }
    }
}
