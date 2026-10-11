/**
 * val-thread -- compile/link validation of every core_thread.h API
 *
 * Core.ST.W5 with "thread" enabled in config.json: builds the OpenThread
 * integration (sdk/thread, LinkLayer_Thread_lib, the FTD library) and calls
 * each public function once, so a signature change or a missing symbol fails
 * the build. On a board it joins the development network, broadcasts on port
 * 4321 every 5 s and sends one unicast back to the last node it heard from;
 * tests/hw-thread is the bench test.
 */

#include "core.h"
#include "core_thread.h"

static volatile uint32_t s_role_changes;
static volatile uint32_t s_rx;
static uint8_t s_peer[16];
static uint16_t s_peer_port;

static void on_role(core_thread_role_t role, void *ctx)
{
    (void)role;
    (void)ctx;
    s_role_changes++;
}

static void on_rx(const uint8_t *data, uint16_t len, const uint8_t src[16], uint16_t port, void *ctx)
{
    (void)data;
    (void)len;
    (void)ctx;
    s_rx++;
    for (int i = 0; i < 16; i++)
        s_peer[i] = src[i];
    s_peer_port = port;
}

int main(void)
{
    core_init();

    core_thread_on_role_change(on_role, NULL);
    (void)core_thread_start();
    (void)core_thread_udp_open(4321, on_rx, NULL);

    uint8_t ext[8];
    core_thread_ext_addr(ext);
    (void)core_thread_instance();

    uint32_t last = core_millis();
    while (1) {
        core_thread_process();

        if (core_millis() - last >= 5000u) {
            last = core_millis();
            if (core_thread_role() >= CORE_THREAD_CHILD) {
                (void)core_thread_udp_broadcast(4321, "val-thread", 10);
                if (s_peer_port != 0)
                    (void)core_thread_udp_send(s_peer, s_peer_port, "unicast", 7);
            }
            (void)core_thread_rloc16();
            (void)core_thread_partition_id();
            (void)core_thread_neighbor_count();
            (void)core_thread_parent_rssi();
            (void)core_thread_last_error();
        }
    }
}
