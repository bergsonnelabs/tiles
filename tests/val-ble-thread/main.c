/**
 * val-ble-thread -- compile/link validation of BLE + Thread in one image
 *
 * Core.ST.W5 with both "ble" and "thread" enabled in config.json: builds ST's
 * concurrent link layer (LinkLayer_BLE_Thread_lib), the BLE stack and
 * OpenThread together, and calls every public core_ble.h and core_thread.h
 * function once, so a signature change, a missing symbol or a clash between
 * the two stacks fails the build. On a board it advertises "val-ble-thread"
 * with one read / notify characteristic and joins the development Thread
 * network; tests/hw-ble-thread is the bench test.
 *
 * config.json also enables "ble_provisioning": the Thread provisioning GATT
 * service (0x5D00, docs/thread-provisioning-protocol.md) is linked, and the
 * provisioning API is called behind a flag that is never set, so the board
 * stays on config.json's network.
 *
 * Order: core_ble_init() before core_thread_start() (BLE first, as ST's
 * BLE_HeartRate_Thread); one loop calls both process functions.
 */

#include "core.h"
#include "core_ble.h"
#include "core_thread.h"

static core_ble_char_t s_status;
static volatile uint32_t s_rx;
static volatile uint32_t s_writes;
static volatile uint32_t s_links;
static uint8_t s_peer[16];
static uint16_t s_peer_port;
static volatile uint8_t s_never;          /* never set: links the calls, never runs them */

/* The gate for BLE writes: the SDK's weak default returns 1 (open). */
int core_thread_prov_allow_write(void)
{
    return s_writes < 1000u;
}

static void on_write(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)data;
    (void)len;
    (void)ctx;
    s_writes++;
}

static void on_link(void *ctx)
{
    (void)ctx;
    s_links++;
}

static void services(void)
{
    core_ble_svc_t svc = core_ble_add_service_id("val", 0x7410);
    s_status = core_ble_add_char_id(svc, "status", 0x7411, CORE_BLE_READ | CORE_BLE_NOTIFY,
                                    CORE_BLE_BYTES(8), NULL, NULL);
    (void)core_ble_add_char(svc, "cmd", CORE_BLE_RW, CORE_BLE_UINT8, on_write, NULL);
    core_ble_svc_t bas = core_ble_add_service_sig("battery", 0x180F);
    (void)core_ble_add_char_sig(bas, "level", 0x2A19, CORE_BLE_READ, CORE_BLE_UINT8, NULL, NULL);
    core_ble_svc_t seq = core_ble_add_service("seq");
    (void)core_ble_add_char_id(seq, "x", 0x7412, CORE_BLE_READ, CORE_BLE_UINT16, NULL, NULL);
}

static void extra(void)
{
}

static void on_role(core_thread_role_t role, void *ctx)
{
    (void)role;
    (void)ctx;
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

    /* BLE first. */
    core_ble_set_services(services);
    (void)core_ble_add_services(extra);
    core_ble_set_tx_power(1);
    core_ble_set_adv_interval(100, 150);
    core_ble_set_conn_params(30, 50, 0, 4000);
    core_ble_set_auto_mtu(1);
    core_ble_on_connect(on_link, NULL);
    core_ble_on_disconnect(on_link, NULL);
    if (0)
        core_ble_enable_pairing();
    core_ble_init();
    core_ble_process();
    (void)core_ble_advertise("val-ble-thread");

    /* Then Thread, on the same radio and sequencer. */
    core_thread_on_role_change(on_role, NULL);
    (void)core_thread_start();
    (void)core_thread_udp_open(4321, on_rx, NULL);
    uint8_t ext[8];
    core_thread_ext_addr(ext);
    (void)core_thread_instance();

    uint32_t last = core_millis();
    while (1) {
        core_ble_process();
        core_thread_process();

        if (core_millis() - last >= 5000u) {
            last = core_millis();
            uint8_t st[8] = { (uint8_t)core_thread_role(), core_thread_neighbor_count() };
            uint16_t rloc = core_thread_rloc16();
            st[2] = (uint8_t)rloc;
            st[3] = (uint8_t)(rloc >> 8);
            uint32_t rx = s_rx;
            for (int i = 0; i < 4; i++)
                st[4 + i] = (uint8_t)(rx >> (8 * i));
            (void)core_ble_set_value(s_status, st, sizeof st);
            if (core_ble_connected() && core_ble_subscribed(s_status))
                (void)core_ble_notify(s_status);

            if (core_thread_role() >= CORE_THREAD_CHILD) {
                (void)core_thread_udp_broadcast(4321, "val-ble-thread", 14);
                if (s_peer_port != 0)
                    (void)core_thread_udp_send(s_peer, s_peer_port, "unicast", 7);
            }
            (void)core_thread_partition_id();
            (void)core_thread_parent_rssi();
            (void)core_thread_last_error();

            core_thread_dataset_t net;
            if (core_thread_get_network(&net) == 0 && core_thread_is_provisioned() == 0 && s_never) {
                net.panid ^= 1u;
                (void)core_thread_set_dataset(&net);
                (void)core_thread_factory_reset();
                (void)core_thread_set_enabled(0);
                (void)core_thread_set_enabled(1);
            }
            if (s_writes > 100u)
                (void)core_ble_stop_advertise();
        }
    }
}
